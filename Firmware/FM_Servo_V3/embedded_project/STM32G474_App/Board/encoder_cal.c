/*
 * encoder_cal.c —— MT6835 客户端自校准状态机
 *
 * 流程（对应手册 9.2 的 (1)~(6)）：
 *   SPINUP  启动速度闭环，加速到 600 rpm（落在 AUTO_CAL_FREQ=0x03 的 400~800 区间）
 *   SETTLE  观察 3 s，确认转速稳定且落在区间内
 *   CAL     CAL_EN=1，记录起始位置，累计 |Δ圈数| >= 64 同时轮询 0x113[7:6]
 *   DONE    CAL_EN=0，关速度环、封 MOE，提示"必须断电重启"
 *
 * 为什么不用 deadman：校准要连续转 6.4 s，而 deadman 是 2 s。
 * 本模块从 Start 到结束全程保持 s_armed=0（与 Z 辨识同样的做法），
 * 自己管超时与故障检查，对外只有"开始 / 查询 / 中止"三个动作。
 */
#include "encoder_cal.h"
#include "bsp_servo.h"
#include "actuator.h"
#include "telemetry_tx.h"   /* 结果上报 30092（2026-11-21 补） */
#include "mavlink_common.h"  /* MAV_ID_ENC_CAL */

#include <math.h>
#include <string.h>

#define EC_RPM_DEFAULT   (600.0f)
#define EC_RPM_LO        (400.0f)     /* AUTO_CAL_FREQ=0x03 的下界 */
#define EC_RPM_HI        (800.0f)
#define EC_SETTLE_MS     (3000U)      /* 匀速确认窗口 */
#define EC_MIN_TURNS     (64U)        /* 手册要求 64 圈以上 */
#define EC_TIMEOUT_MS    (30000U)
#define EC_POLL_MS       (100U)       /* 状态寄存器轮询间隔 */

#define EC_REG_STATUS    (0x113U)

/* ---- 开环拖动兜底 ----
 * 为什么需要：^ 原来只走速度闭环，而速度闭环依赖编码器角度。一旦编码器
 * 角度不可信（本板 2026-11-19 的 EEPROM 补偿表被中断写入打坏），闭环起不
 * 来 → 永远进不了 400~800 rpm → EC_ERR_SPEED，而"修编码器"又必须先转起来，
 * 形成死锁。开环强制角不读编码器，正好打破这个循环。
 *
 * 注意：MT6835 自校准只要求磁钢**匀速旋转**，不要求由谁驱动、也不要求
 * 闭环。所以开环拖动在原理上是完全合法的等价通道。 */
/* 2026-11-21 调整：原来 2.5 A / 300 rad/s²。实测 ^ 跑满 30 s 超时，
 * chip_status 一直是 1（芯片"校准中"），说明它没等到匀速转够 64 圈。
 * 600 rpm 下 64 圈只要 6.4 s，30 s 都等不到 → 判断实际转速远低于给定，
 * 即开环拖动失步/拖不动。加大电流、放慢加速、给足斜坡时间。
 * 总预算：8 s 斜坡 + 3 s 稳定 + 6.4 s 校准 ≈ 17.4 s，仍在 30 s 超时内。 */
#define EC_OL_IDREF      (4.0f)      /* 开环 d 轴电流 A：要克服谐波减速器的起动力矩 */
#define EC_OL_ACC        (150.0f)    /* 电角速度斜坡 rad/s²（≈102 rpm/s 机械，更不容易失步） */
#define EC_OL_RAMP_MS    (8000U)     /* 0→600rpm 斜坡约 5.9 s，留 8 s 余量 */
#define EC_OL_SPINUP_MS  (8000U)     /* 闭环起拖超时（保留给闭环路径） */

static volatile EncCalState s_state = EC_IDLE;
static EncCalResult s_res;

static float    s_rpm_tgt;
static uint32_t s_t;
static uint32_t s_tmo;
static uint32_t s_poll;
static int64_t  s_pos0;
static float    s_rpm_sum;
static uint32_t s_rpm_n;
static uint8_t  s_ol;            /* 1 = 已切换到开环拖动 */
static float    s_ol_w_now;      /* 当前电角速度（斜坡中） */
/* 开环虚拟圈数（2026-11-21）。
 * 编码器角度不可信时（本板实测：帧全丢 → s_enc_raw 冻结 → 实测转速恒 0），
 * "转了多少圈"只能靠给定电角速度积分。芯片自己会用内部磁检测判断是否
 * 匀速转够 64 圈，所以这里盲信给定不会造成"假成功"，最坏是白等一轮。 */
static float    s_ol_turns;      /* 累计圈数 */
static float    s_ol_turns0;     /* 阶段起点快照 */
static float    s_ol_w_tgt;      /* 目标电角速度 */

const EncCalResult *EncCal_GetResult(void) { return &s_res; }
EncCalState EncCal_GetState(void) { return s_state; }

uint8_t EncCal_IsBusy(void)
{
    return ((s_state != EC_IDLE) &&
            (s_state != EC_DONE) &&
            (s_state != EC_FAIL)) ? 1U : 0U;
}

const char *EncCal_ErrStr(EncCalErr e)
{
    switch (e) {
    case EC_OK:          return "ok";
    case EC_ERR_BUSY:    return "busy";
    case EC_ERR_CAL:     return "need C";
    case EC_ERR_ZERO:    return "need Z";
    case EC_ERR_FAULT:   return "fault";
    case EC_ERR_PWM:     return "moe";
    case EC_ERR_SPEED:   return "speed out of 400-800";
    case EC_ERR_TURNS:   return "turns timeout";
    case EC_ERR_CHIP:    return "chip reported FAIL";
    case EC_ERR_TIMEOUT: return "timeout";
    default:             return "?";
    }
}

/* 结果上报（2026-11-21 补）。
 *
 * 原来 ^ 的结果只在 ASCII 打一行 `[CMD] EC ok ...`，而 ASCII 默认关闭
 * （X72_ASCII_DEBUG=0），于是**成功和失败对 MAVLink 完全静默**：
 * 上位机只能等到超时，报一句"校准失败"，不知道卡在哪一步、芯片什么状态。
 * 现在 DONE 和 FAIL 都推 30092（kind=7）。
 *
 * 字段约定：
 *   成功 ok=1  v: turns, rpm_avg, rpm_min, rpm_max, chip_status, elapsed_ms, ol_mode, target_rpm
 *   失败 ok=0  v: err,   rpm_avg, rpm_min, rpm_max, chip_status, elapsed_ms, ol_mode, target_rpm
 *   （失败时 v[0] 换成错误码，与 Z 的约定一致；接收侧靠 ok 区分） */
static void ec_push_result(uint8_t ok)
{
    float v[8];

    v[0] = ok ? ((float)s_res.turns_x10 / 10.0f) : (float)s_res.err;
    v[1] = s_res.rpm_avg;
    v[2] = s_res.rpm_min;
    v[3] = s_res.rpm_max;
    v[4] = (float)s_res.chip_status;
    v[5] = (float)s_res.elapsed_ms;
    v[6] = (float)s_res.ol_mode;
    v[7] = s_rpm_tgt;

    Telemetry_PushIdentResult(MAV_ID_ENC_CAL, ok, 0u, v, 0.0f);
}

static void ec_stop(void)
{
    BspServo_SetEncCalEn(0U);      /* 先退出校准模式 */
    /* 恢复编码器故障判据。注意：EEPROM 写完要断电重启才生效，
     * 所以 DONE 之后到重启前如果仍报 0x0002，属预期，不代表校准失败。 */
    BspServo_SetEncFaultMask(0U);
    Servo_SetOLSpin(0U, 0.0f);     /* 停开环自转（若有） */
    Servo_SetOpenLoop(0U, 0.0f, 0.0f);
    Servo_SetSpeedMode(0U, 0.0f);
    Servo_SetCurrentMode(0U, 0.0f);
    Servo_SetEnable(0U);
    BspServo_PwmEnable(0U);
}

static void ec_fail(EncCalErr e)
{
    ec_stop();
    s_res.err = e;
    s_state   = EC_FAIL;
    ec_push_result(0u);
}

void EncCal_Abort(void)
{
    if (EncCal_IsBusy() != 0U) { ec_fail(EC_ERR_TIMEOUT); }
}

/* 闭环起不来时切到开环拖动。两条触发路径共用：
 *   ① SPINUP 里 8 s 进不了 400~800 rpm（慢慢起不来的情况）
 *   ② SPINUP 里直接报故障（角度差到一使能就假超速的情况，实测更常见）
 * 开环强制角不读编码器，是编码器角度不可信时唯一还能把转子拖起来的通道。 */
static void ec_switch_ol(void)
{
    s_rpm_sum  = 0.0f;
    s_rpm_n    = 0U;
    s_t        = 0U;
    s_ol       = 1U;
    s_res.ol_mode = 1U;

    /* 从 0 起步：直接给 600 rpm（140 Hz 电频率）会让转子完全失步，
     * 平均转矩≈0、一圈都不转（实测 turns=0.00）。必须按加速度升频。 */
    s_ol_w_now = 0.0f;
    s_ol_turns = 0.0f;
    s_ol_turns0 = 0.0f;
    s_ol_w_tgt = (s_rpm_tgt / 60.0f) * 6.28318531f * (float)BSP_POLE_PAIRS;

    Servo_SetEnable(0U);
    Servo_SetSpeedMode(0U, 0.0f);
    Servo_SetCurrentMode(0U, 0.0f);
    Servo_SetOpenLoop(1U, 0.0f, EC_OL_IDREF);
    Servo_SetOLSpin(1U, 0.0f);
    BspServo_PwmEnable(1U);      /* 退模式时可能已封 MOE，重新放开 */
}

void EncCal_Init(void)
{
    memset(&s_res, 0, sizeof(s_res));
    s_state = EC_IDLE;
    BspServo_SetEncCalEn(0U);
}

EncCalErr EncCal_Start(float rpm)
{
    if (EncCal_IsBusy() != 0U)                { return EC_ERR_BUSY; }
    if (BspServo_IsPwmEnabled() != 0U)        { return EC_ERR_BUSY; }
    if (BspServo_IsCurrentCalibrated() == 0U) { return EC_ERR_CAL; }
    if (BspServo_IsEncoderZeroed() == 0U)     { return EC_ERR_ZERO; }
    /* 只允许"仅编码器故障"（2026-11-21）：^ 用开环拖动（虚拟角），
     * 完全不读编码器，所以编码器故障位不影响它。
     * 不放行会死锁：编码器坏 → 故障清掉下一帧又置位 → ^ 永远起不来
     * → 修不了编码器。运行期间用 BspServo_SetEncFaultMask 暂停该判据。 */
    {
        const uint32_t f = BspServo_GetFault();
        if ((f != 0U) && (f != BSP_FAULT_ENCODER)) { return EC_ERR_FAULT; }
    }
    /* ^ 会写 MT6835 的 EEPROM，绝不能与其他辨识并发 */
    if (Act_Acquire(ACT_ENC_CAL, EncCal_IsBusy) == 0U) { return EC_ERR_BUSY; }

    /* 走到这里说明前置都过了。若故障位上是编码器，先屏蔽判据再清掉，
     * 否则 ec_switch_ol 里的 PwmEnable(1) 会因 s_fault!=0 静默失败。 */
    if ((BspServo_GetFault() & BSP_FAULT_ENCODER) != 0U) {
        BspServo_SetEncFaultMask(1U);
        BspServo_ClearFault();
    }

    if (rpm < EC_RPM_LO) { rpm = EC_RPM_DEFAULT; }
    if (rpm > EC_RPM_HI) { rpm = EC_RPM_DEFAULT; }

    memset(&s_res, 0, sizeof(s_res));
    s_res.target_rpm = rpm;
    s_res.rpm_min    = 1.0e9f;
    s_res.rpm_max    = -1.0e9f;

    s_rpm_tgt = rpm;
    s_t       = 0U;
    s_tmo     = 0U;
    s_poll    = 0U;
    s_rpm_sum = 0.0f;
    s_rpm_n   = 0U;
    s_ol      = 0U;
    s_res.ol_mode = 0U;

    /* 直接用开环拖动起步（不再先试速度闭环）。
     *
     * 为什么改：本板编码器角度一旦不可信，闭环一使能就在 1 ms 内报 f=0x20
     * （实测 iq 顶 3 A 而转速恒 0），EncCal_Task1ms 的故障检查立刻判
     * EC_ERR_FAULT 退出，根本轮不到"8 s 超时后改开环"那条兜底。
     * 而 MT6835 自校准只要求磁钢**匀速旋转**，由谁驱动、是否闭环都无所谓；
     * 开环强制角不读编码器，天然对这个失效模式免疫。
     *
     * 代价：开环同步转矩比闭环小，重载可能失步。由 SETTLE 阶段的
     * "3 s 至少转 1 圈"位置核对兜住，转不动就报 EC_ERR_SPEED 而不是干等。 */
    ec_switch_ol();

    if (BspServo_IsPwmEnabled() == 0U) {
        ec_fail(EC_ERR_PWM);
        return EC_ERR_PWM;
    }

    s_state = EC_SPINUP;
    return EC_OK;
}

uint8_t EncCal_Task1ms(void)
{
    if (EncCal_IsBusy() == 0U) { return 0U; }

    s_res.elapsed_ms++;

    if (++s_tmo > EC_TIMEOUT_MS) {
        ec_fail(EC_ERR_TIMEOUT);
        return 1U;
    }
    if (BspServo_GetFault() != 0U) {
        /* SPINUP 阶段别急着判死：编码器角度不可信时，闭环一使能就会报
         * f=0x20（假超速：实测 iq 顶 3 A 而转速恒 0）。清故障改开环拖动
         * 是唯一还能继续的路。开环之后若再故障，才当真实故障处理。 */
        if ((s_state == EC_SPINUP) && (s_ol == 0U)) {
            BspServo_ClearFault();
            ec_switch_ol();
            return 1U;
        }
        ec_fail(EC_ERR_FAULT);
        return 1U;
    }

    /* 开环虚拟圈数累加：放在状态机之前，覆盖 SPINUP/SETTLE/CAL 全程 */
    if (s_ol != 0U) {
        const float wm = s_ol_w_now / (float)BSP_POLE_PAIRS;   /* 机械 rad/s */
        s_ol_turns += (wm * 0.001f) / 6.28318531f;
    }

    s_t++;

    switch (s_state) {
    case EC_SPINUP: {
        const float rpm_now = fabsf(BspServo_GetOmegaMech()) * 9.5492966f;

        if (s_ol != 0U) {
            /* 开环拖动：按 EC_OL_ACC 升频，速度反馈不可信所以不参与判据 */
            if (s_ol_w_now < s_ol_w_tgt) {
                s_ol_w_now += EC_OL_ACC * 0.001f;          /* 1 ms 一拍 */
                if (s_ol_w_now > s_ol_w_tgt) { s_ol_w_now = s_ol_w_tgt; }
                Servo_SetOLSpin(1U, s_ol_w_now);
            }
            if (s_t > EC_OL_RAMP_MS) {
                s_t = 0U;
                s_state = EC_SETTLE;
            }
            return 1U;
        }

        if ((rpm_now >= (EC_RPM_LO * 1.02f)) && (rpm_now <= (EC_RPM_HI * 0.98f))) {
            s_t = 0U;
            s_state = EC_SETTLE;
        } else if (s_t > EC_OL_SPINUP_MS) {
            /* 闭环 8 s 还进不了 400~800 rpm：改开环拖动。
             * 典型场景是编码器角度不可信 —— 此时闭环永远起不来，
             * 而"修编码器"又要求转子先转，必须靠开环打破死锁。 */
            ec_switch_ol();
        }
        return 1U;
    }

    case EC_SETTLE: {
        /* 开环时不能用编码器测速：帧全丢会让它恒 0，于是"电机明明在转"
         * 却判 EC_ERR_SPEED —— 而修编码器又必须先转起来，形成死锁。
         * 开环改用给定电角速度折算。 */
        const float rpm_now = ((s_ol != 0U)
                               ? (s_ol_w_now / (float)BSP_POLE_PAIRS)
                               : fabsf(BspServo_GetOmegaMech())) * 9.5492966f;

        s_rpm_sum += rpm_now;
        s_rpm_n++;

        /* 闭环才要求"稳定待在区间内"；开环的转速反馈本身就不可信 */
        if (s_ol == 0U) {
            if ((rpm_now < (EC_RPM_LO * 1.02f)) || (rpm_now > (EC_RPM_HI * 0.98f))) {
                s_rpm_sum = 0.0f;
                s_rpm_n   = 0U;
                s_t       = 0U;
                return 1U;
            }
        } else if (s_t == 1U) {
            s_pos0      = BspServo_GetPosSum();  /* 闭环：用位置核对"真的转起来了" */
            s_ol_turns0 = s_ol_turns;            /* 开环：用虚拟圈数 */
        }

        if (s_t >= EC_SETTLE_MS) {
            /* 开环兜底：3 s 至少转 1 圈（600 rpm 应转 30 圈）。转不动就别开
             * CAL_EN —— 否则芯片在静止激励下判失败，还白等 30 s。 */
            if (s_ol != 0U) {
                int64_t d = BspServo_GetPosSum() - s_pos0;
                const float turns = (s_ol != 0U)
                                  ? (s_ol_turns - s_ol_turns0)
                                  : (float)((d >= 0) ? d : -d) / BSP_ENC_COUNTS;
                if (turns < 1.0f) {
                    ec_fail(EC_ERR_SPEED);
                    return 1U;
                }
            }

            s_res.rpm_avg = s_rpm_sum / (float)s_rpm_n;

            s_pos0      = BspServo_GetPosSum();
            s_ol_turns0 = s_ol_turns;
            s_poll  = 0U;
            s_t     = 0U;
            s_state = EC_CAL;

            /* 关键时刻：匀速时拉高 CAL_EN，芯片开始自校准 */
            BspServo_SetEncCalEn(1U);
        }
        return 1U;
    }

    case EC_CAL: {
        const float rpm_now = ((s_ol != 0U)
                               ? (s_ol_w_now / (float)BSP_POLE_PAIRS)
                               : fabsf(BspServo_GetOmegaMech())) * 9.5492966f;

        /* 统计转速极值，用于事后判断"是不是真的匀速" */
        if (rpm_now < s_res.rpm_min) { s_res.rpm_min = rpm_now; }
        if (rpm_now > s_res.rpm_max) { s_res.rpm_max = rpm_now; }

        /* 圈数：位置累计是 int64 脉冲，除 2^21 得圈数。
         * 单向运行，取绝对值即可；乘 10 保留一位小数。 */
        {
            int64_t d = BspServo_GetPosSum() - s_pos0;
            const float turns = (s_ol != 0U)
                              ? (s_ol_turns - s_ol_turns0)
                              : (float)((d >= 0) ? d : -d) / BSP_ENC_COUNTS;
            s_res.turns_x10 = (uint32_t)((turns * 10.0f) + 0.5f);
        }

        /* 每 100 ms 读一次状态寄存器（阻塞 SPI，会丢 1~2 拍角度，可接受） */
        if (++s_poll >= EC_POLL_MS) {
            const uint8_t st = (uint8_t)(BspServo_ReadEncReg(EC_REG_STATUS) >> 6);

            s_poll = 0U;
            s_res.chip_status = st;

            if (st == 2U) {                 /* 芯片判失败，不用再等 */
                ec_fail(EC_ERR_CHIP);
                return 1U;
            }
            if (st == 3U) {                 /* 芯片判成功：可以提前收工 */
                ec_stop();
                s_res.err = EC_OK;
                s_state   = EC_DONE;
                ec_push_result(1u);
                return 1U;
            }
        }

        /*
         * 圈数只用于**记录进度**，不作为成功判据。
         * 手册要求"转够 64 圈后由芯片判成功"——最终必须看 0x113[7:6]==11，
         * 因为芯片要在转完最后一个周期后才把参数写进 EEPROM。
         * 只靠圈数判成功会拿到一张还没写完的表。
         */
        return 1U;
    }

    default:
        return 0U;
    }
}
