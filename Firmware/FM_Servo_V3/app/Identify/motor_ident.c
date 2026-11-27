/*
 * motor_ident.c —— 极对数 / 方向 / 电角度零点辨识（VESC 三段式策略）
 *
 * 完整流程（全部在 TIM6 1 ms 上下文，20 kHz 控制节拍只负责执行电流环）：
 *
 *   RAMP   200 ms   电流 0 -> id_ref 线性上升，转子被吸到虚拟电角度 0
 *   STEP   1.9 s    虚拟角每次 +120°，正 30 次 + 反 30 次
 *                   -> dmech_avg = atan2(Σsin|Δθm|, Σcos|Δθm|)
 *                   -> ratio = 120° / dmech_avg,  dir = sign(正向组 Δθm)
 *   SWEEP_F 1.3 s   按 ratio 每电周期采 3 点，走满一个机械圈
 *   SWEEP_R 1.3 s   反向再走一遍，与正向合并
 *                   -> offset = atan2(Σsin φ, Σcos φ),  φ = wrap(θv - dir·ratio·θm)
 *   DONE            封锁 MOE，写零点 zero = z_old + offset/(dir·ratio)
 *
 * 三个设计要点（都是为了避开 FM_XC 那套实现里的坑）：
 *   1. 极对数用"角度差"不用"计数差"：丢 1~2 个脉冲只让 ratio 变动 0.1%，
 *      不会像 enc_line/Δcount 那样把误差放大；
 *   2. 圆形平均：所有角度平均走 atan2(Σsin, Σcos)，天然处理 0/2π 环绕，
 *      不需要 "|b1-b0| >= π 则 += 2π" 的特判；
 *   3. 正向 + 反向各扫一遍：齿槽与摩擦让转子"从正逼近"和"从负逼近"停在不同位置，
 *      两遍平均正好把这段滞后抵消掉。
 *
 * 与 FM_XC 的差别：那边默认参数（wait_time_s / test_angle_rad / angle_add_rad）
 * 全工程没有赋值，等于写完没调通；这里把每一步的时序都写成编译期常量，
 * 并且每次采样都做有效性判定（卡住/丢步的样本直接丢弃，而不是硬算一个值）。
 */
#include "motor_ident.h"
#include "bsp_servo.h"
#include "actuator.h"

#include <math.h>
#include <string.h>

/* ---------------- 时序与判据 ---------------- */
#define MI_STEP_ELEC       (2.09439510239f)   /* 120° 电角度 = 2π/3 */
#define MI_TWO_PI          (6.28318530718f)
#define MI_PI              (3.14159265359f)
#define MI_SETTLE_MS       (150U)             /* 每点稳定时间。历史 30 → 60 → 150 ms。
                                               * 150 ms 的依据（2026-11-20 实测，FAIL 行会打 mdeg）：
                                               *   2 A → 每步 637 mdeg（名义 8571）
                                               *   3 A → 2270 mdeg
                                               *   4 A → 5545 mdeg，但 max 已到 8724（接近名义）
                                               * 说明多数步是被稳定时间截断，不是转矩不够：
                                               * 60 ms 时转子还在粘滑爬行，读数只取到中途位置，
                                               * ratio 被算成 22。代价是总时长约 22 s，
                                               * 故 MI_TIMEOUT_MS 同步放宽。 */
#define MI_STEP_N          (30U)              /* 单向步进次数（正 30 + 反 30 = 20 个电周期） */
#define MI_SWEEP_PER_REV   (3U)               /* 每电周期采样点数（120° 一点） */
#define MI_RAMP_MS         (200U)
#define MI_ID_MIN          (0.5f)
#define MI_ID_MAX          (5.0f)
#define MI_TIMEOUT_MS      (45000U)           /* 硬超时。150 ms/点下 STEP 9.1 s + SWEEP 12.7 s
                                               * ≈ 22 s，原 20 s 会在扫整圈中途掐断 */
#define MI_DMECH_MIN       (0.002f)           /* 单步机械角下限（rad）。
                                               * 14 极对应为 0.1496；低于 2 mrad 视为没动 */
#define MI_RATIO_MIN       (1.0f)
#define MI_RATIO_MAX       (64.0f)
#define MI_RATIO_TOL       (0.40f)            /* |ratio - round(ratio)| 超过即判不可信。
                                               * 下游只用 ratio_int（整数极对数），
                                               * 小数部分仅作可信度参考，所以 0.25 偏严：
                                               * 13.6~14.4 都能正确取整到 14，没必要
                                               * 因为 0.3 的摩擦滞后就整轮失败。 */

/* ---- 连续拖动法（2026-11-20 替换 120° 阶跃）----
 * 阶跃法的根本问题是恒定电流矢量 = 无阻尼单摆：4 A 下同步刚度约 5 N·m/rad、
 * J=7.67e-5，固有频率 41 Hz，150 ms 稳定窗里摆 6 个周期，采样点相位随机，
 * 实测同一步位移 244~>10000 mdeg（40 倍离散）。连续拖动无阶跃，转子准静态
 * 跟随，摩擦滞后在正反两遍里符号相反、平均抵消。 */
#define MI_SPIN_W_E        (15.708f)   /* 拖动电角速度 rad/s = 2π·2.5 Hz
                                        * 机械 2.5/14 = 0.179 rev/s ≈ 10.7 rpm */
#define MI_SPIN_RAMP_MS    (600U)      /* 0 → ωe 的速度斜坡，避免起拖失步 */
#define MI_SPIN1_MS        (4200U)     /* ① 比例段：3.6 s 匀速 ≈ 0.64 机械圈 */
#define MI_SPIN2_MS        (7000U)     /* ② 零点段：单方向 ≈ 1.07 机械圈，正反各一遍 */
#define MI_SPIN_SMP_MS     (5U)        /* 零点段采样间隔 */
#define MI_SPIN_SETTLE_MS  (150U)     /* 停转后等转子停稳再读机械角。
                                       * 原来一停就立刻读 p_now，转子还在惯性滑行，
                                       * 机械角偏大 → ratio 系统性偏小（实测 14 极
                                       * 读成 13.92）。150 ms 足够 14 极电机在
                                       * ωe=0 的定位转矩下停住。 */
/* 故障注入（只用于验证重试分支，出货必须为 0）：
 * 1 = 第 1 次尝试把机械角人为加 5%，ratio 会算成 ~13.3 → 必判 RATIO 失败，
 *     从而强制走重试；第 2/3 次不注入，应当成功。
 * 用来确认"重试计数、回 RAMP 重取基准、最终能成功"这条链路真的通。 */
#ifndef MI_TEST_BAD_FIRST
#define MI_TEST_BAD_FIRST  (0)
#endif

#if (PROD_BUILD != 0)
/* 生产构建绝不允许注入：就算有人把上面的宏改成 1，这里也强制归零 */
#undef  MI_TEST_BAD_FIRST
#define MI_TEST_BAD_FIRST  (0)
#endif

#define MI_RATIO_RETRY     (2U)       /* RATIO 判定失败后的自动重试次数。
                                       * 拖拽法的误差主要来自摩擦滞后和起拖瞬间，
                                       * 两次独立结果不相关，重试能把偶发失败
                                       * 压下去。每次只重跑 SPIN1（+4.2 s），
                                       * 三次合计约 27 s，仍在 45 s 硬超时内。 */
#define MI_SPIN_MIN_RAD    (0.35f)     /* 3.6 s 连 20° 都走不满 -> 判卡死 */            /* |ratio - round(ratio)| 超过即判不可信 */

#define MI_ENC_RAD_PER_CNT (MI_TWO_PI / BSP_ENC_COUNTS)

/* 模型侧的固定电角度相位（rad）。
 * three_loop_deploy.c:48 是 `theta_sum = 14*theta_m + 1.57079632679 + 0.00105*omega_m`，
 * 也就是说模型认为 theta_m = 0 时电角度是 +90°。零点必须把这段 π/2 一起吃掉，
 * 否则辨识得再准，闭环电角度也恒差 90°——力矩要么退化成 0（正交）要么反向。
 * 这个常数与模型强绑定：一旦 Simulink 侧改了 theta_off，这里必须同步。 */
#define MI_MODEL_ELEC_OFS  (1.57079632679f)

/* ---------------- 状态 ---------------- */
static volatile MotorIdentState s_state = MOTOR_IDENT_IDLE;
static MotorIdentResult   s_res;
static float              s_idref;

static uint16_t           s_t;          /* 当前阶段内的 tick */
static uint16_t           s_p;          /* 当前阶段内的点序号 */
static uint16_t           s_npoint;     /* 扫描阶段总点数 */
static uint8_t            s_ratio_retry; /* RATIO 失败后的已重试次数 */
static uint32_t           s_total_ms;

static float              s_theta_v;    /* 虚拟电角度，始终 wrap 在 [-π, π] */
static float              s_th_e;       /* 连续拖动：命令电角累计（rad，单调） */

/* RAMP 结束时记下的基准（三个量必须同一时刻读，差一拍只是 50 us 的角度误差） */
static int64_t            s_p_start;
static uint32_t           s_raw_start;
static float              s_theta_start;

/* STEP 阶段累积 */
static int64_t            s_p_prev;
static float              s_ssin, s_scos;
static float              s_dsum_fwd;   /* 正向组的带符号角差之和，只用来判方向 */

/* SWEEP 阶段累积（正反两遍共用，这样两遍自然等权平均） */
static float              s_osin, s_ocos;

/* ---------------- 小工具 ---------------- */

static float wrap_pi(float x)
{
    /* 标准 wrap 到 [-π, π]：k = round(x/2π) */
    x -= MI_TWO_PI * floorf((x * (1.0f / MI_TWO_PI)) + 0.5f);
    return x;
}

static void finish_fail(MotorIdentErr e)
{
    /* 顺序：先撤虚拟角（让控制节拍回到故障路径）再封 MOE，与 servo_cmd 一致 */
    Servo_SetOpenLoop(0U, 0.0f, 0.0f);
    BspServo_PwmEnable(0U);
    s_res.err = e;
    s_state   = MOTOR_IDENT_FAIL;
}

static void finish_ok(void)
{
    float offset = atan2f(s_osin, s_ocos);
    float z_old  = s_theta_start - ((float)s_raw_start * MI_ENC_RAD_PER_CNT);
    /* Δz 推导：
     *   辨识给出 dir·ratio·θm + φ ≡ θv，而模型算 ratio·θm' + π/2 要等于真实电角度。
     *   解出 θm' = θm + dir·(φ − π/2)/ratio，其中 dir 乘子是"把 signed 坐标的增量
     *   换算回原始编码器坐标"的符号（1/dir = dir，因为 dir ∈ {+1,−1}）。 */
    float delta  = (((offset - MI_MODEL_ELEC_OFS) / (float)s_res.ratio_int)
                    * (float)s_res.dir);
    float z_new  = z_old + delta;

    /* 先封锁输出再写零点：写零点会立刻改变 theta，若控制律还在跑会用旧角度继续 */
    Servo_SetOpenLoop(0U, 0.0f, 0.0f);
    BspServo_PwmEnable(0U);

    /* 符号必须在写零点之前生效：下一拍控制律就要用新的 theta 定义 */
    Servo_SetDirSign(s_res.dir);
    BspServo_SetEncoderZero(z_new);

    s_res.offset_rad = offset;
    s_res.zero_mech  = z_new;
    s_res.err        = MOTOR_IDENT_OK;
    s_state          = MOTOR_IDENT_DONE;
}

/* ---------------- 各阶段 ---------------- */

static uint8_t step_ramp(void)
{
    s_t++;
    if (s_t <= MI_RAMP_MS) {
        float k = (float)s_t / (float)MI_RAMP_MS;
        Servo_SetOpenLoop(1U, s_theta_v, s_idref * k);
        return 0U;
    }

    /* 转子已稳定在虚拟电角度 0，记录基准并进入步进 */
    s_p_start     = BspServo_GetPosSum();
    s_raw_start   = BspServo_GetEncoderRaw();
    s_theta_start = BspServo_GetThetaCached();
    s_p_prev      = s_p_start;

    s_ssin = 0.0f; s_scos = 0.0f; s_dsum_fwd = 0.0f;
    s_t = 0U;
    s_p = 0U;
    s_state = MOTOR_IDENT_STEP;
    return 0U;
}

static uint8_t spin_finish_ratio(float th_e, float th_m)
{
    float am = fabsf(th_m);

    s_res.dir = (th_m >= 0.0f) ? 1 : -1;

    if (am < MI_SPIN_MIN_RAD) {
        /* 转了 3.6 s 连 20° 都不到 —— 卡死 / 功率线 / 抱闸 */
        s_res.ratio     = 0.0f;
        s_res.ratio_int = 0U;
        s_res.dmech_avg = am;
        s_res.dmech_min = am;
        s_res.dmech_max = am;
        finish_fail(MOTOR_IDENT_ERR_STUCK);
        return 1U;
    }

    s_res.ratio     = th_e / am;
    s_res.ratio_int = (uint16_t)(s_res.ratio + 0.5f);
    s_res.resid     = fabsf(s_res.ratio - (float)s_res.ratio_int);

    /* 回显统一成"每 120° 电角对应的机械角"，与旧步进法同量纲（正常 8571 mdeg），
     * 方便与历史数据横向对比。 */
    s_res.dmech_avg = am * (MI_STEP_ELEC / th_e);
    s_res.dmech_min = s_res.dmech_avg;
    s_res.dmech_max = s_res.dmech_avg;

    if ((s_res.ratio_int != (uint16_t)BSP_POLE_PAIRS) || (s_res.resid > MI_RATIO_TOL)) {
        if (s_ratio_retry < MI_RATIO_RETRY) {
            s_ratio_retry++;
            s_t = 0U;
            s_p = 0U;
            s_state = MOTOR_IDENT_RAMP;   /* 回 ramp：重新升流并重取机械基准，
                                           * 不是简单重跑同一段 */
            return 0U;
        }
        finish_fail(MOTOR_IDENT_ERR_RATIO);
        return 1U;
    }

    s_osin = 0.0f; s_ocos = 0.0f;
    s_res.n_sweep = 0U;
    s_t = 0U;
    s_state = MOTOR_IDENT_SWEEP_FWD;
    return 0U;
}

/* ① 连续正转，测极对数：ratio = 命令电角累计 ÷ 编码器机械角累计 */
static uint8_t step_step(void)
{
    s_t++;

    if (s_t == 1U) {
        s_th_e       = 0.0f;
        s_res.n_step = 0U;
        s_res.n_fail = 0U;
        Servo_SetOLSpin(1U, 0.0f);        /* 角由 20 kHz ISR 累加，这里只给速度 */
        return 0U;
    }

    {
        float k = (float)(s_t - 1U) / (float)MI_SPIN_RAMP_MS;
        float w;

        if (k > 1.0f) { k = 1.0f; }
        w = MI_SPIN_W_E * k;
        Servo_SetOLSpin(1U, w);
        s_th_e += w * 0.001f;             /* 命令电角精确积分，不依赖编码器 */
        s_res.n_step++;
    }

    if (s_t < MI_SPIN1_MS) { return 0U; }

    if (s_t == MI_SPIN1_MS) {
        Servo_SetOLSpin(0U, 0.0f);        /* 先停：定位转矩会把转子拉住 */
        return 0U;
    }
    /* 停稳后再读机械角。原来一停就立刻读，转子还在滑行，th_m 偏大，
     * ratio 系统性偏小（14 极读成 13.92）—— 这是本函数最容易忽略的误差源。 */
    if (s_t < (MI_SPIN1_MS + MI_SPIN_SETTLE_MS)) { return 0U; }

    {
        int64_t p_now = BspServo_GetPosSum();
#if (MI_TEST_BAD_FIRST != 0)
        if ((MI_TEST_BAD_FIRST >= 2) || (s_ratio_retry == 0U)) {
            p_now += (int64_t)((float)BSP_ENC_COUNTS / 20.0f);   /* 偏 5% */
        }
#endif
        {
            const float th_m = (float)(p_now - s_p_start) * MI_ENC_RAD_PER_CNT;
            return spin_finish_ratio(s_th_e, th_m);
        }
    }
}

/* ② 连续扫整圈，累加相位误差 -> 零点。正反各一遍。
 * 注意：**不重置** s_p_start / s_theta_start（在 step_ramp 里与 s_raw_start
 * 同一时刻记录），否则 θm 变成相对量，φ 会带一个常量偏移，零点直接错。 */
static uint8_t step_sweep(void)
{
    float dir_w;

    s_t++;

    if (s_t == 1U) {
        Servo_SetOLSpin(1U, 0.0f);        /* 反向那一遍先停一拍再换向 */
        return 0U;
    }

    dir_w = (s_state == MOTOR_IDENT_SWEEP_FWD) ? MI_SPIN_W_E : -MI_SPIN_W_E;

    {
        float k = (float)(s_t - 1U) / (float)MI_SPIN_RAMP_MS;

        if (k > 1.0f) { k = 1.0f; }
        Servo_SetOLSpin(1U, dir_w * k);
    }

    if (s_t < MI_SPIN2_MS) {
        if ((s_t % MI_SPIN_SMP_MS) == 0U) {
            const int64_t p_now  = BspServo_GetPosSum();
            const float   th_m   = s_theta_start +
                                   ((float)(p_now - s_p_start) * MI_ENC_RAD_PER_CNT);
            const float   e_elec = (float)s_res.dir * (float)s_res.ratio_int * th_m;
            const float   phi    = wrap_pi(Servo_GetOLTheta() - e_elec);

            s_osin += sinf(phi);
            s_ocos += cosf(phi);
            s_res.n_sweep++;
        }
        return 0U;
    }

    Servo_SetOLSpin(0U, 0.0f);

    if (s_state == MOTOR_IDENT_SWEEP_FWD) {
        s_t = 0U;
        s_state = MOTOR_IDENT_SWEEP_REV;
        return 0U;
    }

    finish_ok();
    return 1U;
}

/* ---------------- 公开接口 ---------------- */

MotorIdentErr MotorIdent_Start(float id_ref_a)
{
    float r = id_ref_a;

    if (MotorIdent_IsBusy() != 0U)             { return MOTOR_IDENT_ERR_BUSY; }
    /* MOE 已经开着说明有闭环在跑（使能状态），这里再挡一道。
     * 原来只靠 do_align() 的 s_armed 检查，换条调用路径就漏了。 */
    if (BspServo_IsPwmEnabled() != 0U)         { return MOTOR_IDENT_ERR_BUSY; }
    if (BspServo_IsCurrentCalibrated() == 0U)  { return MOTOR_IDENT_ERR_CAL; }
    if (BspServo_GetFault() != 0U)             { return MOTOR_IDENT_ERR_FAULT; }
    /* 执行器占用仲裁：占不到就返回 busy，别让两个辨识互相踩 */
    if (Act_Acquire(ACT_MOTOR_IDENT, MotorIdent_IsBusy) == 0U) { return MOTOR_IDENT_ERR_BUSY; }

    if (r < MI_ID_MIN) { r = MI_ID_MIN; }
    if (r > MI_ID_MAX) { r = MI_ID_MAX; }

    memset(&s_res, 0, sizeof(s_res));
    s_res.dmech_min = 1.0e9f;
    s_res.dmech_max = -1.0e9f;

    s_idref    = r;
    s_theta_v  = 0.0f;
    s_t        = 0U;
    s_p        = 0U;
    s_npoint   = 0U;
    s_ratio_retry = 0U;
    s_total_ms = 0U;
    s_ssin = 0.0f; s_scos = 0.0f; s_dsum_fwd = 0.0f;
    s_osin = 0.0f; s_ocos = 0.0f;

    Servo_SetOpenLoop(1U, s_theta_v, 0.0f);    /* 从 0 起，由 RAMP 升流 */
    BspServo_PwmEnable(1U);
    if (BspServo_IsPwmEnabled() == 0U) {       /* MOE 静默失败必须回读，不能假定成功 */
        Servo_SetOpenLoop(0U, 0.0f, 0.0f);
        s_res.err = MOTOR_IDENT_ERR_PWM;
        s_state   = MOTOR_IDENT_FAIL;
        return MOTOR_IDENT_ERR_PWM;
    }

    s_state = MOTOR_IDENT_RAMP;
    return MOTOR_IDENT_OK;
}

uint8_t MotorIdent_Task1ms(void)
{
    if ((s_state == MOTOR_IDENT_IDLE) ||
        (s_state == MOTOR_IDENT_DONE) ||
        (s_state == MOTOR_IDENT_FAIL)) {
        return 0U;
    }

    s_total_ms++;
    if (s_total_ms > MI_TIMEOUT_MS) { finish_fail(MOTOR_IDENT_ERR_TIMEOUT); return 1U; }
    if (BspServo_GetFault() != 0U)  { finish_fail(MOTOR_IDENT_ERR_FAULT_RUN); return 1U; }

    switch (s_state) {
    case MOTOR_IDENT_RAMP: return step_ramp();
    case MOTOR_IDENT_STEP: return step_step();
    case MOTOR_IDENT_SWEEP_FWD:
    case MOTOR_IDENT_SWEEP_REV: return step_sweep();
    default:
        finish_fail(MOTOR_IDENT_ERR_TIMEOUT);
        return 1U;
    }
}

void MotorIdent_Abort(void)
{
    if (MotorIdent_IsBusy() != 0U) {
        Servo_SetOpenLoop(0U, 0.0f, 0.0f);
        BspServo_PwmEnable(0U);
    }
    s_state   = MOTOR_IDENT_IDLE;
    s_res.err = MOTOR_IDENT_OK;
}

uint8_t MotorIdent_IsBusy(void)
{
    return ((s_state != MOTOR_IDENT_IDLE) &&
            (s_state != MOTOR_IDENT_DONE) &&
            (s_state != MOTOR_IDENT_FAIL)) ? 1U : 0U;
}

void MotorIdent_SetCached(uint16_t ratio_int, float ratio, int8_t dir)
{
    if (ratio_int == 0U) { return; }
    if (MotorIdent_IsBusy() != 0U) { return; }   /* 正在跑就别覆盖 */
    s_res.ratio_int = ratio_int;
    s_res.ratio     = ratio;
    s_res.dir       = (dir < 0) ? -1 : 1;
    s_res.err       = MOTOR_IDENT_OK;
}

MotorIdentState MotorIdent_GetState(void) { return s_state; }

const MotorIdentResult *MotorIdent_GetResult(void) { return &s_res; }

const char *MotorIdent_StateStr(void)
{
    switch (s_state) {
    case MOTOR_IDENT_IDLE:      return "IDLE";
    case MOTOR_IDENT_RAMP:      return "RAMP";
    case MOTOR_IDENT_STEP:      return "STEP";
    case MOTOR_IDENT_SWEEP_FWD: return "SWEEP_F";
    case MOTOR_IDENT_SWEEP_REV: return "SWEEP_R";
    case MOTOR_IDENT_DONE:      return "DONE";
    case MOTOR_IDENT_FAIL:      return "FAIL";
    default:                    return "?";
    }
}

const char *MotorIdent_ErrStr(MotorIdentErr e)
{
    switch (e) {
    case MOTOR_IDENT_OK:          return "ok";
    case MOTOR_IDENT_ERR_BUSY:    return "busy";
    case MOTOR_IDENT_ERR_CAL:     return "no_cal";
    case MOTOR_IDENT_ERR_FAULT:   return "fault";
    case MOTOR_IDENT_ERR_PWM:     return "pwm_off";
    case MOTOR_IDENT_ERR_TIMEOUT: return "timeout";
    case MOTOR_IDENT_ERR_STUCK:   return "stuck";
    case MOTOR_IDENT_ERR_RATIO:   return "ratio_bad";
    case MOTOR_IDENT_ERR_FAULT_RUN: return "fault_run";
    default:                      return "?";
    }
}
