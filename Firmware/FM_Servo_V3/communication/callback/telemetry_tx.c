/*
 * telemetry_tx.c —— 上位机遥测发送：Iq / Id / 母线电压（100 Hz，USART1 DMA）
 *
 * 报文（MAVLink v2，0xFD 帧头）：
 *   30025 CurrentCtlInput   LEN=57 crc_extra=191   设备→主机
 *     +0  idq_tar_A[2]            f32   Id 给定 0 / Iq 给定（限幅后 iq_used）
 *     +8  iabc_now_A[3]           f32   三相实测电流 iu / iv / iw
 *     +20 elec_theta_rad          f32   电角度
 *     +24 dc_bus_now_V            f32   母线电压
 *     +28 uabc_tar_V[3]           f32   相电压指令 (duty-0.5)*vbus
 *     +40 driver_temp             f32   NTC_GAN 温度 °C（PA4/ADC2_IN17；开路短路为 NaN）
 *     +44 elec_angle_speed_rad_s  f32   电角速度
 *     +48 iq_offset_A             f32   0
 *     +52 mode                    u8    2
 *     +53 motor_temp              f32   NTC_MOTOR 温度 °C（PC4/ADC2_IN5；LEN 53→57 新增）
 *       ★ 旧上位机按帧头 len 切片、只读已知偏移，尾部多 4 B 不影响兼容
 *   30027 CurrentCtlOutput  LEN=32 crc_extra=58    设备→主机
 *     +0  uabc_tar_comp_V[3]      f32
 *     +12 idq_now_A[2]            f32   Id / Iq 实测（与生成代码同一 Park 变换）
 *     +20 uabc_tar_org_V[3]       f32
 *   30040 MotorPosSensorOutput LEN=12 crc_extra=160  设备→主机
 *     +0  enc_sum_p               int64 相对上电点的累积编码器脉冲（2^21/圈）
 *     +8  elec_angle_now_rad      f32   电角度（与 30025 +20 同源，不含 π/2 补偿）
 *
 * 上位机换算：圈数 = enc_sum_p / 2097152；机械角 = (enc_sum_p mod 2097152)/2097152*2π
 *
 * 设计约束（对应《ADC注入ISR与BSP驱动说明》的实时纪律）：
 *   1. 组帧只在 1 ms 任务里做（10 分频 = 100 Hz），绝不进 20 kHz 电流环 ISR；
 *   2. UART 走 DMA 非阻塞；上一帧未发完直接丢帧并计数，不阻塞控制节拍；
 *   3. 三条报文拼成一次 DMA 传输（133 B ≈ 1.44 ms @921600），避免半帧交错；
 *   4. idq 用与 three_loop_deploy.c 完全相同的变换（含 π/2 与 0.00105·ωm 补偿），
 *      保证上位机看到的 Iq/Id 就是控制律内部用的值。
 *
 * 局限性：本文件是调试遥测的临时通道，字段取自 X72 方言的“控制环内量”分组；
 *         量产阶段应由 communication/ 协议层统一发报，不在 Board 层直发 UART。
 */
#include "telemetry_tx.h"
#include "bsp_servo.h"
#include "servo_cmd.h"
#include "cog_ff.h"          /* 30090 状态字的 cog.valid 位 */
#include "deadband_comp.h"   /* 30090 状态字的死区补偿位 */
#include "mavlink_common.h"  /* MAV_ST_* flags 位定义 */
#include "app_param.h"       /* AppParam_GetControlWord()：30090 要回真实控制字 */
#include "usart.h"

#include <math.h>
#include <string.h>

#define MSG_CUR_IN        30025u
#define MSG_CUR_OUT       30027u
#define MSG_POS_SENSOR    30040u
#define MSG_CTRL_STATUS   30090u
#define MSG_LOOP_STATUS   30091u
#define MSG_IDENT_RESULT  30092u
#define MSG_POS_STATUS    30100u
#define CRC_EXTRA_IN      191u
#define CRC_EXTRA_OUT     58u
#define CRC_EXTRA_POS     160u
#define CRC_EXTRA_ST      90u
#define CRC_EXTRA_LP      91u
#define CRC_EXTRA_ID      92u
#define CRC_EXTRA_PS      100u
#define LEN_IN            61u   /* 57 + spd_ref_rpm；上位机按 payload.length>=61 判断 */
#define LEN_OUT           32u
#define LEN_POS           12u
#define LEN_ST            40u    /* 30090 控制状态字，替代 ASCII 的 S 行 */
#define LEN_LP            48u    /* 30091 环路状态，替代 ASCII 的 K 行 */
#define LEN_ID            40u    /* 30092 辨识结果事件帧 */
#define LEN_PS            48u    /* 30100 位置环状态（位置/目标/误差/状态机） */
#define FRAME_IN          (10u + LEN_IN + 2u)      /* 73 B */
#define FRAME_OUT         (10u + LEN_OUT + 2u)     /* 44 B */
#define FRAME_POS         (10u + LEN_POS + 2u)     /* 24 B */
#define FRAME_ST          (10u + LEN_ST + 2u)      /* 52 B */
#define FRAME_LP          (10u + LEN_LP + 2u)      /* 60 B */
#define FRAME_ID          (10u + LEN_ID + 2u)      /* 52 B */
#define FRAME_PS          (10u + LEN_PS + 2u)      /* 60 B */

#define TX_DIVIDER        10u                      /* 1 kHz / 10 = 100 Hz */
#define TX_STUCK_MAX      20u                      /* 连续 200 ms 发不出去即判定链路卡死 */
#define POLE_PAIRS        14.0f                    /* 与 three_loop_deploy.c 的 14.0 一致 */
#define ELEC_OFFSET       1.57079632679f           /* 生成代码 theta_sum 的 +π/2 */
#define ELEC_COMP         0.00105f                 /* 生成代码的 0.00105*omega_m */

#define MODE_REPORT       2u                       /* 2 = 速度模式（仅用于显示） */

static uint8_t  s_frame[FRAME_IN + FRAME_OUT + FRAME_POS + FRAME_ST + FRAME_LP + FRAME_ID + FRAME_PS];
static uint8_t  s_seq;
static uint8_t  s_busy_snap;                       /* 最近一次 DMA 启动失败时的 (gState<<4)|hdma->State */
static uint32_t s_div;
static uint32_t s_tx_ok;
static uint32_t s_tx_drop;
static uint32_t s_tx_stuck;                        /* 连续忙计数 */
static uint32_t s_tx_recover;                      /* 强制中止次数 */
static uint8_t  s_ready;

/* ---------------- CRC-16/X.25（MAVLink 标准，含 crc_extra） ---------------- */
static uint16_t crc_accum(uint8_t b, uint16_t crc)
{
    uint8_t t = (uint8_t)(b ^ (uint8_t)(crc & 0xFFu));
    t ^= (uint8_t)(t << 4);
    return (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4));
}

static uint16_t crc_calc(const uint8_t *p, uint16_t n, uint8_t extra)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0u; i < n; i++) {
        crc = crc_accum(p[i], crc);
    }
    return crc_accum(extra, crc);
}

/* ---------------- 帧组装 ---------------- */
static uint16_t frame_build(uint8_t *f, uint32_t msgid, uint8_t seq,
                            const uint8_t *pl, uint16_t len, uint8_t extra)
{
    uint16_t crc;

    f[0] = 0xFDu;                          /* STX */
    f[1] = (uint8_t)len;                   /* payload 长度 */
    f[2] = 0u;                             /* incompat */
    f[3] = 0u;                             /* compat */
    f[4] = seq;                            /* 序号 */
    f[5] = 1u;                             /* sysid */
    f[6] = 1u;                             /* compid */
    f[7] = (uint8_t)(msgid & 0xFFu);
    f[8] = (uint8_t)((msgid >> 8) & 0xFFu);
    f[9] = (uint8_t)((msgid >> 16) & 0xFFu);
    memcpy(&f[10], pl, len);

    /*
     * MAVLink v2 标准 CRC 覆盖范围：从 len 字节（f[1]）起，含 len/incompat/
     * compat/seq/sysid/compid/msgid(3) + payload，共 9+len 字节，最后追加
     * crc_extra。★ 不含帧首 0xFD 魔数。
     *
     * 旧实现从 f[0] 起算（10+len），自洽但不符合规范：pymavlink / QGC /
     * Mission Planner 会把这些帧全部判为 CRC 错。抓包实证（2134 帧）：
     * 含魔数口径 2134/2134 命中，标准口径 0/2134。
     *
     * 同步改动的工具：Agnet/serial_link_check.py、verify_host_keyparams.py、
     * verify_fw_frame.py、mavlink_proto.py 与 servo-web/index.html 的 mavCrc
     * 调用点。详见 Agnet/CRC口径与seq修正说明.md。
     */
    crc = crc_calc(&f[1], (uint16_t)(9u + len), extra);
    f[10u + len] = (uint8_t)(crc & 0xFFu);
    f[11u + len] = (uint8_t)(crc >> 8);

    return (uint16_t)(12u + len);
}

/* Cortex-M4 为小端，float 直接内存拷贝即符合帧格式的 LE 要求 */
static void put_f32(uint8_t *p, float v)   { memcpy(p, &v, sizeof(float)); }
static void put_i64(uint8_t *p, int64_t v) { memcpy(p, &v, sizeof(int64_t)); }

void Telemetry_Init(void)
{
    s_seq        = 0u;
    s_busy_snap  = 0u;
    s_div       = 0u;
    s_tx_ok     = 0u;
    s_tx_drop   = 0u;
    s_tx_stuck  = 0u;
    s_tx_recover = 0u;
    s_ready     = 0u;          /* 默认闸门关闭：等 Telemetry_Start() 再开流 */
}

void Telemetry_Start(void)
{
    s_ready = 1u;
}

/* ---------------- 100 Hz 组帧发送 ---------------- */
static volatile uint8_t s_tx_pause;

/* ---- 30092 辨识结果推送槽 ---- */
static uint8_t s_id_pay[LEN_ID];
static uint8_t s_id_left;        /* 还要发几帧 */

void Telemetry_PushIdentResult(uint8_t kind, uint8_t ok, uint16_t n,
                               const float *v8, float resid)
{
    uint8_t k;

    memset(s_id_pay, 0, sizeof(s_id_pay));
    s_id_pay[0] = kind;
    s_id_pay[1] = ok;
    s_id_pay[2] = (uint8_t)(n & 0xFFu);
    s_id_pay[3] = (uint8_t)((n >> 8) & 0xFFu);
    for (k = 0u; (k < 8u) && (v8 != 0); k++) {
        memcpy(&s_id_pay[4u + k * 4u], &v8[k], sizeof(float));
    }
    memcpy(&s_id_pay[36], &resid, sizeof(float));

    /* 与发送侧抢同一个槽：短暂关中断，保证三帧内容一致 */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_id_left = 5u;
        if (primask == 0U) { __enable_irq(); }
    }
}

void Telemetry_SetTxPause(uint8_t pause) { s_tx_pause = (pause != 0U) ? 1U : 0U; }

void Telemetry_Task1ms(void)
{
    float iu, iv, iw, iq_ref, duty[3];
    float theta_m, omega_m, vbus;
    float th, cs, sn, ialpha, ibeta, id_now, iq_now;
    float ua, ub, uc, temp_gan, temp_motor;
    float spd_ref_rpm;
    uint8_t  p_in[LEN_IN];
    uint8_t  p_out[LEN_OUT];
    uint8_t  p_pos[LEN_POS];
    uint8_t  p_st[LEN_ST];
    uint8_t  p_lp[LEN_LP];
    uint8_t  p_id[LEN_ID];
    uint8_t  p_ps[LEN_PS];
    uint16_t n_in, n_out, n_pos, n_st, n_lp, n_id, n_ps;
    uint8_t  seq0;
    int64_t  enc_sum;

    if (s_ready == 0u) return;
    /* ASCII 行正在独占 UART：本拍直接让路，不推进分频，避免打乱 100 Hz 节拍 */
    if (s_tx_pause != 0U) return;
    if (++s_div < TX_DIVIDER) return;
    s_div = 0u;

    /* 上一帧 DMA 还没发完：本拍丢弃，绝不在 1 ms 任务里等 */
    if (huart1.gState != HAL_UART_STATE_READY) {
        s_tx_drop++;
        /*
         * 自愈：连续 20 拍（200 ms）都是忙，说明 TC 中断没被服务或 DMA 出错，
         * 链路已经死了。强制中止一次把 gState 拉回 READY，避免遥测永久停发。
         * 正常情况下 133 B @921600 只需 1.44 ms，绝不可能连续忙 200 ms。
         */
        if (++s_tx_stuck >= TX_STUCK_MAX) {
            (void)HAL_UART_AbortTransmit(&huart1);
            huart1.gState = HAL_UART_STATE_READY;
            s_tx_stuck    = 0u;
            s_tx_recover++;
        }
        return;
    }
    s_tx_stuck = 0u;

    Servo_GetSample(&iu, &iv, &iw, &iq_ref, duty);
    theta_m = BspServo_GetThetaCached();   /* 只读：绝不在此上下文推进测速低通 */
    omega_m = BspServo_GetOmegaMech();
    vbus    = BspServo_GetVbus();
    enc_sum = BspServo_GetPosSum();
    /* 速度环斜坡后的给定，换算回**用户坐标系** rpm（传感器正方向）。
     * 内部 s_spd_ref_ramp 在模型坐标系（dir 已乘），所以这里再乘一次 dir 还原。
     * 上位机拿它与 elec_angle_speed_rad_s 一起画，就能看出跟踪误差与超调。 */
    spd_ref_rpm = Servo_GetSpeedRampRef() * 60.0f / 6.283185307179586f;

    /* idq 直接取控制律算好的值，**不要在这里重算 Park**。
     *
     * 2026-11-17 修正：原来这里用 iu/iv 自己算 Clarke+Park，结果在同一工况下
     * 报出的 id 波动（sd 0.129 A @ id=2 A）比 ISR 内部真实值（sd 0.033 A）
     * 大 4 倍。角度用的是 GetThetaCached（滞后一拍）、电流是 s_iu/s_iv 快照，
     * 两者与 ISR 当拍计算存在相位/时序差 —— 静止时看不出，一有电流就放大成
     * 假噪声。之前据此判断"电流采样噪声大"并去改 ADC 采样时间和触发点，
     * 方向是错的：采样本身干净，是观测路径的问题。
     *
     * s_cur_id/s_cur_iq 在电流分支和模型分支都会更新（见 servo_isr.c 的注释），
     * 所以这里取到的就是控制实际用的那个 dq 值。
     * 仍保留 th 的计算？不需要了 —— ialpha/ibeta/th 只服务于这两个量。 */
    Servo_GetIdqMeas(&id_now, &iq_now);

    ua = (duty[0] - 0.5f) * vbus;
    ub = (duty[1] - 0.5f) * vbus;
    uc = (duty[2] - 0.5f) * vbus;

    temp_gan     = BspServo_GetTempGan();      /* NTC_GAN  (PA4/ADC2_IN17) */
    temp_motor   = BspServo_GetTempMotor();    /* NTC_MOTOR(PC4/ADC2_IN5)  */

    /* 组帧前先取序号基准：三条报文 seq = seq0, seq0+1, seq0+2 */
    seq0 = s_seq;

    /* ---- 30025 CurrentCtlInput ---- */
    memset(p_in, 0, sizeof(p_in));
    put_f32(&p_in[0],  0.0f);                 /* idq_tar_A[0] = Id 给定（控制律恒为 0） */
    put_f32(&p_in[4],  iq_ref);               /* idq_tar_A[1] = Iq 给定 */
    put_f32(&p_in[8],  iu);                   /* iabc_now_A[0] */
    put_f32(&p_in[12], iv);                   /* iabc_now_A[1] */
    put_f32(&p_in[16], iw);                   /* iabc_now_A[2] */
    put_f32(&p_in[20], POLE_PAIRS * theta_m); /* elec_theta_rad */
    put_f32(&p_in[24], vbus);                 /* dc_bus_now_V   ★母线电压 */
    put_f32(&p_in[28], ua);                   /* uabc_tar_V[0] */
    put_f32(&p_in[32], ub);                   /* uabc_tar_V[1] */
    put_f32(&p_in[36], uc);                   /* uabc_tar_V[2] */
    put_f32(&p_in[40], temp_gan);             /* driver_temp：NTC_GAN 温度（°C），开路/短路为 NaN */
    put_f32(&p_in[44], POLE_PAIRS * omega_m); /* elec_angle_speed_rad_s */
    put_f32(&p_in[48], 0.0f);                 /* iq_offset_A */
    p_in[52] = MODE_REPORT;                   /* mode */
    put_f32(&p_in[53], temp_motor);           /* motor_temp：NTC_MOTOR 温度（°C），LEN 53→57 新增 */
    put_f32(&p_in[57], spd_ref_rpm);          /* spd_ref_rpm：速度环斜坡后给定（rpm），LEN 57→61 新增 */

    /* ---- 30027 CurrentCtlOutput ---- */
    memset(p_out, 0, sizeof(p_out));
    put_f32(&p_out[0],  ua);                  /* uabc_tar_comp_V[0]（未做补偿） */
    put_f32(&p_out[4],  ub);
    put_f32(&p_out[8],  uc);
    put_f32(&p_out[12], id_now);              /* idq_now_A[0] = Id 实测 */
    put_f32(&p_out[16], iq_now);              /* idq_now_A[1] = Iq 实测 */
    put_f32(&p_out[20], ua);                  /* uabc_tar_org_V[0] */
    put_f32(&p_out[24], ub);
    put_f32(&p_out[28], uc);

    /* ---- 30040 MotorPosSensorOutput：多圈累积位置 + 电角度 ---- */
    memset(p_pos, 0, sizeof(p_pos));
    put_i64(&p_pos[0], enc_sum);              /* enc_sum_p：2^21 脉冲/圈 */
    put_f32(&p_pos[8], POLE_PAIRS * theta_m); /* elec_angle_now_rad */

    /* ---- 30090 AppCtrlStatus：替代 ASCII 的 S 行（2026-11-21 飞控协议化）----
     * 为什么新开一条而不扩 30055：30055 已被上位机按固定偏移解析，
     * 改布局要连带改解析代码；新消息可并行存在，脚本逐个切换、随时回退。 */
    memset(p_st, 0, sizeof(p_st));
    {
        uint8_t  ffen = 0u;
        float    fa = 0.0f, ff_f = 0.0f, fv = 0.0f;
        uint8_t  flags = 0u;
        float    id_m = 0.0f, iq_m = 0.0f;
        CogFfInfo ci;

        Servo_GetFeedforward(&ffen, &fa, &ff_f, &fv);
        CogFf_GetInfo(&ci);
        Servo_GetIdqMeas(&id_m, &iq_m);

        if (BspServo_IsCurrentCalibrated() != 0U) { flags |= MAV_ST_CAL; }
        if (BspServo_IsEncoderZeroed()     != 0U) { flags |= MAV_ST_ZERO; }
        if (ServoCmd_IsArmed()             != 0U) { flags |= MAV_ST_ARMED; }
        if (ffen != 0u)                           { flags |= MAV_ST_FF; }
        if (DeadbandComp_IsEnabled()      != 0U) { flags |= MAV_ST_DEADBAND; }
        if (Servo_IsHarmonic()            != 0U) { flags |= MAV_ST_HARM; }
        if (Servo_GetKtCompEn()           != 0U) { flags |= MAV_ST_KTCOMP; }
        if (ci.valid != 0U)                       { flags |= MAV_ST_COG; }

        /* controlword 回**真实保存值**，不是只填 bit0。
         * 原来写的是 `(IsArmed())?1:0`，于是上位机设 cw=15 后读回 1，
         * 会误判成"只有使能位生效"（halt 等位看起来全被丢掉）。
         * 30068 的应答里也是用 AppParam_GetControlWord()，两处必须同源。 */
        {
            const uint16_t cw_now = AppParam_GetControlWord();
            p_st[0] = (uint8_t)(cw_now & 0xFFu);
            p_st[1] = (uint8_t)((cw_now >> 8) & 0xFFu);
        }
        p_st[2]  = (uint8_t)ServoCmd_ActiveMode();
        p_st[3]  = flags;
        put_f32(&p_st[4],  iq_m);
        put_f32(&p_st[8],  iq_ref);
        put_f32(&p_st[12], omega_m * 60.0f / 6.283185307179586f);
        put_f32(&p_st[16], theta_m);
        put_f32(&p_st[20], vbus);
        put_f32(&p_st[24], temp_motor);
        put_f32(&p_st[28], temp_gan);
        put_f32(&p_st[32], Servo_GetKtComp());
        p_st[36] = (uint8_t)(BspServo_GetFault() & 0xFFu);
        p_st[37] = (uint8_t)((BspServo_GetFault() >> 8) & 0xFFu);
        p_st[38] = (uint8_t)(s_tx_drop & 0xFFu);
        p_st[39] = (uint8_t)((s_tx_drop >> 8) & 0xFFu);
    }

    n_in  = frame_build(&s_frame[0],              MSG_CUR_IN,     seq0,                        p_in,  LEN_IN,  CRC_EXTRA_IN);
    n_out = frame_build(&s_frame[n_in],           MSG_CUR_OUT,    (uint8_t)(seq0 + 1u),        p_out, LEN_OUT, CRC_EXTRA_OUT);
    n_pos = frame_build(&s_frame[n_in + n_out],   MSG_POS_SENSOR, (uint8_t)(seq0 + 2u),        p_pos, LEN_POS, CRC_EXTRA_POS);
    /* ---- 30091 AppLoopStatus：替代 ASCII 的 K 行 ----
     * 单位统一为对外的工程单位（rpm / A / °），不再沿用内部 rad/s。 */
    memset(p_lp, 0, sizeof(p_lp));
    {
        float    w_ref = 0.0f, iq_cmd = 0.0f, i_term = 0.0f;
        float    kp = 0.0f, ki_ts = 0.0f;
        float    r_ohm = 0.0f, ld_h = 0.0f, psi_wb = 0.0f, kt = 0.0f;
        float    eoff = 0.0f, acc = 0.0f;
        uint32_t exec = 0u;

        Servo_GetSpeedState(&w_ref, &iq_cmd, &i_term, &exec, 0);
        Servo_GetSpeedGains(&kp, &ki_ts);
        /* 必须报告**生效值**而不是基础值：增益调度开启时 PI 用的是
         * Servo_GetSchedGains() 覆盖后的那一组。实测踩过：只取基础值时报
         * 583.38e-3，而 ASCII 的 K 行报 250.00e-3 —— 上位机对不上账。 */
        if (Servo_GetSchedEnable() != 0U) {
            Servo_GetSchedGains(&kp, &ki_ts);
        }
        Servo_GetIdentParams(&r_ohm, &ld_h, &psi_wb, &kt);
        eoff = Servo_GetElecOff();
        /* 报**实际生效**的加速度，而不是配置值：30097 的时间模式会把
         * a 反算并锁存（100→300 rpm/0.5 s 只有 41.9 rad/s²，与配置的 400 差 10 倍），
         * 报配置值的话上位机会按 400 去解释一条按 41.9 走的曲线。 */
        Servo_GetSpeedTrajActual(&acc, 0, 0, 0, 0);

        p_lp[0]  = (uint8_t)((Servo_IsSpeedMode() != 0U) ? 1u : 0u);
        p_lp[1]  = (uint8_t)Servo_GetSrc();
        p_lp[2]  = (uint8_t)(Servo_GetSchedEnable() & 0xFFu);
        p_lp[3]  = BspServo_GetEncDiag();   /* 编码器故障现场（原保留字节） */
        put_f32(&p_lp[4],  Servo_GetSpeedRampRef() * 60.0f / 6.283185307179586f); /* wref_rpm */
        put_f32(&p_lp[8],  omega_m * 60.0f / 6.283185307179586f);                 /* wmeas_rpm */
        put_f32(&p_lp[12], iq_cmd);          /* iqc_a */
        put_f32(&p_lp[16], i_term);          /* iterm_a */
        put_f32(&p_lp[20], kp);              /* spd_kp */
        put_f32(&p_lp[28], Servo_GetSpeedIqMax());
        put_f32(&p_lp[24], acc);             /* acc_rads2 */
        put_f32(&p_lp[32], eoff * 57.2957795131f);   /* eoff_deg */
        put_f32(&p_lp[36], psi_wb * 1000.0f);        /* psi_mWb */
        {
            /* 过流现场（2026-11-21 正式化）：kt 槽改报 ovc_peak_u、ld 槽改报 ovc_raw_u。
             * 原因：ASCII 关闭后 BspServo_GetOvcInfo() 没有任何 MAVLink 出口，
             * 过流时看不到"峰值多大 / 采样码值多少"，无法区分真过流与采样链故障。
             * 代价是 30091 暂时不报 kt/ld，标定值要从 30082 float 13 读。 */
            float pu = 0.0f, pw = 0.0f; uint16_t ru = 0U, rw = 0U;
            BspServo_GetOvcInfo(&pu, &pw, &ru, &rw);
            put_f32(&p_lp[40], pu);                  /* ovc_peak_u (A) */
            put_f32(&p_lp[44], (float)ru);           /* ovc_raw_u (cnt) */
        }
        /* put_f32(&p_lp[40], kt * 1000.0f);  kt_mNm_per_A */
        /* put_f32(&p_lp[44], ld_h * 1.0e6f); ld_uH */
    }

    n_st  = frame_build(&s_frame[n_in + n_out + n_pos], MSG_CTRL_STATUS, (uint8_t)(seq0 + 3u), p_st, LEN_ST, CRC_EXTRA_ST);
    n_lp  = frame_build(&s_frame[n_in + n_out + n_pos + n_st], MSG_LOOP_STATUS, (uint8_t)(seq0 + 4u), p_lp, LEN_LP, CRC_EXTRA_LP);

    /* ---- 30100 AppPosStatus：位置环状态（替代"没有出口"的空白）----
     * 位置环的状态机与跟随误差此前没有任何 MAVLink 出口，上位机只能靠
     * 30040 的计数变化猜"到没到位 / 是不是在追"。这里与 30090/30091 同频
     * （100 Hz）。不在位置模式时也照发，st=0(IDLE)，字段保持最后值。
     *
     * 布局（48 B，小端）：
     *   0  p_now     f32 位置环实际机械角（多圈，含编码器零点）
     *   4  p_tgt     f32 用户目标
     *   8  p_tgt_c   f32 规划目标（含补偿）
     *  12  p_ref     f32 规划器参考
     *  16  err       f32 p_ref - p_now（跟随误差）
     *  20  kp        f32 生效位置增益 1/s
     *  24  vmax      f32 rad/s
     *  28  errmax    f32 rad
     *  32  i_term    f32 rad/s
     *  36  travel    f32 本次行程 rad
     *  40  fault_cnt u32 累计跟随故障次数
     *  44  st        u8  0 IDLE / 1 MOVE / 2 HOLD / 3 DONE / 4 ERR
     *  45  settled   u8  1 = 到位
     *  46  comp_en   u8  目标补偿开关
     *  47  pos_mode  u8  1 = 当前在位置模式
     */
    memset(p_ps, 0, sizeof(p_ps));
    {
        ServoPosDiag_t pd;

        Servo_GetPosDiag(&pd);
        /* 2026-11-24 位置环已改输出端口径：所有位置/速度/误差字段都已是输出端，
         * 不再需要 ÷ACT_GEAR_RATIO。kp 无量纲；vmax/errmax/i_term 现在也是输出端口径。 */
        put_f32(&p_ps[0],  pd.p_now);
        put_f32(&p_ps[4],  pd.p_tgt);
        put_f32(&p_ps[8],  pd.p_tgt_c);
        put_f32(&p_ps[12], pd.p_ref);
        put_f32(&p_ps[16], pd.err);
        put_f32(&p_ps[20], pd.kp);
        put_f32(&p_ps[24], pd.vmax);
        put_f32(&p_ps[28], pd.errmax);
        put_f32(&p_ps[32], pd.i_term);
        put_f32(&p_ps[36], pd.travel);
        p_ps[40] = (uint8_t)(pd.fault_cnt & 0xFFu);
        p_ps[41] = (uint8_t)((pd.fault_cnt >> 8) & 0xFFu);
        p_ps[42] = (uint8_t)((pd.fault_cnt >> 16) & 0xFFu);
        p_ps[43] = (uint8_t)((pd.fault_cnt >> 24) & 0xFFu);
        p_ps[44] = pd.st;
        p_ps[45] = pd.settled;
        p_ps[46] = pd.comp_en;
        p_ps[47] = (uint8_t)((Servo_IsPosMode() != 0U) ? 1u : 0u);
    }
    n_ps  = frame_build(&s_frame[n_in + n_out + n_pos + n_st + n_lp],
                        MSG_POS_STATUS, (uint8_t)(seq0 + 5u), p_ps, LEN_PS, CRC_EXTRA_PS);

    /* ---- 30092 AppIdentResult：只在有待发结果时占位，否则本次不追加 ---- */
    n_id = 0u;
    if (s_id_left != 0u) {
        memcpy(p_id, s_id_pay, LEN_ID);
        s_id_left--;
        n_id = frame_build(&s_frame[n_in + n_out + n_pos + n_st + n_lp + n_ps],
                           MSG_IDENT_RESULT, (uint8_t)(seq0 + 6u), p_id, LEN_ID, CRC_EXTRA_ID);
    }

    if (HAL_UART_Transmit_DMA(&huart1, s_frame, (uint16_t)(n_in + n_out + n_pos + n_st + n_lp + n_ps + n_id)) == HAL_OK) {
        /*
         * 只有帧真的交给 DMA 才推进序号。
         * 旧实现在组帧参数里直接 s_seq++，一旦 HAL_UART_Transmit_DMA 返回
         * HAL_BUSY，这三条帧并没有发出去，序号却被吃掉 3 个 —— 抓包看到
         * "单次跳 156 帧"就是 52 次启动失败各吞 3 个序号造成的假丢帧。
         */
        s_seq = (uint8_t)(seq0 + 6u + ((n_id != 0u) ? 1u : 0u));
        s_tx_ok++;
    } else {
        s_tx_drop++;
        /* 留证：高 4 位 gState，低 4 位 hdmatx->State。0x0F 表示 hdmatx 为空指针 */
        s_busy_snap = (uint8_t)(((uint8_t)huart1.gState << 4) |
                                (uint8_t)((huart1.hdmatx != 0) ? (huart1.hdmatx->State & 0x0Fu) : 0x0Fu));
    }
}

uint32_t Telemetry_GetTxCount(void)     { return s_tx_ok; }
uint32_t Telemetry_GetDropCount(void)   { return s_tx_drop; }
uint8_t  Telemetry_GetBusySnap(void)    { return s_busy_snap; }
uint32_t Telemetry_GetRecoverCount(void){ return s_tx_recover; }
