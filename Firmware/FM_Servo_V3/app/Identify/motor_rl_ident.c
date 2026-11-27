/*
 * motor_rl_ident.c —— 在线辨识相电阻 R 与 dq 电感 L
 *
 * R 段：照搬 VESC mcpwm_foc_measure_resistance() 的口径（bldc-master/motor/mcpwm_foc.c:1783）
 *      虚拟角锁 0 -> 电流斜坡 -> 稳态后 R = Σ|v| / Σ|i|
 * L 段：VESC 用 HFI+FFT，X72 没有那套基础设施，改成 d 轴阶跃瞬态拟合
 *      v = R·i + L·di/dt，详见 motor_rl_ident.h 的说明。
 *
 * 全部跑在 TIM6 1 ms 上下文，20 kHz 节拍只负责执行给定与记录。
 */
#include "motor_rl_ident.h"
#include "bsp_servo.h"
#include "actuator.h"

#include <math.h>
#include <string.h>

/* ---------------- 时序与判据 ---------------- */
#define RL_R_RAMP_MS      (200U)      /* VESC 用 200 步 × 1 ms */
#define RL_R_SETTLE_MS    (100U)      /* VESC 是 50 ms；这里给足，因为 X72 的电流环带宽 1 kHz，
                                       * 且转子要真正被磁场吸住不动（ωe=0 才有 |v|=R|i|） */
#define RL_R_SAMPLE_MS    (50U)       /* VESC 采 20~200 点；50 点足够，且样品是 1 ms 间隔的稳态值 */
#define RL_L_PRE_TICKS    (40U)       /* 2 ms 给定 0，让上一段电流衰减干净 */
#define RL_TS             (50e-6f)

/* 注入参数：1 kHz、1.5 V、3 个整周期。
 *   为什么是 1 kHz：ωL = 2π·1000·113.5e-6 = 0.713 Ω，与 R ≈ 0.40 Ω 同量级，
 *   两者都能从 |Z| 里分辨出来；频率再低 ωL 就被 R 淹没，再高则 20 kHz 采样
 *   每个周期点数不够（1 kHz 正好 20 点/周期，3 周期 = 60 拍整）。
 *   为什么是 3 个整周期：单点 DFT 只有在整周期上才正交，非整周期会有泄漏。 */
#define RL_INJ_FREQ       (1000.0f)
#define RL_INJ_AMP_DFLT   (1.0f)   /* 约 1.56 A 电流幅值：既不饱和到失真，又足够压过死区 */
#define RL_INJ_CYC        (3U)
#define RL_INJ_N          (60U)       /* 3 周期 × 20 拍 */
#define RL_L_MIN          (5.0e-6f)   /* 5 uH */
#define RL_L_MAX          (3.0e-3f)   /* 3 mH */

#define RL_ID_MIN_A       (0.5f)
#define RL_ID_MAX_A       (4.0f)      /* 空载 4 A 已是 Kt=0.0936 时 0.37 N·m，够用且安全 */
#define RL_I_MIN_AVG      (0.2f)      /* 平均电流低于此说明根本没建立，直接判失败 */
#define RL_TIMEOUT_MS     (5000U)

/* ---------------- 状态 ---------------- */
static volatile RlIdentState s_state = RL_IDENT_IDLE;
static RlIdentResult s_res;
static float    s_amp;
static float    s_inj_amp = RL_INJ_AMP_DFLT;
static uint16_t s_t;
static uint32_t s_tmo;

static float    s_vsum;
static float    s_isum;
static uint16_t s_nsamp;

typedef struct {
    float id;
    float ph;
} rl_pt_t;

static rl_pt_t   s_tr[RL_INJ_N];
static uint16_t  s_tr_n;

const RlIdentResult *RlIdent_GetResult(void) { return &s_res; }
RlIdentState RlIdent_GetState(void) { return s_state; }

uint8_t RlIdent_IsBusy(void)
{
    return ((s_state != RL_IDENT_IDLE) &&
            (s_state != RL_IDENT_DONE) &&
            (s_state != RL_IDENT_FAIL)) ? 1U : 0U;
}

const char *RlIdent_ErrStr(RlIdentErr e)
{
    switch (e) {
    case RL_IDENT_OK:            return "ok";
    case RL_IDENT_ERR_BUSY:      return "busy";
    case RL_IDENT_ERR_CAL:       return "need C";
    case RL_IDENT_ERR_ZERO:      return "need Z";
    case RL_IDENT_ERR_FAULT:     return "fault";
    case RL_IDENT_ERR_PWM:       return "moe";
    case RL_IDENT_ERR_CURRENT:   return "no current";
    case RL_IDENT_ERR_FIT:       return "fit bad";
    case RL_IDENT_ERR_TIMEOUT:   return "timeout";
    default:                     return "?";
    }
}

static void rl_stop(void)
{
    Servo_SetVoltInject(0U, 0.0f, 0.0f);
    Servo_SetOpenLoop(0U, 0.0f, 0.0f);
    Servo_SetCurrentMode(0U, 0.0f);
    BspServo_PwmEnable(0U);
}

static void rl_fail(RlIdentErr e)
{
    rl_stop();
    s_res.err = e;
    s_state   = RL_IDENT_FAIL;
}

void RlIdent_Abort(void)
{
    if (RlIdent_IsBusy() != 0U) {
        rl_fail(RL_IDENT_ERR_TIMEOUT);
    }
}

void RlIdent_SetCached(float r_ohm, float l_h)
{
    if (RlIdent_IsBusy() != 0U) { return; }
    if ((r_ohm <= 0.0f) || (l_h <= 0.0f)) { return; }
    s_res.r_ohm = r_ohm;
    s_res.l_h   = l_h;
    s_res.err   = RL_IDENT_OK;
}

void RlIdent_SetInjAmp(float amp_v)
{
    if (amp_v < 0.2f) { amp_v = 0.2f; }
    if (amp_v > 3.0f) { amp_v = 3.0f; }
    s_inj_amp = amp_v;
}

float RlIdent_GetInjAmp(void) { return s_inj_amp; }

void RlIdent_Init(void)
{
    memset(&s_res, 0, sizeof(s_res));
    s_state = RL_IDENT_IDLE;
}

/*
 * L 拟合。
 *   对每个相邻点对求 di/dt，只在电流确实在动（|di/dt| > RL_DI_MIN）时取值，
 *   L_k = (vd_k − R·i_k) / (di/dt)_k
 *
 * 为什么用平均 vd（当前点）而不是两点平均：vd 是 PI 在这一拍的输出，
 * 与"这一拍的 di/dt"配对才是同一时刻的量；错开一拍在 1 kHz 带宽下就有 17% 误差。
 */
static uint8_t rl_calc_l(void)
{
    const float r = s_res.r_ohm;
    const float w = BSP_TWO_PI * RL_INJ_FREQ;
    float sc = 0.0f, ss = 0.0f;
    uint16_t i;

    /* 单点 DFT（正交解调）：整周期上 cos/sin 与信号正交，
     * 所以只要累加 Σi·cos 与 Σi·sin 就能取到基波幅度，不需要 FFT。 */
    for (i = 0U; i < s_tr_n; i++) {
        sc += s_tr[i].id * cosf(s_tr[i].ph);
        ss += s_tr[i].id * sinf(s_tr[i].ph);
    }

    {
        const float i_amp = (2.0f / (float)s_tr_n) * sqrtf((sc * sc) + (ss * ss));

        if (i_amp < 0.05f) {                /* 电流没响应：接线/限流/母线问题 */
            rl_fail(RL_IDENT_ERR_CURRENT);
            return 0U;
        }

        {
            const float z  = s_inj_amp / i_amp;         /* |Z| = V/I */
            float x2 = (z * z) - (r * r);               /* (ωL)² = |Z|² − R² */

            if (x2 <= 0.0f) {
                rl_fail(RL_IDENT_ERR_FIT);              /* R 测大了或 I 测小了 */
                return 0U;
            }

            {
                const float l = sqrtf(x2) / w;

                if ((l < RL_L_MIN) || (l > RL_L_MAX)) {
                    rl_fail(RL_IDENT_ERR_FIT);
                    return 0U;
                }

                s_res.l_h      = l;
                s_res.l_points = s_tr_n;
                s_res.i_amp    = i_amp;
                s_res.z_ohm    = z;
                s_res.inj_amp  = s_inj_amp;
            }
        }
    }

    rl_stop();
    s_res.err = RL_IDENT_OK;
    s_state   = RL_IDENT_DONE;
    return 1U;
}

/* ---------------- ISR 侧 ---------------- */

void RlIdent_PushInject(float id_now, float phase)
{
    if (s_state != RL_IDENT_L_INJ) { return; }
    if (s_tr_n < RL_INJ_N) {
        s_tr[s_tr_n].id = id_now;
        s_tr[s_tr_n].ph = phase;
        s_tr_n++;
    }
}

/* ---------------- 1 ms 状态机 ---------------- */

RlIdentErr RlIdent_Start(float test_current_a)
{
    float a = test_current_a;

    if (RlIdent_IsBusy() != 0U)                { return RL_IDENT_ERR_BUSY; }
    if (BspServo_IsPwmEnabled() != 0U)         { return RL_IDENT_ERR_BUSY; }
    if (BspServo_IsCurrentCalibrated() == 0U)  { return RL_IDENT_ERR_CAL; }
    if (BspServo_IsEncoderZeroed() == 0U)      { return RL_IDENT_ERR_ZERO; }
    if (BspServo_GetFault() != 0U)             { return RL_IDENT_ERR_FAULT; }
    if (Act_Acquire(ACT_RL_IDENT, RlIdent_IsBusy) == 0U) { return RL_IDENT_ERR_BUSY; }

    if (a < RL_ID_MIN_A) { a = RL_ID_MIN_A; }
    if (a > RL_ID_MAX_A) { a = RL_ID_MAX_A; }

    memset(&s_res, 0, sizeof(s_res));
    s_amp    = a;
    s_t      = 0U;
    s_tmo    = 0U;
    s_vsum  = 0.0f;
    s_isum  = 0.0f;
    s_nsamp = 0U;
    s_tr_n  = 0U;

    /* 虚拟角 0 + id 从 0 起步：转子被磁场吸住，ωe=0 才有 |v| = R|i| */
    Servo_SetOpenLoop(1U, 0.0f, 0.0f);
    BspServo_PwmEnable(1U);

    if (BspServo_IsPwmEnabled() == 0U) {
        rl_fail(RL_IDENT_ERR_PWM);
        return RL_IDENT_ERR_PWM;
    }

    s_state = RL_IDENT_R_RAMP;
    return RL_IDENT_OK;
}

uint8_t RlIdent_Task1ms(void)
{
    if (RlIdent_IsBusy() == 0U) { return 0U; }

    if (++s_tmo > RL_TIMEOUT_MS) {
        rl_fail(RL_IDENT_ERR_TIMEOUT);
        return 1U;
    }
    if (BspServo_GetFault() != 0U) {
        rl_fail(RL_IDENT_ERR_FAULT);
        return 1U;
    }

    s_t++;

    switch (s_state) {
    case RL_IDENT_R_RAMP: {
        float k = (float)s_t / (float)RL_R_RAMP_MS;
        if (k > 1.0f) { k = 1.0f; }
        Servo_SetOpenLoop(1U, 0.0f, s_amp * k);
        if (s_t >= RL_R_RAMP_MS) {
            s_t = 0U;
            s_state = RL_IDENT_R_SETTLE;
        }
        return 1U;
    }

    case RL_IDENT_R_SETTLE:
        Servo_SetOpenLoop(1U, 0.0f, s_amp);
        if (s_t >= RL_R_SETTLE_MS) {
            s_t     = 0U;
            s_vsum  = 0.0f;
            s_isum  = 0.0f;
            s_nsamp = 0U;
            s_state = RL_IDENT_R_SAMPLE;
        }
        return 1U;

    case RL_IDENT_R_SAMPLE: {
        float vd = 0.0f, vq = 0.0f, idm = 0.0f, iqm = 0.0f;

        Servo_GetOpenLoopVdq(&vd, &vq);
        Servo_GetIdqMeas(&idm, &iqm);

        s_vsum += sqrtf((vd * vd) + (vq * vq));
        s_isum += sqrtf((idm * idm) + (iqm * iqm));
        s_nsamp++;

        if (s_t >= RL_R_SAMPLE_MS) {
            if ((s_nsamp == 0U) || (s_isum < (RL_I_MIN_AVG * (float)s_nsamp))) {
                rl_fail(RL_IDENT_ERR_CURRENT);
                return 1U;
            }

            s_res.r_ohm = s_vsum / s_isum;
            s_res.v_avg = s_vsum / (float)s_nsamp;
            s_res.i_avg = s_isum / (float)s_nsamp;

            /* 切到 L 段：退出电流环与虚拟角环，改用单频电压注入。
             * 必须先关电流环 —— PI 会把外部注入的正弦当成扰动压掉，
             * 那样测出来的不是电机的 |Z|，而是控制器的抑制比。 */
            Servo_SetOpenLoop(0U, 0.0f, 0.0f);
            Servo_SetCurrentMode(0U, 0.0f);
            Servo_SetVoltInject(1U, s_inj_amp, RL_INJ_FREQ);

            s_tr_n  = 0U;
            s_t     = 0U;
            s_state = RL_IDENT_L_INJ;
        }
        return 1U;
    }

    case RL_IDENT_L_INJ:
        /* ISR 每拍 PushInject 记录；60 拍 = 3 个整周期 */
        if (s_tr_n >= RL_INJ_N) {
            Servo_SetVoltInject(0U, 0.0f, 0.0f);
            (void)rl_calc_l();
            return 1U;
        }
        return 0U;

    default:
        return 0U;
    }
}
