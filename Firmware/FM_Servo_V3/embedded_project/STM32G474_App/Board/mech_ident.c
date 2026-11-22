/**
  ******************************************************************************
  * @file    mech_ident.c
  * @brief   机械环节辨识（谐振扫频 / 摩擦曲线 / 折算惯量 / 齿槽转矩）
  ******************************************************************************
  * 四种模式共用同一套"进环 → 激励 → 同步解调 → 退环"框架，只换激励与算法。
  *
  * 为什么用**同步解调**而不是录波形再 FFT：
  *   激励频率是我们自己产生的，所以基波的 cos/sin 参考是已知的，
  *   单点 DFT 就够（Σv·cos、Σv·sin），不用存 1024 点、不用传、不用上位机算 FFT。
  *   代价是每拍多一次 cosf + 两次乘加，约 60 cycle。
  *
  * 采样延迟补偿：iq 给定在 N 拍生效，速度在 N+1 拍测到，差 1 拍（50 us）。
  *   1000 Hz 时这是 18 度，必须补 —— 输出相位 = atan2(vs,vc) − 2π·f·Ts。
  ******************************************************************************
  */

#include "mech_ident.h"
#include "bsp_servo.h"
#include "actuator.h"

#include <math.h>

/* ---------------- 参数 ---------------- */
#define MI2_TS            (0.00005f)      /* 20 kHz */
#define MI2_RAD_PER_RPM   (0.10471975512f)

/* 激励幅值与频率范围：纯交流（无直流分量），平均转矩为 0，不会一直加速。
 * 下限 20 Hz：再低速度摆幅 = A·Kt/(J·2πf) 会到几十 rpm，空载台架不安全。 */
/* 0.20 A：20 Hz 时速度摆幅 ≈ Kt·i/(J·2πf)，J=2.3e-4 时约 66 rpm —— 已经不小了。
 * 第一次用 0.40 A 时摆到 130 rpm，摩擦与齿隙都进入非线性区，
 * 低频增益会虚高（实测 20 Hz 增益 17.4 rad/s per A，比理论高 5 倍）。 */
#define MI2_SWEEP_AMP     (0.20f)         /* A，低频段 */
/* 高频段用更大的幅值：速度响应 ∝ 1/f，1 kHz 时 0.2 A 只产生 0.026 rad/s，
 * 而测量噪声（8 拍差分后）约 0.014 rad/s，信噪比不到 2。
 * 0.5 A 给 0.13 rad/s，信噪比 ~9。高频时转子只微振（1.3 rpm），
 * 不会进摩擦非线性区，所以放大幅值是安全的。 */
#define MI2_SWEEP_AMP_HI  (0.50f)         /* A，f > MI2_SWEEP_F_SPLIT */
#define MI2_SWEEP_F_SPLIT (300.0f)        /* Hz，幅值与累加时长的分界 */

/* ---- 中心差分（替代单拍差分）----
 * 单拍差分 ω = Δθ/Ts 的量化噪声是 ±1 count/Ts = ±0.06 rad/s，
 * 而 1 kHz 的机械响应只有 0.03 rad/s 量级 —— 完全被噪声盖住，
 * 这就是之前 ">400 Hz 测不到" 的真正原因（不是没有响应，是测不出来）。
 *
 * 改成 8 拍差分（中心在第 4 拍）：
 *     ω(t−4) = (θ(t) − θ(t−8)) / (8·Ts)
 * 噪声降到 ±2 count/(8·Ts) = ±0.015 rad/s（好 4 倍），
 * 代价是 4 拍延迟（1 kHz 时 72°，按频率精确补偿）与差分器本身的 sinc 低通
 * （1 kHz 处衰减到 0.755，按频率补偿回来）。
 * 为什么不用 16 拍：sinc 衰减到 0.235，补偿要乘 4.3 倍，噪声反而放大。 */
#define MI2_DIFF_N        (8U)
#define MI2_TH_N          (16U)
#define MI2_SWEEP_F_LO    (20.0f)
#define MI2_SWEEP_HI      (1000.0f)
#define MI2_SETTLE_MS     (40U)           /* 换频点后等 2 个周期 */

/* 摩擦：转速点覆盖静摩擦区到黏滞区，低速密采样 */
static const float MI2_FRIC_RPM[MI2_FRIC_PTS] =
    { 1.0f, 2.0f, 3.0f, 6.0f, 12.0f, 30.0f, 80.0f, 200.0f };
/* 1 rpm 时速度环的积分要爬很久才稳（实测 1.2 s 时 3 rpm 读到 0.499 A，
 * 而 12 rpm 只读到 0.070 A —— 明显还在爬）。加到 2 s 稳定 + 500 ms 平均。 */
#define MI2_FRIC_SETTLE_MS (3000U)
#define MI2_FRIC_AVG_N     (500U)         /* 500 ms */
/* 回归用的最小角速度（rad/s）：1.0 ≈ 9.5 rpm，即排除 1/2/3/6 rpm 的静摩擦点。
 * 见回归处的说明：那几个点的 iq 是动摩擦的 2 倍量级，会把斜率压成负值。 */
#define MI2_FRIC_WMIN_RADS (1.0f)

/* 惯量：两个转矩幅值，测跨过 [w1,w2] 的时间 */
#define MI2_INERTIA_IQ1   (1.0f)          /* A */
#define MI2_INERTIA_IQ2   (2.0f)          /* A */
#define MI2_INERTIA_W1    (1.0f)          /* rad/s，避开静摩擦区 */
#define MI2_INERTIA_W2    (6.0f)
#define MI2_INERTIA_REST  (600U)          /* ms，等完全停住 */
#define MI2_INERTIA_MAX   (2000U)         /* ms，超时判失败 */

/* 齿槽：低速匀速，转 1 圈 */
#define MI2_COG_RPM       (10.0f)   /* 2 rpm 转 1 圈要 30 s，超过超时；10 rpm 是 6 s */
#define MI2_COG_SETTLE_MS (800U)
#define MI2_COG_SCALE     (1000.0f)       /* iq → int16，±32 A 不溢出 */

/* 用实测 psi（4.455 mWb）而不是模型值（7.593）：后者偏大 41%，
 * 会把 J 算小 41%。psi 本身的不确定性会等比例传到 J 上，无法回避。 */
/* Kt = 1.5·p·ψ。ψ 现在有 + 命令直接测（4.333 mWb，两个转速一致到 1.85%），
 * 与之前"3000 rpm 饱和点反推"的 4.455 mWb 相差 2.7% —— 两条独立方法互证。
 * 取 4.333 这组：J = 5.32e-5 时 Kt 从 0.0936 变 0.0910，
 * 折算 J 从 5.32e-5 变 5.47e-5（差 2.8%，对前馈系数影响可忽略）。
 *
 * ⚠ 2026-11-20：这个常量**只作 fallback**。现在 + 辨识会把 psi 写进控制律
 * （`Servo_SetIdentPsiEx`），Kt 随之变成 1.5·p·ψ_id（今天 = 0.0883），
 * 与这里的 0.0910 差 3%。若仍用常量，J 会整体偏 3% ——
 * 而 J 要喂给加速度前馈，偏 3% 就是前馈偏 3%。故运行时改读 `Servo_GetKtNow()`。
 * 这个宏保留：Kt 未辨识（上电默认 4.333）时两者数值相同，可作对照基准。 */
#define MI2_KT            (0.0910f)       /* N·m/A = 1.5 * 14 * 4.333e-3（fallback） */

/* 本次辨识实际用的 Kt（N·m/A）。在加速段开始时锁存一次 ——
 * 辨识期间电机会升温，GetKtNow() 里的 ktc 会跟着变，
 * 两段加速若各取一次，ΔT 就要带上 Kt 的差，凭空引入误差。 */
static float s_kt_used = MI2_KT;

static MechIdentState  s_state = MI2_IDLE;
static MechIdentResult s_res;
static uint8_t         s_mode;
static uint16_t        s_t;

/* 激励与解调 */
static volatile uint8_t  s_inj_en;
static volatile float    s_inj_amp;
static volatile float    s_inj_dph;
static volatile uint8_t  s_inj_ac = 1U;   /* 1=正弦（扫频） 0=直流（惯量） */
static float             s_inj_ph;
static float             s_dm_c, s_dm_s;
static volatile float    s_dm_vc, s_dm_vs;
static volatile float    s_dm_ic, s_dm_is;      /* iq 的同步解调（分母用实测而非给定） */
static volatile float    s_dm_wabs;             /* Σ|w|，诊断差分通道是否有效 */
static float             s_th_hist[MI2_TH_N];   /* 机械角历史，做 8 拍差分 */
static uint8_t           s_th_i;
static volatile uint32_t s_dm_n;
static volatile uint8_t  s_dm_on;

/* 模式 1 */
static uint8_t  s_pt;
static uint32_t s_need;

/* 模式 2 */
static float    s_sum_iq, s_sum_rpm;
static uint16_t s_navg;

/* 模式 3 */
static float    s_wprev;
static uint32_t s_tick;
static uint32_t s_t1, s_t2;
static uint8_t  s_leg;          /* 0=等静止 1=第一段 2=第二段 */
static uint8_t  s_got1, s_got2;

/* 模式 4 */
/* 运行时箱数（128/256/512），默认见 MI2_COG_BINS_DFLT。
 * 数组按 MAX 分配、按 s_cog_bins 使用 —— 换箱数不需要重新编译。 */
static uint16_t s_cog_bins = MI2_COG_BINS_DFLT;
static float    s_cog_acc[MI2_COG_BINS_MAX];  /* 正向累加 */
static uint16_t s_cog_cnt[MI2_COG_BINS_MAX];
static float    s_cog_rev[MI2_COG_BINS_MAX];  /* 反向累加（换掉摩擦常值偏置） */
static uint16_t s_cog_rcnt[MI2_COG_BINS_MAX];

void MechIdent_SetCogBins(uint16_t n)
{
    if ((n != 128U) && (n != 256U) && (n != 512U)) { return; }
    s_cog_bins = n;
}
uint16_t MechIdent_GetCogBins(void) { return s_cog_bins; }
static uint8_t  s_cog_leg;                    /* 0 = 正向圈，1 = 反向圈 */
static float    s_cog_th0;
static uint8_t  s_cog_run;

static void mi2_stop_excite(void)
{
    s_inj_en = 0U;
    s_dm_on  = 0U;
}

static void mi2_fail(MechIdentErr e)
{
    mi2_stop_excite();
    Servo_SetSpeedMode(0U, 0.0f);
    BspServo_PwmEnable(0U);
    s_res.err = (uint8_t)e;
    s_state   = MI2_FAIL;
}

static void mi2_set_freq(float f)
{
    s_inj_dph = BSP_TWO_PI * f * MI2_TS;
    s_inj_ph  = 0.0f;
    /* 整周期累加：非整周期会有泄漏，低频时尤其明显（20 Hz 只有 4~5 个周期）。
     * 高频段把窗口从 0.20 s 加到 0.40 s：噪声按 1/√N 收敛，
     * 多一倍时间换 1.41 倍信噪比，而 1 kHz 处响应本来就只有 0.03 rad/s 量级。 */
    {
        const float win = (f > MI2_SWEEP_F_SPLIT) ? 0.40f : 0.20f;
        float np = win * f;
        if (np < 5.0f) { np = 5.0f; }
        s_need = (uint32_t)(np * (1.0f / (f * MI2_TS)) + 0.5f);
    }
}

static void mi2_begin_meas(void)
{
    s_dm_vc = 0.0f;
    s_dm_vs = 0.0f;
    s_dm_ic = 0.0f;
    s_dm_is = 0.0f;
    s_dm_wabs = 0.0f;

    /* 高频段加大激励幅值（速度响应 ∝ 1/f，不加大就淹没在噪声里） */
    {
        const float f = s_inj_dph / (BSP_TWO_PI * MI2_TS);
        s_inj_amp = (f > MI2_SWEEP_F_SPLIT) ? MI2_SWEEP_AMP_HI : MI2_SWEEP_AMP;
    }
    s_dm_n  = 0U;
    s_dm_on = 1U;
    s_t     = 0U;
    s_state = MI2_MEAS;
}

void MechIdent_Init(void)
{
    s_state = MI2_IDLE;
    s_inj_en = 0U;
    s_dm_on  = 0U;
}

MechIdentState MechIdent_GetState(void)          { return s_state; }

uint8_t MechIdent_IsBusy(void)
{
    return ((s_state != MI2_IDLE) && (s_state != MI2_DONE) &&
            (s_state != MI2_FAIL)) ? 1U : 0U;
}
const MechIdentResult *MechIdent_GetResult(void) { return &s_res; }

const char *MechIdent_ErrStr(MechIdentErr e)
{
    switch (e) {
    case MI2_OK:          return "ok";
    case MI2_ERR_BUSY:    return "busy (send D first)";
    case MI2_ERR_CAL:     return "current not calibrated (C)";
    case MI2_ERR_ZERO:    return "elec angle not aligned (Z)";
    case MI2_ERR_FAULT:   return "fault present (X)";
    case MI2_ERR_MODE:    return "bad mode (use _1.._4)";
    case MI2_ERR_MOTION:  return "motor did not follow";
    case MI2_ERR_DATA:    return "data not usable (too small / unstable)";
    default:              return "?";
    }
}

MechIdentErr MechIdent_Start(uint8_t mode)
{
    if ((s_state != MI2_IDLE) && (s_state != MI2_DONE) && (s_state != MI2_FAIL)) {
        return MI2_ERR_BUSY;
    }
    if (BspServo_IsPwmEnabled() != 0U)        { return MI2_ERR_BUSY; }
    if (BspServo_IsCurrentCalibrated() == 0U) { return MI2_ERR_CAL; }
    if (BspServo_IsEncoderZeroed() == 0U)     { return MI2_ERR_ZERO; }
    if (BspServo_GetFault() != 0U)            { return MI2_ERR_FAULT; }
    if ((mode < 1U) || (mode > 4U))           { return MI2_ERR_MODE; }
    if (Act_Acquire(ACT_MECH_IDENT, MechIdent_IsBusy) == 0U) { return MI2_ERR_BUSY; }

    mi2_stop_excite();
    s_res.mode  = mode;
    s_res.err   = MI2_OK;
    s_mode      = mode;
    s_t         = 0U;
    s_pt        = 0U;
    s_navg      = 0U;
    s_sum_iq    = 0.0f;
    s_sum_rpm   = 0.0f;
    s_leg       = 0U;
    s_got1      = 0U;
    s_got2      = 0U;
    s_cog_run   = 0U;
    s_cog_leg   = 0U;
    s_th_i      = 0U;
    {
        uint16_t i;          /* 必须是 16 位：箱数 256/512 时 uint8 的 i<512 恒成立 → 死循环 */
        for (i = 0U; i < s_cog_bins; i++) {
            s_cog_acc[i] = 0.0f; s_cog_cnt[i] = 0U;
            s_cog_rev[i] = 0.0f; s_cog_rcnt[i] = 0U;
        }
    }

    Servo_SetEnable(0U);
    Servo_SetMode(SERVO_MODE_SPEED);

    if ((mode == 1U) || (mode == 3U)) {
        /* 电流环模式：iq 自己给，不用速度环 */
        Servo_SetCurrentMode(1U, 0.0f);
        BspServo_PwmEnable(1U);
        if (mode == 1U) {
            s_inj_amp = MI2_SWEEP_AMP;
            s_inj_ac  = 1U;
            mi2_set_freq(MI2_SWEEP_F_LO);
            s_inj_en = 1U;
        }
        s_res.sweep_n = 0U;
        s_res.fric_n  = 0U;
    } else {
        /* 速度环模式：摩擦与齿槽都要稳态转速 */
        if (mode == 2U) {
            Servo_SetSpeedMode(1U, MI2_FRIC_RPM[0] * MI2_RAD_PER_RPM);
        } else {
            s_cog_leg = 0U;          /* 先跑正向圈 */
            Servo_SetSpeedMode(1U, MI2_COG_RPM * MI2_RAD_PER_RPM);
        }
        BspServo_PwmEnable(1U);
    }

    if (BspServo_IsPwmEnabled() == 0U) {
        s_res.err = MI2_ERR_FAULT;
        s_state   = MI2_FAIL;
        return MI2_ERR_FAULT;
    }

    s_state = MI2_SETTLE;
    return MI2_OK;
}

void MechIdent_Abort(void)
{
    mi2_stop_excite();
    if ((s_state == MI2_IDLE) || (s_state == MI2_DONE) || (s_state == MI2_FAIL)) {
        s_state = MI2_IDLE;
        return;
    }
    Servo_SetSpeedMode(0U, 0.0f);
    BspServo_PwmEnable(0U);
    s_state = MI2_IDLE;
}

/* ============================ ISR 侧 ============================ */

uint8_t MechIdent_Refs(float *iq_ref)
{
    if (s_inj_en == 0U) { return 0U; }

    if (s_inj_ac == 0U) {
        /* 直流激励（惯量测试）：给定恒定。
         * ⚠ 这里不能走正弦分支 —— dph=0 时 sin(0)=0，给定会恒为 0，
         * 现象是"惯量测试跑完但电机根本没动"（第一次实现就踩了这个）。 */
        s_dm_s = 0.0f;
        s_dm_c = 0.0f;
        if (iq_ref) { *iq_ref = s_inj_amp; }
        return 1U;
    }

    s_inj_ph += s_inj_dph;
    if (s_inj_ph >= BSP_TWO_PI) { s_inj_ph -= BSP_TWO_PI; }

    /* sin/cos 都算，sin 给给定，cos+sin 给解调（省一次三角） */
    s_dm_s = sinf(s_inj_ph);
    s_dm_c = cosf(s_inj_ph);

    if (iq_ref) { *iq_ref = s_inj_amp * s_dm_s; }
    return 1U;
}

void MechIdent_Step(float omega_fast, float iq_now, float theta_multi)
{
    /* ---- 机械角历史（8 拍差分用）----
     * 每个控制拍都推进，与是否在解调无关，这样差分窗口始终是连续的。 */
    s_th_hist[s_th_i] = theta_multi;
    s_th_i = (uint8_t)((s_th_i + 1U) % MI2_TH_N);

    /* ---- 同步解调（模式 1）----
     * 用 8 拍差分而不是传入的单拍 omega_fast：见 MI2_DIFF_N 的注释。
     * 差分后的速度对应 4 拍之前的时刻，相位在计算时补偿。 */
    if (s_dm_on != 0U) {
        const uint8_t i8 = (uint8_t)((s_th_i + MI2_TH_N - MI2_DIFF_N) % MI2_TH_N);
        const float   w  = (theta_multi - s_th_hist[i8]) / ((float)MI2_DIFF_N * MI2_TS);

        s_dm_vc += w * s_dm_c;
        s_dm_vs += w * s_dm_s;
        s_dm_wabs += fabsf(w);
        /* 同时解调 iq：高频时电流环本身有衰减与滞后，用"给定幅值"做分母
         * 会把电流环的频响算进机械频响里。用实测 iq 就把这一项约掉了。 */
        s_dm_ic += iq_now * s_dm_c;
        s_dm_is += iq_now * s_dm_s;
        s_dm_n++;
    }
    (void)omega_fast;   /* 保留参数：惯量/齿槽仍可用它，扫频已改用差分 */

    /* ---- 惯量：跨阈值计时（模式 3）---- */
    if ((s_mode == 3U) && (s_leg != 0U)) {
        const float a = fabsf(omega_fast);
        s_tick++;

        if ((s_t1 == 0U) && (a >= MI2_INERTIA_W1) && (s_wprev < MI2_INERTIA_W1)) {
            s_t1 = s_tick;
        }
        if ((s_t2 == 0U) && (a >= MI2_INERTIA_W2) && (s_wprev < MI2_INERTIA_W2)) {
            s_t2 = s_tick;
        }
        s_wprev = a;
    }

    /* ---- 齿槽：按机械角分箱（模式 4，正反各一圈）----
     * 反向圈的 rel 用 (θ0 + 2π) − θ：这样两向都映射到同一个角度区间
     * [θ0, θ0+2π]，bin 索引严格对应。若反向用 (θ0 − θ)，
     * 起止点会差一个过冲量（约 0.1 rad ≈ 2 个箱），两向平均就错位了。 */
    if ((s_mode == 4U) && (s_cog_run != 0U)) {
        /* ⚠ 分箱用的角度与"转完一圈"的进度必须是两个量：
         *   分箱：br = θ − θ0（θ0 恒为正向下标那一点，两向都用它，
         *         这样同一个物理位置永远落同一个箱）
         *   进度：正向 = br，反向 = 2π − br
         * 一开始把两者写成一个式子，反向时 rel 起始就是 2π、
         * 立刻满足"转完一圈"，结果反向圈一个点都没采到（实测 rev 与 fwd 完全相同）。 */
        const float br = theta_multi - s_cog_th0;

        if ((br >= 0.0f) && (br < BSP_TWO_PI)) {
            uint32_t b = (uint32_t)(br * (1.0f / BSP_TWO_PI) * (float)s_cog_bins);
            if (b >= s_cog_bins) { b = s_cog_bins - 1U; }
            if (s_cog_leg == 0U) {
                s_cog_acc[b] += iq_now;
                s_cog_cnt[b]++;
            } else {
                s_cog_rev[b] += iq_now;
                s_cog_rcnt[b]++;
            }
        }
    }
}

/* ============================ 1ms 状态机 ============================ */

void MechIdent_Task1ms(void)
{
    float iq = 0.0f, wr = 0.0f, itm = 0.0f, ramp = 0.0f;
    uint32_t ex = 0U;

    if ((s_state == MI2_IDLE) || (s_state == MI2_DONE) || (s_state == MI2_FAIL)) {
        return;
    }
    if (BspServo_GetFault() != 0U) { mi2_fail(MI2_ERR_FAULT); return; }

    s_t++;

    /* ================= 模式 1：扫频 ================= */
    if (s_mode == 1U) {
        if (s_state == MI2_SETTLE) {
            if (s_t >= MI2_SETTLE_MS) { mi2_begin_meas(); }
            return;
        }
        if (s_state == MI2_MEAS) {
            if (s_dm_n < s_need) { return; }

            s_dm_on = 0U;
            {
                const float n = (float)s_dm_n;
                const float f = s_inj_dph / (BSP_TWO_PI * MI2_TS);
                float amp_v  = 2.0f * sqrtf((s_dm_vc * s_dm_vc) + (s_dm_vs * s_dm_vs)) / n;
                const float iq_amp = 2.0f * sqrtf((s_dm_ic * s_dm_ic) + (s_dm_is * s_dm_is)) / n;

                /* ---- 差分器的 sinc 低通补偿 ----
                 * N 拍差分 = 一个平均滤波器 + 微分，频响 |sin(ωT/2)/(ωT/2)|，
                 * T = N·Ts。1 kHz、N=8 时衰减到 0.755，不补会系统性低估高频增益。
                 * 低频时 x→0，sinc→1，补偿量自然趋于 1，不用分频段。 */
                {
                    const float x = 3.14159265f * f * ((float)MI2_DIFF_N * MI2_TS);
                    const float sinc = (x > 1.0e-4f) ? (sinf(x) / x) : 1.0f;
                    if (sinc > 0.05f) { amp_v /= sinc; }
                }

                /* ---- 相位：速度相对激励 ----
                 * ⚠ atan2 的参数顺序：设 v = A·sin(ωt+φ)，则
                 *     Σv·sin = A·N/2·cosφ   Σv·cos = A·N/2·sinφ
                 *   所以 φ = atan2(Σv·cos, Σv·sin)。
                 * 写成 atan2(vs, vc) 会得到 90°−φ，实测表现为低频相位 +156°
                 * 而不是 -90°（惯性环节的积分特性）。
                 *
                 * 延迟补偿方向：N 拍差分代表的是 (t − N/2·Ts) 时刻的速度，
                 * 即 v_meas(t) = v_true(t − N/2·Ts) = A·sin(ωt + φ − ω·N/2·Ts)。
                 * 用当前拍相位解调得到的是 φ − ω·N/2·Ts，所以**真实相位要加回来**。
                 * （原实现用单拍差分、减 1 拍；延迟小且方向相反，两者都只差十几度，
                 *   所以一直没暴露。现在 8 拍差分在 1 kHz 是 72°，方向错就是 144°。） */
                {
                    float ph = atan2f(s_dm_vc, s_dm_vs) * (180.0f / 3.14159265f);
                    ph += 360.0f * f * (0.5f * (float)MI2_DIFF_N * MI2_TS);
                    while (ph >  180.0f) { ph -= 360.0f; }
                    while (ph < -180.0f) { ph += 360.0f; }

                    /* 增益乘 dir：dir=-1 时 iq 正方向与编码器正方向相反，
                     * 不归一化会让 Bode 图整体翻 180 度，看不出是谐振还是反相。 */
                    s_res.sweep[s_pt].freq  = f;
                    /* 分母用**实测 iq 幅值**：高频时电流环本身有衰减与滞后，
                     * 用给定幅值做分母会把电流环的频响算进机械频响里。
                     * iq 幅值过小（<20 mA）说明电流环根本没跟上，退回给定值。 */
                    s_res.sweep[s_pt].gain  =
                        (amp_v / ((iq_amp > 0.02f) ? iq_amp : s_inj_amp))
                        * (float)Servo_GetDirSign();
                    s_res.sweep[s_pt].phase = ph;
                    s_res.sweep_n = (uint8_t)(s_pt + 1U);

                    if (s_pt == 0U) {
                        s_res.dbg_wmean = s_dm_wabs / n;
                        s_res.dbg_wamp  = amp_v;
                        s_res.dbg_iamp  = iq_amp;
                    }
                }
            }

            s_pt++;
            if (s_pt >= MI2_SWEEP_PTS) {
                mi2_stop_excite();
                Servo_SetCurrentMode(0U, 0.0f);
                BspServo_PwmEnable(0U);
                s_state = MI2_DONE;
                return;
            }
            /* 下一频点：频率按对数均匀铺开 */
            {
                const float r = MI2_SWEEP_HI / MI2_SWEEP_F_LO;
                const float g = powf(r, 1.0f / (float)(MI2_SWEEP_PTS - 1U));
                float f = MI2_SWEEP_F_LO * powf(g, (float)s_pt);
                mi2_set_freq(f);
            }
            s_state = MI2_SETTLE;
            s_t     = 0U;
            return;
        }
        return;
    }

    /* ================= 模式 2：摩擦曲线 ================= */
    if (s_mode == 2U) {
        if (s_state == MI2_SETTLE) {
            if (s_t >= MI2_FRIC_SETTLE_MS) {
                s_sum_iq  = 0.0f;
                s_sum_rpm = 0.0f;
                s_navg    = 0U;
                s_t       = 0U;
                s_state   = MI2_MEAS;
            }
            return;
        }
        if (s_state == MI2_MEAS) {
            float wmeas = BspServo_GetOmegaMech();
            Servo_GetSpeedState(&wr, &iq, &itm, &ex, &ramp);
            s_sum_iq  += iq;
            s_sum_rpm += wmeas;
            s_navg++;

            if (s_navg >= MI2_FRIC_AVG_N) {
                const float n = (float)s_navg;
                s_res.fric[s_pt].rpm     = MI2_FRIC_RPM[s_pt];
                s_res.fric[s_pt].iq      = s_sum_iq / n;
                s_res.fric[s_pt].rpm_act = s_sum_rpm / n / MI2_RAD_PER_RPM;
                s_res.fric_n = (uint8_t)(s_pt + 1U);

                /* 没转起来：iq 是堵转电流，不能当摩擦 */
                if (fabsf(s_res.fric[s_pt].rpm_act) < (0.7f * MI2_FRIC_RPM[s_pt])) {
                    mi2_fail(MI2_ERR_MOTION);
                    return;
                }

                s_pt++;
                if (s_pt >= MI2_FRIC_PTS) {
                    /* 线性回归 iq = a + b·w，**只用动摩擦区**的点。
                     *
                     * 2026-11-21：原来 8 点全用，实测给出 fric_a=0.2279、
                     * visc=**-0.00517**（负斜率，物理上不成立）。原因是
                     * 1/2/3/6 rpm 落在静摩擦（Stribeck）区，iq 是动摩擦的
                     * 2 倍量级（注释里就记着"1.2 s 时 3 rpm 读到 0.499 A，
                     * 12 rpm 只读到 0.070 A"）——这几个点把截距抬高、斜率
                     * 压成负数。排除后只剩 12/30/80/200 四点，全部在动摩擦区。
                     *
                     * 截距因此代表**动摩擦**（前馈里 tanh 饱和后的值），
                     * 静摩擦由起动助推 ff_boost 负责，两者不混。 */
                    {
                        const float wmin = MI2_FRIC_WMIN_RADS;
                        float sw = 0.0f, si = 0.0f, sww = 0.0f, swi = 0.0f;
                        uint8_t k, n_used = 0U;
                        for (k = 0U; k < MI2_FRIC_PTS; k++) {
                            const float w = s_res.fric[k].rpm_act * MI2_RAD_PER_RPM;
                            const float i2 = s_res.fric[k].iq;
                            if (w < wmin) { continue; }
                            sw += w; si += i2; sww += w * w; swi += w * i2;
                            n_used++;
                        }
                        if (n_used < 2U) {
                            mi2_fail(MI2_ERR_DATA);
                            return;
                        }
                        {
                            const float nn = (float)n_used;
                            const float den = (nn * sww) - (sw * sw);
                            if (fabsf(den) < 1e-9f) {
                                mi2_fail(MI2_ERR_DATA);
                                return;
                            }
                            s_res.visc   = ((nn * swi) - (sw * si)) / den;
                            s_res.fric_a = (si / nn) - (s_res.visc * (sw / nn));
                            if (s_res.fric_a < 0.0f) { s_res.fric_a = 0.0f; }
                        }

                        /* 诊断量：静 / 动摩擦比（不参与标定，供特性分析）
                         * 低速三点（1/2/3 rpm）在粘滑区，iq 忽大忽小，
                         * 取 |iq| 的最大值代表"起转要克服的静摩擦"；
                         * 动摩擦取动摩擦区的最小 iq（该区 iq 随转速单调上升）。
                         * 比值 = 静/动，实测约 2，正是 ff_boost 默认 1.0
                         * （≈额外补一份动摩擦）的来源。 */
                        {
                            float iq_lo = 0.0f, iq_dyn = 0.0f;
                            uint8_t got_dyn = 0U;

                            for (k = 0U; k < MI2_FRIC_PTS; k++) {
                                const float iq_abs = fabsf(s_res.fric[k].iq);
                                const float w = s_res.fric[k].rpm_act * MI2_RAD_PER_RPM;

                                if ((k < 3U) && (iq_abs > iq_lo)) { iq_lo = iq_abs; }
                                if (w >= MI2_FRIC_WMIN_RADS) {
                                    if ((got_dyn == 0U) || (iq_abs < iq_dyn)) {
                                        iq_dyn = iq_abs;
                                        got_dyn = 1U;
                                    }
                                }
                            }
                            s_res.fric_iq_lo = iq_lo;
                            s_res.fric_ratio = (iq_dyn > 1.0e-3f) ? (iq_lo / iq_dyn) : 0.0f;
                        }
                    }
                    Servo_SetSpeedMode(0U, 0.0f);
                    BspServo_PwmEnable(0U);
                    s_state = MI2_DONE;
                    return;
                }

                Servo_SetSpeedMode(1U, MI2_FRIC_RPM[s_pt] * MI2_RAD_PER_RPM);
                s_state = MI2_SETTLE;
                s_t     = 0U;
            }
            return;
        }
        return;
    }

    /* ================= 模式 3：折算惯量 ================= */
    if (s_mode == 3U) {
        /* 时序：静止 → iq1 加速 → 静止 → iq2 加速 → 算 */
        if (s_state == MI2_SETTLE) {
            if (s_t < 1U) { s_inj_amp = 0.0f; s_t1 = s_t2 = 0U; s_tick = 0U; s_wprev = 0.0f; }
            if (s_t >= MI2_INERTIA_REST) {
                /* 用 s_got1 判断阶段，不能用 s_leg ——
                 * s_leg 在第一段结束时被清 0（表示"不在加速中"），
                 * 若拿它当阶段标志，第二轮会重新跑第一段，永远进不了第二段
                 * （现象是 "MI start inertia" 之后无任何输出、卡到别人 busy）。 */
                if (s_got1 == 0U) {
                    s_inj_amp = MI2_INERTIA_IQ1;
                    s_leg = 1U;
                    /* 只在**第一段**锁存 Kt（s_t 在进分支前已经 ++，
                     * 所以不能用 s_t<1U 做"刚进入"的判据 —— 那条件永不成立，
                     * 实测踩过一次：tq 仍是 0.0910，用的还是旧常量）。
                     * 两段之间电机会升温，若第二段重新取，ΔT 里会混进 Kt 的变化。 */
                    s_kt_used = Servo_GetKtNow();
                } else {
                    s_inj_amp = MI2_INERTIA_IQ2;
                    s_leg = 2U;
                }
                s_t1 = s_t2 = 0U;
                s_tick = 0U;
                s_wprev = 0.0f;
                s_inj_en  = 1U;
                s_inj_ac  = 0U;        /* 直流：只要恒定转矩，不要交流 */
                s_inj_dph = 0.0f;
                s_inj_ph  = 0.0f;
                s_t = 0U;
                s_state = MI2_MEAS;
            }
            return;
        }
        if (s_state == MI2_MEAS) {
            /* 跨过 w2 或超时 */
            if ((s_t2 != 0U) || (s_t > MI2_INERTIA_MAX)) {
                s_inj_en  = 0U;
                s_inj_amp = 0.0f;

                if (s_t2 == 0U) { mi2_fail(MI2_ERR_MOTION); return; }

                {
                    const float dt = (float)(s_t2 - s_t1) * MI2_TS;
                    const float dw = MI2_INERTIA_W2 - MI2_INERTIA_W1;
                    if (dt < 1e-6f) { mi2_fail(MI2_ERR_DATA); return; }

                    if (s_leg == 1U) {
                        s_res.acc1 = dw / dt;
                        s_res.tq1  = s_kt_used * MI2_INERTIA_IQ1;
                        s_got1 = 1U;
                        s_leg  = 0U;
                        s_state = MI2_SETTLE;
                        s_t = 0U;
                        return;
                    }

                    /* 第二段：J = ΔT / (Δw·(1/t2 − 1/t1))，
                     * 两式相除把未知的摩擦项消掉了。 */
                    s_res.acc2 = dw / dt;
                    s_res.tq2  = s_kt_used * MI2_INERTIA_IQ2;

                    if (s_got1 == 0U) { mi2_fail(MI2_ERR_DATA); return; }
                    {
                        const float dt1 = dw / s_res.acc1;
                        const float dt2 = dw / s_res.acc2;
                        const float den = dw * ((1.0f / dt2) - (1.0f / dt1));
                        if (fabsf(den) < 1e-6f) { mi2_fail(MI2_ERR_DATA); return; }
                        s_res.j_eq = (s_res.tq2 - s_res.tq1) / den;

                        if ((s_res.j_eq < 1e-6f) || (s_res.j_eq > 1.0f)) {
                            mi2_fail(MI2_ERR_DATA);       /* 超出合理范围 */
                            return;
                        }
                    }
                    Servo_SetCurrentMode(0U, 0.0f);
                    BspServo_PwmEnable(0U);
                    s_state = MI2_DONE;
                }
            }
            return;
        }
        return;
    }

    /* ================= 模式 4：齿槽（正反各一圈） ================= */
    if (s_mode == 4U) {
        if (s_state == MI2_SETTLE) {
            if (s_t >= MI2_COG_SETTLE_MS) {
                /* 只有正向圈取起点。反向圈必须沿用同一个 θ0，
                 * 否则 θ0 被重设到反向起点，br 从 0 起、进度从 2π 起，
                 * 立刻判"已转完"，反向数据全空。 */
                if (s_cog_leg == 0U) {
                    s_cog_th0 = BspServo_GetThetaMulti();
                }
                s_cog_run = 1U;
                s_t       = 0U;
                s_state   = MI2_MEAS;
            }
            return;
        }
        if (s_state == MI2_MEAS) {
            const float br   = BspServo_GetThetaMulti() - s_cog_th0;
            /* 正向：进度 = br；反向：转子往回走，br 递减，进度 = 2π − br */
            const float prog = (s_cog_leg == 0U) ? br : (BSP_TWO_PI - br);

            /* 转够一圈就收（或超时） */
            if ((prog >= BSP_TWO_PI) || (s_t > 15000U)) {
                s_cog_run = 0U;

                if (prog < (0.8f * BSP_TWO_PI)) {
                    mi2_fail(MI2_ERR_MOTION);      /* 没转到一圈 */
                    return;
                }

                /* ---- 正向圈完：掉头再跑一圈 ---- */
                if (s_cog_leg == 0U) {
                    s_cog_leg = 1U;
                    s_t       = 0U;
                    s_state   = MI2_SETTLE;        /* 回 SETTLE 等速度反向建立 */
                    Servo_SetSpeedMode(1U, -MI2_COG_RPM * MI2_RAD_PER_RPM);
                    return;
                }

                /* ---- 两向都完：平均 ---- */
                {
                    float lo = 1e9f, hi = -1e9f;
                    float lo_f = 1e9f, hi_f = -1e9f;
                    float lo_r = 1e9f, hi_r = -1e9f;
                    float sum_avg = 0.0f;
                    uint16_t k;
                    uint8_t n_ok = 0U;

                    for (k = 0U; k < s_cog_bins; k++) {
                        const float vf = (s_cog_cnt[k] > 0U)
                                       ? (s_cog_acc[k] / (float)s_cog_cnt[k]) : 0.0f;
                        const float vr = (s_cog_rcnt[k] > 0U)
                                       ? (s_cog_rev[k] / (float)s_cog_rcnt[k]) : vf;
                        /* 两向都采到才平均；某一向缺这个箱就用另一向，避免留 0 洞 */
                        const float v = ((s_cog_cnt[k] > 0U) && (s_cog_rcnt[k] > 0U))
                                      ? (0.5f * (vf + vr)) : ((s_cog_cnt[k] > 0U) ? vf : vr);

                        s_res.cog[k] = (int16_t)(v * MI2_COG_SCALE);
                        sum_avg += v;
                        n_ok++;

                        if (vf < lo_f) { lo_f = vf; }
                        if (vf > hi_f) { hi_f = vf; }
                        if (vr < lo_r) { lo_r = vr; }
                        if (vr > hi_r) { hi_r = vr; }
                        if (v < lo) { lo = v; }
                        if (v > hi) { hi = v; }
                    }
                    s_res.cog_n      = s_cog_bins;
                    s_res.cog_pp     = hi - lo;
                    s_res.cog_pp_fwd = hi_f - lo_f;
                    s_res.cog_pp_rev = hi_r - lo_r;
                    /* 平均表的均值：正向表均值约等于摩擦常值 + 齿槽均值，
                     * 平均后摩擦项抵消，这个值应接近 0（残留说明两向不对称） */
                    s_res.cog_dc     = (n_ok > 0U) ? (sum_avg / (float)n_ok) : 0.0f;

                    /* 相位参考：把 s_cog_th0 归一化到 [0,2π)。
                     * 用 fmodf 而不是减 2π 的倍数 —— θ_multi 在长时间运行后
                     * 可能累积到几十 rad，循环减法次数不可控。 */
                    {
                        float ph = s_cog_th0;
                        ph -= BSP_TWO_PI * floorf(ph * (1.0f / BSP_TWO_PI));
                        s_res.cog_phase = ph;
                    }
                }
                Servo_SetSpeedMode(0U, 0.0f);
                BspServo_PwmEnable(0U);
                s_state = MI2_DONE;
            }
            return;
        }
        return;
    }
}
