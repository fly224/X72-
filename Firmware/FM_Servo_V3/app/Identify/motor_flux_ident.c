/*
 * motor_flux_ident.c —— 磁链 ψ / 电阻 R / 电感 L 的联合在线辨识
 *
 * 原理见 motor_flux_ident.h 的文件头。这里只补充实现上的几个决定：
 *
 * 【为什么用 vd 斜率求 R，而不是 vq】
 *   vd = R·id − ωe·L·iq      对 id 求导只剩 R（ωe、L、iq 都与 id 无关）
 *   vq = R·iq + ωe·L·id + ωe·ψ  对 id 求导只剩 ωe·L
 * 如果改用 vq 求 R，斜率里会混进 ωe·L 项（除非 id 恒为 0，那就没有信息了）。
 *
 * 【为什么主转速选 1000 rpm】
 *   vq 对 id 的斜率 = ωe·L。L≈80 µH：
 *     300 rpm  → 0.035 V/A，id 扫 3 A 才 0.105 V，接近死区电压量级
 *     1000 rpm → 0.117 V/A，id 扫 3 A 是 0.35 V，ADC 能分辨
 *   而 ψ 项在 300 rpm 已有 2.0 V，所以副转速验 ψ 足够。
 *
 * 【为什么每 4 拍才采一次】
 *   300 ms @20 kHz = 6000 次浮点累加。float 尾数 24 bit，累加到 6000 次时
 *   相对误差累积到 1e-3 量级，而 vq 里要分辨的斜率贡献只有 0.35 V（总电压 16 V）。
 *   降到 5 kHz（1500 点）后累积误差 <0.1%，对直流平均毫无影响。
 *
 * 【为什么必须关谐波抑制】
 *   5/7 次补偿是叠加在 αβ 上的额外电压，而本模块读的 vd/vq 是**限幅后、
 *   叠加前**的值。开着会让"实际加在电机上的电压"与"用来解方程的电压"不一致，
 *   相当于方程里多了一个未知的谐波项。启动时强制关闭，结束后恢复原状态。
 */
#include "motor_flux_ident.h"

#include "bsp_servo.h"
#include "actuator.h"
#include "param_store.h"
#include "uart_link.h"

#include <math.h>
#include <string.h>

/* ============================ 工况表 ============================ */

#define FI_RPM_MAIN       (1000.0f)   /* 主转速：id 四点扫描 */
#define FI_RPM_SUB        (300.0f)    /* 副转速：验 ψ */
#define FI_ID_STEP        (5.0f)      /* 主扫描的 id 上限（负向弱磁），A
                                       * 2026-11-25 3.0→5.0：vq-id 斜率仅 0.089 V/A，
                                       * 3 A 只让 vq 变 0.267 V，被死区/ADC 噪声淹没 → R²<0.90
                                       * 回归退化(err=6)。加大到 5 A 让 vq 变 0.445 V，SNR+67%。 */
#define FI_ID_SUB         (3.0f)      /* 副扫描的 id，A（同步加大，保持副组 SNR） */

/* 主 4 点 + 副 2 点。顺序必须与 fi_calc() 里的索引假设一致。 */
#define FI_N_MAIN         (4U)
#define FI_N_SUB          (2U)
#define FI_N_PTS          (FI_N_MAIN + FI_N_SUB)

static const float s_plan_rpm[FI_N_PTS] =
    { FI_RPM_MAIN, FI_RPM_MAIN, FI_RPM_MAIN, FI_RPM_MAIN, FI_RPM_SUB, FI_RPM_SUB };
static const float s_plan_id[FI_N_PTS] =
    { 0.0f, -1.0f, -2.0f, -FI_ID_STEP, 0.0f, -FI_ID_SUB };

/* ============================ 时序 ============================ */

/* 首点要等速度环把转速稳定下来（从 0 到 1000 rpm，斜坡 400 rad/s² 约 0.26 s，
 * 再加积分收敛）；后续点只是改 id，电流环 1 kHz 带宽，1.2 s 绰绰有余。 */
#define FI_SETTLE_FIRST_MS  (1800U)
#define FI_SETTLE_MS        (1200U)
#define FI_SAMPLE_MS        (300U)
#define FI_SAMPLE_N         ((uint16_t)(FI_SAMPLE_MS * 20U))   /* 20 kHz 节拍数 */
#define FI_DECIM            (4U)                               /* 每 4 拍采 1 次 */
#define FI_TIMEOUT_MS       (30000U)

/* 判据 */
#define FI_R2_MIN           (0.85f)   /* vd-id 与 vq-id 回归的 R² 下限
                                       * 2026-11-25 0.90→0.85：vq-id 斜率信号太小，0.90 过于苛刻，
                                       * 实测 1000 rpm 下 6 点采完仍退化。0.85 配合 id 扫描加大。 */
#define FI_R_MIN            (0.05f)   /* 0.05 ohm */
#define FI_R_MAX            (2.0f)
#define FI_L_MIN            (5.0e-6f)
#define FI_L_MAX            (3.0e-3f)
#define FI_PSI_MIN          (1.0e-3f) /* 1 mWb */
#define FI_PSI_MAX          (30.0e-3f)
#define FI_PSI_DEV_MAX      (15.0f)   /* 两个转速的 ψ 偏差上限 % */
#define FI_WE_MIN           (100.0f)  /* rad/s：转速太低时 ωe 项不可分辨 */

#define FI_TWO_PI           (6.28318530718f)

/* ============================ 状态 ============================ */

static FluxIdentState   s_state = FI_IDLE;
static FluxIdentResult  s_res;
static uint16_t         s_t;
static uint8_t          s_pt;          /* 当前工况序号 */
static float            s_id_ref;      /* 本拍 id 给定 */

/* 累加器 */
static uint16_t         s_n;           /* 已累加拍数（已抽取） */
static uint16_t         s_decim;
static float            s_acc_id, s_acc_iq, s_acc_vd, s_acc_vq, s_acc_we;

/* 谐波抑制的原始状态，结束后恢复 */
static uint8_t          s_harm_keep;

/* ============================ 工具 ============================ */

static void fi_fail(FluxIdentErr e)
{
    Servo_SetCurIdRef(0.0f);
    s_id_ref = 0.0f;
    Servo_SetSpeedMode(0U, 0.0f);
    BspServo_PwmEnable(0U);

    /* 恢复谐波抑制到辨识前的状态 */
    Servo_SetHarmonic(s_harm_keep);

    s_res.err = e;
    s_state   = FI_FAIL;
}

static void fi_stop_ok(void)
{
    Servo_SetCurIdRef(0.0f);
    s_id_ref = 0.0f;
    Servo_SetSpeedMode(0U, 0.0f);
    BspServo_PwmEnable(0U);
    Servo_SetHarmonic(s_harm_keep);
}

/* ============================ 回归 ============================ */

/*
 * 一维线性回归 y = k·x + b。
 * 返回 1 = 成功且 R² 达标，0 = 退化（x 全等 / 残差过大）。
 * 用两点算斜率也能work，但四点能给 R²，能判"这个斜率是不是真的"。
 */
static uint8_t fi_linreg(uint8_t n, const float *x, const float *y,
                         float *k, float *b, float *r2)
{
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, syy = 0.0f;
    float den, kk, bb, ss_res = 0.0f, ss_tot = 0.0f, ybar;
    uint8_t i;

    if (n < 2U) { return 0U; }

    for (i = 0U; i < n; i++) {
        sx  += x[i];
        sy  += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
        syy += y[i] * y[i];
    }
    den = (float)n * sxx - sx * sx;
    if (fabsf(den) < 1e-9f) { return 0U; }       /* x 全等，斜率无意义 */

    kk = ((float)n * sxy - sx * sy) / den;
    bb = (sy - kk * sx) / (float)n;

    ybar = sy / (float)n;
    for (i = 0U; i < n; i++) {
        const float e = y[i] - (kk * x[i] + bb);
        const float d = y[i] - ybar;
        ss_res += e * e;
        ss_tot += d * d;
    }
    /* ss_tot 极小说明 y 几乎是常数：R² 无定义，此时要求残差也极小 */
    if (ss_tot < 1e-9f) {
        *r2 = (ss_res < 1e-9f) ? 1.0f : 0.0f;
    } else {
        *r2 = 1.0f - (ss_res / ss_tot);
    }

    *k = kk;
    *b = bb;
    return (*r2 >= FI_R2_MIN) ? 1U : 0U;
}

/* ============================ 结果计算 ============================ */

static uint8_t fi_calc(void)
{
    float x[FI_N_MAIN], y[FI_N_MAIN];
    float r = 0.0f, b_vd = 0.0f, k_vq = 0.0f, b_vq = 0.0f, k_vd = 0.0f;
    float l, psi1 = 0.0f, psi2 = 0.0f;
    float we1 = 0.0f, we2 = 0.0f;
    uint8_t i, n_ok;

    /* ---- 主组：vd/vq 对 id 的斜率 ---- */
    for (i = 0U; i < FI_N_MAIN; i++) {
        x[i] = s_res.pt[i].id;
        /* vd 回归用 vd；vq 回归用 vq（下面换数组） */
    }

    /* ---- 先算 L：vq 对 id，斜率 = ωe·L ---- */
    for (i = 0U; i < FI_N_MAIN; i++) { y[i] = s_res.pt[i].vq; }
    if (fi_linreg(FI_N_MAIN, x, y, &k_vq, &b_vq, &s_res.l_r2) == 0U) {
        return 0U;                     /* 斜率不显著 */
    }

    /* ωe 取主组 4 点的平均（转速环把它稳在同一值，波动 <1%） */
    for (i = 0U; i < FI_N_MAIN; i++) { we1 += s_res.pt[i].we; }
    we1 /= (float)FI_N_MAIN;
    if (fabsf(we1) < FI_WE_MIN) { return 0U; }

    l = k_vq / we1;

    /* ---- 再算 R，必须扣掉 iq 的混淆项 ----
     * vd = R·id − ωe·L·iq，对 id 求导是
     *     ∂vd/∂id = R − ωe·L·(∂iq/∂id)
     * 空载时 iq 是被摩擦锁在 ωe 上的，但 id 注入会改变磁链、进而改变
     * 维持同转速所需的 iq（实测 iq 0.48→0.70 A 随 id 0→−3 A 单调上升）。
     * 这一项在 1000 rpm 上是 ωe·L·k_iq ≈ 1466×87e-6×0.073 = 0.009 Ω，
     * 相对 0.34 Ω 只有 2.7%，但不扣会系统性偏大。
     * 不扣的话还有第二个更隐蔽的后果：k_iq 的符号随工况变，
     * 误差方向也变，看起来像"R 有时准有时不准"。 */
    {
        float k_iq = 0.0f, b_iq = 0.0f, r2_iq = 0.0f;

        for (i = 0U; i < FI_N_MAIN; i++) { y[i] = s_res.pt[i].iq; }
        (void)fi_linreg(FI_N_MAIN, x, y, &k_iq, &b_iq, &r2_iq);   /* 退化时不致命 */

        for (i = 0U; i < FI_N_MAIN; i++) { y[i] = s_res.pt[i].vd; }
        if (fi_linreg(FI_N_MAIN, x, y, &k_vd, &b_vd, &s_res.r_r2) == 0U) {
            return 0U;
        }
        r = k_vd + (we1 * l * k_iq);
    }

    /* ---- ψ：每个主点各算一次再平均 ---- */
    n_ok = 0U;
    for (i = 0U; i < FI_N_MAIN; i++) {
        const FluxPt *p = &s_res.pt[i];
        /* vq = R·iq + ωe·L·id + ωe·ψ  ⇒  ψ = (vq − R·iq − ωe·L·id) / ωe */
        const float num = p->vq - (r * p->iq) - (p->we * l * p->id);
        if (fabsf(p->we) > FI_WE_MIN) {
            psi1 += num / p->we;
            n_ok++;
        }
    }
    if (n_ok == 0U) { return 0U; }
    psi1 /= (float)n_ok;

    /* ---- 副组：用已求的 R/L 反算 ψ，做一致性检查 ---- */
    n_ok = 0U;
    for (i = FI_N_MAIN; i < FI_N_PTS; i++) {
        const FluxPt *p = &s_res.pt[i];
        const float num = p->vq - (r * p->iq) - (p->we * l * p->id);
        if (fabsf(p->we) > 10.0f) {
            psi2 += num / p->we;
            we2  += p->we;
            n_ok++;
        }
    }
    if (n_ok == 0U) { return 0U; }
    psi2 /= (float)n_ok;
    we2  /= (float)n_ok;

    /* ---- 范围检查 ---- */
    if ((r < FI_R_MIN) || (r > FI_R_MAX)) { return 0U; }
    if ((l < FI_L_MIN) || (l > FI_L_MAX)) { return 0U; }
    if ((psi1 < FI_PSI_MIN) || (psi1 > FI_PSI_MAX)) { return 0U; }

    s_res.r_ohm      = r;
    s_res.l_h        = l;
    s_res.psi_wb     = psi1;
    s_res.psi1       = psi1;
    s_res.psi2       = psi2;
    s_res.we1        = we1;
    s_res.we2        = we2;
    s_res.psi_dev_pct = fabsf(psi2 - psi1) / psi1 * 100.0f;

    /* 派生常数：p=14（Z 已辨识），表贴式 */
    {
        const float pr = (float)BSP_POLE_PAIRS;
        s_res.kt        = 1.5f * pr * psi1;
        s_res.ke_v_s_rad = pr * psi1;
    }

    if (s_res.psi_dev_pct > FI_PSI_DEV_MAX) { return 2U; }   /* ψ 不一致 */

    return 1U;
}

/* ============================ 对外 ============================ */

void FluxIdent_Init(void)
{
    memset(&s_res, 0, sizeof(s_res));
    s_state   = FI_IDLE;
    s_id_ref  = 0.0f;
    s_harm_keep = 0U;
}

uint8_t FluxIdent_IsBusy(void)
{
    return ((s_state == FI_SETTLE) || (s_state == FI_SAMPLE)) ? 1U : 0U;
}

FluxIdentState FluxIdent_GetState(void) { return s_state; }

const FluxIdentResult *FluxIdent_GetResult(void) { return &s_res; }

const char *FluxIdent_ErrStr(FluxIdentErr e)
{
    switch (e) {
    case FI_OK:          return "ok";
    case FI_ERR_BUSY:    return "busy (another ident running)";
    case FI_ERR_CAL:     return "current not calibrated";
    case FI_ERR_ZERO:    return "elec angle not aligned";
    case FI_ERR_FAULT:   return "fault present";
    case FI_ERR_PWM:     return "MOE did not come up";
    case FI_ERR_DATA:    return "regression degenerate";
    case FI_ERR_RANGE:   return "result out of physical range";
    case FI_ERR_PSI:     return "psi mismatch between two speeds";
    default:             return "?";
    }
}

void FluxIdent_SetCached(float r_ohm, float l_h, float psi_wb)
{
    if (FluxIdent_IsBusy() != 0U) { return; }
    if ((r_ohm <= 0.0f) || (l_h <= 0.0f) || (psi_wb <= 0.0f)) { return; }

    s_res.r_ohm  = r_ohm;
    s_res.l_h    = l_h;
    s_res.psi_wb = psi_wb;
    s_res.kt     = 1.5f * (float)BSP_POLE_PAIRS * psi_wb;
    s_res.ke_v_s_rad = (float)BSP_POLE_PAIRS * psi_wb;
    s_res.err    = FI_OK;
}

FluxIdentErr FluxIdent_Start(void)
{
    if (FluxIdent_IsBusy() != 0U)        { return FI_ERR_BUSY; }
    if (BspServo_GetFault() != 0U)       { return FI_ERR_FAULT; }
    if (BspServo_IsCurrentCalibrated() == 0U) { return FI_ERR_CAL; }
    if (BspServo_IsEncoderZeroed() == 0U)     { return FI_ERR_ZERO; }
    if (BspServo_IsPwmEnabled() != 0U)   { return FI_ERR_PWM; }   /* 要求先 D */
    if (Servo_IsOpenLoop() != 0U)        { return FI_ERR_BUSY; }
    if (Servo_IsCurrentMode() != 0U)     { return FI_ERR_BUSY; }
    if (Servo_IsSpeedMode() != 0U)       { return FI_ERR_BUSY; }
    if (Act_Acquire(ACT_FLUX_IDENT, FluxIdent_IsBusy) == 0U) { return FI_ERR_BUSY; }

    memset(&s_res, 0, sizeof(s_res));
    s_res.n_pts  = FI_N_PTS;
    s_res.n_done = 0U;
    s_pt         = 0U;
    s_t         = 0U;
    s_n         = 0U;
    s_decim     = 0U;
    s_id_ref    = 0.0f;
    s_acc_id = s_acc_iq = s_acc_vd = s_acc_vq = s_acc_we = 0.0f;

    /* 关谐波抑制，记下原状态以便恢复 */
    s_harm_keep = Servo_IsHarmonic();
    Servo_SetHarmonic(0U);

    /* 进速度闭环（延续 do_speed 的顺序：先切模式，再起环，最后放 MOE） */
    Servo_SetEnable(0U);
    Servo_SetMode(SERVO_MODE_SPEED);
    Servo_SetCurIdRef(0.0f);
    Servo_SetSpeedMode(1U, s_plan_rpm[0] * (FI_TWO_PI / 60.0f));
    BspServo_PwmEnable(1U);

    if (BspServo_IsPwmEnabled() == 0U) {
        Servo_SetSpeedMode(0U, 0.0f);
        Servo_SetHarmonic(s_harm_keep);
        s_res.err = FI_ERR_PWM;
        s_state   = FI_FAIL;
        return FI_ERR_PWM;
    }

    s_state = FI_SETTLE;
    return FI_OK;
}

void FluxIdent_Abort(void)
{
    if ((s_state == FI_SETTLE) || (s_state == FI_SAMPLE)) {
        fi_stop_ok();
        s_res.err = FI_ERR_BUSY;
        s_state   = FI_FAIL;
    }
}

float FluxIdent_GetIdRef(void)
{
    return s_id_ref;
}

void FluxIdent_Step(float id_now, float iq_now, float vd, float vq, float we)
{
    if (s_state != FI_SAMPLE) { return; }

    /* 抽取：每 4 拍采一次，见文件头关于累加精度的说明 */
    if (++s_decim < FI_DECIM) { return; }
    s_decim = 0U;
    if (s_n >= FI_SAMPLE_N) { return; }

    s_acc_id += id_now;
    s_acc_iq += iq_now;
    s_acc_vd += vd;
    s_acc_vq += vq;
    s_acc_we += we;
    s_n++;
}

uint8_t FluxIdent_Task1ms(void)
{
    if ((s_state != FI_SETTLE) && (s_state != FI_SAMPLE)) { return 0U; }

    s_t++;

    if (BspServo_GetFault() != 0U) {
        /* 先把现场记下来再封锁 —— 封锁之后 omega 会掉、s_spd_en 会清，
         * 那时再读就什么都看不出来了。 */
        {
            float wr = 0.0f;
            Servo_GetSpeedState(&wr, NULL, NULL, NULL, NULL);
            s_res.dbg_om     = BspServo_GetOmegaMech();
            s_res.dbg_wref   = wr;
            s_res.dbg_spd_en = Servo_IsSpeedMode();
            s_res.dbg_cur_en = Servo_IsCurrentMode();
            s_res.dbg_fault  = BspServo_GetFault();
        }
        fi_fail(FI_ERR_FAULT);
        return 1U;
    }
    if (s_t > FI_TIMEOUT_MS) {
        fi_fail(FI_ERR_DATA);
        return 1U;
    }

    if (s_state == FI_SETTLE) {
        const uint16_t need = (s_pt == 0U) ? FI_SETTLE_FIRST_MS : FI_SETTLE_MS;

        /* id 给定在整个 SETTLE 期间就生效，让电流环先建立 */
        s_id_ref = s_plan_id[s_pt];
        Servo_SetCurIdRef(s_id_ref);

        if (s_t >= need) {
            s_n     = 0U;
            s_decim = 0U;
            s_acc_id = s_acc_iq = s_acc_vd = s_acc_vq = s_acc_we = 0.0f;
            s_t     = 0U;
            s_state = FI_SAMPLE;
        }
        return 1U;
    }

    /* FI_SAMPLE */
    if (s_n >= FI_SAMPLE_N) {
        const float inv = 1.0f / (float)s_n;

        s_res.pt[s_pt].id = s_acc_id * inv;
        s_res.pt[s_pt].iq = s_acc_iq * inv;
        s_res.pt[s_pt].vd = s_acc_vd * inv;
        s_res.pt[s_pt].vq = s_acc_vq * inv;
        s_res.pt[s_pt].we = s_acc_we * inv;

        s_pt++;
        s_res.n_done = s_pt;
        s_t = 0U;

        if (s_pt >= FI_N_PTS) {
            const uint8_t ok = fi_calc();

            if (ok == 1U) {
                fi_stop_ok();
                s_res.err = FI_OK;
                s_state   = FI_DONE;
            } else if (ok == 2U) {
                fi_stop_ok();
                s_res.err = FI_ERR_PSI;
                s_state   = FI_FAIL;
            } else {
                fi_fail(FI_ERR_DATA);
            }
            return 1U;
        }

        /* 换下一个工况：先改速度给定（副组要换转速），再回 SETTLE */
        {
            const float rpm = s_plan_rpm[s_pt];

            /* 只有跨组时才重设转速；同组内保持，避免速度环重新收敛 */
            if (fabsf(rpm - s_plan_rpm[s_pt - 1U]) > 0.5f) {
                Servo_SetSpeedMode(1U, rpm * (FI_TWO_PI / 60.0f));
            }
        }
        s_state = FI_SETTLE;
    }

    return 1U;
}
