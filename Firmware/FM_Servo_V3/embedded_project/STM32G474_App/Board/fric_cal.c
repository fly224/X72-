/**
  ******************************************************************************
  * @file    fric_cal.c
  * @brief   速度环摩擦标定（多点 + 正反平均 + 最小二乘，约 12 s）
  ******************************************************************************
  * 为什么从"两点单向"改成"三点正反 + 回归"（2026-11-21）：
  *
  *   两点法的 visc 是 iq2 与 iq1 的**差分除以转速差**，而 Δiq 只有 ~0.04 A、
  *   齿槽残余与速度纹波就在 ±0.01 A 量级 —— 实测给出 4.8e-3，而 1000 rpm
  *   稳态反推是 2.3e-3，回写后 1000 rpm 的 iterm 从 +0.050 A 掉到 −0.145 A
  *   （明显过补偿）。差分对单点噪声没有抑制能力。
  *
  *   现在：
  *     3 个转速点（15 / 60 / 200 rpm），每点正反各测一次，
  *     正反取**绝对值**平均 —— 摩擦在正反两向都阻碍运动、方向相反，
  *     取绝对值才对；直接代数平均会相消成 0。
  *     同时齿槽（位置函数）与重力偏置在正反里符号相反，幅值平均自动抵消。
  *     三点做最小二乘 iq = a + b·ω，斜率 b 比两点差分稳得多。
  *
  * 时序（每段）：稳定 1.2 s + 平均 0.8 s，共 6 段 = 12 s。
  * 平均窗口 800 ms 的取法见 FC_AVG_MS 的说明（必须覆盖多个齿槽周期）。
  *
  * 回写规则：
  *   库仑项 a 一定回写（前馈主导项，测得的准）。
  *   黏滞 b 也回写，但先做合理性检查（0 <= b <= FC_VISC_MAX），
  *   超出就判失败而不是硬塞一个离谱值。
  ******************************************************************************
  */

#include "fric_cal.h"
#include "bsp_servo.h"
#include "actuator.h"

#include <math.h>

/* 三个转速点（rpm）。
 * 低速点用 30 而不是 15：15 rpm 下一圈要 4 s，1 s 窗口只覆盖 90°，
 * 正反两次停在不同齿槽相位，平均不掉（实测 fric 三次标定 0.073/0.084/0.099，
 * 重复性 ±15%）。30 rpm 一圈 2 s，1 s 覆盖 180°（12 槽约 6 个齿槽周期），
 * 齿槽偏置被压到可忽略。100 居中，300 已过齿槽门控上限，靠多圈平均。 */
#define FC_NPTS          (3U)
static const float FC_RPM_PTS[FC_NPTS] = { 30.0f, 100.0f, 300.0f };

#define FC_SETTLE_MS     (1200U)   /* 每段稳定时间 */
/* 平均窗口 1 s：见上面转速点的说明，必须覆盖足够多的齿槽周期。
 * 6 段 × (1.2+1.0) = 13.2 s。 */
#define FC_AVG_MS        (1000U)

/* 结果可信度门限 */
#define FC_RPM_RATIO_MIN (0.80f)   /* 实测转速必须达到给定的这个比例 */
#define FC_VISC_MAX      (0.012f)  /* 黏滞上限：约 5 倍于 1000 rpm 反推值(2.3e-3)，
                                    * 超过说明回归被噪声带偏，判失败 */

static FricCalState  s_state = FC_IDLE;
static FricCalResult s_res;
static uint16_t      s_t;
static uint16_t      s_n;
static float         s_sum_iq;
static float         s_sum_rpm;

static uint8_t       s_pt;                 /* 当前转速点 0..FC_NPTS-1 */
static uint8_t       s_rev;                /* 0 正转 / 1 反转 */
static float         s_iq[FC_NPTS][2];     /* 每点的 iq（带符号） */
static float         s_rpm[FC_NPTS][2];    /* 每点的实测转速（带符号） */

static float         s_ff_keep[3];      /* 标定前的 acc/fric/visc，失败要还原 */
static uint8_t       s_ff_keep_en;

static const float FC_RAD_PER_RPM = 0.10471975512f;

static void fc_fail(FricCalErr e)
{
    s_res.err = (uint8_t)e;
    s_state   = FC_FAIL;
    /* 退回标定前的状态：关环 + 封锁，前馈参数还原 */
    Servo_SetSpeedMode(0U, 0.0f);
    BspServo_PwmEnable(0U);
    Servo_SetFeedforward(s_ff_keep_en,
                         s_ff_keep[0], s_ff_keep[1], s_ff_keep[2]);
}

static void fc_enter_speed(float rpm)
{
    Servo_SetSpeedMode(1U, rpm * FC_RAD_PER_RPM);
    BspServo_PwmEnable(1U);
    s_t = 0U;
}

/* 进入第 (s_pt, s_rev) 段，正转或反转 */
static void fc_enter_segment(void)
{
    fc_enter_speed((s_rev == 0U) ? FC_RPM_PTS[s_pt] : -FC_RPM_PTS[s_pt]);
    s_state = FC_SETTLE1;
}

/* 6 段采完：正反幅值平均 → 三点最小二乘 → 回写前馈 */
static void fc_finish(void)
{
    float w[FC_NPTS], iq[FC_NPTS], rpm[FC_NPTS];
    float sw = 0.0f, si = 0.0f, sww = 0.0f, swi = 0.0f;
    float den, a, b;
    uint8_t k;

    for (k = 0U; k < FC_NPTS; k++) {
        /* 绝对值平均：正转 iq 为正、反转 iq 为负，直接代数平均会相消成 0。
         * 摩擦在正反两向都阻碍运动、幅值相同，取 |·| 才是同一个物理量；
         * 齿槽与重力偏置在正反里符号相反，幅值平均自动抵消它们。 */
        rpm[k] = 0.5f * (fabsf(s_rpm[k][0]) + fabsf(s_rpm[k][1]));
        iq[k]  = 0.5f * (fabsf(s_iq[k][0])  + fabsf(s_iq[k][1]));
        w[k]   = rpm[k] * FC_RAD_PER_RPM;

        sw  += w[k];
        si  += iq[k];
        sww += w[k] * w[k];
        swi += w[k] * iq[k];
    }

    den = ((float)FC_NPTS * sww) - (sw * sw);
    if (fabsf(den) < 1e-9f) {
        fc_fail(FC_ERR_SPREAD);
        return;
    }
    b = (((float)FC_NPTS * swi) - (sw * si)) / den;
    a = (si / (float)FC_NPTS) - (b * sw / (float)FC_NPTS);

    if ((b < 0.0f) || (b > FC_VISC_MAX)) {
        fc_fail(FC_ERR_SPREAD);
        return;
    }
    if (a < 0.0f) { a = 0.0f; }

    s_res.iq1   = iq[0];
    s_res.iq2   = iq[FC_NPTS - 1U];
    s_res.rpm1  = rpm[0];
    s_res.rpm2  = rpm[FC_NPTS - 1U];
    s_res.w1    = w[0];
    s_res.w2    = w[FC_NPTS - 1U];
    s_res.visc  = b;
    /* Kt 读当前生效值（含温度补偿），不能写死：tc_nm 只用于报告，
     * 但"两处不同源"是本工程反复踩的坑。 */
    s_res.tc_nm = Servo_GetKtNow() * a;

    /* 回写规则（2026-11-21 实测后确定）
     *
     * 库仑项 a：一定回写。多点和正反平均让它的重复性从 ±10% 提到 ±2%
     * （0.079~0.084 A），而且它是前馈的主导项。
     *
     * 黏滞项 b：**只报告，不回写**。这台机器的摩擦不是一条直线：
     *   15→200 rpm 段斜率 0.0056，而 200→1000 rpm 段只有 ~0.0028
     *   （低速段是混合润滑/Stribeck，高速段趋平）。
     * 实测把 b 回写后，1000 rpm 的 iterm 从 +0.049 A 变成 −0.209 A ——
     * 稳态转速靠积分兜住，但 iq 里常驻一项负积分，动态会凹、静态多发热。
     * 所以 visc 保留标定前的值（1000 rpm 稳态反推或 30082 手调），
     * 回归值写进结果帧供参考（v[2]），并在报告里标成"低速段阻尼"。 */
    Servo_SetFeedforward(1U, s_ff_keep[0], a, s_ff_keep[2]);

    Servo_SetSpeedMode(0U, 0.0f);
    BspServo_PwmEnable(0U);
    s_state = FC_DONE;
}

void FricCal_Init(void)
{
    s_state = FC_IDLE;
    s_t     = 0U;
    s_n     = 0U;
}

FricCalErr FricCal_Start(void)
{
    uint8_t k;

    if (s_state != FC_IDLE && s_state != FC_DONE && s_state != FC_FAIL) {
        return FC_ERR_BUSY;
    }
    if (BspServo_IsPwmEnabled() != 0U) { return FC_ERR_BUSY; }
    if (BspServo_IsCurrentCalibrated() == 0U) { return FC_ERR_CAL; }
    if (BspServo_IsEncoderZeroed() == 0U)     { return FC_ERR_ZERO; }
    if (BspServo_GetFault() != 0U)            { return FC_ERR_FAULT; }
    if (Act_Acquire(ACT_FRIC_CAL, FricCal_IsBusy) == 0U) { return FC_ERR_BUSY; }

    /* 记下原前馈设置：失败时还原，避免"标定失败反而把前馈搞坏" */
    Servo_GetFeedforward(&s_ff_keep_en, &s_ff_keep[0], &s_ff_keep[1], &s_ff_keep[2]);
    Servo_SetFeedforward(0U, s_ff_keep[0], s_ff_keep[1], s_ff_keep[2]);

    s_res.err  = FC_OK;
    s_res.iq1  = 0.0f;
    s_res.iq2  = 0.0f;
    s_res.visc = 0.0f;
    s_res.tc_nm = 0.0f;
    s_res.rpm1 = 0.0f;
    s_res.rpm2 = 0.0f;
    s_res.w1   = FC_RPM_PTS[0] * FC_RAD_PER_RPM;
    s_res.w2   = FC_RPM_PTS[FC_NPTS - 1U] * FC_RAD_PER_RPM;

    for (k = 0U; k < FC_NPTS; k++) {
        s_iq[k][0] = s_iq[k][1] = 0.0f;
        s_rpm[k][0] = s_rpm[k][1] = 0.0f;
    }
    s_pt = 0U;
    s_rev = 0U;
    s_sum_iq = s_sum_rpm = 0.0f;
    s_n = 0U;

    /* 顺序：先起环再放 MOE（与 do_speed 一致，放开那一拍积分从 0 起步） */
    Servo_SetEnable(0U);
    Servo_SetMode(SERVO_MODE_SPEED);
    Servo_SetSpeedMode(1U, FC_RPM_PTS[0] * FC_RAD_PER_RPM);
    BspServo_PwmEnable(1U);

    if (BspServo_IsPwmEnabled() == 0U) {
        s_res.err = FC_ERR_FAULT;
        s_state   = FC_FAIL;
        Servo_SetFeedforward(s_ff_keep_en, s_ff_keep[0], s_ff_keep[1], s_ff_keep[2]);
        return FC_ERR_FAULT;
    }

    s_state = FC_SETTLE1;
    s_t     = 0U;
    return FC_OK;
}

void FricCal_Abort(void)
{
    if ((s_state == FC_IDLE) || (s_state == FC_DONE) || (s_state == FC_FAIL)) {
        s_state = FC_IDLE;
        return;
    }
    fc_fail(FC_ERR_MOTION);     /* 被外部打断：按失败处理并还原状态 */
    s_state = FC_IDLE;
}

FricCalState FricCal_GetState(void)        { return s_state; }

uint8_t FricCal_IsBusy(void)
{
    return ((s_state != FC_IDLE) && (s_state != FC_DONE) &&
            (s_state != FC_FAIL)) ? 1U : 0U;
}
const FricCalResult *FricCal_GetResult(void) { return &s_res; }

const char *FricCal_ErrStr(FricCalErr e)
{
    switch (e) {
    case FC_OK:          return "ok";
    case FC_ERR_BUSY:    return "busy (send D first)";
    case FC_ERR_CAL:     return "current not calibrated (C)";
    case FC_ERR_ZERO:    return "elec angle not aligned (Z)";
    case FC_ERR_FAULT:   return "fault present (X)";
    case FC_ERR_MOTION:  return "motor did not follow (check load/phase)";
    case FC_ERR_SPREAD:  return "regression rejected (visc slope out of range)";
    default:             return "?";
    }
}

void FricCal_Task1ms(void)
{
    float iq = 0.0f, wr = 0.0f;
    float wmeas = 0.0f, itm = 0.0f, kp = 0.0f, kits = 0.0f, ramp = 0.0f;
    uint32_t ex = 0U;

    if ((s_state == FC_IDLE) || (s_state == FC_DONE) || (s_state == FC_FAIL)) {
        return;
    }

    /* 任何时刻有故障就中止：继续转没意义，而且可能是接线/相序问题 */
    if (BspServo_GetFault() != 0U) {
        fc_fail(FC_ERR_FAULT);
        return;
    }

    s_t++;

    switch (s_state) {
    case FC_SETTLE1:
        if (s_t >= FC_SETTLE_MS) {
            s_sum_iq  = 0.0f;
            s_sum_rpm = 0.0f;
            s_n       = 0U;
            s_t       = 0U;
            s_state   = FC_AVG1;
        }
        break;

    case FC_AVG1:
        Servo_GetSpeedState(&wr, &iq, &itm, &ex, &ramp);
        wmeas = BspServo_GetOmegaMech();
        s_sum_iq  += iq;
        s_sum_rpm += wmeas;
        s_n++;
        if (s_t >= FC_AVG_MS) {
            const float n = (float)s_n;

            if (n < 1.0f) {
                fc_fail(FC_ERR_SPREAD);
                return;
            }
            s_iq[s_pt][s_rev]  = s_sum_iq / n;
            s_rpm[s_pt][s_rev] = s_sum_rpm / n / FC_RAD_PER_RPM;

            /* 没转起来：给定 15 rpm 但实测不到 80%，说明卡住或相序反了。
             * 此时 iq 是"堵转电流"，拿去当前馈会造成危险的正反馈。 */
            if (fabsf(s_rpm[s_pt][s_rev]) < (FC_RPM_RATIO_MIN * FC_RPM_PTS[s_pt])) {
                fc_fail(FC_ERR_MOTION);
                return;
            }

            /* 下一段：先反向同一点，再进下一个转速点 */
            if (s_rev == 0U) {
                s_rev = 1U;
            } else {
                s_rev = 0U;
                s_pt++;
            }
            if (s_pt >= FC_NPTS) {
                fc_finish();
                return;
            }
            fc_enter_segment();
        }
        break;

    default:
        break;
    }
}

/* 两个工作点转速的对外真值（rpm）。命令层拼回显必须读它，
 * 不能在别处再写一份 —— 硬编码的 "3rpm" 在低速点改到 15 rpm 后就不对了。 */
float FricCal_GetW1Rpm(void) { return FC_RPM_PTS[0]; }
float FricCal_GetW2Rpm(void) { return FC_RPM_PTS[FC_NPTS - 1U]; }
