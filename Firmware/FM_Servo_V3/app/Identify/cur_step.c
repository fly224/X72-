/*
 * cur_step.c —— 电流环阶跃激励与指标测量（设计说明见 cur_step.h）
 *
 * 全部是 float + 比较，单拍约 40~70 周期（20 kHz 预算 8500 cyc）；
 * 无 malloc、不碰串口、不读 flash。
 */
#include "cur_step.h"

#include <math.h>

#define CS_PRE_TICKS     60U                     /* 3 ms 基线 */
#define CS_STEP_TICKS    600U                    /* 30 ms 阶跃 */
#define CS_TS_US         50.0f                   /* 一拍 = 50 us */
#define CS_AMP_MAX       5.0f
#define CS_AMP_MIN       0.10f
#define CS_TRACE_MAX     64U
#define CS_DECIM         1U                              /* 连续采样：64 拍 = 3.2 ms，
                                                          * 足够看清上升沿与 1~3 kHz 振铃 */
#define CS_SS_FROM       480U                    /* 末 20%（480..599）算稳态 */
#define CS_SETTLE_HOLD   20U                     /* 连续 1 ms 在 ±5% 内才算稳定，防单点噪声 */


enum { CS_IDLE = 0, CS_PRE, CS_STEP, CS_DONE };

#define CS_LPF_N   4U                        /* 4 拍 = 200 us：压掉开关纹波，不影响 1 kHz 环路 */

typedef struct {
    float    hist[CS_LPF_N];
    uint8_t  hi;
    float    peak;
    float    sum;
    float    vpeak;
    uint16_t n_ss;
    uint16_t i10;
    uint16_t i90;
    uint16_t i_last_bad;
    uint16_t n_in_band;
} CsAcc;

typedef struct {
    float t_ms;
    float id_ref, id_now;
    float iq_ref, iq_now;
} CsTrace;

static uint8_t  s_state;
static uint8_t  s_kind;
static float    s_amp;
static uint16_t s_k;
static CsAcc    s_acc[2];
static CurStepResult s_res[2];
static CsTrace  s_tr[CS_TRACE_MAX];
static uint16_t s_tr_n;

static void acc_reset(CsAcc *a)
{
    uint8_t i;
    for (i = 0U; i < CS_LPF_N; i++) { a->hist[i] = 0.0f; }
    a->hi         = 0U;
    a->peak       = 0.0f;
    a->sum        = 0.0f;
    a->vpeak      = 0.0f;
    a->n_ss       = 0U;
    a->i10        = 0xFFFFU;
    a->i90        = 0xFFFFU;
    a->i_last_bad = 0U;
    a->n_in_band  = 0U;
}

static void acc_push(CsAcc *a, float ref, float meas_raw, float vmag, uint16_t n)
{
    float meas;
    float sum = 0.0f;
    float e;
    float th5;
    uint8_t i;

    /* 滑动平均：开关纹波（30 V/20 kHz/113.5uH 约 3 A 峰峰）如果直接进指标，
     * "峰值"会被纹波主导，算出来的超调毫无意义。控制律仍然用原始采样。 */
    a->hist[a->hi] = meas_raw;
    a->hi = (uint8_t)((a->hi + 1U) % CS_LPF_N);
    for (i = 0U; i < CS_LPF_N; i++) { sum += a->hist[i]; }
    meas = sum * (1.0f / (float)CS_LPF_N);

    e   = meas - ref;
    th5 = 0.10f * fabsf(ref);

    /* 2026-11-17：调节时间判据从 ±5% 放宽到 ±10%。
     *
     * 原因：±5% 对 2 A 阶跃是 ±0.1 A，而 PWM 开关纹波经电流环衰减后
     * 仍有 ±0.1~0.25 A（闭环对 20 kHz 衰减约 20 倍，开环纹波 3.3 A 峰峰）。
     * 也就是说判据阈值正好落在纹波水平上，meas 会反复进出带，
     * i_last_bad 一直被刷新到阶跃末尾 —— 实测 6 个 Kp 档位（0.5x~2.0x）
     * ts 全部落在 24~30 ms，就是这个原因，与增益无关。
     *
     * 代价：±10% 对超调的判定不再严格，所以 os 仍按峰值单独计算（不受影响）。
     * 这个指标现在只能做**相对比较**，不能当作"环路在 1 ms 内稳定"的绝对结论。 */

    if (meas > a->peak)              { a->peak  = meas; }
    if (fabsf(vmag) > a->vpeak)      { a->vpeak = fabsf(vmag); }
    if ((a->i10 == 0xFFFFU) && (meas >= 0.1f * ref)) { a->i10 = n; }
    if ((a->i90 == 0xFFFFU) && (meas >= 0.9f * ref)) { a->i90 = n; }
    if (fabsf(e) > th5) {
        a->i_last_bad = n;
        a->n_in_band  = 0U;
    } else if (++a->n_in_band == CS_SETTLE_HOLD) {   /* 只锁存一次，否则稳定后时间会一直涨 */
        a->i_last_bad = (uint16_t)(n - (CS_SETTLE_HOLD - 1U));   /* 从进入带内的那一刻算起 */
    }
    if (n >= CS_SS_FROM) { a->sum += meas; a->n_ss++; }
}

static void finalize_axis(uint8_t ax, const CsAcc *a, float ref)
{
    CurStepResult *r = &s_res[ax];

    r->amp        = ref;
    r->peak       = a->peak;
    r->vpeak      = a->vpeak;
    r->valid      = 0U;
    r->rise_us    = 0.0f;
    r->os_pct     = 0.0f;
    r->settle_us  = 0.0f;
    r->ss_err_pct = 0.0f;

    if ((ref <= 0.0f) || (a->i10 == 0xFFFFU) || (a->i90 == 0xFFFFU)) {
        return;                                   /* 没上升沿：样本无效 */
    }
    r->rise_us = (float)(a->i90 - a->i10) * CS_TS_US;
    r->os_pct  = (a->peak / ref - 1.0f) * 100.0f;
    r->settle_us = (float)(a->i_last_bad + 1U) * CS_TS_US;
    if (a->n_ss > 0U) {
        r->ss_err_pct = ((a->sum / (float)a->n_ss) / ref - 1.0f) * 100.0f;
    }
    r->valid = 1U;
}

void CurStep_Start(CurStepKind kind, float amp_a)
{
    float a = amp_a;

    if (a < 0.0f) { a = -a; }
    if (a < CS_AMP_MIN) { a = CS_AMP_MIN; }
    if (a > CS_AMP_MAX) { a = CS_AMP_MAX; }

    s_amp   = a;
    s_kind  = (uint8_t)kind;
    s_k     = 0U;
    s_tr_n  = 0U;
    acc_reset(&s_acc[0]);
    acc_reset(&s_acc[1]);
    s_res[0].valid = 0U;
    s_res[1].valid = 0U;
    s_state = CS_PRE;
}

void CurStep_Abort(void) { s_state = CS_IDLE; }
void CurStep_Clear(void) { s_state = CS_IDLE; }

uint8_t CurStep_IsActive(void) { return (s_state != CS_IDLE) ? 1U : 0U; }
uint8_t CurStep_IsDone(void)   { return (s_state == CS_DONE) ? 1U : 0U; }

uint8_t CurStep_HasAxis(uint8_t axis)
{
    if (axis == 0U) { return ((s_kind == CUR_STEP_ID)   || (s_kind == CUR_STEP_BOTH)) ? 1U : 0U; }
    return ((s_kind == CUR_STEP_IQ) || (s_kind == CUR_STEP_BOTH)) ? 1U : 0U;
}

uint8_t CurStep_Refs(float *id_ref, float *iq_ref)
{
    if (s_state == CS_IDLE) {
        /* 未接管：**绝对不能动调用方的给定**。
         *
         * 这里原来写的是 `*id_ref = 0; *iq_ref = 0; return 0;`，
         * 调用点的注释还写着"非活动时原样返回"—— 注释和行为是反的。
         * 后果：纯力矩模式（M/N 或 >iq）的给定在进电流环前被抹成 0，
         * 查询里 iq_tar=0.30 一切正常，但实测 iq 恒为 0、
         * 三相电压指令只有几十 mV 噪声 —— 看着像功率级坏了，
         * 实际是给定在软件里就被清零了。 */
        return 0U;
    }
    if ((s_state == CS_DONE) || (s_state == CS_PRE)) {
        *id_ref = 0.0f;                           /* 结束后强制回 0：等命令层封锁 */
        *iq_ref = 0.0f;
        return 1U;
    }
    *id_ref = CurStep_HasAxis(0U) ? s_amp : 0.0f;
    *iq_ref = CurStep_HasAxis(1U) ? s_amp : 0.0f;
    return 1U;
}

void CurStep_Push(float id_now, float iq_now, float vd, float vq)
{
    float vmag = sqrtf(vd * vd + vq * vq);

    if (s_state == CS_PRE) {
        if (++s_k >= CS_PRE_TICKS) { s_k = 0U; s_state = CS_STEP; }
        return;
    }
    if (s_state != CS_STEP) { return; }

    {
        uint16_t n = s_k;

        if (CurStep_HasAxis(0U)) { acc_push(&s_acc[0], s_amp, id_now, vmag, n); }
        if (CurStep_HasAxis(1U)) { acc_push(&s_acc[1], s_amp, iq_now, vmag, n); }

        if (((n % CS_DECIM) == 0U) && (s_tr_n < CS_TRACE_MAX)) {
            s_tr[s_tr_n].t_ms   = (float)n * (CS_TS_US / 1000.0f);
            s_tr[s_tr_n].id_ref = CurStep_HasAxis(0U) ? s_amp : 0.0f;
            s_tr[s_tr_n].id_now = id_now;
            s_tr[s_tr_n].iq_ref = CurStep_HasAxis(1U) ? s_amp : 0.0f;
            s_tr[s_tr_n].iq_now = iq_now;
            s_tr_n++;
        }

        if (++s_k >= CS_STEP_TICKS) {
            s_state = CS_DONE;
            finalize_axis(0U, &s_acc[0], CurStep_HasAxis(0U) ? s_amp : 0.0f);
            finalize_axis(1U, &s_acc[1], CurStep_HasAxis(1U) ? s_amp : 0.0f);
        }
    }
}

const CurStepResult *CurStep_GetResult(uint8_t axis)
{
    return (axis > 1U) ? &s_res[0] : &s_res[axis];
}

uint16_t CurStep_GetTraceN(void) { return s_tr_n; }

void CurStep_GetTracePoint(uint16_t i, float *t_ms,
                           float *id_ref, float *id_now,
                           float *iq_ref, float *iq_now)
{
    if (i >= s_tr_n) { return; }
    if (t_ms)   { *t_ms   = s_tr[i].t_ms; }
    if (id_ref) { *id_ref = s_tr[i].id_ref; }
    if (id_now) { *id_now = s_tr[i].id_now; }
    if (iq_ref) { *iq_ref = s_tr[i].iq_ref; }
    if (iq_now) { *iq_now = s_tr[i].iq_now; }
}
