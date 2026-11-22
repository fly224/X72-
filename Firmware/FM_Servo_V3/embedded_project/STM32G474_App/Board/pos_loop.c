/**
 * @file    pos_loop.c
 * @brief   位置环实现：规划 + P + 速度前馈，输出速度给定
 *
 * 详细设计说明见 pos_loop.h 的文件头。这里只记实现层面的几个约束：
 *
 * 1. **Step 必须与速度环同频**（本工程 2 kHz）。规划器的 ts 就是 Step 的
 *    调用周期，两者不一致会让规划出来的速度与实际执行的速度差一个比例，
 *    表现为定位时间对不上、末端过冲。
 * 2. **目标变更后的第一拍才用实时位置作规划起点**。命令层拿到位置到 ISR
 *    真正执行之间可能隔几毫秒，用"命令时刻的位置"当起点会让规划一开始就
 *    与实际偏离，起步那拍直接顶一个 v_max 的冲击。
 * 3. **到位判据要三个条件同时满足**：规划完成 + 位置在窗口内 + 速度足够低。
 *    只看规划完成会误判 —— 摩擦大时规划走完了、电机还在后面爬。
 */
#include "pos_loop.h"
#include "motion_profile.h"

#include <math.h>
#include <string.h>

static uint8_t   s_en;
static pos_state_t s_st;

static profile_t s_pf;
static float     s_ts;
static float     s_ts_ms;

static float     s_kp;
static float     s_ki;
static float     s_i_term;        /* 位置环积分项，单位 rad/s（与速度给定同量纲） */
static float     s_vmax;
static float     s_amax;
static float     s_jmax;
static float     s_errmax;

static float     s_p_tgt;          /* 目标位置（模型坐标） */
static float     s_p_tgt_c;        /* 补偿后的规划目标（= s_p_tgt ± s_comp） */
static float     s_travel;         /* 本次定位的行程 |s_p_tgt - p_meas|，restart 时算 */
static float     s_comp     = POS_COMP_DFLT;   /* 补偿量 rad */
static uint8_t   s_comp_en  = POS_COMP_EN_DFLT;/* 默认关闭，见 pos_loop.h 的实测警告 */
static float     s_comp_min = POS_COMP_MIN;    /* 行程阈值 */
static volatile uint8_t s_restart; /* 1 = 下一拍用实时位置重设规划起点 */
static volatile uint8_t s_hold_req;

/* 诊断量 */
static float     s_p_ref;
static float     s_v_ff;
static float     s_err;
static uint32_t  s_fault_cnt;

/* 计时（毫秒，按 s_ts 累加，与调用频率解耦） */
static float     s_err_ms;
static float     s_done_ms;

void PosLoop_Init(float ts)
{
    if (!(ts > 0.0f)) { ts = 0.0005f; }

    s_ts    = ts;
    s_ts_ms = ts * 1000.0f;

    s_kp     = POS_KP_DFLT;
    s_ki     = POS_KI_DFLT;
    s_i_term = 0.0f;
    s_vmax   = POS_VMAX_DFLT;
    s_amax   = POS_AMAX_DFLT;
    s_jmax   = POS_JMAX_DFLT;
    s_errmax = POS_ERRMAX_DFLT;

    profile_init(&s_pf, PROFILE_MODE_TRAP, s_ts, s_vmax, s_amax, s_jmax);

    s_en       = 0U;
    s_st       = POS_ST_IDLE;
    s_p_tgt    = 0.0f;
    s_p_tgt_c  = 0.0f;
    s_travel   = 0.0f;
    s_restart  = 0U;
    s_hold_req = 0U;
    s_p_ref    = 0.0f;
    s_v_ff     = 0.0f;
    s_err      = 0.0f;
    s_fault_cnt = 0U;
    s_err_ms   = 0.0f;
    s_done_ms  = 0.0f;
}

void PosLoop_Enable(float p_now)
{
    s_p_tgt    = p_now;
    s_p_tgt_c  = p_now;
    s_p_ref    = p_now;      /* 诊断量立刻跟上：否则使能后第一拍之前
                              * `p=` 显示的是上一轮遗留的规划位置，
                              * 与 tgt= 差一截，看着像"没到位" */
    s_v_ff     = 0.0f;
    s_restart  = 1U;
    s_hold_req = 0U;
    s_en       = 1U;
    s_st       = POS_ST_HOLD;
    s_err_ms   = 0.0f;
    s_done_ms  = 0.0f;
    s_err      = 0.0f;
    s_i_term   = 0.0f;
}

void PosLoop_Disable(void)
{
    s_en       = 0U;
    s_st       = POS_ST_IDLE;
    s_restart  = 0U;
    s_hold_req = 0U;
    s_err      = 0.0f;
    s_v_ff     = 0.0f;
    s_err_ms   = 0.0f;
    s_done_ms  = 0.0f;
    s_i_term   = 0.0f;
}

uint8_t PosLoop_IsEnabled(void) { return s_en; }

void PosLoop_SetTarget(float p_abs)
{
    if (s_en == 0U) { return; }

    /* 目标完全没变：直接返回，**绝不能置 s_restart**。
     *
     * 为什么：上位机为了续 deadman（2 s），会每 1.35 s 重发同一目标。
     * 若每次都置 s_restart，PosLoop_Step 就会执行
     *     profile_start(p_meas, v_meas)
     * 用**实测位置与带噪声的测速**重新规划 —— 规划器于是跟着 v_meas 的
     * 噪声走，位置缓慢漂移。实测 d=20 rad 时 2 s 内漂 -9705"，
     * d=125.66 rad 时稳态偏差 -4119"（正是 20 圈测试里那个恒定偏置）。
     *
     * 分开两个阈值：
     *   · <=1e-6（重发同一目标，字符串格式化后完全相等）-> 直接忽略
     *   · >POS_I_RESET_E（真正的换目标）-> 清积分
     */
    if (fabsf(p_abs - s_p_tgt) <= 1e-6f) {
        return;
    }

    if (fabsf(p_abs - s_p_tgt) > POS_I_RESET_E) {
        s_i_term = 0.0f;
    }

    s_p_tgt   = p_abs;
    s_p_tgt_c = p_abs;      /* restart 那一拍按行程决定是否加补偿 */
    s_restart = 1U;         /* 起点在下一拍用实时位置取，避免"命令时刻"偏差 */
    s_st      = POS_ST_MOVE;
    s_err_ms  = 0.0f;
    s_done_ms = 0.0f;
}

void PosLoop_Hold(void)
{
    if (s_en == 0U) { return; }

    s_i_term   = 0.0f;      /* 改回保持 = 放弃原来的定位意图，积分无意义 */
    s_hold_req = 1U;
    s_restart  = 1U;
    s_st       = POS_ST_HOLD;
    s_err_ms   = 0.0f;
    s_done_ms  = 0.0f;
}

float PosLoop_Step(float p_meas, float v_meas)
{
    float p_ref, v_ff, e, v_cmd;

    if (s_en == 0U) { return 0.0f; }

    /* ---- 目标变更：用实时位置重设规划起点 ---- */
    if (s_hold_req != 0U) {
        s_hold_req = 0U;
        s_p_tgt    = p_meas;
    }
    if (s_restart != 0U) {
        s_restart = 0U;
        s_travel  = fabsf(s_p_tgt - p_meas);

        /* 目标补偿：长行程的"少走量"是个稳定常数（见 pos_loop.h 的标定表）。
         * 加在**规划目标**上，不改 s_p_tgt —— 对外回显与用户指令保持一致。 */
        s_p_tgt_c = s_p_tgt;
        if ((s_comp_en != 0U) && (s_comp > 0.0f) && (s_travel > s_comp_min)) {
            s_p_tgt_c += (s_p_tgt > p_meas) ? s_comp : -s_comp;
        }

        profile_start(&s_pf, (double)p_meas, v_meas);
        profile_set_target(&s_pf, (double)s_p_tgt_c);
        s_err_ms  = 0.0f;
        s_done_ms = 0.0f;
        if (fabsf(s_p_tgt_c - p_meas) < POS_WIN_DFLT) {
            s_st = POS_ST_HOLD;
        } else {
            s_st = POS_ST_MOVE;
        }
    }

    /* ---- 规划推进 ---- */
    p_ref = (float)profile_step(&s_pf);
    v_ff  = s_pf.v;
    e     = p_ref - p_meas;

    s_p_ref = p_ref;
    s_v_ff  = v_ff;
    s_err   = e;

    /* ---- 跟随误差保护 ----
     * 规划跑到前面太远 = 堵转 / 编码器丢步 / 负载远超能力。
     * 判据用"规划与实际之差"，不用"目标与实际之差"：后者在刚起步时
     * 天然就很大（目标可能在 3 rad 外），会把正常定位误判成故障。 */
    if (fabsf(e) > s_errmax) {
        s_err_ms += s_ts_ms;
        if (s_err_ms > (float)POS_ERR_HOLD_MS) {
            if (s_st != POS_ST_ERR) {
                s_st = POS_ST_ERR;
                s_fault_cnt++;
            }
            s_i_term = 0.0f;    /* 故障态清积分：恢复后不要带着饱和量冲一下 */
            return 0.0f;    /* 故障态不再给速度，由调用方封锁 */
        }
    } else {
        s_err_ms = 0.0f;
    }

    /* ---- 到位判定 ----
     *
     * 判据 = 规划完成 + 位置在窗口内 + 连续维持 POS_DONE_MS。
     *
     * ⚠ 2026-11-21：**去掉了"实测速度在窗口内"这一项**。原判据是
     *   `|v_meas| < POS_VWIN_DFLT(0.5 rad/s)`，但静止测速（编码器差分）
     *   的噪声尖峰能到 20 rad/s —— 实测 20 拍里 19 拍超 0.5，所以位置
     *   明明停稳了也判不成立，st 永远停在 MOVE，上位机等不到"到位"。
     *
     *   速度判据原本只想防"路过目标被误判到位"，但这件事用时间窗更稳：
     *   穿越时 |e| 会在几毫秒内涨出 10 mrad 窗口，不可能连续维持 100 ms。
     *   反之 100 ms 内 |e| 一直在窗口内，就一定是静止（0.1 rad/s 以下的
     *   爬行，判到位也无害）。测速这个量在这里不可信，不该进判据。
     */
    if ((s_pf.status == PROFILE_DONE) && (fabsf(e) < POS_WIN_DFLT)) {
        s_done_ms += s_ts_ms;
        if (s_done_ms >= (float)POS_DONE_MS) {
            s_st = POS_ST_DONE;
        }
    } else {
        s_done_ms = 0.0f;
        if (s_st == POS_ST_DONE) {
            /* 被外力推走：回到 MOVE 状态，P 环继续拉回来 */
            s_st = POS_ST_MOVE;
        }
    }

    /* ---- P + I + 前馈 ----
     *
     * 为什么必须加 I：纯 P 的输出是"速度给定"，而停止后要靠它推动电机微调。
     * 该给定小于静摩擦对应值时电机完全不动 —— 实测死区位约 0.01 rad
     * （= I_fric/(Kp_spd·Kp_pos) = 0.15/(0.583×25)），
     * 于是"停在哪儿"由摩擦齿合的随机性决定，重复性 σ 卡在 18~21"。
     * 积分把这个死区磨掉：只要还有误差，积分就会继续累积到最后推得动。
     *
     * 抗饱和用条件积分 + 回灌（与速度环/电流环同一套）：
     *   · 未饱和，或误差与输出反向（该退饱和了）→ 正常累积；
     *   · 饱和且误差同向 → 按 0.2 把超出量回灌，避免积分继续涨。
     * 不这么做的后果是：大行程起步时积分在饱和区狂涨，到速后要很久才退出来，
     * 表现为"每次到位都过冲一下再慢慢爬回来"。
     */
    {
        float v_raw = v_ff + (s_kp * e) + s_i_term;
        float v_sat = v_raw;

        if (v_sat >  s_vmax) { v_sat =  s_vmax; }
        if (v_sat < -s_vmax) { v_sat = -s_vmax; }

        /* ⚠ Ki = 0 时必须让积分恒为 0，**连抗饱和回灌也不能执行**。
         *
         * 2026-11-18 实测到的坑：原代码把回灌放在 if/else 的 else 里，
         * 而那个 else 与 s_ki 无关 —— 于是 Ki=0 时只要输出饱和，
         * s_i_term 就会被 -0.2*(v_raw-v_sat) 一路推到 -POS_I_MAX(-2.0)，
         * 而 Ki=0 又无法把它积分回去，只能等下一次**反向**饱和才推到 +2.0。
         *
         * 后果：速度给定里长期挂着一个 ±2 rad/s 的伪偏置，
         *       位置偏差 = 2.0/Kp。Kp=100 时正好 0.020 rad = 4124"，
         *       与 20 圈重复定位测到的 -4120" 恒定偏差精确吻合。
         *       （10 rad 行程实测：iv 钉在 -2.000，稳态误差 -0.019~-0.021 rad）
         *
         * 这也解释了"目标补偿 0.01998 rad"为什么看起来能修好均值 ——
         * 它抵消的是这个伪偏置，而不是什么齿槽势阱。补偿一开，
         * 目标被推到"势阱壁"上的说法也就不成立了。
         */
        if (s_ki > 0.0f) {
            if ((v_sat == v_raw) || ((e * v_raw) < 0.0f)) {
                s_i_term += (s_ki * e * s_ts);
            } else {
                s_i_term -= 0.2f * (v_raw - v_sat);
            }
            if (s_i_term >  POS_I_MAX) { s_i_term =  POS_I_MAX; }
            if (s_i_term < -POS_I_MAX) { s_i_term = -POS_I_MAX; }
        } else {
            s_i_term = 0.0f;
        }

        v_cmd = v_sat;
    }

    /* ---- 末端软着陆 ----
     * 用"目标 − 实测"（剩余距离）而不是 e（规划 − 实测）：
     * 匀速段 e 恒等于 Vmax/Kp ≈ 1 rad，永远进不了窗口，第一版就是这么失效的。 */
    {
        const float e_rem = s_p_tgt_c - p_meas;

        if (fabsf(e_rem) < POS_SLOW_WIN) {
            float vlim = POS_SLOW_VMAX * (fabsf(e_rem) / POS_SLOW_WIN);
            if (vlim < POS_SLOW_VMIN) { vlim = POS_SLOW_VMIN; }
            if (v_cmd >  vlim) { v_cmd =  vlim; }
            if (v_cmd < -vlim) { v_cmd = -vlim; }
        }
    }

    return v_cmd;
}

/* ---------------- 参数 ---------------- */

/* Kp 的下限不能是常数（2026-11-21 审计修）。
 *
 * 纯 P 环的匀速段跟随误差 = v_ff/Kp ≈ Vmax/Kp。若 Kp < Vmax/errmax，
 * 规划一进入匀速段误差就必然越过 errmax，持续 POS_ERR_HOLD_MS 之后落
 * FOLLOW 故障 —— 电机停在半路，现象像"堵转"，其实是参数不可能成立。
 *
 * 旧实现允许 Kp 低到 1，而 Vmax/errmax = 25/6 ≈ 4.2，于是 Kp=1~4 是
 * "一跑就故障"的死区。这里留一倍裕度要求
 *       Kp ≥ 2·Vmax/errmax
 * 当前默认（Vmax=25, errmax=6）→ 下限 8.33。Kp 改小会被抬到下限，
 * 改 Vmax / errmax 之后也会重新夹一次 s_kp，避免"先设 Kp 再改 errmax"
 * 绕过下限。 */
static float kp_floor(void)
{
    const float vm = (s_vmax   > 0.5f) ? s_vmax   : POS_VMAX_DFLT;
    const float em = (s_errmax > 0.1f) ? s_errmax : POS_ERRMAX_DFLT;
    float k = 2.0f * vm / em;

    if (!(k > 1.0f)) { k = 1.0f; }      /* 同时挡 NaN */
    if (k > 200.0f)  { k = 200.0f; }
    return k;
}

void PosLoop_SetKp(float kp)
{
    const float lo = kp_floor();

    if (!(kp > lo)) { kp = lo; }        /* 同时挡 NaN */
    if (kp > 200.0f) { kp = 200.0f; }
    s_kp = kp;
}

void PosLoop_SetLimits(float vmax, float amax, float jmax)
{
    if (vmax < 0.5f)    { vmax = 0.5f; }
    if (vmax > 120.0f)  { vmax = 120.0f; }
    if (amax < 10.0f)   { amax = 10.0f; }
    if (amax > 5000.0f) { amax = 5000.0f; }
    if (jmax < 100.0f)  { jmax = 100.0f; }

    s_vmax = vmax;
    s_amax = amax;
    s_jmax = jmax;

    /* vmax 变大可能把 Kp 的下限抬上去，这里重新夹一次 */
    if (s_kp < kp_floor()) { s_kp = kp_floor(); }

    /* 限幅改了必须重建规划器：v_max/a_max/j_max 存在 profile_t 内部 */
    profile_init(&s_pf, s_pf.mode, s_ts, s_vmax, s_amax, s_jmax);
    profile_start(&s_pf, (double)s_p_ref, 0.0f);
    profile_set_target(&s_pf, (double)s_p_tgt);
}

void PosLoop_SetErrMax(float e)
{
    if (e < 0.1f)  { e = 0.1f; }
    if (e > 50.0f) { e = 50.0f; }
    s_errmax = e;

    /* errmax 改小会把 Kp 下限抬上去（同 SetLimits 的理由） */
    if (s_kp < kp_floor()) { s_kp = kp_floor(); }
}

void PosLoop_ClearFault(void)
{
    s_err_ms  = 0.0f;
    s_done_ms = 0.0f;
    s_i_term  = 0.0f;

    /* 故障清除后**不自动接着追原目标**：那个目标正是把误差拉大的原因
     * （堵转、负载卡死、编码器跳变），直接续追会立刻二次触发。
     * 改成"停在当前位置"，由用户重新下目标。 */
    if (s_st == POS_ST_ERR) {
        s_hold_req = 1U;
        s_restart  = 1U;
        s_st       = POS_ST_HOLD;
    }
}

/* ---------------- 诊断 ---------------- */

float    PosLoop_GetKp(void)      { return s_kp; }
float    PosLoop_GetVmax(void)    { return s_vmax; }
float    PosLoop_GetErrMax(void)  { return s_errmax; }
float    PosLoop_GetITerm(void)   { return s_i_term; }
float    PosLoop_GetTargetComp(void) { return s_p_tgt_c; }
float    PosLoop_GetTravel(void)     { return s_travel; }

/* ---------------- 目标补偿参数（对外接口） ---------------- */
void PosLoop_SetComp(float rad)
{
    if (!(rad > 0.0f)) { rad = 0.0f; }        /* 同时挡 NaN */
    if (rad > POS_COMP_MAX) { rad = POS_COMP_MAX; }
    s_comp = rad;
}
float PosLoop_GetComp(void)     { return s_comp; }

void PosLoop_SetCompEn(uint8_t en) { s_comp_en = (en != 0U) ? 1U : 0U; }
uint8_t PosLoop_GetCompEn(void)    { return s_comp_en; }

void PosLoop_SetCompMin(float rad)
{
    if (rad < 0.0f)  { rad = 0.0f; }
    if (rad > 1000.0f) { rad = 1000.0f; }
    s_comp_min = rad;
}
float PosLoop_GetCompMin(void)  { return s_comp_min; }
float    PosLoop_GetTarget(void)  { return s_p_tgt; }
float    PosLoop_GetRef(void)     { return s_p_ref; }
float    PosLoop_GetVff(void)     { return s_v_ff; }
float    PosLoop_GetErr(void)     { return s_err; }
uint8_t  PosLoop_GetState(void)   { return (uint8_t)s_st; }
uint8_t  PosLoop_IsSettled(void)  { return (s_st == POS_ST_DONE) ? 1U : 0U; }
uint32_t PosLoop_GetFaultCnt(void){ return s_fault_cnt; }
