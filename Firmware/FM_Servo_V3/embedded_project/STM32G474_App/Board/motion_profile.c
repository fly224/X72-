/**
 * @file    motion_profile.c
 * @brief   速度斜坡（梯形 / S 曲线）+ 位置限速层实现
 *
 * 移植说明（2026-11-15）：
 *   本文件由 X72 轨迹规划验证工程 motion_profile.c 原样拷入（sha1 一致），
 *   逻辑一字未改，因此那份 ramp_trap.csv / ramp_scurve.csv 的数值结论仍然成立。
 *   仅补充本注释。速度模式只用到层 1 的 ramp_step()，层 2 留给将来的位置模式。
 *   移植依赖：只用到 fabsf/sqrtf（<math.h>），无动态分配、无 printf、无全局状态。
 */
#include "motion_profile.h"
#include <math.h>

#define PF_EPS 1e-6f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ==================== 层1：速度斜坡发生器 ==================== */

void ramp_init(ramp_t *rg, profile_mode_t mode, float ts, float a_max, float j_max)
{
    rg->mode  = mode;
    rg->ts    = ts;
    rg->a_max = a_max;
    rg->j_max = (mode == PROFILE_MODE_TRAP) ? 1.0e9f : j_max;
    rg->v     = 0.0f;
    rg->a     = 0.0f;
}

/**
 * 速度斜坡：把 v 平滑地拉到 v_target
 * 梯形  : 加速度限幅（a 阶跃）—— v 每周期变化 ±a_max*ts
 * S曲线 : 加速度以 |jerk| <= j_max 追随 ±a_max，且提前收 a 避免速度超调
 *
 * 提前收的量：以 -sign(a) 把 a 收到 0 时速度还会变化 a*|a|/(2j)，所以当
 * 剩余速度差 |v_target-v| <= a^2/(2j) 时就该把 a 归零。
 */
float ramp_step(ramp_t *rg, float v_target)
{
    const float ts = rg->ts;
    float dv = v_target - rg->v;
    float dv_max = rg->a_max * ts;

    if (rg->mode == PROFILE_MODE_TRAP) {
        if (dv > dv_max) {
            rg->v += dv_max;
            rg->a  = rg->a_max;
        } else if (dv < -dv_max) {
            rg->v -= dv_max;
            rg->a  = -rg->a_max;
        } else {
            rg->a  = dv / ts;          /* 本周期正好到位，a 取实际值 */
            rg->v  = v_target;
        }
        return rg->v;
    }

    /* ---- S 曲线 ---- */
    {
        float dv_coast = rg->a * fabsf(rg->a) / (2.0f * rg->j_max); /* 有符号 */
        float a_des;

        /* 量化死区：剩余速度差小于"一个周期 jerk 能改变的速度量"时直接到位。
           否则 a 会在 ±j*ts 之间来回，形成 a=±1 的极限环（巡航段尤其明显） */
        if (fabsf(dv) <= rg->j_max * ts * ts) {
            rg->v = v_target;
            rg->a = 0.0f;
            return rg->v;
        }

        if (dv > PF_EPS) {
            a_des = (dv <= dv_coast) ? 0.0f : rg->a_max;
        } else if (dv < -PF_EPS) {
            a_des = (-dv <= -dv_coast) ? 0.0f : -rg->a_max;
        } else {
            a_des = 0.0f;
        }

        rg->a = clampf(rg->a + clampf(a_des - rg->a,
                                      -rg->j_max * ts, rg->j_max * ts),
                       -rg->a_max, rg->a_max);
        rg->v += rg->a * ts;

        /* 零速捕获：目标为 0 且残余速度已小于"一个周期 jerk 能修掉的量"时
           直接钉死，否则会在 0 附近形成 a²/(2j) 量级的极限环
           （表现为末端 ±0.005 的来回抖动、位置永远差最后一点） */
        if (v_target == 0.0f && fabsf(rg->v) < 4.0f * rg->j_max * ts * ts) {
            rg->v = 0.0f;
            rg->a = 0.0f;
        }
    }
    return rg->v;
}

/* ==================== 层2：位置限速 ==================== */

void profile_init(profile_t *pf, profile_mode_t mode, float ts,
                  float v_max, float a_max, float j_max)
{
    pf->mode   = mode;
    pf->ts     = ts;
    pf->v_max  = v_max;
    pf->a_max  = a_max;
    pf->j_max  = (mode == PROFILE_MODE_TRAP) ? 1.0e9f : j_max;
    pf->p      = 0.0f;
    pf->p_target = 0.0f;
    pf->v      = 0.0f;
    pf->a      = 0.0f;
    pf->v_lim  = 0.0f;
    pf->status = PROFILE_DONE;
    ramp_init(&pf->rg, pf->mode, ts, a_max, pf->j_max);
}

void profile_start(profile_t *pf, double p_start, float v_start)
{
    pf->p = p_start;
    pf->v = v_start;
    pf->a = 0.0f;
    pf->rg.v = v_start;
    pf->rg.a = 0.0f;
    pf->status = PROFILE_RUN;
}

void profile_set_target(profile_t *pf, double p_target)
{
    pf->p_target = p_target;
    pf->status   = PROFILE_RUN;
}

/**
 * 从 v 减速到 v_end 所需距离（a 从 0 开始的最优 jerk 限幅减速）
 * 梯形  : d = (v^2-v_end^2)/(2a)
 * S曲线 : 加速度曲线关于减速中点反对称 => 平均速度=(v+v_end)/2
 *         dv >= a^2/j -> d = (v+v_end)/2 * (dv/a + a/j)
 *         dv <  a^2/j -> d = (v+v_end) * sqrt(dv/j)
 */
float profile_decel_distance(const profile_t *pf, float v, float v_end)
{
    float dv, a, j;

    if (v < v_end) v = v_end;
    dv = v - v_end;
    if (dv < PF_EPS) return 0.0f;

    a = pf->a_max;
    j = pf->j_max;

    if (pf->mode == PROFILE_MODE_TRAP || j <= PF_EPS) {
        return dv * dv / (2.0f * a);
    }
    if (dv >= a * a / j) {
        float tj = a / j;
        return 0.5f * (v + v_end) * (dv / a + tj);
    } else {
        return (v + v_end) * sqrtf(dv / j);
    }
}

/** 反解：s_abs 内、终速 v_end 时允许的最大速度（decel_distance 对 v 单调） */
float profile_solve_v(const profile_t *pf, float s_abs, float v_end)
{
    float lo, hi, mid;
    int   i;

    if (s_abs <= 0.0f) return v_end;

    if (pf->mode == PROFILE_MODE_TRAP) {
        return sqrtf(v_end * v_end + 2.0f * pf->a_max * s_abs);
    }
    lo = v_end;
    hi = v_end + 2.0f * pf->v_max + 1.0f;
    for (i = 0; i < 8; i++) {
        mid = 0.5f * (lo + hi);
        if (profile_decel_distance(pf, mid, v_end) < s_abs) lo = mid;
        else                                                hi = mid;
    }
    return 0.5f * (lo + hi);
}

double profile_step(profile_t *pf)
{
    float  ts, dir, s_eff, v_abs_tgt, v_tgt;
    double s_rem, s_abs, s_left;

    if (pf->status != PROFILE_RUN) return pf->p;

    ts    = pf->ts;
    s_rem = pf->p_target - pf->p;
    dir   = (s_rem >= 0.0) ? 1.0f : -1.0f;
    s_abs = fabs(s_rem);

    /* S 曲线：profile_decel_distance() 假设"a 从 0 开始减速"，
       若当前 a != 0，需要额外预留把 a 收到 0 的那段时间走过的距离：
       s_res = v * |a| / j   （a = 0 时为 0，不会重复扣减） */
    s_eff = (float)s_abs;
    if (pf->mode == PROFILE_MODE_SCURVE) {
        float s_res = fabsf(pf->v) * fabsf(pf->a) / pf->j_max;
        s_eff = (float)s_abs - s_res;
        if (s_eff < 0.0f) s_eff = 0.0f;
    }

    /* 距离限速 + 到位限速 */
    v_abs_tgt = profile_solve_v(pf, s_eff, 0.0f);
    if (v_abs_tgt > pf->v_max) v_abs_tgt = pf->v_max;
    /* 到位限速：每周期只走剩余距离的 1/K，指数收敛且天然不过冲；
       末端残差（< 一个 jerk 时间的位移）交由位置环消化 */
    {
        float v_pos = (float)s_abs / (4.0f * ts);
        if (v_abs_tgt > v_pos) v_abs_tgt = v_pos;
    }
    if (v_abs_tgt < 0.0f) v_abs_tgt = 0.0f;
    pf->v_lim = v_abs_tgt;

    /* 带符号目标速度交给斜坡跟踪（方向翻转时斜坡自然过零再反向） */
    v_tgt = dir * v_abs_tgt;
    pf->v = ramp_step(&pf->rg, v_tgt);
    pf->a = pf->rg.a;

    pf->p += (double)pf->v * (double)ts;

    /* 终点吸附：沿目标方向、本周期位移已覆盖剩余距离，或残差已小于 1 个周期位移 */
    s_left = fabs(pf->p_target - pf->p);
    if ((pf->v * dir >= 0.0f && s_left <= fabs((double)pf->v * ts) + 1e-12) ||
        s_left <= 0.02 * (double)pf->v_max * ts) {
        pf->p = pf->p_target;
        pf->v = 0.0f;
        pf->a = 0.0f;
        pf->rg.v = 0.0f;
        pf->rg.a = 0.0f;
        pf->v_lim = 0.0f;
        pf->status = PROFILE_DONE;
    }

    return pf->p;
}
