/**
 * @file    vel_profile.c
 * @brief   速度模式给定层实现：速度轨迹积分成位置给定
 *
 * 关键设计点（都是踩过的坑，不要随意简化）：
 *   1) 使能瞬间 pos_ref 必须等于实际位置，否则位置环 P=50.265 会把一个阶跃
 *      误差直接变成 120 rad/s 的速度指令，表现为上电"抽一下"。
 *   2) 换模式/改限幅时保留 rg.v 与 rg.a，否则 ramp_init() 会把速度清零，
 *      在运行中切换就是一次速度突降。
 *   3) 跟随误差必须限幅：堵转或电流限幅时 v_ref 还在积分，pos_ref 会一直
 *      跑到无穷远，解除堵转后就是一个巨大的位置阶跃。限幅后 pos_ref 停在
 *      theta±err_max，电机跟上来再继续积分。
 */
#include "vel_profile.h"

static void rg_reload(vel_profile_t *vp)
{
    float v = vp->rg.v;
    float a = vp->rg.a;

    ramp_init(&vp->rg, (profile_mode_t)vp->mode, vp->ts, vp->a_max, vp->j_max);
    vp->rg.v = v;
    vp->rg.a = a;
}

void VelProfile_Init(vel_profile_t *vp, float ts)
{
    vp->ts        = (ts > 0.0f) ? ts : VEL_TS_DEFAULT;
    vp->a_max     = VEL_A_MAX_DEFAULT;
    vp->j_max     = VEL_J_MAX_DEFAULT;
    vp->err_max   = VEL_ERR_MAX_DEFAULT;
    vp->v_target  = 0.0f;
    vp->v_ref     = 0.0f;
    vp->pos_ref   = 0.0f;
    vp->err_now   = 0.0f;
    vp->mode      = (uint8_t)VEL_MODE_SCURVE;   /* 默认 S 曲线：冲击小 */
    vp->active    = 0u;
    vp->clamp_cnt = 0u;

    ramp_init(&vp->rg, (profile_mode_t)vp->mode, vp->ts, vp->a_max, vp->j_max);
}

void VelProfile_SetMode(vel_profile_t *vp, uint8_t mode)
{
    uint8_t m = (mode == (uint8_t)VEL_MODE_TRAP) ? (uint8_t)VEL_MODE_TRAP
                                                 : (uint8_t)VEL_MODE_SCURVE;
    if (m == vp->mode) {
        return;
    }
    vp->mode = m;
    rg_reload(vp);          /* 保留 v/a，运行中切换不跳变 */
}

void VelProfile_SetLimits(vel_profile_t *vp, float a_max, float j_max)
{
    if (a_max > 0.0f) {
        vp->a_max = a_max;
    }
    if (j_max > 0.0f) {
        vp->j_max = j_max;
    }
    rg_reload(vp);
}

void VelProfile_SetTarget(vel_profile_t *vp, float v_target)
{
    if (v_target > VEL_V_TARGET_MAX) {
        v_target = VEL_V_TARGET_MAX;
    } else if (v_target < -VEL_V_TARGET_MAX) {
        v_target = -VEL_V_TARGET_MAX;
    }
    vp->v_target = v_target;
}

void VelProfile_Enable(vel_profile_t *vp, float theta_now)
{
    vp->pos_ref = theta_now;      /* 坑 1：给定对齐实际位置 */
    vp->err_now = 0.0f;
    vp->rg.v    = 0.0f;           /* 斜坡从 0 起，按 a/j 限幅加速到 v_target */
    vp->rg.a    = 0.0f;
    vp->v_ref   = 0.0f;
    vp->clamp_cnt = 0u;
    vp->active  = 1u;
}

void VelProfile_Disable(vel_profile_t *vp, float theta_now)
{
    vp->active  = 0u;
    vp->rg.v    = 0.0f;
    vp->rg.a    = 0.0f;
    vp->v_ref   = 0.0f;
    vp->pos_ref = theta_now;
    vp->err_now = 0.0f;
}

float VelProfile_Step(vel_profile_t *vp, float theta_now)
{
    float e;

    if (vp->active == 0u) {
        vp->pos_ref = theta_now;      /* 待命：给定贴住实际位置，随时可无跳变使能 */
        vp->v_ref   = 0.0f;
        return vp->pos_ref;
    }

    /* 层 1：速度斜坡（梯形/S 曲线由 rg.mode 决定） */
    vp->v_ref = ramp_step(&vp->rg, vp->v_target);

    /* 速度积分成位置给定（float 足够：定位精度由外环决定，速度模式不追求绝对位置） */
    vp->pos_ref += vp->v_ref * vp->ts;

    /* 坑 3：跟随误差限幅 */
    e = vp->pos_ref - theta_now;
    vp->err_now = e;
    if (e > vp->err_max) {
        vp->pos_ref = theta_now + vp->err_max;
        vp->clamp_cnt++;
    } else if (e < -vp->err_max) {
        vp->pos_ref = theta_now - vp->err_max;
        vp->clamp_cnt++;
    }

    return vp->pos_ref;
}

float    VelProfile_GetVref(const vel_profile_t *vp)     { return vp->v_ref; }
float    VelProfile_GetTarget(const vel_profile_t *vp)   { return vp->v_target; }
float    VelProfile_GetPosRef(const vel_profile_t *vp)   { return vp->pos_ref; }
uint8_t  VelProfile_GetMode(const vel_profile_t *vp)     { return vp->mode; }
uint32_t VelProfile_GetClampCount(const vel_profile_t *vp) { return vp->clamp_cnt; }
