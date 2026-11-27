/**
 * @file    motion_profile.h
 * @brief   伺服轨迹规划：速度斜坡发生器（梯形 / S 曲线）+ 位置限速层
 *
 * ============================ 两层结构 ============================
 *  层2  位置限速 profile_step()  : 剩余距离 -> v_target（只在定位时用）
 *  层1  速度斜坡 ramp_step()     : v_target -> v_ref（带 a / jerk 限幅）
 *                                  ↑ 速度控制模式只用这一层
 *  梯形  : 只有加速度限幅       -> v-t 是折线，a-t 是方波
 *  S曲线 : 加速度 + jerk 限幅   -> a-t 是梯形，jerk 有界，冲击小
 * ==================================================================
 *
 * 单位必须全链路一致：p(rad), v(rad/s), a(rad/s^2), j(rad/s^3), ts(s)
 * 用 rpm 时先换算：rad/s = rpm * 2*pi/60
 */
#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>

typedef enum {
    PROFILE_MODE_TRAP   = 0,  /* 梯形 */
    PROFILE_MODE_SCURVE = 1   /* S 曲线 */
} profile_mode_t;

typedef enum {
    PROFILE_DONE = 0,
    PROFILE_RUN  = 1
} profile_status_t;

/* ---------------- 层1：速度斜坡发生器（速度控制模式直接用） ---------------- */
typedef struct {
    profile_mode_t mode;
    float ts;
    float a_max;
    float j_max;    /* 梯形模式内部置极大值 */
    float v;        /* 当前速度给定 */
    float a;        /* 当前加速度给定（可作电流前馈） */
} ramp_t;

void  ramp_init(ramp_t *rg, profile_mode_t mode, float ts, float a_max, float j_max);
float ramp_step(ramp_t *rg, float v_target);   /* 每周期调用，返回 v_ref */

/* ---------------- 层2：位置限速 + 位置积分 ---------------- */
typedef struct {
    profile_mode_t mode;
    float ts;
    float v_max;
    float a_max;
    float j_max;

    /* 位置必须用 double 或 int64 计数累加：
       float 在 p=1e2 量级时 ulp 已约 7.6e-6，而 v*Ts 可能只有 5e-7，
       p += v*Ts 会被浮点吃掉 -> 末端"永远走不完"。 */
    double p_target;
    double p;
    float v;        /* 当前速度给定 */
    float a;        /* 当前加速度给定 */
    float v_lim;    /* 本周期距离限速值（诊断用） */
    ramp_t rg;
    profile_status_t status;
} profile_t;

void  profile_init(profile_t *pf, profile_mode_t mode, float ts,
                   float v_max, float a_max, float j_max);
void  profile_start(profile_t *pf, double p_start, float v_start);
void  profile_set_target(profile_t *pf, double p_target);
double profile_step(profile_t *pf);

/* 减速到 v_end 所需距离；反解：该距离内允许的最大速度 */
float profile_decel_distance(const profile_t *pf, float v, float v_end);
float profile_solve_v(const profile_t *pf, float s_abs, float v_end);

#endif
