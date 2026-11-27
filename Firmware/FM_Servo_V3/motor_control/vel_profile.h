/**
 * @file    vel_profile.h
 * @brief   速度模式给定层：梯形 / S 曲线速度轨迹 -> 位置给定（供三环模型跟踪）
 *
 * ============================ 为什么要这一层 ============================
 * 生成代码 three_loop_deploy 的输入只有 pos_ref / en，没有 omega_ref：
 *   pos_ref -(P=50.265, 1kHz)-> w_cmd -(PI, 2kHz)-> iq_ref -(20kHz)-> duty
 * 所以"速度模式"不能直接改模型输入，只能在 Board 层把速度轨迹积分成位置给定：
 *
 *   v_target --ramp_step()--> v_ref --积分 (v_ref*ts)--> pos_ref --> 模型位置环
 *
 * 稳态关系（推导见说明文档）：
 *   位置环 P 控制 + 速度环 PI 无静差  =>  实际转速 = v_ref（无静差）
 *   但存在固定位置滞后  e = v_ref / Kp = v_ref / 50.265  （20 rad/s 时约 0.40 rad）
 *   该滞后对"速度"给定无影响，只影响绝对位置；因此 err_max 必须大于 v_max/Kp。
 *
 * 单位全链路一致：pos(rad, 机械角), v(rad/s), a(rad/s^2), j(rad/s^3), ts(s)
 * 与 three_loop_deploy 的 theta_m / omega_m 同一坐标系，不要混入电角度。
 * ======================================================================
 */
#ifndef VEL_PROFILE_H
#define VEL_PROFILE_H

#include <stdint.h>
#include "motion_profile.h"

/* ---------------- 默认参数（14 对极关节，48V/1mOhm/9A 限流） ---------------- */
#define VEL_TS_DEFAULT        (0.001f)    /* 1 kHz：与模型位置环同频 */
#define VEL_A_MAX_DEFAULT     (200.0f)    /* rad/s^2 机械角 */
#define VEL_J_MAX_DEFAULT     (2000.0f)   /* rad/s^3 机械角 */
#define VEL_ERR_MAX_DEFAULT   (1.0f)      /* rad，跟随误差上限（防堵转跑飞） */
#define VEL_V_TARGET_MAX      (100.0f)    /* rad/s，指令硬限幅；模型内部 w 限 ±120 */

typedef enum {
    VEL_MODE_TRAP   = PROFILE_MODE_TRAP,     /* 梯形：a 阶跃，v-t 折线 */
    VEL_MODE_SCURVE = PROFILE_MODE_SCURVE    /* S 曲线：jerk 有界，a 梯形 */
} vel_mode_t;

typedef struct {
    ramp_t  rg;          /* 层 1 斜坡发生器（参考实现） */
    float   ts;          /* 采样周期 s */
    float   a_max;       /* rad/s^2 */
    float   j_max;       /* rad/s^3，梯形模式内部按无穷大处理 */
    float   v_target;    /* 速度指令 rad/s（已限幅） */
    float   v_ref;       /* 斜坡输出 rad/s */
    float   pos_ref;     /* 积分得到的位置给定 rad */
    float   err_max;     /* 位置跟随误差上限 rad */
    float   err_now;     /* 诊断：本拍 位置给定-实际位置 */
    uint8_t mode;        /* vel_mode_t */
    uint8_t active;      /* 1=运行，0=待命（给定贴住实际位置） */
    uint32_t clamp_cnt;  /* 跟随误差被限幅的拍数，持续增长说明堵转/限流 */
} vel_profile_t;

/* 初始化（内部按 ts 建斜坡，默认 S 曲线 + 默认限幅，速度目标 0） */
void  VelProfile_Init(vel_profile_t *vp, float ts);

/* 切换梯形/S曲线：保留当前 v、a，不会产生速度跳变 */
void  VelProfile_SetMode(vel_profile_t *vp, uint8_t mode);

/* 改加速度/jerk 限幅：同样保留当前 v、a */
void  VelProfile_SetLimits(vel_profile_t *vp, float a_max, float j_max);

/* 下发速度指令（rad/s 机械角，内部限幅到 ±VEL_V_TARGET_MAX） */
void  VelProfile_SetTarget(vel_profile_t *vp, float v_target);

/* 进入速度模式：位置给定对齐到当前实际位置，斜坡从 0 起（不会跳变） */
void  VelProfile_Enable(vel_profile_t *vp, float theta_now);

/* 退出速度模式：清斜坡，给定贴住实际位置 */
void  VelProfile_Disable(vel_profile_t *vp, float theta_now);

/*
 * 每 ts 调用一次（本工程在 TIM6 的 1 ms 任务里）。
 * 返回位置给定；未使能时返回 theta_now。
 */
float VelProfile_Step(vel_profile_t *vp, float theta_now);

float    VelProfile_GetVref(const vel_profile_t *vp);
float    VelProfile_GetTarget(const vel_profile_t *vp);
float    VelProfile_GetPosRef(const vel_profile_t *vp);
uint8_t  VelProfile_GetMode(const vel_profile_t *vp);
uint32_t VelProfile_GetClampCount(const vel_profile_t *vp);

#endif /* VEL_PROFILE_H */
