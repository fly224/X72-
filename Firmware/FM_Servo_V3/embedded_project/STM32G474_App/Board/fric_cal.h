/**
  ******************************************************************************
  * @file    fric_cal.h
  * @brief   速度环摩擦两点标定（空载）
  ******************************************************************************
  * 为什么要单独标定：摩擦前馈的幅度必须实测，拍脑袋填会两头不讨好 ——
  *   填小了起不到解决静摩擦起跳的作用（3 rpm 超调 68.8% 就是它造成的）；
  *   填大了低速稳态会反向补偿（积分要往反方向拉回来），表现为转速小幅爬行。
  *
  * 原理：空载稳态时速度环输出 iq 全部用于克服摩擦（惯性项为 0，加速度为 0），
  *   所以取两个转速点的稳态 iq：
  *     iq1 @ w1（3 rpm，黏滞可忽略）→ 等效库仑摩擦电流
  *     iq2 @ w2（100 rpm）           → 加上黏滞项
  *   线性插值出黏滞系数  (iq2-iq1)/(w2-w1)
  *
  * ⚠ 前馈量**直接用电流**，不经过 Kt 换算：
  *   模型 psi=7.593 mWb 与实测 4.455 mWb 差 41%，用 Kt 换算会把误差带进前馈。
  *   标定测到的就是"需要多少安培"，直接拿来用最稳。Tc(N·m) 只作报告用。
  *
  * 标定全程约 4 s，期间必须关前馈（否则测到的是"含前馈"的 iq，
  * 会把前馈量算进摩擦，形成正反馈）。
  ******************************************************************************
  */

#ifndef __FRIC_CAL_H
#define __FRIC_CAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    FC_IDLE = 0,
    FC_SETTLE1,      /* 3 rpm 稳定中 */
    FC_AVG1,         /* 3 rpm 采样平均 */
    FC_SETTLE2,      /* 100 rpm 稳定中 */
    FC_AVG2,
    FC_DONE,
    FC_FAIL
} FricCalState;

typedef enum {
    FC_OK = 0,
    FC_ERR_BUSY,
    FC_ERR_CAL,      /* 电流零点未标定 */
    FC_ERR_ZERO,     /* 电角度未对齐 */
    FC_ERR_FAULT,    /* 有故障位 */
    FC_ERR_MOTION,   /* 转不起来 / 速度跟不上 */
    FC_ERR_SPREAD    /* 采样抖动过大，结果不可信 */
} FricCalErr;

typedef struct {
    float iq1;       /* 低速工作点稳态 iq（A）—— 前馈幅值直接用它 */
    float iq2;       /* 高速工作点稳态 iq（A） */
    float w1;        /* rad/s */
    float w2;
    float visc;      /* (iq2-iq1)/(w2-w1)，A/(rad/s) */
    float tc_nm;     /* 等效库仑摩擦 N·m（按实测 Kt=0.0936 换算，仅供参考） */
    float rpm1;      /* 实测转速，用于判断"是否真的转起来了" */
    float rpm2;
    uint8_t err;
} FricCalResult;

FricCalErr          FricCal_Start(void);
void                FricCal_Abort(void);
FricCalState        FricCal_GetState(void);
uint8_t             FricCal_IsBusy(void);
const FricCalResult *FricCal_GetResult(void);
void                FricCal_Task1ms(void);
void                FricCal_Init(void);
const char         *FricCal_ErrStr(FricCalErr e);

/* 两个工作点的转速（rpm）。命令层拼回显要用它——
 * 原来 servo_cmd.c 把 "3rpm/100rpm" 硬编码在字符串里，
 * 而 2026-11-21 已把低速点改到 15 rpm，回显就与实际不符了。
 * 回显必须读真值，不能另写一份。 */
float               FricCal_GetW1Rpm(void);
float               FricCal_GetW2Rpm(void);

#ifdef __cplusplus
}
#endif

#endif /* __FRIC_CAL_H */
