/*
 * cur_step.h —— 电流环阶跃激励 + 片内指标测量（调 PI 用）
 *
 * 为什么不用 100 Hz 遥测看阶跃：
 *   电流环带宽 ~1 kHz，10%-90% 上升只有 250 us。100 Hz 遥测 10 ms 一帧，
 *   一次阶跃只能采到 1~2 个点，看到的是噪声不是响应。
 *   所以激励、采样、指标计算全部放在 20 kHz 控制节拍里，串口只传结果与抽取波形。
 *
 * 时序（一拍 = 50 us）：
 *   PRE  60 拍 = 3 ms   两轴给定 0，建立基线
 *   STEP 600 拍 = 30 ms 给定 = amp
 *   结束 -> DONE，给定强制回 0，由命令层封锁 MOE 并打印结果
 *
 * 用法（必须在同一个控制拍内按顺序调用）：
 *   if (CurStep_IsActive()) { CurStep_Refs(&id_ref, &iq_ref); }   // 先取给定
 *   ... Park / PI / 限幅 ...
 *   if (CurStep_IsActive()) { CurStep_Push(id_now, iq_now, vd, vq); }  // 再记实测
 *
 * 指标（每轴独立）：上升时间 10%-90%、超调、调节时间(±5%)、稳态误差、峰值。
 * 波形：每 20 拍存一点，共 32 点（1 ms 分辨率，只看形状）。
 */
#ifndef CUR_STEP_H
#define CUR_STEP_H

#include <stdint.h>

typedef enum {
    CUR_STEP_IQ   = 0,   /* 只打 iq：有力矩，转子会加速 */
    CUR_STEP_ID   = 1,   /* 只打 id：表贴式无转矩，转子不动，最适合整定 */
    CUR_STEP_BOTH = 2    /* 两轴同时 */
} CurStepKind;

typedef struct {
    float amp;          /* 给定幅值 [A] */
    float rise_us;      /* 10% -> 90% */
    float os_pct;       /* 超调 [%] */
    float settle_us;    /* 进入 ±5% 且不再离开 */
    float ss_err_pct;   /* 稳态误差 [%]（末 20% 平均） */
    float peak;         /* 峰值 [A] */
    float vpeak;        /* 峰值 |v| [V] */
    uint8_t valid;      /* 1 = 幅值足够、有上升沿 */
} CurStepResult;

void    CurStep_Start(CurStepKind kind, float amp_a);
void    CurStep_Abort(void);
void    CurStep_Clear(void);          /* 结果读走后调用，回到 IDLE */
uint8_t CurStep_IsActive(void);       /* 正在跑（含 DONE 但结果未取走） */
uint8_t CurStep_IsDone(void);

uint8_t CurStep_Refs(float *id_ref, float *iq_ref);   /* 1 = 本拍由激励模块接管 */
void    CurStep_Push(float id_now, float iq_now, float vd, float vq);

const CurStepResult *CurStep_GetResult(uint8_t axis); /* 0 = id, 1 = iq */
uint8_t  CurStep_HasAxis(uint8_t axis);
uint16_t CurStep_GetTraceN(void);
void     CurStep_GetTracePoint(uint16_t i, float *t_ms,
                               float *id_ref, float *id_now,
                               float *iq_ref, float *iq_now);

#endif /* CUR_STEP_H */
