/*
 * motor_rl_ident.h —— 在线参数辨识：相电阻 R 与 dq 电感 L（VESC 策略）
 *
 * 来源与取舍（bldc-master / motor/mcpwm_foc.c）：
 *
 *   R：直接照搬 mcpwm_foc_measure_resistance() 的口径（:1783）
 *      · 虚拟电角度锁死，给固定方向电流，转子被磁场吸住不动 → ωe = 0
 *      · 稳态时 |v| = R·|i|（没有 ωe·ψ 和 ωe·L·i 项）
 *      · 采样 N 次，R = Σ|v| / Σ|i|，用**矢量模长**而不是单一轴，
 *        这样不必关心 dq 轴分配，也不会因为电流落在某个轴而偏
 *      · 电流斜坡 200 ms 起步，避免阶跃冲击（VESC 用 utils_step_towards）
 *
 *   L：**不能照搬**。VESC 现版 mcpwm_foc_measure_inductance()（:1895）走的是
 *      HFI 六矢量高频注入 + FFT 的 bin0（1/L 平均值）与 bin2（二次谐波，
 *      用来分离 Ld/Lq），依赖它自己的 m_hfi 模块、FFT 与专用采样时序。
 *      X72 没有这套基础设施，硬搬等于重写一个 HFI 观测器。
 *
 *      这里改用等效的**阶跃瞬态拟合**：
 *        电压方程 v = R·i + L·di/dt 在任何时刻都成立，
 *        给 d 轴电流阶跃（表贴式电机 Ld=Lq，且 id 不产生转矩，转子不动），
 *        用 20 kHz 记录 64 拍 (i, vd)，对每一点解
 *            L_k = (vd_k − R·i_k) / (di/dt)_k
 *        取电流确实在变化的点求平均，并用离散度判断可信度。
 *
 *      与 HFI 的差别要讲清楚：
 *        · HFI 在转子静止、电流近零时测，不受死区与饱和影响，精度更高；
 *        · 阶跃法工作电流大（2 A），铁芯可能已开始饱和，测出的 L 偏小，
 *          且死区电压（~0.18 V）会混进 vd 里。所以结果更适合当"额定工况下的
 *          等效 L"，而不是小信号 Ld。
 *        · 好处是不需要额外硬件、不占 CPU、3.2 ms 就测完。
 *
 * 安全：
 *   · 启动前要求 C（电流零点）已完成、Z（电角度）已对齐、无故障；
 *   · R 阶段走虚拟角电流环（id 可配、iq=0），电流由 PI 限制；
 *   · L 阶段走编码器电角度电流环，id 阶跃 2 A、iq=0，不产生转矩；
 *   · 全程 1 ms 检查故障，结束必然封锁 MOE。
 */
#ifndef MOTOR_RL_IDENT_H
#define MOTOR_RL_IDENT_H

#include <stdint.h>

typedef enum {
    RL_IDENT_IDLE = 0,
    RL_IDENT_R_RAMP,     /* 电流 0 -> 目标值，200 ms */
    RL_IDENT_R_SETTLE,   /* 100 ms 等转子锁死、电流稳态 */
    RL_IDENT_R_SAMPLE,   /* 50 ms 累加 |v| / |i| */
    RL_IDENT_L_INJ,      /* 60 拍 1 kHz 单频电压注入并记录 (id, phase) */
    RL_IDENT_DONE,
    RL_IDENT_FAIL,
} RlIdentState;

typedef enum {
    RL_IDENT_OK = 0,
    RL_IDENT_ERR_BUSY = 1,
    RL_IDENT_ERR_CAL = 2,        /* 没做 C */
    RL_IDENT_ERR_ZERO = 3,       /* 没做 Z */
    RL_IDENT_ERR_FAULT = 4,
    RL_IDENT_ERR_PWM = 5,
    RL_IDENT_ERR_CURRENT = 6,    /* 电流没建立起来（接触器/接线/限流） */
    RL_IDENT_ERR_FIT = 7,        /* L 有效点太少或离散过大 */
    RL_IDENT_ERR_TIMEOUT = 8,
} RlIdentErr;

typedef struct {
    float r_ohm;          /* 相电阻（dq 等效） */
    float l_h;            /* dq 电感（阶跃法，额定电流下的等效值） */
    float v_avg;          /* R 测量时的平均 |v| */
    float i_avg;          /* R 测量时的平均 |i| */
    float z_ohm;          /* |Z| = V_amp / I_amp（注入法的中间量，便于核对） */
    float i_amp;          /* 注入产生的电流幅值 */
    float inj_amp;        /* 注入电压幅值 */
    uint16_t l_points;    /* 参与 DFT 的采样点数 */
    RlIdentErr err;
} RlIdentResult;

void            RlIdent_Init(void);
RlIdentErr      RlIdent_Start(float test_current_a);
void            RlIdent_Abort(void);
uint8_t         RlIdent_Task1ms(void);      /* 1 = 状态推进或结束（命令层打印用） */
RlIdentState    RlIdent_GetState(void);
uint8_t         RlIdent_IsBusy(void);
const RlIdentResult *RlIdent_GetResult(void);
const char     *RlIdent_ErrStr(RlIdentErr e);

/* 从 Flash 恢复上次辨识结果（只用于回显与对照，不改电流环增益 ——
 * 增益改不改要人工确认，否则模型和固件两处参数会悄悄失配）。 */
void RlIdent_SetCached(float r_ohm, float l_h);

/* 改注入电压幅值（V）。用来看 L 随注入电流的变化 —— 铁芯饱和会让 L 明显下降，
 * 只测一个点无法判断"L 偏小"是模型错还是饱和。 */
void RlIdent_SetInjAmp(float amp_v);
float RlIdent_GetInjAmp(void);

/* ---- ISR 侧（20 kHz，只在单频注入分支里调用） ---- */
/* 记录本拍 d 轴电流与注入相位，供单点 DFT 提取基波幅度 */
void    RlIdent_PushInject(float id_now, float phase);

#endif /* MOTOR_RL_IDENT_H */
