/*
 * deadband_comp.h —— 逆变桥死区补偿
 *
 * 移植自 FM_XC `simulink_code_gen/current_ctl_loop_task.c:31`
 * `current_ctl_loop_task_deadband_comp_ua()`，按 X72 的语义重写为单相函数。
 *
 * 为什么需要它
 * ------------
 * TIM1 的 DeadTime = 50（DTG[7:5]=001 段，(64+50)×5.88ns ≈ 670 ns）。
 * 死区期间上下管都关断，电流只能走体二极管续流，导致实际输出电压比指令
 * 偏差一个固定的占空比量：
 *
 *      Δd = t_dead × f_pwm = 670 ns × 20 kHz ≈ 1.34 %
 *
 * 这个偏差的符号跟着相电流方向走，所以表现为"正负半周各少一点"，
 * 平均看是增益损失 + 过零附近的力矩脉动。高速大电流时占比小，
 * 但 X72 在 3 rpm 这种低速轻载下相电压本来就小，1.34% 的占空比误差
 * 已经能明显影响转速平稳度。
 *
 * 补偿策略（与 FM_XC 一致）
 * ------------------------
 *    |i| > I_LIN        -> 补偿 +DU_MAX（饱和）
 *    |i| <= I_LIN       -> 补偿 (i/I_LIN) × DU_MAX（线性过渡）
 *
 * 线性过渡是为了避免电流过零时补偿量在 ±DU_MAX 之间硬跳变，那会在
 * 过零点引入抖动，反而放大脉动。I_LIN 取 2~3 倍电流噪声水平。
 *
 * 注意：duty 的语义是"上管导通比例"（见 bsp_servo.c 里 PWM2 的说明），
 * 补偿直接加在这个域上，与 FM_XC 的 u_tar 域等价。
 */
#ifndef DEADBAND_COMP_H
#define DEADBAND_COMP_H

#include <stdint.h>

/* 死区补偿量（占空比域）。改死区时间或 PWM 频率时要同步这里。
 *
 * ⚠ 2026-11-17 修正：原来写 670 ns 是按 `(64+50)×2×tDTS` 算的，
 * 但那条公式只对 **DTG[7:5] = 10x** 成立。DTG=50 = 0b00110010，
 * 最高位（bit7）= 0，属于 **0xx 段**，公式是 `DT = DTG[7:0] × tDTS`：
 *
 *      DT = 50 × 5.88 ns = 294 ns      （CKD=DIV1，tDTS=1/170MHz）
 *
 * 原来的 670 ns 把补偿量放大了 2.28 倍，这就是"一开补偿就过补偿、
 * 3 rpm 跑成 0.10 rpm、clamp 涨到 748"的直接原因 —— 不是补偿原理错，
 * 是量给多了。 */
#define DBC_T_DEAD_NS   (294.0f)      /* 与 TIM1 DeadTime=50 对应（0xx 段） */
#define DBC_PWM_HZ      (20000.0f)    /* 中心对齐 ARR=4250 @170MHz */
#define DBC_DU_MAX      ((DBC_T_DEAD_NS * 1.0e-9f) * DBC_PWM_HZ)   /* ≈0.0059 */

/* 线性过渡区半宽（A）。低于此值按比例补偿，超过则饱和。
 *
 * 原值 0.6 A 对 X72 太宽：3 rpm 空载相电流只有 0.08~0.2 A，
 * 落在 1/3~1/7 处，补偿被线性缩到很小、形状也不对。
 * 电流噪声约 0.05 A，取 0.15 A 让"有明确方向的电流"拿到全量补偿，
 * 只有真正的过零区才按比例过渡。 */
#define DBC_I_LIN_A     (0.15f)

/* 转速门控（rad/s，机械角速度）。
 *
 * 2026-11-17 实测：du=0.59% 时
 *   30 rpm  σ +7.6%（三轮全正：+9/+5/+8）
 *   100 rpm σ +6.5%
 *   260 rpm σ **−25.5%**（两轮 −9/−42，明显恶化）
 *   500/1000 rpm 在噪声内
 *
 * 260 rpm 恰好落在速度环的高纹波带（见 速度扫描_1-1000rpm.md），
 * 补偿在那里不是抵消畸变而是往一个已经在振的环里再注入一份扰动。
 * 高速端惯性本来就把死区畸变滤掉了，补偿也没有收益。
 * 所以按转速门控：100 rpm 以下全量，200 rpm 以上完全关闭。
 */
#define DBC_W_FULL      (10.47f)    /* 100 rpm */
#define DBC_W_ZERO      (20.94f)    /* 200 rpm */

/* 对单相占空比做死区补偿。i_phase 用该相实测电流（A）。
 * 输出自动限幅到 [0,1]。 */
float DeadbandComp_Apply(float duty, float i_phase);

/* 开关（默认关）。便于做 A/B 对比验证补偿到底有没有用。 */
void    DeadbandComp_SetEnable(uint8_t en);

/* 每拍喂一次机械角速度（rad/s），内部换算成补偿门控系数。
 * 100 rpm 以下全量、200 rpm 以上关闭、中间线性过渡（理由见 DBC_W_FULL 注释）。 */
void    DeadbandComp_SetSpeed(float w_mech);
uint8_t DeadbandComp_IsEnabled(void);

/* 补偿量（占空比）。运行时可改，用于标定：理论值只是起点，
 * 真正的量取决于"电流环积分已经补掉多少平均值"，只能用实验找。
 * 范围 0 ~ 0.05（5% 占空比），超出视为误操作直接夹住。 */
void    DeadbandComp_SetDu(float du);
float   DeadbandComp_GetDu(void);

/* 当前补偿量（占空比），供串口诊断 */
float   DeadbandComp_GetDuMax(void);
#endif /* DEADBAND_COMP_H */
