/*
 * integration_example.c —— STM32G474 (X72_FOC 板) 接入 ERT 生成代码的最小示例
 *
 * 本文件是「接线模板」，不是可直接编译的固件：
 *   - BSP_* 函数需要按本工程的 ADC / SPI / TIM 驱动实现（见 README.md）
 *   - 只演示控制律调用时序与物理量标定，不含故障保护、状态机、CAN 协议
 *
 * 硬件依据：PG_X72_CAN / STM32G474CEU6 @170MHz，TIM1 中心对齐三相互补 PWM，
 *           3x 内置 OPAMP 差分放大（1mOhm 分流 + 30x + 1.65V 偏置），
 *           MT6835 21bit SPI 绝对磁编码器，VM=48V。
 *
 * 调用契约：
 *   three_loop_deploy_initialize()  上电调用一次
 *   ThreeLoop_Tuning_Init()         上电一次：填 U.par_c/par_w/par_p（默认值 + 固定时序项）
 *   three_loop_deploy_step()        控制 ISR 每 50 us 一次（20 kHz）
 *
 * 精度：生成代码已 single 化（real32_T + sinf/cosf/sqrtf），M4F 硬件 FPU 可直接执行。
 * 实机仍需用 DWT->CYCCNT 复测 step() 单拍 < 50 us（8500 cyc @170MHz）。
 * 若回退 double 版：单拍 9282 cyc ≈ 54.6 us > 50 us 会丢 ADC2 触发、静默掉回 10 kHz，
 * 届时必须把 Ts_i 改回 100e-6 重新生成，使 dt/N_w/N_p 匹配实际节拍。
 *
 * TIM1 中心对齐 PWM：PSC=0、ARR=4249 @170MHz -> 20 kHz 载波，ADC1/ADC2 由
 * TIM1_TRGO 触发；每个 PWM 周期更新一次 duty。
 *
 * 内部节拍（由 step() 内的分频计数实现，无需额外定时器）：
 *   电流环 每 ISR  -> 20 kHz
 *   速度环 每 10   ->  2 kHz
 *   位置环 每 20   ->  1 kHz
 *   Y.ticks[3] 分别是三环累计执行次数，比值应恒为 20 : 2 : 1。
 */

#include "three_loop_deploy.h"
#include "three_loop_tuning.h"
#include <stdint.h>
#include <math.h>

/* 说明：生成模型现有 11 个输入 = 8 个信号 + 3 个参数向量 Inport
 *       (U.par_c[11] / U.par_w[5] / U.par_p[4])。
 *       参数向量由 ThreeLoop_Tuning_Init() 在上电时填入，ISR 里不要写。 */

/* ---------------- 硬件标定（按原理图/数据手册填写） ---------------- */
#define PWM_ARR            4249U        /* 170MHz / (2*20kHz) - 1；以 CubeMX 实际配置为准 */
#define ADC_VREF           3.3f         /* V */
#define ADC_BITS           4096.0f      /* 12 bit */
#define CURRENT_BIAS       1.65f        /* HALF_VREF, V */
#define CURRENT_GAIN       30.0f        /* 差分放大倍数 */
#define R_SHUNT            0.001f       /* Ohm */
#define I_PER_V            (1.0f/(R_SHUNT*CURRENT_GAIN))   /* A/V = 33.33 */
#define ENC_COUNTS         2097152.0f   /* 2^21, MT6835 */
#define TWO_PI             6.28318530717958647692
#define POLE_PAIRS         14.0f        /* 与模型 P.p 一致 */
#define VBUS_DIV           (1.0f)       /* TODO: 按 VM 分压比填写实际系数 */

/* BSP 占位：由现有固件实现 */
extern uint16_t BSP_ADC_ReadPhaseA(void);
extern uint16_t BSP_ADC_ReadPhaseB(void);
extern uint16_t BSP_ADC_ReadPhaseC(void);
extern uint16_t BSP_ADC_ReadVbus(void);
extern uint32_t BSP_MT6835_ReadRaw21(void);
extern float    BSP_GetPosRefRad(void);       /* CAN / 上位机位置指令：多圈累积机械角 rad */
extern float    BSP_GetDirSign(void);         /* 方向辨识得到的 +1 / -1 */
extern uint8_t  BSP_ServoEnabled(void);       /* 使能 + 故障保护综合 */
extern void     BSP_TIM1_SetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);
extern void     BSP_FaultHandler(void);

static float s_theta_prev = 0.0f;
static float s_wm_lpf     = 0.0f;
static uint8_t s_first    = 1U;

static inline float adc_to_amp(uint16_t raw)
{
    float v = ((float)raw) * (ADC_VREF / ADC_BITS);
    return (v - CURRENT_BIAS) * I_PER_V;
}

static inline uint16_t duty_to_ccr(double d)
{
    if (d < 0.0) d = 0.0;
    if (d > 1.0) d = 1.0;
    return (uint16_t)(d * (real32_T)PWM_ARR + 0.5);
}

void Servo_Init(void)
{
    three_loop_deploy_initialize();       /* 清 PI 积分/分频计数/tick */
    ThreeLoop_Tuning_Init();              /* 填 U.par_c/par_w/par_p 默认值 + 固定时序项 */
    three_loop_deploy_U.en = 0.0;
    s_first = 1U;
}

/* TIM1 Update ISR（20 kHz）：请把本函数放进 HAL_TIM_PeriodElapsedCallback
 * 或直接放进 stm32g4xx_it.c 的 TIM1_UP_TIM16_IRQHandler，且不要在 ISR 内 printf。 */
void Servo_20kHz_ISR(void)
{
    float ia, ib, ic, th, dwm, wm;
    uint32_t enc;

    /* 1) 三相电流：ADC 已在 TIM1_TRGO 触发时刻完成采样 */
    ia = adc_to_amp(BSP_ADC_ReadPhaseA());
    ib = adc_to_amp(BSP_ADC_ReadPhaseB());
    ic = adc_to_amp(BSP_ADC_ReadPhaseC());
    /* 若硬件只有两路采样，可改用 ic = -ia - ib */

    /* 2) 编码器：MT6835 21bit 单圈绝对角 -> 机械角 [rad]
     *    注意：编码器零点偏置需标定；控制器内部 theta_e = p*theta_m + pi/2 + ... */
    enc = BSP_MT6835_ReadRaw21();
    th  = (float)(TWO_PI * (real32_T)enc / (real32_T)ENC_COUNTS);

    /* 3) 机械角速度：差分 + 一阶低通（截止约 200 Hz），单位 rad/s */
    if (s_first) { dwm = 0.0f; s_first = 0U; }
    else {
        float d = th - s_theta_prev;
        if (d >  3.14159265f) d -= (float)TWO_PI;   /* 单圈回绕 */
        if (d < -3.14159265f) d += (float)TWO_PI;
        dwm = d / 0.00005f;                        /* Ts_i = 50 us */
    }
    s_theta_prev = th;
    s_wm_lpf += 0.06f * (dwm - s_wm_lpf);
    wm = s_wm_lpf;

    /* 4) 填写生成代码输入结构体 */
    three_loop_deploy_U.ia       = (real32_T)ia;
    three_loop_deploy_U.ib       = (real32_T)ib;
    three_loop_deploy_U.ic       = (real32_T)ic;
    three_loop_deploy_U.vbus     = (real32_T)(((float)BSP_ADC_ReadVbus()) *
                                   (ADC_VREF / ADC_BITS) / VBUS_DIV);
    three_loop_deploy_U.theta_m  = (real32_T)th;      /* 机械角，rad */
    three_loop_deploy_U.omega_m  = (real32_T)wm;      /* 机械角速度，rad/s */
    /* 文档 §3.3：pos_ref 多圈 vs theta_m 单圈，跨 2π 会跳变导致过流，必须折叠 */
    three_loop_deploy_U.pos_ref  = ThreeLoop_WrapPosRefEx(
        (real32_T)BSP_GetPosRefRad(), (real32_T)th, (real32_T)BSP_GetDirSign());
    three_loop_deploy_U.en       = (real32_T)BSP_ServoEnabled();

    /* 5) 执行三环控制律（一个 ISR 内完成，内部分频到 2k/1k） */
    three_loop_deploy_step();

    /* 6) 占空比 -> TIM1 三路 CCR（互补输出/死区由 BDTR 配置） */
    BSP_TIM1_SetDuty(duty_to_ccr(three_loop_deploy_Y.duty[0]),
                     duty_to_ccr(three_loop_deploy_Y.duty[1]),
                     duty_to_ccr(three_loop_deploy_Y.duty[2]));

    /* 7) 在线诊断（建议经 CAN 低频上传，不要在 ISR 内格式化打印）
     *    Y.ticks[0]:Y.ticks[1]:Y.ticks[2] 应约为 20:2:1
     *    Y.iq_ref 可用于限流监视；Y.iq_used 为实际限幅后的电流指令 */
    if (fabsf(three_loop_deploy_Y.iq_ref) > 9.0f) {
        /* BSP_FaultHandler(); 按需启用 */
    }
}
