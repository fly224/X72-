/*
 * bsp_servo.c —— PG_X72_CAN 板级驱动：相电流、母线电压、MT6835、三相 PWM、故障封锁
 *
 * 硬件依据（Netlist_PG_X72_CAN_2026-11-07.enet / X72_FOC.ioc）：
 *   OPA1(PA2/ADC1_IN3) <- R4(1mOhm) = U 相；OPA2(PA6/ADC2_IN3) <- R3(1mOhm) = W 相；
 *   OPA3(PB1/ADC3_IN1) 板上无采样网络，不使用。
 *   TIM1 中心对齐 ARR=4250 -> 20kHz，TRGO=OC4REF 触发 ADC1/2 注入。
 *   SPI1(PB3/4/5) Mode3 10.625MHz + DMA1_Ch1/2 -> MT6835；PC11 为软件片选。
 *
 * ISR 纪律：本文件所有回调只做寄存器/缓存操作，不解析协议、不打印、不等待外设；
 *          SPI1 只由 20kHz 控制节拍启动、由 DMA 完成中断收尾，主循环不得调用 HAL_SPI_*。
 */
#include "bsp_servo.h"
#include "uart_link.h"          /* 仅用于启动分步打点，运行期不调用 */
#include "enc_comp.h"           /* 编码器角度谐波补偿（30098 写系数，本文件用于速度） */
/* 状态指示灯的"参数辨识"判据要查这六个模块的忙状态。
 * 这些头只依赖 <stdint.h>，与本文件无循环依赖。 */
#include "motor_ident.h"
#include "motor_rl_ident.h"
#include "motor_flux_ident.h"
#include "mech_ident.h"
#include "fric_cal.h"
#include "encoder_cal.h"

#include "adc.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* ---------------- 静态状态（无动态分配） ---------------- */
/* 极对数：30098 的系数是电角度域，换算成机械角要除以它。
 * 与 three_loop_deploy.c 的 14.0、telemetry_tx.c 的 POLE_PAIRS 必须一致；
 * 换电机时这三处一起改。 */
#define BSP_ENC_COMP_POLES  (14.0f)
static volatile uint32_t s_fault = 0U;
static volatile uint8_t  s_pwm_enabled = 0U;

static volatile float s_iu = 0.0f;          /* U 相电流，A */
static volatile float s_iw = 0.0f;          /* W 相电流，A */
static volatile float s_vbus = 48.0f;       /* 母线电压，V；上电先取额定值 */

/* ---------------- 保护状态（2026-11-17） ----------------
 * 全部在 BspServo_SlowTask1ms（1 kHz）里推进，不进 20 kHz 控制环：
 * 母线跌落/泵升是毫秒级现象，1 kHz 足够，且避免在电流环里加分支。
 */
static volatile float    s_derate = 1.0f;   /* 温度降额系数 0..1 */
static uint16_t s_ov_cnt;                   /* 过压消抖计数 */
static uint16_t s_uv_cnt;                   /* 欠压消抖计数 */
static uint16_t s_vb_blank = BSP_VBUS_BLANK;

static uint16_t s_adc1_regular[4];          /* ADC1 常规 DMA：IN1(VBUS)/温度/IN14/IN11 */

/* ---------------- NTC 温度（ADC2 常规组） ----------------
 * 拓扑（Hardware/Netlist/net_report.md 已核对）：
 *     3V3 — R56/R57 (50kΩ 0.1%) — 节点 — 被测 NTC — GND
 *     节点 — R54/R55 (1kΩ) — PA4 / PC4（ADC 高阻输入）+ 100nF 到地
 *   ADC 输入电流为 nA 级，1kΩ 上的压降可忽略，故 V(ADC) = V(节点)。
 *   25 °C 时 NTC=50kΩ → 分压中点 1.65 V → 码值 ≈ 2048（自检基准）。
 * 板上 NTC_GAN = R53 RL0603N503F4150FTF：50kΩ@25°C，B(25/85)=4150K，1%。
 * NTC_MOTOR 走 U14 外接 2P 探头，**与板上 NTC 不是同一规格**：
 *   实测（2026-11-15，板温 31.6°C、电机未运行）按 50k/B4150 换算得 69°C，
 *   反算 R≈8.36kΩ；而 10kΩ/B3950 探头在 ~29°C 正好是 8.4kΩ，与板温自洽。
 *   → 默认按 10k/B3950（电机内置 NTC 最常见规格）。换探头只改下面三个宏，
 *     并用两点法复核（冰水 0°C、沸水 100°C，或与热电偶对比）。
 */
#define NTC_R_PULL_OHM   50000.0f          /* R56 / R57 上拉（两路相同） */
#define NTC_KELVIN       273.15f

/* NTC_GAN：板上 R53，参数由型号确定 */
#define NTC_GAN_R25      50000.0f
#define NTC_GAN_BETA     4150.0f
/* NTC_MOTOR：U14 外接探头（默认 10k/B3950，待实物确认） */
#define NTC_MOT_R25      10000.0f
#define NTC_MOT_BETA     3950.0f

#define NTC_T25_KELVIN   298.15f
#define NTC_PERIOD_MS    100U              /* 10 Hz：热时间常数为秒级，够用 */
#define NTC_TIMEOUT_MS   50U               /* 转换迟迟不回来（被注入长期占用）→ 判失败重来 */

static uint16_t          s_ntc_raw[2];      /* DMA 目标：[0]=NTC_GAN [1]=NTC_MOTOR */
static volatile uint8_t  s_ntc_state;       /* 0=空闲 1=转换中 2=数据就绪 */
static uint16_t          s_ntc_div;
static volatile float    s_temp_gan;        /* 有效值见 s_ntc_ok；未采到由 Getter 返回 NaN */
static volatile float    s_temp_motor;
static volatile uint32_t s_ntc_ok;
static uint32_t          s_ntc_fail;

static uint8_t  s_enc_tx[BSP_ENC_FRAME_LEN] = {
    BSP_ENC_CMD_CONT_READ, BSP_ENC_CMD_ADDR_LOW, 0xFFU, 0xFFU, 0xFFU, 0xFFU
};
static uint8_t  s_enc_rx[BSP_ENC_FRAME_LEN];
static volatile uint8_t  s_spi_busy = 0U;
static volatile uint32_t s_enc_raw = 0U;    /* 0 .. 2^21-1 */
static volatile uint8_t  s_enc_status = 0U;
static volatile uint8_t  s_enc_err_cnt = 0U;
static volatile uint8_t  s_enc_jump_cnt = 0U;   /* 角度连续性：连续跳变帧数 */
static volatile uint8_t  s_enc_seen;            /* 1 = s_enc_raw 已有有效值 */
static volatile float    s_theta = 0.0f;
static volatile float    s_omega_mech = 0.0f;
/* 未滤波单拍速度：只给机械辨识用（谐振扫频要看到 100 Hz 以上，
 * 8 拍累计 + 95 Hz 低通会把要测的东西直接抹掉）。
 * 量化噪声 ±1 count / 50us = ±0.06 rad/s，同步解调长时间累加后可忽略。 */
static float             s_omega_fast = 0.0f;
static volatile float    s_enc_zero = 0.0f; /* 编码器零点偏置，rad */
static volatile uint8_t  s_enc_zero_ok;     /* 1 = 已做过电角度对齐（Z）；RAM，复位清零 */
static float   s_theta_prev = 0.0f;
static float   s_theta_multi;          /* 多圈机械角（与 s_theta 同零点） */
static float   s_tm_base;              /* 多圈角 = s_tm_base + s_pos_sum·(2π/2^21) */
static uint8_t s_tmulti_first = 1U;
static uint8_t s_theta_first = 1U;
/* 8 拍累计测速的状态（BSP_SPEED_AVG_N）：只在 GetThetaRad() 里访问，无需 volatile */
static float     s_spd_acc = 0.0f;
static uint8_t   s_spd_cnt = 0U;

/*
 * 多圈位置累积（在原始计数域做，不受零点偏置影响）：
 *   s_pos_sum  = 相对上电点的累积脉冲，int64 可存 ±2^63，按 2^21/圈 折算约 ±4e12 圈
 *   s_enc_prev = 上一拍的原始计数，只由 GetThetaRad() 更新（20kHz 唯一调用点）
 * 回绕判定用 ±半圈门限：20kHz 下最高 100 rad/s 机械 ≈ 1669 counts/拍，
 * 远小于 1048576，因此不会误判。
 */
static volatile int64_t  s_pos_sum = 0;
static uint32_t s_enc_prev = 0U;
static uint8_t  s_pos_first = 1U;

static inline void EncCsLow(void)
{
    HAL_GPIO_WritePin(BSP_ENC_CS_PORT, BSP_ENC_CS_PIN, GPIO_PIN_RESET);
}
static inline void EncCsHigh(void)
{
    HAL_GPIO_WritePin(BSP_ENC_CS_PORT, BSP_ENC_CS_PIN, GPIO_PIN_SET);
}

static inline float RawToVolt(uint16_t raw)
{
    return (float)raw * (BSP_ADC_VREF / BSP_ADC_COUNTS);
}

/*
 * 电流零点标定。
 * 为什么必须标：运放失调 + 1% 电阻 + 分压误差会让"零电流"偏离 1.65 V。
 * 实测未标定时 iu=-0.201 A、iw=-1.410 A（等效偏置 -6.0 mV / -42.3 mV），
 * 电流环会把这个虚假电流当真去积分，一使能就输出错误力矩矢量。
 * 标定期间 MOE 必须为 0（无输出、电机静止），否则采到的是真实电流。
 */
#define BSP_CAL_SAMPLES   4000U             /* 4000 x 50 us = 200 ms @20 kHz */

static volatile float    s_bias_u = BSP_CURRENT_BIAS_V;
static volatile float    s_bias_w = BSP_CURRENT_BIAS_V;
static volatile float    s_cal_sum_u;
static volatile float    s_cal_sum_w;
static volatile uint32_t s_cal_n;
static volatile uint8_t  s_cal_state;      /* 0=未标定 1=标定中 2=已完成 */
static volatile uint8_t  s_cal_why;
static volatile uint8_t  s_enc_flt_mask;   /* 1 = 暂不置编码器故障（^ 运行期间） */        /* 标定被拒的原因：0=正常 1=偏置出窗 2=两相差过大 */
static volatile uint8_t  s_enc_bad_run;    /* 连续坏帧数（CRC/status/跳变），触发 SPI 重建 */
static volatile uint16_t s_enc_ok_run;     /* 连续有效帧数，达 BSP_ENC_OK_N 清编码器故障 */
static volatile uint8_t  s_enc_reinit_req; /* 1 = 请求在 1 ms 任务里重建 SPI1 */
static uint32_t          s_enc_rec_t0;     /* 重建限流窗口起点 */
static uint8_t           s_enc_rec_n;      /* 窗口内已重建次数 */

static void enc_spi_recover(void);         /* 定义在 MT6835 段，1 ms 任务要用 */

/*
 * 过流判定消抖 + 现场留证。
 * 背景：MOE=0 时电流很干净（±0.03 A），一旦放开主输出，50 us 节拍里会采到
 * 开关瞬间的单点尖峰。单次超阈值就跳闸会让电机根本起不来，而真实过流是持续的，
 * 所以要求"连续 N 次超阈值"才判故障（N=8 -> 400 us）。
 * 同时把第一次超阈值的原始码值/电流存下来，'S' 可回读，用于区分
 * "噪声尖峰"与"真的过流"。
 */
#define BSP_OVC_DEBOUNCE  8U                /* 连续 8 次 = 400 us @20 kHz */
#define BSP_OVC_BLANK     40U               /* 开通消隐 2 ms @20 kHz
                                             * 原来给 20 ms，结果把一次真实的
                                             * 43.8 A 飞车冲击整段遮住了（fault 仍为 0）。
                                             * 开通瞬态实测只有几十 us，2 ms 足够。 */

static volatile uint8_t  s_ovc_cnt;
static volatile uint16_t s_ovc_blank;       /* >0 时暂停过流判定（开通瞬态） */
static volatile float    s_ovc_peak_u;      /* 全程最大 |iu| 及其码值 */
static volatile float    s_ovc_peak_w;
static volatile uint16_t s_ovc_raw_u;
static volatile uint16_t s_ovc_raw_w;

static inline void FaultSet(uint32_t bit)
{
    s_fault |= bit;
    /* 故障即刻封锁主输出，计数器与 TRGO 继续运行以便诊断 */
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    s_pwm_enabled = 0U;
}

void BspServo_RaiseFault(uint32_t bit)
{
    FaultSet(bit);
}

uint32_t BspServo_GetFault(void)   { return s_fault; }
void     BspServo_ClearFault(void) { s_fault = 0U; s_enc_err_cnt = 0U; s_enc_jump_cnt = 0U; }

/* ================= 看门狗（IWDG，250 ms） =================
 * 为什么直接写寄存器而不走 HAL：HAL_IWDG 模块在 stm32g4xx_hal_conf.h 里是
 * 注释掉的，打开它要连带把 stm32g4xx_hal_iwdg.c 加进 Keil 工程 —— 这个工程
 * 有"重新生成后源文件丢失、旧 .o 让链接悄悄成功"的前科，能不动工程文件就不动。
 * IWDG 的全部配置只有 4 个寄存器，直接写反而更可控。
 *
 * 时序：KR=0xCCCC 启动（LSI 同时起振）→ KR=0x5555 解锁 PR/RLR →
 *       PR=4(/64)、RLR=125 → 等 SR 的 PVU/RVU 落 → KR=0xAAAA 重装。
 * 超时 = 125 × 64 / 32000 Hz = 250 ms（LSI 26~34 kHz 对应 235~308 ms）。
 *
 * ⚠ 喂狗必须在主循环。1 ms 定时器中断里喂会掩盖"主循环卡死"——
 *   那正是 2026-11-17 真实发生过、只能烧录才能恢复的故障形态。
 */
#define BSP_WDG_ENABLE   (1)        /* 单步调试时改 0 重编，否则断点停住会被复位 */

void BspServo_WdgInit(void)
{
#if BSP_WDG_ENABLE
    uint32_t guard = 0U;

    IWDG->KR  = 0xCCCCu;            /* 启动 */
    IWDG->KR  = 0x5555u;            /* 允许写 PR/RLR */
    IWDG->PR  = 4u;                 /* /64 */
    IWDG->RLR = 125u;               /* 250 ms */
    while (((IWDG->SR & 0x3u) != 0U) && (guard < 1000000U)) { guard++; }
    IWDG->KR  = 0xAAAAu;
#endif
}

void BspServo_WdgKick(void) { IWDG->KR = 0xAAAAu; }

/* ================= 温度降额 ================= */
float BspServo_GetDerate(void) { return s_derate; }

void BspServo_GetVbusLimits(float *ov, float *uv)
{
    if (ov) { *ov = BSP_VBUS_OV_TRIP; }
    if (uv) { *uv = BSP_VBUS_UV_TRIP; }
}

/* 线性降额：T0 以下 1.0，T1 以上 0.0。NaN（NTC 开路/短路）按 1.0 处理。 */
static float derate_linear(float t, float t0, float t1)
{
    if (isnan(t))  { return 1.0f; }
    if (t <= t0)   { return 1.0f; }
    if (t >= t1)   { return 0.0f; }
    return (t1 - t) / (t1 - t0);
}

static void thermal_update(void)
{
    const float dg = derate_linear(BspServo_GetTempGan(),   BSP_DERATE_GAN_T0, BSP_DERATE_GAN_T1);
    const float dm = derate_linear(BspServo_GetTempMotor(), BSP_DERATE_MOT_T0, BSP_DERATE_MOT_T1);

    s_derate = (dg < dm) ? dg : dm;
}

/* ================= 母线过压 / 欠压 =================
 * 判据放在 1 kHz 任务里并做 20 ms 消抖：母线在每次开关沿都有几十微秒的
 * 跌落与振铃，单拍比阈值必然误跳。
 *
 * 过压：任何状态都判 —— 错接电源或制动回馈泵升，即使没使能也要报。
 * 欠压：只在本级输出使能时判 —— 只插 USB、母线断电是正常工况，
 *       那种情况下判欠压会让板子永远带着故障位。
 */
static void vbus_protect(void)
{
    float v;

    if (s_vb_blank != 0U) { s_vb_blank--; return; }

    v = s_vbus;
    if (isnan(v) || (v <= 0.0f) || (v > 200.0f)) {   /* 分压/ADC 异常：不判，免误跳 */
        s_ov_cnt = 0U;
        s_uv_cnt = 0U;
        return;
    }

    if (v > BSP_VBUS_OV_TRIP) {
        if (s_ov_cnt < BSP_VBUS_DEBOUNCE) { s_ov_cnt++; }
        if (s_ov_cnt >= BSP_VBUS_DEBOUNCE) { FaultSet(BSP_FAULT_OVERVOLT); }
    } else {
        s_ov_cnt = 0U;
    }

    if (BspServo_IsPwmEnabled() != 0U) {
        if (v < BSP_VBUS_UV_TRIP) {
            if (s_uv_cnt < BSP_VBUS_DEBOUNCE) { s_uv_cnt++; }
            if (s_uv_cnt >= BSP_VBUS_DEBOUNCE) { FaultSet(BSP_FAULT_UNDERVOLT); }
        } else {
            s_uv_cnt = 0U;
        }
    } else {
        s_uv_cnt = 0U;
    }
}

void BspServo_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ---------------- 初始化 ---------------- */
/* ---------------- MT6835 客户端自校准的引脚与寄存器访问 ---------------- */
void BspServo_InitEncCalPin(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* 默认低电平 = 不进校准模式。拉高会在下一个 SCK 周期后开始自校准，
     * 所以只允许在校准状态机里改变它。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

void BspServo_SetEncCalEn(uint8_t en)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (en != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * 单字节读寄存器（MT6835 手册 7.6.4）。
 * 24 bit 帧：C3~C0=0011(读) | A11~A0 | DO7~DO0
 *   tx[0] = 0x30 | (addr >> 8)
 *   tx[1] = addr & 0xFF
 *   tx[2] = 0x00（dummy，接收数据在这里）
 *
 * 与 20 kHz 角度 DMA 共用 SPI1：先等当前传输结束（最多 5 ms），
 * 再用阻塞方式发 3 个字节。整帧约 2.3 us @10.6 MHz，丢 1 拍角度可接受。
 */
uint8_t BspServo_ReadEncReg(uint16_t addr)
{
    uint8_t tx[3];
    uint8_t rx[3] = { 0U, 0U, 0U };
    uint32_t t0;

    tx[0] = (uint8_t)(0x30U | (uint8_t)((addr >> 8) & 0x0FU));
    tx[1] = (uint8_t)(addr & 0xFFU);
    tx[2] = 0x00U;

    t0 = HAL_GetTick();
    while ((s_spi_busy != 0U) && ((HAL_GetTick() - t0) < 5U)) { }

    s_spi_busy = 1U;
    EncCsLow();
    (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3U, 10U);
    EncCsHigh();
    s_spi_busy = 0U;

    return rx[2];
}

HAL_StatusTypeDef BspServo_Init(void)
{
    HAL_StatusTypeDef st;

    BspServo_DwtInit();

    /* MT6835 的 CAL_EN (PB9)：必须先拉低，避免上电瞬间进未知状态 */
    BspServo_InitEncCalPin();

    /* 1) 电流运放：OPA3 输入悬空，不启动 */
    st = HAL_OPAMP_Start(&hopamp1);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }
    st = HAL_OPAMP_Start(&hopamp2);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }

    /* 2) ADC 单端校准，减小偏置误差 */
    st = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }
    st = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }

    /* 3) 常规转换 DMA
     *    ADC1：VBUS/NTC 之外的固定 4 通道（Rank1 = PA0 = VBUS），持续搬运。
     *    ADC2：只服务 NTC，改成"10 Hz 单次扫描"，见下方配置。
     */
    st = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_regular, 4U);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }

    /*
     * ADC2 常规组（NTC）与 20 kHz 注入（W 相电流）共用同一个 ADC，必须隔离：
     *   - CubeMX 默认 ContinuousConvMode/DMAContinuousRequests = ENABLE，
     *     照原样启动会让 ADC2 满速扫描 NTC（2 通道 ≈1.7 us 一轮，DMA 请求
     *     ~570 kHz），既挤总线又长期占用 ADC2。
     *   - DMA 句柄默认 CIRCULAR；改 NORMAL 后 HAL 才能在完成回调里把 ADC
     *     状态机归位，否则第二次 HAL_ADC_Start_DMA 会一直返回 HAL_BUSY。
     *   - 只改 HAL 变量与 CFGR 的 CONT/DMACFG 位，**不重新 HAL_ADC_Init**
     *     （那会复位注入配置），更**不调用 HAL_ADC_Stop_DMA**
     *     （它清 ADEN，20 kHz 注入链会一起失效）。
     */
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    CLEAR_BIT(ADC2->CFGR, ADC_CFGR_CONT | ADC_CFGR_DMACFG);
    if (hadc2.DMA_Handle == 0) { FaultSet(BSP_FAULT_BSP_INIT); return HAL_ERROR; }
    hadc2.DMA_Handle->Init.Mode = DMA_NORMAL;
    st = HAL_DMA_Init(hadc2.DMA_Handle);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }
    /*
     * CubeMX 只给 DMA1_Channel4 配了参数，**没有勾 NVIC**（adc.c 里只有 Instance 赋值，
     * stm32g4xx_it.c 里也没有 DMA1_Channel4_IRQHandler）。少了这一段，DMA 硬件照样
     * 把数据搬完，但 HAL 的完成回调永不触发 —— 实测表现就是 NTC 状态卡在"转换中"、
     * 温度恒为 NaN（1560 帧全 NaN）。ISR 见 stm32g4xx_it.c。
     * 优先级取最低 15：ADC 注入(0)/SPI1(1)/USART1(8) 都在它之上，绝不抢控制环。
     */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* 4) 注入转换中断：由 TIM1_TRGO(OC4REF) 触发，软件不启动转换 */
    st = HAL_ADCEx_InjectedStart_IT(&hadc1);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }
    st = HAL_ADCEx_InjectedStart_IT(&hadc2);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }

    /* 5) 编码器先取一帧，保证首次控制节拍有有效角度 */

    /* 4.9) SPI1 降速到 5.3 MHz（2026-11-22）
     *
     * CubeMX 配的是 Prescaler=16 → 170/16 = 10.6 MHz。MT6835 上限是 16 MHz
     * （mt6835.txt:552），余量只有 34%；两次 0x0002 都指向偶发错位。
     * 降到 /32 = 5.3 MHz 后，6 字节传输从 4.5 µs 变 9 µs，仍远小于 50 µs
     * 控制周期，对时序没有影响，但采样裕度翻倍。
     * 不直接改 spi.c：那是 CubeMX 生成文件，重新生成会被覆盖。
     * 降速后紧接着 DeInit/Init，正好把外设拉到干净状态再开始读帧。 */
    if (hspi1.Init.BaudRatePrescaler != SPI_BAUDRATEPRESCALER_32) {
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
        (void)HAL_SPI_DeInit(&hspi1);
        (void)HAL_SPI_Init(&hspi1);
    }

    /*
     * 注入触发实况快照（一次性）。
     * 2026-11-15 实测 adc1=60020 / adc2=30010：ADC2 每两次触发才完成一次，
     * 控制节拍被迫降到 10 kHz。CFGR 的 JEXTEN/JEXTSEL 与 CR 的 JADSTART
     * 是唯一能证明"配置 vs 硬件实际"的证据，先把它打出来。
     *   顺序：ADC1_CFGR ADC1_CR ADC1_SQR1 | ADC2_CFGR ADC2_CR ADC2_SQR1 | CCR
     */
    st = BspServo_StartEncoderDma();
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }

    /* 6) 启动 TIM1 三路 PWM 通道与计数器。
     *    不调用 HAL_TIMEx_PWMN_Start()：该 API 会置位 BDTR.MOE，在随后清零之前
     *    存在六路 50% 占空比输出窗口，48V 母线上不可接受。
     *    这里直接置 CCER 的互补通道使能位，全程保持 MOE=0。
     *    TIM1_UP 不跑控制律，控制律挂在 ADC 注入完成回调。
     */
    st = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (st == HAL_OK) st = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    if (st == HAL_OK) st = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    if (st != HAL_OK) { FaultSet(BSP_FAULT_BSP_INIT); return st; }

    SET_BIT(htim1.Instance->CCER,
            TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

    /* 7) 上电全程封锁：计数器运行、TRGO 继续触发 ADC，但六路输出无驱动 */
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
    s_pwm_enabled = 0U;

    return HAL_OK;
}

void BspServo_PwmEnable(uint8_t enable)
{
    if (enable) {
        if (s_fault != 0U) return;              /* 故障未清除不允许使能 */
        SET_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
        s_pwm_enabled = 1U;
        s_ovc_blank   = BSP_OVC_BLANK;          /* 开通瞬态消隐 20 ms */
        s_ovc_cnt     = 0U;
    } else {
        CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
        s_pwm_enabled = 0U;
        s_ovc_blank   = 0U;
    }
}

uint8_t BspServo_IsPwmEnabled(void) { return s_pwm_enabled; }

void BspServo_SetDuty(float d1, float d2, float d3)
{
    /* 全程 float：M4F 走 VCVT/MUL.F32，约 20 周期；
     * 若用 double，3 次 dmul + 3 次 d2iz 是软浮点，实测 1025 周期 */
    const float d[3] = { d1, d2, d3 };
    uint16_t ccr[3];

    for (uint8_t i = 0U; i < 3U; i++) {
        float x = d[i];
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        /*
         * ⚠ TIM1 的 CH1~CH3 在 X72_FOC.ioc 里配的是 TIM_OCMODE_PWM2（不是 PWM1）。
         *   PWM2 + OCPolarity=HIGH 的语义：CNT < CCR 时 OCxREF=0，
         *   所以上管导通时间 = ARR - CCR，即"高电平占空比 = 1 - CCR/ARR"。
         *   本函数对外承诺的是**上管导通比例 d**，因此必须写 (1-d)。
         *
         * 历史坑（2026-11-15）：此处原来写的是 x*ARR（PWM1 语义），三相输出整体
         * 反相 -> 线电压符号翻转 -> 电流环从负反馈变成**正反馈** ->
         * 使能后立刻 20 A，闭环对齐时冲过 40~47 A。表现为"一开 MOE 就过流"，
         * 而 MOE=0 时电流完全干净（0.01 A），极易误判成硬件直通。
         *
         * 若将来把 .ioc 改成 PWM1，这里必须同步改回 x*ARR，否则会二次反相。
         */
        ccr[i] = (uint16_t)((1.0f - x) * (float)BSP_PWM_ARR + 0.5f);
    }
    /* 直接写比较寄存器，避免在 20kHz 节拍里调用 HAL 大函数 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr[0]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr[1]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr[2]);
}

void BspServo_ReadInjected(float *iu, float *iw)
{
    uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint16_t raw_w = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    float vu = RawToVolt(raw_u);
    float vw = RawToVolt(raw_w);

    /* 标定窗口：只在 MOE=0 时由 BspServo_StartCurrentCal() 打开，累加原始电压 */
    if (s_cal_state == 1U) {
        s_cal_sum_u += vu;
        s_cal_sum_w += vw;
        if (++s_cal_n >= BSP_CAL_SAMPLES) {
            const float bu = s_cal_sum_u / (float)s_cal_n;
            const float bw = s_cal_sum_w / (float)s_cal_n;

            /* ---- 合理性检查（2026-11-21 加，实机踩到）----
             *
             * 静止、MOE=0 时运放输出必须停在**中点附近**（≈Vref/2 = 1.65 V）。
             * 若接近 0 或接近满量程，说明采样链没工作：
             *   运放无供电 / 1.65 V 偏置源丢失 / ADC 通道断 / 采样电阻开路。
             *
             * 为什么必须拒绝：照样写 bias 的话，真的通电后 iu = (vu − 0)×33.3
             * 会恒为 0，电流环积分一路涨到限幅 —— 电机要么飞车要么撞过流，
             * 而且现象是"标定通过、使能就炸"，极难反推。
             *
             * 实机证据：U/W 两相同时读到 raw=0/2 cnt（0 V），bias 被设成 0.0016 V，
             * 标定"成功"，iu 恒为 0。当时是因为 USART 溢出把命令通道卡死了，
             * 才没走到使能那一步。
             *
             * 两相之差也查：正常两路偏置应当一致，差 >0.2 V（≈6.7 A）说明
             * 有一路接错或器件不对称。 */
            const float dv = (bu > bw) ? (bu - bw) : (bw - bu);

            if ((bu < BSP_CAL_BIAS_MIN_V) || (bu > BSP_CAL_BIAS_MAX_V)
             || (bw < BSP_CAL_BIAS_MIN_V) || (bw > BSP_CAL_BIAS_MAX_V)
             || (dv > BSP_CAL_BIAS_MAX_DV)) {
                s_cal_state = 0U;                 /* 不写 bias，保持上一次的值 */
                s_cal_why   = (dv > BSP_CAL_BIAS_MAX_DV) ? 2U : 1U;
                FaultSet(BSP_FAULT_CUR_SENSE);
            } else {
                s_bias_u   = bu;
                s_bias_w   = bw;
                s_cal_why  = 0U;
                s_cal_state = 2U;
            }
        }
    }

    s_iu = (vu - s_bias_u) * BSP_AMP_PER_VOLT;
    s_iw = (vw - s_bias_w) * BSP_AMP_PER_VOLT;
    if (iu) *iu = s_iu;
    if (iw) *iw = s_iw;

    /* 全程峰值留证：用最大绝对值，便于判断开通冲击到底有多大 */
    if (fabsf(s_iu) > fabsf(s_ovc_peak_u)) { s_ovc_peak_u = s_iu; s_ovc_raw_u = raw_u; }
    if (fabsf(s_iw) > fabsf(s_ovc_peak_w)) { s_ovc_peak_w = s_iw; s_ovc_raw_w = raw_w; }

    if (s_ovc_blank != 0U) {
        /*
         * 开通消隐：MOS 从全关到 50% 的第一个 PWM 周期，会给电机相间电容与
         * 电缆电容充电，1 mOhm + 30 倍增益下能采到 -11~-13 A、持续几十 us 的
         * 冲击。这是瞬态而非过流，实测就是在 MOE=1 那一刻跳闸、随后电流正常。
         * 消隐期间不判故障；真实过流（短路/堵转）会持续到消隐之后。
         */
        s_ovc_blank--;
        s_ovc_cnt = 0U;
    } else if (fabsf(s_iu) > BSP_CURRENT_TRIP_A || fabsf(s_iw) > BSP_CURRENT_TRIP_A) {
        if (++s_ovc_cnt >= BSP_OVC_DEBOUNCE) {
            FaultSet(BSP_FAULT_OVERCURRENT);
        }
    } else {
        s_ovc_cnt = 0U;                 /* 未连续超限：计数清零，不判故障 */
    }
}

/* ---------------- 电流零点标定接口 ---------------- */

uint8_t BspServo_StartCurrentCal(void)
{
    /* 有输出时标定 = 采到真实电流；电机在转时 = 采到反电动势引起的电流 */
    if (s_pwm_enabled != 0U) {
        return 0U;
    }
    if (fabsf(s_omega_mech) > 1.0f) {
        return 0U;
    }
    if (s_cal_state == 1U) {
        return 0U;
    }
    s_cal_sum_u = 0.0f;
    s_cal_sum_w = 0.0f;
    s_cal_n = 0U;
    s_cal_state = 1U;          /* 由 ReadInjected 在 20 kHz 节拍里累加，200 ms 后置 2 */
    return 1U;
}

uint8_t BspServo_IsCurrentCalibrated(void) { return (s_cal_state == 2U) ? 1U : 0U; }
uint8_t           BspServo_GetCalReject(void) { return s_cal_why; }
void BspServo_SetEncFaultMask(uint8_t on) { s_enc_flt_mask = (on != 0U) ? 1U : 0U; }

void BspServo_SetCurrentBias(float u, float w)
{
    /* 恢复路径：不采样，直接把存下来的零漂写回去。
     * s_cal_state 必须置 2（已完成），否则 C/Z/使能的联锁会拒绝一切操作；
     * 但"已完成"不等于"当前温度下准确"，所以状态行仍显示 cal=1，
     * 需要重新标定时再发一次 C 即可覆盖。 */
    s_bias_u = u;
    s_bias_w = w;
    s_cal_state = 2U;
}

void BspServo_GetCurrents(float *iu, float *iw)
{
    if (iu != 0) { *iu = s_iu; }
    if (iw != 0) { *iw = s_iw; }
}

void BspServo_GetCurrentBias(float *u, float *w)
{
    if (u != 0) { *u = s_bias_u; }
    if (w != 0) { *w = s_bias_w; }
}

void BspServo_GetOvcInfo(float *iu, float *iw, uint16_t *raw_u, uint16_t *raw_w)
{
    if (iu != 0)    { *iu = s_ovc_peak_u; }
    if (iw != 0)    { *iw = s_ovc_peak_w; }
    if (raw_u != 0) { *raw_u = s_ovc_raw_u; }
    if (raw_w != 0) { *raw_w = s_ovc_raw_w; }
}

void BspServo_ClearOvcInfo(void)
{
    s_ovc_peak_u = 0.0f;
    s_ovc_peak_w = 0.0f;
    s_ovc_raw_u  = 0U;
    s_ovc_raw_w  = 0U;
    s_ovc_cnt    = 0U;
}

float BspServo_GetVbus(void) { return s_vbus; }

/* ---------------- NTC 换算与采样 ---------------- */

/*
 * 构造静默 NaN（位模式 0x7FC00000）。
 * 不能直接写 NAN 宏：当前 AC6 优化选项下编译器认为 NaN/Inf 不存在
 * （-Wnan-infinity-disabled），宏会被折叠成不可预期值。位拷贝是明确定义行为。
 */
static float Ntc_NaN(void)
{
    uint32_t bits = 0x7FC00000u;
    float f;

    memcpy(&f, &bits, sizeof(f));
    return f;
}

/*
 * 码值 → 摄氏温度。r25/beta 按通道分别传入（两路 NTC 规格不同，见上面的宏）
 *   V = code/4095 * 3.3
 *   R = Rpull * V / (3.3 - V)                     （被测 NTC 在下臂）
 *   T = 1 / (1/T25 + ln(R/R25)/B) - 273.15
 * 边界：code==0（NTC 开路，V→0）、code>=4095（短路，V→3.3）都返回 NaN，
 *       上位机按 "--" 显示。宁可显示没有，也不显示一个编出来的温度。
 */
static float Ntc_CodeToCelsius(uint16_t code, float r25, float beta)
{
    float v;
    float r;
    float inv_t;

    if ((code == 0U) || (code >= 4095U)) {
        return Ntc_NaN();
    }
    v = (float)code * (BSP_ADC_VREF / BSP_ADC_COUNTS);
    r = NTC_R_PULL_OHM * v / (BSP_ADC_VREF - v);
    inv_t = (1.0f / NTC_T25_KELVIN) + (logf(r / r25) / beta);
    if (!(inv_t > 0.0f)) {                  /* 取反写法：NaN 也走这条 */
        return Ntc_NaN();
    }
    return (1.0f / inv_t) - NTC_KELVIN;
}

/*
 * ADC 常规组 DMA 完成回调（弱函数覆盖）。
 * 只认 ADC2：ADC1 常规组是 continuous+circular 的持续搬运，不经过这里。
 * 中断里只置状态位，logf 换算留到 1 ms 任务 —— 保持"ISR 不解析、不计算"的纪律。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2) {
        s_ntc_state = 2U;
    }
}

float BspServo_GetTempGan(void)   { return (s_ntc_ok != 0U) ? s_temp_gan   : Ntc_NaN(); }
float BspServo_GetTempMotor(void) { return (s_ntc_ok != 0U) ? s_temp_motor : Ntc_NaN(); }

uint8_t BspServo_GetNtcRaw(uint16_t *gan, uint16_t *motor)
{
    uint8_t ok = (s_ntc_ok != 0U) ? 1U : 0U;

    if (gan != 0)   { *gan   = s_ntc_raw[0]; }
    if (motor != 0) { *motor = s_ntc_raw[1]; }
    return ok;
}

/* ---------------- RGB 状态指示灯 ----------------
 * 网表实证（Netlist_PG_X72_CAN_2026-11-07.enet）：
 *   LED2(gge198) COM+ -> 3V3，R-/G-/B- 经限流电阻分别接 PC13/PC14/PC15。
 *   => 共阳，GPIO 写 0 点亮、写 1 熄灭；上电 MX_GPIO_Init 全置 1（灭）。
 * 状态优先级：故障 > 辨识 > 运行 > 待机。全部在 1 kHz 慢任务里推进。
 *   · 故障：红色快闪 5 Hz（200ms 周期，100ms 亮）—— 最醒目，优先于一切
 *   · 参数辨识：蓝色快闪 5 Hz（200ms 周期，100ms 亮）—— 电气/机械辨识进行中
 *   · 运行：绿色慢闪 2 Hz（500ms 周期，250ms 亮）
 *   · 待机：绿色常亮（板子上电健康、标定在位，只是没使能）
 * 状态切换时相位清零并先写"亮"，保证故障/辨识一出现灯立刻可见，不吞半个周期。
 */
#define BSP_LED_FAULT_HALF_MS   100U    /* 红：半周期 100ms -> 5 Hz */
#define BSP_LED_IDENT_HALF_MS   100U    /* 蓝：半周期 100ms -> 5 Hz */
#define BSP_LED_RUN_HALF_MS     250U    /* 绿：半周期 250ms -> 2 Hz */

/* 辨识忙判据集中于此：覆盖电气(MotorIdent/RlIdent/FluxIdent)、
 * 机械(MechIdent)、摩擦(FricCal)、编码器校准(EncCal)六个模块。
 * 每个模块自身用 IDLE/DONE/FAIL 表示结束，中间态即"进行中"。
 * 注：mech/fric 无 IsBusy()，用 GetState() 排除终态判定。 */
static uint8_t led_ident_busy(void)
{
    if (MotorIdent_IsBusy() != 0U)  { return 1U; }
    if (RlIdent_IsBusy() != 0U)     { return 1U; }
    if (FluxIdent_IsBusy() != 0U)   { return 1U; }
    if (EncCal_IsBusy() != 0U)      { return 1U; }
    {
        const MechIdentState ms = MechIdent_GetState();
        if ((ms != MI2_IDLE) && (ms != MI2_DONE) && (ms != MI2_FAIL)) { return 1U; }
    }
    {
        const FricCalState fs = FricCal_GetState();
        if ((fs != FC_IDLE) && (fs != FC_DONE) && (fs != FC_FAIL)) { return 1U; }
    }
    return 0U;
}

static uint16_t      s_led_phase_ms = 0U;
static uint8_t       s_led_last_mode = 0xFFU;   /* 0xFF = 未初始化，强制首拍刷新 */
static BspLedState_t s_led_state = BSP_LED_IDLE;

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    /* 共阳低有效：先把三路统一写 1（全灭），再点亮目标色，避免换色时串色。 */
    HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_SET);
    if (r != 0U) { HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET); }
    if (g != 0U) { HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_RESET); }
    if (b != 0U) { HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_RESET); }
}

void BspServo_LedTask1ms(void)
{
    /* 优先级：故障 > 辨识 > 运行 > 待机。
     * MOE 由使能命令/保护共同维护；辨识也会开 MOE，但必须归入"辨识"而非"运行"，
     * 所以辨识判据放在 MOE 之前。故障仍为最高优先级。 */
    const uint8_t mode = (s_fault != 0U) ? 2U
                       : (led_ident_busy() != 0U) ? 3U
                       : (s_pwm_enabled != 0U) ? 1U
                       : 0U;
    uint8_t on;

    if (mode != s_led_last_mode) {
        s_led_last_mode = mode;
        s_led_phase_ms  = 0U;               /* 切换即刻进入"亮"，不延迟 */
        s_led_state = (mode == 2U) ? BSP_LED_FAULT
                    : (mode == 3U) ? BSP_LED_IDENT
                    : (mode == 1U) ? BSP_LED_RUN
                    : BSP_LED_IDLE;
    }

    if (s_led_phase_ms < 0xFFFFU) { s_led_phase_ms++; }

    if (mode == 2U) {
        on = (s_led_phase_ms < BSP_LED_FAULT_HALF_MS) ? 1U : 0U;
        if (s_led_phase_ms >= (2U * BSP_LED_FAULT_HALF_MS)) { s_led_phase_ms = 0U; }
        led_set(on, 0U, 0U);                /* 红快闪：故障 */
    } else if (mode == 3U) {
        on = (s_led_phase_ms < BSP_LED_IDENT_HALF_MS) ? 1U : 0U;
        if (s_led_phase_ms >= (2U * BSP_LED_IDENT_HALF_MS)) { s_led_phase_ms = 0U; }
        led_set(0U, 0U, on);                /* 蓝快闪：参数辨识 */
    } else if (mode == 1U) {
        on = (s_led_phase_ms < BSP_LED_RUN_HALF_MS) ? 1U : 0U;
        if (s_led_phase_ms >= (2U * BSP_LED_RUN_HALF_MS)) { s_led_phase_ms = 0U; }
        led_set(0U, on, 0U);                /* 绿慢闪：运行 */
    } else {
        led_set(0U, 1U, 0U);                /* 绿常亮：待机 */
    }
}

BspLedState_t BspServo_LedGetState(void) { return s_led_state; }

void BspServo_SlowTask1ms(void)
{
    /* ADC1 常规 Rank1 = PA0(VBUS_ADC)，分压比 1/21（R64=100k、R63=100k、R65=10k）；
     * D12 钳位 3V3。注意：这里原来写"1/201"是早期算错留下的注释，
     * BSP_VBUS_DIVIDER 已经是 1/21，两者不一致曾让人误以为读数放大 10 倍。 */
    float vadc = (float)s_adc1_regular[0] * (BSP_ADC_VREF / BSP_ADC_COUNTS);
    s_vbus = vadc / BSP_VBUS_DIVIDER;

    vbus_protect();          /* 过压/欠压 + 20 ms 消抖，跳闸即清 MOE */

    /*
     * NTC：10 Hz 软启动一次 ADC2 常规组（Rank1=IN17 GAN，Rank2=IN5 MOTOR）。
     * 刻意不用 ContinuousConvMode / DMA 循环：ADC2 同时承担 20 kHz 注入采样
     * （W 相电流，TIM1_TRGO 触发），常规组若满速扫描会持续占用 ADC2 并产生
     * ~570 kHz 的 DMA 请求。这里每 100 ms 只换 2 个通道（约 1.7 us），
     * 占空比 <0.01%，对注入链路的影响可以忽略。
     */
    if (s_ntc_state == 2U) {                       /* 数据就绪 → 换算 */
        s_ntc_state = 0U;
        s_temp_gan   = Ntc_CodeToCelsius(s_ntc_raw[0], NTC_GAN_R25, NTC_GAN_BETA);
        s_temp_motor = Ntc_CodeToCelsius(s_ntc_raw[1], NTC_MOT_R25, NTC_MOT_BETA);
        s_ntc_ok++;
        s_ntc_div = 0U;
    } else if (s_ntc_state == 0U) {                 /* 空闲 → 到点启动 */
        if (++s_ntc_div >= NTC_PERIOD_MS) {
            s_ntc_div = 0U;
            if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_ntc_raw, 2U) == HAL_OK) {
                s_ntc_state = 1U;
            } else {
                s_ntc_fail++;                       /* HAL_BUSY 等：下一轮再来 */
            }
        }
    } else {                                        /* 转换中：等，超时判失败 */
        if (++s_ntc_div >= NTC_TIMEOUT_MS) {
            s_ntc_div = 0U;
            s_ntc_state = 0U;
            s_ntc_fail++;
        }
    }

    /* 温度降额：两路取更保守的一路，写进 s_derate 供控制环读。
     * 放在最后 —— NTC 换算刚在上面更新，用最新值。 */
    thermal_update();

    /* 编码器 SPI 自愈（2026-11-22）：连续坏帧（CRC/status/跳变）→ 重建 SPI1。
     * 两次 0x0002 历来只能靠整机软复位恢复，而现场没人会去点复位；
     * 这里改成几百微秒的局部动作，下一拍就能接上角度。
     * 放在 1 ms 任务而不是 20 kHz ISR：DeInit+Init 要百微秒级，ISR 里做会超时。 */
    if (s_enc_reinit_req != 0U) {
        s_enc_reinit_req = 0U;
        enc_spi_recover();
    }

    /* 状态指示灯放最后：用本拍最新的 s_fault / s_pwm_enabled 判定。 */
    BspServo_LedTask1ms();
}

/* ---------------- MT6835 角度（DMA 异步，ISR 只读缓存） ---------------- */

/* MT6835 帧的 CRC-8：poly X8+X2+X+1(=0x07)，ANGLE[20] 最先移入，
 * 覆盖 ANGLE[20:0] + STATUS[2:0] 共 24 位（Hardware/mt6835.txt:637）。 */
static uint8_t enc_crc8(uint32_t angle, uint8_t status)
{
    uint32_t data = ((angle & 0x1FFFFFUL) << 3) | ((uint32_t)status & 0x07UL);
    uint8_t  crc  = (uint8_t)BSP_ENC_CRC_INIT;
    int8_t   i;

    for (i = 23; i >= 0; i--) {
        const uint8_t bit = (uint8_t)((data >> (uint8_t)i) & 1UL);
        const uint8_t fb  = (uint8_t)(((crc >> 7) & 1U) ^ bit);
        crc = (uint8_t)(crc << 1);
        if (fb != 0U) { crc ^= 0x07U; }
    }
    return crc;
}

/* 一帧无效：CRC 错 / status 非 0 / 角度越界 / 跳变，四条路径统一走这里。
 * 分开维护计数会出现"某一类坏帧不触发自愈"的漏洞。 */
static void enc_bad_frame(void)
{
    if (s_enc_err_cnt < 0xFFU) { s_enc_err_cnt++; }
    if (s_enc_bad_run < 0xFFU) { s_enc_bad_run++; }
    s_enc_ok_run = 0U;

    if ((s_enc_flt_mask == 0U) && (s_enc_err_cnt >= BSP_ENC_ERR_MAX)) {
        FaultSet(BSP_FAULT_ENCODER);
    }
    /* 连续坏帧达阈值就请求重建。^ 自校准期间不重建 —— 它不读编码器，
     * 重建只会干扰开环拖动。 */
    if ((s_enc_flt_mask == 0U) && (s_enc_bad_run >= BSP_ENC_REINIT_N)) {
        s_enc_reinit_req = 1U;
    }
}

/* 重建 SPI1。调用者保证 s_spi_busy == 0（没有在途 DMA）。
 *
 * 为什么不是"只清 OVR 了事"：两次 0x0002（11-18、11-22）的表现都是
 * "帧看似合法但角度乱跳"，异常不在单个标志上，而是外设/DMA 状态已错位。
 * DeInit 会 FORCE_RESET 外设并走 MspDeInit（关时钟、复位 PB3/4/5、拆 DMA、
 * 关 NVIC），Init 再完整重建 —— 等价于把 SPI1 拉回刚上电的状态。
 * 代价约百微秒，只在持续坏帧时走一次。 */
static void enc_spi_recover(void)
{
    const uint32_t now = HAL_GetTick();

    if (s_spi_busy != 0U) { return; }         /* 有传输在跑，下拍再来 */

    /* 限流：100 ms 窗口内最多重建 3 次。
     * 必要性：若坏帧是"持续性的"（例如 CRC 初值配错、编码器没供电），
     * 不做限流就会每 20 帧请求一次，变成每 2.5 ms 重建一次 SPI ——
     * 既拖垮 1 ms 任务，也让角度永远接不上。限流后保持故障位等人工处理，
     * 这也是我们希望暴露而不是掩盖的情形。 */
    if ((uint32_t)(now - s_enc_rec_t0) >= 100U) {
        s_enc_rec_t0 = now;
        s_enc_rec_n  = 0U;
    }
    if (s_enc_rec_n >= 3U) {
        s_enc_reinit_req = 0U;
        return;
    }
    s_enc_rec_n++;

    s_spi_busy = 1U;                          /* 占锁：20 kHz ISR 不再发起新传输 */
    EncCsHigh();
    (void)HAL_SPI_DeInit(&hspi1);
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        s_spi_busy = 0U;
        FaultSet(BSP_FAULT_SPI);
        return;
    }

    /* 重建基准：不清 s_enc_seen，重建后第一帧会被判成跳变、立刻又进坏帧计数，
     * 刚恢复就开始下一轮拉锯。 */
    s_enc_seen     = 0U;
    s_enc_err_cnt  = 0U;
    s_enc_jump_cnt = 0U;
    s_enc_bad_run  = 0U;
    s_enc_ok_run   = 0U;
    s_spi_busy     = 0U;                      /* 放开，ISR 下一拍重新发起 */
}

void BspServo_ClearFaultBits(uint32_t mask)
{
    s_fault &= ~mask;
}

HAL_StatusTypeDef BspServo_StartEncoderDma(void)
{
    if (s_spi_busy) return HAL_BUSY;

    s_spi_busy = 1U;
    EncCsLow();                                  /* CSN 下降沿锁存 0x003~0x006 */
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(&hspi1, s_enc_tx, s_enc_rx,
                                                       BSP_ENC_FRAME_LEN);
    if (st != HAL_OK) {
        EncCsHigh();
        s_spi_busy = 0U;
        FaultSet(BSP_FAULT_SPI);
    }
    return st;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    EncCsHigh();
    s_spi_busy = 0U;

    /* rx[2]=ANGLE[20:13], rx[3]=ANGLE[12:5], rx[4]=ANGLE[4:0]|STATUS[2:0], rx[5]=CRC */
    uint32_t angle = ((uint32_t)s_enc_rx[2] << 13) |
                     ((uint32_t)s_enc_rx[3] << 5)  |
                     ((uint32_t)s_enc_rx[4] >> 3);
    uint8_t status = (uint8_t)(s_enc_rx[4] & 0x07U);

    if (status != 0U || angle >= (uint32_t)BSP_ENC_COUNTS) {
        /* 保留上一帧有效角度，避免坏帧污染控制环 */
        enc_bad_frame();
        return;
    }

#if (BSP_ENC_CRC_EN != 0)
    /* CRC 校验（2026-11-22 新增）：MT6835 自己给出 24 位的 CRC-8，而原实现
     * 拿了 rx[5] 却从不校验。缺了这一步，"数据错位但 status 恰好为 0"的帧
     * 会直接进控制环，只能靠下面的跳变检测事后发现 —— 11-18 与 11-22 两次
     * 0x0002 都是这个形态（诊断位 status=0、跳变饱和）。 */
    if (enc_crc8(angle, status) != s_enc_rx[5]) {
        enc_bad_frame();
        return;
    }
#endif

    /* 角度连续性（2026-11-17 新增）：
     * status 位和越界检查抓不到"位翻转但数值仍合法"的帧，而那种帧会让电角度
     * 直接跳变 —— 比丢帧危险得多（位置环会当成真实位移去追）。
     * 21 位角必须按模 2^21 取最短差值，否则每圈过零都会误判成 2^21 的跳变。
     * 阈值 1/32 圈：20 kHz 下 4000 rpm 的合法变化只有约 7000 count，余量 9 倍。 */
    if (s_enc_seen != 0U) {
        int32_t d = (int32_t)angle - (int32_t)s_enc_raw;
        const int32_t half = (int32_t)(BSP_ENC_COUNTS * 0.5f);

        if (d >  half) { d -= (int32_t)BSP_ENC_COUNTS; }
        if (d < -half) { d += (int32_t)BSP_ENC_COUNTS; }
        if (d < 0)     { d = -d; }

        if ((uint32_t)d > BSP_ENC_JUMP_MAX) {
            if (s_enc_jump_cnt < 0xFFU) { s_enc_jump_cnt++; }
            /* 跳变保留 3 帧即报警（比帧错误更敏感：位置环会把跳变当成真实位移去追） */
            if ((s_enc_jump_cnt >= BSP_ENC_JUMP_N) && (s_enc_flt_mask == 0U)) {
                FaultSet(BSP_FAULT_ENCODER);
            }
            enc_bad_frame();
            return;                       /* 丢这一帧，继续用上一帧角度 */
        }
    }
    s_enc_jump_cnt = 0U;
    s_enc_seen = 1U;
    s_enc_err_cnt = 0U;
    s_enc_bad_run = 0U;

    /* 连续有效帧够多 = SPI 确实恢复了 → 按位清掉编码器故障，不再需要人工软复位。
     * 只清这一位：过流/过压等更严重的故障不能被自愈顺手抹掉。 */
    if (s_enc_ok_run < BSP_ENC_OK_N) {
        s_enc_ok_run++;
        if (s_enc_ok_run >= BSP_ENC_OK_N) {
            BspServo_ClearFaultBits(BSP_FAULT_ENCODER);
        }
    }

    s_enc_raw = angle;
    s_enc_status = status;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;
    EncCsHigh();
    s_spi_busy = 0U;
    FaultSet(BSP_FAULT_SPI);
}

uint32_t BspServo_GetEncoderRaw(void)    { return s_enc_raw; }
uint8_t  BspServo_GetEncoderStatus(void) { return s_enc_status; }

/* 编码器故障的现场（2026-11-21）。置 BSP_FAULT_ENCODER 的两条路径完全
 * 不同——status≠0/角度越界（帧内容坏）与连续跳变（位翻转但数值合法）——
 * 只看一个故障位没法区分，而 30091 的 p[3] 正好空着：
 *   b0..2 status（MT6835 的 3 个状态位）
 *   b3..5 连续错误计数（饱和到 7）
 *   b6..7 连续跳变计数（饱和到 3） */
uint8_t  BspServo_GetEncDiag(void)
{
    uint8_t e = (s_enc_err_cnt > 7U) ? 7U : s_enc_err_cnt;
    uint8_t j = (s_enc_jump_cnt > 3U) ? 3U : s_enc_jump_cnt;

    return (uint8_t)((s_enc_status & 0x07U) | (uint8_t)(e << 3) | (uint8_t)(j << 6));
}
void BspServo_SetEncoderZero(float zero_rad)
{
    /* 零点偏置在非实时上下文归一化，保证控制节拍里只需一次减法 */
    float z = fmodf(zero_rad, BSP_TWO_PI);
    if (z < 0.0f) z += BSP_TWO_PI;
    s_enc_zero = z;
    s_enc_zero_ok = 1U;
}

/* 电角度是否已对齐（Z 成功过一次）。电流闭环模式的联锁条件：
 * 零点未标定时 Park 变换用的 θ 与磁场无关，闭环必然失控。复位后清零。 */
uint8_t BspServo_IsEncoderZeroed(void) { return s_enc_zero_ok; }
float   BspServo_GetEncoderZero(void) { return s_enc_zero; }
float    BspServo_GetOmegaMech(void)     { return s_omega_mech; }

/* 未滤波单拍角速度（rad/s，编码器坐标系）。带宽到 10 kHz，
 * 机械辨识专用；**不要**拿它做速度反馈——量化噪声 ±0.06 rad/s。 */
float    BspServo_GetOmegaFast(void)     { return s_omega_fast; }

float BspServo_GetThetaRad(void)
{
    /* 每个 20kHz 控制节拍调用一次：内部完成机械角、单圈回绕与速度低通。
     * 其它上下文（1ms 任务、遥测组帧）必须用 BspServo_GetThetaCached()，
     * 否则会以 50us 的分母去消化 1ms 的角差，把测速低通拖偏。 */
    float theta = (float)s_enc_raw * (BSP_TWO_PI / BSP_ENC_COUNTS) + s_enc_zero;
    /* 第一项 < 2pi，s_enc_zero 已归一化到 < 2pi，故最多减一次，执行时间确定 */
    if (theta >= BSP_TWO_PI) theta -= BSP_TWO_PI;

    /* 多圈累积（原始计数域差分，含回绕修正） */
    if (s_pos_first) {
        s_pos_first = 0U;
    } else {
        int32_t d = (int32_t)s_enc_raw - (int32_t)s_enc_prev;
        if (d >  (int32_t)(BSP_ENC_COUNTS * 0.5f)) { d -= (int32_t)BSP_ENC_COUNTS; }
        if (d < -(int32_t)(BSP_ENC_COUNTS * 0.5f)) { d += (int32_t)BSP_ENC_COUNTS; }
        s_pos_sum += (int64_t)d;
    }
    s_enc_prev = s_enc_raw;

    float d = 0.0f;
    if (s_theta_first) {
        s_theta_first = 0U;
    } else {
        d = theta - s_theta_prev;
        if (d >  3.14159265f) d -= BSP_TWO_PI;
        if (d < -3.14159265f) d += BSP_TWO_PI;
    }
    s_theta_prev = theta;
    s_theta = theta;
    s_omega_fast = d / BSP_TS_CURRENT;      /* 单拍差分，无低通 */

    /* 多圈机械角：**必须由整数计数重算**，不能用 float 逐拍累加。
     *
     * 旧实现是 `s_theta_multi += d`，看起来没问题，但 d 带编码器 ±1 count 量化
     * （2^21/圈 → 3.0 µrad）。静止时 d 大部分是 0、偶尔 ±1 count —— 这是**随机游走**：
     *   2 kHz × 5 s = 1e4 拍，σ ≈ 3 µrad × √1e4 ≈ 300 µrad。
     * 实测表现正是：位置环"定位偏差"在 ±300 µrad 量级，且同一目标每次上电后
     * 随运行时间增大（2026-11-17 精度测试：−580 / +288 µrad 两次不同）。
     *
     * s_pos_sum 是 int64 精确累加（整数域差分 + 回绕修正），由它重算的误差有界
     * （< 1 个 float ulp ≈ 0.6 µrad @ 5 rad）且**不累积**。
     * base 取第一拍的单圈角，保持与 s_theta 完全同零点 —— 位置环靠这一点
     * 才能和 30040 遥测的 enc_sum 对得上。 */
    if (s_tmulti_first != 0U) {
        s_tmulti_first = 0U;
        s_tm_base      = theta;
        s_theta_multi  = theta;
    } else {
        s_theta_multi = s_tm_base
                      + (float)((double)s_pos_sum * (6.28318530717958647692 / 2097152.0));
    }

    /* 速度：8 拍累计差分（详见 bsp_servo.h 的说明）。
     * 单拍差分在高分辨率编码器上并不更好——量化噪声 ±1 count 直接被 50 us 放大，
     * 实测让 30 rpm 的 omega 读数 std 达到 18.8 rpm，速度环没法用。 */
    /* ---- 编码器角度谐波补偿（机械角域，2026-11-23）----
     *
     * MT6835 的读数含**随转子位置重复**的误差。实测 1560 rpm、两次独立录波
     * 按角度分箱后曲线相关 0.999：整圈峰峰 7 mrad，其中 4 次/转 2.4 mrad、
     * 2 次 0.8 mrad、3 次 0.6 mrad。属确定性几何误差（磁钢/安装），可补偿。
     *
     * 为什么必须在**测速**这一层补：角度误差直接变成速度读数误差
     *   w_err = d(eps)/dt = k·A_k·ω·cos(kθ)
     * 1560 rpm 时 4 次/转项对应 7.08 rpm（实测速度纹波 7.05 rpm，吻合），
     * 速度环把它当真实误差，输出 ±0.36 A 的假力矩 → 又变成真的力矩脉动。
     * 1560 rpm 实测：补偿前 rms 10.7 rpm，用拟合系数离线补偿后 1.9 rpm。
     *
     * 系数来自 30098（EncComp_Eval 返回"加到电角度上的修正量"），这里除以
     * 极对数换成机械角。30098 未写入时 Eval 恒 0，本段对既有行为零影响。
     *
     * 只补速度，不动 s_theta_multi（整数脉冲累积）与位置环：位置链路的一致
     * 性（含 enc_sum 遥测）留给后续单独处理，避免一次改两处不好定位。 */
    {
        static float   s_ecm_prev;
        static uint8_t s_ecm_first = 1U;
        const float ecm = EncComp_Eval(theta) * (1.0f / BSP_ENC_COMP_POLES);

        if (s_ecm_first != 0U) {
            s_ecm_first = 0U;
        } else {
            const float dec = ecm - s_ecm_prev;

            /* 改系数会让 ecm 整体平移，那一拍不补，避免速度读数出现台阶。
             * 正常单拍变化远小于此：1560 rpm、4 次/转 @2.4 mrad 时约 2e-5 rad。 */
            if (fabsf(dec) < 0.002f) { s_spd_acc += dec; }
        }
        s_ecm_prev = ecm;
    }

    s_spd_acc += d;
    if (++s_spd_cnt >= BSP_SPEED_AVG_N) {
        const float w_raw = s_spd_acc / (BSP_TS_CURRENT * (float)BSP_SPEED_AVG_N);
        s_spd_acc = 0.0f;
        s_spd_cnt = 0U;
        s_omega_mech += BSP_SPEED_LPF_ALPHA * (w_raw - s_omega_mech);
    }
    return theta;
}

/* 只读：上一次控制节拍算出的单圈机械角（rad），不推进速度与累积 */
float BspServo_GetThetaCached(void) { return s_theta; }

/* 多圈机械角（rad，不折叠）。位置环用它替代单圈 theta_m，
 * 从而支持大范围绝对定位（±π 之外的移动不再被折叠到最近等价点）。 */
float BspServo_GetThetaMulti(void) { return s_theta_multi; }

/*
 * 只读：相对上电点的累积脉冲（多圈），除以 BSP_ENC_COUNTS 即圈数。
 *
 * s_pos_sum 是 int64，在 20kHz ISR 里 += ，在 1ms 任务里读：
 * Cortex-M4 上 64 位存取不是原子操作，若读侧被 ISR 抢占并写入，
 * 可能读到"高 32 位是旧值、低 32 位是新值"的撕裂值（跳 ±2^32 脉冲 ≈ ±2048 圈）。
 * 因此读侧用 PRIMASK 做一次性快照。调用频率仅 100 Hz，关中断窗口约 10 个周期，
 * 对 50us 控制周期的影响可忽略；若原状态本就关中断，则保持原状返回。
 */
int64_t BspServo_GetPosSum(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int64_t v = s_pos_sum;
    if (primask == 0U) {
        __enable_irq();
    }
    return v;
}
