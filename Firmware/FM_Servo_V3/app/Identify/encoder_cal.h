/*
 * encoder_cal.h —— MT6835 客户端自校准（利用芯片内置的自校准能力）
 *
 * 为什么优先用芯片自校准，而不是软件查表：
 *   MT6835 自带三层校准（出厂 / 客户端自校准 / NLC 对拖）。客户端自校准只要
 *   匀速转 64 圈、把 CAL_EN 拉高，芯片就会自己算出由磁铁偏差与安装偏心引起的
 *   非线性并写进 EEPROM，INL 从 ±0.5° 降到 ±0.07°。这是**芯片内部完成**的，
 *   不占 CPU、不占 Flash、精度还比软件 LUT 高。
 *
 *   参考实现对比（2026-11-16 查证）：
 *     · VESC：只有 enc_corr[360] 的**应用**逻辑（mcpwm_foc.c:3256，按机械角
 *       0~359° 查表后从电角度减掉）+ 掉电存储，固件里**没有生成这张表的算法**，
 *       表由 VESC Tool / Lisp 从外部写。
 *     · FM_XC：只有 elec_angle_compensation 一个常数偏置（motor_ctl_loop.c:74），
 *       没有偏心补偿。
 *     · Infineon IMR 与若干开源 FOC：软件 LUT（开环转一圈记录传感器读数，
 *       与理想线性曲线比较，128~1024 点，正反各一遍抵消摩擦/齿槽）。
 *
 *   所以软件 LUT 是"没有芯片自校准时的退路"，而 MT6835 有，就用它。
 *   CAL_EN 已在硬件上接到 PB9（网表 pin_map.csv:48 / 295）。
 *
 * ⚠ 厂商明确的收尾要求（手册 9.2 第 5、6 步）：
 *   成功后等待 >6 秒，**必须给芯片断电**；重新上电后才能做别的操作。
 *   固件无法自己断 3.3 V（板上没有编码器电源开关），所以这一步只能提示用户。
 *
 * 硬件约束（本板）：
 *   · AUTO_CAL_FREQ 用出厂默认 0x03 = 400~800 rpm，所以校准转速取 600 rpm；
 *   · 600 rpm 空载 64 圈 = 6.4 s；
 *   · 校准期间速度环必须稳在区间内，否则芯片会判失败（0x113[7:6]=10）。
 */
#ifndef ENCODER_CAL_H
#define ENCODER_CAL_H

#include <stdint.h>

typedef enum {
    EC_IDLE = 0,
    EC_SPINUP,      /* 启动速度环，等转速进入区间 */
    EC_SETTLE,      /* 稳定若干秒，确认匀速 */
    EC_CAL,         /* CAL_EN 已拉高，累计圈数并轮询状态 */
    EC_DONE,
    EC_FAIL,
} EncCalState;

typedef enum {
    EC_OK = 0,
    EC_ERR_BUSY = 1,
    EC_ERR_CAL = 2,          /* 电流零点未标定 */
    EC_ERR_ZERO = 3,         /* 电角度未对齐 */
    EC_ERR_FAULT = 4,
    EC_ERR_PWM = 5,
    EC_ERR_SPEED = 6,        /* 转速没进入 400~800 rpm 区间 */
    EC_ERR_TURNS = 7,        /* 圈数到不了（堵转 / 限流） */
    EC_ERR_CHIP = 8,         /* 芯片上报校准失败 (0x113[7:6]=10) */
    EC_ERR_TIMEOUT = 9,
} EncCalErr;

typedef struct {
    float    target_rpm;
    float    rpm_avg;        /* 校准期间平均转速 */
    float    rpm_min;
    float    rpm_max;
    uint32_t turns_x10;      /* 累计圈数 ×10（用绝对值，单向运行） */
    uint32_t elapsed_ms;
    uint8_t  chip_status;    /* 最后一次读到的 0x113[7:6]：0/1/2/3 */
    uint8_t  ol_mode;        /* 1 = 闭环起不来，已切开环拖动完成本次校准 */
    uint8_t  auto_cal_freq;  /* 0x00E[3:1] 自校准转速区间（出厂默认 0x03=400~800rpm）。
                              * 诊断 chip_status=2 用：若被 EEPROM 写坏成别的值，
                              * 700rpm 会落在区间外 → 芯片立即判失败。 */
    EncCalErr err;
} EncCalResult;

void           EncCal_Init(void);
EncCalErr      EncCal_Start(float rpm);
void           EncCal_Abort(void);
uint8_t        EncCal_Task1ms(void);     /* 1 = 状态推进或结束 */
EncCalState    EncCal_GetState(void);
uint8_t        EncCal_IsBusy(void);
const EncCalResult *EncCal_GetResult(void);
const char    *EncCal_ErrStr(EncCalErr e);

#endif /* ENCODER_CAL_H */
