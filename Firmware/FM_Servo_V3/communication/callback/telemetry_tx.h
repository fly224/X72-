/*
 * telemetry_tx.h —— 上位机遥测发送（Iq / Id / 母线电压）
 *
 * 只发两条 MAVLink v2 报文，与 pg_x72_dialect 的字段定义一致：
 *   30025 CurrentCtlInput   iabc_now_A / dc_bus_now_V / idq_tar_A / elec_theta_rad …
 *   30027 CurrentCtlOutput  idq_now_A（Id / Iq 实测）
 */
#ifndef TELEMETRY_TX_H
#define TELEMETRY_TX_H

#include <stdint.h>

/* 初始化发送上下文；由 Servo_Init() 调用 */
void     Telemetry_Init(void);
/* 打开 100 Hz 发送闸门：由 Servo_Init() 在打印完启动状态行之后调用，
 * 保证 [BOOT] 文本不会被 DMA 占住 UART 而丢掉。 */
void     Telemetry_Start(void);

/* 1 ms 周期调用；内部 10 分频，即 100 Hz 发一组（30025+30027+30040 = 133 B） */
void     Telemetry_Task1ms(void);

uint32_t Telemetry_GetTxCount(void);

/* 推送一条辨识结果事件帧（30092）：
 *   kind 见 mavlink_common.h 的 MAV_ID_*；v8 为 8 个语义随 kind 的浮点。
 * 内部连发 5 帧（0.5 s @100 Hz）—— 协议层没有 ACK，靠重复保证不丢。
 * 只保留最近一条，不排队：辨识是串行的，同一时刻不会有两个结果。 */
void     Telemetry_PushIdentResult(uint8_t kind, uint8_t ok, uint16_t n,
                                   const float *v8, float resid);
/* ASCII 行要独占 UART 时置 1：TIM6 里的 100 Hz DMA 会主动让路 */
void     Telemetry_SetTxPause(uint8_t pause);
uint32_t Telemetry_GetDropCount(void);
uint32_t Telemetry_GetRecoverCount(void);

/* 最近一次 HAL_UART_Transmit_DMA 启动失败的现场快照：(gState << 4) | hdmatx->State。
 * 0x00 = 从未失败；0x0F 低半字节 = hdmatx 为空。供 [DIAG] 定位 HAL_BUSY 来源。 */
uint8_t  Telemetry_GetBusySnap(void);

#endif /* TELEMETRY_TX_H */
