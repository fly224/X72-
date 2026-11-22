/*
 * uart_link.h —— USART1 的二进制发送通道
 *
 * 2026-11-21：ASCII 通道整体删除（banner / ReportStatus / SendAscii /
 * TrySendAscii / AsciiBusy / ReportDiag / ReportRegs / Trace）。
 * 理由：默认配置下它们本来就靠 X72_ASCII_DEBUG=0 由链接器剔除，
 * 留着只让"到底哪条通道在说话"变模糊；而 ASCII 横幅会插进 MAVLink 流里，
 * 上位机得先跳过一串可读文本。
 *
 * 现在本模块只负责：
 *   1. MAVLink 应答帧的阻塞发送（TrySendBinary）
 *   2. UART 错误恢复（ORE/FE/NE/PE），防止 gState 卡在 BUSY_TX 后遥测永久停发
 *   3. HardFault 现场打印（系统已挂，此时 MAVLink 不可能工作，
 *      只剩寄存器轮询这一条路 —— 它不属于"正常运行时的通讯"，故保留）
 *
 * 不负责：MAVLink 组帧（telemetry_tx.c / mavlink_host.c）。
 */
#ifndef UART_LINK_H
#define UART_LINK_H

#include <stdint.h>

#define UART_LINK_FW_VER   "0.1.0-debug"

/* 复位链路计数。在 MX_USART1_UART_Init() 之后、BSP 初始化之前调用。 */
void UartLink_Init(void);

/* 二进制帧发送（MAVLink 应答）：暂停遥测 DMA → 逐字节写 TDR → 等 TC → 恢复。
 * 按 len 发送，不依赖 '\0' 结尾（MAVLink 载荷含 0x00）。
 * 忙时返回 0，调用方下一拍重试。 */
uint8_t UartLink_TrySendBinary(const uint8_t *buf, uint16_t len);

/* HardFault 现场打印：只用 USART1 寄存器轮询（不依赖 HAL、不依赖 SysTick），
 * 因此在 fault 上下文里也能出字。传 SCB 里的故障寄存器与本刻 MSP。 */
void UartLink_FaultDump(uint32_t cfsr, uint32_t hfsr, uint32_t mmfar,
                        uint32_t bfar, uint32_t msp);

/* 累计 UART 错误数（ORE/FE/NE/PE），供 30091 诊断读回。 */
uint32_t UartLink_GetErrorCount(void);

#endif /* UART_LINK_H */
