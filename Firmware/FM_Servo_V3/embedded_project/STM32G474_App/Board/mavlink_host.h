#ifndef MAVLINK_HOST_H
#define MAVLINK_HOST_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * mavlink_host.h —— MAVLink 主机指令接收与分发（2026-11-18，首期操作类消息）
 *
 * 职责（严格对齐《固件协议层_职责与编写指南.md》）：
 *   收：USART1 字节流中识别 0xFD 帧 → 流式解析 → X25 CRC 校验 → 完整帧入队；
 *   分发：MESSAGE → decode → 权限/状态检查（app_param set_app_*）→ 回读 → encode 回发；
 *   不在此文件做控制运算、不直接改 Servo_* 变量。
 *
 * 接入点：
 *   - ServoCmd_Poll() 的字节循环里，0xFD 帧头交给 MavHost_FeedByte()；
 *   - 主循环 ServoCmd_Poll() 之后调 MavHost_Task() 做分发与应答发送。
 *   - AppParam_Init() 在 Servo_Init 里调；MavHost_Init() 同理。
 *
 * 首期实现消息（其余 82 条按"未实现静默丢弃"，符合指南第 5 节）：
 *   30000 READ_PARAM / 30054 AppControlWord / 30056 AppOpMode
 *   30068 AppPermissionConfig / 30069 SystemCmd
 * 遥测（30025/30027/30040）仍由 telemetry_tx.c 发，本层不碰。
 * ------------------------------------------------------------------------- */

void MavHost_Init(void);

/* 从 USART1 收字节流喂入帧解析器（在 ServoCmd_Poll 的字节循环里调用）。
 * 返回 1 = 该字节已被 MAVLink 接收器消费（0xFD 帧首，或正处于收帧中的载荷/CRC）；
 * 返回 0 = 与 MAVLink 无关（空闲且非 0xFD），调用方应按普通字节处理。
 * ★ 调用方必须把返回 1 的字节从 ASCII/MIT/POSF 分派里排除 —— payload 里可能
 *   出现 0x02/0x04 等引导字符，若被当作帧头会把 MAVLink 帧劈开。 */
uint8_t MavHost_FeedByte(uint8_t b);

/* 主循环调用：若有完整帧则分发（decode→权限→set→回读→encode 应答）。
 * 非阻塞：UART 忙时保留应答到下一拍重试。 */
void MavHost_Task(void);

/* 诊断计数（供状态行/上位机查询）：ok=已处理帧, badcrc=CRC 错, unknown=未登记 msgid,
 * locked=被权限/状态门卫拒绝, txdrop=应答发送忙丢弃。 */
void MavHost_GetStats(uint16_t *ok, uint16_t *badcrc, uint16_t *unknown,
                      uint16_t *locked, uint16_t *txdrop);

/* 收帧侧丢帧细分（2026-11-21）。为什么单独一个接口：
 * 现象是"某条 156 B 请求偶发没有应答，遥测照常"——可能有两处：
 *   crc_err = 帧长度/CRC 不符（字节在链路上丢了，或长度与 s_meta 不一致）
 *   drop    = 上一帧还没被取走，新帧就把 s_frame 顶掉了（主循环来不及）
 * 两处的对策完全不同（前者查链路/中断延迟，后者查主循环耗时），
 * 所以必须分开计数，不能合成一个"丢帧"。
 * 通过 30082 的 int 槽 9（link_diag，只读）暴露给上位机。 */
void MavHost_GetRxDiag(uint16_t *crc_err, uint16_t *drop);

#endif /* MAVLINK_HOST_H */
