#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * can_proto.c —— CAN FD 最小协议层（阶段 0：内环自检）
 *
 * 目标：在不接 CAN 分析仪、不接第二个节点的前提下，验证
 *   ① FDCAN 内核时钟 85 MHz / 1 Mbit/s 标称 + 5 Mbit/s 数据段能否 Init；
 *   ② BRS 打开后 8/16/24/32/48/64 六种 DLC 能否自发自收且内容一致；
 *   ③ 过滤器、FIFO 深度、HAL 收发路径是否正常。
 *
 * 内部回环（FDCAN_MODE_INTERNAL_LOOPBACK）不需要外部 ACK，也不经过收发器，
 * 所以它只能证明"固件配置对"，不能证明"线束和端接对"。后者要靠双节点互发。
 * ------------------------------------------------------------------------- */

void    CanProto_Init(void);        /* 正常模式启动（1M/5M、BRS 开） */
void    CanProto_Stop(void);
uint8_t CanProto_IsReady(void);

/* 内环自检：1=全部通过。结果同时以 30092 kind=MAV_ID_CAN 上报。
 * 会短暂 Stop/DeInit/Init 两次，期间不响应总线，调用前须 disarm。 */
uint8_t CanProto_SelfTest(void);

void    CanProto_GetStats(uint16_t *tx, uint16_t *rx, uint16_t *err,
                          uint8_t *tec, uint8_t *rec, uint8_t *state);

/* ---- 双节点链路测试（阶段 1）----
 * role 1 = 主发：每 1 ms 发一帧 0x100/24 B，seq 递增，最长 30 s
 * role 2 = 从收：1 ms 轮询 RX，按 seq 统计丢帧/重复，最长 30 s
 * 停止（含超时）时以 30092 kind=MAV_ID_CAN_TEST 上报。 */
/* ---- L1 正式协议层（对齐 X72关节模组_CANFD协议_工程落地_v1.1 第 4 节）----
 * ID：SYNC 0x080 / CMD_AGG 0x100(64B) / CMD_MOTION 0x180+n /
 *     FB_MOTION 0x200+n(24B) / FB_DIAG 0x300+n(48B) / HEARTBEAT 0x380+n(8B)
 * 失效安全：连续 10 ms 无有效 CMD_AGG 置超时（上层进阻尼）。
 * selftest=1 时每 1 ms 自发一帧 CMD_AGG 并逐字段回验（内部回环用）。 */
void    CanProto_L1Start(uint8_t selftest);
void    CanProto_L1Stop(void);
void    CanProto_L1Task1ms(void);

void    CanProto_Task1ms(void);
uint8_t CanProto_TestStart(uint8_t role);
void    CanProto_TestStop(void);

#endif
