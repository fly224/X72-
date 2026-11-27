/* canopen_bridge.h —— 串口(MAVLink) → CANopen 网关（A 板作主站，B 板作从站）
 *
 * 数据流
 * ------
 *   上位机 --MAVLink/USART1--> A 板
 *        30056 模式        → SDO 写 0x6060 (i8)
 *        30054 控制字      → SDO 写 0x6040 (u16)
 *        30019 位置/速度   → SDO 写 0x607A (i32) / 0x60FF (i32)
 *        30025 力矩        → SDO 写 0x2004 (i32, mNm)
 *              ↓ SDO 队列 → 1ms 状态机 → FDCAN1 (0x601 请求)
 *   B 板 --CANopen--> A 板
 *        0x581 SDO 应答（0x60 成功 / 0x80 abort）
 *        0x181 TPDO1（0x6041 状态字 + 0x6064 位置）
 *        0x701 心跳（NMT 状态）
 *              ↓ CanBridge_OnFrame → 状态缓存 → 30090/30091 回传上位机
 *
 * 为什么第一版用 SDO 而不用 RPDO
 * ------------------------------
 * SDO 有应答、能报 abort code，且从站 OD 已就绪，不必先配 0x1400/0x1600 映射
 * ——映射配错的典型现象是"帧发了、从站毫无反应"，很难查。代价是单次事务约
 * 0.3 ms 往返，因此**给定更新限 100 Hz（10 ms）**；从站内部位置环仍是 2 kHz，
 * 不受影响。RPDO 留作后续优化（届时注意 0x201 与 L1 FB_MOTION 的 ID 冲突）。
 *
 * 三个必须知道的前置条件
 * ----------------------
 * 1. **从站上电是静默的**（canopen.c 的 Task1ms 注释）：B 板只有收到 NMT Start
 *    才会发 Boot-up / 心跳 / TPDO。所以 A 板必须周期发 NMT Start，否则总线上
 *    一条报文都没有，现象会像"CAN 硬件坏了"。
 * 2. **单事务串行**：同一时刻只有一条 SDO 在飞。并发会在 0x581 上撞车——应答
 *    只有 COB-ID 没有序号，无法区分是哪条请求的回执。
 * 3. **控制字直通**：CiA402 的使能序列（FaultReset→Shutdown→SwitchOn→Enable）
 *    由上位机按 0x6041 状态字推进，本模块不自动帮它跑状态机——自动推进容易在
 *    故障未清时强行上使能，比不动更危险。
 *
 * 编译开关：CANOPEN_BRIDGE=1 时启用（见 canopen.h）。
 * 启用后 A 板**不再作为 CANopen 从站**（Canopen_Init/Task1ms 在本模式下不调用），
 * 避免它与主站抢 0x181 TPDO。
 */

#ifndef CANOPEN_BRIDGE_H
#define CANOPEN_BRIDGE_H

#include <stdint.h>

/* 上电初始化：清空队列与状态缓存 */
void     CanBridge_Init(void);

/* 1 ms 节拍：NMT Start 周期发送 + SDO 状态机推进 + 超时重试 */
void     CanBridge_Task1ms(void);

/* CAN 接收分发（只吃 0x581 / 0x181 / 0x701，返回 1 表示已消费该帧） */
uint8_t  CanBridge_OnFrame(uint32_t cob, const uint8_t *d, uint8_t dlc);

/* ---- 指令注入（mavlink_host.c 调用）---- */
void     CanBridge_SetMode(int8_t mode);            /* 0x6060 */
void     CanBridge_SetControlWord(uint16_t cw);     /* 0x6040 */
void     CanBridge_SetTargetPos(int32_t pulse);     /* 0x607A，单圈 2^21 计数 */
void     CanBridge_SetTargetVel(int32_t pulse_s);   /* 0x60FF */
void     CanBridge_SetTorque(int32_t mnm);          /* 0x2004，毫牛·米 */

/* ---- 状态查询（遥测组帧用）---- */
void     CanBridge_GetStatus(uint16_t *sw, int32_t *pos, uint8_t *nmt);

/* ---- 诊断计数（30092 kind=11 可上报）----
 * ok=应答成功 bad=abort tmo=超时放弃 q=当前排队深度 */
void     CanBridge_GetStats(uint16_t *ok, uint16_t *bad, uint16_t *tmo,
                            uint8_t *q_depth, uint32_t *last_abort);

#endif /* CANOPEN_BRIDGE_H */
