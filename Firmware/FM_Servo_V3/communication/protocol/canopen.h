#ifndef CANOPEN_H
#define CANOPEN_H

#include <stdint.h>

/* ===========================================================================
 * canopen.h —— 标准 CANopen 从站（CiA 301 + CiA 402），对齐雷赛 H2X-CAN506 手册
 *
 * 为什么单独一个模块而不是塞进 can_proto.c：
 *   can_proto.c 里那套是**自定义 CAN FD 协议**（0x180=CMD_MOTION、0x200=FB_MOTION、
 *   64 B 聚合帧）。标准 CANopen 的 COB-ID 是 0x181+n(TPDO1) / 0x201+n(RPDO1)，
 *   与它**直接冲突**，两套不能共存于同一条总线。所以这里独立实现，
 *   由 CANOPEN_ENABLE 决定是否启用；can_proto 的自定义 L1 默认关闭。
 *
 * 帧格式：标准 CANopen 是**经典 CAN 帧、最多 8 字节**。现有 can_l1_send() 发的是
 *   FDCAN_FD_CAN + BRS，标准主站不认，所以本模块自带 canopen_send() 走
 *   FDCAN_CLASSIC_CAN + BRS_OFF（FDCAN 外设本身支持同一总线上混合两种格式）。
 *
 * 阶段：本文件先实现 P1 —— 对象字典 + SDO + NMT + Heartbeat。
 *   PDO（0x201/0x181）与 PP/PV 模式接入留到 P3/P4，接口已预留。
 * ======================================================================== */

/* 节点号 1~127（0 保留给广播） */
#ifndef CANOPEN_NODE_ID
#define CANOPEN_NODE_ID     (1u)
#endif

/* 预定义连接集（CiA 301 附录 A，与手册 §2.3 一致） */
#define COB_NMT             (0x000u)
#define COB_SYNC            (0x080u)
#define COB_EMCY            (0x080u + CANOPEN_NODE_ID)
#define COB_TPDO1           (0x180u + CANOPEN_NODE_ID)
#define COB_RPDO1           (0x200u + CANOPEN_NODE_ID)
#define COB_TPDO2           (0x280u + CANOPEN_NODE_ID)
#define COB_RPDO2           (0x300u + CANOPEN_NODE_ID)
#define COB_SDO_TX          (0x580u + CANOPEN_NODE_ID)   /* 从 → 主：应答 */
#define COB_SDO_RX          (0x600u + CANOPEN_NODE_ID)   /* 主 → 从：请求 */
#define COB_HB              (0x700u + CANOPEN_NODE_ID)

/* NMT 状态（= 0x701 心跳报文的第一字节） */
typedef enum {
    NMT_BOOTUP      = 0x00,
    NMT_STOPPED     = 0x04,
    NMT_OPERATIONAL = 0x05,
    NMT_PREOP       = 0x7F
} canopen_nmt_t;

/* 操作模式（6060H），与手册 §3.1.1 一致 */
#define MODE_PP             (1)
#define MODE_PV             (3)
#define MODE_HOMING         (6)
/* MIT（阻抗）模式：CiA402 无标准定义，本实现用厂商扩展模式号 12，
 * 与固件内部 APP_OP_MODE_MIT 一致。参数对象见 canopen.c 的 0x2002~0x2004。 */
#define MODE_MIT            (12)

void     Canopen_Init(void);
void     Canopen_Task1ms(void);

/* 收到一帧时调用（由 FDCAN RX 分发器按 COB-ID 送给本模块）。
 * 返回 1 = 本模块已处理，0 = 不归我管。 */
uint8_t  Canopen_OnFrame(uint32_t cob, const uint8_t *d, uint8_t dlc);

/* 对外查询 */
uint16_t Canopen_StatusWord(void);      /* 0x6041 */
uint8_t  Canopen_NmtState(void);
uint16_t Canopen_PulsePerRev(void);     /* 0x2003，脉冲当量 */
int8_t   Canopen_Mode(void);            /* 0x6060 */

/* 反馈注入（P4 由控制环调用）：pulse 单位，输出端 */
void     Canopen_SetFeedback(int32_t pos_pulse, int32_t vel_pulse_s);

/* 主站自测模式（编译宏 CANOPEN_MASTER_TEST=1 时有效）：
 * 一块板当最小主站发 SDO 读、另一块当从站应答，用于双板验证链路。 */
#if defined(CANOPEN_MASTER_TEST) && (CANOPEN_MASTER_TEST != 0)
void     Canopen_MasterPoll1ms(void);
uint8_t  Canopen_MasterOnFrame(uint32_t cob, const uint8_t *d, uint8_t dlc);
void     Canopen_MasterGetStats(float *v8);
uint16_t Canopen_MasterRxOk(void);
uint16_t Canopen_MasterRxBad(void);
#endif

#endif /* CANOPEN_H */
