/*
 * servo_cmd.h —— 控制命令层（MAVLink 动作入口 + 联锁）
 *
 * 2026-11-21：ASCII 单字符通道已整体删除，USART1 只跑 MAVLink v2。
 * 本文件现在提供两件事：
 *   1. 控制装配与联锁（Arm/Disarm/ClearFault/CtrlTorque/CtrlSpeed/CtrlPos/CtrlMit）
 *   2. 校准/辨识的统一动作入口 ExecAction（30069 直调）
 *
 * 设计约束：
 *   1. 收到字节的中断（USART1，优先级 8）只把字节放进环形队列，不做解析；
 *   2. 解析与分发放在主循环，不占 20 kHz 节拍；
 *   3. deadman 由 TIM6 1 ms 中断递减，到点只置 kill 标志，主循环执行封锁。
 */
#ifndef SERVO_CMD_H
#define SERVO_CMD_H

#include <stdint.h>

/* 打开 USART1 的 RXNE 中断（不启动 HAL 接收：HAL 的接收会占用 huart1.gState，
 * 与 100 Hz 遥测 DMA 发送互斥，两者不能共存） */
void    ServoCmd_Init(void);

/* USART1 中断里调用：只保存字节与标志 */
void    ServoCmd_Feed(uint8_t b);

/* 主循环调用：解析、执行、回显、处理 deadman 触发的封锁 */
void    ServoCmd_Poll(void);

/* TIM6 1 ms 调用：deadman 倒计时（不做任何 IO） */
void    ServoCmd_Task1ms(void);

uint8_t ServoCmd_IsArmed(void);
/* 命令环形缓冲溢出丢字节计数（2026-11-21）。原来溢出是静默丢弃，
 * 一帧 156 B 的请求掉几个字节后，固件侧 CRC 错/pending 顶替都不计数，
 * 表现成"偶发无应答"无法定位。此计数在 30082 的只读槽 q_ovf。
 * 正常工况应恒为 0；一旦增长，说明主循环有超过缓冲窗口（256 B ≈ 2.8 ms）
 * 的阻塞，要去查那个阻塞源，而不是加大缓冲掩盖它。 */
uint16_t ServoCmd_GetRxOverflow(void);

/* 由 USART1_IRQHandler 调用：累加 ORE/FE/NE/PE（供 30091 诊断读回） */
void    ServoCmd_NoteUartErr(void);

/* ---- 供 MAVLink 协议层调用的受控入口（2026-11-18）----
 * 这些函数复用 E/D/X 命令完全相同的联锁（故障/标定/零点/MOE 回读），
 * 协议层不得绕过它们直接改 Servo_* 内部变量。
 * 返回值 = 1 表示操作成功，0 表示被联锁拒绝。执行时仍会 emit 一行 ASCII 回显。 */
uint8_t ServoCmd_Arm(void);                 /* = E 命令 */
uint8_t ServoCmd_Disarm(void);              /* = D 命令 */
uint8_t ServoCmd_ClearFault(void);          /* = X 命令（仅在未使能时有效） */

/* ---- 4 控制模式给定入口（供 MAVLink 经 app_param 调用，2026-11-18）----
 * 完全复用 M/N/W/Q/P 命令的装配联锁（故障/标定/零点/辨识/MOE 回读）与顺序。
 * 行为：
 *   - 已在目标环 → 只更新给定，不重开 MOE（保活重发不会反复通断）；
 *   - 不在目标环且已使能 → 先自动封锁（等价 D）再装配，避免两环同开竞态；
 *   - 未使能 → 直接完整装配。
 * 返回 1=成功（ armed 且目标环在环），0=被联锁拒绝。执行时会 emit 一行 ASCII 回显。 */
uint8_t ServoCmd_CtrlTorque(float iq_a);
uint8_t ServoCmd_CtrlSpeed(float w_rad_s);   /* 入参 rad/s（内部换算 rpm） */
uint8_t ServoCmd_CtrlPos(float p_rad);       /* 入参机械角 rad（编码器坐标系） */
uint8_t ServoCmd_CtrlMit(float p, float v, float kp, float kd, float tau);

/* 当前实际在环模式（供状态字判定）: 0=未使能 1=位置 3=速度 4=力矩 12=MIT */
int8_t ServoCmd_ActiveMode(void);


/* ---- 统一动作入口（飞控协议 30069 与 ASCII 单字符共用同一实现）----
 * 2026-11-21：为"放弃 ASCII 单字符命令、改用飞控协议"加的过渡层。
 *
 * 实现方式是**合成 ASCII 命令注入**：把动作映射回等价字符，喂进现有解析器。
 * 好处：零逻辑复制，行为与手工敲命令逐字节一致，40+ 条命令一次全部可达。
 * 代价：暂时仍会 emit ASCII 回显；且动作是"入队"而非同步完成。
 *
 * 下一阶段把各分支的逻辑下沉成返回状态码的函数后，ASCII 接收路径即可删除，
 * 届时本函数的实现会从"注入字符"换成"直接调用"，对外接口不变。
 *
 * 返回值：见下面的 SC_EXEC_*。1 只表示"已入队"，不代表执行成功 ——
 * 结果要靠状态遥测（30090/30091/30092）确认。
 * 编号 4..14 与 APP_SYS_CMD_* 的 1/2/3 连号，30069 直接透传即可。 */
#define SC_ACT_CAL_CURRENT   (4U)    /* C  电流零点标定（200 ms，需停机） */
#define SC_ACT_ALIGN_ZERO    (5U)    /* Z  电角度对齐（约 22 s，需 cal=1） */
#define SC_ACT_IDENT_FLUX    (6U)    /* +  磁链 / Kt 辨识 */
#define SC_ACT_IDENT_RL      (7U)    /* @  R / L 辨识 */
#define SC_ACT_ENC_CAL       (8U)    /* ^  编码器线性度自校准（开环拖动） */
#define SC_ACT_FRIC_2PT      (9U)    /* $  摩擦两点标定（写前馈 + 使能） */
#define SC_ACT_MECH_SWEEP    (10U)   /* (  机械扫频 20..1000 Hz */
#define SC_ACT_MECH_FRIC     (11U)   /* )  摩擦曲线 8 点 1..200 rpm */
#define SC_ACT_MECH_INERTIA  (12U)   /* <  折算惯量 */
#define SC_ACT_MECH_COG      (13U)   /* }  齿槽转矩 128 箱 */
#define SC_ACT_ABORT_IDENT   (14U)   /* 中止所有正在跑的辨识 */
/* 20 = CAN FD 内环自检（2026-11-22）。结果走 30092 kind=10，
 * 返回 SC_EXEC_ACCEPT 只表示"已执行"，通过与否看 30092 的 ok 位。 */
#define SC_ACT_CAN_SELFTEST  (20U)
/* 21/22/23 = CAN 双节点链路测试：21 主发、22 从收、23 停止并上报。
 * 持续跑到 23 或 30 s 超时，方便两个节点轮流接同一个串口。 */
#define SC_ACT_CAN_TEST_TX   (21U)
#define SC_ACT_CAN_TEST_RX   (22U)
#define SC_ACT_CAN_TEST_STOP (23U)

/* ExecAction 返回码（2026-11-21 细分）。
 * 原来只有 1/0 两种，于是"还在闭环里、请先 D"和"没有这个动作号"都表现成
 * 同一个 0 → 30069 回 OUT_OF_RANGE。现场看到 OUT_OF_RANGE 只会怀疑动作号写错，
 * 不会想到是联锁拒绝 —— 这类"两种原因一个码"正是最难查的。
 * 语义与 ASCII 时代一致（更严一点：ASCII 的 C/Z 只查 moe，这里还要求 armed=0），
 * 从严是故意的：armed=1 表示上位机**打算闭环**，此时插一次标定等于在环里改零点。 */
#define SC_EXEC_REJECT       (0U)    /* 动作号非法 / 子模块拒绝 */
#define SC_EXEC_ACCEPT       (1U)    /* 已入队 */
#define SC_EXEC_NEED_DISARM  (2U)    /* 仍在闭环（armed=1）：先封锁再来 */

uint8_t ServoCmd_ExecAction(uint8_t act);

#endif /* SERVO_CMD_H */
