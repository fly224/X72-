#ifndef MAVLINK_COMMON_H
#define MAVLINK_COMMON_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * mavlink_common.h —— MAVLink v2 公共层：X25 CRC + 首期消息元数据 + 帧组装/解析
 *
 * 依据：
 *   - 上位机 host/mavlink_proto.py（X25 覆盖 buf[1..9] 含 SEQ + payload + crc_extra，
 *     不含 0xFD 魔数），与 telemetry_tx.c 的 frame_build 完全同一口径。
 *   - host/mavlink_layouts.json（91 条消息表）；本层只登记首期操作类 9 条，
 *     其余消息按"未实现静默丢弃"处理，符合《固件协议层指南》第 4/5 节。
 *
 * 帧格式（MAVLink v2）：
 *   0xFD | LEN | INCOMPAT | COMPAT | SEQ | SYSID | COMPID | MSGID[3,LE] | PAYLOAD[LEN] | CRC16[2,LE]
 *   CRC = X25(buf[1 .. 9+LEN-1]) 再累加 crc_extra(msgid)
 *
 * 设计纪律：
 *   - 全部静态分配，无 malloc；
 *   - CRC/组帧函数供收发两侧共用，避免收发各写一份口径漂移（历史上踩过含魔数口径的坑）；
 *   - msgid 元数据集中一张表，增删消息只改这一处。
 * ------------------------------------------------------------------------- */

#define MAV_STX             (0xFDu)
#define MAV_HEADER_LEN      (10u)     /* STX + LEN + 6 头字节 + MSGID[3] = 10 */
#define MAV_CRC_LEN         (2u)
#define MAV_MAX_PAYLOAD     (192u)    /* 2026-11-20：131 → 192，容纳 30082 AppDebugParam
                                       * （16f+16u32+16i32）。帧长 12+192=204 < 256，
                                       * s_rx_i 是 uint8 仍够用；s_frame/s_ack 各涨 61 B。
                                       * 原注释：30061 AppMotionInfo 是首期最长载荷 131。 */
#define MAV_MAX_FRAME       (MAV_HEADER_LEN + MAV_MAX_PAYLOAD + MAV_CRC_LEN)

/* 首期登记的消息 ID */
#define MAV_MSG_READ_PARAM        (30000u)
#define MAV_MSG_APP_CONTROL_WORD  (30054u)
#define MAV_MSG_APP_OP_MODE       (30056u)
#define MAV_MSG_APP_PERMISSION    (30068u)
#define MAV_MSG_SYSTEM_CMD        (30069u)
#define MAV_MSG_APP_STATUS_INFO   (30055u)
#define MAV_MSG_APP_IDENT_RESULT  (30092u)  /* 40B：辨识结果事件帧，替代 IDENT/[MI]/FC/RL 文本 */
#define MAV_MSG_APP_COG_TABLE     (30093u)  /* 48B：齿槽表分块回读（请求-应答） */
#define MAV_MSG_APP_RECORD        (30094u)  /* 144B：录波 触发/配置/分块回传 */
#define MAV_MSG_APP_TORQUE_CMD    (30095u)  /* 12B：力矩模式的 id 给定与斜坡时间常数 */
#define MAV_MSG_APP_ADV_CFG       (30096u)  /* 24B：速度环陷波配置（f0/深度/Q），带回读 */
#define MAV_MSG_APP_SPEED_TRAJ    (30097u)  /* 32B：速度轨迹"到达时间"优先配置，带回读 */
#define MAV_MSG_APP_ENC_COMP      (30098u)  /* 136B：编码器角度软件补偿（16 阶谐波，带回读） */
#define MAV_MSG_APP_POS_STATUS    (30100u)  /* 48B：位置环状态（位置/目标/误差/状态机），100 Hz 遥测 */
#define MAV_MSG_APP_LOOP_STATUS   (30091u)  /* 48B：环路状态，替代 ASCII 的 K 行 */
#define MAV_MSG_APP_CTRL_STATUS   (30090u)  /* 40B：控制状态字，替代 ASCII 的 S 行 */
#define MAV_MSG_APP_DEBUG_PARAM   (30082u)  /* 192B：16f+16u32+16i32，运行时调参口 */
#define MAV_MSG_APP_MOTION_INFO   (30061u)
#define MAV_MSG_HEARTBIT          (30077u)
#define MAV_MSG_CUSTOM_INFO       (30076u)
/* 4 控制模式给定消息（2026-11-18） */
#define MAV_MSG_POS_SPEED_CTL_IN  (30019u)  /* 位置/速度环输入：pos_tar_p / speed_tar_p_s */
#define MAV_MSG_CURRENT_CTL_IN    (30025u)  /* 电流环输入：idq_tar_A[2] */
#define MAV_MSG_MIT_CTL_IN        (30065u)  /* MIT 输入：pos/speed/tq */
#define MAV_MSG_MIT_CTL_CONFIG    (30066u)  /* MIT 配置：kp/kd/kt */
/* 速度模式轨迹配置（2026-11-19，本板自定义，占用空闲 ID 30089）：
 * payload: uint8 mode(0=梯形/1=S曲线) + pad3 + float j_max(rad/s^3) = 8 B */
#define MAV_MSG_VEL_PROFILE_CFG   (30089u)

/* 消息元数据：长度（payload 字节数）与 crc_extra，来自 mavlink_layouts.json */
typedef struct {
    uint32_t msgid;
    uint8_t  len;
    uint8_t  crc_extra;
} MavMsgMeta_t;

/* 按 msgid 查元数据；返回 0 表示未登记（调用方按噪声/静默丢弃处理） */
const MavMsgMeta_t *MavMsg_Find(uint32_t msgid);

/* X25 CRC：初值 0xFFFF，逐字节累加，最后累加 extra。
 * p 指向 LEN 字节（即帧的 buf[1]），n = 9 + payload_len。 */
uint16_t MavCrc_Calc(const uint8_t *p, uint16_t n, uint8_t extra);

/* 组装一帧到 out（out 至少 12+len 字节），返回帧长（12+len）。
 * seq/sysid/compid 由调用方给定；payload 可为 0（len=0 的空载荷帧）。 */
uint16_t MavFrame_Build(uint8_t *out, uint32_t msgid, uint8_t seq,
                        uint8_t sysid, uint8_t compid,
                        const uint8_t *payload, uint8_t len, uint8_t extra);


/* ---- 30092 AppIdentResult 的 kind 定义（与 servo_cmd.c 的推送点一一对应）----
 * 这里的注释就是协议契约，必须与实际推送的 v[] 逐位一致。
 * 2026-11-21 验证时发现 4 处注释与实际字段不符（当初按计划写的），已按实况修正。
 * 空着的槽位一律填 0，接收侧不要依赖未定义位。
 *
 * **失败帧（ok=0）的通用约定**：v[0] 一律换成**错误码**（整数转 float 发），
 * 其余槽位放"已经测到的现场量"，用来定位卡在哪一步。同一个 kind 在 ok=0 时
 * v[0] 的语义与 ok=1 不同，接收侧必须**先看 ok 再解释 v[]**。
 * 2026-11-21 之前 kind 1/2/3/4/5/6 的失败路径只打 ASCII，而 ASCII 默认关闭
 * （X72_ASCII_DEBUG=0）→ 失败完全静默，上位机只能报"超时/失败"，
 * 拿不到错误码。现已全部补推 30092。 */
#define MAV_ID_FLUX      (1u)   /* v: psi_mWb, kt_mNm, Tref_C, R_mOhm, L_uH, ke, dev_pct
                                 * 失败 v: err, n_done, psi_mWb, R_mOhm, L_uH, dev_pct, dbg_om, dbg_wref */
#define MAV_ID_RL        (2u)   /* v: R_mOhm, L_uH, Z_ohm, iamp_A, Vavg_V, Iavg_A, inj_amp_A
                                 * 失败 v: err, R_mOhm, L_uH, Z_ohm, iamp_A, Vavg_V, Iavg_A, inj_amp_A
                                 *   v[1]=0 → 连 R 段都没跑；v[2]=0 而 v[1] 有值 → 注入段失败 */
#define MAV_ID_INERTIA   (3u)   /* v: J_e4(=J*1e4), acc1, acc2, tq1, tq2
                                 * 失败（3/4/5 共用一套）：err, mode, cog_n, fric_n, sweep_n, J_e4, visc, 0 */
/* 4 = 摩擦曲线：**诊断入口**，不是标定入口。
 *     v: fric_a, visc, n_pts, iq_static(静摩擦 A), ratio_static(静/动比)
 *     单向 8 点、重复性 ±12%~21%，只适合看特性（Stribeck 拐点、润滑状态、
 *     ff_boost 依据）；标定参数请用 kind=9（$，三点正反，重复性 0.3%）。 */
#define MAV_ID_FRIC_CURVE (4u)  /* v: fric_a, visc, n_pts, iq_static, ratio */
#define MAV_ID_COG       (5u)   /* v: pp_A, pp_fwd_A, pp_rev_A, dc_A */
#define MAV_ID_SWEEP     (6u)   /* v: wmean_rads, wamp_rads, iamp_A */
#define MAV_ID_ENC_CAL   (7u)   /* v: turns, rpm_avg, rpm_min, rpm_max,
                                 *    chip_status, elapsed_ms, ol_mode, target_rpm */
#define MAV_ID_ALIGN     (8u)   /* v: ratio, dir, off_deg, zero_deg, n_step, n_sweep, n_fail, resid */
/* 9 = 摩擦标定：**唯一的标定入口**（名字里的 2PT 是历史遗留，2026-11-21 起
 *     是 3 点 × 正反 = 6 段 + 最小二乘，13 s）。
 *     v: iq1_a, iq2_a, visc, tc_nm, rpm1, rpm2（成功）
 *     v: err,  iq1_a, iq2_a, rpm1, rpm2, visc（失败，ok=0） */
#define MAV_ID_FRIC_2PT  (9u)   /* v: iq1_a, iq2_a, visc, tc_nm, rpm1, rpm2 */
/* 10 = CAN FD 内环自检（2026-11-22）
 * v: tx, rx, err, tec, bitrate_nom, bitrate_data, dlc_mask, rec */
#define MAV_ID_CAN       (10u)
/* 11 = CAN FD 双节点链路测试（2026-11-22）
 * v: role(1 tx/2 rx), tx_ok, tx_fail, rx_ok, lost, dup, tec, rec */
#define MAV_ID_CAN_TEST  (11u)

/* ---- 30090 AppCtrlStatus 的 p[3] flags 位定义 ---- */
#define MAV_ST_CAL        (1u << 0)   /* 电流零点已标定 */
#define MAV_ST_ZERO       (1u << 1)   /* 电角度零点已对齐（z） */
#define MAV_ST_ARMED      (1u << 2)   /* 已使能（en） */
#define MAV_ST_FF         (1u << 3)   /* 速度前馈总开关 */
#define MAV_ST_DEADBAND   (1u << 4)   /* 死区补偿 */
#define MAV_ST_HARM       (1u << 5)   /* 5/7 次谐波抑制 */
#define MAV_ST_KTCOMP     (1u << 6)   /* Kt 温度补偿 */
#define MAV_ST_COG        (1u << 7)   /* 齿槽表已装载且零点一致 */

#endif /* MAVLINK_COMMON_H */
