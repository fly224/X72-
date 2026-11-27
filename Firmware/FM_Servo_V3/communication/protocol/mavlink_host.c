/*
 * mavlink_host.c —— MAVLink 主机指令接收与分发（见 mavlink_host.h）
 *
 * 首期 5 条操作消息：30000 READ_PARAM / 30054 控制字 / 30056 模式 /
 * 30068 权限 / 30069 系统命令。全部经 app_param 的 set/get 门卫，
 * 本协议层不直接改 Servo_* 变量。
 *
 * 应答机制（对齐 host/mavlink_proto.py 的 write_mechanism）：
 *   设备回显同 msgid 作为应答，payload = 回读后的真实值；无独立 ACK。
 * 读机制：30000 payload = uint32 struct_id；设备用 struct_id 作 msgid 回发结构体
 *   （首期只支持回发 30055 AppStatusInfo，其余 struct_id 回空帧并计 unknown）。
 */
#include "mavlink_host.h"
#include "mavlink_common.h"
#include "app_param.h"
#include "uart_link.h"
#include "bsp_servo.h"
#include "telemetry_tx.h"
#include "pos_loop.h"
#include "deadband_comp.h"
#include "cog_ff.h"          /* 齿槽前馈开关（整数通道 cog_en） */
#include "servo_cmd.h"       /* 只读诊断：命令队列溢出计数（q_ovf） */
#include "param_store.h"     /* 只读诊断：参数加载失败原因（ps_diag） */
#include "enc_comp.h"        /* 编码器角度软件补偿（30098） */
/* 直接访问 huart1 做 gState 卡死自愈（与 telemetry_tx.c 同款兜底）。
 * 不加这行会报 undeclared identifier 'huart1'，且不能靠 uart_link.h 转一手 ——
 * 那边只暴露 TrySend*，故意不把 HAL 句柄泄漏给上层。 */
#include "usart.h"
#include <math.h>
#include <string.h>

/* 4 模式单位换算：脉冲(int64 p) / 脉冲每秒(float p_s) → rad / rad/s。
 * 当量 2^21 counts/rev，与 BspServo_GetPosSum() 同口径。 */
#define MAV_P2RAD(p)   ((float)(p) * APP_TWO_PI / APP_ENC_COUNTS)
#define MAV_PS2RADS(v) ((v) * APP_TWO_PI / APP_ENC_COUNTS)

/* 小端 f32 解码（Cortex-M4 原生小端，memcpy 即可，避免 strict-aliasing 违规） */
static float ld_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}
static int64_t ld_i64(const uint8_t *p)
{
    int64_t v;
    memcpy(&v, p, 8);
    return v;
}

/* ---- 录波数据的单位还原 ----
 * 缓冲区里是 int16 原始码，三个通道各用各的标度（见 servo_isr.c 录制段）：
 *   ch0 iq     : raw = iq × 131        → A
 *   ch2 vq_int : raw = Vq_int × 1000   → V（mV 存进来的）
 *   ch1 iu     : raw = iu × 131        → A，另配同拍电角度 2π→65536
 * 换算是"接口边界"的事，放固件做一次，上位机不再记一套系数 ——
 * ASCII 时代每个脚本各写一份 131，改一处漏一处。 */
static float rec_val_units(int16_t raw)
{
    if ((Servo_GetRecordChannel() == 2U) || (Servo_GetRecordChannel() == 3U)) {
        return (float)raw * 0.001f;              /* mV → V */
    }
    return (float)raw * (1.0f / 131.0f);         /* iq / iu → A */
}

static float rec_th_units(int16_t raw)
{
    uint16_t u = (uint16_t)raw;                  /* 位模式，不是有符号值 */
    return (float)u * (6.283185307179586f / 65536.0f);
}

/* ---- 帧接收状态机 ---- */
typedef enum {
    MAV_RX_STX = 0U,      /* 等 0xFD */
    MAV_RX_HEAD,          /* 收 LEN..MSGID[2]（9 字节，索引 1..9） */
    MAV_RX_PAYLOAD,       /* 收 payload */
    MAV_RX_CRC            /* 收 CRC 低/高 */
} MavRxState_t;

static MavRxState_t s_rx_state = MAV_RX_STX;
static uint8_t  s_frame[MAV_MAX_FRAME];   /* 完整帧：frame[0]=0xFD */
static uint8_t  s_rx_len;                 /* payload 长度（frame[1]） */
static uint8_t  s_rx_i;                   /* 当前收字节在 frame 中的索引 */
static uint16_t s_rx_crc_err;
static uint16_t s_rx_unknown;

/* ---- 完整帧待处理队列（单缓冲，主循环取走；覆盖计数） ---- */
static uint8_t  s_pending;                /* 1 = 有一帧待分发 */
static uint8_t  s_pend_len;               /* 待分发帧长度（frame 前 s_pend_len 字节） */
static uint16_t s_drop;                   /* 上一帧未取走又被新帧覆盖 */

/* ---- 应答缓冲 + 发送状态（主循环非阻塞重试） ---- */
static uint8_t  s_ack[MAV_MAX_FRAME];
static uint8_t  s_ack_len;
static uint8_t  s_ack_pending;            /* 1 = 应答待发 */
static uint16_t s_tx_drop;
static uint8_t  s_tx_seq;                 /* 应答帧序号，独立递增 */

/* ---- 统计 ---- */
static uint16_t s_ok;
static uint16_t s_locked;
static uint16_t s_reset_req;              /* 1 = 收到复位命令且应答已发出，待触发复位 */

/* ---- 链路自愈（2026-11-20 实测后加）----
 *
 * 故障形态：某次 MAVLink 写命令之后，ASCII 与所有 ACK 同时哑掉，
 * 但 100 Hz 遥测照发、seq 连续（无复位）。机理：
 *   1) UartLink_TrySendBinary 因 huart1.gState != READY 返回 0；
 *   2) s_ack_pending 保持 1，MavHost_Task 每拍开头就 return —— **不再 dispatch**，
 *      于是后续 30056/30019/30054 全部不执行，现象是"命令发了、电机不动、无 ACK"；
 *   3) ASCII 走同一个 gState 检查，一起哑。
 * 遥测侧本来就有自愈（telemetry_tx.c 连续 20 拍忙就 Abort + 强置 READY），
 * 所以只有遥测活着。这里给应答侧补两层同样的兜底。
 *
 * 正常时序：58 B @921600 ≈ 0.63 ms。20 ms / 30 ms 的阈值不会误伤。 */
#define MAV_GSTATE_STUCK_MS  (20u)   /* gState 连续非 READY 超过它就强制 Abort */
#define MAV_ACK_STUCK_MS     (30u)   /* 单条应答发不出去超过它就丢弃，恢复分发 */

static uint32_t s_busy_since;             /* gState 变忙的起始 tick，0 = 当前不忙 */
static uint32_t s_ack_since;              /* 本条应答开始尝试的 tick，0 = 无 */
static uint16_t s_uart_recover;           /* 强制 Abort 次数 */
static uint16_t s_ack_drop;               /* 因超时丢弃的应答数 */

/* 解析失败重同步阈值：连续 N 个非 0xFD 字节或 CRC 错后强制回 STX */
#define MAV_RESYNC_LIMIT   (16u)
static uint16_t s_idle_bytes;

void MavHost_Init(void)
{
    s_rx_state   = MAV_RX_STX;
    s_rx_len     = 0u;
    s_rx_i       = 0u;
    s_rx_crc_err = 0u;
    s_rx_unknown = 0u;
    s_pending    = 0u;
    s_pend_len   = 0u;
    s_drop       = 0u;
    s_ack_len    = 0u;
    s_ack_pending = 0u;
    s_busy_since = 0u;
    s_ack_since  = 0u;
    s_uart_recover = 0u;
    s_ack_drop     = 0u;
    s_tx_drop    = 0u;
    s_tx_seq     = 0u;
    s_ok         = 0u;
    s_locked     = 0u;
    s_reset_req  = 0u;
    s_idle_bytes = 0u;
    memset(s_frame, 0, sizeof(s_frame));
    memset(s_ack, 0, sizeof(s_ack));
}

/* 帧收完整：校验 CRC，成功则挂到 pending 队列 */
static void rx_frame_done(uint8_t len_total)
{
    uint32_t msgid;
    const MavMsgMeta_t *meta;
    uint16_t crc_calc, crc_recv;

    msgid = (uint32_t)s_frame[7] | ((uint32_t)s_frame[8] << 8) |
            ((uint32_t)s_frame[9] << 16);
    meta  = MavMsg_Find(msgid);
    if (meta == 0) {
        if (s_rx_unknown < 0xFFFFu) { s_rx_unknown++; }
        return;                          /* 未登记消息：静默丢弃 */
    }
    if (meta->len != s_rx_len) {
        if (s_rx_crc_err < 0xFFFFu) { s_rx_crc_err++; }
        return;                          /* 长度与元数据不符：按坏帧丢 */
    }

    crc_calc = MavCrc_Calc(&s_frame[1], (uint16_t)(9u + s_rx_len), meta->crc_extra);
    crc_recv = (uint16_t)(s_frame[10u + s_rx_len] |
                          ((uint16_t)s_frame[11u + s_rx_len] << 8));
    if (crc_calc != crc_recv) {
        if (s_rx_crc_err < 0xFFFFu) { s_rx_crc_err++; }
        return;
    }

    if (s_pending != 0U) {
        /* 上一帧还没分发：绝不能覆盖 s_frame —— dispatch() 读的是同一缓冲，
         * 旧实现"覆盖并计数"实际是把新帧当成旧帧执行（错分发，不是丢帧）。
         * 指令类消息低频，丢弃新帧 + 上位机重发是可接受策略。 */
        if (s_drop < 0xFFFFu) { s_drop++; }
        return;
    }
    s_pend_len = len_total;
    s_pending  = 1U;
}

uint8_t MavHost_FeedByte(uint8_t b)
{
    switch (s_rx_state) {
    case MAV_RX_STX:
        if (b == MAV_STX) {
            s_frame[0]   = b;
            s_rx_i       = 1u;
            s_rx_state   = MAV_RX_HEAD;
            s_idle_bytes = 0u;
            return 1U;                    /* 帧首被消费 */
        }
        if (++s_idle_bytes >= MAV_RESYNC_LIMIT) { s_idle_bytes = 0u; }
        return 0U;                        /* 空闲非帧首：交还调用方 */

    case MAV_RX_HEAD:
        s_frame[s_rx_i++] = b;
        if (s_rx_i >= 10u) {              /* 收满 LEN..MSGID[2]（索引 1..9） */
            s_rx_len   = s_frame[1];
            if (s_rx_len > MAV_MAX_PAYLOAD) {
                s_rx_state = MAV_RX_STX;  /* 超长：防御性丢弃，避免越界 */
            } else if (s_rx_len == 0u) {
                s_rx_i = 10u;
                s_rx_state = MAV_RX_CRC;
            } else {
                s_rx_state = MAV_RX_PAYLOAD;
            }
        }
        return 1U;                        /* 收帧中的字节一律消费 */

    case MAV_RX_PAYLOAD:
        s_frame[s_rx_i++] = b;
        if (s_rx_i >= (uint8_t)(10u + s_rx_len)) { s_rx_state = MAV_RX_CRC; }
        return 1U;

    case MAV_RX_CRC:
        s_frame[s_rx_i++] = b;
        if (s_rx_i >= (uint8_t)(12u + s_rx_len)) {
            rx_frame_done((uint8_t)(12u + s_rx_len));
            s_rx_state = MAV_RX_STX;      /* 无论成败都回到等 STX，实现重同步 */
            s_rx_i     = 0u;
        }
        return 1U;

    default:
        s_rx_state = MAV_RX_STX;
        return 0U;
    }
}

/* ---- 30082 AppDebugParam：运行时调参表（2026-11-20）----
 *
 * 为什么要它：Z 对齐的注入电流、开环电压限幅这类"试几个值才知道合不合适"的量
 * 原来是编译期常量，改一次就要重编译 + 烧录，而每次烧录又会擦掉参数页
 * （cal/z/psi 全归零），等于每调一次参数都要重做整套标定。
 *
 * 载荷布局（与 mavlink_layouts.csv 一致）：16×f32 + 16×u32 + 16×i32 = 192 B
 *   Debug_float[0..15]   值
 *   Debug_uint32[0]      float 写入掩码：bit i = 1 才写第 i 项；为 0 表示纯读
 *   Debug_uint32[15]     整数写入掩码：bit i = 1 才写 Debug_int32[i]（2026-11-21 加）
 *   Debug_int32[0..15]   开关 / 档位（布尔按非 0 即开）
 * 为什么必须用掩码：整表写无法区分"写 0"和"不写"，而 0 对多数增益非法，
 * 会被各自的钳位悄悄改掉，回读就对不上，现场会以为"写进去了但没生效"。
 *
 * 越界一律拒绝并计入 s_locked，不静默截断。
 */
typedef struct {
    const char *name;
    uint8_t (*set)(float v);   /* 1 = 接受 */
    float   (*get)(void);
} DbgParam_t;

/* 整数量通道（开关 / 档位）。2026-11-21 加，用来接管 ASCII 的
 * `B`（死区补偿）、`%`（5/7 次谐波）、`{`（起动助推）、`-`（电角补偿）
 * 以及几个"以前只能靠命令切换、状态读不回来"的开关。
 *
 * 为什么不塞进 float 表：`B`/`%` 是布尔，用 float 表达要来回比较 ±0.5，
 * 一旦有人写 0.5 就会出现"既不算开也不算关"的含糊态。整数通道语义干净。
 *
 * 掩码独立一槽（Debug_uint32[15]，偏移 124），不与 float 掩码共用：
 * 共用就无法表达"只写开关不写增益"这种最常见的操作。 */
typedef struct {
    const char *name;
    uint8_t (*set)(int32_t v);  /* 1 = 接受 */
    int32_t (*get)(void);
} DbgIntParam_t;

/* 现有接口多为 void 且有内部钳位，统一包成"回读比较"判断是否真的接受 */
static uint8_t dbg_set_cur_kp(float v)
{
    float kp, ki;
    Servo_GetCurGains(&kp, &ki);
    Servo_SetCurGains(v, ki);
    Servo_GetCurGains(&kp, &ki);
    return (fabsf(kp - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_cur_kp(void) { float kp, ki; Servo_GetCurGains(&kp, &ki); return kp; }

static uint8_t dbg_set_cur_ki(float v)
{
    float kp, ki;
    Servo_GetCurGains(&kp, &ki);
    Servo_SetCurGains(kp, v);
    Servo_GetCurGains(&kp, &ki);
    return (fabsf(ki - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_cur_ki(void) { float kp, ki; Servo_GetCurGains(&kp, &ki); return ki; }

static uint8_t dbg_set_spd_kp(float v)
{
    float kp, ki;
    Servo_GetSpeedGains(&kp, &ki);
    Servo_SetSpeedGains(v, ki);
    Servo_GetSpeedGains(&kp, &ki);
    return (fabsf(kp - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_spd_kp(void) { float kp, ki; Servo_GetSpeedGains(&kp, &ki); return kp; }

static uint8_t dbg_set_spd_ki(float v)
{
    float kp, ki;
    Servo_GetSpeedGains(&kp, &ki);
    Servo_SetSpeedGains(kp, v);
    Servo_GetSpeedGains(&kp, &ki);
    return (fabsf(ki - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_spd_ki(void) { float kp, ki; Servo_GetSpeedGains(&kp, &ki); return ki; }

static uint8_t dbg_set_spd_iqmax(float v)
{
    Servo_SetSpeedIqMax(v);
    return (fabsf(Servo_GetSpeedIqMax() - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_spd_iqmax(void) { return Servo_GetSpeedIqMax(); }

static uint8_t dbg_set_pos_kp(float v)
{
    PosLoop_SetKp(v);
    return (fabsf(PosLoop_GetKp() - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_pos_kp(void) { return PosLoop_GetKp(); }

static uint8_t dbg_set_ff_acc(float v)
{
    uint8_t en = 0u; float a = 0.0f, f = 0.0f, vs = 0.0f;
    Servo_GetFeedforward(&en, &a, &f, &vs);
    Servo_SetFeedforward(en, v, f, vs);
    Servo_GetFeedforward(&en, &a, &f, &vs);
    return (fabsf(a - v) < 1.0e-9f) ? 1u : 0u;
}
static float dbg_get_ff_acc(void)
{
    uint8_t en = 0u; float a = 0.0f, f = 0.0f, vs = 0.0f;
    Servo_GetFeedforward(&en, &a, &f, &vs); return a;
}
static uint8_t dbg_set_ff_fric(float v)
{
    uint8_t en = 0u; float a = 0.0f, f = 0.0f, vs = 0.0f;
    Servo_GetFeedforward(&en, &a, &f, &vs);
    Servo_SetFeedforward(en, a, v, vs);
    Servo_GetFeedforward(&en, &a, &f, &vs);
    return (fabsf(f - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_ff_fric(void)
{
    uint8_t en = 0u; float a = 0.0f, f = 0.0f, vs = 0.0f;
    Servo_GetFeedforward(&en, &a, &f, &vs); return f;
}
static uint8_t dbg_set_ff_visc(float v)
{
    uint8_t en = 0u; float a = 0.0f, f = 0.0f, vs = 0.0f;
    Servo_GetFeedforward(&en, &a, &f, &vs);
    Servo_SetFeedforward(en, a, f, v);
    Servo_GetFeedforward(&en, &a, &f, &vs);
    return (fabsf(vs - v) < 1.0e-9f) ? 1u : 0u;
}
static float dbg_get_ff_visc(void)
{
    uint8_t en = 0u; float a = 0.0f, f = 0.0f, vs = 0.0f;
    Servo_GetFeedforward(&en, &a, &f, &vs); return vs;
}

static uint8_t dbg_set_dead_du(float v)
{
    DeadbandComp_SetDu(v);
    return (fabsf(DeadbandComp_GetDu() - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_dead_du(void) { return DeadbandComp_GetDu(); }

static uint8_t dbg_set_cur_iqmax(float v)
{
    Servo_SetCurIqMax(v);
    return (fabsf(Servo_GetCurIqMax() - v) < 1.0e-6f) ? 1u : 0u;
}
static float dbg_get_cur_iqmax(void) { return Servo_GetCurIqMax(); }

static uint8_t dbg_set_psi(float v)
{
    Servo_SetIdentPsi(v);
    return (fabsf(Servo_GetKtNow() - 1.5f * 14.0f * v) < 1.0e-3f) ? 1u : 0u;
}
static float dbg_get_psi(void)
{
    float r = 0.0f, l = 0.0f, p = 0.0f, kt = 0.0f;
    Servo_GetIdentParams(&r, &l, &p, &kt);
    return p;
}

/* 电角补偿：对外单位用 deg（电角度），与 ASCII 的 `-` 命令一致。
 * 内部是 rad，转换只在这里做一次 —— 单位只活在接口边界，不放注释里。 */
static uint8_t dbg_set_eleoff_deg(float v)
{
    if (v != v) { return 0u; }                       /* NaN */
    if ((v < -30.0f) || (v > 30.0f)) { return 0u; }  /* 超范围视为填写错误 */
    Servo_SetElecOff(v * 0.01745329252f);
    return 1u;
}
static float dbg_get_eleoff_deg(void) { return Servo_GetElecOff() * 57.2957795131f; }

/* 起动助推倍数：底层 setter 是 void，包一层返回 1 才能进参数表。
 * 这是"表驱动"的一个必要代价 —— 不要为了省包装去改底层签名，
 * 底层被 20 kHz 环路调用，签名稳定比接口美观重要。 */
static uint8_t dbg_set_ff_boost(float v)
{
    if (v != v) { return 0u; }
    if ((v < 0.0f) || (v > 3.0f)) { return 0u; }   /* 与底层钳位一致，提前拒 */
    Servo_SetFeedforwardBoost(v);
    return 1u;
}

/* ---- 整数量通道的包装 ----
 * 布尔一律按 "非 0 即开" 解释，回读只返回 0/1，避免出现 2、-1 这类既非开
 * 也非关的值。前馈总开关必须走 Get-then-Set：Servo_SetFeedforward() 是
 * 四参数接口，只改 en 时其余三项要用当前值兜住，否则会把标定好的
 * ff_acc / ff_fric / ff_visc 一起写成 0（这就是"漏写字段静默清零"）。 */
static uint8_t dbg_set_dead_en(int32_t v)
{
    DeadbandComp_SetEnable((v != 0) ? 1u : 0u);
    return 1u;
}
static int32_t dbg_get_dead_en(void) { return (DeadbandComp_IsEnabled() != 0U) ? 1 : 0; }

static uint8_t dbg_set_harm_en(int32_t v)
{
    Servo_SetHarmonic((v != 0) ? 1u : 0u);
    return 1u;
}
static int32_t dbg_get_harm_en(void) { return (Servo_IsHarmonic() != 0U) ? 1 : 0; }

static uint8_t dbg_set_ff_en(int32_t v)
{
    uint8_t en = 0u;
    float   a = 0.0f, f = 0.0f, visc = 0.0f;

    Servo_GetFeedforward(&en, &a, &f, &visc);
    Servo_SetFeedforward((v != 0) ? 1u : 0u, a, f, visc);
    return 1u;
}
static int32_t dbg_get_ff_en(void)
{
    uint8_t en = 0u;
    float   a = 0.0f, f = 0.0f, visc = 0.0f;

    Servo_GetFeedforward(&en, &a, &f, &visc);
    return (en != 0u) ? 1 : 0;
}

static uint8_t dbg_set_cog_en(int32_t v)
{
    CogFf_SetEnable((v != 0) ? 1u : 0u);
    return 1u;
}
static int32_t dbg_get_cog_en(void) { return (CogFf_IsEnabled() != 0U) ? 1 : 0; }

static uint8_t dbg_set_ktc_en(int32_t v)
{
    Servo_SetKtCompEn((v != 0) ? 1u : 0u);
    return 1u;
}
static int32_t dbg_get_ktc_en(void) { return (Servo_GetKtCompEn() != 0U) ? 1 : 0; }

static uint8_t dbg_set_sched_en(int32_t v)
{
    Servo_SetSchedEnable((v != 0) ? 1u : 0u);
    return 1u;
}
static int32_t dbg_get_sched_en(void) { return (Servo_GetSchedEnable() != 0U) ? 1 : 0; }

/* ---- 位置补偿三参数（原 ASCII 0x04 PCMP 帧，2026-11-21 迁到 30082）----
 * 为什么不放 float 通道：float 16 槽已满，而 30082 的载荷是硬布局
 * （16f + 16u32 + 16i32 = 192 B），加 float 槽要改两边协议；整数通道还有 10 个空槽。
 * 用 mrad 表达：补偿量本身是 0~50 mrad 量级，1 mrad 分辨率对定位足够，
 * 而且原 PCMP 回显本来就是 mrad —— 换算不改变精度观感。
 * 上限 100 mrad / 100 rad 是硬挡：PosLoop 内部还会再钳一次，这里挡的是"手滑输错量级"。 */
#define DBG_COMP_MRAD_MAX   (100000)     /* 100 mrad */
#define DBG_COMP_MIN_MRAD_MAX (100000)   /* 100 rad  */

static uint8_t dbg_set_comp_mrad(int32_t v)
{
    if ((v < 0) || (v > DBG_COMP_MRAD_MAX)) { return 0u; }
    PosLoop_SetComp((float)v * 0.001f);
    return 1u;
}
static int32_t dbg_get_comp_mrad(void)
{
    return (int32_t)(PosLoop_GetComp() * 1000.0f + 0.5f);
}

static uint8_t dbg_set_comp_min_mrad(int32_t v)
{
    if ((v < 0) || (v > DBG_COMP_MIN_MRAD_MAX)) { return 0u; }
    PosLoop_SetCompMin((float)v * 0.001f);
    return 1u;
}
static int32_t dbg_get_comp_min_mrad(void)
{
    return (int32_t)(PosLoop_GetCompMin() * 1000.0f + 0.5f);
}

static uint8_t dbg_set_comp_en(int32_t v)
{
    PosLoop_SetCompEn((v != 0) ? 1u : 0u);
    return 1u;
}
static int32_t dbg_get_comp_en(void) { return (PosLoop_GetCompEn() != 0U) ? 1 : 0; }

/* 只读链路诊断（槽 9）。上半字 = s_drop（pending 被顶掉），下半字 = CRC/长度错。
 * 只读是故意的：这不是参数，写它没有意义；set=NULL 时 30082 会拒写并计入
 * s_locked，上位机 dbg_write 会看到写入项数不变。 */
static int32_t dbg_get_link_diag(void)
{
    uint16_t crc = 0u, drop = 0u;

    MavHost_GetRxDiag(&crc, &drop);
    return (int32_t)(((uint32_t)drop << 16) | (uint32_t)crc);
}

/* 只读：命令环形缓冲溢出丢字节（槽 10）。正常恒 0。 */
static int32_t dbg_get_q_ovf(void)
{
    return (int32_t)ServoCmd_GetRxOverflow();
}

/* 只读：USART 错误标志累计（槽 11：ORE/FE/NE/PE）。
 * 与 q_ovf 的分工：q_ovf 是"软件缓冲满"，uart_err 是"硬件 RDR 没被及时取走"。
 * 前者查主循环阻塞，后者查中断被屏蔽/关中断临界区。 */
static int32_t dbg_get_uart_err(void)
{
    uint32_t n = UartLink_GetErrorCount();

    return (int32_t)((n > 0x7FFFFFFFu) ? 0x7FFFFFFF : n);
}

/* 只读：参数加载诊断（槽 13）。低字节 = 失败原因，高字节 = Flash 里的 version。
 * 见 param_store.c 的说明：这四项对应完全不同的处置，不能合成一个"没参数"。 */
static int32_t dbg_get_ps_diag(void)
{
    return (int32_t)(((uint32_t)ParamStore_GetLoadVer() << 8) |
                     (uint32_t)ParamStore_GetLoadWhy());
}

/* 槽 14 只读：缺相检测打包诊断（见 bsp_servo.h 的位定义）。
 * 为什么不开成三个 float 槽：float 通道已满，而这里要的只是
 * "结论 + 两个百分比"，一个 int 够用，也便于脚本一次读完。 */
static int32_t dbg_get_ph_diag(void)
{
    return (int32_t)Servo_GetPhaseMon();
}

/* 槽 15：缺相是否升级为故障。默认 0（只诊断不跳闸）—— 缺相跳闸会直接断输出，
 * 关节负载侧会掉下来，这个决定必须由使用者显式打开。 */
static uint8_t dbg_set_ph_trip(int32_t v)
{
    Servo_SetPhaseTrip((v != 0) ? 1u : 0u);
    return 1u;
}
static int32_t dbg_get_ph_trip(void) { return (Servo_GetPhaseTrip() != 0U) ? 1 : 0; }

/* 只读：已成功分发的 MAVLink 帧数（槽 12）。配合 link_diag 可区分
 * "请求没到"（rx_ok 不涨）与"应答没回"（rx_ok 涨、但上位机收不到）。 */
static int32_t dbg_get_rx_ok(void)
{
    uint16_t ok = 0u;

    MavHost_GetStats(&ok, 0, 0, 0, 0);
    return (int32_t)ok;
}
/* 索引表。NULL 表示该槽位未使用（写入被拒、读出为 0）。
 * 顺序即对外契约，只能追加不能插队——上位机按索引调。 */static const DbgParam_t s_dbg_param[16] = {
    { "align_id",  Servo_SetAlignId,  Servo_GetAlignId  },   /* 0  Z 注入电流 A */
    { "ol_vlim",   Servo_SetOLVlim,   Servo_GetOLVlim   },   /* 1  开环电压限幅 ×vbus */
    { "cur_kp",    dbg_set_cur_kp,    dbg_get_cur_kp    },   /* 2 */
    { "cur_ki_ts", dbg_set_cur_ki,    dbg_get_cur_ki    },   /* 3 */
    { "spd_kp",    dbg_set_spd_kp,    dbg_get_spd_kp    },   /* 4 */
    { "spd_ki_ts", dbg_set_spd_ki,    dbg_get_spd_ki    },   /* 5 */
    { "spd_iqmax", dbg_set_spd_iqmax, dbg_get_spd_iqmax },   /* 6  A */
    { "pos_kp",    dbg_set_pos_kp,    dbg_get_pos_kp    },   /* 7  1/s */
    { "ff_acc",    dbg_set_ff_acc,    dbg_get_ff_acc    },   /* 8  A/(rad/s^2) */
    { "ff_fric",   dbg_set_ff_fric,   dbg_get_ff_fric   },   /* 9  A */
    { "ff_visc",   dbg_set_ff_visc,   dbg_get_ff_visc   },   /* 10 A/(rad/s) */
    { "dead_du",   dbg_set_dead_du,   dbg_get_dead_du   },   /* 11 占空比 0~1 */
    { "cur_iqmax", dbg_set_cur_iqmax, dbg_get_cur_iqmax },   /* 12 A */
    { "psi",       dbg_set_psi,       dbg_get_psi       },   /* 13 Wb（会连带 Kt） */
    { "eleoff_deg", dbg_set_eleoff_deg, dbg_get_eleoff_deg },/* 14 电角度补偿 deg（原 '-'） */
    { "ff_boost",  dbg_set_ff_boost,  Servo_GetFeedforwardBoost }, /* 15 起动助推倍数（原 '{'） */
};

/* 整数通道表：开关与档位。索引即契约，只能追加。
 * 这几项以前只能靠 ASCII 单字符切换，且状态读不回来 —— 最典型的是
 * ff_en：以前只有 `$` 标定成功才会置 1，`S`/`K` 行都看不出它是关的，
 * 只能靠"把增益拉到极端值看有没有反应"这种反向实验来确认。 */
static const DbgIntParam_t s_dbg_int[16] = {
    { "dead_en",  dbg_set_dead_en,  dbg_get_dead_en  },   /* 0 死区补偿（原 'B'） */
    { "harm_en",  dbg_set_harm_en,  dbg_get_harm_en  },   /* 1 5/7 次谐波（原 '%'） */
    { "ff_en",    dbg_set_ff_en,    dbg_get_ff_en    },   /* 2 速度前馈总开关 */
    { "cog_en",   dbg_set_cog_en,   dbg_get_cog_en   },   /* 3 齿槽前馈 */
    { "ktc_en",   dbg_set_ktc_en,   dbg_get_ktc_en   },   /* 4 Kt 温度补偿 */
    { "sched_en", dbg_set_sched_en, dbg_get_sched_en },   /* 5 增益调度 */
    { "comp_mrad",     dbg_set_comp_mrad,     dbg_get_comp_mrad     }, /* 6 位置补偿量 */
    { "comp_min_mrad", dbg_set_comp_min_mrad, dbg_get_comp_min_mrad }, /* 7 补偿行程阈值 */
    { "comp_en",       dbg_set_comp_en,       dbg_get_comp_en       }, /* 8 补偿使能 */
    { "link_diag",     0,                     dbg_get_link_diag     }, /* 9 只读：收帧丢帧细分 */
    { "q_ovf",         0,                     dbg_get_q_ovf         }, /* 10 只读：命令队列溢出 */
    { "uart_err",      0,                     dbg_get_uart_err      }, /* 11 只读：USART 错误标志 */
    { "rx_ok",         0,                     dbg_get_rx_ok         }, /* 12 只读：已分发帧数 */
    { "ps_diag",       0,                     dbg_get_ps_diag       }, /* 13 只读：参数加载诊断 */
    { "ph_diag",       0,                     dbg_get_ph_diag       }, /* 14 只读：缺相检测打包 */
    { "ph_trip",       dbg_set_ph_trip,       dbg_get_ph_trip       }, /* 15 缺相是否跳闸 */
};

/* ---- 应答组帧工具 ---- */
static void ack_prepare(uint32_t msgid, const uint8_t *payload, uint8_t len){
    const MavMsgMeta_t *meta = MavMsg_Find(msgid);
    uint8_t extra = (meta != 0) ? meta->crc_extra : 0u;

    s_ack_len = MavFrame_Build(s_ack, msgid, s_tx_seq, 1u, 1u,
                               payload, len, extra);
    s_tx_seq++;                          /* 序号只在成功组帧后推进 */
    s_ack_pending = 1U;
}

/* 把 AppParamStatus 编码到应答 payload 的第 0 字节（简单错误码） */
static uint8_t status_byte(AppParamStatus_t st)
{
    return (uint8_t)st;
}

/* ---- 分发一条完整帧 ---- */
static void dispatch(void)
{
    uint32_t msgid;
    const uint8_t *pl = &s_frame[10];
    uint8_t resp[16];                /* 最大应答：MIT 配置/给定 = 状态1 + f32*2 = 9 B */
    uint8_t resp_len = 0u;
    AppParamStatus_t st;

    msgid = (uint32_t)s_frame[7] | ((uint32_t)s_frame[8] << 8) |
            ((uint32_t)s_frame[9] << 16);

    memset(resp, 0, sizeof(resp));
    switch (msgid) {
    case MAV_MSG_APP_CONTROL_WORD: {
        uint16_t cw;
        uint8_t  halt;
        memcpy(&cw,   &pl[0], 2);
        halt = pl[2];
        st = AppParam_SetControlWord(cw, halt);
        resp_len = 2u;
        resp[0] = status_byte(st);
        resp[1] = (uint8_t)(AppParam_GetControlWord() & 0xFFu);
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_APP_OP_MODE: {
        int8_t mode = (int8_t)pl[0];
        st = AppParam_SetOpMode(mode);
        resp_len = 2u;
        resp[0] = status_byte(st);
        resp[1] = (uint8_t)AppParam_GetOpMode();
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_APP_PERMISSION: {
        st = AppParam_SetAuthority(pl[0], pl[1]);
        resp_len = 3u;
        resp[0] = status_byte(st);
        resp[1] = AppParam_GetCommAuthority();
        resp[2] = AppParam_GetInternalAuthority();
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_SYSTEM_CMD: {
        uint8_t cmd = pl[0];
        st = AppParam_ExecSysCmd(cmd);
        resp_len = 2u;
        resp[0] = status_byte(st);
        resp[1] = cmd;
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        if ((st == APP_PARAM_SUCCESS) && (cmd == 2u)) { s_reset_req = 1U; }
        break;
    }
    case MAV_MSG_APP_ADV_CFG: {             /* 30096 速度环陷波配置（可读回） */
        /* 载荷 24 B：uint8 en | uint8 rsv[3] | float f0_hz | float depth_db | float q | float rsv
         * 应答与请求同长度，回填**实际生效**的值 —— 参数会被内部限幅
         * （f0 ≤ 0.45·fs、Q 0.3~10），不回填的话上位机看到的是自己写的值，
         * 而环路里跑的是另一个数。 */
        uint8_t p96[24];
        uint8_t en, order;
        float   f0, dep, q;

        memset(p96, 0, sizeof(p96));
        /* pl[0] = 0xFF 是"只读回"标记。为什么需要：上位机的 send() 会把载荷
         * 补齐到声明长度，发不出"短帧"，所以读回不能靠长度区分。
         * pl[1] = 阶次跟踪（2026-11-23）：0=固定 f0，k=陷波频率跟随 k 次/转。 */
        if ((s_rx_len >= 12u) && (pl[0] != 0xFFu)) {
            en  = (pl[0] != 0u) ? 1u : 0u;
            order = (s_rx_len >= 2u) ? pl[1] : 0u;
            memcpy(&f0,  &pl[4], 4);
            memcpy(&dep, &pl[8], 4);
            if (s_rx_len >= 24u) { memcpy(&q, &pl[12], 4); }
            else                 { q = 2.0f; }
            Servo_SetNotch(en, f0, dep, q);
            Servo_SetNotchOrder(order);
        } else {
            /* 短帧按"只读回"处理：上位机可以先发 4 B 查询当前配置 */
            Servo_GetNotch(&en, &f0, &dep, &q);
            order = Servo_GetNotchOrder();
        }

        Servo_GetNotch(&en, &f0, &dep, &q);
        order = Servo_GetNotchOrder();
        p96[0] = en;
        p96[1] = order;
        memcpy(&p96[4],  &f0,  4);
        memcpy(&p96[8],  &dep, 4);
        memcpy(&p96[12], &q,   4);
        ack_prepare(MAV_MSG_APP_ADV_CFG, p96, 24u);
        if (s_ok < 0xFFFFu) { s_ok++; }
        return;                          /* 已自行组应答 */
    }
    case MAV_MSG_APP_SPEED_TRAJ: {          /* 30097 速度轨迹"到达时间"优先（可读回） */
        /* 载荷 32 B：
         *   off0  uint8  cmd      0 = 只读回，1 = 写入 t_ramp_s
         *   off4  float  t_ramp_s 到达目标转速的时间（s）；<=0 关闭时间优先
         *   off8  float  a_eff    只读：当前锁存并实际使用的加速度
         *   off12 float  j_eff    只读：当前锁存并实际使用的 jerk
         *   off16 float  t_left_s 只读：按当前剩余量估的剩余时间
         *   off20 uint32 flags    只读：bit0 时间模式生效 / bit1 被夹 / bit2 S 曲线
         *   off24 float  acc_cfg  只读：30019 配置的加速度（t=0 时生效）
         *   off28 float  j_cfg    只读：30089 配置的 jerk
         *
         * 为什么单开一条消息而不是扩 30089：固件按 meta->len 严格校验
         * （mavlink_host.c:167，长度不符直接当坏帧丢），改 30089 的长度会让
         * 所有旧上位机的 8 B 帧全部失效。新增一条长度固定的消息零破坏。
         */
        uint8_t  p97[32];
        uint8_t  cmd = (s_rx_len >= 1u) ? pl[0] : 0u;
        float    a_eff = 0.0f, j_eff = 0.0f, t_left = 0.0f;
        float    acc_cfg = 0.0f, j_cfg = 0.0f;
        uint8_t  flags = 0u, mode_now = 0u;
        AppParamStatus_t st97 = APP_PARAM_SUCCESS;

        memset(p97, 0, sizeof(p97));

        if (cmd == 1u) {
            float t;
            if (s_rx_len < 8u) {
                st97 = APP_PARAM_OUT_OF_RANGE;
            } else {
                memcpy(&t, &pl[4], 4);
                if (t != t) { st97 = APP_PARAM_OUT_OF_RANGE; }   /* NaN */
                else        { Servo_SetSpeedTrajTime(t); }
            }
            if (st97 != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        } else if (cmd != 0u) {
            st97 = APP_PARAM_OUT_OF_RANGE;                       /* 未知 cmd */
            if (s_locked < 0xFFFFu) { s_locked++; }
        }

        Servo_GetSpeedTrajActual(&a_eff, &j_eff, &t_left, &flags, &mode_now);
        Servo_GetSpeedProfile(&mode_now, &j_cfg);
        acc_cfg = Servo_GetSpeedAccel();

        p97[0] = 0u;                       /* 应答里 cmd 恒为 0 = 这是回读 */
        p97[1] = (uint8_t)st97;            /* 顺带把本次写入的状态带回来 */
        p97[2] = mode_now;
        {
            const float t_now = Servo_GetSpeedTrajTime();
            memcpy(&p97[4],  &t_now,   4);
        }
        memcpy(&p97[8],  &a_eff,   4);
        memcpy(&p97[12], &j_eff,   4);
        memcpy(&p97[16], &t_left,  4);
        {
            const uint32_t f = (uint32_t)flags;
            memcpy(&p97[20], &f, 4);
        }
        memcpy(&p97[24], &acc_cfg, 4);
        memcpy(&p97[28], &j_cfg,   4);
        ack_prepare(MAV_MSG_APP_SPEED_TRAJ, p97, 32u);
        if (s_ok < 0xFFFFu) { s_ok++; }
        return;                          /* 已自行组应答 */
    }
    case MAV_MSG_APP_ENC_COMP: {            /* 30098 编码器角度软件补偿（可读回） */
        /* 载荷 136 B：cmd(1) status(1) n_harm(1) en(1) rsv[4] | 16×(a,b) f32
         *   cmd = 0 只读回；cmd = 1 写入（n_harm=0 等价于关闭）。
         * 应答与请求同长度，回填**实际生效**的系数：SetHarm 会拒绝 NaN/超范围，
         * 不回填的话上位机以为写进去了，而环路里跑的还是旧系数。 */
        uint8_t p98[136];
        float   a8[ENC_COMP_MAX_H], b8[ENC_COMP_MAX_H];
        uint8_t n = 0u, en = 0u, st98 = (uint8_t)APP_PARAM_SUCCESS;

        memset(p98, 0, sizeof(p98));
        memset(a8, 0, sizeof(a8));
        memset(b8, 0, sizeof(b8));

        if ((s_rx_len >= 136u) && (pl[0] == 1u)) {
            uint8_t k;

            n = pl[2];
            en = pl[3];
            if (n > ENC_COMP_MAX_H) {
                st98 = (uint8_t)APP_PARAM_OUT_OF_RANGE;
            } else {
                for (k = 0u; k < n; k++) {
                    memcpy(&a8[k], &pl[8u + ((uint16_t)k * 8u)], 4);
                    memcpy(&b8[k], &pl[12u + ((uint16_t)k * 8u)], 4);
                }
                if (EncComp_SetHarm(n, a8, b8) == 0u) {
                    st98 = (uint8_t)APP_PARAM_OUT_OF_RANGE;
                } else {
                    EncComp_SetEnable(en);
                }
            }
            if ((st98 != (uint8_t)APP_PARAM_SUCCESS) && (s_locked < 0xFFFFu)) { s_locked++; }
        } else if ((s_rx_len >= 4u) && (pl[0] != 0u)) {
            st98 = (uint8_t)APP_PARAM_OUT_OF_RANGE;      /* 未知 cmd */
            if (s_locked < 0xFFFFu) { s_locked++; }
        }

        EncComp_GetHarm(a8, b8, &n);
        en = EncComp_GetEnable();
        p98[0] = 0u;
        p98[1] = st98;
        p98[2] = n;
        p98[3] = en;
        {
            uint8_t k;

            for (k = 0u; k < ENC_COMP_MAX_H; k++) {
                memcpy(&p98[8u + ((uint16_t)k * 8u)],  &a8[k], 4);
                memcpy(&p98[12u + ((uint16_t)k * 8u)], &b8[k], 4);
            }
        }
        ack_prepare(MAV_MSG_APP_ENC_COMP, p98, 136u);
        if (s_ok < 0xFFFFu) { s_ok++; }
        return;                          /* 已自行组应答 */
    }
    case MAV_MSG_APP_DEBUG_PARAM: {         /* 30082 运行时调参 */
        /* 掩码在 Debug_uint32[0]（偏移 64）；值为 Debug_float[i]（偏移 i*4）。
         * 短帧直接拒：少于 68 B 连掩码都读不全，按越界处理更安全。 */
        uint32_t mask = 0u;
        uint8_t i, n_ok = 0u;

        if (s_rx_len < 68u) {
            resp_len = 2u;
            resp[0] = status_byte(APP_PARAM_OUT_OF_RANGE);
            resp[1] = 0u;
            if (s_locked < 0xFFFFu) { s_locked++; }
            break;
        }
        memcpy(&mask, &pl[64], 4u);
        for (i = 0u; i < 16u; i++) {
            float v;
            if ((mask & (1UL << i)) == 0u) { continue; }
            memcpy(&v, &pl[(uint16_t)i * 4u], 4u);
            if ((s_dbg_param[i].set != 0) && (s_dbg_param[i].set(v) != 0u)) {
                n_ok++;
            } else if (s_locked < 0xFFFFu) {
                s_locked++;              /* 越界/未实现槽位：拒绝并留痕 */
            }
        }

        /* 整数通道（开关/档位）：掩码在 Debug_uint32[15]（偏移 124），
         * 值为 Debug_int32[i]（偏移 128 + i*4）。
         * 为什么要独立掩码：最常见的操作是"只切换一个开关"，若与 float
         * 共用掩码就得同时构造 16 个增益值，反而容易把增益写坏。 */
        if (s_rx_len >= 128u) {
            uint32_t imask = 0u;
            memcpy(&imask, &pl[124], 4u);
            for (i = 0u; i < 16u; i++) {
                int32_t iv;
                if ((imask & (1UL << i)) == 0u) { continue; }
                if ((uint16_t)(132u + ((uint16_t)i * 4u)) > s_rx_len) {
                    if (s_locked < 0xFFFFu) { s_locked++; }   /* 值没发全 */
                    continue;
                }
                memcpy(&iv, &pl[128u + ((uint16_t)i * 4u)], 4u);
                if ((s_dbg_int[i].set != 0) && (s_dbg_int[i].set(iv) != 0u)) {
                    n_ok++;
                } else if (s_locked < 0xFFFFu) {
                    s_locked++;
                }
            }
        }
        resp_len = 2u;
        resp[0] = status_byte(APP_PARAM_SUCCESS);
        resp[1] = n_ok;                  /* 成功写入的项数，上位机据此判断 */
        break;
    }
    case MAV_MSG_APP_COG_TABLE: {           /* 30093 齿槽表分块回读 */
        /* 只回读、不接收装表：`}` 辨识完固件自己 LoadFromIdent()，
         * 所以 PC → 固件的"下发"没有实际需求。上位机要的是核对 ——
         * 以前只能靠 ASCII 的 8 行 `[MI] CG` 拼出 128 点，那是唯一通道。
         * 请求：pl[0]=op（0 只回头部 / 1 要数据块）、pl[1]=块索引。
         * 每块 8 点，128 点共 16 块；块索引越界时 data 全 0 并置 status 非 0。 */
        uint8_t   op = (s_rx_len >= 1u) ? pl[0] : 0u;
        uint8_t   idx = (s_rx_len >= 2u) ? pl[1] : 0u;
        uint8_t   p93[48];
        int16_t   mv[8];
        uint8_t   k, got = 0u;
        CogFfInfo ci;

        CogFf_GetInfo(&ci);
        memset(p93, 0, sizeof(p93));
        p93[0] = op;
        p93[1] = idx;
        /* p93[2] 从"箱数"改成"块数"（2026-11-21）：表加密到 256/512 箱后
         * 256 已经放不进一个字节。块固定 8 点，所以 n_bins = n_blk * 8，
         * 两边都不丢信息，48 B 的布局也不用动。
         * p93[3]：bit0 = valid，bit1 = 本次装载做过相位对齐。 */
        p93[2] = (uint8_t)((ci.n_bins + 7U) / 8U);
        p93[3] = (uint8_t)((ci.valid ? 1U : 0U) | (ci.aligned ? 2U : 0U));
        memcpy(&p93[4],  &ci.pp, 4);
        memcpy(&p93[8],  &ci.phase_deg, 4);
        memcpy(&p93[12], &ci.zero_dev_deg, 4);

        if ((op == 1u) && (ci.valid != 0u)) {
            /* 块索引必须先在 16 位里算完再判范围。
             * 原来写 `(uint8_t)(idx * 8u)`：idx >= 32 时 32*8 = 256 被截成 0，
             * 于是返回第 0 块的数据而不是"越界" —— 上位机拼表会拿到重复块，
             * 而且 status 仍是成功，没有任何指示。这类"静默返回错数据"比崩溃难查。 */
            const uint16_t base = (uint16_t)((uint16_t)idx * 8u);

            if (base < (uint16_t)ci.n_bins) {
                got = CogFf_CopyTableMv(mv, base, 8u);
                for (k = 0u; k < got; k++) {
                    float a = (float)mv[k] * 0.001f;     /* mA → A */
                    memcpy(&p93[16u + ((uint16_t)k * 4u)], &a, 4);
                }
            } else {
                p93[1] = 0xFFu;      /* 越界：明确标记，别让上位机以为这块真是全 0 */
            }
        }
        ack_prepare(MAV_MSG_APP_COG_TABLE, p93, 48u);
        if (s_ok < 0xFFFFu) { s_ok++; }
        return;                          /* 已自行组应答 */
    }
    case MAV_MSG_APP_RECORD: {              /* 30094 录波：触发 / 配置 / 分块回传 */
        /* 替代 ASCII 的 `&`（触发）与 `'`（分频 + 通道编码在百位）。
         * op：0=QUERY 只读状态  1=START  2=STOP  3=DATA 取一块。
         * arg 的含义随 op 变：START 时是通道号（0 iq / 1 iu+θ / 2 vq积分），
         * DATA 时是块索引。每块 16 点（REC_N=1024 → 64 块）。
         * STOP 是 2026-11-21 补的：没有它，op=1 触发后只能等采满 1024 点
         * （div=100 时 5.12 s），期间 op=1/op=3 全部返回"忙"，改不了参数、
         * 也取不到数据 —— 唯一的绕法是软复位。 */
        uint8_t  op  = (s_rx_len >= 1u) ? pl[0] : 0u;
        uint8_t  arg = (s_rx_len >= 2u) ? pl[1] : 0u;
        uint8_t  div = (s_rx_len >= 3u) ? pl[2] : 0u;
        uint8_t  p94[144];
        uint8_t  st_code = 0u;      /* 0 成功 / 1 无输出或没数据 / 2 忙 / 3 参数错 */
        uint8_t  k;
        uint16_t n_pts, n_this = 0u, n_blk = 0u;

        memset(p94, 0, sizeof(p94));

        if (op == 1u) {                          /* START */
            if (BspServo_IsPwmEnabled() == 0u) {
                st_code = 1u;                    /* 没输出，采到的只会是零 */
            } else if (Servo_IsRecording() != 0u) {
                st_code = 2u;
            } else {
                if (div != 0u) { Servo_SetRecordDiv(div); }
                if (arg <= 3u) { Servo_SetRecordChannel(arg); }
                Servo_StartRecord();
            }
        } else if (op == 2u) {                   /* STOP：保留已采点数 */
            if (Servo_IsRecording() == 0u) {
                /* 已经停了（采满自动停，或重复发 STOP）。
                 * 不报错：重复发 STOP 是正常的幂等操作，上位机不必先查状态。 */
                if (Servo_GetRecordN() == 0u) { st_code = 1u; }
            } else {
                Servo_StopRecord();
            }
        } else if (op == 3u) {                   /* DATA */
            if (Servo_IsRecording() != 0u) {
                st_code = 2u;                    /* 还在采，别来取半截 */
            } else {
                n_pts = Servo_GetRecordN();
                if (n_pts == 0u) {
                    st_code = 1u;
                } else {
                    uint16_t base = (uint16_t)((uint16_t)arg * 16u);
                    for (k = 0u; (k < 16u) && ((uint16_t)(base + k) < n_pts); k++) {
                        float v  = rec_val_units(Servo_GetRecordAt((uint16_t)(base + k)));
                        float th = rec_th_units(Servo_GetRecordThetaAt((uint16_t)(base + k)));
                        memcpy(&p94[16u + ((uint16_t)k * 4u)], &v, 4);
                        memcpy(&p94[80u + ((uint16_t)k * 4u)], &th, 4);
                        n_this++;
                    }
                }
            }
        } else if (op != 0u) {
            st_code = 3u;
        }

        n_pts = Servo_GetRecordN();
        n_blk = (uint16_t)((n_pts + 15u) / 16u);
        p94[0]  = op;
        p94[1]  = arg;
        p94[2]  = Servo_GetRecordDiv();
        p94[3]  = Servo_GetRecordChannel();
        p94[4]  = (Servo_IsRecording() != 0u) ? 1u : ((n_pts != 0u) ? 2u : 0u);
        p94[5]  = (uint8_t)n_this;
        p94[6]  = (uint8_t)(n_pts & 0xFFu);
        p94[7]  = (uint8_t)((n_pts >> 8) & 0xFFu);
        p94[8]  = (uint8_t)(n_blk & 0xFFu);
        p94[9]  = (uint8_t)((n_blk >> 8) & 0xFFu);
        p94[10] = (uint8_t)(arg & 0xFFu);
        p94[12] = st_code;
        ack_prepare(MAV_MSG_APP_RECORD, p94, 144u);
        if (s_ok < 0xFFFFu) { s_ok++; }
        return;                          /* 已自行组应答 */
    }
    /* ---- 4 控制模式给定（2026-11-18）----
     * 统一应答：payload[0]=状态字节，其后是回读的真实给定值（工程单位）。
     * 写入被拒时回读仍是旧值，上位机可据此判断是否生效。 */
    case MAV_MSG_CURRENT_CTL_IN: {          /* 30025 力矩：idq_tar_A[1] @ off4 */
        float iq = ld_f32(&pl[4]);
        float iq_back;
        st = AppParam_SetTorqueIq(iq);
        resp_len = 5u;
        resp[0] = status_byte(st);
        iq_back = AppParam_GetTorqueIq();
        memcpy(&resp[1], &iq_back, 4);
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_APP_TORQUE_CMD: {          /* 30095 力矩：id 给定 + iq 斜坡时间常数 */
        /* 为什么不复用 30025：它的 idq_tar_A[0]（偏移 0）本来就是 id，但固件
         * 一直只取 [1]=iq，等于 id 字段从未被解释。现在把它打开，所有老脚本
         * 发 iq 时那个默认 0 就会顺手把 id 清零 —— 改既有协议语义的风险
         * 比新开一条大。所以 iq 继续走 30025，这里只补 ASCII 的 '/' 与 '"'。
         * `M`/`N`（±固定 iq 测试）不单独迁：等价于 30025 给那个常值。 */
        uint8_t  mask = pl[0];
        uint8_t  n_ok = 0u;
        uint8_t  bad  = 0u;
        float    id_r = 0.0f, iq_r = 0.0f, ramp_ms;

        if (s_rx_len < 12u) {
            resp_len = 2u;
            resp[0] = status_byte(APP_PARAM_OUT_OF_RANGE);
            resp[1] = 0u;
            if (s_locked < 0xFFFFu) { s_locked++; }
            break;
        }
        if ((mask & 0x01u) != 0u) {
            float id = ld_f32(&pl[4]);
            if ((id == id) && (fabsf(id) < 1.0e30f)) {
                Servo_SetCurIdRef(id);       /* 底层已按 ±SERVO_CUR_IQ_MAX 钳位 */
                n_ok++;
            } else {
                bad++;
            }
        }
        if ((mask & 0x02u) != 0u) {
            float ms = ld_f32(&pl[8]);
            if ((ms == ms) && (fabsf(ms) < 1.0e30f)) {
                Servo_SetCurRampTau(ms * 0.001f);   /* 底层钳 0..2 s */
                n_ok++;
            } else {
                bad++;
            }
        }
        if (bad != 0u) {
            if (s_locked < 0xFFFFu) { s_locked++; }
            resp[0] = status_byte(APP_PARAM_OUT_OF_RANGE);
        } else {
            resp[0] = status_byte(APP_PARAM_SUCCESS);
        }
        /* 回读真实生效值：上位机据此确认"写进去了"，不靠本地缓存 */
        Servo_GetCurRefs(&id_r, &iq_r);
        ramp_ms = Servo_GetCurRampTau() * 1000.0f;
        resp_len = 10u;
        resp[1] = n_ok;
        memcpy(&resp[2], &id_r, 4);
        memcpy(&resp[6], &ramp_ms, 4);
        break;
    }
    case MAV_MSG_POS_SPEED_CTL_IN: {        /* 30019：两路给定都缓存，按当前 op_mode 装配 */
        /* 原来只解析"当前模式对应的那一路"，切模式后另一路给定丢失。
         * 现在两条字段都进缓存；set 内部只装配当前模式那一路，另一路留作
         * 后续模式切换时由控制字使能直接装配。 */
        float p_rad = MAV_P2RAD(ld_i64(&pl[0]));   /* pos_tar_p：输出端角度（位置环已改输出端口径） */
        float w     = MAV_PS2RADS(ld_f32(&pl[28])); /* speed_tar_p_s @ off28（仍是电机端） */
        /* acc_tar_p_ss @ off32：脉冲/s² → rad/s²。0=旁带未填，跳过（避免每帧
         * 把加速度重置成 0 触发下限夹到 5）。非 0 才写入手写速度环斜坡。 */
        float a_pss = ld_f32(&pl[32]);
        float back;

        st = AppParam_SetPosSpeed(p_rad, w);
        if ((st == APP_PARAM_SUCCESS) && (a_pss != 0.0f)) {
            float a_rad_s2 = MAV_PS2RADS(a_pss);
            if (AppParam_SetSpeedAccel(a_rad_s2) != APP_PARAM_SUCCESS) {
                st = APP_PARAM_OUT_OF_RANGE;
            }
        }
        back = (AppParam_GetOpMode() == APP_OP_MODE_POS) ? AppParam_GetPosP()
                                                          : AppParam_GetSpeedW();
        resp_len = 5u;
        resp[0] = status_byte(st);
        memcpy(&resp[1], &back, 4);
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_VEL_PROFILE_CFG: {            /* 30089：速度轨迹配置 mode+j_max */
        uint8_t mode = pl[0];
        float j_max  = ld_f32(&pl[4]);
        uint8_t m_now;
        float   j_now;

        st = AppParam_SetSpeedProfile(mode, j_max);
        AppParam_GetSpeedProfile(&m_now, &j_now);
        resp_len = 9u;
        resp[0] = status_byte(st);
        resp[1] = m_now;
        memcpy(&resp[5], &j_now, 4);
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_MIT_CTL_CONFIG: {          /* 30066 MIT 配置：kp@0 kd@4 kt@12 */
        st = AppParam_SetMitConfig(ld_f32(&pl[0]), ld_f32(&pl[4]), ld_f32(&pl[12]));
        resp_len = 9u;
        resp[0] = status_byte(st);
        {
            float kp = 0.0f, kd = 0.0f;
            AppParam_GetMit(0, 0, &kp, &kd, 0);
            memcpy(&resp[1], &kp, 4);
            memcpy(&resp[5], &kd, 4);
        }
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_MIT_CTL_IN: {              /* 30065 MIT 给定：pos@0 tq@16 speed@24 */
        float p    = MAV_P2RAD(ld_i64(&pl[0])) * ACT_GEAR_RATIO;  /* pos：输出端→电机端 */
        float tau  = ld_f32(&pl[16]);
        float v    = MAV_PS2RADS(ld_f32(&pl[24])) * ACT_GEAR_RATIO; /* speed：输出端→电机端 */
        float kp = 0.0f, kd = 0.0f;
        AppParam_GetMit(0, 0, &kp, &kd, 0);   /* kp/kd 来自 30066 配置或默认值 */
        st = AppParam_SetMitCmd(p, v, kp, kd, tau);
        resp_len = 9u;                        /* 状态 + 目标p回读 + 实际p_now */
        resp[0] = status_byte(st);
        {
            float rp = 0.0f, p_now = 0.0f;
            AppParam_GetMit(&rp, 0, 0, 0, 0);
            rp /= ACT_GEAR_RATIO;             /* 回读输出端 */
            memcpy(&resp[1], &rp, 4);
            /* 实际位置（theta_multi 口径，rad）：直接用编码器实测角，而不是
             * Servo_GetMitState 的 s_mit_p_now —— 后者在刚使能那一拍还是旧值（0）。
             * 上位机用它贴当前位置起步，避免 kp·e 冲击。 */
            p_now = BspServo_GetThetaMulti() / ACT_GEAR_RATIO;   /* 回读输出端 */
            memcpy(&resp[5], &p_now, 4);
        }
        if (st != APP_PARAM_SUCCESS && s_locked < 0xFFFFu) { s_locked++; }
        break;
    }
    case MAV_MSG_READ_PARAM: {
        uint32_t struct_id;
        memcpy(&struct_id, pl, 4);
        if (struct_id == MAV_MSG_APP_STATUS_INFO) {
            /* 回读 30055 AppStatusInfo（46 B），只填首期能提供的字段 */
            uint8_t p[46];
            uint32_t sw = AppParam_GetStatusWord();
            uint32_t ew = AppParam_GetErrorWord();
            float vbus = BspServo_GetVbus();
            memset(p, 0, sizeof(p));
            memcpy(&p[0],  &sw,   4);
            memcpy(&p[4],  &ew,   4);
            memcpy(&p[8],  &vbus, 4);
            /* 回读实际在环模式（切环被拒时显示真实状态，而非缓存的期望模式） */
            p[44] = (uint8_t)AppParam_GetActiveMode();
            /* p[45]：链路自愈计数（高 4 位 = gState 强制 Abort 次数，
             * 低 4 位 = 应答超时丢弃次数，各饱和到 15）。
             * 放在 30055 的最后一个空闲字节：现场只要读一次状态字，
             * 就能判断"链路是不是刚刚自愈过"，不必再加新消息。 */
            p[45] = (uint8_t)((((s_uart_recover > 15u) ? 15u : s_uart_recover) << 4) |
                              ((s_ack_drop > 15u) ? 15u : s_ack_drop));
            ack_prepare(MAV_MSG_APP_STATUS_INFO, p, 46u);
        } else if ((struct_id == MAV_MSG_APP_CONTROL_WORD) ||
                   (struct_id == MAV_MSG_APP_OP_MODE) ||
                   (struct_id == MAV_MSG_APP_PERMISSION)) {
            uint8_t p[4];
            memset(p, 0, sizeof(p));
            if (struct_id == MAV_MSG_APP_CONTROL_WORD) {
                uint16_t cw = AppParam_GetControlWord();
                memcpy(p, &cw, 2);
                ack_prepare(struct_id, p, 2u);
            } else if (struct_id == MAV_MSG_APP_OP_MODE) {
                p[0] = (uint8_t)AppParam_GetOpMode();
                ack_prepare(struct_id, p, 1u);
            } else {
                p[0] = AppParam_GetCommAuthority();
                p[1] = AppParam_GetInternalAuthority();
                ack_prepare(struct_id, p, 2u);
            }
        } else if (struct_id == MAV_MSG_APP_DEBUG_PARAM) {
            /* 回填 16 个可调参数的当前值（192 B）。
             * 上位机的用法：先读一次拿到全表 → 改其中一项 → 带掩码写回。
             * 这样不用在 PC 侧维护一份参数默认值。 */
            uint8_t p[192];
            uint8_t i;
            uint32_t nvalid = 0u;
            uint32_t nvalid_i = 0u;

            memset(p, 0, sizeof(p));
            for (i = 0u; i < 16u; i++) {
                float v = (s_dbg_param[i].get != 0) ? s_dbg_param[i].get() : 0.0f;
                memcpy(&p[(uint16_t)i * 4u], &v, 4);
                if (s_dbg_param[i].get != 0) { nvalid++; }
            }
            /* 整数通道回填在 Debug_int32[i]（偏移 128 + i*4） */
            for (i = 0u; i < 16u; i++) {
                int32_t iv = (s_dbg_int[i].get != 0) ? s_dbg_int[i].get() : 0;
                memcpy(&p[128u + ((uint16_t)i * 4u)], &iv, 4);
                if (s_dbg_int[i].get != 0) { nvalid_i++; }
            }
            /* Debug_uint32[0] = 写入掩码（读回应恒 0）
             * Debug_uint32[1] = float 有效槽位数
             * Debug_uint32[2] = int   有效槽位数
             * 上位机靠这两个数判断"某个槽位是保留的还是真没实现"，
             * 不用在 PC 侧维护一份槽位表。 */
            memcpy(&p[64 + 4], &nvalid, 4);
            memcpy(&p[64 + 8], &nvalid_i, 4);
            ack_prepare(MAV_MSG_APP_DEBUG_PARAM, p, 192u);
        } else {
            if (s_rx_unknown < 0xFFFFu) { s_rx_unknown++; }
            return;                       /* 未支持的 struct_id：不回帧 */
        }
        if (s_ok < 0xFFFFu) { s_ok++; }
        return;                           /* READ_PARAM 已自行组应答 */
    }
    default:
        if (s_rx_unknown < 0xFFFFu) { s_rx_unknown++; }
        return;                           /* 静默丢弃未实现消息 */
    }

    ack_prepare(msgid, resp, resp_len);
    if (s_ok < 0xFFFFu) { s_ok++; }
}

void MavHost_Task(void)
{
    const uint32_t now = HAL_GetTick();

    /* 0) gState 卡死自愈。
     * 判据用"连续非 READY 超过 20 ms"而不是调用次数 —— 主循环调用频率不确定，
     * 按次数计数会在快主循环下立刻误触发 Abort。 */
    if (huart1.gState != HAL_UART_STATE_READY) {
        if (s_busy_since == 0u) {
            s_busy_since = now;
        } else if ((now - s_busy_since) >= MAV_GSTATE_STUCK_MS) {
            (void)HAL_UART_AbortTransmit(&huart1);
            huart1.gState = HAL_UART_STATE_READY;
            s_busy_since  = 0u;
            if (s_uart_recover < 0xFFFFu) { s_uart_recover++; }
            /* 应答缓冲可能被这次 Abort 打断，强制重来而不是继续等 */
            s_ack_since = 0u;
        }
    } else {
        s_busy_since = 0u;
    }

    /* 1) 先尝试发送上一拍遗留的应答（非阻塞， UART 忙则保留到下一拍） */
    if (s_ack_pending != 0U) {
        if (s_ack_since == 0u) { s_ack_since = now; }

        if (UartLink_TrySendBinary(s_ack, s_ack_len) != 0U) {
            s_ack_pending = 0U;
            s_ack_since   = 0u;
            /* 复位请求在应答真正发出后才执行，否则上位机收不到 ACK */
            if (s_reset_req != 0U) {
                s_reset_req = 0U;
                NVIC_SystemReset();
            }
        } else {
            if (s_tx_drop < 0xFFFFu) { s_tx_drop++; }  /* 仅计数，不丢帧 */
            /* 超时兜底：一条应答连续 30 ms 发不出去就丢弃。
             * 不丢的话下面会一直 return —— ACK 一条发不出，
             * 后面所有命令都不 dispatch，整条 MAVLink 控制链等于断了。 */
            if ((now - s_ack_since) >= MAV_ACK_STUCK_MS) {
                s_ack_pending = 0U;
                s_ack_since   = 0u;
                if (s_ack_drop < 0xFFFFu) { s_ack_drop++; }
            }
            /* 上一帧应答还在发：本拍不分发新帧。否则 ack_prepare() 会覆盖
             * s_ack，出现"指令执行了但上位机收不到 ACK"。UART 恢复后下一拍
             * 再分发，保证 ACK 与指令一一对应。 */
            return;
        }
    }

    /* 2) 分发待处理帧（一拍一帧，避免多条应答挤在同一拍） */
    if (s_pending != 0U) {
        uint8_t len = s_pend_len;
        s_pending = 0U;
        s_pend_len = 0U;
        /* dispatch 读的是 s_frame；注意 s_ack 与 s_frame 是不同缓冲，
         * 本拍刚发送的应答不会被新帧覆盖。 */
        if (len <= sizeof(s_frame)) { (void)len; dispatch(); }
    }
}

void MavHost_GetStats(uint16_t *ok, uint16_t *badcrc, uint16_t *unknown,
                      uint16_t *locked, uint16_t *txdrop)
{
    if (ok      != 0) { *ok      = s_ok; }
    if (badcrc  != 0) { *badcrc  = s_rx_crc_err; }
    if (unknown != 0) { *unknown = s_rx_unknown; }
    if (locked  != 0) { *locked  = s_locked; }
    if (txdrop  != 0) { *txdrop  = s_tx_drop; }
}

void MavHost_GetRxDiag(uint16_t *crc_err, uint16_t *drop)
{
    if (crc_err != 0) { *crc_err = s_rx_crc_err; }
    if (drop    != 0) { *drop    = s_drop; }
}
