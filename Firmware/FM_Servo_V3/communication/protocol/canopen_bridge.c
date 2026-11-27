/* canopen_bridge.c —— 串口(MAVLink) → CANopen 网关实现
 *
 * 设计要点见 canopen_bridge.h 的文件头说明。这里只补两个实现层面的取舍：
 *
 * ① 同对象合并（q_push）
 *    30019 位置/速度给定在网页上位机上可能 200~500 Hz 下发，而一条 SDO 事务
 *    约 0.3 ms。若照单全收，队列会一直积压、发出去的都是"旧给定"。所以入队
 *    时若队列里已有同一 (idx,sub)，就地覆盖成最新值，不再新增——保证总线上
 *    跑的是当下的目标，而不是历史。
 *
 * ② 超时 8 ms / 重试 2 次
 *    8 ms 是实测经验的保守值：1 Mbit/s 下一帧 8 B 约 130 µs，加上从站
 *    1 ms 节拍处理与应答，正常往返 <1 ms。8 ms 没应答基本就是丢帧或从站
 *    没进 Operational，重试两次仍无就放弃并计数（不无限重发堵总线）。
 */

#include "canopen_bridge.h"
#include "canopen.h"
#include "fdcan.h"

#include <string.h>

/* CANOPEN_BRIDGE=0（默认）时整个模块不参与编译：不占 flash，也不可能被误调。
 * 头文件里的声明保留无条件，这样调用点的 #if 判断失效时也能立刻在链接期暴露。 */
#if (CANOPEN_BRIDGE != 0)

#define SDO_Q_LEN       8u
#define SDO_TMO_MS      8u
#define SDO_RETRY_MAX   2u
#define NMT_START_MS    1000u

typedef struct {
    uint8_t idx_lo;
    uint8_t idx_hi;
    uint8_t sub;
    uint8_t n;              /* 1/2/4 字节 */
    uint8_t data[4];
} sdo_req_t;

static sdo_req_t s_q[SDO_Q_LEN];
static uint8_t   s_head;
static uint8_t   s_tail;
static uint8_t   s_inflight;
static uint16_t  s_tmo;
static uint8_t   s_retry;
static sdo_req_t s_cur;

/* 从站状态缓存 */
static uint16_t  s_sw;
static int32_t   s_pos;
static uint8_t   s_nmt;

/* 诊断计数 */
static uint16_t  s_ok;
static uint16_t  s_bad;
static uint16_t  s_tmo_cnt;
static uint32_t  s_abort;
static uint32_t  s_nmt_ms;

/* ---------------------------------------------------------------- 队列 */

/* 同对象合并后再入队（见文件头 ①） */
static void q_push(const sdo_req_t *r)
{
    uint8_t i = s_head;
    uint8_t nx;

    while (i != s_tail) {
        if ((s_q[i].idx_lo == r->idx_lo) && (s_q[i].idx_hi == r->idx_hi)
            && (s_q[i].sub == r->sub)) {
            s_q[i] = *r;                    /* 就地更新，不增长队列 */
            return;
        }
        i = (uint8_t)((i + 1u) % SDO_Q_LEN);
    }

    nx = (uint8_t)((s_tail + 1u) % SDO_Q_LEN);
    if (nx == s_head) {
        s_head = (uint8_t)((s_head + 1u) % SDO_Q_LEN);   /* 满：丢最旧 */
    }
    s_q[s_tail] = *r;
    s_tail = nx;
}

static void sdo_write_u8(uint16_t idx, uint8_t sub, uint8_t v)
{
    sdo_req_t r;
    r.idx_lo = (uint8_t)(idx & 0xFFu);
    r.idx_hi = (uint8_t)(idx >> 8);
    r.sub = sub;
    r.n = 1u;
    r.data[0] = v;
    r.data[1] = 0u;
    r.data[2] = 0u;
    r.data[3] = 0u;
    q_push(&r);
}

static void sdo_write_u16(uint16_t idx, uint8_t sub, uint16_t v)
{
    sdo_req_t r;
    r.idx_lo = (uint8_t)(idx & 0xFFu);
    r.idx_hi = (uint8_t)(idx >> 8);
    r.sub = sub;
    r.n = 2u;
    r.data[0] = (uint8_t)(v & 0xFFu);
    r.data[1] = (uint8_t)((v >> 8) & 0xFFu);
    r.data[2] = 0u;
    r.data[3] = 0u;
    q_push(&r);
}

static void sdo_write_u32(uint16_t idx, uint8_t sub, uint32_t v)
{
    sdo_req_t r;
    r.idx_lo = (uint8_t)(idx & 0xFFu);
    r.idx_hi = (uint8_t)(idx >> 8);
    r.sub = sub;
    r.n = 4u;
    r.data[0] = (uint8_t)(v & 0xFFu);
    r.data[1] = (uint8_t)((v >> 8) & 0xFFu);
    r.data[2] = (uint8_t)((v >> 16) & 0xFFu);
    r.data[3] = (uint8_t)((v >> 24) & 0xFFu);
    q_push(&r);
}

/* ---------------------------------------------------------------- 发送 */

static uint8_t sdo_tx(const sdo_req_t *r)
{
    uint8_t d[8];
    uint8_t ccs;

    /* 加速传送（expedited）：n 由 ccs 的低 2 位编码，n=0 表示无数据 */
    if (r->n == 1u)      { ccs = 0x2Fu; }
    else if (r->n == 2u) { ccs = 0x2Bu; }
    else                 { ccs = 0x23u; }

    d[0] = ccs;
    d[1] = r->idx_lo;
    d[2] = r->idx_hi;
    d[3] = r->sub;
    d[4] = r->data[0];
    d[5] = r->data[1];
    d[6] = r->data[2];
    d[7] = r->data[3];

    return Canopen_SendRaw((uint16_t)COB_SDO_RX, d, 8u);
}

/* ---------------------------------------------------------------- 对外 */

void CanBridge_Init(void)
{
    memset(s_q, 0, sizeof(s_q));
    s_head     = 0u;
    s_tail     = 0u;
    s_inflight = 0u;
    s_tmo      = 0u;
    s_retry    = 0u;

    s_sw  = 0x0040u;        /* CiA402 上电 = Switch on disabled */
    s_pos = 0;
    s_nmt = 0u;             /* 0 = 尚未收到心跳，从站未上线 */

    s_ok      = 0u;
    s_bad     = 0u;
    s_tmo_cnt = 0u;
    s_abort   = 0u;
    s_nmt_ms  = 0u;
}

void CanBridge_Task1ms(void)
{
    uint8_t d[2];

    /* NMT Start 周期重发：从站上电静默，只认 START。重发对已在 Operational
     * 的从站无害（状态不倒退），但能兜住"上电顺序错开"的情况。 */
    if (++s_nmt_ms >= NMT_START_MS) {
        s_nmt_ms = 0u;
        d[0] = 0x01u;                       /* 01 = Start remote node */
        d[1] = (uint8_t)CANOPEN_NODE_ID;
        (void)Canopen_SendRaw((uint16_t)COB_NMT, d, 2u);
    }

    if (s_inflight != 0u) {
        if (s_tmo > 0u) {
            s_tmo--;
            return;
        }
        if (s_retry < SDO_RETRY_MAX) {
            s_retry++;
            s_tmo = SDO_TMO_MS;
            (void)sdo_tx(&s_cur);
            return;
        }
        s_tmo_cnt++;                        /* 重试耗尽：放弃，不堵总线 */
        s_inflight = 0u;
        return;
    }

    if (s_head == s_tail) {
        return;                             /* 空队列 */
    }

    s_cur  = s_q[s_head];
    s_head = (uint8_t)((s_head + 1u) % SDO_Q_LEN);
    if (sdo_tx(&s_cur) != 0u) {
        s_inflight = 1u;
        s_tmo      = SDO_TMO_MS;
        s_retry    = 0u;
    }
    /* 发送失败（FIFO 满）不抛出：s_cur 已被消费，下一拍若队列空则不会重试。
     * 这是有意的——给定每 10 ms 就刷新一次，重试旧值没有意义。 */
}

uint8_t CanBridge_OnFrame(uint32_t cob, const uint8_t *d, uint8_t dlc)
{
    if (cob == (uint32_t)COB_SDO_TX) {
        if (dlc < 8u) {
            return 1u;
        }
        switch (d[0]) {
        case 0x60u:                         /* 下载（写）应答 */
            s_ok++;
            s_inflight = 0u;
            break;
        case 0x80u:                         /* abort：4 字节中止码 */
            s_bad++;
            s_abort = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
                    | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
            s_inflight = 0u;
            break;
        default:
            if ((d[0] & 0xE0u) == 0x40u) {  /* 上传（读）应答 */
                s_ok++;
                s_inflight = 0u;
            }
            break;
        }
        return 1u;
    }

    if (cob == (uint32_t)COB_TPDO1) {
        /* 从站 0x1A00 映射：0x6041(u16) + 0x6064(i32)，小端 */
        if (dlc >= 6u) {
            s_sw  = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
            s_pos = (int32_t)((uint32_t)d[2] | ((uint32_t)d[3] << 8)
                            | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24));
        }
        return 1u;
    }

    if (cob == (uint32_t)COB_HB) {
        if (dlc >= 1u) {
            s_nmt = d[0];                   /* 01=Boot-up 05=Operational 7F=Pre-op */
        }
        return 1u;
    }

    return 0u;                              /* 不是本模块的帧 */
}

void CanBridge_SetMode(int8_t mode)
{
    sdo_write_u8(0x6060u, 0u, (uint8_t)mode);
}

void CanBridge_SetControlWord(uint16_t cw)
{
    sdo_write_u16(0x6040u, 0u, cw);
}

void CanBridge_SetTargetPos(int32_t pulse)
{
    sdo_write_u32(0x607Au, 0u, (uint32_t)pulse);
}

void CanBridge_SetTargetVel(int32_t pulse_s)
{
    sdo_write_u32(0x60FFu, 0u, (uint32_t)pulse_s);
}

void CanBridge_SetTorque(int32_t mnm)
{
    sdo_write_u32(0x2004u, 0u, (uint32_t)mnm);
}

void CanBridge_GetStatus(uint16_t *sw, int32_t *pos, uint8_t *nmt)
{
    if (sw  != 0) { *sw  = s_sw;  }
    if (pos != 0) { *pos = s_pos; }
    if (nmt != 0) { *nmt = s_nmt; }
}

void CanBridge_GetStats(uint16_t *ok, uint16_t *bad, uint16_t *tmo,
                        uint8_t *q_depth, uint32_t *last_abort)
{
    if (ok   != 0) { *ok   = s_ok;      }
    if (bad  != 0) { *bad  = s_bad;     }
    if (tmo  != 0) { *tmo  = s_tmo_cnt; }
    if (q_depth != 0) {
        *q_depth = (uint8_t)((s_tail + SDO_Q_LEN - s_head) % SDO_Q_LEN);
    }
    if (last_abort != 0) { *last_abort = s_abort; }
}

#endif /* CANOPEN_BRIDGE */
