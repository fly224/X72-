/*
 * canopen.c —— 标准 CANopen 从站（CiA 301 + CiA 402）第一版
 *
 * 对齐：雷赛 H2X-CAN506《CANopen 技术指导手册》
 *   §2.3 预定义连接集、§3.1.2 PDS 状态机、附录 D 的 SDO 报文序列。
 *
 * 本阶段（P1）只做：对象字典 + SDO + NMT + Heartbeat。
 *   PDO 与 PP/PV 模式接入留到 P3/P4 —— 但 OD 里对应的对象已经建好，
 *   到时只需在 PDO 回调里读写它们，不用改字典结构。
 *
 * 帧格式：标准 CANopen 用**经典 CAN 帧（8 字节、BRS 关）**。
 *   can_proto.c 的 can_l1_send() 发的是 FD_CAN + BRS，主站不认，所以这里
 *   自带 canopen_send()。FDCAN 外设允许同一总线上混合两种格式。
 */
#include "canopen.h"
#include "fdcan.h"
#include "bsp_servo.h"
#include "app_param.h"
#include "servo_cmd.h"
#include "pos_loop.h"
#include <string.h>

#if defined(CANOPEN_ENABLE) && (CANOPEN_ENABLE == 0)
/* 允许整模块编译期关闭（与 can_proto 的自定义协议二选一时用）。
 * 注意：这里不能引用下面 #else 分支里的静态变量，只能返回常量。 */
void     Canopen_Init(void) {}
void     Canopen_Task1ms(void) {}
uint8_t  Canopen_OnFrame(uint32_t cob, const uint8_t *d, uint8_t dlc)
{ (void)cob; (void)d; (void)dlc; return 0u; }
uint16_t Canopen_StatusWord(void) { return 0x0040u; }
uint8_t  Canopen_NmtState(void)   { return (uint8_t)NMT_PREOP; }
uint16_t Canopen_PulsePerRev(void){ return 10000u; }
int8_t   Canopen_Mode(void)       { return 0; }
void     Canopen_SetFeedback(int32_t p, int32_t v) { (void)p; (void)v; }
#else

/* ===================== 对象字典 ===================== */

typedef enum { OT_U8 = 1, OT_I8, OT_U16, OT_I16, OT_U32, OT_I32 } od_type_t;
typedef enum { OA_RO = 0, OA_WO, OA_RW } od_access_t;

typedef struct {
    uint16_t index;
    uint8_t  sub;
    uint8_t  type;        /* od_type_t */
    uint8_t  access;      /* od_access_t */
    void    *ptr;
} od_item_t;

/* ---- 通讯子协议区（1000h~1FFFh）---- */
static uint32_t od_1000_devtype   = 0x00020192u;  /* 402 伺服驱动 */
static uint8_t  od_1001_errreg    = 0u;
static uint32_t od_1005_cob_sync  = COB_SYNC;
static uint32_t od_1006_cycle_us  = 1000u;
static uint32_t od_1014_emcy_cob  = COB_EMCY;
static uint16_t od_1017_hb_ms     = 0u;       /* 0=关闭：上电完全静默，避免抢占总线 */

/* ---- 制造商特定区 ---- */
static uint32_t od_2000_ppr       = 10000u;       /* 脉冲当量： pulse/输出圈 */
static uint32_t od_2001_gear      = 9u;           /* 减速比（与 ACT_GEAR_RATIO 一致） */
/* MIT 模式的三个增益/前馈量（CiA402 无标准对象，厂商区自定义）：
 *  · 0x2002 kp、0x2003 kd：**电机端**原值 ×1000（直接填固件里调好的 15/30/60），
 *    不做出入口径换算 —— 它们是环路内部参数，按输出端折算会得到 0.007 这种不可读的数。
 *  · 0x2004 tau：**输出端** mN·m，内部 ÷ 减速比 转成电机端 N·m。 */
static uint32_t od_2002_kp        = 15000u;       /* 15.0 N·m/rad（电机端） */
static uint32_t od_2003_kd        = 300u;         /* 0.3 N·m·s/rad（电机端） */
static int32_t  od_2004_tau_mnm   = 0;            /* 输出端 mN·m */
/* 诊断用：最后收到的 RPDO1 原始载荷（8 字节拆两个 u32）。
 * 为什么需要：SWD 读 od_607A 时主站每 300 ms 在改，读到的是撕裂值（实测 0/5/-8 都出现过），
 * 无法判断"收到的字节到底对不对"。存一份快照再由 SDO 读，才是可信的。 */
static uint32_t od_2005_rpdo_lo   = 0;
static uint32_t od_2006_rpdo_hi   = 0;

/* ---- 设备控制区（CiA 402）---- */
static uint16_t od_6040_cw        = 0u;
static uint16_t od_6041_sw        = 0x0040u;      /* 上电 = Switch on disabled */
static int8_t   od_6060_mode      = 0;
static int8_t   od_6061_mode_disp = 0;
static int32_t  od_6064_pos       = 0;            /* 位置反馈 pulse */
static int32_t  od_606C_vel       = 0;            /* 速度反馈 pulse/s */
static int32_t  od_607A_tgt_pos   = 0;
static uint32_t od_6081_prof_vel  = 0;
static uint32_t od_6083_acc       = 0;
static uint32_t od_6084_dec       = 0;
static uint32_t od_6085_qs_dec    = 0;
static int32_t  od_60FF_tgt_vel   = 0;

/* ---- PDO 通讯参数 / 映射（P3）----
 * 手册附录 D 的配置流程要求主站能**重映射** PDO，所以这些对象必须是可写的；
 * 只读的"固定映射"会让标准主站的配置序列在写 0x1600 时被 abort。
 *
 * 映射条目编码（CiA 301 §7.2.2）：bit0-7 长度(bit)、bit8-15 子索引、bit16-31 索引。
 * COB-ID 的 bit31 = 1 表示该 PDO 无效（禁止使用）。 */
static uint8_t  od_1400_nsub     = 3u;                    /* 0x1400 sub0 */
static uint32_t od_1400_cob      = 0x200u + CANOPEN_NODE_ID;
static uint8_t  od_1400_type     = 255u;                  /* 255 = 异步事件 */
static uint16_t od_1400_evt_ms   = 0u;                    /* sub3 事件定时器 */
static uint8_t  od_1600_nmap     = 2u;                    /* 0x1600 sub0：映射个数 */
static uint32_t od_1600_map[4]   = { 0x60400010u, 0x607A0020u, 0u, 0u };

static uint8_t  od_1800_nsub     = 3u;                    /* 0x1800 sub0 */
static uint32_t od_1800_cob      = 0x180u + CANOPEN_NODE_ID;
static uint8_t  od_1800_type     = 255u;
static uint16_t od_1800_evt_ms   = 0u;
/* RPDO2：专供 MIT 的 dq + tau（RPDO1 只放 q）。 */
static uint8_t  od_1401_nsub     = 3u;
static uint32_t od_1401_cob      = 0x300u + CANOPEN_NODE_ID;
static uint8_t  od_1401_type     = 255u;
static uint16_t od_1401_evt_ms   = 0u;
static uint8_t  od_1601_nmap     = 2u;
static uint32_t od_1601_map[4]   = { 0x60FF0020u, 0x20040020u, 0u, 0u };

static uint8_t  od_1A00_nmap     = 2u;
static uint32_t od_1A00_map[4]   = { 0x60410010u, 0x60640020u, 0u, 0u };

#define OD_N  (sizeof(s_od) / sizeof(s_od[0]))

static const od_item_t s_od[] = {
    /* 通讯区 */
    { 0x1000u, 0u, OT_U32, OA_RO, &od_1000_devtype  },
    { 0x1001u, 0u, OT_U8,  OA_RO, &od_1001_errreg   },
    { 0x1005u, 0u, OT_U32, OA_RW, &od_1005_cob_sync },
    { 0x1006u, 0u, OT_U32, OA_RW, &od_1006_cycle_us },
    { 0x1014u, 0u, OT_U32, OA_RW, &od_1014_emcy_cob },
    { 0x1017u, 0u, OT_U16, OA_RW, &od_1017_hb_ms    },   /* 实际只用低 16 位 */
    /* 制造商区 */
    { 0x2000u, 0u, OT_U32, OA_RW, &od_2000_ppr      },
    { 0x2001u, 0u, OT_U32, OA_RW, &od_2001_gear     },
    { 0x2002u, 0u, OT_U32, OA_RW, &od_2002_kp       },
    { 0x2003u, 0u, OT_U32, OA_RW, &od_2003_kd       },
    { 0x2004u, 0u, OT_I32, OA_RW, &od_2004_tau_mnm  },
    { 0x2005u, 0u, OT_U32, OA_RO, &od_2005_rpdo_lo  },
    { 0x2006u, 0u, OT_U32, OA_RO, &od_2006_rpdo_hi  },
    /* 设备控制区（CiA 402） */
    { 0x6040u, 0u, OT_U16, OA_RW, &od_6040_cw       },
    { 0x6041u, 0u, OT_U16, OA_RO, &od_6041_sw       },
    { 0x6060u, 0u, OT_I8,  OA_RW, &od_6060_mode     },
    { 0x6061u, 0u, OT_I8,  OA_RO, &od_6061_mode_disp},
    { 0x6064u, 0u, OT_I32, OA_RO, &od_6064_pos      },
    { 0x606Cu, 0u, OT_I32, OA_RO, &od_606C_vel      },
    { 0x607Au, 0u, OT_I32, OA_RW, &od_607A_tgt_pos  },
    { 0x6081u, 0u, OT_U32, OA_RW, &od_6081_prof_vel },
    { 0x6083u, 0u, OT_U32, OA_RW, &od_6083_acc      },
    { 0x6084u, 0u, OT_U32, OA_RW, &od_6084_dec      },
    { 0x6085u, 0u, OT_U32, OA_RW, &od_6085_qs_dec   },
    { 0x60FFu, 0u, OT_I32, OA_RW, &od_60FF_tgt_vel  },
    /* PDO 参数与映射（P3）：允许主站按手册附录 D 的序列重映射 */
    { 0x1400u, 0u, OT_U8,  OA_RO, &od_1400_nsub    },
    { 0x1400u, 1u, OT_U32, OA_RW, &od_1400_cob     },
    { 0x1400u, 2u, OT_U8,  OA_RW, &od_1400_type    },
    { 0x1400u, 3u, OT_U16, OA_RW, &od_1400_evt_ms  },
    { 0x1600u, 0u, OT_U8,  OA_RW, &od_1600_nmap    },
    { 0x1600u, 1u, OT_U32, OA_RW, &od_1600_map[0]  },
    { 0x1600u, 2u, OT_U32, OA_RW, &od_1600_map[1]  },
    { 0x1600u, 3u, OT_U32, OA_RW, &od_1600_map[2]  },
    { 0x1600u, 4u, OT_U32, OA_RW, &od_1600_map[3]  },
    { 0x1401u, 0u, OT_U8,  OA_RO, &od_1401_nsub    },
    { 0x1401u, 1u, OT_U32, OA_RW, &od_1401_cob     },
    { 0x1401u, 2u, OT_U8,  OA_RW, &od_1401_type    },
    { 0x1401u, 3u, OT_U16, OA_RW, &od_1401_evt_ms  },
    { 0x1601u, 0u, OT_U8,  OA_RW, &od_1601_nmap    },
    { 0x1601u, 1u, OT_U32, OA_RW, &od_1601_map[0]  },
    { 0x1601u, 2u, OT_U32, OA_RW, &od_1601_map[1]  },
    { 0x1601u, 3u, OT_U32, OA_RW, &od_1601_map[2]  },
    { 0x1601u, 4u, OT_U32, OA_RW, &od_1601_map[3]  },
    { 0x1800u, 0u, OT_U8,  OA_RO, &od_1800_nsub    },
    { 0x1800u, 1u, OT_U32, OA_RW, &od_1800_cob     },
    { 0x1800u, 2u, OT_U8,  OA_RW, &od_1800_type    },
    { 0x1800u, 3u, OT_U16, OA_RW, &od_1800_evt_ms  },
    { 0x1A00u, 0u, OT_U8,  OA_RW, &od_1A00_nmap    },
    { 0x1A00u, 1u, OT_U32, OA_RW, &od_1A00_map[0]  },
    { 0x1A00u, 2u, OT_U32, OA_RW, &od_1A00_map[1]  },
    { 0x1A00u, 3u, OT_U32, OA_RW, &od_1A00_map[2]  },
    { 0x1A00u, 4u, OT_U32, OA_RW, &od_1A00_map[3]  },
};

static const od_item_t *od_find(uint16_t idx, uint8_t sub)
{
    uint16_t i;
    for (i = 0u; i < (uint16_t)OD_N; i++) {
        if ((s_od[i].index == idx) && (s_od[i].sub == sub)) { return &s_od[i]; }
    }
    return 0;
}

/* 对象宽度（字节） */
static uint8_t od_width(uint8_t type)
{
    switch (type) {
    case OT_U8: case OT_I8:   return 1u;
    case OT_U16: case OT_I16: return 2u;
    default:                  return 4u;
    }
}

/* 按类型写入（小端，签名扩展由调用方保证） */
static void od_write(const od_item_t *it, const uint8_t *d)
{
    switch (it->type) {
    case OT_U8:  *(uint8_t *)it->ptr  = d[0]; break;
    case OT_I8:  *(int8_t *)it->ptr   = (int8_t)d[0]; break;
    case OT_U16: *(uint16_t *)it->ptr = (uint16_t)(d[0] | ((uint16_t)d[1] << 8)); break;
    case OT_I16: *(int16_t *)it->ptr  = (int16_t)(uint16_t)(d[0] | ((uint16_t)d[1] << 8)); break;
    case OT_U32: *(uint32_t *)it->ptr = (uint32_t)d[0] | ((uint32_t)d[1] << 8)
                                      | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24); break;
    default:     *(int32_t *)it->ptr  = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8)
                                      | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24)); break;
    }
}

static void od_read(const od_item_t *it, uint8_t *d)
{
    switch (it->type) {
    case OT_U8:  d[0] = *(uint8_t *)it->ptr; break;
    case OT_I8:  d[0] = (uint8_t)*(int8_t *)it->ptr; break;
    case OT_U16: { uint16_t v = *(uint16_t *)it->ptr; d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); } break;
    case OT_I16: { uint16_t v = (uint16_t)*(int16_t *)it->ptr; d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); } break;
    default: {
        uint32_t v = (it->type == OT_U32) ? *(uint32_t *)it->ptr
                                          : (uint32_t)*(int32_t *)it->ptr;
        d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8);
        d[2] = (uint8_t)(v >> 16); d[3] = (uint8_t)(v >> 24);
    } break;
    }
}

/* ===================== 发送（经典 CAN 帧） ===================== */

static uint8_t canopen_send(uint16_t cob, const uint8_t *buf, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx;

    tx.Identifier          = cob;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = (uint32_t)len;       /* FDCAN 的 DLC 编码：0~8 直接用数值 */
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;       /* 标准 CANopen 不用 BRS */
    tx.FDFormat            = FDCAN_CLASSIC_CAN;   /* 关键：经典帧，不是 FD */
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0u;
    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)buf) == HAL_OK) ? 1u : 0u;
}

/* ===================== SDO（0x601 收 / 0x581 发） ===================== */

/* Abort code（CiA 301 §7.2.4.3.4） */
#define ABORT_NO_OBJECT      (0x06020000UL)
#define ABORT_READ_ONLY      (0x06010002UL)
#define ABORT_WRITE_ONLY     (0x06010001UL)
#define ABORT_SUB_NOEXIST    (0x06090011UL)
#define ABORT_TYPE_MISMATCH  (0x06070010UL)
#define ABORT_GENERAL        (0x08000000UL)

static void sdo_abort(uint16_t idx, uint8_t sub, uint32_t code)
{
    uint8_t r[8];
    r[0] = 0x80u;
    r[1] = (uint8_t)(idx & 0xFFu);
    r[2] = (uint8_t)(idx >> 8);
    r[3] = sub;
    r[4] = (uint8_t)code; r[5] = (uint8_t)(code >> 8);
    r[6] = (uint8_t)(code >> 16); r[7] = (uint8_t)(code >> 24);
    (void)canopen_send(COB_SDO_TX, r, 8u);
}

static void sdo_handle(const uint8_t *d, uint8_t dlc)
{
    const uint8_t  ccs = (uint8_t)(d[0] >> 5);            /* 1=下载 2=上传 3/4=分段 */
    const uint16_t idx = (uint16_t)(d[1] | ((uint16_t)d[2] << 8));
    const uint8_t  sub = d[3];
    const od_item_t *it;
    uint8_t r[8];

    /* 标准要求 SDO 帧 DLC=8，但手册附录 D 的示例把尾部补零省略了（5~7 字节）。
     * 现场主站两种都有，所以只要求头部 4 字节完整，数据段按需另校验。 */
    if (dlc < 4u) { sdo_abort(idx, sub, ABORT_GENERAL); return; }

    it = od_find(idx, sub);
    if (it == 0) {
        /* 索引存在但子索引不存在，还是整个对象都没有——标准区分，这里统一报"无对象"，
         * 因为本实现全部是单子索引对象。 */
        sdo_abort(idx, sub, ABORT_NO_OBJECT);
        return;
    }

    if (ccs == 1u) {                       /* ---- 下载（主站写）---- */
        /* 快速传输：byte0 bit1=1 表示 expedited，bit2=1 表示长度有效 */
        const uint8_t expedited = (uint8_t)((d[0] >> 1) & 1u);
        if (expedited == 0u) { sdo_abort(idx, sub, ABORT_GENERAL); return; }  /* 不支持分段 */

        /* CiA 301 §7.2.4.3.2：bit2=1 表示长度有效，bit3-4 是**未使用**字节数，
         * 有效字节 = 4 - n。原来写成 3 - n，所有下载都会算成长度不符而被 abort。 */
        const uint8_t n = (uint8_t)(4u - ((d[0] >> 2) & 3u));
        const uint8_t need = od_width(it->type);

        if (it->access == OA_RO) { sdo_abort(idx, sub, ABORT_READ_ONLY); return; }
        if (n != need) { sdo_abort(idx, sub, ABORT_TYPE_MISMATCH); return; }
        if (dlc < (uint8_t)(4u + n)) { sdo_abort(idx, sub, ABORT_GENERAL); return; }

        od_write(it, &d[4]);

        /* 写入 6040 控制字时立刻跑一次状态机（P1 阶段先只更新状态字） */
        r[0] = 0x60u;
        r[1] = (uint8_t)(idx & 0xFFu); r[2] = (uint8_t)(idx >> 8); r[3] = sub;
        memset(&r[4], 0, 4);
        (void)canopen_send(COB_SDO_TX, r, 8u);
        return;
    }

    if (ccs == 2u) {                       /* ---- 上传（主站读）---- */
        const uint8_t need = od_width(it->type);

        if (it->access == OA_WO) { sdo_abort(idx, sub, ABORT_WRITE_ONLY); return; }

        /* 上传应答：0x43(4B) / 0x47(3B) / 0x4B(2B) / 0x4F(1B)，
         * 即 0x43 | (未使用字节数 << 2)。原来写成减法，2 字节对象会回 0x41。 */
        r[0] = (uint8_t)(0x43u | ((4u - need) << 2));
        r[1] = (uint8_t)(idx & 0xFFu); r[2] = (uint8_t)(idx >> 8); r[3] = sub;
        memset(&r[4], 0, 4);
        od_read(it, &r[4]);
        (void)canopen_send(COB_SDO_TX, r, 8u);
        return;
    }

    sdo_abort(idx, sub, ABORT_GENERAL);     /* 分段传输不支持 */
}

/* ===================== PDO（P3） =====================
 *
 * 支持主站按手册附录 D 的序列**重映射**：0x1600/0x1A00 的映射表可写，
 * 打包/解包时实时按表解析 —— 所以不允许"固定映射只读"，否则标准主站
 * 写映射那一刻就会被 abort，整条配置链断掉。
 *
 * 长度按映射条目的 bit0-7 取（8/16/32 bit），本实现的 OD 全是这几种宽度。
 */

static uint8_t pdo_pack(uint8_t *buf, const uint32_t *map, uint8_t n)
{
    uint8_t len = 0u, i, b;

    for (i = 0u; i < n; i++) {
        const uint16_t idx  = (uint16_t)(map[i] >> 16);
        const uint8_t  sub  = (uint8_t)((map[i] >> 8) & 0xFFu);
        const uint8_t  bits = (uint8_t)(map[i] & 0xFFu);
        const od_item_t *it = od_find(idx, sub);
        uint8_t raw[4];
        uint8_t nb;

        if ((it == 0) || (bits == 0u)) { continue; }
        nb = (uint8_t)(bits / 8u);
        if (nb > od_width(it->type)) { nb = od_width(it->type); }
        if ((uint16_t)(len + nb) > 8u) { break; }      /* 经典帧最多 8 字节 */
        od_read(it, raw);
        for (b = 0u; b < nb; b++) { buf[len++] = raw[b]; }
    }
    return len;
}

static void pdo_unpack(const uint8_t *buf, uint8_t dlc, const uint32_t *map, uint8_t n)
{
    uint8_t len = 0u, i, b;

    for (i = 0u; i < n; i++) {
        const uint16_t idx  = (uint16_t)(map[i] >> 16);
        const uint8_t  sub  = (uint8_t)((map[i] >> 8) & 0xFFu);
        const uint8_t  bits = (uint8_t)(map[i] & 0xFFu);
        const od_item_t *it = od_find(idx, sub);
        uint8_t raw[4] = { 0u, 0u, 0u, 0u };
        uint8_t nb;

        if ((it == 0) || (bits == 0u)) { continue; }
        nb = (uint8_t)(bits / 8u);
        if ((uint16_t)(len + nb) > dlc) { break; }
        for (b = 0u; b < nb; b++) { raw[b] = buf[len++]; }
        if (it->access != OA_RO) { od_write(it, raw); }
    }
}

/* TPDO1 发送：bit31=1 表示被主站禁止 */
static void pdo_send_tpdo1(void)
{
    uint8_t b[8];
    uint8_t n;

    if ((od_1800_cob & 0x80000000UL) != 0u) { return; }
    n = pdo_pack(b, od_1A00_map, od_1A00_nmap);
    if (n != 0u) {
        (void)canopen_send((uint16_t)(od_1800_cob & 0x7FFu), b, n);
    }
}

/* ===================== NMT 与心跳 ===================== */

static canopen_nmt_t s_nmt = NMT_BOOTUP;
static uint16_t      s_hb_ms;
static uint8_t       s_boot_pending = 1u;

static void nmt_handle(const uint8_t *d, uint8_t dlc)
{
    if (dlc < 2u) { return; }
    const uint8_t cmd  = d[0];
    const uint8_t node = d[1];

    if ((node != 0u) && (node != (uint8_t)CANOPEN_NODE_ID)) { return; }  /* 不是发给我的 */

    switch (cmd) {
    case 0x01u: s_nmt = NMT_OPERATIONAL; break;      /* Start：Task1ms 里会补发 Boot-up */
    case 0x02u: s_nmt = NMT_STOPPED;     break;      /* Stop Remote Node */
    case 0x80u: s_nmt = NMT_PREOP;       break;      /* Enter Pre-operational */
    case 0x81u:                                       /* Reset Node */
    case 0x82u:                                       /* Reset Communication */
        s_nmt = NMT_PREOP;
        s_boot_pending = 1u;
        break;
    default: break;
    }
}

/* ===================== CiA402 状态机 + 模式桥接（P4） =====================
 *
 * 状态与迁移完全照手册 §3.1.2（与 can_proto.c 里那套一致，但那套服务于
 * 自定义 L1 协议；两条协议栈独立，各管各的状态）。
 * 手册比 CiA 301 多两条细节，这里按手册实现：
 *   · Switched on(0x23) 收 0x0000 → Ready to switch on(0x21)
 *   · Quick stop active(0x07) 收 0x000F → Operation enabled(0x27)
 */
typedef enum {
    ST_NOT_READY = 0, ST_SWITCH_ON_DISABLED, ST_READY_TO_SWITCH_ON,
    ST_SWITCHED_ON, ST_OPERATION_ENABLED, ST_QUICK_STOP,
    ST_FAULT
} cia_state_t;

static cia_state_t s_state = ST_NOT_READY;
/* CAN 通信超时（CiA402 安全要求）：PDO 断了必须停机。
 * 没这条的后果实测过：主站停发后从站一直 armed=True，串口 Disarm 也被
 * 状态机每拍的"重新使能"顶掉，只能软复位才停得住 —— 现场就是主站崩了电机还在通电保持。 */
#define CANOPEN_RX_TMO_MS   (500u)
static uint32_t    s_rx_tick;
static uint16_t    s_cw_last;
static int8_t      s_mode_last;

/* ================= 三环口径约定（务必读，否则会静默偏 9 倍）=================
 *
 * 减速比 ACT_GEAR_RATIO = 9（app_param.h:52，与 can_proto.c 的 CAN_GEAR_RATIO
 * 有 _Static_assert 强制相等）。0x2000 脉冲当量是**输出端**每圈脉冲数（默认 10000）。
 *
 * 三个环的**坐标口径**并不统一（2026-11-26 逐文件核对 servo_isr.c 确认）：
 *
 *   环         口径          依据（servo_isr.c）
 *   位置环     输出端 rad     pos_coord_now() = theta_multi / ACT_GEAR_RATIO  (L1398)
 *   速度环     电机端 rad/s   速度给定直接对 BspServo_GetOmegaMech() 做差（电机端）
 *   MIT 环     电机端 rad     ep = s_mit_p_shape - theta_multi，theta_multi 是电机端多圈角 (L2788)
 *
 * 历史：远程 8f25e49「位置环输出端口径重构」把位置环坐标从电机端改成了输出端
 * （Vmax/Amax/Errmax ÷9，Kp 不变）；速度环、MIT 环**没有改**，仍是电机端。
 *
 * 因此 pulse（输出端）换算到内部口径时：
 *   位置 0x607A/0x6064  → 输出端 rad，**直接**按 2π/ppr，不乘减速比；
 *   速度 0x60FF/0x606C  → 输出端 pulse/s，先 2π/ppr 转输出端 rad/s，再 **× 减速比** 到电机端；
 *   MIT   q/dq          → 输出端，先 2π/ppr，再 **× 减速比** 到电机端；
 *   MIT   tau           → 输出端 mN·m，×0.001 转 N·m 再 **÷ 减速比** 到电机端；
 *   MIT   kp/kd         → 已是电机端 N·m/rad，/1000 还原，不做折算。
 *
 * 反馈（6064/606C 注入）反过来：编码器给的是电机端（theta_multi/omega_mech），
 * 两者都要 ÷ 减速比 才得到输出端 pulse。
 */
#define TWO_PI_F    (6.2831853f)

static float pulse2rad(int32_t p)
{
    return (float)p * TWO_PI_F / (float)od_2000_ppr;
}

static int32_t rad2pulse(float r)
{
    return (int32_t)(r * (float)od_2000_ppr / TWO_PI_F);
}

static uint8_t cia_cmd(uint16_t cw)
{
    const uint8_t b1 = (uint8_t)((cw >> 1) & 1u);
    const uint8_t b2 = (uint8_t)((cw >> 2) & 1u);
    const uint8_t b3 = (uint8_t)((cw >> 3) & 1u);

    if ((b1 == 0u) && (b2 == 0u)) { return 1u; }   /* Disable voltage */
    if ((b1 == 0u) && (b2 == 1u)) { return 2u; }   /* Quick stop */
    if ((b1 == 1u) && (b2 == 0u)) { return 3u; }   /* Shutdown */
    if (b3 == 0u)                 { return 4u; }   /* Switch on */
    return 5u;                                     /* Enable operation */
}

static void cia_step(uint16_t cw)
{
    const uint8_t fault = (BspServo_GetFault() != 0u) ? 1u : 0u;
    const uint8_t fr    = (uint8_t)(((cw & 0x0080u) != 0u) &&
                                    ((s_cw_last & 0x0080u) == 0u));
    const uint8_t cmd   = cia_cmd(cw);

    s_cw_last = cw;
    if (s_state == ST_NOT_READY) { s_state = ST_SWITCH_ON_DISABLED; }

    if (fault != 0u) {
        if (s_state != ST_FAULT) { s_state = ST_FAULT; (void)ServoCmd_Disarm(); }
        return;
    }
    if (s_state == ST_FAULT) {
        if (fr != 0u) {
            (void)ServoCmd_ClearFault();
            s_state = ST_SWITCH_ON_DISABLED;
        }
        return;
    }

    switch (s_state) {
    case ST_SWITCH_ON_DISABLED:
        if (cmd == 3u) { s_state = ST_READY_TO_SWITCH_ON; }
        else if (cmd == 5u) {                        /* 兼容"一键使能"主站 */
            if (ServoCmd_Arm() != 0u) { s_state = ST_OPERATION_ENABLED; }
        }
        break;
    case ST_READY_TO_SWITCH_ON:
        if (cmd == 4u) { s_state = ST_SWITCHED_ON; }
        else if (cmd == 5u) {
            if (ServoCmd_Arm() != 0u) { s_state = ST_OPERATION_ENABLED; }
        } else if ((cmd == 1u) || (cmd == 2u)) { s_state = ST_SWITCH_ON_DISABLED; }
        break;
    case ST_SWITCHED_ON:
        if (cmd == 5u) {
            if (ServoCmd_Arm() != 0u) { s_state = ST_OPERATION_ENABLED; }
        } else if (cmd == 3u) { s_state = ST_READY_TO_SWITCH_ON; }   /* 手册：0x23 -0x0000-> 0x21 */
        else if ((cmd == 1u) || (cmd == 2u)) { s_state = ST_SWITCH_ON_DISABLED; }
        break;
    case ST_OPERATION_ENABLED:
        /* ★ 每周期刷新 deadman（2026-11-25 实测踩坑）：
         * ServoCmd 内部有 CMD_DEADMAN_MS=2000 的保活，只在状态**迁移**那一下
         * 调 Arm() 的话，2 秒后伺服从内部自动失能，而状态机还以为在 enabled ——
         * 现象是状态字报 0x0227 但电机 armed=False、mode=NONE（状态字撒谎）。
         * ServoCmd_Arm() 是幂等的（已使能时只刷新 deadman），所以每拍调是安全的。 */
        if (ServoCmd_IsArmed() == 0u) {
            /* 伺服侧已经掉了（deadman 超时/被别处失能）→ 状态如实退回 */
            s_state = ST_SWITCHED_ON;
            break;
        }
        (void)ServoCmd_Arm();                  /* 保活 */
        if (cmd == 1u) { (void)ServoCmd_Disarm(); s_state = ST_SWITCH_ON_DISABLED; }
        else if (cmd == 2u) { (void)ServoCmd_Disarm(); s_state = ST_QUICK_STOP; }
        else if (cmd == 3u) { (void)ServoCmd_Disarm(); s_state = ST_READY_TO_SWITCH_ON; }
        break;
    case ST_QUICK_STOP:
        if (cmd == 5u) {                             /* 手册：0x07 -0x000F-> 0x27 */
            if (ServoCmd_Arm() != 0u) { s_state = ST_OPERATION_ENABLED; }
        } else if (cmd == 1u) { s_state = ST_SWITCH_ON_DISABLED; }
        break;
    default:
        s_state = ST_SWITCH_ON_DISABLED;
        break;
    }
}

static uint16_t cia_sw(void)
{
    uint16_t sw;

    switch (s_state) {
    case ST_NOT_READY:          sw = 0x0000u; break;
    case ST_SWITCH_ON_DISABLED: sw = 0x0040u; break;
    case ST_READY_TO_SWITCH_ON: sw = 0x0021u; break;
    case ST_SWITCHED_ON:        sw = 0x0023u; break;
    case ST_OPERATION_ENABLED:  sw = 0x0027u; break;
    case ST_QUICK_STOP:         sw = 0x0007u; break;
    case ST_FAULT:              sw = 0x0008u; break;
    default:                    sw = 0x0000u; break;
    }
    /* bit9 Remote 常置；bit10 Target reached 取位置环 settled；bit12 跟随中 */
    sw |= 0x0200u;
    if ((AppParam_GetStatusWord() & 0x0004u) != 0u) { sw |= 0x0400u; }
    return sw;
}

/* 模式切换：已使能时直接改 6060 会被联锁拒绝（A5），所以先失能再切 */
static void mode_apply(void)
{
    if (od_6060_mode == s_mode_last) { return; }
    if (ServoCmd_IsArmed() != 0u) {
        (void)ServoCmd_Disarm();
        s_state = ST_SWITCHED_ON;
    }
    switch (od_6060_mode) {
    case MODE_PP:  (void)AppParam_SetOpMode(APP_OP_MODE_POS);   break;
    case MODE_PV:  (void)AppParam_SetOpMode(APP_OP_MODE_VEL);   break;
    case MODE_MIT: (void)AppParam_SetOpMode(APP_OP_MODE_MIT);   break;
    default: break;
    }
    s_mode_last = od_6060_mode;
}

static void drive_apply(void)
{
    /* 通信超时检查（在状态机之前）：Operational 且已使能时，PDO 断流超时 → 强制停机。
     * 必须同时清 od_6040_cw，否则下一拍 cia_step 又看到"使能"命令把电机重新拉起来。 */
    if ((s_state == ST_OPERATION_ENABLED) &&
        ((HAL_GetTick() - s_rx_tick) > CANOPEN_RX_TMO_MS)) {
        (void)ServoCmd_Disarm();
        s_state    = ST_SWITCHED_ON;
        od_6040_cw = 0u;
        s_cw_last  = 0u;
    }

    cia_step(od_6040_cw);
    mode_apply();
    od_6041_sw = cia_sw();
    od_6061_mode_disp = od_6060_mode;

    if (s_state != ST_OPERATION_ENABLED) { return; }

    switch (od_6060_mode) {
    case MODE_PP: {
        /* 轮廓参数（2026-11-25）：6081/6083 是**输出端** pulse/s、pulse/s²，
         * 位置环的 vmax/amax 也是输出端口径（POS_VMAX_DFLT=25/9=2.778 rad/s），
         * 所以直接按 2π/ppr 换算即可，不需要再过减速比。
         * 为 0 表示主站没配，保持位置环原限幅不动。 */
        const float ppr = (float)od_2000_ppr;
        if ((od_6081_prof_vel != 0u) && (od_6083_acc != 0u)) {
            const float vmax = (float)od_6081_prof_vel * TWO_PI_F / ppr;
            const float amax = (float)od_6083_acc     * TWO_PI_F / ppr;
            /* jmax 取 amax 的 20 倍（≈ 0.2 s 加到满加速度），S 曲线用不到时也不碍事 */
            PosLoop_SetLimits(vmax, amax, amax * 20.0f);
        }
        /* ★ 只在 607A **变化**时下发一次（2026-11-25 实测踩坑）：
         * 原来每 1 ms 都调一次 SetPosP(607A)，等于不断把目标重置成 607A 的值 ——
         * 后果是串口设的任何目标都会在下一个 CAN 周期被覆盖回 607A，
         * 表现为"串口给目标没反应、位置环像死了一样"。实测就是这么被坑的。
         * CiA402 的 PP 语义本来就是"新设定点（bit4 上升沿）才采纳"，所以按
         * 值变化触发既符合标准又不会互相抢控制权。 */
        static int32_t pp_last;
        static uint8_t pp_init;
        /* 首次也必须下发（2026-11-25 实测踩坑）：
         * 原来"首次只记基准不动作"，但 RPDO 往往在使能之前就把 607A 写成目标了，
         * 于是基准一记就是 2000，之后值不再变化 → **永远不下发**，表现成"PDO 收到了、
         * 状态字也 enabled，但电机纹丝不动"。改成首次也下发。 */
        if ((pp_init == 0u) || (od_607A_tgt_pos != pp_last)) {
            pp_last = od_607A_tgt_pos;
            pp_init = 1u;
            (void)AppParam_SetPosP(pulse2rad(od_607A_tgt_pos));
        }
        break;
    }
    case MODE_PV:
        /* 60FF 是**输出端** pulse/s；速度环是**电机端** rad/s，所以要 × 减速比。
         * 这一步漏乘会让给定慢 9 倍（输出 40 r/min 时电机只给 4.4 r/min）。 */
        (void)AppParam_SetSpeedW((float)od_60FF_tgt_vel * TWO_PI_F
                                 / (float)od_2000_ppr
                                 * (float)ACT_GEAR_RATIO);
        break;
    case MODE_MIT:
        /* 换算约定（与 can_proto.c 的 L1 完全一致，避免两条协议栈行为不同）：
         *   q/dq  输出端 → 电机端：× 减速比
         *   tau   输出端 mN·m → 电机端 N·m：×0.001 ÷ 减速比
         *   kp/kd 已是电机端，直接 /1000 还原 */
        (void)AppParam_SetMitCmd(
            pulse2rad(od_607A_tgt_pos) * (float)ACT_GEAR_RATIO,
            pulse2rad(od_60FF_tgt_vel) * (float)ACT_GEAR_RATIO,
            (float)od_2002_kp / 1000.0f,
            (float)od_2003_kd / 1000.0f,
            (float)od_2004_tau_mnm * 0.001f / (float)ACT_GEAR_RATIO);
        break;
    default:
        break;
    }
}

/* ===================== 故障码映射 + EMCY（CiA 301 §7.2.5） =====================
 *
 * BSP_FAULT_* 是私有 11 位，必须翻译成 CiA 301 标准错误码，否则主站拿到一堆
 * 未知位，只能显示"未知故障"。分类依据 CiA 301 附录 C：
 *   0x2xxx 电流 / 0x3xxx 电压 / 0x4xxx 温度 / 0x5xxx 硬件 / 0x7xxx 外设 / 0x8xxx 运动
 */
static uint16_t fault_to_cia(uint32_t f)
{
    if ((f & (1UL << 0))  != 0u) { return 0x2310u; }  /* 过流 */
    if ((f & (1UL << 6))  != 0u) { return 0x3210u; }  /* 过压 */
    if ((f & (1UL << 7))  != 0u) { return 0x3220u; }  /* 欠压 */
    if ((f & (1UL << 1))  != 0u) { return 0x7300u; }  /* 编码器 */
    if ((f & (1UL << 4))  != 0u) { return 0x7300u; }  /* SPI → 传感器链路 */
    if ((f & (1UL << 10)) != 0u) { return 0x2310u; }  /* 电流采样 */
    if ((f & (1UL << 8))  != 0u) { return 0x8611u; }  /* 跟随误差 */
    if ((f & (1UL << 5))  != 0u) { return 0x8400u; }  /* 超速 */
    if ((f & (1UL << 9))  != 0u) { return 0x3130u; }  /* 缺相 */
    if ((f & (1UL << 2))  != 0u) { return 0x5000u; }  /* ADC 同步 */
    if ((f & (1UL << 3))  != 0u) { return 0x5000u; }  /* BSP 初始化 */
    return 0x1000u;                                    /* 其他 → 一般错误 */
}

static void emcy_send(uint16_t code, const uint8_t *manu)
{
    uint8_t d[8];
    d[0] = (uint8_t)(code & 0xFFu);
    d[1] = (uint8_t)(code >> 8);
    d[2] = od_1001_errreg;
    d[3] = manu ? manu[0] : 0u;
    d[4] = manu ? manu[1] : 0u;
    d[5] = manu ? manu[2] : 0u;
    d[6] = 0u; d[7] = 0u;
    (void)canopen_send(COB_EMCY, d, 8u);
}

/* 每拍：更新 0x1001；故障出现/消失各发一帧 EMCY（边沿触发，不刷屏） */
static void fault_sync(void)
{
    static uint32_t f_last;
    static uint16_t code_last;
    const uint32_t f = BspServo_GetFault();

    od_1001_errreg = (f != 0u) ? 1u : 0u;

    if (f != f_last) {
        const uint16_t code = fault_to_cia(f);
        if (f != 0u) {
            uint8_t manu[3];          /* 厂商字段带原始 BSP 位，便于查私有手册 */
            manu[0] = (uint8_t)(f & 0xFFu);
            manu[1] = (uint8_t)((f >> 8) & 0xFFu);
            manu[2] = (uint8_t)((f >> 16) & 0xFFu);
            emcy_send(code, manu);
        } else if (code_last != 0u) {
            emcy_send(0x0000u, 0);
        }
        code_last = code;
        f_last = f;
    }
}

/* ===================== 主站自测（CANOPEN_MASTER_TEST） =====================
 *
 * 用途：两块板互联时，一块当从站（正常固件）、另一块烧本模式当**最小主站**，
 * 周期发 SDO 读 0x1000，校验应答 —— 这样在没有 USB-CAN 分析仪的情况下也能
 * 验证"物理层 + SDO 协议"整条链。
 *
 * 它只实现"SDO 读一个对象"，不是完整主站：够验证链路，不参与实际控制。
 * 结果通过 30092 kind=11 上报（与 can_monitor.py 兼容）：
 *   v8[0]=3(role) v8[1]=rx_ok v8[2]=fail v8[3]=last_index
 *   v8[4]=uptime_s v8[5]=TEC*1000+REC v8[6]=last_data v8[7]=tx_ok
 */
#if defined(CANOPEN_MASTER_TEST) && (CANOPEN_MASTER_TEST != 0)

/* 全流程验证：NMT Start → CiA402 三连迁移 → 读 TPDO 状态字/位置。
 * 每 300 ms 走一步，同时统计 TPDO 到达次数，用来判断：
 *   · NMT Start 后 TPDO 是否开始发（count 增长）→ 验证 NMT + PDO 上行
 *   · 状态字是否按 0x0040 → 0x0021 → 0x0023 → 0x0027 迁移 → 验证状态机
 *   · RPDO 写入是否真的进了从站 → 状态字变化本身就是证据
 */
static uint16_t s_m_tpdo, s_m_sw, s_m_step, s_m_uptime;
static uint32_t s_m_pos;

void Canopen_MasterPoll1ms(void)
{
    static uint16_t tmo;

    s_m_uptime++;
    if (++tmo < 300u) { return; }          /* 300 ms 一步 */
    tmo = 0u;

    /* 功能测试序列：SDO 配模式 → 三连使能 → RPDO 给目标位置。
     * 目标是**输出端 1 圈**（10000 pulse），到位后 TPDO 的 6064 应接近 10000。 */
    switch (s_m_step) {
    case 0: {                              /* NMT Start */
        uint8_t d[2] = { 0x01u, (uint8_t)CANOPEN_NODE_ID };
        (void)canopen_send(COB_NMT, d, 2u);
        break;
    }
    case 1: {                              /* SDO 写 6060 = 1（PP 模式） */
        uint8_t d[8] = { 0x2Fu, 0x60u, 0x60u, 0x00u, 0x01u, 0u, 0u, 0u };
        (void)canopen_send((uint16_t)(0x600u + CANOPEN_NODE_ID), d, 8u);
        break;
    }
    case 2: case 3: case 4: case 5: {      /* RPDO1：FaultReset → Shutdown → SwitchOn → Enable */
        const uint16_t cw = (s_m_step == 2u) ? 0x0080u      /* Fault reset（上升沿） */
                          : (s_m_step == 3u) ? 0x0006u      /* Shutdown */
                          : (s_m_step == 4u) ? 0x0007u      /* Switch on */
                                             : 0x000Fu;     /* Enable operation */
        uint8_t d[8];
        d[0] = (uint8_t)(cw & 0xFFu); d[1] = (uint8_t)(cw >> 8);
        d[2] = 0u; d[3] = 0u; d[4] = 0u; d[5] = 0u;
        d[6] = 0u; d[7] = 0u;
        (void)canopen_send((uint16_t)(0x200u + CANOPEN_NODE_ID), d, 8u);
        break;
    }
    case 6: {                              /* RPDO1：给目标位置 2000 pulse（72° 输出） */
        const int32_t tg = 2000;
        uint8_t d[8];
        d[0] = 0x1Fu; d[1] = 0x00u;        /* 0x0F + bit4（立即有效）*/
        d[2] = (uint8_t)(tg & 0xFF); d[3] = (uint8_t)((tg >> 8) & 0xFF);
        d[4] = (uint8_t)((tg >> 16) & 0xFF); d[5] = (uint8_t)((tg >> 24) & 0xFF);
        d[6] = 0u; d[7] = 0u;
        (void)canopen_send((uint16_t)(0x200u + CANOPEN_NODE_ID), d, 8u);
        break;
    }
    default: {                             /* 保持：控制字 0x0F（不重复触发新设定点） */
        uint8_t d[8];
        d[0] = 0x0Fu; d[1] = 0x00u;
        d[2] = (uint8_t)(2000 & 0xFF); d[3] = (uint8_t)((2000 >> 8) & 0xFF);
        d[4] = 0u; d[5] = 0u; d[6] = 0u; d[7] = 0u;
        (void)canopen_send((uint16_t)(0x200u + CANOPEN_NODE_ID), d, 8u);
        break;
    }
    }
    /* 每 3 步插一次 SDO 读 0x2005（RPDO 原始载荷快照），用来确认
     * "从站实际收到的字节"是否就是主站发的 —— 这是判断 607A 问题的唯一可信途径。 */
    if ((s_m_step >= 8u) && ((s_m_step % 3u) == 0u)) {
        uint8_t q[8] = { 0x40u, 0x05u, 0x20u, 0x00u, 0u, 0u, 0u, 0u };
        (void)canopen_send((uint16_t)(0x600u + CANOPEN_NODE_ID), q, 8u);
    }
    if (s_m_step < 20u) { s_m_step++; }
}

/* 捕获 TPDO1（0x181+n）：默认映射 = 6041(2B) + 6064(4B) */
static uint16_t s_m_emcy_cnt, s_m_emcy_code;
static uint32_t s_m_sdo_val;
static uint16_t s_m_sdo_cnt;

static uint16_t s_m_last_cob;   /* 最近收到的帧 COB-ID（诊断） */

uint8_t Canopen_MasterOnFrame(uint32_t cob, const uint8_t *d, uint8_t dlc)
{
    s_m_last_cob = (uint16_t)cob;   /* 记录一切帧，用于看主站到底收到了什么 */
    /* EMCY（0x081+n）：8 字节 = 错误码(2B) + 错误寄存器(1B) + 厂商(5B) */
    if (cob == COB_EMCY) {
        if (dlc >= 2u) {
            s_m_emcy_cnt++;
            s_m_emcy_code = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
        }
        return 1u;
    }
    /* SDO 应答（0x581）：记录读回的 0x2005 值 */
    if (cob == (uint32_t)(0x580u + CANOPEN_NODE_ID)) {
        if ((dlc >= 8u) && (d[0] == 0x43u) && (d[1] == 0x05u) && (d[2] == 0x20u)) {
            s_m_sdo_val = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
                        | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
            s_m_sdo_cnt++;
        }
        return 1u;
    }
    if (cob != (uint32_t)(0x180u + CANOPEN_NODE_ID)) { return 0u; }
    if (dlc < 6u) { return 1u; }

    s_m_tpdo++;
    s_m_sw  = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
    s_m_pos = (uint32_t)d[2] | ((uint32_t)d[3] << 8)
            | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24);
    return 1u;
}

void Canopen_MasterGetStats(float *v8)
{
    v8[0] = 6.0f;                              /* role 6 = 功能测试（PP 定位） */
    v8[1] = (float)s_m_tpdo;                   /* 收到的 TPDO 数 */
    v8[2] = (float)s_m_sw;                     /* 状态字 6041 */
    v8[3] = (float)s_m_last_cob;               /* 最近收到的帧 COB-ID（诊断） */
    v8[4] = (float)(s_m_uptime / 1000u);
    v8[5] = (float)(hfdcan1.Instance->ECR & 0xFFFFu);
    v8[6] = (float)(hfdcan1.Instance->PSR & 0xFFFFu);
    v8[7] = (float)s_m_step;                   /* 走到第几步 */
}

uint16_t Canopen_MasterRxOk(void)  { return s_m_tpdo; }
uint16_t Canopen_MasterRxBad(void) { return (s_m_tpdo == 0u) ? 1u : 0u; }

#endif /* CANOPEN_MASTER_TEST */

/* ===================== 对外接口 ===================== */

void Canopen_Init(void)
{
    s_nmt = NMT_BOOTUP;
    s_boot_pending = 1u;
    s_hb_ms = 0u;
    od_6041_sw = 0x0040u;        /* Switch on disabled */
    od_6061_mode_disp = 0;
}

uint8_t Canopen_OnFrame(uint32_t cob, const uint8_t *d, uint8_t dlc)
{
    if (cob == COB_NMT)    { nmt_handle(d, dlc); return 1u; }
    if (cob == COB_SDO_RX) { sdo_handle(d, dlc); return 1u; }

    /* RPDO1：只有 NMT = Operational 才处理（CiA 301：PDO 在预操作态不工作） */
    if (s_nmt == NMT_OPERATIONAL) {
        if (((od_1400_cob & 0x80000000UL) == 0u) &&
            (cob == (uint16_t)(od_1400_cob & 0x7FFu))) {
            /* 记时间戳（通信超时用）+ 存原始载荷快照（诊断用） */
            s_rx_tick = HAL_GetTick();
            od_2005_rpdo_lo = (uint32_t)d[0] | ((uint32_t)d[1] << 8)
                            | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
            od_2006_rpdo_hi = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
                            | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
            pdo_unpack(d, dlc, od_1600_map, od_1600_nmap);
            return 1u;
        }
        if (((od_1401_cob & 0x80000000UL) == 0u) &&
            (cob == (uint16_t)(od_1401_cob & 0x7FFu))) {
            pdo_unpack(d, dlc, od_1601_map, od_1601_nmap);
            return 1u;
        }
    }
    return 0u;
}

void Canopen_Task1ms(void)
{
    /* Boot-up：**只在收到 NMT Start 后**才发（2026-11-25 双板实测的最终方案）。
     *
     * 演进过程记录：
     *  · 原来上电即发 → 两块板同时上电时，先起来的板发 Boot-up 无人 ACK，
     *    TEC 涨满、持续重发占住总线，对端 FDCAN 等不到 11 个隐性位 → Start 失败。
     *    现象："谁先复位谁成功"，另一块 INIT 卡死。
     *  · 改成延后 3 s → 仍会撞（对端可能也在同一时刻发）。
     *  · 最终方案：**从站上电后完全静默**，连心跳默认也是关的（0x1017=0）。
     *    主站先站稳并周期性发 NMT Start，从站收到后才开始说话 —— 启动顺序天然确定。 */
    if ((s_boot_pending != 0u) && (s_nmt != NMT_BOOTUP)) {
        uint8_t b[1];
        b[0] = (uint8_t)NMT_BOOTUP;
        if (canopen_send(COB_HB, b, 1u) != 0u) {
            s_boot_pending = 0u;
            s_nmt = NMT_PREOP;
            s_hb_ms = 0u;
        }
    }

    /* Heartbeat：0x1017 周期（ms），0 = 关闭 */
    if (s_nmt != NMT_BOOTUP) {
        if (++s_hb_ms >= od_1017_hb_ms) {
            uint8_t b[1];
            s_hb_ms = 0u;
            b[0] = (uint8_t)s_nmt;
            (void)canopen_send(COB_HB, b, 1u);
        }
    }

    /* 故障 → 0x1001 + EMCY（必须在状态机之前，状态字才反映最新故障） */
    fault_sync();

    /* P4：状态机 + 模式桥接（内部会更新 od_6041_sw / od_6061_mode_disp） */
    drive_apply();

    /* 反馈注入（2026-11-25）：6064/606C 之前恒为 0 —— 主站读位置永远是 0，
     * 等于闭环没法用。位置环本身是输出端口径，而编码器给的是电机端，
     * 所以两者都要 ÷ 减速比。
     * 用 theta_multi（绝对多圈角）而不是 pos_coord_now()：后者带 s_pos_base
     * 偏移，每次进位置模式会重算，主站看到的位置会跳。 */
    {
        const float ppr  = (float)od_2000_ppr;
        const float gear = (float)ACT_GEAR_RATIO;
        const float th   = BspServo_GetThetaMulti() / gear;    /* 输出端 rad */
        const float w    = BspServo_GetOmegaMech()  / gear;    /* 输出端 rad/s */
        Canopen_SetFeedback((int32_t)(th * ppr / TWO_PI_F),
                            (int32_t)(w  * ppr / TWO_PI_F));
    }

    /* TPDO1：只在 Operational 发（预操作态标准上不传 PDO） */
    if (s_nmt == NMT_OPERATIONAL) { pdo_send_tpdo1(); }
}

uint16_t Canopen_StatusWord(void)   { return od_6041_sw; }
uint8_t  Canopen_NmtState(void)     { return (uint8_t)s_nmt; }
uint16_t Canopen_PulsePerRev(void)  { return (uint16_t)(od_2000_ppr & 0xFFFFu); }
int8_t   Canopen_Mode(void)         { return od_6060_mode; }

void Canopen_SetFeedback(int32_t pos_pulse, int32_t vel_pulse_s)
{
    od_6064_pos = pos_pulse;
    od_606C_vel = vel_pulse_s;
}

#endif /* CANOPEN_ENABLE */
