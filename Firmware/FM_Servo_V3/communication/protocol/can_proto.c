/*
 * can_proto.c —— CAN FD 最小协议层（阶段 0：内环自检）
 * 见 can_proto.h 的说明。
 */
#include "can_proto.h"
#include "fdcan.h"

/* 标准 CANopen 从站（canopen.c）。与下面自定义 L1 协议的 COB-ID 冲突，
 * 两者只能启用一个；标准主站场景下关掉 L1、只留 CANopen。 */
#define CANOPEN_ENABLE   (1)
#include "canopen.h"
#include "telemetry_tx.h"
#include "mavlink_common.h"
#include "bsp_servo.h"
#include "servo_cmd.h"
#include "app_param.h"
#include <string.h>

/* 上电自动进入链路测试模式（2026-11-22）
 *
 * 为什么需要：现场只有一个 STLink，两个节点要轮流接串口。靠 action 21/22
 * 启动状态的话，换 USB 时只要节点掉一次电（XT30 松动、48V 瞬断），
 * s_test_role 就归零，换回来 action 23 什么也读不到 —— 实测踩了两次。
 * 改成上电自动跑，重启后状态自恢复，换线不再影响测试。
 *
 * 0 = 正常固件（默认）；1 = 上电自动 TX；2 = 上电自动 RX。
 * 测试时分别烧 2 和 1，测完改回 0 再出正式固件。
 */
#ifndef CAN_TEST_AUTO_ROLE
#define CAN_TEST_AUTO_ROLE  (0)
#endif

/* 发送分频：每 N 个 1 ms 拍发一帧。
 *
 * 为什么需要：1 kHz 下 TX FIFO（3 深）会被一直填满，"入队成功"和"真上了
 * 总线"完全分不清 —— 实测 tx_ok=1000/s、TEC 在 0~115 之间漂、LEC 却是
 * "无错"，三个数字互相矛盾。降到 1 Hz 时 FIFO 永远是空的，tx_ok 就等于
 * 真正上总线的帧数，TEC 的读法也唯一了：被 ACK → 0，没人 ACK → 每次 +8。
 *
 * 1000=1 kHz  100=100 Hz  10=100 Hz 以下  1000 之外按需改。
 */
#ifndef CAN_TEST_TX_DIV_MS
#define CAN_TEST_TX_DIV_MS (1u)
#endif

/* 内部回环（1=开）：FDCAN1 收到的就是自己发的帧，不经过收发器和线束。
 *
 * 用来把"芯片内部发送通路"和"外部物理层"彻底分开：
 *   回环下 rx 数 ≈ tx 数  → 芯片侧正常，问题在外部（收发器/线束/对端）
 *   回环下 rx 数 << tx 数 → 问题在芯片侧（FIFO/寄存器/位定时）
 * 不依赖对端节点，一个串口就够。
 */
#ifndef CAN_TEST_LOOPBACK
#define CAN_TEST_LOOPBACK (0)
#endif

/* 测试帧是否用 BRS 把数据段切到 5 Mbit/s。
 * 0 = 整帧跑 1 Mbit/s（FD 格式但不加速），用来把"数据段"这个变量隔离出去。 */
#ifndef CAN_TEST_BRS
#define CAN_TEST_BRS (1)
#endif


static uint16_t s_tx;
static uint16_t s_rx;
static uint16_t s_err;
static uint8_t  s_ready;

/* 自检用：DLC 码与实际字节数一一对应 */
static const uint8_t s_dlc_code[6] = {
    FDCAN_DLC_BYTES_8,  FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_24,
    FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_64
};
static const uint8_t s_dlc_bytes[6] = { 8u, 16u, 24u, 32u, 48u, 64u };

static uint8_t can_cfg_filter_start(void)
{
    /* 自检/单节点阶段：全部收进 FIFO0，不做 ID 过滤。
     * L1（双节点）再上"本节点单播 + 广播"两条规则。 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK) {
        return 0u;
    }
    /* Start 每次只试一次：总线不空闲时可能退不出 INIT。这里重试 5 轮。
     * 宁可返回 0 让上层知道"没跑起来"，也不要留一个静默不工作的 FDCAN
     * —— 那种状态下发送计数照涨、总线上却没帧，极易误判成对端坏了。 */
    {
        uint8_t k;
        for (k = 0u; k < 5u; k++) {
            (void)HAL_FDCAN_Start(&hfdcan1);
            if ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0u) {
                return 1u;
            }
        }
    }
    return 0u;
}

static uint8_t can_cfg_filter_start(void);
static void    can_clear_ecr(void);
static void    can_enter_init(void);

void CanProto_Init(void)
{
    s_tx = 0u;
    s_rx = 0u;
    s_err = 0u;

    /* 先解锁受保护寄存器。这一步不能省：实测直接写 CCCR.TEST / TDCR
     * 全部被静默丢弃，症状是"TDC 开了等于没开""回环设了却还在正常模式"。 */
    can_enter_init();

#if (CAN_TEST_LOOPBACK != 0)
    /* 内部回环：CCCR.TEST + TEST.LBCK，并用 MON 切断与真实总线的相互影响。 */
    hfdcan1.Instance->CCCR |= (FDCAN_CCCR_TEST | FDCAN_CCCR_MON);
    hfdcan1.Instance->TEST |= FDCAN_TEST_LBCK;
    /* 回环时 TX 直连 RX、环路延迟≈0：不能按收发器延迟做补偿，
     * 否则采样点被人为推后，数据段必然 CRC 错（实测 rx_ok=0、LEC=CRC）。 */
    (void)HAL_FDCAN_DisableTxDelayCompensation(&hfdcan1);
#else
    /* 5 Mbit/s 数据段必须开 TDC（发送延迟补偿）。
     *
     * ISO 11898-1 要求数据段 >1 Mbit/s 时用次级采样点(SSP)采样自己的位。
     * 不补偿的话发送端按标称段采样点判自己的数据段：85 MHz 下 1 mtq
     * ≈ 11.8 ns，收发器环路延迟就有十几到二十几个 mtq，数据段必然判错
     * —— 现象正是"帧入了 FIFO 却上不了总线、LEC 报 CRC"。
     * TDCO 用实测的 PSR.TDCV = 22 mtq；TDCF=0 表示固定 SSP、不用过滤窗口。
     */
    (void)HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 22u, 0u);
    (void)HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
#endif

    can_clear_ecr();        /* 上电可能带着上次复位前的错误计数 */
    s_ready = can_cfg_filter_start();   /* 内部会清 INIT 并启动，失败重试 5 轮 */

#if (PROD_BUILD != 0)
    CanProto_L1Start(0u);      /* 生产：FDCAN 起来后直接跑 L1，等主站发 CMD_AGG */
#else
#if (CAN_TEST_AUTO_ROLE != 0)
    /* 测试固件：上电即进入角色，重启/换线后自动恢复 */
    (void)CanProto_TestStart((uint8_t)CAN_TEST_AUTO_ROLE);
#endif
#endif

    Canopen_Init();          /* 清对象字典状态，Boot-up 由 Task1ms 第一拍发出 */
}

void CanProto_Stop(void)
{
    (void)HAL_FDCAN_Stop(&hfdcan1);
    s_ready = 0u;
}

uint8_t CanProto_IsReady(void) { return s_ready; }

/* 发一帧、轮询收一帧，比对 ID / DLC / 首字节 */
#if (PROD_BUILD == 0)
static uint8_t can_send_recv(uint8_t dlc_code, uint8_t nbytes, uint8_t pat)
{
    FDCAN_TxHeaderTypeDef tx;
    FDCAN_RxHeaderTypeDef rx;
    uint8_t  buf[64];
    uint8_t  rd[64];
    uint32_t t0;

    memset(buf, pat, sizeof(buf));

    tx.Identifier          = 0x201u;              /* 与 FB_MOTION 同段，便于看波形 */
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = dlc_code;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_ON;        /* 关键：数据段切 5 M */
    tx.FDFormat            = FDCAN_FD_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0u;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, buf) != HAL_OK) {
        s_err++;
        return 0u;
    }
    s_tx++;

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 5u) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0u) {
            continue;
        }
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, rd) != HAL_OK) {
            s_err++;
            return 0u;
        }
        s_rx++;

        if ((rx.Identifier != 0x201u) ||
            (rx.DataLength != dlc_code) ||
            (rd[0] != pat) ||
            (rd[nbytes - 1u] != pat)) {
            s_err++;
            return 0u;
        }
        return 1u;
    }

    s_err++;                    /* 超时：没有回环回来 */
    return 0u;
}

uint8_t CanProto_SelfTest(void)
{
    FDCAN_InitTypeDef save;
    FDCAN_ErrorCountersTypeDef ec;
    float v8[8];
    uint8_t i, pass = 1u, mask = 0u;
    uint16_t tx0 = s_tx, rx0 = s_rx, er0 = s_err;
    uint8_t  tec = 0u, rec = 0u;

    save = hfdcan1.Init;        /* 保存正常模式参数，结束后原样恢复 */

    /* ---- 切内部回环 ---- */
    (void)HAL_FDCAN_Stop(&hfdcan1);
    (void)HAL_FDCAN_DeInit(&hfdcan1);

    hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        hfdcan1.Init = save;    /* Init 失败也要把参数恢复，避免下次正常模式带着 loopback */
        s_ready = 0u;
        memset(v8, 0, sizeof(v8));
        v8[1] = 1.0f;           /* v8[1] = init_fail */
        v8[4] = 1000000.0f;
        v8[5] = 5000000.0f;
        Telemetry_PushIdentResult(MAV_ID_CAN, 0u, 0u, v8, 0.0f);
        return 0u;
    }
    if (can_cfg_filter_start() == 0u) {
        pass = 0u;
    }

    /* ---- 六种 DLC 各来一帧 ---- */
    if (pass != 0u) {
        for (i = 0u; i < 6u; i++) {
            if (can_send_recv(s_dlc_code[i], s_dlc_bytes[i],
                              (uint8_t)(0xA0u + i)) != 0u) {
                mask |= (uint8_t)(1u << i);
            } else {
                pass = 0u;
            }
        }
    }

    (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
    tec = (uint8_t)ec.TxErrorCnt;
    rec = (uint8_t)ec.RxErrorCnt;

    /* ---- 恢复正常模式 ---- */
    (void)HAL_FDCAN_Stop(&hfdcan1);
    (void)HAL_FDCAN_DeInit(&hfdcan1);
    hfdcan1.Init = save;
    if (HAL_FDCAN_Init(&hfdcan1) == HAL_OK) {
        s_ready = can_cfg_filter_start();
    } else {
        s_ready = 0u;
        pass = 0u;
    }

    /* ---- 上报 30092 kind=MAV_ID_CAN ----
     * v8[0]=本次 tx, v8[1]=本次 rx, v8[2]=本次 err,
     * v8[3]=TEC, v8[4]=标称波特率, v8[5]=数据波特率,
     * v8[6]=通过的 DLC 掩码, v8[7]=REC */
    v8[0] = (float)(s_tx - tx0);
    v8[1] = (float)(s_rx - rx0);
    v8[2] = (float)(s_err - er0);
    v8[3] = (float)tec;
    v8[4] = 1000000.0f;
    v8[5] = 5000000.0f;
    v8[6] = (float)mask;
    v8[7] = (float)rec;

    Telemetry_PushIdentResult(MAV_ID_CAN, pass, (uint16_t)(s_rx - rx0), v8, 0.0f);
    return pass;
}
#endif /* PROD_BUILD */

void CanProto_GetStats(uint16_t *tx, uint16_t *rx, uint16_t *err,
                       uint8_t *tec, uint8_t *rec, uint8_t *state)
{
    FDCAN_ErrorCountersTypeDef ec;

    if (tx)    { *tx = s_tx; }
    if (rx)    { *rx = s_rx; }
    if (err)   { *err = s_err; }
    if (tec || rec) {
        memset(&ec, 0, sizeof(ec));
        (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
        if (tec) { *tec = (uint8_t)ec.TxErrorCnt; }
        if (rec) { *rec = (uint8_t)ec.RxErrorCnt; }
    }
    if (state) {
        *state = (uint8_t)((HAL_FDCAN_GetState(&hfdcan1) == HAL_FDCAN_STATE_BUSY) ? 1u : 0u);
    }
}

/* ================= 双节点链路测试（阶段 1） =================
 *
 * 为什么用"持续发送 + 手动停"而不是"发 2000 帧自动停"：
 * 现场只有一个 STLink，两个节点要轮流接串口。固定帧数的话，
 * 换 COM 的两分钟里测试早就结束、结果也丢了。改成持续跑，
 * 任意时刻用 action=23 停并取结果。
 *
 * seq 用 u16，比较必须用无符号差 (uint16_t)(now - last)：
 * 回绕时 0xFFFF→0x0000 的差是 1，直接比大小会误判成丢 65535 帧。
 */
#define CAN_TEST_ID        (0x100u)      /* CMD_AGG 段，正式协议也用这个 ID */
#define CAN_ECHO_ID        (0x101u)      /* ping-pong 回显通道 */
#define CAN_TEST_DLC       (FDCAN_DLC_BYTES_24)
#define CAN_TEST_BYTES     (24u)
#define CAN_TEST_TMO_MS    (600000u)




static volatile uint8_t  s_test_role;     /* 0idle 1tx 2rx 3ping 4echo 5loopback 6l1 7l1-passive */
static volatile uint32_t s_tx_ok;
static volatile uint32_t s_tx_fail;
static volatile uint32_t s_rx_ok;
static volatile uint32_t s_rx_lost;
static volatile uint32_t s_rx_dup;
static volatile uint32_t s_rx_resync;   /* seq 大跳变：对端重启/断流，不计丢帧 */
static volatile uint32_t s_rx_zero;     /* 收到 seq==0 的次数：对端重启证据 */
static uint16_t s_cmd_age;              /* L1：距上一条有效 CMD_AGG 的 ms 数 */
static uint8_t  s_cmd_to;               /* L1：10 ms 无指令已置超时 */
static uint8_t  s_cmd_ever;             /* L1：是否收到过 CMD_AGG（主站是否曾上线） */
static uint8_t s_rc_div;                /* bus-off 恢复检查分频（10 拍 = 10 ms） */
static volatile uint32_t s_rc_cnt;      /* bus-off 自动恢复次数（诊断） */
static uint16_t s_rep_div;              /* 周期上报分频：1000 拍 = 1 s */
static uint16_t s_tx_div;               /* 发送分频，见 CAN_TEST_TX_DIV_MS */
static uint16_t s_test_seq;
static uint16_t s_rx_last;
static uint8_t  s_rx_first;
static uint32_t s_test_t0;

/* 显式进入 INIT + CCE，解锁受保护寄存器（TEST/MON/TDCR/DBTP）。
 *
 * 教训：之前假设"HAL_FDCAN_Init() 结束时会停在 INIT+CCE"，于是直接写
 * CCCR.TEST / TDCR，结果**全部被静默丢弃**（CCCR 读回来 TEST/MON 都是 0），
 * 表现为"TDC 开了等于没开""回环设了还在正常模式"。写保护位必须先确认
 * INIT=1 且 CCE=1，不能靠推断。
 */
static void can_enter_init(void)
{
    uint32_t guard;

    hfdcan1.Instance->CCCR |= FDCAN_CCCR_INIT;
    guard = 0u;
    while (((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0u) && (++guard < 100000u)) { }
    hfdcan1.Instance->CCCR |= FDCAN_CCCR_CCE;
}

/* 清零 TEC/REC（必须在 INIT+CCE 窗口内调用）。 */
static void can_clear_ecr(void)
{
    hfdcan1.Instance->ECR = 0u;
}

/* 读 PSR（协议状态）并打包成一个 float。
 *
 * 关键坑：LEC 是"读后清零"的，而且进入 Bus-off / INIT 也会被重置成 7。
 * 最初每秒读一次，结果 TEC 明明涨到 116、LEC 却永远是"无错" —— 因为
 * 一轮错误(33 ms)外加 Bus-off 早把 LEC 冲掉了。改成每 1 ms 采样一次并
 * 锁存最后一个非 7 的 LEC，同时数 Bus-off 上升沿，才有诊断价值。
 *
 * LEC 含义：1 位填充 2 格式 3 ACK 4 位错误 5 CRC 6 显性序列 7 无错
 */
static volatile uint8_t s_lec_sticky;     /* 最近一次非"无错"的 LEC */
static volatile uint8_t s_bo_cnt;         /* Bus-off 次数（上升沿计数） */
static uint8_t          s_bo_prev;

static void can_diag_poll(void)
{
    uint32_t psr = hfdcan1.Instance->PSR;   /* 读一次就清 LEC，1 ms 粒度足够 */
    uint32_t lec = (psr & FDCAN_PSR_LEC_Msk) >> FDCAN_PSR_LEC_Pos;
    uint8_t  bo  = ((psr & FDCAN_PSR_BO_Msk) != 0u) ? 1u : 0u;

    if (lec != 7u) { s_lec_sticky = (uint8_t)lec; }
    if ((bo != 0u) && (s_bo_prev == 0u) && (s_bo_cnt < 255u)) { s_bo_cnt++; }
    s_bo_prev = bo;
}

#if (PROD_BUILD == 0)
static void can_push_test_result(uint8_t role)
{
    FDCAN_ErrorCountersTypeDef ec;
    float v8[8];
    uint8_t ok;
    uint16_t n;

    memset(&ec, 0, sizeof(ec));
    (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

    v8[0] = (float)role;
    v8[1] = (float)s_tx_ok;
    v8[2] = (float)s_tx_fail;
    v8[3] = (float)s_rx_ok;
    v8[4] = (float)s_rx_lost;
    v8[5] = (float)(s_rx_dup + s_rx_resync);
    v8[6] = (float)ec.TxErrorCnt;
    v8[7] = (float)ec.RxErrorCnt;

    if (role == 1u) {
        /* 发送侧：没有入队失败、且 TEC 没涨，说明每一帧都被至少一个节点 ACK 了 */
        ok = (uint8_t)((s_tx_fail == 0u) && (ec.TxErrorCnt == 0u)) ? 1u : 0u;
        n  = (uint16_t)((s_tx_ok > 0xFFFFu) ? 0xFFFFu : s_tx_ok);
    } else {
        /* 接收侧：收到过帧、无丢帧、无重复 */
        ok = (uint8_t)((s_rx_ok > 0u) && (s_rx_lost == 0u) && (s_rx_dup == 0u)) ? 1u : 0u;
        n  = (uint16_t)((s_rx_ok > 0xFFFFu) ? 0xFFFFu : s_rx_ok);
    }

#if defined(CANOPEN_MASTER_TEST) && (CANOPEN_MASTER_TEST != 0)
    /* 主站自测：用 kind=11 上报 SDO 统计（can_monitor.py 可读） */
    Canopen_MasterGetStats(v8);
    ok = (uint8_t)((Canopen_MasterRxOk() > 0u) && (Canopen_MasterRxBad() == 0u)) ? 1u : 0u;
    n  = Canopen_MasterRxOk();
#endif
    Telemetry_PushIdentResult(MAV_ID_CAN_TEST, ok, n, v8, 0.0f);
}

/* 寄存器快照上报（kind=12，1 Hz）。
 *
 * 用来回答一个用计数器绕不出来的问题：`s_tx_ok` 每秒加 1000、`tx_fail` 却是 0、
 * TEC 也是 0，可对端每秒只收到 42 帧 —— 帧到底卡在 FIFO 里，还是根本没进 FIFO？
 *   TXBC   TX 缓冲区配置（NTDB/TFQS/TFQM），配置错了 FIFO 深度就不是 3
 *   TXFQS  bit[2:0]=空闲槽数，bit21=TFQF 满标志
 *   TXBRP  挂起请求位图：某位一直置 1 说明那一槽发不出去
 *   TXBTO  发送完成位图
 */
static void can_push_regs(uint8_t role)
{
    float v8[8];

    v8[0] = (float)(hfdcan1.Instance->TXBC  & 0xFFFFFFFFu);
    v8[1] = (float)(hfdcan1.Instance->TXFQS & 0xFFFFFFFFu);
    v8[2] = (float)(hfdcan1.Instance->TXBRP & 0xFFFFFFFFu);
    v8[3] = (float)(hfdcan1.Instance->TXBTO & 0xFFFFFFFFu);
    v8[4] = (float)(hfdcan1.Instance->PSR   & 0xFFFFFFFFu);
    v8[5] = (float)(hfdcan1.Instance->ECR   & 0xFFFFFFFFu);
    v8[6] = (float)(hfdcan1.Instance->CCCR  & 0xFFFFFFFFu);
    v8[7] = (float)s_rc_cnt;       /* 自动恢复次数 */

    Telemetry_PushIdentResult(12u, 1u, 0u, v8, 0.0f);   /* kind=12，与周期上报 11 区分 */
}

/* 周期上报（1 s）。字段口径（2026-11-22 第 2 版）：
 *   v8[0] role       1=txtest 2=rxtest 3=tx+rx
 *   v8[1] rx_ok      低 16 位（累积）
 *   v8[2] rx_lost    小跳变丢帧（真丢）
 *   v8[3] dup+resync 重复 + 大跳变失步
 *   v8[4] uptime_s   连续增长 = 节点没重启
 *   v8[5] TEC*1000+REC
 *   v8[6] PSR 包     LEC(锁存) + Bus-off 次数*16
 *   v8[7] tx_ok      低 16 位（入队成功次数，不等于上总线成功）
 *   resid  tx_fail   入队失败次数（TX FIFO 满 = 总线来不及发）
 */
static void can_push_periodic(uint8_t role)
{
    FDCAN_ErrorCountersTypeDef ec;
    float v8[8];

    memset(&ec, 0, sizeof(ec));
    (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

    v8[0] = (float)role;
    v8[1] = (float)(s_rx_ok & 0xFFFFu);
    v8[2] = (float)s_rx_lost;
    v8[3] = (float)(s_rx_dup + s_rx_resync);
    v8[4] = (float)(HAL_GetTick() / 1000u);
    v8[5] = (float)((uint32_t)ec.TxErrorCnt * 1000u + (uint32_t)ec.RxErrorCnt);
    v8[6] = (float)(((uint32_t)s_lec_sticky + ((uint32_t)s_bo_cnt << 4)) +
                    ((uint32_t)s_rx_zero * 1000u));
    v8[7] = (float)(s_tx_ok & 0xFFFFu);

    if (role == 7u) {
        /* 这一档没有有效 CMD_AGG，rx_ok 恒 0，所以借用三个字段做超时观测：
         *   v8[1] = s_cmd_age(ms)   v8[2] = s_cmd_to(0/1)   v8[7] = armed */
        v8[1] = (float)s_cmd_age;
        v8[2] = (float)s_cmd_to;
        v8[7] = (float)((ServoCmd_IsArmed() != 0u) ? 1u : 0u);
    }

    Telemetry_PushIdentResult(MAV_ID_CAN_TEST, 1u,
                              (uint16_t)(s_rx_ok & 0xFFFFu), v8,
                              (float)s_tx_fail);
}

uint8_t CanProto_TestStart(uint8_t role)
{
    if ((role < 1u) || (role > 7u)) { return 0u; }
    if (s_ready == 0u) {
        CanProto_Init();
        if (s_ready == 0u) { return 0u; }
    }

    s_tx_ok = 0u;
    s_tx_fail = 0u;
    s_rx_ok = 0u;
    s_rx_lost = 0u;
    s_rx_dup = 0u;
    s_rx_resync = 0u;
    s_rx_zero = 0u;
    s_rep_div = 0u;
    s_test_seq = 0u;
    s_rx_last = 0u;
    s_rx_first = 0u;
    s_lec_sticky = 7u;      /* 7 = 无错（PSR 的"未更新"码），便于区分"真的没错" */
    s_bo_cnt = 0u;
    s_bo_prev = 0u;
    s_tx_div = 0u;
    s_test_t0 = HAL_GetTick();
    s_test_role = role;

    if (role == 6u) {
        CanProto_L1Start(1u);      /* 协议层 + 自环验证 */
    } else if (role == 7u) {
        CanProto_L1Start(0u);      /* 协议层被动：用于实测 10 ms 失效安全 */
    }
    return 1u;
}

void CanProto_TestStop(void)
{
    uint8_t role = s_test_role;

    if (role == 0u) { return; }
    s_test_role = 0u;
    CanProto_L1Stop();

    can_push_test_result(role);

    /* 只撤销挂起的发送请求，不再做 Stop/DeInit/Init + 清 ECR。
     *
     * 原来那套有致命副作用：退出 INIT 要求总线上出现 11 个连续隐性位，
     * 若此刻对端正连续发错误帧，守卫超时后 FDCAN 会**停在 INIT 里**，
     * 之后既不应答也不收帧 —— 现场表现就是"对端板子像坏了一样"。
     * 600 s 测试窗口一到就会自动触发，实测把一次正常的双节点测试
     * 变成了"对端消失"。TEC 不清也无所谓：每成功发一帧它自己会 −1。
     */
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, 0x3FFFFFFFU);
}

static void can_test_tx_step(void)
{
    FDCAN_TxHeaderTypeDef tx;
    uint8_t buf[CAN_TEST_BYTES];

    memset(buf, 0, sizeof(buf));
    buf[0] = (uint8_t)(s_test_seq & 0xFFu);
    buf[1] = (uint8_t)(s_test_seq >> 8);
    buf[2] = 0x5Au;                       /* 固定图样，便于抓包认帧 */
    buf[CAN_TEST_BYTES - 1u] = 0xA5u;

    tx.Identifier          = CAN_TEST_ID;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = CAN_TEST_DLC;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if (CAN_TEST_BRS != 0)
    tx.BitRateSwitch       = FDCAN_BRS_ON;
#else
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
#endif
    tx.FDFormat            = FDCAN_FD_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0u;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, buf) == HAL_OK) {
        s_tx_ok++;
    } else {
        s_tx_fail++;
    }
    s_test_seq++;
}

/* bus-off 自动恢复。
 *
 * STM32 的 FDCAN 进入 bus-off 后，**硬件会把 CCCR.INIT 置 1 并停在那里**；
 * 必须由软件把 INIT 清 0，芯片才开始"128 × 11 个连续隐性位"的恢复流程。
 * 不处理的话节点一旦 bus-off 就永久退出总线 —— 之后的现场表现是：
 * TXBRP 的请求全挂着、TEC 冻在 248、发送计数还在涨但总线上没有帧，
 * 两块板互相看着都像"板子坏了"。烧录、换线、对端暂时掉线都会触发。
 *
 * 这里不判断 PSR.BO：INIT 只可能是硬件在 bus-off 时置的（本工程启停之后
 * 不再写 INIT），所以看到 INIT=1 就直接清，硬件自己会走恢复。
 */
#endif /* PROD_BUILD */

static void can_busoff_recover(void)
{
    if ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) != 0u) {
        hfdcan1.Instance->CCCR &= ~FDCAN_CCCR_INIT;
        s_rc_cnt++;
    }
}

/* 回显帧：ECHO 角色收到 PING 后立刻在同一节拍回一帧，ID 换成 CAN_ECHO_ID。
 *
 * 为什么要做 ping-pong：只靠"发送侧统计"没法区分"帧没上总线"和"对端没收下"，
 * 而 TEC 的变化又要靠推断。带回显后，收敛到一句话：
 *   回显数 ≈ 发送数  → 来回都通；
 *   回显数 << 发送数  → 中途丢了，再看 TEC/LEC 判丢在哪一段。
 * 而且任意一侧的数字都能自证，不用再猜串口接的是哪块板。
 */
#if (PROD_BUILD == 0)
static void can_echo_send(uint16_t seq)
{
    FDCAN_TxHeaderTypeDef tx;
    uint8_t buf[CAN_TEST_BYTES];

    memset(buf, 0, sizeof(buf));
    buf[0] = (uint8_t)(seq & 0xFFu);
    buf[1] = (uint8_t)(seq >> 8);
    buf[2] = 0xE5u;                       /* 回显标记，抓包时一眼认出 */

    tx.Identifier          = CAN_ECHO_ID;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = CAN_TEST_DLC;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if (CAN_TEST_BRS != 0)
    tx.BitRateSwitch       = FDCAN_BRS_ON;
#else
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
#endif
    tx.FDFormat            = FDCAN_FD_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0u;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, buf) == HAL_OK) {
        s_tx_ok++;
    } else {
        s_tx_fail++;
    }
}

/* want_id：要接收的 ID；do_echo=1 时每收到一帧立刻回显 */
static void can_test_rx_step(uint16_t want_id, uint8_t do_echo)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u) {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t rd[64];
        uint16_t seq, d;

        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, rd) != HAL_OK) {
            break;
        }
        if ((rx.Identifier != want_id) || (rx.DataLength != CAN_TEST_DLC)) {
            continue;
        }
        seq = (uint16_t)(rd[0] | ((uint16_t)rd[1] << 8));

        if (do_echo != 0u) {
            can_echo_send(seq);           /* 先回显，再统计，避免占用延迟 */
        }

        /* 对端是否在重启：只有 TestStart 才会把 seq 清零，正常运行永远不回 0。
         * 收到一次 0 就说明对端刚重置过（看门狗/掉电/复位），这比"失步"
         * 更能区分"对端丢帧"和"对端重启"。 */
        if (seq == 0u) { s_rx_zero++; }

        if (s_rx_first == 0u) {
            s_rx_first = 1u;
            s_rx_last  = seq;
            s_rx_ok    = 1u;
            continue;
        }

        d = (uint16_t)(seq - s_rx_last);   /* 无符号差，处理 u16 回绕 */
        if (d == 0u) {
            s_rx_dup++;
        } else if (d == 1u) {
            s_rx_ok++;
        } else if (d <= 100u) {
            /* 100 ms 以内的跳变才算真丢帧（FIFO 溢出/总线错误）。
             * 超过 100 的一律判为失步：对端重启、seq 归零、长时间断流都会
             * 让 u16 差变成 65535 这种大数，直接累加会得出"丢了 425 万帧"
             * 这种荒唐结果（实测踩过）。失步只重设基准，不计丢帧。 */
            s_rx_lost += (uint32_t)(d - 1u);
            s_rx_ok++;
        } else {
            s_rx_resync++;
        }
        s_rx_last = seq;
    }
}

/* 发送节拍：每 CAN_TEST_TX_DIV_MS 个 1 ms 拍放一帧出去 */
static uint8_t can_tx_due(void)
{
#if (CAN_TEST_TX_DIV_MS <= 1)
    return 1u;
#else
    if (++s_tx_div >= (uint16_t)CAN_TEST_TX_DIV_MS) {
        s_tx_div = 0u;
        return 1u;
    }
    return 0u;
#endif
}


#endif /* PROD_BUILD */

/* ===================== L1 正式协议层 =====================
 * 对齐 Agnet/X72关节模组_CANFD协议_工程落地_v1.1.html §4
 *
 * ID（11 位）：
 *   EMCY        0x000 / 0x000+n   双向   16 B  事件
 *   SYNC        0x080            主→广播 16 B  1 ms
 *   CMD_AGG     0x100            主→广播 64 B  1 ms
 *   CMD_MOTION  0x180+n          主→从   32 B  按需
 *   FB_MOTION   0x200+n          从→主   24 B  1 ms
 *   FB_FAST     0x280+n          从→主   16 B  可选
 *   FB_DIAG     0x300+n          从→主   48 B  20 ms
 *   HEARTBEAT   0x380+n          从→主    8 B  100 ms
 */
#define CAN_ID_EMCY        (0x000u)
#define CAN_ID_SYNC        (0x080u)
#define CAN_ID_CMD_AGG     (0x100u)
#define CAN_ID_CMD_MOTION  (0x180u)
#define CAN_ID_FB_MOTION   (0x200u)
#define CAN_ID_FB_FAST     (0x280u)
#define CAN_ID_FB_DIAG     (0x300u)
#define CAN_ID_HB         (0x380u)

/* 节点号/槽位：暂时用编译宏，后续搬到 app_param 由产线写入（见文档 §4 未决项） */
#ifndef CAN_NODE_ID
#define CAN_NODE_ID        (1u)
#endif
#ifndef CAN_SLOT_ID
#define CAN_SLOT_ID        (0u)
#endif

#define CAN_AGG_TMO_MS     (10u)    /* 10 个 1 ms 周期无指令 → 失效安全 */
#define CAN_DIAG_DIV_MS    (20u)
#define CAN_HB_DIV_MS      (100u)
#define CAN_KT_NM_PER_A    (0.0910f)

/* 减速比：**临时用编译常量**。ParamBlob 已经 64 B 全满（最后一个 float 被 psi_t_ref
 * 占掉），加一项必须升 PS_VERSION，会把现场已标定的零点/齿槽表全部作废，代价太大。
 * 文档把它列为 P2（配合 rated_torque 一起进参数区），届时按机型写 Flash。
 * 在此之前：总线上的一切角度/转速都是**输出侧**，内部乘除用这个常数。
 *
 * ⚠ 2026-11-25 修正 10 → 9：原注释写"本机型减速比 = 10（用户 2026-11-22 确认）"，
 * 但减速器规格书明确是 **9:1**（额定 6 N·m/360 r/min 输出端那一栏），
 * 且固件位置环用的 `ACT_GEAR_RATIO` 也是 9（app_param.h:52）。
 * 两处不一致会让 CAN 侧的目标位置/力矩**偏 11%**，且诊断上很难看出来
 * （电机端读数永远正常，只有输出端折算错）。
 * 保持一致：CAN_GEAR_RATIO 必须恒等于 ACT_GEAR_RATIO。 */
#ifndef CAN_GEAR_RATIO
#define CAN_GEAR_RATIO     (9.0f)
#endif

/* 编译期检查：万一以后有人只改一处，这里会直接报错而不是静默偏 11%。
 * 用 (int) 转换比较 —— 浮点比较放进数组维度/静态断言会被当成 VLA 扩展，
 * 转成整数常量后才是合法的整型常量表达式。 */
_Static_assert((int)CAN_GEAR_RATIO == (int)ACT_GEAR_RATIO,
               "CAN_GEAR_RATIO must equal ACT_GEAR_RATIO");

#define CAN_DAMP_KD        (0.50f)   /* 超时后第一级阻尼的 kd（N·m·s/rad，电机侧） */
#define CAN_DISARM_MS      (50u)     /* 阻尼后再过 50 ms 仍无指令 → 断使能 */

/* 小端手工读写，不依赖结构体对齐 */
static void can_wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void can_wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t can_rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t can_rd_u32(const uint8_t *p)
{
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void can_wr_f32(uint8_t *p, float f)
{
    uint32_t v;
    memcpy(&v, &f, 4);
    can_wr_u32(p, v);
}

static float can_rd_f32(const uint8_t *p)
{
    uint32_t v = can_rd_u32(p);
    float f;
    memcpy(&f, &v, 4);
    return f;
}

/* CMD_AGG 0x100 / 64 B：0..1 cycle_cnt, 2 flags, 3 n_slots, 4+12i(q,dq,tau) */
typedef struct {
    uint16_t cycle_cnt;
    uint8_t  flags;         /* bit0 同步采样 bit1 广播使能 bit2 快停 */
    uint8_t  n_slots;
    float    q_des[5];
    float    dq_des[5];
    float    tau_ff[5];
} CanAggCmd;

/* FB_MOTION 0x200+n / 24 B */
typedef struct {
    uint16_t statusword;
    uint8_t  mode_act;
    uint8_t  err_code;
    float    q_act;
    float    dq_act;
    float    tau_act;
    uint16_t temp_motor;    /* 0.1 °C */
    uint16_t vbus;          /* 0.01 V */
    uint8_t  seq_echo;
    uint8_t  flags;
    uint16_t follow_err;    /* 0.1 mrad */
} CanFbMotion;

static void can_agg_pack(uint8_t *p, const CanAggCmd *c)
{
    uint8_t i;

    memset(p, 0, 64);
    can_wr_u16(p, c->cycle_cnt);
    p[2] = c->flags;
    p[3] = c->n_slots;
    for (i = 0u; i < 5u; i++) {
        can_wr_f32(p + 4u + 12u * i,       c->q_des[i]);
        can_wr_f32(p + 4u + 12u * i + 4u,  c->dq_des[i]);
        can_wr_f32(p + 4u + 12u * i + 8u,  c->tau_ff[i]);
    }
}

static uint8_t can_agg_unpack(const uint8_t *p, CanAggCmd *c)
{
    uint8_t i;

    c->cycle_cnt = can_rd_u16(p);
    c->flags     = p[2];
    c->n_slots   = p[3];
    if ((c->n_slots < 1u) || (c->n_slots > 5u)) { return 0u; }
    for (i = 0u; i < 5u; i++) {
        c->q_des[i]  = can_rd_f32(p + 4u + 12u * i);
        c->dq_des[i] = can_rd_f32(p + 4u + 12u * i + 4u);
        c->tau_ff[i] = can_rd_f32(p + 4u + 12u * i + 8u);
    }
    return 1u;
}

static void can_fbm_pack(uint8_t *p, const CanFbMotion *f)
{
    memset(p, 0, 24);
    can_wr_u16(p + 0,  f->statusword);
    p[2] = f->mode_act;
    p[3] = f->err_code;
    can_wr_f32(p + 4,  f->q_act);
    can_wr_f32(p + 8,  f->dq_act);
    can_wr_f32(p + 12, f->tau_act);
    can_wr_u16(p + 16, f->temp_motor);
    can_wr_u16(p + 18, f->vbus);
    p[20] = f->seq_echo;
    p[21] = f->flags;
    can_wr_u16(p + 22, f->follow_err);
}

static void can_fbm_unpack(const uint8_t *p, CanFbMotion *f)
{
    f->statusword = can_rd_u16(p + 0);
    f->mode_act   = p[2];
    f->err_code   = p[3];
    f->q_act      = can_rd_f32(p + 4);
    f->dq_act     = can_rd_f32(p + 8);
    f->tau_act    = can_rd_f32(p + 12);
    f->temp_motor = can_rd_u16(p + 16);
    f->vbus       = can_rd_u16(p + 18);
    f->seq_echo   = p[20];
    f->flags      = p[21];
    f->follow_err = can_rd_u16(p + 22);
}

/* 组帧自检：打包一个已知结构再解回来，逐字节比对。
 * 结果通过 s_rx_zero 上报（L1 模式下监控的 zc 列 = 1 表示通过）。 */
static uint8_t can_fbm_selfcheck(void)
{
    CanFbMotion a;
    CanFbMotion b;
    uint8_t buf[24];

    memset(&a, 0, sizeof(a));
    a.statusword = 0x1234u;
    a.mode_act   = 12u;
    a.err_code   = 3u;
    a.q_act      = 1.5f;
    a.dq_act     = -2.25f;
    a.tau_act    = 0.75f;
    a.temp_motor = 456u;
    a.vbus       = 4800u;
    a.seq_echo   = 0x5Au;
    a.flags      = 0xA5u;
    a.follow_err = 1234u;

    can_fbm_pack(buf, &a);
    memset(&b, 0, sizeof(b));
    can_fbm_unpack(buf, &b);

    return (uint8_t)((memcmp(&a, &b, sizeof(a)) == 0) ? 1u : 0u);
}

/* ---- L1 运行态 ---- */
static uint8_t  s_l1_on;
static uint8_t  s_l1_st;            /* 1 = role 6 协议自环验证 */
static uint16_t s_l1_cyc;
static uint16_t s_diag_div;
static uint16_t s_hb_div;
static CanAggCmd s_agg;
static uint32_t s_sync_cnt;

static uint8_t can_l1_send(uint16_t id, const uint8_t *buf, uint32_t dlc)
{
    FDCAN_TxHeaderTypeDef tx;

    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = dlc;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if (CAN_TEST_BRS != 0)
    tx.BitRateSwitch       = FDCAN_BRS_ON;
#else
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
#endif
    tx.FDFormat            = FDCAN_FD_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0u;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)buf) == HAL_OK) {
        s_tx_ok++;
        return 1u;
    }
    s_tx_fail++;
    return 0u;
}

/* ================= CiA402 八态状态机（2026-11-25） =================
 *
 * 原实现只有 3 态（Fault / Operation enabled / Switch on disabled），是"私有状态字
 * 套了一层 CiA402 外衣"：主站发 Shutdown 无从响应，Fault reset 也没有上升沿语义。
 * 这里补成 CiA402 §6.1 的完整八态。
 *
 * 控制字入口统一为 16 位（`can_cia402_step(cw)`）：
 *   · 现在由 CMD_AGG 的 flags 翻译而来（can_flags_to_cw）；
 *   · 将来接标准 CANopen 主站时，直接喂 0x6040 的 16 位即可，状态机不用改。
 *
 * 与原有接口的映射：Operation enabled→ServoCmd_Arm，其余→Disarm，
 * Fault reset→ServoCmd_ClearFault。Arm 有联锁（未标定/未使能零点会失败），
 * 失败时**留在 SWITCHED_ON**，让主站从状态字 0x0023 看出"使能被拒"，
 * 而不是假装已使能。
 */
typedef enum {
    CIA402_NOT_READY = 0,
    CIA402_SWITCH_ON_DISABLED,
    CIA402_READY_TO_SWITCH_ON,
    CIA402_SWITCHED_ON,
    CIA402_OPERATION_ENABLED,
    CIA402_QUICK_STOP_ACTIVE,
    CIA402_FAULT_REACTION,
    CIA402_FAULT
} cia402_state_t;

static cia402_state_t s_cia_state = CIA402_NOT_READY;
static uint16_t       s_cia_cw_last;

/* CiA402 §6.1 命令解码：只看 bit1(Enable voltage) / bit2(Quick stop) / bit3(Enable op) */
enum { CIA_CMD_DISABLE_V = 1, CIA_CMD_QUICK_STOP, CIA_CMD_SHUTDOWN,
       CIA_CMD_SWITCH_ON, CIA_CMD_ENABLE_OP };

static uint8_t cia402_cmd(uint16_t cw)
{
    const uint8_t b1 = (uint8_t)((cw >> 1) & 1u);
    const uint8_t b2 = (uint8_t)((cw >> 2) & 1u);
    const uint8_t b3 = (uint8_t)((cw >> 3) & 1u);

    if ((b1 == 0u) && (b2 == 0u)) { return CIA_CMD_DISABLE_V; }
    if ((b1 == 0u) && (b2 == 1u)) { return CIA_CMD_QUICK_STOP; }
    if ((b1 == 1u) && (b2 == 0u)) { return CIA_CMD_SHUTDOWN; }
    if (b3 == 0u)                 { return CIA_CMD_SWITCH_ON; }
    return CIA_CMD_ENABLE_OP;
}

/* flags → 16 位控制字（保持原有帧格式不变）
 *   bit1 使能       → Enable operation (0x0F)
 *   bit2 快停       → Quick stop       (0x02)，优先级最高
 *   bit3 Shutdown   → 0x06
 *   bit4 Switch on  → 0x07
 *   bit5 Fault reset→ bit7（状态机内部做上升沿） */
static uint16_t can_flags_to_cw(uint8_t f)
{
    uint16_t cw = 0x0000u;

    if ((f & 0x04u) != 0u) { cw = 0x0002u; }            /* 快停优先 */
    else if ((f & 0x02u) != 0u) { cw = 0x000Fu; }       /* 使能 */
    else if ((f & 0x08u) != 0u) { cw = 0x0006u; }       /* Shutdown */
    else if ((f & 0x10u) != 0u) { cw = 0x0007u; }       /* Switch on */
    if ((f & 0x20u) != 0u) { cw |= 0x0080u; }           /* Fault reset */
    return cw;
}

static void can_cia402_step(uint16_t cw)
{
    const uint8_t fault = (BspServo_GetFault() != 0u) ? 1u : 0u;
    const uint8_t fr    = (uint8_t)(((cw & 0x0080u) != 0u) &&
                                    ((s_cia_cw_last & 0x0080u) == 0u));  /* 上升沿 */
    const uint8_t cmd   = cia402_cmd(cw);

    s_cia_cw_last = cw;

    if (s_cia_state == CIA402_NOT_READY) {
        s_cia_state = CIA402_SWITCH_ON_DISABLED;
    }

    /* 故障优先，任何状态都直接进 FAULT（略过 REACTION，BSP 已封锁输出） */
    if (fault != 0u) {
        if (s_cia_state != CIA402_FAULT) {
            s_cia_state = CIA402_FAULT;
            (void)ServoCmd_Disarm();
        }
        return;
    }
    if (s_cia_state == CIA402_FAULT) {
        if (fr != 0u) {
            (void)ServoCmd_ClearFault();
            s_cia_state = CIA402_SWITCH_ON_DISABLED;
        }
        return;
    }

    switch (s_cia_state) {
    case CIA402_SWITCH_ON_DISABLED:
        if (cmd == CIA_CMD_SHUTDOWN) { s_cia_state = CIA402_READY_TO_SWITCH_ON; }
        /* 兼容路径（2026-11-25）：老上位机只发 flags bit1=1（=Enable operation），
         * 不会跑 Shutdown→Switch on→Enable 的三步序列。严格 CiA402 下这里应当
         * 拒绝，但现场设备不可能为了新固件全部改主站，所以允许**补齐前置状态**
         * 直接使能。标准主站走正常序列时不会进这条分支（状态已经 >= SWITCHED_ON）。 */
        else if (cmd == CIA_CMD_ENABLE_OP) {
            if (ServoCmd_Arm() != 0u) { s_cia_state = CIA402_OPERATION_ENABLED; }
        }
        break;

    case CIA402_READY_TO_SWITCH_ON:
        if (cmd == CIA_CMD_SWITCH_ON) { s_cia_state = CIA402_SWITCHED_ON; }
        else if (cmd == CIA_CMD_ENABLE_OP) {      /* 同上：兼容一键使能 */
            if (ServoCmd_Arm() != 0u) { s_cia_state = CIA402_OPERATION_ENABLED; }
        }
        else if ((cmd == CIA_CMD_DISABLE_V) || (cmd == CIA_CMD_QUICK_STOP)) {
            s_cia_state = CIA402_SWITCH_ON_DISABLED;
        }
        break;

    case CIA402_SWITCHED_ON:
        if (cmd == CIA_CMD_ENABLE_OP) {
            /* Arm 失败（未标定/未标零点/辨识占用）→ 留在 SWITCHED_ON，状态字如实反映 */
            if (ServoCmd_Arm() != 0u) { s_cia_state = CIA402_OPERATION_ENABLED; }
        } else if (cmd == CIA_CMD_SHUTDOWN) { s_cia_state = CIA402_READY_TO_SWITCH_ON; }
        else if ((cmd == CIA_CMD_DISABLE_V) || (cmd == CIA_CMD_QUICK_STOP)) {
            s_cia_state = CIA402_SWITCH_ON_DISABLED;
        }
        break;

    case CIA402_OPERATION_ENABLED:
        if (cmd == CIA_CMD_DISABLE_V) { (void)ServoCmd_Disarm(); s_cia_state = CIA402_SWITCH_ON_DISABLED; }
        else if (cmd == CIA_CMD_QUICK_STOP) { (void)ServoCmd_Disarm(); s_cia_state = CIA402_QUICK_STOP_ACTIVE; }
        else if (cmd == CIA_CMD_SHUTDOWN) { (void)ServoCmd_Disarm(); s_cia_state = CIA402_READY_TO_SWITCH_ON; }
        break;

    case CIA402_QUICK_STOP_ACTIVE:
        if (cmd == CIA_CMD_DISABLE_V) { s_cia_state = CIA402_SWITCH_ON_DISABLED; }
        break;

    default:
        s_cia_state = CIA402_SWITCH_ON_DISABLED;
        break;
    }
}

/* CiA402 状态字（§6.3） */
static uint16_t can_cia402_sw(void)
{
    uint16_t sw;

    switch (s_cia_state) {
    case CIA402_NOT_READY:          sw = 0x0000u; break;
    case CIA402_SWITCH_ON_DISABLED: sw = 0x0040u; break;
    case CIA402_READY_TO_SWITCH_ON: sw = 0x0021u; break;
    case CIA402_SWITCHED_ON:        sw = 0x0023u; break;
    case CIA402_OPERATION_ENABLED:  sw = 0x0027u; break;
    case CIA402_QUICK_STOP_ACTIVE:  sw = 0x0007u; break;
    case CIA402_FAULT_REACTION:     sw = 0x000Fu; break;
    case CIA402_FAULT:              sw = 0x0008u; break;
    default:                        sw = 0x0000u; break;
    }
    /* bit10 Target reached：沿用私有状态字的 bit2 */
    if ((AppParam_GetStatusWord() & 0x0004u) != 0u) { sw |= 0x0400u; }
    sw |= 0x0200u;                        /* bit9 Remote：本节点接受远程控制 */
    return sw;
}

/* 把 CMD_AGG 的槽位真正下发到伺服（输出侧 → 电机侧的换算在这里） */
static void can_l1_apply(const CanAggCmd *c)
{
    uint8_t slot = (uint8_t)CAN_SLOT_ID;
    float p0, v0, kp0, kd0, tau0;

    /* 先跑 CiA402 状态机：快停 / 使能 / 故障复位统一由它处理，
     * 原来这里只判 bit2 快停、其余靠 ServoCmd_IsArmed()，没有状态概念。 */
    can_cia402_step(can_flags_to_cw(c->flags));

    if (slot >= c->n_slots) { return; }      /* 本节点不在这一帧的覆盖范围 */
    /* 非使能态只解析、不下发（等价于原来的 "未使能只解析"） */
    if (s_cia_state != CIA402_OPERATION_ENABLED) { return; }

    AppParam_GetMit(&p0, &v0, &kp0, &kd0, &tau0);   /* kp/kd 用 30066 配的值 */

    switch (AppParam_GetActiveMode()) {
    case APP_OP_MODE_MIT:
        (void)AppParam_SetMitCmd(c->q_des[slot]  * CAN_GEAR_RATIO,
                                 c->dq_des[slot] * CAN_GEAR_RATIO,
                                 kp0, kd0,
                                 c->tau_ff[slot] / CAN_GEAR_RATIO);
        break;
    case APP_OP_MODE_POS:
        (void)AppParam_SetPosP(c->q_des[slot] * CAN_GEAR_RATIO);
        break;
    default:
        break;                               /* 速度/力矩模式不吃 q_des */
    }
}

static void can_l1_gen_cmd(CanAggCmd *c, uint16_t cyc)
{
    uint8_t i;

    memset(c, 0, sizeof(*c));
    c->cycle_cnt = cyc;
    c->flags     = 0x01u;
    c->n_slots   = 1u;
    for (i = 0u; i < 5u; i++) {
        c->q_des[i]  = (float)cyc + 0.125f * (float)i;
        c->dq_des[i] = -(float)cyc - 0.5f * (float)i;
        c->tau_ff[i] = 2.0f * (float)cyc;
    }
}

static uint8_t can_l1_verify(const CanAggCmd *c)
{
    uint8_t i;

    if (c->n_slots != 1u) { return 0u; }
    if (c->flags != 0x01u) { return 0u; }
    for (i = 0u; i < 5u; i++) {
        if (c->q_des[i]  != ((float)c->cycle_cnt + 0.125f * (float)i)) { return 0u; }
        if (c->dq_des[i] != (-(float)c->cycle_cnt - 0.5f * (float)i)) { return 0u; }
        if (c->tau_ff[i] != (2.0f * (float)c->cycle_cnt)) { return 0u; }
    }
    return 1u;
}

void CanProto_L1Start(uint8_t selftest)
{
    s_agg.cycle_cnt = 0u;
    s_agg.n_slots   = 0u;
    s_cmd_age       = 0xFFFFu;
    s_cmd_to        = 0u;
    s_cmd_ever      = 0u;
    s_l1_cyc        = 0u;
    s_diag_div      = 0u;
    s_hb_div        = 0u;
    s_tx_ok         = 0u;
    s_tx_fail       = 0u;
    s_rx_ok         = 0u;
    s_rx_lost       = 0u;
    s_rx_dup        = 0u;
    s_rx_resync     = 0u;
    s_l1_st         = selftest;
    s_l1_on         = 1u;
    s_rx_zero       = (uint32_t)can_fbm_selfcheck();   /* zc 列显示：1=组帧自检通过 */
}

void CanProto_L1Stop(void)
{
    s_l1_on = 0u;
}

static void can_l1_rx(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u) {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t rd[64];

        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, rd) != HAL_OK) {
            break;
        }

        /* 标准 CANopen 优先处理（NMT 0x000 / SDO 0x601）。命中就直接吃掉这一帧，
         * 不落到下面的自定义 L1 分支 —— 两套 ID 体系不能混判。 */
#if defined(CANOPEN_MASTER_TEST) && (CANOPEN_MASTER_TEST != 0)
        /* 主站自测：只捕获 SDO 应答（0x581），从站逻辑不参与 */
        if (Canopen_MasterOnFrame(rx.Identifier, rd, (uint8_t)rx.DataLength) != 0u) {
            continue;
        }
#else
        if (Canopen_OnFrame(rx.Identifier, rd, (uint8_t)rx.DataLength) != 0u) {
            continue;
        }
#endif
        if (s_l1_on == 0u) { continue; }     /* 自定义 L1 未启用，其余帧忽略 */

        switch (rx.Identifier) {
        case CAN_ID_CMD_AGG:
            if (rx.DataLength == FDCAN_DLC_BYTES_64) {
                CanAggCmd tmp;
                if (can_agg_unpack(rd, &tmp) != 0u) {
                    s_agg      = tmp;
                    s_cmd_age  = 0u;
                    s_cmd_to   = 0u;
                    s_cmd_ever = 1u;      /* 主站上线：之后断流才需要失效安全 */
                    s_rx_ok++;                 /* 已解析的聚合指令数 */
                    can_l1_apply(&s_agg);
                    if (s_l1_st != 0u) {
                        if (can_l1_verify(&s_agg) == 0u) { s_rx_lost++; }
                    }
                } else {
                    s_rx_dup++;                /* n_slots 非法 */
                }
            }
            break;

        case CAN_ID_SYNC:
            s_sync_cnt++;
            break;

        default:
            break;
        }
    }
}

void CanProto_L1Task1ms(void)
{
    uint8_t buf[64];

#if defined(CANOPEN_MASTER_TEST) && (CANOPEN_MASTER_TEST != 0)
    Canopen_MasterPoll1ms();     /* 主站自测：100 ms 发一次 SDO 读 */

    /* 独立上报：can_push_periodic() 在 #if (PROD_BUILD==0) 里，生产构建取不到，
     * 而主站自测正是要在生产构建上跑，所以这里自己报一条（1 Hz，kind=11）。 */
    {
        static uint16_t rep_div;
        if (++rep_div >= 1000u) {
            float mv8[8];
            rep_div = 0u;
            Canopen_MasterGetStats(mv8);
            Telemetry_PushIdentResult(MAV_ID_CAN_TEST,
                                      (uint8_t)((Canopen_MasterRxOk() > 0u) &&
                                                (Canopen_MasterRxBad() == 0u)) ? 1u : 0u,
                                      Canopen_MasterRxOk(), mv8, 0.0f);
        }
    }
#else
    Canopen_Task1ms();
#endif

    /* 收帧必须**无条件**跑：CANopen（NMT/SDO）不依赖自定义 L1 是否启用，
     * 原来把它放在 `if (s_l1_on == 0) return;` 之后，标准主站发的 SDO 会被直接丢掉。 */
    can_l1_rx();

    if (s_l1_on == 0u) { return; }

    /* 失效安全：只在主站**曾经上线**（收到过 CMD_AGG）后才计时并断使能。
     *
     * 2026-11-22 实测根因：s_cmd_age 初始 0xFFFF，且原来无条件计时，
     * 于是上电第一个 1 ms 拍就置 s_cmd_to、60 ms 后 Disarm ——
     * 把 MAVLink 串口调试的使能也一起封掉，现象是"使能成功、0.1 s 内
     * 又失能、电机不动"。加 s_cmd_ever 门控后：主站没上过线就不强制
     * Disarm，MAVLink 调试恢复正常；主站上过线再断流，才走阻尼→断使能。 */
    if (s_cmd_ever != 0u) {
        if (s_cmd_age < 0xFFFFu) { s_cmd_age++; }
        if ((s_cmd_age > CAN_AGG_TMO_MS) && (s_cmd_to == 0u)) {
            s_cmd_to = 1u;
            /* 第一级：先阻尼，不要一上来就断使能。
             * MIT 下 kp=0、kd 保持，等效于给转子一个黏滞阻力，
             * 对腿关节比"自由停车"安全（不会因重力直接掉下来）。 */
            if (AppParam_GetActiveMode() == APP_OP_MODE_MIT) {
                (void)AppParam_SetMitCmd(BspServo_GetThetaMulti(), 0.0f,
                                         0.0f, CAN_DAMP_KD, 0.0f);
            }
        }
        /* 第二级：阻尼后仍持续无指令 → 断使能封锁输出 */
        if ((s_cmd_to != 0u) && (s_cmd_age >= (CAN_AGG_TMO_MS + CAN_DISARM_MS))) {
            if (ServoCmd_IsArmed() != 0u) { (void)ServoCmd_Disarm(); }
        }
    }

    /* 协议自环：自己发一帧 CMD_AGG，收到后由 can_l1_verify 逐字段比对 */
    if (s_l1_st != 0u) {
        CanAggCmd c;
        s_l1_cyc++;
        can_l1_gen_cmd(&c, s_l1_cyc);
        can_agg_pack(buf, &c);
        (void)can_l1_send(CAN_ID_CMD_AGG, buf, FDCAN_DLC_BYTES_64);
    }

    /* 主站不在（10 ms 无 CMD_AGG）时只发 1 Hz 心跳做"我在"，
     * 不跑 1 kHz 的 FB_MOTION —— 否则没人 ACK，TEC 一路涨到 bus-off，
     * 变成"每个节点都在刷错误帧"的恶性循环。主站一来（收到 CMD_AGG）
     * 自动恢复到 1 kHz。 */
    if (s_cmd_to != 0u) {
        if (++s_hb_div >= 1000u) {
            s_hb_div = 0u;
            memset(buf, 0, 8);
            buf[0] = (uint8_t)(CAN_NODE_ID & 0xFFu);
            buf[1] = 0x02u;                       /* bit1 = 等待主站 */
            (void)can_l1_send((uint16_t)(CAN_ID_HB + CAN_NODE_ID), buf,
                              FDCAN_DLC_BYTES_8);
        }
        return;
    }

    /* FB_MOTION：1 kHz */
    {
        CanFbMotion f;
        float id_m = 0.0f;
        float iq_m = 0.0f;

        Servo_GetIdqMeas(&id_m, &iq_m);
        f.statusword = can_cia402_sw();
        f.mode_act   = (uint8_t)ServoCmd_ActiveMode();
        f.err_code   = (uint8_t)(BspServo_GetFault() & 0x7Fu);
        f.q_act      = BspServo_GetThetaMulti() / CAN_GEAR_RATIO;
        f.dq_act     = BspServo_GetOmegaMech() / CAN_GEAR_RATIO;
        f.tau_act    = iq_m * CAN_KT_NM_PER_A * Servo_GetKtComp() * CAN_GEAR_RATIO;
        f.temp_motor = (uint16_t)(BspServo_GetTempMotor() * 10.0f);
        f.vbus       = (uint16_t)(BspServo_GetVbus() * 100.0f);
        f.seq_echo   = (uint8_t)(s_agg.cycle_cnt & 0xFFu);
        f.flags      = (s_cmd_to != 0u) ? 0x10u : 0x00u;   /* bit4 跟随超限 */
        f.follow_err = 0u;
        can_fbm_pack(buf, &f);
        (void)can_l1_send((uint16_t)(CAN_ID_FB_MOTION + CAN_NODE_ID), buf,
                          FDCAN_DLC_BYTES_24);
    }

    /* HEARTBEAT：100 ms */
    if (++s_hb_div >= CAN_HB_DIV_MS) {
        s_hb_div = 0u;
        memset(buf, 0, 8);
        buf[0] = (uint8_t)(CAN_NODE_ID & 0xFFu);
        buf[1] = (uint8_t)((s_cmd_to != 0u) ? 0x01u : 0x00u);
        (void)can_l1_send((uint16_t)(CAN_ID_HB + CAN_NODE_ID), buf,
                          FDCAN_DLC_BYTES_8);
    }

    /* FB_DIAG：20 ms */
    if (++s_diag_div >= CAN_DIAG_DIV_MS) {
        FDCAN_ErrorCountersTypeDef ec;
        s_diag_div = 0u;
        memset(&ec, 0, sizeof(ec));
        (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
        memset(buf, 0, 48);
        can_wr_u16(buf + 0, (uint16_t)s_rx_ok);
        can_wr_u16(buf + 2, (uint16_t)s_rx_lost);
        buf[4] = ec.TxErrorCnt;
        buf[5] = ec.RxErrorCnt;
        buf[6] = (uint8_t)s_cmd_age;
        buf[7] = (uint8_t)s_sync_cnt;
        (void)can_l1_send((uint16_t)(CAN_ID_FB_DIAG + CAN_NODE_ID), buf,
                          FDCAN_DLC_BYTES_48);
    }
}

void CanProto_Task1ms(void)
{
    uint8_t role = s_test_role;

    can_diag_poll();          /* 1 ms 采一次 PSR：LEC 读后即清，慢了就丢 */

    /* 每 10 ms 检查一次是否卡在 bus-off 的 INIT 里；与角色无关，必须一直跑 */
    if (++s_rc_div >= 10u) {
        s_rc_div = 0u;
        can_busoff_recover();
    }

#if (PROD_BUILD != 0)
    CanProto_L1Task1ms();
    (void)role;
#else
    if (role == 0u) { return; }

    if ((HAL_GetTick() - s_test_t0) >= CAN_TEST_TMO_MS) {
        CanProto_TestStop();
        return;
    }

    if (role == 1u) {
        if (can_tx_due()) { can_test_tx_step(); }
    } else if (role == 2u) {
        can_test_rx_step(CAN_TEST_ID, 0u);
    } else if (role == 3u) {
        /* PING：发 0x100，收回显 0x101。rx_ok = 完整往返次数 */
        if (can_tx_due()) { can_test_tx_step(); }
        can_test_rx_step(CAN_ECHO_ID, 0u);
    } else if (role == 4u) {
        /* ECHO：收 0x100，立刻回 0x101。
         * rx_ok = 收到多少 ping，tx_ok = 回了多少 echo，两者应基本相等。 */
        can_test_rx_step(CAN_TEST_ID, 1u);
    } else if ((role == 6u) || (role == 7u)) {
        /* role 6：L1 + 自环验证；role 7：L1 被动（无指令，测失效安全） */
        CanProto_L1Task1ms();
    } else {
        /* role 5：自环（配合 CAN_TEST_LOOPBACK=1）。
         * 发 0x100 并接收自己发的 0x100 —— 不经过收发器和线束，
         * 用来单独验证位定时 / BRS 数据段 / FIFO / 过滤器 / seq 统计。
         * 判据：rx_ok 与 tx_ok 基本相等、TEC=0、LEC=无错。 */
        if (can_tx_due()) { can_test_tx_step(); }
        can_test_rx_step(CAN_TEST_ID, 0u);
    }

    /* 每秒上报一次，便于用串口看接收速率与 uptime 趋势 */
    if (++s_rep_div >= 1000u) {
        s_rep_div = 0u;
        can_push_periodic(role);
    } else if (s_rep_div == 500u) {
        /* 错开半秒发寄存器快照：Telemetry_PushIdentResult 共用同一个载荷槽，
         * 同一拍连发两条会互相覆盖。 */
        can_push_regs(role);
    }
#endif /* PROD_BUILD */
}

#if (PROD_BUILD != 0)
/* 生产构建：测试入口保留符号、直接拒绝。
 * 这样 servo_cmd.c 的 action 20/21/22/23 不需要改，也不会链接不到。 */
uint8_t CanProto_SelfTest(void)            { return 0u; }
uint8_t CanProto_TestStart(uint8_t role)   { (void)role; return 0u; }
void    CanProto_TestStop(void)            { }
#endif
