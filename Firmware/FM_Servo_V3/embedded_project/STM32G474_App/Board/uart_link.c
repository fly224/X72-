/*
 * uart_link.c —— USART1 链路自检与错误恢复
 *
 * 为什么单独一个文件：
 *   串口联通是"能不能往下走"的第一道门。它必须在 BSP 初始化之前就能出字，
 *   而且必须能区分"TX 没通" / "MCU 卡住" / "BSP 失败" 三种情况。
 *   所以横幅用阻塞发送（不依赖 DMA、不依赖 TIM6、不依赖 100 Hz 遥测上下文），
 *   后面的二进制遥测则仍走 telemetry_tx.c 的 DMA 通道，两者互不覆盖。
 *
 * 编码说明：全部输出纯 ASCII（< 0x80），因此 MAVLink 解析器不会把横幅
 *           误当帧头（0xFD）；串口助手无论设 UTF-8 还是 GBK 都显示正常。
 */
#include "uart_link.h"
#include "telemetry_tx.h"
#include "usart.h"

#include <string.h>

#define UART_LINK_TIMEOUT_MS   20u
#define UART_LINK_WAIT_MS      2u

static uint32_t s_err_count;

/*
 * 拼串上界：所有 put_* 最多写到 s_lim（该字节留给 '\0'）。
 * 教训：横幅文案长度会随版本变，2026-11-15 曾因 buf[320] 装 380 B 越界写了
 * 60 B，把 HAL 的返回地址压成 "====\r\n"，返回时跳进 ASCII 取指触发 IBUSERR。
 * 因此这里不再靠"目测够大"，而是逐字节卡上界。
 */
static char *s_lim;

#define LIM_SET(buf)   (s_lim = (buf) + sizeof(buf) - 1u)

/*
 * 禁止内联。
 * 教训：AC6 -O3 会把 put_u32 的十进制循环在每个调用点完全展开，每个数位
 * 生成一条"常数魔乘 + 比较"，10 位 × 6 个调用点 = 1200+ 条指令。
 * 2026-11-15 加一行诊断输出时 Code 从 53424 涨到 59224（+5800 B），
 * 就是 uart_link_ReportDiag 一个函数被展开成 5676 B。
 * 这四个函数必须保持单一副本，否则每次加文案都会再涨几 KB。
 */
#define UL_NOINLINE    __attribute__((noinline))

/* ---------------- 极简格式化（不引入 printf，省 flash 与栈） ---------------- */
static UL_NOINLINE char *put_str(char *p, const char *s)
{
    while ((*s != '\0') && (p < s_lim)) {
        *p++ = *s++;
    }
    return p;
}

static UL_NOINLINE char *put_u32(char *p, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0u;

    if (v == 0u) {
        if (p < s_lim) {
            *p++ = '0';
        }
        return p;
    }
    while ((v != 0u) && (n < sizeof(tmp))) {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while ((n > 0u) && (p < s_lim)) {
        *p++ = tmp[--n];
    }
    return p;
}

static UL_NOINLINE char *put_hex2(char *p, uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    if (p < s_lim) {
        *p++ = hex[(v >> 4) & 0x0Fu];
    }
    if (p < s_lim) {
        *p++ = hex[v & 0x0Fu];
    }
    return p;
}

static UL_NOINLINE char *put_hex8(char *p, uint32_t v)
{
    p = put_hex2(p, (uint8_t)(v >> 24));
    p = put_hex2(p, (uint8_t)(v >> 16));
    p = put_hex2(p, (uint8_t)(v >> 8));
    p = put_hex2(p, (uint8_t)v);
    return p;
}

/* 只有条件成立才追加 " NAME"，用于把 CFSR 位翻成可读标签 */
static UL_NOINLINE char *put_flag(char *p, uint32_t cfsr, uint8_t bit, const char *name)
{
    if ((cfsr & (1uL << bit)) != 0u) {
        if (p < s_lim) {
            *p++ = ' ';
        }
        p = put_str(p, name);
    }
    return p;
}

/* 定点 1 位小数；NaN/Inf 输出 "nan"，避免上位机拿到 "0.0" 造成误判 */
static UL_NOINLINE char *put_f1(char *p, float v)
{
    uint32_t bits;
    int32_t  s;
    uint32_t u;

    memcpy(&bits, &v, sizeof(bits));
    if ((bits & 0x7F800000u) == 0x7F800000u) {
        return put_str(p, "nan");
    }

    s = (int32_t)((v >= 0.0f) ? (v * 10.0f + 0.5f) : (v * 10.0f - 0.5f));
    if (s < 0) {
        if (p < s_lim) {
            *p++ = '-';
        }
        u = (uint32_t)(-s);
    } else {
        u = (uint32_t)s;
    }
    p = put_u32(p, u / 10u);
    if (p < s_lim) {
        *p++ = '.';
    }
    if (p < s_lim) {
        *p++ = (char)('0' + (u % 10u));
    }
    return p;
}

/* ---------------- 发送 ---------------- */

/* ---------------- 非阻塞发送（直接写 TDR） ----------------
 * 历史坑（两次）：这里最初用 HAL_UART_Transmit（阻塞轮询），后来改用
 * HAL_UART_Transmit_DMA，两种都栽在同一件事上——**和 100 Hz 遥测共用
 * huart1 的 HAL 状态机**：遥测那帧的 TC 中断可能在我们的发送过程中把
 * gState 置回 READY，于是 TIM6 又起一帧 DMA，两边同时写 TDR。
 * 现象：开机第一行能出，之后回显时有时无，甚至永久发不出。
 *
 * 现在完全绕开 HAL：
 *   1) 先确认没有 DMA 在飞（gState==READY）；
 *   2) 让遥测暂停（TIM6 里直接 return，不会起新 DMA）；
 *   3) 逐字节轮询 TXE 写 TDR，最后等 TC；全程不碰 gState；
 *   4) 恢复遥测。
 * 一行 140 字符约 1.6 ms，放在主循环里执行，不占 20 kHz/1 ms 预算。
 */
#define UART_LINK_LINE_BUF   192u
#define UART_LINK_TC_GUARD   200000u     /* TC 等待上限（约 1.2 ms 的轮询次数） */

static char     s_ascii_buf[UART_LINK_LINE_BUF];
/* s_ascii_count 在本文件上方已定义 */



/* 二进制帧发送：与 UartLink_TrySendAscii 同一链路纪律，按 len 发送（可含 0x00）。
 * 注意：本函数在主循环/1ms 任务里调用，逐字节写 TDR 会阻塞约 len 字节时间，
 * MAVLink 应答帧最长 12+46=58 B @921600 ≈ 0.63 ms，可接受。 */
uint8_t UartLink_TrySendBinary(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint32_t guard;

    if ((buf == 0) || (len == 0u)) { return 0u; }
    if (huart1.gState != HAL_UART_STATE_READY) { return 0u; }

    Telemetry_SetTxPause(1u);
    if (huart1.gState != HAL_UART_STATE_READY) {   /* 复查：置位前 TIM6 可能已起 DMA */
        Telemetry_SetTxPause(0u);
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        guard = 0u;
        while ((USART1->ISR & USART_ISR_TXE) == 0u) {
            if (++guard > UART_LINK_TC_GUARD) { break; }
        }
        USART1->TDR = buf[i];
    }
    guard = 0u;
    while ((USART1->ISR & USART_ISR_TC) == 0u) {
        if (++guard > UART_LINK_TC_GUARD) { break; }
    }

    Telemetry_SetTxPause(0u);
    return 1u;
}

/* ---------------- 运行态诊断 ----------------
 * 目的：给出 20 kHz 控制节的实测周期预算。
 *   50 us @170 MHz = 8500 周期；峰值占比超过 ~70% 就必须拆 ISR。
 * 只输出一次，不进入周期流，避免污染 100 Hz 帧序列。
 */

/* ---------------- 寄存器快照 ---------------- */

/* ---------------- 上电横幅 ---------------- */
void UartLink_Init(void)
{
    /* 2026-11-21：ASCII 横幅已删除。
     * 上电后 USART1 不再输出任何可读文本，上位机收到的第一个字节就是
     * MAVLink 帧头（0xFD）—— 不用再在日志里跳过一串 banner。
     * 启动是否正常改由 30090 的 fault 位与 30091 状态读回判断。 */
    s_err_count = 0u;
}

/* ---------------- 启动结果 ---------------- */

/* ---------------- 错误恢复：防止 gState 永久卡在 BUSY_TX ---------------- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    s_err_count++;

    /* ORE 一旦置位且不清，后续所有接收都会失败；四个错误标志一律清掉 */
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (huart->gState == HAL_UART_STATE_BUSY_TX) {
        (void)HAL_UART_AbortTransmit(huart);   /* 内部会停 DMA 并把 gState 复位 */
    }
    huart->gState = HAL_UART_STATE_READY;      /* 双保险：HAL 万一没复位也能自愈 */
}

/* ---------------- 初始化分步打点 ---------------- */

/* ---------------- 故障现场：不经 HAL 的裸发送 ----------------
 * fault 上下文里 SysTick 不再推进、HAL 状态机也不可信，所以这里只用
 * USART1 的 TXE 标志轮询，配一个计数上限防止死等。
 */
static void RawPutChar(char c)
{
    uint32_t guard = 0u;

    while (((USART1->ISR & USART_ISR_TXE_TXFNF) == 0u) && (guard < 400000u)) {
        guard++;
    }
    USART1->TDR = (uint32_t)(uint8_t)c;
}

void UartLink_FaultDump(uint32_t cfsr, uint32_t hfsr, uint32_t mmfar,
                        uint32_t bfar, uint32_t msp)
{
    static char s_buf[512];       /* 静态缓冲：fault 时不能再吃栈 */
    char *p = s_buf;

    LIM_SET(s_buf);
    p = put_str(p, "\r\n[FAULT] HardFault\r\n[FAULT] CFSR=");
    p = put_hex8(p, cfsr);
    p = put_str(p, " HFSR=");
    p = put_hex8(p, hfsr);
    p = put_str(p, "\r\n[FAULT] MMFAR=");
    p = put_hex8(p, mmfar);
    p = put_str(p, " BFAR=");
    p = put_hex8(p, bfar);
    p = put_str(p, "\r\n[FAULT] MSP=");
    p = put_hex8(p, msp);
    p = put_str(p, "\r\n[FAULT] bits:");

    /* MemManage */
    p = put_flag(p, cfsr, 0u,  "IACCVIOL");
    p = put_flag(p, cfsr, 1u,  "DACCVIOL");
    p = put_flag(p, cfsr, 3u,  "MUNSTK");
    p = put_flag(p, cfsr, 4u,  "MSTK");
    p = put_flag(p, cfsr, 7u,  "MMARVALID");
    /* BusFault */
    p = put_flag(p, cfsr, 8u,  "IBUSERR");
    p = put_flag(p, cfsr, 9u,  "PRECISERR");
    p = put_flag(p, cfsr, 10u, "IMPRECISERR");
    p = put_flag(p, cfsr, 12u, "STKERR");
    p = put_flag(p, cfsr, 15u, "BFARVALID");
    /* UsageFault */
    p = put_flag(p, cfsr, 16u, "UNDEFINSTR");
    p = put_flag(p, cfsr, 17u, "INVSTATE");
    p = put_flag(p, cfsr, 18u, "INVPC");
    p = put_flag(p, cfsr, 19u, "NOCP");
    p = put_flag(p, cfsr, 24u, "UNALIGNED");
    p = put_flag(p, cfsr, 25u, "DIVBYZERO");

    p = put_str(p, "\r\n[FAULT] halted, reset to retry\r\n");
    *p = '\0';

    for (const char *q = s_buf; *q != '\0'; q++) {
        RawPutChar(*q);
    }

    /*
     * 异常帧窗口：硬件把 R0-R3/R12/LR/PC/xPSR 压在 MSP 之上，
     * 这里从当前 MSP 往高地址扫 20 个字，把落在 Flash 段（0x0800xxxx）
     * 的值标出来 —— 那就是故障时的 PC/LR，可直接回查 .map。
     */
    p = s_buf;
    p = put_str(p, "[FAULT] stack window (flash-range marked '*'):\r\n");
    for (uint32_t i = 0u; i < 20u; i++) {
        uint32_t v = ((volatile const uint32_t *)msp)[i];
        p = put_str(p, (i == 0u) ? "  " : " ");
        if ((v & 0xFF000000u) == 0x08000000u) {
            *p++ = '*';
        } else {
            *p++ = ' ';
        }
        p = put_hex8(p, v);
        if ((i % 5u) == 4u) {
            p = put_str(p, "\r\n");
        }
    }
    *p = '\0';

    for (const char *q = s_buf; *q != '\0'; q++) {
        RawPutChar(*q);
    }
}

uint32_t UartLink_GetErrorCount(void) { return s_err_count; }
