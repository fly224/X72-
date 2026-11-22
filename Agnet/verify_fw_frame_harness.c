/*
 * verify_fw_frame_harness.c —— 主机侧回放固件 telemetry_tx.c，抓取真实发送字节
 *
 * 做法：把 Board/telemetry_tx.c 原样参与编译，只 stub 掉 HAL 发送与 BSP 采样，
 *       让固件代码自己组帧、自己算 CRC，然后打印整帧十六进制。
 *       产出的帧再交给 Python（mavlink_fields，布局来自 91 个 mavlink_msg_*.h）
 *       与 Node（servo-web 页面 decodeFrame）分别解析，构成
 *       固件 → 协议布局 → 上位机 的三方闭环校验。
 *
 * 构建：见 Agnet/verify_fw_frame.sh
 */
#include "stm32g4xx_hal.h"
#include "bsp_servo.h"
#include "telemetry_tx.h"

#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;

static uint8_t  s_cap[512];
static uint16_t s_cap_len;
static uint8_t  s_stream[20000];      /* 累积 1 s 的全部字节：100 组 x 133 B */
static uint32_t s_stream_len;

/* 用 stub 顶替 DMA 发送：把字节拷出来，既留最后一帧也累积整段流 */
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    (void)huart;
    s_cap_len = (Size > sizeof(s_cap)) ? (uint16_t)sizeof(s_cap) : Size;
    memcpy(s_cap, pData, s_cap_len);

    if ((uint32_t)s_cap_len <= (sizeof(s_stream) - s_stream_len)) {
        memcpy(&s_stream[s_stream_len], s_cap, s_cap_len);
        s_stream_len += s_cap_len;
    }
    return HAL_OK;
}

/* telemetry_tx.c 的 200 ms 卡死自愈会调它；stub 直接放行 */
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
    huart->gState = HAL_UART_STATE_READY;
    return HAL_OK;
}

/* BSP 采样 stub：固定一组已知值，便于断言 */
float BspServo_GetThetaCached(void) { return 0.25f; }  /* 机械角 rad（只读快照） */
float BspServo_GetOmegaMech(void)   { return 3.0f;  }  /* 机械角速度 rad/s */
float BspServo_GetVbus(void)        { return 48.3f; }  /* 母线电压 V */

/*
 * 累积脉冲取 2097152 + 524288 = 1.25 圈：
 *   期望上位机换算出 turns=1.25、机械角=π/2（0.25 圈），
 *   单圈归一化与"多圈不失真"两件事同时被这一个值锁住。
 */
int64_t BspServo_GetPosSum(void)    { return 2621440LL; }

/* NTC 温度 stub：固定两个非对称值，便于断言"两个字段没串位"
 * （若 +40 与 +53 写反，断言会立刻发现） */
float BspServo_GetTempGan(void)     { return 41.2f; }   /* 驱动器温度 °C */
float BspServo_GetTempMotor(void)   { return 36.8f; }   /* 电机温度 °C */

void Servo_GetSample(float *iu, float *iv, float *iw, float *iq_ref, float *duty)
{
    *iu = 3.0f; *iv = -4.0f; *iw = 1.0f;
    *iq_ref = 2.5f;
    duty[0] = 0.55f; duty[1] = 0.45f; duty[2] = 0.50f;
}

int main(void)
{
    int i;

    huart1.gState = HAL_UART_STATE_READY;
    Telemetry_Init();
    Telemetry_Start();          /* 启动闸门：固件里由 Servo_Init() 末尾打开 */

    /*
     * 模拟 1 s 的 1 ms 任务：1000 次调用。
     * 内部 10 分频 => 期望正好 100 组，每组 133 B（65 + 44 + 24），合计 13300 B。
     * 这个数字同时锁定了"100 Hz 帧率"和"三条报文成对拼一次 DMA"两件事。
     */
    for (i = 0; i < 1000; i++) {
        Telemetry_Task1ms();
        huart1.gState = HAL_UART_STATE_READY;   /* stub 不等 DMA，手动放行 */
    }

    printf("LEN %u\n", (unsigned)s_stream_len);
    printf("HEX ");
    for (i = 0; i < (int)s_stream_len; i++) printf("%02X", s_stream[i]);
    printf("\n");
    printf("STAT tx=%lu drop=%lu recover=%lu\n",
           (unsigned long)Telemetry_GetTxCount(),
           (unsigned long)Telemetry_GetDropCount(),
           (unsigned long)Telemetry_GetRecoverCount());
    return 0;
}
