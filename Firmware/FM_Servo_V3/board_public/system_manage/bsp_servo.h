#ifndef BSP_SERVO_H

/* 生产固件开关（2026-11-22）
 * 0 = 开发构建：保留 CAN 自环/PING/ECHO/寄存器快照、辨识故障注入等测试支架
 * 1 = 生产构建：上述代码整块被预处理掉，只留 FDCAN + L1 协议 + bus-off 恢复；
 *              测试入口保留同名符号但直接拒绝，避免上位机误触发。
 * 这些支架在开发期极有用（几秒验证失败路径），但出货固件不该带。 */
#ifndef PROD_BUILD
#define PROD_BUILD   (1)
#endif

#define BSP_SERVO_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* ---------------- 硬件标定（依据 PG_X72_CAN 网表与 X72_FOC.ioc） ---------------- */
#define BSP_PWM_ARR              (4250U)          /* TIM1 中心对齐，170MHz/(2*4250)=20kHz */
#define BSP_TS_CURRENT           (0.00005f)       /* 电流环周期 50us */
#define BSP_ADC_VREF             (3.3f)           /* VREF+ = REF3033 3.3V */
#define BSP_ADC_COUNTS           (4095.0f)        /* 12bit */
#define BSP_CURRENT_BIAS_V       (1.65f)          /* HALF_VREF，U11 + R1/R2 分压 */
#define BSP_SHUNT_OHM            (0.001f)         /* R3 / R4 = 1mOhm */
#define BSP_OPAMP_GAIN           (30.0f)          /* R30/R34=30k 与 R28/R32=1k */
#define BSP_AMP_PER_VOLT         (1.0f / (BSP_SHUNT_OHM * BSP_OPAMP_GAIN))
/*
 * 母线分压（网表 pin_map.csv 第 121/215/217/221 行实证）：
 *   VM --R64(0402WGF1003=100k)-- $1N184 --R63(100k)-- $1N185 --R65(0402WGF1002=10k)-- GND
 *   VBUS_ADC 由 $1N185 经 R66(330R) 引出，ADC 高阻无直流压降
 *   => 分压比 = 10k/(100k+100k+10k) = 1/21
 * 历史坑：曾按"R65=1k"写成 1/201，母线读数被放大 10 倍（48V 显示成 480V）。
 */
#define BSP_VBUS_DIVIDER         (1.0f / 21.0f)
#define BSP_ENC_COUNTS           (2097152.0f)     /* 2^21，MT6835 */
#define BSP_TWO_PI               (6.28318530717958647692f)
/* ---------------- 测速：8 拍累计差分 + 一阶低通 ----------------
 * 2026-11-15 实测（30 rpm 速度闭环）暴露的问题：
 *   单拍差分（50 us）时，MT6835 的量化噪声 ±1 count 折算成 ±5.7 rpm，
 *   再叠加"某拍读到上一拍旧值"的隔拍效应，30025 的 omega 字段 std 达 18.8 rpm，
 *   而用多圈计数差分算出的真实转速 std 只有 3.4 rpm。
 *   速度环 Kp 一放大，这 18.8 rpm 噪声就会被直接放大成 iq 抖动。
 *
 * 改法：8 拍（400 us）累计后再差分，等效采样 2.5 kHz。
 *   · 量化噪声降到 ±1 count / 400us = ±0.71 rpm
 *   · 隔拍（少读一次）只影响 1/8，不会出现"0 和 2x 交替"
 *   · 低通截止仍是 ~95 Hz：alpha = 1-exp(-2*pi*fc*T)，T=400us -> 0.212
 *   代价：多 200 us 平均延迟，在 8 Hz 位置环处相位滞后 0.6 度，可忽略。
 *
 * 历史：原来 alpha=0.06（按 10 kHz 写，95 Hz）；切回 20 kHz 后同一 alpha
 * 变成 190 Hz，低速量化噪声通过过多，出现 ~8 Hz 振荡，故改 0.03。
 */
#define BSP_SPEED_AVG_N          (8U)          /* 累计拍数：8 * 50us = 400us */
/* 测速一阶低通系数。fc = -ln(1-a)/(2*pi*Ts)，Ts=0.4 ms。
 * 0.21 -> 95 Hz（原值）；0.35 -> 171 Hz（2026-11-22 试验：速度环相位裕度
 * 只有 42°，其中 39° 被这个低通吃掉，是低速纹波压不下去的主因之一）。
 * 代价是噪声放大，若电流出现高频抖动或发热，退回 0.21。 */
#define BSP_SPEED_LPF_ALPHA      (0.35f)       /* 2.5 kHz 下等效 171 Hz */
#define BSP_CURRENT_TRIP_A       (12.0f)          /* 硬件过流阈值，先于 9A 软件限幅 */
/* 电流零点标定的合理性窗口（V，运放输出）。
 * 静止 + MOE=0 时输出应停在中点 ≈Vref/2 = 1.65 V。
 * 落在窗口外说明采样链没工作（无供电/偏置丢失/通道断），
 * 此时拒绝标定并落 BSP_FAULT_CUR_SENSE，防止"标定通过、使能就炸"。 */
#define BSP_CAL_BIAS_MIN_V       (0.30f)
#define BSP_CAL_BIAS_MAX_V       (3.00f)
#define BSP_CAL_BIAS_MAX_DV      (0.20f)          /* 两相偏置允许差，0.2 V ≈ 6.7 A */
#define BSP_ENC_ERR_MAX          (5U)             /* 连续编码器异常次数 */

/* ---- 编码器帧 CRC 校验与 SPI 自愈（2026-11-22）----
 *
 * 背景：MT6835 的 6 字节帧里 rx[5] 是 CRC-8（poly X8+X2+X+1，覆盖
 * ANGLE[20:0]+STATUS[2:0] 共 24 位，见 Hardware/mt6835.txt:637）。
 * 原实现只取角度与 STATUS，**从不校验 CRC**，于是"数据错位但 status 恰好
 * 为 0"的帧会直接进控制环，只能靠角度跳变检测事后发现 —— 2026-11-18 与
 * 11-22 两次 0x0002 都是这个形态（诊断位 status=0 但跳变饱和）。
 *
 * BSP_ENC_CRC_INIT：datasheet 未写明初值。取 0 是标准做法；若实测发现
 * 上电即持续报编码器故障（err 饱和），说明初值应为 0xFF，改这里重烧即可。
 *
 * 自愈：连续 BSP_ENC_REINIT_N 帧坏（CRC/status/跳变任一）→ 在 1 ms 任务里
 * 重建 SPI1（DeInit+Init=一次外设复位，清 OVR 与状态机异常）。两次故障历来
 * 只能靠整机软复位恢复，而现场没人会去点复位，所以做成局部动作。
 * 恢复后连续 BSP_ENC_OK_N 帧有效 → 按位清掉编码器故障，不需要人工介入。 */
#define BSP_ENC_CRC_EN           (1)
#define BSP_ENC_CRC_INIT         (0x00U)
#define BSP_ENC_REINIT_N         (20U)
#define BSP_ENC_OK_N             (200U)

#define BSP_ENC_CS_PORT          GPIOC
#define BSP_ENC_CS_PIN           GPIO_PIN_11      /* PC11 -> MT6835 CSN */
#define BSP_ENC_FRAME_LEN        (6U)             /* 2B 命令 + 0x003~0x006 共 4B */
#define BSP_ENC_CMD_CONT_READ    (0xA0U)          /* C3~C0=1010 连续读角度 */
#define BSP_ENC_CMD_ADDR_LOW     (0x03U)          /* 起始地址 0x003 */

/* 故障位 */
#define BSP_FAULT_OVERCURRENT    (1U << 0)
#define BSP_FAULT_ENCODER        (1U << 1)
#define BSP_FAULT_ADC_SYNC       (1U << 2)
#define BSP_FAULT_BSP_INIT       (1U << 3)
#define BSP_FAULT_SPI            (1U << 4)
#define BSP_FAULT_OVERSPEED      (1U << 5)   /* 力矩模式下超速：空载小电流也会一直加速 */
#define BSP_FAULT_OVERVOLT       (1U << 6)   /* 母线过压：错接电源 / 减速器回馈泵升 */
#define BSP_FAULT_UNDERVOLT      (1U << 7)   /* 母线欠压：只在本级输出使能时判 */
#define BSP_FAULT_FOLLOW         (1U << 8)   /* 位置环跟随误差超限：堵转/丢步/过载 */
#define BSP_FAULT_PHASE_LOSS     (1U << 9)   /* 缺相：某一相实测无电流（见 Servo_PhaseMonTick） */
#define BSP_FAULT_CUR_SENSE      (1U << 10)  /* 电流采样异常：零点不在中点/两相差过大（2026-11-21） */

/* ---------------- 母线保护（2026-11-17 建，2026-11-23 改阈值） ----------------
 * OV = 60 V：功率管 JMSH1004BGQ 是 100 V 耐压，48 V 系统配 63 V 电容。
 *   60 V = 48 V 标称的 1.25 倍，是常规过压点；但距 63 V 电容额定只剩
 *   3 V（4.8%）余量 —— 制动回馈泵升必须靠硬件泄放兜住，不能指望这一层。
 * UV = 28 V：48 V 标称的 58%。
 *   ⚠ 台架 30.7 V 供电，28 V 阈值留 2.7 V（9%）余量，编码器校准/加减速
 *   的母线跌落不再误触欠压（原 30 V 只剩 0.7 V 余量，一跑就报 0x0080）。
 *   注意：这牺牲了"48V 系统欠压保护点"，正规 48 V 供电时建议改回 30 V。
 * 两个阈值都带 20 ms 消抖：母线在开关瞬间会有几十微秒的跌落/尖峰，
 * 单拍判据必然误跳。
 */
#define BSP_VBUS_OV_TRIP         (60.0f)
#define BSP_VBUS_UV_TRIP         (28.0f)
#define BSP_VBUS_DEBOUNCE        (20U)      /* 连续 20 次 @1 kHz = 20 ms */
#define BSP_VBUS_BLANK           (300U)     /* 上电 300 ms 内不判：分压/ADC 未稳 */

/* ---------------- 温度降额（2026-11-17） ----------------
 * 从 T0 开始按比例限流，到 T1 限到 0。取两路中更保守的一路。
 * GaN：NTC 贴在驱动/散热器上，结温远高于读数，所以 85 °C 就开始降。
 * 电机：绕组 NTC，F 级绝缘（155 °C）留 15 °C 余量，110→140 °C。
 * NTC 开路/短路时驱动层返回 NaN，此时不降额（宁可少一层保护，
 * 也不要因为测温故障把电机锁死）。
 */
#define BSP_DERATE_GAN_T0        (85.0f)
#define BSP_DERATE_GAN_T1        (105.0f)
#define BSP_DERATE_MOT_T0        (110.0f)
#define BSP_DERATE_MOT_T1        (140.0f)

/* 编码器角度连续性：20 kHz 下 4000 rpm 的合法变化约 7000 count，
 * 取 1/32 圈 = 65536 count（≈0.196 rad）留 9 倍余量。
 * 抓的是"status 正常但角度被干扰改坏"的帧 —— 那种帧会直接让电角度跳变。
 */
#define BSP_ENC_JUMP_MAX         (65536UL)
#define BSP_ENC_JUMP_N           (3U)       /* 连续 3 帧跳变即判故障 */

/* ---------------- 驱动层 API（bsp_servo.c） ---------------- */
HAL_StatusTypeDef BspServo_Init(void);
void              BspServo_DwtInit(void);
void              BspServo_PwmEnable(uint8_t enable);
uint8_t           BspServo_IsPwmEnabled(void);
/* 参数必须是 float：Cortex-M4F 没有 double 硬件，
 * 之前用 double 传入导致每拍 3 次 __aeabi_dmul + 3 次 __aeabi_d2iz，
 * 实测吃掉 1025 个周期（10.6% 的 20 kHz 预算）。 */
void              BspServo_SetDuty(float d1, float d2, float d3);
void              BspServo_ReadInjected(float *iu, float *iw);
/* 只读最近一次注入结果（不重新触发 ADC），供命令/遥测侧使用 */
void              BspServo_GetCurrents(float *iu, float *iw);
float             BspServo_GetVbus(void);

/* ---- NTC 温度（ADC2 常规组，10 Hz）----
 * NTC_GAN   = PA4 / ADC2_IN17，板上 R53 RL0603N503F4150FTF（50k, B=4150K）
 * NTC_MOTOR = PC4 / ADC2_IN5 ，经 U14 外接 2P 探头
 * 两者均与 50kΩ 上拉（R56/R57）分压；开路/短路时返回 NaN，不伪造读数。
 */
float             BspServo_GetTempGan(void);
float             BspServo_GetTempMotor(void);
/* 最近一次的原始码值（标定用）；返回 1 = 有效，0 = 尚未采到 */
uint8_t           BspServo_GetNtcRaw(uint16_t *gan, uint16_t *motor);

/* ---- 电流零点标定 ----
 * 条件：MOE=0（无输出）且 |机械角速度| < 1 rad/s，约 200 ms 完成（4000 次注入采样）。
 * 返回 1 = 已启动；0 = 条件不满足（有输出/电机在转/正在标定）。
 * 标定值只存 RAM，掉电丢失 —— 上电后需重新标定一次。
 */
uint8_t           BspServo_StartCurrentCal(void);
uint8_t           BspServo_IsCurrentCalibrated(void);
uint8_t           BspServo_GetCalReject(void);   /* 0=正常 1=偏置出窗 2=两相差过大（2026-11-21） */

/* 临时屏蔽"编码器故障"判据（2026-11-21）。
 *
 * 存在理由：^ 是唯一能修编码器 EEPROM 的通道，而它要求 fault==0 才能启动；
 * 编码器一旦持续跳变，故障清掉下一帧又置位 → ^ 永远起不来 → 死锁。
 * ^ 本身用开环拖动（虚拟角），完全不读编码器，所以这段屏蔽是安全的。
 * 只在 ^ 运行期间置 1，结束/中止时清 0；上电默认 0。 */
void              BspServo_SetEncFaultMask(uint8_t on);
/* 按位清故障（2026-11-22）：编码器自愈只能清自己那一位，
 * 不能用 BspServo_ClearFault() 全清 —— 那会把过流/过压等更严重的故障一起抹掉。 */
void              BspServo_ClearFaultBits(uint32_t mask);
void              BspServo_GetCurrentBias(float *u, float *w);
/* 从 Flash 恢复零漂：直接置"已标定"，跳过 200 ms 采样。
 * 只在 ParamStore_Load 成功且 CRC 通过后调用 —— 温漂大的场合仍应重新 C 一次。 */
void              BspServo_SetCurrentBias(float u, float w);
/* 过流留证：全程最大 |i| 及对应原始码值（0 = 尚未发生）；ClearOvcInfo 清零 */
void              BspServo_GetOvcInfo(float *iu, float *iw, uint16_t *raw_u, uint16_t *raw_w);
void              BspServo_ClearOvcInfo(void);
void              BspServo_SlowTask1ms(void);
HAL_StatusTypeDef BspServo_StartEncoderDma(void);
float             BspServo_GetThetaRad(void);
float             BspServo_GetThetaCached(void);
/* 多圈机械角（rad，不折叠，与 GetThetaCached 同零点）。
 * 位置环的 theta_pos 输入用它：单圈角跨 2π 会产生突跳，导致 pos_ref 必须折叠，
 * 而折叠只能保证 ±π 内正确。用多圈角后 pos_ref 可以直接给绝对目标。 */
float             BspServo_GetThetaMulti(void);
int64_t           BspServo_GetPosSum(void);
/* 多圈角零点偏移 s_tm_base（上电第一拍单圈角）。theta_multi = s_tm_base +
 * s_pos_sum·(2π/2^21)；外部协议若用 enc_sum 基准给目标，须加这个偏移才能
 * 和 theta_multi（MIT/位置环控制基准）对齐（2026-11-25 修复 MIT 45mrad 偏移）。 */
float             BspServo_GetThetaBase(void);
float             BspServo_GetOmegaMech(void);
/* 未滤波单拍角速度（rad/s）。机械辨识（谐振/惯量/齿槽）专用：
 * 常规测速是 8 拍累计 + 95 Hz 低通，测不到 100 Hz 以上的谐振。 */
float             BspServo_GetOmegaFast(void);
uint32_t          BspServo_GetEncoderRaw(void);

/* ---- MT6835 客户端自校准支持（CAL_EN + 寄存器读写）----
 * 网表：MT6835.4 (CAL_EN) -> STM32 PB9。
 * PB9 在 .ioc 里没有分配，所以这里直接初始化 GPIO，不依赖 CubeMX 重新生成。
 * ReadEncReg 走阻塞式 SPI（3 字节），会短暂占用与 20 kHz 角度 DMA 共用的总线；
 * 内部先等 s_spi_busy 清零，期间控制节拍靠上一次的角度值，丢 1~2 拍，校准场景可接受。
 */
void              BspServo_InitEncCalPin(void);
void              BspServo_SetEncCalEn(uint8_t en);
uint8_t           BspServo_ReadEncReg(uint16_t addr);
void              BspServo_WriteEncReg(uint16_t addr, uint8_t data);
uint8_t           BspServo_BurnEeprom(void);   /* 返回 1 = 芯片回 0x55 确认 */
uint8_t           BspServo_GetEncoderStatus(void);
/* 编码器故障现场打包：b0..2 status、b3..5 连续错误数、b6..7 连续跳变数。
 * 置 BSP_FAULT_ENCODER 的两条路径（帧内容坏 vs 跳变）靠它区分。 */
uint8_t           BspServo_GetEncDiag(void);
void              BspServo_SetEncoderZero(float zero_rad);
/* 读回当前零点（rad，已归一化到 [0,2π)），供 Flash 持久化使用 */
float             BspServo_GetEncoderZero(void);
uint8_t           BspServo_IsEncoderZeroed(void);   /* 1 = Z 已成功过（电流闭环联锁用） */
uint32_t          BspServo_GetFault(void);
void              BspServo_ClearFault(void);
void              BspServo_RaiseFault(uint32_t bit);

/* ---- RGB 状态指示灯（共阳，GPIO 低电平点亮）----
 * PC13=红(LED1)、PC14=绿(LED2)、PC15=蓝(LED3)。由 BspServo_SlowTask1ms 每 1 ms 推进：
 *   故障(s_fault!=0)      -> 红色快闪 5 Hz（100ms 亮 / 100ms 灭）
 *   参数辨识进行中         -> 蓝色快闪 5 Hz（100ms 亮 / 100ms 灭）
 *   运行(PWM 使能在环)     -> 绿色慢闪 2 Hz（250ms 亮 / 250ms 灭）
 *   待机(未使能、无故障)   -> 绿色常亮
 * 优先级：故障 > 辨识 > 运行 > 待机。状态切换立即从"亮"相位开始。不进 20 kHz 控制环。
 */
typedef enum {
    BSP_LED_IDLE  = 0U,   /* 待机：绿常亮 */
    BSP_LED_RUN   = 1U,   /* 运行：绿慢闪 2 Hz */
    BSP_LED_FAULT = 2U,   /* 故障：红快闪 5 Hz */
    BSP_LED_IDENT = 3U    /* 参数辨识：蓝快闪 5 Hz */
} BspLedState_t;

void           BspServo_LedTask1ms(void);
BspLedState_t  BspServo_LedGetState(void);
uint32_t          Servo_GetInjectedMissCount(void);
uint32_t          Servo_GetMaxStepCycles(void);

/* ---- 看门狗（IWDG，直接寄存器操作，不依赖 HAL 模块）----
 * 250 ms 超时。**必须在主循环里喂**，不能在 1 ms 定时器中断里喂 ——
 * 中断还能跑而主循环卡死，正是最需要复位的场景。
 * 调试时停在断点会被复位，需要单步就把 BSP_WDG_ENABLE 改 0 重编。
 */
void              BspServo_WdgInit(void);
void              BspServo_WdgKick(void);

/* ---- 温度降额系数（1.00 = 不限流，0.00 = 完全封锁）----
 * 由 BspServo_SlowTask1ms 每 1 ms 更新，控制环在所有模式下统一乘这个系数。
 */
float             BspServo_GetDerate(void);
/* 母线保护阈值回读（供诊断/上位机显示） */
void              BspServo_GetVbusLimits(float *ov, float *uv);

/* 极对数：必须与 three_loop_deploy.c 里写死的 14.0F 一致。
 * Z 辨识会给出实测值（ratio=13.98），但模型侧是常数，所以这里也必须是常数。
 * 原来只有 servo_isr.c 内部的 SERVO_CUR_POLE，现在提到头文件，
 * 供磁链辨识等模块共用一份定义。 */
#define BSP_POLE_PAIRS    (14.0f)

/* ---------------- 控制接线 API（servo_isr.c） ---------------- */
#define SERVO_MODE_POSITION   (0u)   /* 位置模式（模型三环）：pos_ref 由 Servo_SetPosRef 给定 */
#define SERVO_MODE_VELOCITY   (1u)   /* 速度轨迹模式（模型三环）：pos_ref 由速度轨迹积分得到 */
#define SERVO_MODE_SPEED      (2u)   /* 速度闭环模式（手写）：速度误差直接进 PI -> iq_ref */
#define SERVO_MODE_POS_LOOP   (3u)   /* 位置环模式（手写）：规划 + P -> 速度环（2026-11-17） */

void Servo_Init(void);
void Servo_ControlTick(void);        /* ADC1/2 注入完成后调用，20kHz */
void Servo_SlowTask1ms(void);        /* TIM6 1ms 任务调用 */
void Servo_SetPosRef(float rad);     /* 位置指令，机械角 rad（位置模式有效） */
void Servo_SetEnable(uint8_t en);    /* 使能标志；PWM 主输出另由 BspServo_PwmEnable 控制 */

/* ---- 速度模式（梯形 / S 曲线速度轨迹） ---- */
void  Servo_SetMode(uint8_t mode);          /* SERVO_MODE_POSITION / SERVO_MODE_VELOCITY */
void  Servo_SetVelRef(float rad_s);         /* 速度指令，机械角 rad/s，内部限幅 ±100 */

/* ---- 开环 dq 电流闭环（仅用于电角度/相序对齐，正常运行时必须关闭）----
 * 与闭环的区别：θ 用**虚拟电角度** theta_e（不是编码器角度），所以不依赖零点标定；
 * 电流由 PI 限制在 id_ref_a（限幅 0~5 A），不会像纯电压开环那样"要么没力要么过流"。
 * iq 给定恒为 0，矢量只沿虚拟 d 轴 —— 转子被吸到该方向后即可反算零点。
 */
void    Servo_SetOpenLoop(uint8_t enable, float theta_e, float id_ref_a);
uint8_t Servo_IsOpenLoop(void);
float   Servo_GetOLTheta(void);   /* 当前虚拟电角度 rad，[0, 2π) */

/* 开环匀速拖动（编码器自校准专用）。en=1 后虚拟角由 20 kHz ISR 自行累加，
 * 调用方不要再调 Servo_SetOpenLoop()（会覆盖角度把转子拽回起点）。
 * w_elec 是**电**角速度 rad/s；机械 rpm -> w_e = rpm/60*2π*极对数。 */
void    Servo_SetOLSpin(uint8_t en, float w_elec);
uint8_t Servo_IsOLSpinning(void);

/* ---- 辨识参数接管控制律（P0-1）----
 * @ 提供 R/L，+ 提供 ψ：写入后立即生效（ψ -> Kt 与 BEMF 前馈，L -> dq 解耦）。
 * 越界/NaN 一律拒绝，不会写坏环路；上电默认值等于原编译期常量。 */
void    Servo_SetIdentPsi(float psi_wb);
/* 同上，但记录"辨识时的电机温度"作为 Kt / BEMF 前馈的温度基准。
 * ψ 是有温度的量，不记基准就会出现"热机辨识、冷机二次补偿"的 8% 误差。
 * t_mot_c 为 NaN 或超出 −20~150 °C 时退回 20 °C 基准（等同 Servo_SetIdentPsi）。 */
void    Servo_SetIdentPsiEx(float psi_wb, float t_mot_c);
/* 当前生效的 ψ 温度基准（°C）与是否已知 */
float   Servo_GetPsiTRef(void);
uint8_t Servo_PsiTRefOk(void);

/* ---- 运行时调参（MAVLink 30082 AppDebugParam 的槽位 0/1）----
 * 这两个量原来是编译期常量：改一次就要重编译 + 烧录，而每次烧录都会擦掉
 * 参数页（cal/z/psi 全归零），等于每调一个参数就要重做整套标定。
 * 都带越界保护，返回 1 = 接受、0 = 拒绝（调用方不要静默截断）。 */
uint8_t Servo_SetAlignId(float a);   /* Z 对齐注入的 d 轴电流 A，0.2~4.0 */
float   Servo_GetAlignId(void);
uint8_t Servo_SetOLVlim(float f);    /* 开环电压限幅系数 ×vbus，0.02~0.60 */
float   Servo_GetOLVlim(void);
void    Servo_SetIdentRl(float r_ohm, float l_h);
void    Servo_GetIdentParams(float *r_ohm, float *ld_h, float *psi_wb, float *kt);
uint8_t Servo_IdentApplied(void);

/* 电角度补偿（rad，叠加在电流环 Park 变换的 th_e 上）。
 * 正常为 0；仅用于编码器零点/线性度异常时的现场排查：
 * 扫这个值找"正反向都出力"的点，可判定是零点错还是编码器非线性。
 * 只影响电流环，不改变位置/速度环使用的机械角与 Flash 里的零点。 */
void    Servo_SetElecOff(float rad);
float   Servo_GetElecOff(void);

/* ---- 控制状态机（给定源，单一真值）----
 * SPEED/POS/MIT 的层级关系由 s_src 派生，调用方不需要也不应该逐个开下层环。
 * 返回 NONE 表示没有给定源在跑（封锁态）。 */
typedef enum {
    SERVO_SRC_NONE = 0, SERVO_SRC_TORQUE, SERVO_SRC_SPEED, SERVO_SRC_POS,
    SERVO_SRC_MIT, SERVO_SRC_OPENLOOP, SERVO_SRC_NUM
} ServoSrc;

ServoSrc    Servo_GetSrc(void);
uint8_t     Servo_GetSrcSeq(void);
const char *Servo_SrcStr(void);          /* "TORQ"/"SPD"/"POS"/"MIT"/"OL"/"NONE" */
/* 最近一拍限幅后的 dq 电压（虚拟角电流环分支的输出）。
 * R 辨识用 |v| = R|i| 反推相电阻，必须读限幅后的值 —— 限幅前的 PI 输出
 * 在电压顶死时会偏大，直接拿去算 R 会系统性偏高。 */
void    Servo_GetOpenLoopVdq(float *vd, float *vq);

/* ---- 单频电压注入（L 辨识用）----
 * 虚拟角固定 0，d 轴注入 amp·sin(2π·f·t) 的开环电压（不走电流环，否则被压掉）。
 * 依赖：MOE=1、母线 > 5 V。注入期间不产生转矩（id 轴），转子不会转。
 */
void    Servo_SetVoltInject(uint8_t enable, float amp_v, float freq_hz);
uint8_t Servo_IsVoltInject(void);

/* ---- 5/7 次谐波电流抑制（MSRF）----
 * 把实测 αβ 电流变到 −5θe / +7θe 两个同步坐标系，各用一组 PI 压到 0，
 * 反变换后叠加到电压指令。频率随转速自动跟随，不需要重算系数。
 * 依赖电流环（力矩模式或速度闭环），关掉时清积分。
 * 诊断：id5 / iq5 / id7 / iq7 是压完之后的残余量（A），越接近 0 说明效果越好。
 */
void    Servo_SetHarmonic(uint8_t enable);
uint8_t Servo_IsHarmonic(void);
void    Servo_GetHarmonicIdq(float *id5, float *iq5, float *id7, float *iq7);

/* ---- 20 kHz 波形记录（谐波分析用）----
 * 100 Hz 遥测无法观察 6 次谐波（30 rpm 时 34 Hz 已贴 Nyquist，更高转速直接混叠）。
 * 这里在控制节拍里直接采实测 iq，1024 点。
 *
 * 分频由 Servo_SetRecordDiv 控制：
 *   div=1  → 20 kHz，1024 点 = 51 ms，分辨率 19.5 Hz（原行为，看 5/7 次用）
 *   div=5  →  4 kHz，1024 点 = 256 ms，分辨率 3.9 Hz（100 rpm 的 6 次 140 Hz 才看得清）
 * 缩放 131：int16 范围对应 ±250 A。采满自动停止，由命令层逐行 hex 送出。
 */
void     Servo_StartRecord(void);
/* 提前停止（2026-11-21）。原来只有"采满 1024 点自动停"，一旦触发就无法中止：
 * div=100 时窗口 5.12 s，现场想改参数重采只能干等，而且 op=1 期间一直返回"忙"。
 * 与 StartRecord 一样是单字节写，20 kHz 里只读它，不存在 tearing。 */
void     Servo_StopRecord(void);
uint8_t  Servo_IsRecording(void);
uint16_t Servo_GetRecordN(void);
int16_t  Servo_GetRecordAt(uint16_t i);
void     Servo_SetRecordDiv(uint8_t div);   /* 1..100，每 div 拍存一点 */
uint8_t  Servo_GetRecordDiv(void);

/* 录制通道：0 = iq（默认，兼容 harm_fft.py），1 = 相电流 iu + 同拍电角度，
 *           2 = vq 积分项（mV，2026-11-20 加）。
 *
 * 通道 1 的存在是为了看"相电流过零附近有没有死区平台"：
 * 单纯存电流不够，必须同拍存角度才能做周期同步平均 ——
 * 100 rpm 的转速波动有 9%，靠时间反推相位 6 个周期就糊了。
 * 角度编码 2π→65536，按 uint16 解。
 *
 * 通道 2 是"BEMF 前馈是否与温度一致"的判据：vq 总和在前馈误差下不变
 * （PI 会抵消），而积分项会镜像该误差 —— 改前 2000 rpm/84 °C 约 −0.98 V，
 * 前馈乘 Kt 补偿后回到 0 附近。见 servo_isr.c 的通道注释。 */
void     Servo_SetRecordChannel(uint8_t ch);
uint8_t  Servo_GetRecordChannel(void);
int16_t  Servo_GetRecordThetaAt(uint16_t i);

/* ---- 缺相检测（两相采样，2026-11-21）----
 * 本板只采 U、W，V 相由 KCL 重构，所以"某相无电流"在 KCL 上不可见，
 * 必须用三相电流的 RMS 关系反推。判据与实测阈值见 servo_isr.c 的说明。
 *
 * 20 kHz 侧每拍调用（内部 10 kHz 抽取、200 ms 结算一个窗口）；
 * 1 ms 侧只负责"是否升级为故障"。 */
void     Servo_PhaseMonTick(float iu, float iw, float th_cmd);
/* 打包诊断：b0=消抖后结论(0正常 1 U缺 2 V缺 3 W缺)
 *           b1=本窗口原始结论(0/1/2/3，4=不可判：无激励或角度没扫开)
 *           b2=rms(iu+iw)/max(rms_iu,rms_iw)×100（健康≈100，V 缺≈0）
 *           b3=min/max(rms_iu,rms_iw)×100（健康≈100） */
uint32_t Servo_GetPhaseMon(void);
void     Servo_SetPhaseTrip(uint8_t en);   /* 1 = 判定缺相后置 BSP_FAULT_PHASE_LOSS */
uint8_t  Servo_GetPhaseTrip(void);

/* ---- 速度环陷波（谐振抑制，2026-11-21）----
 * 作用在**反馈**上，只削 f0 附近的环路增益，不动给定响应。
 * depth_db 是混合深度（0 = 直通，40 ≈ 完整陷波），Q 决定带宽。
 * 参数只存 RAM，上电默认关闭；配置走 30096 AppAdvCfg。 */
void    Servo_SetNotch(uint8_t en, float f0_hz, float depth_db, float q);
void    Servo_GetNotch(uint8_t *en, float *f0_hz, float *depth_db, float *q);
void    Servo_SetNotchOrder(uint8_t order);       /* 阶次跟踪：0=固定 f0，k=跟随 k 次/转 */
uint8_t Servo_GetNotchOrder(void);

/* ---- 编码器电角度下的 dq 电流闭环（力矩模式）----
 * 与上面"虚拟角"版本共用同一套 PI/限幅，唯一区别是 θ 来自编码器：
 *   th_e = 14*(dir*theta_m) + π/2 + 1.005e-3*(dir*omega_m)
 * 即与 three_loop_deploy.c:51 完全相同的电角度定义，所以必须先做完电角度辨识（Z）。
 * id 给定恒为 0，iq 由 iq_ref_a 给定（限幅 ±5 A）。
 * 只在 MOE=1 时积分：MOE 关闭期间保持中点并清积分，避免开通瞬间的积分冲击。
 */
void    Servo_SetCurrentMode(uint8_t enable, float iq_ref_a);
uint8_t Servo_IsCurrentMode(void);
void    Servo_GetIdqMeas(float *id, float *iq);

/* ---- 力矩模式的给定整形与精度补偿 ----
 *
 * iq 给定经一阶低通后再进电流环（默认 20 ms）。为什么需要：
 *   1 A 阶跃 → 加速度 Kt·Δi/J = 0.091/5.47e-5 ≈ 1660 rad/s²，
 *   齿轮间隙和齿槽会被整个激起来。20 ms 把 d(iq)/dt 压到 50 A/s，
 *   对应 8 Hz 带宽，仍在多数力控外环之上。
 *   做阶跃响应测试时把 tau 设 0，让给定直通。
 */
void    Servo_SetCurIqMax(float a);        /* 力矩模式 iq 限幅，0.05~5 A */
float   Servo_GetCurIqMax(void);
void    Servo_SetDerate(float k);          /* 温度降额系数 0..1，1 ms 任务写 */
float   Servo_GetDerate(void);

/* ---- MIT 阻抗控制（2026-11-17）----
 * τ = kp·(p_des − p) + kd·(v_des − v) + τ_ff，内部除以 Kt 变成 iq。
 * 与速度环互斥：Servo_SetMitEn(1) 前必须先 Servo_SetCurrentMode(1, 0)。
 * p 用多圈机械角（与 Servo_SetPosRef 同坐标系）。
 */
void    Servo_SetMitEn(uint8_t en);
void    Servo_SetMitCmd(float p, float v, float kp, float kd, float tau);
uint8_t Servo_IsMit(void);
void    Servo_GetMitState(float *tau, float *ep, float *kp, float *kd, float *p_now);
float   Servo_GetMitDt(void);              /* 上拍用到的帧龄 s（外推诊断） */
void    Servo_MitTask1ms(void);            /* 命令超时：帧停 20 ms 冻结 v_des/τ_ff */

void    Servo_SetCurRampTau(float tau_s);  /* 一阶低通时间常数，0 = 直通 */
float   Servo_GetCurRampTau(void);
float   Servo_GetCurIqRamp(void);          /* 斜坡后的实际给定（诊断） */

/* 死区电压补偿增益。0 = 关闭（默认），1 = 按 DTG 算的理论值。
 * 理论值 0.0059·Vbus ≈ 0.18 V，除以 R=0.4 Ω 相当于 0.45 A ——
 * 小力矩时它比给定本身还大，所以要精确力矩就得补。 */
void    Servo_SetCurDeadGain(float g);
float   Servo_GetCurDeadGain(void);

/* 力矩估算（N·m），Kt = 1.5·p·ψ = 0.0910 N·m/A（ψ 用实测 4.333 mWb）。
 * 2026-11-19 起乘温度补偿系数 s_kt_comp（见下方 Kt 补偿接口）。 */
float   Servo_GetTorqueEst(void);

/* ---- Kt 温度补偿（2026-11-19 实测 ψ 温漂 α=−0.12 %/K）----
 * 只影响力矩换算（MIT 的 τ→iq、τ 回读、力矩估算），不影响速度/位置环。
 * Servo_UpdateKtComp 由 1 ms 任务每拍调用，入参为电机 NTC 温度（°C，NaN 则保持不变）。 */
void    Servo_UpdateKtComp(float t_mot_c);
float   Servo_GetKtComp(void);          /* 当前补偿系数，1.0 = 未补偿 */
float   Servo_GetKtNow(void);           /* 当前生效的 Kt = 0.0910 × 系数，N·m/A */
void    Servo_SetKtCompEn(uint8_t en);  /* 0 关闭（系数归 1），1 开启（默认） */
uint8_t Servo_GetKtCompEn(void);

/* ---- 电流环增益与 id 给定（运行时改，供阶跃整定用）----
 * 两个增益同时缩放只改带宽、零点不变；单独改会破坏极点抵消，仅作调试。 */
void    Servo_SetCurGains(float kp, float ki_ts);
void    Servo_GetCurGains(float *kp, float *ki_ts);
void    Servo_SetCurIdRef(float a);
void    Servo_GetCurRefs(float *id_ref, float *iq_ref);

/* ---- 速度闭环（Board 层实现：2 kHz 速度环 -> iq_ref -> 20 kHz 电流环）----
 * 与 SERVO_MODE_VELOCITY 的本质区别：
 *   那条路是"速度轨迹积分成位置给定"，速度误差不参与运算，靠位置环 P 间接给出
 *   转速，严格说只是速度指令跟踪；本接口让 (w_ref - w_meas) 真正进入 PI。
 *
 * 算法与 simulink_code_gen/fcn_three_speed_loop_deploy.m 一一对应：
 *   每 10 拍（2 kHz）算一次，其余拍保持输出
 *   i_term += ki*Ts*err，clamp ±i_qmax；out = kp*err + i_term，clamp ±i_qmax
 * 参数同源：kp = 0.07292272 A/(rad/s)，ki*Ts = 10.190689*500us = 5.0953445e-3。
 *
 * 依赖三道联锁（与力矩模式相同）：C 电流零点、Z 电角度、无故障位。
 * w_ref 用**编码器坐标系**（rad/s，与 BspServo_GetOmegaMech 同号），
 * 内部按 dir_sign 换算到模型坐标系，所以 dir=-1 时也不是正反馈。
 */
void    Servo_SetSpeedMode(uint8_t enable, float w_ref_rad_s);
uint8_t Servo_IsSpeedMode(void);
void    Servo_SetSpeedGains(float kp, float ki_ts);
void    Servo_GetSpeedGains(float *kp, float *ki_ts);

/* ---- 给定斜坡（加减速限制）----
 * 把阶跃给定限速成斜坡后再喂 PI。原来直接给阶跃时，进环那一拍 kp*err 就顶到
 * iq 限幅，速度环实际工作在 bang-bang 区（实测超调 30 rpm 31%、3 rpm 68.8%）。
 *
 * acc 单位 rad/s²，默认 400（≈3820 rpm/s，3000 rpm 用 0.79 s 到速）。
 * 设到 20000 等效于关闭斜坡（一拍就跨完），用于对比纯 PI 的阶跃响应。
 * 下限 5：全 0 会让给定永远不动，现象与"参数没生效"无法区分。
 */
void    Servo_SetSpeedAccel(float acc_rad_s2);
float   Servo_GetSpeedAccel(void);
/* 速度给定轨迹：mode 0=梯形 1=S 曲线；j_max rad/s^3（仅 S 曲线用）。
 * 作用于当前手写速度环的给定斜坡，运行中改不跳变。 */
void    Servo_SetSpeedProfile(uint8_t mode, float j_max);
void    Servo_GetSpeedProfile(uint8_t *mode, float *j_max);
/* 轨迹时间优先：给"从这一刻起到目标转速的过渡时间"，由固件反算加速度。
 *   梯形  ：a = Δω / t
 *   S 曲线：a = 2Δω / t，j = 4Δω / t²
 * 只在速度模式（非位置环）生效；t<=0 关闭，回落到 acc/j 配置。
 * 注意这是"到达时间"语义，不是"加速度"语义：改了 t 会覆盖 30019/30089 的 a/j。 */
void    Servo_SetSpeedTrajTime(float t_ramp_s);
float   Servo_GetSpeedTrajTime(void);
void    Servo_GetSpeedTrajActual(float *a, float *j, float *t_left,
                                 uint8_t *flags, uint8_t *mode);
/* 斜坡后的实际给定（模型坐标系 rad/s）。与 Servo_GetSpeedState 的 w_ref 一起看，
 * 才能分辨"给定没变"和"给定变了但斜坡还没走完"。 */
float   Servo_GetSpeedRampRef(void);
/* 速度环出口 iq 限幅（A）。默认 3 A（空载 30 rpm 只要 0.35 A，15 倍余量）；
 * 带载后按需放宽，上限 9 A = 模型设计值，仍远低于 12 A 硬件跳闸。 */
void    Servo_SetSpeedIqMax(float a);
float   Servo_GetSpeedIqMax(void);

/* ---- 位置环（手写，2026-11-17）----
 * 结构：轨迹规划（梯形/S 曲线）→ v_cmd = v_ff + Kp·(p_ref − p) → 现有速度环。
 * 与 SERVO_MODE_POSITION（模型三环）的区别：这条链走手写电流环，因此继承
 * 死区补偿、5/7 次谐波抑制、温度降额、增益调度；模型链只有自己的那套。
 *
 * p_target 用**编码器坐标系**（机械角 rad），内部按 dir_sign 换算到模型坐标系。
 *
 * ⚠ 2026-11-21 起，位置环坐标基准在**每次进入位置模式时对齐到当前单圈角**
 * （0~2π），不再沿上电以来的累积多圈角。`Servo_SetPosMode(1, tgt)` 的 tgt
 * 仍按**对齐前**的坐标解释（内部会自动平移），所以 do_enable 传
 * GetThetaMulti() 是安全的；对上位机则是"先切模式、再读 30100 p_now、再下发"。
 * 详见 servo_isr.c 的 SERVO_POS_COORD_AT_ENTRY。
 *
 * 联锁同速度模式：C 电流零点、Z 电角度、无故障位、不在辨识中。
 */
void    Servo_SetPosMode(uint8_t enable, float p_target_rad);
uint8_t Servo_IsPosMode(void);
void    Servo_SetPosTarget(float p_target_rad);  /* 已在位置环内：只改目标 */
void    Servo_ResetPosCmdLatch(void);           /* 清故障后允许重新下发同一目标 */
void    Servo_SetPosHold(void);                   /* 目标改回当前实际位置 */
void    Servo_GetPosState(float *p_ref, float *p_err, float *v_ff,
                          float *p_tgt, float *vmax, uint8_t *st);
float   Servo_GetPosKp(void);
void    Servo_SetPosKp(float kp);

/* ---- 位置环诊断快照（30100 AppPosStatus 的数据源）----
 * 2026-11-21 新增：位置环的状态机与误差此前**没有任何 MAVLink 出口**，
 * 上位机无法判定"到位 / 追赶中 / 跟随故障"，只能靠 30040 计数变化猜。
 *
 * 坐标口径：
 *   · p_now / p_tgt / p_ref / err 全部在**编码器坐标系**（与 BspServo_GetThetaMulti
 *     同号），单位 rad，多圈。
 *   · p_now 是**位置环实际使用**的角（含编码器零点），与 30090 的 pos 字段
 *     （单圈、含零点）差一个多圈展开量；静止且未跨圈时两者数值相同。
 *   · err = p_ref - p_now，是**规划与实际之差**（跟随误差），不是"目标与实测"。
 *     刚起步时后者天然很大，会误判。
 *
 * 未使能位置模式时各字段保持最后一次的值，st 会回到 POS_ST_IDLE(0)。 */
typedef struct {
    float    p_now;       /* 位置环实际用的机械角，rad（多圈） */
    float    p_tgt;       /* 用户目标，rad（不含补偿） */
    float    p_tgt_c;     /* 规划目标，rad（含补偿；comp_en=0 时等于 p_tgt） */
    float    p_ref;       /* 规划器当前参考，rad */
    float    err;         /* p_ref - p_now，rad */
    float    i_term;      /* 积分项输出，rad/s（Ki=0 时恒为 0） */
    float    travel;      /* 本次定位行程 |目标-起点|，rad */
    float    kp;          /* 生效位置增益，1/s */
    float    vmax;        /* 规划限速，rad/s */
    float    errmax;      /* 跟随误差上限，rad */
    uint32_t fault_cnt;   /* 累计跟随故障次数 */
    uint8_t  st;          /* pos_state_t：0 IDLE / 1 MOVE / 2 HOLD / 3 DONE / 4 ERR */
    uint8_t  settled;     /* 1 = POS_ST_DONE */
    uint8_t  comp_en;     /* 目标补偿开关 */
} ServoPosDiag_t;

void    Servo_GetPosDiag(ServoPosDiag_t *d);

/* 位置环当前坐标（编码器方向）。
 *
 * 2026-11-21 起，位置环坐标在**每次进入位置模式时对齐到当前单圈角**（0~2π），
 * 之后连续。要"以当前位置为目标"时用它，**不要**再用 BspServo_GetThetaMulti() ——
 * 后者是裸多圈角，不含坐标偏置，两者差 s_pos_base，混用会让目标偏出整圈。
 * 与 30100 的 p_now 完全一致。 */
float   Servo_GetPosCoordNow(void);

/* ---- 速度环前馈（加速度 + 摩擦 + 黏滞）----
 * 三项都是"让 PI 少干活"：加速度项把斜坡段的力矩需求提前给足，
 * 摩擦项解决静摩擦起跳（3 rpm 超调 68.8% 的唯一解药）。
 * acc_gain 单位 A/(rad/s^2)，默认 1.72e-3（J/Kt×0.7）；
 * fric_a 单位 A，默认 0.321（Tc/Kt）；visc 单位 A/(rad/s)，默认 2.14e-4。
 * en=0 时三项都不生效（但参数保留）。
 */
void    Servo_SetFeedforward(uint8_t en, float acc_gain, float fric_a, float visc);
void    Servo_GetFeedforward(uint8_t *en, float *acc_gain, float *fric_a, float *visc);
/* 起动助推倍数：静摩擦约是动摩擦的 2 倍，标定只测得到稳态（动摩擦），
 * 所以起动瞬间按 (1 + boost) 倍给摩擦前馈。默认 1.0（= 两倍）。
 * 判据：|w_meas| < 0.3·|w_ramp|，稳态自然不触发。0 = 关闭助推。 */
void    Servo_SetFeedforwardBoost(float boost);
float   Servo_GetFeedforwardBoost(void);

/* ---- 抗饱和方式 ----
 * kb > 0：back-calculation（把超出限幅的量按 kb 回灌积分，退饱和快）
 * kb = 0：条件积分（饱和且误差同向时冻结，老逻辑，保留作回退）
 * 默认 0.30，对应跟踪时间常数 Tt = 500us/0.3 ≈ 1.7 ms。
 */
/* ---- 速度环增益调度（多段 PID 查表）----
 * 按 |w_ramp| 在 7 个段点间线性插值：零速段低 Kp 抑制齿槽抖动，
 * 100~500 rpm 用实测最优 0.583，1000 rpm 以上降 Kp 保电压余量。
 * en=0 时退回 Servo_SetSpeedGains 设置的单组增益（A/B 对比用）。 */
void    Servo_SetSchedEnable(uint8_t en);
uint8_t Servo_GetSchedEnable(void);
/* 最近一次查到的增益（诊断；en=0 时保持上一次的值） */
void    Servo_GetSchedGains(float *kp, float *kits);

void    Servo_SetSpeedKb(float kb);
float   Servo_GetSpeedKb(void);
/* 最近一拍的前馈总量（A），诊断用 */
float   Servo_GetSpeedFF(void);
/* 最近一拍的齿槽前馈量（A）。齿槽前馈按机械角查 128 点表，
 * 只在 |w_ramp| < 52 rad/s（496 rpm）时生效。 */
float   Servo_GetSpeedCog(void);

/* 诊断快照：iq 环输出、积分项、2 kHz 实际执行次数、斜坡后的给定（可传 NULL） */
void    Servo_GetSpeedState(float *w_ref, float *iq_cmd, float *i_term, uint32_t *exec,
                            float *w_ramp);

/* ---- 编码器方向符号（由一个辨识 MotorIdent 给出）----
 * 模型是 `elec = 14*theta_m + π/2`，斜率固定为正；只靠平移零点无法把方向翻过来。
 * 所以当辨识出 dir = -1（相序与编码器反向）时，在**送模型的入口**把 theta/omega
 * 取反，模型看到的就等价于 `-14*theta + π/2`，与真实电角度同号。
 * 默认 +1：未做过辨识时行为与原来完全一致。 */
void   Servo_SetDirSign(int8_t dir);
int8_t Servo_GetDirSign(void);
void  Servo_SetVelProfileMode(uint8_t m);   /* 0=梯形 1=S 曲线（默认 S 曲线） */
void  Servo_SetVelLimits(float a_max, float j_max);  /* rad/s^2, rad/s^3 */
float Servo_GetVelRef(void);                /* 斜坡当前输出 rad/s */
float Servo_GetPosRef(void);                /* 实际送给模型的位置给定 rad */
uint32_t Servo_GetVelClampCount(void);      /* 跟随误差被限幅的拍数（堵转诊断） */

void Servo_GetSample(float *iu, float *iv, float *iw, float *iq_ref, float *duty); /* 最近一拍快照（遥测用） */

/* ---- 辨识/标定参数持久化 ----
 * 把当前 bias / zero / dir / ratio / 速度环增益打包写进 Flash 最后一页。
 * 返回 1 = 成功（含"内容未变化、无需写"），0 = 被拒绝（MOE=1 或未标定/未对齐）或写失败。
 * 自动路径：1 ms 任务每 500 ms 调一次；手动路径：A 命令。 */
uint8_t Servo_ParamSaveNow(void);

#endif /* BSP_SERVO_H */
