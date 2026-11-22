# ADC 注入 ISR + BSP 驱动交付说明

> 2026-11-14　|　工程：`Firmware/FM_Servo_V3/embedded_project/STM32G474_App`
> 依据：`X72_FOC.ioc`、网表 `Netlist_PG_X72_CAN_2026-11-07.enet`、MT6835 手册 7.6.9、`three_loop_deploy.h`

---

## 0. 结论先行

本轮把上一轮 NO-GO 判定中的**第 4 项（ADC 注入 ISR 未接线）与第 7 项（BSP 无实现）落地**，同时把三环控制律加入 Keil 工程并接上 20 kHz 电流环。

| 项 | 状态 |
|---|---|
| BSP 驱动（电流/母线/编码器/PWM/保护） | ✅ 已实现 |
| ADC1/2 注入完成 → `step()` → TIM1 CCR 链路 | ✅ 已接通 |
| 双电阻相序（Critical） | ✅ 已按 `ia=Iu`、`ib=-(Iu+Iw)` 重构 V 相 |
| 三环控制律加入工程 | ✅ `three_loop_deploy.c` 已加入，工程源文件 38 → 41 |
| host GCC 语法检查 | ✅ `bsp_servo.c` / `servo_isr.c` / `main.c` 均退出码 0 |
| Keil ARM 编译 / Flash 烧录 | ✅ 21:25 完成：0 错误 0 警告、Verify OK（详见《编译烧录记录_20261114.md》） |

固件已于 21:25 烧入并复位运行。**上电全程封锁 PWM**（`BDTR.MOE` 清零），计数器与 TRGO 继续运行以便诊断，但六路输出无驱动。必须显式调用 `BspServo_PwmEnable(1)` 且 `Servo_SetEnable(1)` 才可能输出。

---

## 1. 新增/修改文件

| 文件 | 内容 |
|---|---|
| `Board/bsp_servo.h` | 硬件标定宏、故障位、BSP 与控制接线 API |
| `Board/bsp_servo.c` | 运放/ADC/PWM/母线/MT6835 DMA/故障封锁；`HAL_SPI_TxRxCpltCallback`、`HAL_SPI_ErrorCallback` |
| `Board/servo_isr.c` | `HAL_ADCEx_InjectedConvCpltCallback`（20 kHz 控制节拍）、`HAL_TIM_PeriodElapsedCallback`（TIM6 1 ms）、`Servo_Init()` |
| `Core/Src/main.c` | 仅改两处 `USER CODE`：Includes 加 `#include "bsp_servo.h"`，USER CODE 2 调 `Servo_Init()` |
| `MDK-ARM/X72_FOC.uvprojx` | 新文件组 `Application/User/Servo`（3 个 .c）；IncludePath 加 `../Board;../../../simulink_code_gen` |
| `Agnet/add_servo_sources.py` | 幂等工程接入脚本，`--verify` 只核验 |

**`stm32g4xx_it.c` 未改动**：控制逻辑通过覆盖 HAL 弱函数回调接入，CubeMX 重新生成不会丢失接线。已核验 4 个回调在 HAL 库中均为 `__weak` 且签名一致。

---

## 2. 中断与时序链

```
TIM1 中心对齐 ARR=4250，OC4REF(Pulse=1) -> TRGO
  -> ADC1 注入 IN3(PA2/U相) + ADC2 注入 IN3(PA6/W相) 同时触发
  -> ADC1_2_IRQHandler (NVIC 优先级 0) -> HAL_ADC_IRQHandler(&hadc1/2)
  -> HAL_ADCEx_InjectedConvCpltCallback：ADC1 置 bit0、ADC2 置 bit1
  -> 两路齐全才执行 Servo_ControlTick()（保证两路电流都新鲜）
      读 JDR1 -> 电流标定 -> 双电阻重构 -> 填 ExtU -> step() -> 写 CCR1/2/3
      -> 发起下一帧 MT6835 SPI DMA（异步，不等待）
  -> DMA1_Ch2 完成中断 (优先级 1) -> HAL_SPI_TxRxCpltCallback 更新角度缓存
TIM6 (1 kHz, 优先级 4) -> HAL_TIM_PeriodElapsedCallback -> Servo_SlowTask1ms()
```

不使用 `TIM1_UP_TIM16_IRQn`：中心对齐下 UIF 上溢/下溢各一次可能形成 40 kHz，且与 TIM16 共享向量；更要紧的是更新中断到来时注入转换未必完成。

**双 ADC 计数机制**：两个注入完成回调都置位后才跑控制律；1 ms 任务连续 3 ms 未见节拍则置 `BSP_FAULT_ADC_SYNC` 并封锁输出，避免 ADC2 未启动导致控制环静默停摆。

---

## 3. 标定参数（依据网表 / 数据手册）

| 参数 | 值 | 依据 |
|---|---|---|
| PWM ARR | 4250 | `tim.c: htim1.Init.Period = 4250`，170 MHz/(2×4250)=20 kHz |
| 电流换算 | `I=(raw/4095×3.3−1.65)×33.333` A | 1 mΩ 分流 + 30× 同相放大 + 1.65 V 偏置；12 bit/3.3 V 约 0.0269 A/LSB，满量程 ±55 A |
| 母线分压比 | 1/201 | VM→R64(100k)→R63(100k)→R65(1k)→GND，D12 钳位 3V3；48 V→0.2388 V |
| 编码器 | 2^21 = 2 097 152 | MT6835 21 bit，连续读命令 `0xA0 0x03`（手册 7.6.9），CSN 下降沿锁存 0x003~0x006 |
| 角度帧 | 6 B：2 B 命令 + ANGLE[20:13]/[12:5]/[4:0]+STATUS/CRC | `rx[2]<<13 \| rx[3]<<5 \| rx[4]>>3` |
| 速度估算 | 相邻 50 µs 角度差分 + 一阶低通 α=0.06（约 190 Hz） | 含单圈回绕处理 |
| 硬件过流阈值 | 12 A | 先于生成代码 9 A 软件限幅 |
| SPI | Mode 3，10.625 MHz | `.ioc` CPOL=High/CPHA=2Edge/PSC=16；MT6835 上限 16 MHz |

---

## 4. 关键设计决策

1. **双电阻相序**：网表 OPA1=U 相、OPA2=W 相、OPA3/ADC3 悬空。生成代码 Clarke 只用 `ia`、`ib`（`U.ic` 实测 0 次引用），故 `U.ia=Iu`、`U.ib=-(Iu+Iw)` 重构 V 相，`U.ic=Iw` 仅作监视。**直接把 W 相当 `ib` 会造成 120° 错相、FOC 失效**。
2. **编码器不阻塞 ISR**：MT6835 只接了 SPI（ABZ/UVW 未引出），20 kHz ISR 内轮询 `HAL_SPI_TransmitReceive` 会占约 5 µs 且违反 ISR 最小化。改为每拍末尾启动 DMA、完成中断更新缓存，ISR 只读上一帧 `s_enc_raw`。SPI1 仅由该节拍与 DMA 完成中断使用，主循环不得调 `HAL_SPI_*`。
3. **上电封锁**：`BspServo_Init()` 最后清 `BDTR.MOE`；故障时 `FaultSet()` 立即清 MOE。计数器与 TRGO 继续运行，控制律仍执行但无输出。
4. **确定性**：角度归一化用条件减法（零点偏置在设置时归一化），ISR 内不调用 `fmodf`；所有缓冲静态分配，无 `malloc`；所有 HAL 启动调用检查返回值，失败置 `BSP_FAULT_BSP_INIT` 并保持封锁。
5. **DWT 实测钩子**：`BspServo_DwtInit()` 使能 `DWT->CYCCNT`，每拍记录 `Servo_GetMaxStepCycles()`，用于验证 `double` 版 `step()`（sin/cos/sqrt 走软件库）在 50 µs/8500 周期内是否留有余量。

---

## 5. 使能与安全流程

```c
Servo_Init();                         /* main() USER CODE 2 已完成 */
Servo_SetEncoderZero(zero_rad);       /* 必须标定：机械零点与电零点对齐 */
Servo_SetPosRef(0.0f);
BspServo_ClearFault();
BspServo_PwmEnable(1);                /* 开 MOE */
Servo_SetEnable(1);                   /* 开控制律使能 */
```

⚠️ 首次联调必须**低压限流、电机空载**；先确认 `en=0` 时 `duty=0.5` 且 MOE 封锁，再逐步开环 → 电流闭环 → 三环。辨识类操作禁止带载或带减速器。任何故障位非零时 `BspServo_PwmEnable(1)` 会被拒绝。

---

## 6. 本轮验证结果

| 项 | 结果 |
|---|---|
| host GCC `-fsyntax-only -std=c99 -DSTM32G474xx` | `bsp_servo.c` / `servo_isr.c` / `main.c` 均退出码 0（仅 CMSIS 在 x86 上的指针宽度警告，与业务代码无关） |
| HAL 回调弱定义签名 | 4/4 一致（`HAL_ADCEx_InjectedConvCpltCallback`、`HAL_SPI_TxRxCpltCallback`、`HAL_SPI_ErrorCallback`、`HAL_TIM_PeriodElapsedCallback`） |
| 工程引用 | 41 个 `<FilePath>`，文件缺失 0；`../Board` 与 `../../../simulink_code_gen` 存在且已入 IncludePath |
| `main.c` | 仅两处 `USER CODE` 区被修改，其余生成代码未动 |
| `stm32g4xx_it.c` | 未改动 |

---

## 7. 待办与明确局限

- **已用 Keil AC6 全量重建并烧录（21:25，0 错误 0 警告、Verify OK）**，但未做 SWD 运行态读回与示波器时序验证；`BspServo_Init()` 里常规 DMA 与注入触发并存、双 ADC 注入同步、MT6835 DMA 角度帧均需在板子上确认。
- **母线电压**取自 ADC1 常规 DMA Rank1（PA0），分压比按网表电阻值解析，**未经万用表/ADC 实测校准**；D12 钳位与 C1/C2 瞬态未建模。
- **编码器零点偏置、单圈跨圈累积、CRC 校验（rx[5]）**尚未实现；MT6835 STATUS 异常累计 5 次才置故障，弱磁/高速工况需实机评估。
- **`double` 版 `step()` 性能未知**：需上板后用 `Servo_GetMaxStepCycles()` 读 DWT 周期数；超预算需把模型改 `single` 重新 codegen。
- **常规 DMA 与注入触发并存**、ADC1/2 同触发的完成先后，仅按 HAL/.ioc 静态推理，需示波器或实机验证。
- `pos_ref` / `en` 目前是 `servo_isr.c` 里的静态变量，**固件协议层仍为空**：上位机还不能真正下发指令，需按《固件协议层_职责与编写指南》接入 database 后才有完整控制链。
- 参数（电流标定、过流阈值、速度低通系数）集中在 `bsp_servo.h`，实机标定后应改源头而非散落到业务代码。
