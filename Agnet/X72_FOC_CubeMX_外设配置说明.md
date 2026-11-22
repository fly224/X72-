# X72_FOC — STM32CubeMX 外设配置说明（依据 PG_X72_CAN 原理图）

> 生成日期：2026-11-14　|　板卡：`SCH_PG_X72_CAN_2026-11-07` / 网表 `Netlist_PG_X72_CAN_2026-11-07.enet`
> 工程文件：`Firmware/FM_Servo_V3/embedded_project/STM32G474_App/X72_FOC.ioc`

---

## 0. 结论先行

| 项 | 结果 |
|---|---|
| 主控 | **STM32G474CEU6**（U4，UFQFPN48），与 `Mcu.CPN` / `Mcu.Package` 一致 |
| 配置基线 | 复用参考工程 `FM_XC-master/.../FineMotionDriver.ioc`（CubeMX 6.16.0 / FW_G4 V1.6.2），按本板网表逐脚核验后落地 |
| 核验结论 | **49 脚：38 PASS / 2 WARN / 0 FAIL / 9 INFO（电源与系统脚）**，参与判定的 40 脚一致率 **95.0%** |
| 2 个 WARN | PA13/SWDIO、PA14/SWCLK 在 `.ioc` 中未显式配置为 Serial Wire |
| 主要风险 | **OPA3 / ADC3 第三相电流通道在本板上无外部采样网络**（详见 §5 R1） |
| 代码生成 | **已完成**：CubeMX 初始化代码（1196 文件 / 76 MB）与 MDK-ARM 工程已落地，静态核验 105 项 FAIL=0；详见 `Agnet/X72_FOC_代码生成报告.md`（本文 §6 步骤 1 与 §8「未生成代码」的表述以该报告为准） |

**为什么复用而非手写 `.ioc`**：CubeMX 的 `.ioc` 是 MicroXplorer 私有格式（`File.Version=6`、`MxDb.Version=DB.6.0.160`），手工拼装极易被 GUI 判为损坏或静默丢弃配置项。参考工程与本板 36 个复用引脚逐一对应、且经过实机验证，复用 + 逐脚核验是风险最低的路径；仅把 `ProjectName` / `ProjectFileName` 改为 `X72_FOC` 以匹配文件名。

---

## 1. 芯片与时钟树

| 项 | 配置值 | 依据 |
|---|---|---|
| MCU | STM32G474CEU6, UFQFPN48 | 网表 U4 `Manufacturer Part`；`Mcu.CPN` |
| HSE | **8 MHz** 外部晶振，PF0-OSC_IN / PF1-OSC_OUT | 网表 `OSC_IN`/`OSC_OUT` → X2 `X32258MSB4SI`，负载电容 C27/C28 = 20 pF |
| PLL | M=/2 → VCO 输入 4 MHz；N=85 → VCO 340 MHz；SYSCLK = **170 MHz** | `RCC.PLLM/PLLN/PLLRCLKFreq_Value` |
| 总线 | AHB / APB1 / APB2 / Timer / ADC12 / ADC345 / FDCAN 全为 170 MHz | `RCC.*Freq_Value` |
| VDDA | 3.3 V 经 L11 磁珠；U11(RS621XF) + R1/R2 产生 `HALF_VREF` ≈ 1.65 V | 网表 `VCCA`、`HALF_VREF` |
| VREF+ | 外部基准 **REF3033 (U20) 3.3 V**，非 VDDA 直连 | 网表 `VCC3V3_REF` |
| SysTick | 1 ms（NVIC 优先级 15） | `VP_SYS_VS_Systick` |

> `RCC.EXTERNAL_CLOCK_VALUE=12288000` 是外部**时钟源输入**参数残留（如 MCO 输入），本工程 `PLLSourceVirtual=RCC_PLLSOURCE_HSE`，不参与 PLL 计算；但 8 MHz HSE 需与 BOM 实物核对（见 §5 R5）。

---

## 2. 外设配置总表

| 外设 | CubeMX 配置 | 引脚 | 原理图网络 / 器件 | 关键参数与量化核算 |
|---|---|---|---|---|
| **TIM1** 三相 PWM | 中心对齐 1、CH1/2/3 + CH1N/2N/3N、PWM2 模式、OSSR/OSSI 使能、TRGO = OC4REF | PA8/PA9/PA10（HIN1-3）、PB13/PB14/PB15（LIN1-3） | → FD6288Q (U13) → 6× JMSH1004BGQ | `ARR = 4250` → **170 MHz / (2×4250) = 20 kHz**；`DeadTime = 50`；`Pulse(OC4)=1` 用作 ADC 注入触发点 |
| **OPAMP1** | Standalone / HighSpeed | PA1(VINP)、PA3(VINM)、PA2(VOUT) | `OPA1_P/N/OUT`，R28/R29=1 kΩ，R30=30 kΩ，采样电阻 R4 = 1 mΩ（U 相） | 同相增益 ≈ **×30**，偏置来自 `HALF_VREF`；PA2 与 ADC1_IN3 共享 |
| **OPAMP2** | Standalone / HighSpeed | PA7(VINP)、PA5(VINM)、PA6(VOUT) | `OPA2_P/N/OUT`，R32/R33=1 kΩ，R34=30 kΩ，采样电阻 R3 = 1 mΩ（**W 相**） | 同相增益 ≈ ×30；PA6 与 ADC2_IN3 共享 |
| **OPAMP3** | Standalone / HighSpeed | PB0(VINP)、PB2(VINM)、PB1(VOUT) | `OPA3_P/N/OUT` —— **网表中仅 U4 三个引脚，无任何外部元件** | 见 §5 R1 |
| **ADC1** | 注入：1 通道 `IN3`(PA2)，TIM1_TRGO 上升沿；常规 4 通道 + DMA 循环 | PA2 / PA0 / PB11 / PB12 | `VBUS_ADC`(R66 分压)、内部温度、`PB11_ADC1`、`PB12_ADC2` | 注入采样 2.5 cycles；常规 IN1=母线电压、IN14、IN11；DMA1_Ch3 半字循环 |
| **ADC2** | 注入：1 通道 `IN3`(PA6)，TIM1_TRGO；常规 2 通道 + DMA | PA6 / PA4 / PC4 | `NTC_GAN`(R55/NTC)、`NTC_MOTOR`(R54/NTC) | 常规 IN17、IN5，24.5 cycles；DMA1_Ch4 |
| **ADC3** | 注入：1 通道 `IN1`(PB1)，TIM1_TRGO；常规 VREFINT | PB1 | `OPA3_OUT` | DMA1_Ch7；读数有效性见 §5 R1 |
| **SPI1** | Full-Duplex Master, 8 bit, **Mode 3**, 预分频 16 | PB3(SCK)、PB4(MISO)、PB5(MOSI)；PC11 = 软件 CS | → MT6835GT-STD (U2) CSN/MISO/MOSI/SCK | **170 MHz / 16 = 10.625 Mbit/s**；MT6835 手册 7.6.1 明确「SPI 使用模式 3（CPOL=1, CPHA=1）」，最高 16 MHz → 配置合规；PB3/PB4 上拉、三脚 Very High 速率 |
| **USART1** | Asynchronous, **921600 baud** | PB6(TX)、PB7(RX) | → THVD1426DRLR (U5) RS485 / CN8 调试口 | DMA1_Ch5(RX)/Ch6(TX)，低优先级；与 README 的 MAVLink 调试通道、上位机 COM27@921600 一致 |
| **FDCAN1** | FD 无比特率切换（FD_NO_BRS），5 标准 + 5 扩展滤波 | PA11(RX)、PA12(TX) | → SIT1042ATK/3 (U15)，CANH/CANL 经共模电感 L12 | Prescaler 17 → TQ = 100 ns，1+Seg1(7)+Seg2(2) = 10 TQ → **1 Mbps**；FIFO0 = CANopen，FIFO1 = MAVCAN |
| **TIM2** | PWM CH1，`ARR = 8500-1` | PA15（Label `BREAK_PWM`） | 网表 `PA15`（MCU 引脚，无标注外部负载） | 170 MHz / 8500 = **20 kHz**，与 TIM1 同频，用于刹车/抱闸控制 |
| **TIM5** | 内部时钟，`ARR = 34000-1` | 无输出 | — | 170 MHz / 34000 = **5 kHz 位置环**（README 一致） |
| **TIM6 / TIM16** | PSC = 169，ARR = 999 | 无输出 | — | 170 MHz / 170 / 1000 = **1 kHz / 1 ms 非实时任务** |
| **GPIO** | 输出 | PC13/PC14/PC15（LED_R/G/B，初始 SET）、PC11（SPI1_CS）、PC10（预留）、PB9（CAL_EN，下拉） | LED2 / MT6835 CSN、CAL_EN | PB9 → MT6835 自动校准使能 |
| **DMA** | 7 路请求 | DMA1 Ch1/2 SPI1 TX/RX、Ch3/4/7 ADC1/2/3、Ch5/6 USART1 RX/TX | — | ADC 为循环半字；SPI/USART 为字节单次 |
| **NVIC** | PriorityGroup 4 | — | — | ADC1_2=**0**（电流环最高）、DMA1_1/2=1、TIM6=4、FDCAN1_IT0/1 与 TIM1_UP_TIM16=5、USART1=8、TIM5=9、SysTick=15 |

---

## 3. 实时时序链（与 README 架构对齐）

```
TIM1 中心对齐 20 kHz ── OC4REF (Pulse=1) ── TRGO 上升沿 ──┬─> ADC1 注入 (PA2, U 相)
                                                        ├─> ADC2 注入 (PA6, W 相)
                                                        └─> ADC3 注入 (PB1, 悬空/见 R1)
        └─> ADC1_2_IRQn (优先级 0) ─> 电流环 FOC 20 kHz
TIM5  ─> 5 kHz 位置环
TIM6 / TIM16 ─> 1 ms 非实时任务（参数 FlashDB、通讯扫描）
```

PWM 中心对齐 + OC4 在计数起点触发注入，可避开上下管开关瞬间的采样尖峰；`OffStateRunMode/OffStateIDLEMode = ENABLE` 保证刹车时六路输出进入安全状态。

---

## 4. 一致性核验（可复现）

```bash
cd G:/FOC/X72
python Hardware/Netlist/parse_enet.py --dump-csv --report   # 网表 → pin_map.csv / net_report.md
python Hardware/Netlist/verify_ioc.py --csv                 # .ioc 与网表逐脚比对 → verify_ioc.csv
```

核验结果：`核验脚数=49  PASS=38  WARN=2  FAIL=0  INFO=9`，一致率 `38/40 = 95.0%`。
判定表的期望值写在 `verify_ioc.py` 的 `EXPECTED_SIGNAL` 显式常量中（依据：网表网络名 + STM32G474 复用功能表 + MT6835 手册），硬件改版需同步更新该表。

**三方交叉验证**：① `.enet` 网表（186 元件 / 106 网络）给出物理连接；② `mt6835.txt` 手册给出 SPI Mode 3 / 16 MHz 上限；③ 参考工程 `.ioc` 给出量产参数。三者一致处才判 PASS。

---

## 5. 风险与待确认项（逐条裁定）

| 编号 | 级别 | 问题 | 证据 | 建议 |
|---|---|---|---|---|
| **R1** | **Major** | **第三相电流通道（OPAMP3 / ADC3）在板上无采样网络** | 网表 `OPA3_P`/`OPA3_N`/`OPA3_OUT` 三个网络**只有 U4 的 PB0/PB1/PB2**；1 mΩ 采样电阻只有 R4(U 相 `U_GND`) 与 R3(W 相 `W_GND`)，**无 `V_GND` 网络**；`HALF_VREF` 仅接 OPA1_P(R31)/OPA2_P(R35) | 本板是 **双电阻采样（U、W）**，第三相须由 `Iv = -(Iu + Iw)` 重构。请确认固件未把 ADC3 注入值当作真实 V 相电流参与 Clarke 变换；否则禁用 OPAMP3/ADC3 注入或把 ADC3 改作母线/温度监控 |
| **R2** | Major | SWD 未在 `.ioc` 显式启用（2 个 WARN） | `.ioc` 中无 `PA13.Signal` / `PA14.Signal` / `SYS_JTMS-SWDIO`；网表 SWDIO/SWCLK 确实接到 CN8 pin4/pin3 且 10 kΩ 上拉 | G4 复位后 PA13/PA14 默认即 SWD 功能、实机可调试，但建议在 GUI 中 `SYS → Debug = Serial Wire` 显式锁定（§6 步骤 3），避免后续代码重映射引脚后失联 |
| **R3** | Minor | PB11 / PB12 悬空却参与 ADC1 常规 DMA 采样 | 网表 `PB11_ADC1`、`PB12_ADC2` 均只有 U4 引脚；`.ioc` 把它们配成 ADC1_IN14 / ADC1_IN11 并列入常规转换 Rank3/4 | 悬空 ADC 输入读数不确定；确认软件未用于保护/判据，必要时在 CubeMX 中从常规序列移除 |
| **R4** | Minor | 网表命名与实际功能不符（命名质量门） | `PB12_ADC2` 实为 `ADC1_IN11`；`CAN1_RX/TX` 实为 `FDCAN1`；`OPA2_*` 采样的是 **W 相**而非 V 相；`PA15` 无功能标注（实为 BREAK_PWM） | 下一版原理图统一命名，避免 CubeMX 配置与固件注释误判 |
| **R5** | Minor | HSE 频率需与 BOM 复核 | `.ioc` 按 8 MHz（4 MHz×85=340 MHz→170 MHz SYSCLK）；网表晶振 `X32258MSB4SI` + 20 pF 通常为 8 MHz，但 `.ioc` 残留 `EXTERNAL_CLOCK_VALUE=12288000` | 上电用 MCO/示波器量 SYSCLK；若为 12.288 MHz 需重算 PLL 与全部波特率/PWM 频率 |
| **R6** | Minor | `DeadTime=50` 未与功率级联合标定 | FD6288Q + JMSH1004BGQ，`.ioc` 用户常量 `TIM_1_8_PWM_DT=0.000050`、`TIM_1_8_DEADTIME_CLOCKS=80` 与寄存器值 50 单位不一致 | 用双通道示波器测 HO/LO 交叠，按 MOSFET 开关时间与预驱传播延迟标定；勿直接沿用常量 |
| **R7** | Info | NRST 复位电容 C45 = 10 µF | 网表 `NRST` → C45 `CL05A106MQ5NUNC`(10 µF) + R62 10 kΩ 上拉 | 常见值为 100 nF；10 µF 会显著拉长上电复位释放时间，确认与 Bootloader/看门狗时序匹配 |
| **R8** | Info | BOOT0 10 kΩ 下拉、VREF+ 外部 REF3033 | `BOOT0` → R58→GND；`VCC3V3_REF` → U20(REF3033) | 配置正确，无需改动 |

---

## 6. 在 STM32CubeMX 中完成落地的操作序列

1. 安装 **STM32CubeMX ≥ 6.16.0** 与固件包 **STM32Cube FW_G4 V1.6.2**（本机当前未找到可执行文件，故本次未自动运行）。
2. `File → Open Project` 选择 `Firmware/FM_Servo_V3/embedded_project/STM32G474_App/X72_FOC.ioc` —— **不要新建工程**，否则会丢失全部引脚锁定与 IP 参数。
3. `Pinout & Configuration → System Core → SYS`：`Debug = Serial Wire`（补齐 R2 的 2 个 WARN）；`Disable Dead Battery Signals` 保持。
4. `Clock Configuration`：核对 HSE = 8 MHz → PLL → SYSCLK 170 MHz（R5）。
5. 按 §5 裁定结果处理 R1 / R3（例如禁用 OPAMP3 与 ADC3 注入通道、从 ADC1 常规序列移除悬空通道）。
6. `Project Manager`：Toolchain = `MDK-ARM V5.32`；`Heap/Stack = 0x800`；勾选 `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`；`Keep User Code when re-generating` 已开启。
7. 点击 **GENERATE CODE**，产出 `Core/Inc`、`Core/Src`、`Drivers`、`MDK-ARM/X72_FOC.uvprojx`。
8. 生成后再次运行 `python Hardware/Netlist/verify_ioc.py --csv` 确认引脚未被 CubeMX 重排。

> 生成代码后请按 README 的 `AGENTS.md` 约定：只在 `USER CODE BEGIN/END` 区修改，`simulink_code_gen/` 禁手改。

---

## 7. 本次交付文件

| 文件 | 内容 |
|---|---|
| `Firmware/FM_Servo_V3/embedded_project/STM32G474_App/X72_FOC.ioc` | CubeMX 工程（610 行，仅改 ProjectName/ProjectFileName） |
| `Agnet/X72_FOC_CubeMX_外设配置说明.md` | 本文档 |
| `Hardware/Netlist/parse_enet.py` | 网表解析（元件、引脚-网络映射） |
| `Hardware/Netlist/verify_ioc.py` | `.ioc` ↔ 网表自动核验，含显式期望表 |
| `Hardware/Netlist/pin_map.csv` | 516 条引脚记录（UTF-8 BOM） |
| `Hardware/Netlist/net_report.md` | 106 个网络的全部连接点（UTF-8 BOM） |
| `Hardware/Netlist/verify_ioc.csv` | 49 脚逐脚判定表（UTF-8 BOM） |

---

## 8. 明确局限

- `Hardware/` 下 4 份随机命名的 PDF（数据手册）**未解析**：本机无 PDF 文本提取库/`pdftotext`；`SCH_PG_X72_CAN_2026-11-07.pdf` 亦未做图形化比对。本文所有引脚与外设结论来自 **`.enet` 网表 + MT6835 文本手册 + 参考工程 `.ioc`** 三方交叉验证，未用 PDF 佐证标注文字。
- **未实际运行 STM32CubeMX、未生成初始化 C 代码**；`.ioc` 的 GUI 兼容性与生成结果需在 CubeMX 中确认。
- 死区时间、ADC 注入采样时刻、CAN 终端匹配、运放偏置精度等均需**示波器/实机测量**，本文只做配置层核验。
- R1、R3 属于硬件与固件语义一致性问题，需由固件负责人结合 `bsp_api.c` / 电流环代码最终裁定，本文仅给出网表层面的证据。
