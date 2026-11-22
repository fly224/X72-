# X72_FOC 代码生成报告（Toolchain / IDE = MDK-ARM）

> 生成日期：2026-11-14　|　工程：`Firmware/FM_Servo_V3/embedded_project/STM32G474_App/`
> 关联：`X72_FOC.ioc`、`X72_FOC_CubeMX_外设配置说明.md`

---

## 0. 结论先行

| 项 | 结果 |
|---|---|
| 交付形态 | **可独立编译的 CubeMX 初始化工程**：`Core/Inc` + `Core/Src` + `Drivers`（CMSIS + HAL）+ `MDK-ARM/X72_FOC.uvprojx` |
| 规模 | **1196 个文件 / 76 MB**（Core/Inc 11、Core/Src 12、CMSIS 978、HAL 183、MDK-ARM 10、`.ioc` + `.mxproject`） |
| 工具链 | **MDK-ARM**（与截图下拉框、`ProjectManager.TargetToolchain=MDK-ARM V5.32` 一致） |
| 静态核验 | 工程源文件引用 **38** 个 + 本地 `#include` **67** 处 = **105 项，FAIL = 0**（见 `verify_generated_code.csv`） |
| 参数一致性 | 生成代码中的 TIM1 / SPI1 / FDCAN1 / USART1 / PLL 数值与 `.ioc` 逐项一致（§3） |
| 未执行 | 本机 **未实际运行 CubeMX GENERATE CODE**，也**未做 Keil 实机编译**（原因见 §1、§7） |

**代码来源**：不是手写、也不是本机 CubeMX 6.9.1 重新生成，而是**与 `X72_FOC.ioc` 同源、由 CubeMX 6.16.0 已生成的初始化代码**（参考工程 `FineMotionDriver`），按本工作区适配后落地。文件头均保留 STMicroelectronics / CubeMX 生成标记。

---

## 1. 为什么没有直接执行 CubeMX 的 GENERATE CODE

| 阻碍 | 证据 |
|---|---|
| 版本倒挂 | 本机 `D:\STM32CubeMX\STM32CubeMX.exe` = **6.9.1**（注册表 / `.installationinformation` 的 `APP_VER=6.9.1`）；`X72_FOC.ioc` 的 `MxCube.Version=6.16.0`。旧版打开新版配置会触发迁移，可能丢失 6.16 新增字段 |
| 固件包缺失 | `.ioc` 指定 `FirmwarePackage=STM32Cube FW_G4 V1.6.2`；本机 `C:\Users\92714\STM32Cube\Repository` 只有 **V1.3.0 / V1.5.1 / V1.5.2**，无 V1.6.2 → 生成会降级到旧 HAL 或要求联网登录 myST |
| 数据库差异 | 本机 `db/package.xml` = `DB.6.0.181`，`.ioc` 记录 `DB.6.0.160`；数据库较新，但固件包不匹配才是决定性问题 |

因此采用「同源生成代码 + 本地适配」路径，避免用错误 HAL 版本生成出与 `.ioc` 不符的初始化代码。若后续补齐 V1.6.2，可按 §6 在本机重新生成。

---

## 2. 生成代码清单与工程适配

### 2.1 落地内容

| 目录 | 文件数 | 内容 |
|---|---:|---|
| `Core/Inc` | 11 | `main.h`、`adc/dma/fdcan/gpio/opamp/spi/tim/usart.h`、`stm32g4xx_hal_conf.h`、`stm32g4xx_it.h`；`main.h` 内含 `TIM_1_8_PERIOD_CLOCKS=4250` 等 UserConstants |
| `Core/Src` | 12 | 各外设 `MX_*_Init()`、`stm32g4xx_hal_msp.c`、`stm32g4xx_it.c`、`system_stm32g4xx.c`、`main.c` |
| `Drivers/CMSIS` | 978 | CMSIS 5 内核 + STM32G4xx Device 头文件、启动模板 |
| `Drivers/STM32G4xx_HAL_Driver` | 183 | HAL/LL 驱动源码与头文件（对应 FW_G4 V1.6.2） |
| `MDK-ARM` | 10 | `X72_FOC.uvprojx` / `.uvoptx`、`startup_stm32g474xx.s`、`stm32g474xx_flash.sct`、`DebugConfig`、`RTE` |

### 2.2 为可独立编译所做的 4 项适配

参考工程的 `.uvprojx` 里除 CubeMX 文件外还手工加入了 **99 个应用层源文件**（`app/`、`app/Identify/`、`motor_control/`、`communication/`、`database/`、`board_public/flash_db/`、`common/`、`simulink_code_gen/`、`Board/`），而本工作区这些架构目录目前为空，直接打开会报大量文件缺失。

| 适配 | 处理 | 影响 |
|---|---|---|
| 源文件组 | 移除 99 个应用层引用，保留 **38 个 CubeMX 源文件** | 工程变为纯净初始化工程 |
| IncludePath | 从 30+ 条（含 `../../../app`、`../Board` 等）精简为 5 条 CubeMX 默认路径 | 后续接入应用层时需重新添加 |
| 工程名 | `FineMotionDriver` → `X72_FOC`（`.uvprojx`、`.uvoptx`、`RTE/*`、RTE 目录名） | 输出文件 `X72_FOC.axf/hex` |
| `main.c` USER CODE | 清空 3 个应用层 `#include` 与 9 行 `Register*()` / `HalInit()` / `BspInit()` 调用，**保留 `USER CODE BEGIN/END` 标记**并写入中文占位注释 | CubeMX 重新生成时不丢结构，后续可恢复 |

被清空的原始 USER CODE 内容（接入应用层时恢复）：

```c
#include "board.h"
#include "app_scheduler.h"
#include "app_status_check.h"

RegisterSetAppCallback(set_app);
RegisterCheckErrorCallback(app_get_check_error_val);
RegisterCheckWarningCallback(app_get_check_warning_val);
RegisterCheckStatusCallback(app_get_check_status_val);
RegisterCheckIoInputsStatusCallback(app_get_check_di_io_val);
RegisterSendCallback(mavlink_send_data);
HalInit();
BspInit();
```

适配与核验脚本：`Agnet/prepare_mdk_project.py`（`--verify` 仅核验，不改文件），可反复运行。

---

## 3. 生成代码 ↔ `.ioc` 参数一致性（逐项核对）

| 外设 | `.ioc` | 生成代码 | 判定 |
|---|---|---|---|
| TIM1 | `Period=4250`、中心对齐1、PWM2、DeadTime=50、TRGO=OC4REF | `tim.c: htim1.Init.Period=4250` / `TIM_COUNTERMODE_CENTERALIGNED1` / `TIM_OCMODE_PWM2` / `DeadTime=50` / `TIM_TRGO_OC4REF`，OC4 `Pulse=1` | ✅ |
| SPI1 | 分频16、CPOL=High、CPHA=2Edge、8bit | `spi.c: SPI_BAUDRATEPRESCALER_16` / `SPI_POLARITY_HIGH` / `SPI_PHASE_2EDGE` / `SPI_DATASIZE_8BIT` | ✅ |
| FDCAN1 | Prescaler17、Seg1=7、Seg2=2、FD_NO_BRS、5 滤波 | `fdcan.c: NominalPrescaler=17`、`NominalTimeSeg1=7`、`NominalTimeSeg2=2`、`FDCAN_FRAME_FD_NO_BRS`、`StdFiltersNbr=5` | ✅ |
| USART1 | 921600 | `usart.c: huart1.Init.BaudRate=921600` | ✅ |
| RCC | HSE→PLL M/2、N=85、SYSCLK 170 MHz | `main.c: RCC_PLLM_DIV2` / `PLLN=85` / `RCC_PLLR_DIV2` / `FLASH_LATENCY_4` | ✅ |
| 时钟推导 | 170 MHz / (2×4250) = 20 kHz | TIM1 中心对齐 → 20 kHz 电流环 | ✅ |

静态核验：38 个 `<FilePath>` 全部存在；Core 下 67 处 `#include "..."` 全部可在 `Core/Inc`、`Drivers/**/Inc` 中解析；**105 项判定 FAIL=0**。

---

## 4. 后续接入四层源码（下一步建议）

1. 把参考工程 `FM_Servo_V3/` 下的 `app/`、`motor_control/`、`communication/`、`database/`、`board_public/`、`common/`、`simulink_code_gen/` 以及 `embedded_project/STM32G474_App/Board/` 复制到本工作区同名目录（上一轮已建好空骨架）。
2. 在 Keil 中重新添加那 99 个源文件组与 IncludePath（或直接用参考工程的 `.uvprojx` 作为文件组清单）。
3. 恢复 §2.2 中的 USER CODE 内容。
4. 按 `AGENTS.md` 约定：只改 `USER CODE BEGIN/END` 区，`simulink_code_gen/` 禁手改。
5. 按 `X72_FOC_CubeMX_外设配置说明.md` §5 裁定 R1（OPA3/ADC3 第三相悬空）后再接入电流环。

---

## 5. 本机重新生成代码的两种方式

**前置条件**：CubeMX ≥ 6.16.0 且安装 `STM32Cube FW_G4 V1.6.2`（本机 6.9.1 + 缺包，不满足）。

- **GUI**：打开 `X72_FOC.ioc` → `Project Manager` 确认 Toolchain/IDE = MDK-ARM → `GENERATE CODE`。注意 CubeMX 会覆盖 `.uvprojx`，请先备份本报告 §2.2 的适配结果。
- **命令行（脚本已备好）**：
  ```bat
  cd /d D:\STM32CubeMX
  jre\bin\java -jar STM32CubeMX.exe -q "G:\FOC\X72\Agnet\cubemx_generate.txt"
  ```
  `Agnet/cubemx_generate.txt` 内容为 `config load` → `generate code` → `exit`。`generate code` **只更新 `Core/` 与 `Drivers/`，不覆盖 MDK-ARM 工程文件**，因此不会破坏本次裁剪结果；若需连工程文件一起重生成，把该行换成 `project generate` 并先备份。

---

## 6. 明确局限

- **未运行 CubeMX GENERATE CODE**：代码来自与 `.ioc` 同源的 6.16.0 生成结果，不是本机 6.9.1 的产物（§1 给出版本与固件包证据）。
- **未做 Keil 实机编译**：本机注册表有 Keil μVision 5.43a（`E:\keil5c51\51\ARM`），但未定位到可执行的 `UV4.exe`，且未验证 license。静态核验 105 项 FAIL=0 **不等于编译通过**；首次编译可能还需处理器件包 `Keil.STM32G4xx_DFP`、启动文件与分散加载脚本。
- 生成代码对应 **FW_G4 V1.6.2**；若本机只有 V1.5.x，请勿混用不同版本 HAL。
- `X72_FOC_CubeMX_外设配置说明.md` §5 的 R1~R8 硬件风险（尤其 OPA3/ADC3 第三相悬空、SWD 未显式启用）仍然有效，代码生成不改变硬件事实。
- `main.c` 的 USER CODE 已清空，因此当前工程**只做外设初始化、不跑 FOC**；`while(1)` 为空循环。

---

## 7. 本轮新增/修改文件

| 文件 | 说明 |
|---|---|
| `Firmware/.../STM32G474_App/Core/`、`Drivers/`、`MDK-ARM/`、`.mxproject` | CubeMX 生成的初始化代码与 MDK-ARM 工程（1196 文件） |
| `Agnet/X72_FOC_代码生成报告.md` | 本报告 |
| `Agnet/prepare_mdk_project.py` | 工程适配 + 静态核验脚本 |
| `Agnet/verify_generated_code.csv` | 105 项判定明细（UTF-8 BOM） |
| `Agnet/cubemx_generate.txt` | CubeMX 静默生成脚本（补齐 V1.6.2 后可用） |
| `Agnet/X72_FOC_CubeMX_外设配置说明.md` | §0 状态已更新为「代码生成已完成」 |
