# STM32G474 三环 FOC 控制律 · ERT 代码生成交付（2026-11-15 修订）

从三环闭环仿真模型中提取控制器，用 **Embedded Coder / ERT (`ert.tlc`)** 生成 ANSI C，目标硬件
**STM32G474CEU6（Cortex-M4F @170 MHz，PG_X72_CAN 板）**。目录：
`G:\FOC\X72\Firmware\FM_Servo_V3\simulink_code_gen`。

> **2026-11-15 18:00 更新**：按用户要求把电流环切回设计值 **20 kHz**（`Ts_i=50e-6`）。
> `dt=5e-5`、`N_w=10`、`N_p=20`、`gain_comp=0.00105` 全部自动派生，速度环 2 kHz / 位置环 1 kHz 不变，
> PID 默认值未改。详见 `sim_check\results\仿真效果报告.html`。
> ⚠ 实机要真跑 20 kHz 需要单精度（double 版 9282 cyc > 50 µs 会掉回 10 kHz）。
> **2026-11-15 18:40 已完成 single 化**：`real32_T` + `sinf/cosf/sqrtf`，静态 RAM 392→232 B；
> 详见 `单精度化说明.md`。烧录前仍需 `DWT->CYCCNT` 复测单拍周期。
> **2026-11-15 20:38 最终收口生成**：消除 `gain_comp` 单精度量化诊断、补齐
> `integration_example.c:140` 缺失的右括号（此前从未参与 host 编译故未暴露）、
> `fabs`→`fabsf`、末尾 `bdclose` 清理 dirty 模型。完整文件清单 / 哈希 / 验证证据见
> **`代码生成交付清单.txt`**；生成 C 文本与 18:37 版哈希完全一致（single 化数值未变）。

| 项 | 值 |
|---|---|
| 生成时间 | 2026-11-15 **20:38**（最终收口；原 11-14），MATLAB R2023b / Simulink Coder 23.2 |
| 目标 | `ert.tlc` + `ARM Compatible->ARM Cortex-M` + Nonreusable function |
| 模型 | `three_loop_deploy.slx`：**11 输入** → `ThreeLoop_Controller` 原子子系统 → 5 输出 |
| 节拍 | 单 `step()` @ **50 µs（20 kHz）**；位置环 /20→1 kHz，速度环 /10→2 kHz，电流环每拍 |
| 参数 | `U.par_c[11] / U.par_w[5] / U.par_p[4]` 为顶层 Inport，由 `ThreeLoop_Tuning_Init()` 填默认值 |
| 精度 | **单精度 `real32_T`（float）** + `sinf/cosf/sqrtf`；静态 RAM 232 B（U 128 / Y 40 / DW 64） |
| 校验 | SIL 3000 拍最大误差 **1.79e-7**（float 量级）；tick = 3000 / 300 / 150（20:2:1）；参数接口自检 14/14 PASS |

## 一、生成边界

MCU 只跑控制律，不跑被控对象。生成模型已移除 SPS PMSM、受控电压源、平均逆变器、`powergui`、
激励、Scope、Rate Transition；检索确认无相关依赖。

时序说明：仿真模型是显式三速率，部署模型用**单速率 + chart 内部分频**，裸机一个控制 ISR 调 `step()` 即可。
**参数源 `three_loop_params.m` 的 `Ts_i=50e-6` 是唯一节拍基准**，`dt/N_w/N_p/gain_comp` 全部由它派生，
不再手改生成代码（`patch_gen_code.py` 已退役为核验脚本）。

## 二、C 接口

```c
#include "three_loop_deploy.h"
#include "three_loop_tuning.h"

/* 输入：three_loop_deploy_U，28 x real32_T = 128 B（含对齐） */
typedef struct {
  real32_T ia, ib, ic;        /* 三相电流 [A] */
  real32_T vbus;              /* 母线 [V] */
  real32_T theta_m;           /* 编码器机械角 [rad]，单圈 */
  real32_T omega_m;           /* 机械角速度 [rad/s] */
  real32_T pos_ref;           /* 位置指令 [rad]，须经 wrap 后写入 */
  real32_T en;                /* 1=使能 */
  real32_T par_c[11];         /* 电流环参数：kp ki dt Ld Lq psi max_duty overmod mag_vd_max dec imax */
  real32_T par_w[5];          /* 速度环参数：kp_w ki_w Ts_w i_max N_w */
  real32_T par_p[4];          /* 位置环参数：kp_p w_max Ts_p N_p */
} ExtU_three_loop_deploy_T;

/* 输出：three_loop_deploy_Y，9 x real32_T = 40 B（含对齐） */
typedef struct {
  real32_T duty[3], iq_ref, w_ref, iq_used, ticks[3];
} ExtY_three_loop_deploy_T;

extern void three_loop_deploy_initialize(void);   /* 上电一次 */
extern void three_loop_deploy_step(void);         /* 20 kHz 控制 ISR 每拍 */

/* 可调参数接口（手工层，不随 codegen 覆盖） */
void ThreeLoop_Tuning_Init(void);
ThreeLoop_ParamStatus_T ThreeLoop_ParamSet(uint8_t id, int32_t raw, uint8_t is_enabled);
ThreeLoop_ParamStatus_T ThreeLoop_ParamGet(uint8_t id, int32_t *raw);
real32_T ThreeLoop_WrapPosRefEx(real32_T pos_ref_multi, real32_T theta_m, real32_T dir_sign);
```

静态 RAM（host nm 实测）：`U=128 B`、`Y=40 B`、`DW=64 B`，合计 **232 B**
（比 double 版 392 B 少 41%）；`step` 0x8e0 B（host x64 -O2 参考值，ARM 需目标编译确认）。

**参数 ID / 定点换算 / 范围 / 展开映射**见 `PID参数对外接口说明.md` §3，摘要：

| ID | 参数 | 默认 | scale | 范围 |
|---|---|---|---|---|
| 0x01 / 0x02 | `CUR_KP` / `CUR_KI` | 0.713142 / 2378.186 | ×1e3 | 0.05–3 / 100–10000 |
| 0x03 / 0x04 | `VEL_KP` / `VEL_KI` | 0.072923 / 10.190689 | ×1e6 / ×1e3 | 0–1 / 0–100 |
| 0x05 / 0x06 | `POS_KP` / `VEL_MAX` | 50.265482 / 120 | ×1e3 | 0–200 / 1–200 |
| 0x07 / 0x08 | `I_MAX` / `DUTY_MAX` | 9 / 0.95 | ×1e3 | 0.5–9 / 0.5–0.98 |
| 0x81–0x85 | 只读：`TS_US/N_W/N_P/GAIN_COMP/POLE_PAIRS` | 50/10/20/1050/14 | — | 写入返回 READ_ONLY |

## 三、目录文件

| 文件 | 说明 |
|---|---|
| `three_loop_deploy.c/.h`、`*_private.h`、`*_types.h`、`rtwtypes.h`、`rtmodel.h` | 6 个**生成文件**，加入 MDK 工程 |
| `three_loop_tuning.h/.c` | **新增手工层**：PID 参数对外接口（也要加入工程） |
| `integration_example.c` | TIM1 ISR 接线模板（含 Init / wrap 调用） |
| `sil_test.c` + `sil_input.csv` / `sil_expected.csv` | host SIL 数值一致性 |
| `tuning_test.c` | 参数接口 14 项自检 |
| `代码生成交付清单.txt` | **本轮最终生成清单**：文件 SHA256、接口/时序、8 项验证证据、收口修正与局限 |
| `model_io_check.txt` | 生成后接口/依赖核验（11/11 输入、参数可调=1、无固定 PID 字面量） |
| `patch_gen_code.py` | 已退役为核验脚本（`--check` 输出 4 项「模型侧已修正」） |
| `three_loop_deploy.slx`、`build_three_loop_deploy.m`、`generate_three_loop_code.m`、`fcn_*_single.m`（部署用）、`fcn_*_deploy.m`（double 历史 / 闭环 MIL 用） | 部署模型与构建/生成脚本 |
| `three_loop_params.m`（在上级 `FM_Servo_V3/`） | **唯一参数源**，改这里再重新生成 |
| `PID参数对外接口说明.md`、`参数接口与修正报告.html`、`单精度化说明.md` | 本轮接口说明、修正汇总与单精度化说明 |
| `three_loop_deploy_ert_rtw/` | ERT 完整生成包 + `_backup_20261115/`（手改版 C 备份）+ `_host_build/` |

> 加入固件的 `.c` 只有 `three_loop_deploy.c` + `three_loop_tuning.c`；
> `integration_example.c` 需按 BSP 改造；`sil_test.c` / `tuning_test.c` 含 `main()`，**不要**加入固件工程；
> `ert_main.c` 已排除。

## 四、集成步骤（X72_FOC / STM32G474）

1. **加文件**：目标工程 `..\embedded_project\STM32G474_App\MDK-ARM`，加入 6 个生成文件 + `three_loop_tuning.c`，本目录加入 Include Path。
2. **上电顺序（不能反）**：
   `three_loop_deploy_initialize()` → `AppParamInit()`（FlashDB）→ `ThreeLoop_Tuning_Init()`（默认值 + 固定时序项）
   → 回放已保存参数（`ThreeLoop_ParamSet(id, raw, 0)`）→ 才启动控制 ISR。
3. **20 kHz 控制 ISR**：TIM1 中心对齐 PWM `PSC=0, ARR=4249` @170 MHz，ADC1/ADC2 由 `TIM1_TRGO` 触发，
   每个 PWM 周期（50 µs）调一次 `step()`。按 `integration_example.c`：读电流/编码器 → 填 8 个信号字段 → `step()` → `duty` 写 CCR。
   **`U.par_*` 不在 ISR 里写**。
   ⚠ 当前代码已 single 化；若回退 double 版，单拍 9282 cyc > 50 µs 会丢触发掉回 10 kHz，
   届时必须把 `Ts_i` 改回 `100e-6` 重新生成以匹配实际节拍。烧录前用 DWT 复测单拍周期。
4. **位置指令必须 wrap**：`U.pos_ref = ThreeLoop_WrapPosRefEx(pos_ref_multi, theta_m, dir_sign)`。
   多圈 `pos_ref` 与单圈 `theta_m` 直接相减跨 2π 会跳变 → 过流跳闸（文档 §3.3）。
5. **电流标定**：1 mΩ 分流 + 差分 30× + 1.65 V 偏置 → `I[A] = (ADC_V − 1.65)/0.03`（约 0.0269 A/LSB）。
6. **编码器**：MT6835 21 bit，`theta_m = 2π·code/2^21`；**零点偏置需标定**（控制器内部固定 +π/2 对齐）。
   `omega_m` 用相邻 50 µs 角度差分 + 低通。
7. **参数写入**：走 `database` 的 `set_app_*` → `ThreeLoop_ParamSet`；使能态拒写、权限总闸 30068、
   范围检查、定点换算、FlashDB 持久化都在 database 层。**禁止直接写 `U.par_c[2]` 等固定项**。
8. **保护**：`en=0` 时生成代码清零积分、输出 duty=0.5；过流/过压/编码器异常在 BSP 层拉低 `en` 并封锁 PWM。

编译选项：Keil AC6 `--cpu Cortex-M4.fp.sp` + C99 + O2；或 arm-none-eabi-gcc
`-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -std=c99`。

## 五、验证（本轮实测，可复核）

| 验证 | 结果 |
|---|---|
| 4 处频率错配 | `patch_gen_code.py --check` → 4/4「模型侧已修正」，不再替换文件 |
| 接口核验 | `model_io_check.txt`：11/11 输入、5/5 输出、无 SPS、参数可调=1、无固定 PID 字面量 |
| host 编译 | MinGW GCC `-std=c99 -O2 -Wall` ×2，0 error / 0 warning |
| SIL 数值一致性 | 3000 拍 `max|Δduty| = 1.79e-7` PASS（单精度容差 1e-6）；ticks 3000/300/150（20:2:1） |
| 单精度核验 | `model_io_check.txt`：`real32_T` 接口 + `sinf/cosf/sqrtf`；nm 显示 RAM 392→232 B |
| 参数接口自检 | `tuning_test.exe` **14/14 PASS**（默认值、定点、只读、使能拒写、越界、跨 2π wrap、固定项保护） |
| 符号 | `nm`：`step` T 0x930 B、`initialize` T 0x10 B、U 0xe0 / Y 0x48 / DW 0x60 |

复现（本机 `C:\Windows\TEMP` 不可写，MATLAB 与 gcc 都需重定向）：

```bat
set TMP=C:\Users\92714\WorkBuddy\MBD开发\three_loop_pmsm\tmp
set TEMP=%TMP%
set TMPDIR=%TMP%
set MATLAB_PREFDIR=%TMP%\prefs

gcc -std=c99 -O2 -Wall -I. sil_test.c three_loop_deploy.c three_loop_tuning.c -o sil_test.exe -lm
sil_test.exe
gcc -std=c99 -O2 -Wall -I. tuning_test.c three_loop_deploy.c three_loop_tuning.c -o tuning_test.exe -lm
tuning_test.exe
python patch_gen_code.py --check

matlab -sd "G:\FOC\X72\Firmware\FM_Servo_V3\simulink_code_gen" -batch "generate_three_loop_code"
```

> 文档 §5.3 提到的 `save_system` 临时目录阻塞，在上述 TMP/TEMP/TMPDIR + `MATLAB_PREFDIR`
> 重定向后**不再出现**；本轮已成功重建 `.slx` 并正规生成代码。

## 六、局限与待办

1. **已 single 化，但实机周期数仍需复测**。生成代码已是 `real32_T` + `sinf/cosf/sqrtf`，
   文档配置⑤单精度实测 1813 cyc（< 50 µs 预算），但本轮只做了 host x64 编译，
   **没有 ARM 目标编译和实机测量**。烧录前请用 `DWT->CYCCNT` 确认 `step()` 单拍
   < 50 µs（8500 cyc @170 MHz）；若超限需减少运行时分支或查表化 sin/cos。
   闭环 MIL 仿真仍用 double 控制器（与历史结果同口径），详见 `单精度化说明.md` §5。
2. **节拍已切回 20 kHz**：`dt=5e-5`、`N_w=10`、`N_p=20`、`gain_comp=0.00105`，PID 默认值未改；
   速度环 2 kHz / 位置环 1 kHz 保持不变。切换只需改 `Ts_i` 一处。
3. **未做 ARM 目标编译**：本机无 `arm-none-eabi-gcc`，只做 host 语法与数值验证。
4. **协议层未落地**：`database/` 为空，`set/get_app_*` 仍是接口契约；参数 ID 0x01–0x08 需同步进
   MAVLink 消息表与上位机字段表。
5. **位置模式限 ±π**：wrap 只保证单圈内正确；大范围绝对定位需按文档 §6.3 增加多圈 `theta_pos` 输入。
6. **未建模**：死区、管压降、母线纹波、ADC/编码器量化、齿槽转矩、弱磁、轨迹规划；低速测速量化与
   电流环带宽实测仍是待办（文档 §6.4/§6.5）。
7. 固定项虽在 `U` 中可见但**不可写**：其他模块直接写 `U.par_c[2]` 会绕过校验，集成评审中列为禁止项。

Markdown / HTML / CSV / 文本报告均为 UTF-8 BOM，便于导入飞书/Lark。
