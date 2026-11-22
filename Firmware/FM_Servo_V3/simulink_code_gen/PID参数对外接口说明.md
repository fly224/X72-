# 三环 PID 参数对外接口与文档问题修正

> ⚠ **2026-11-15 18:00 节拍已切回 20 kHz**：本文正文中的
> `Ts_i=100e-6 / dt=1e-4 / N_w=5 / N_p=10 / gain_comp=0.0021 / tick 10:2:1`
> 是 **10 kHz 阶段的历史记录**，保留用于对照。当前权威值见 `README.md` 与
> `sim_check/results/仿真效果报告.html`：`Ts_i=50e-6`、`dt=5e-5`、`N_w=10`、`N_p=20`、
> `gain_comp=0.00105`、tick 20:2:1；**速度环 2 kHz / 位置环 1 kHz 与 8 个 PID 默认值均未变**。
> 参数 ID / 定点换算 / 范围 / database 桥接 / 整定公式全部继续适用。
> 实机要真跑 20 kHz 需要单精度（double 版 9282 cyc > 50 µs 会掉回 10 kHz）。
> **18:40 已完成 single 化**：生成代码 `real32_T` + `sinf/cosf/sqrtf`，RAM 392→232 B，
> 参数接口/定点 ID/范围全部继续适用（详见 `单精度化说明.md`）。本文正文的 `real_T` 类型描述为历史记录。

> 2026-11-15 · 依据 `G:\FOC\X72\Agnet\电流环整定与三环频率修正.md`
> 目标：按文档修掉 4 处错配 + 坐标系 bug，并把「改增益必须重新 codegen」变成**运行时可调接口**。

---

## 1. 结论先行

| 项 | 修改前 | 修改后 |
|---|---|---|
| 控制周期 | 模型 20 kHz / 实机 10 kHz，`dt` 手写补丁 | `three_loop_params.m` 的 `Ts_i=100e-6` 为唯一源，重新生成即带 `dt=1e-4`、`N_w=5`、`N_p=10`、`gain_comp=0.0021` |
| 4 处修正的落地方式 | 手改生成 C，`generate` 一跑全没；`patch_gen_code.py` 幂等补字面量 | 模型侧正规派生，补丁脚本**退役为核验**（`--check` 输出 4 项「模型侧已修正」） |
| PID 参数 | `InlineParams=on` 固化成字面量，改一次要重建模型 | 模型新增 3 个顶层 Inport（`par_c[11] / par_w[5] / par_p[4]`），生成 C 从 `U.par_*[]` 读取 |
| 对外接口 | 无 | `ThreeLoop_Tuning_T` + `ThreeLoop_ParamSet/Get`（定点 ID 0x01–0x08）+ 位置 wrap |
| 位置坐标系 | `pos_ref` 多圈 vs `theta_m` 单圈 → 跨 2π 过流 | `ThreeLoop_WrapPosRef()` 折到 ±π，host 自检通过 |

**验证**：SIL 3000 拍 `max|duty_C − duty_Simulink| = 7.17e-12`；参数接口自检 **14/14 PASS**；
`patch_gen_code.py --check` 4/4「模型侧已修正」。生成模型 11 输入 / 5 输出，主代码 343 行。

---

## 2. 文档问题逐条处置

| 文档章节 | 问题 | 本轮处置 | 证据 |
|---|---|---|---|
| §3.1 | 电流环 `dt` 写 5e-5、实际 1e-4，`Ki_eff` 只有一半 | `P.Ts_i=100e-6` → `par_c(3)=1e-4` | `three_loop_deploy.c:146-149` 从 `U.par_c[2]` 取 dt；`three_loop_tuning.c` `TL_TS_I=100e-6` |
| §3.2 | 速度环 `/10`、位置环 `/20` 在 10 kHz 下变成 1 kHz / 500 Hz | `N_w=round(Ts_w/Ts_i)=5`、`N_p=round(Ts_p/Ts_i)=10` | `three_loop_deploy.c:74` 用 `U.par_p[3]`、`:87` 用 `U.par_w[4]` |
| §5.1 末行 | 手改版漏掉 `gain_comp` 翻倍 | `gain_comp = p·1.5·Ts_i = 14×1.5×100µs = 0.0021` | `three_loop_deploy.c:51` |
| §3.3 | 多圈 `pos_ref` 与单圈 `theta_m` 跨 2π 跳变 → 过流 | 新增 `ThreeLoop_WrapPosRef/Ex()`，含 `dir_sign` | `tuning_test.c` 跨 2π 用例 PASS（err=0.10 rad） |
| §5.3 | `save_system` 被 `C:\Windows\Temp` 权限阻塞 | 用工程内 TMP/TEMP/TMPDIR + `MATLAB_PREFDIR` 重定向后**可以正常保存并生成** | 本轮 `generate_three_loop_code` 退出 0 |
| §6.1 | 改参数必须重建模型 | 参数改 Inport，运行时可调 | `model_io_check.txt` `参数可调=1 / 无固定字面量=1` |

> **节拍口径变化**：现在 `step()` 是 10 kHz。SIL 末拍 `ticks = 3000 / 600 / 300`，
> 即 **10 kHz : 2 kHz : 1 kHz**（比值 10:2:1），不再是 20 kHz 时代的 20:2:1。
> 速度环 `Ts_w=500µs`、位置环 `Ts_p=1ms` 仍然成立。

---

## 3. 对外接口总览

```
上位机 (MAVLink / CiA402, 定点 int32)
    │  读参数 30000 / 写参数（需权限总闸 30068）
    ▼
database/app_param.c   set_app_*() / get_app_*()
    │  状态/范围/单位换算/FlashDB 持久化；使能态拒写
    ▼
three_loop_tuning.c    ThreeLoop_ParamSet(id, raw, is_enabled)
    │  影子结构体 + 一次性展开（原子提交）
    ▼
three_loop_deploy_U.par_c[11] / par_w[5] / par_p[4]
    ▼
three_loop_deploy_step()   ← 10 kHz ISR（只填信号，不解析协议）
```

**分层纪律**（对齐 `固件协议层_职责与编写指南.md` §5/§7）：协议解析与参数写入在 1 ms 非实时任务，
ISR 里只填 8 个信号字段并调 `step()`；`sil_test.c` / `tuning_test.c` 只在 host 用，不进固件。

### 3.1 可调参数表（8 项，ID 0x01–0x08）

| ID | 名称 | 结构体字段 | 单位 | 默认值 | 定点 scale | raw 默认 | 允许范围 | 整定方向 |
|---|---|---|---|---|---|---|---|---|
| 0x01 | `CUR_KP` | `kp_i` | V/A | 0.713141532365 | ×1e3 | 713 | 0.05 – 3.0 | ↑ 电流环带宽↑，但延时/噪声敏感 |
| 0x02 | `CUR_KI` | `ki_i` | V/(A·s) | 2378.18563877 | ×1e3 | 2378186 | 100 – 10000 | 必须随 `kp_i` 按 `R/L` 同步 |
| 0x03 | `VEL_KP` | `kp_w` | A/(rad/s) | 0.0729227202682 | ×1e6 | 72923 | 0 – 1.0 | ↑ 速度刚度↑，过大易振荡 |
| 0x04 | `VEL_KI` | `ki_w` | A/(rad/s)/s | 10.1906891515 | ×1e3 | 10191 | 0 – 100 | 消稳态速度误差 / 恒负载 |
| 0x05 | `POS_KP` | `kp_p` | (rad/s)/rad | 50.2654824574 | ×1e3 | 50265 | 0 – 200 | 位置闭环带宽 ≈ `kp_p/2π` Hz |
| 0x06 | `VEL_MAX` | `w_max` | rad/s | 120.0 | ×1e3 | 120000 | 1 – 200 | 位置环输出限速（≈1146 rpm） |
| 0x07 | `I_MAX` | `i_max` | A | 9.0 | ×1e3 | 9000 | 0.5 – 9.0 | **不得超过硬件峰值 9 A** |
| 0x08 | `DUTY_MAX` | `max_duty` | — | 0.95 | ×1e3 | 950 | 0.5 – 0.98 | 电压圆上限 = `Vbus/√3·max_duty` |

### 3.2 只读参数（ID 0x81–0x85，上位机显示/自检，写入返回 `READ_ONLY`）

| ID | 名称 | 值 | 说明 |
|---|---|---|---|
| 0x81 | `CTRL_TS_US` | 100 | 实际 ISR 周期 [µs] |
| 0x82 | `N_W` | 5 | 速度环分频，10 kHz/5 = 2 kHz |
| 0x83 | `N_P` | 10 | 位置环分频，10 kHz/10 = 1 kHz |
| 0x84 | `GAIN_COMP` | 2100 (×1e6 = 0.0021) | `p·1.5·Ts_i` |
| 0x85 | `POLE_PAIRS` | 14 | 辨识实测 ratio≈14 |

### 3.3 `U.par_*` 展开映射（**固定项禁止上位机改写**）

| 数组 | 索引 | 含义 | 来源 |
|---|---|---|---|
| `par_c[11]` | 0,1 | `kp_i`, `ki_i` | ✅ 可调 |
| | 2 | `dt = 1e-4` | 🔒 由 `Ts_i` 派生 |
| | 3,4,5 | `Ld, Lq, psi` | 🔒 电机标定（113.5µH / 7.593mWb） |
| | 6 | `max_duty` | ✅ 可调 |
| | 7,8,9 | `overmod=1`, `mag_vd_max=0.98`, `dec=3` | 🔒 调制/解耦结构 |
| | 10 | `i_max` | ✅ 可调 |
| `par_w[5]` | 0,1 | `kp_w`, `ki_w` | ✅ 可调 |
| | 2,4 | `Ts_w=5e-4`, `N_w=5` | 🔒 时序 |
| | 3 | `i_max`（与 `par_c[10]` 同步） | ✅ 自动同步 |
| `par_p[4]` | 0,1 | `kp_p`, `w_max` | ✅ 可调 |
| | 2,3 | `Ts_p=1e-3`, `N_p=10` | 🔒 时序 |

> ⚠️ 虽然生成代码把 `dt/N_w/N_p` 也暴露成 `U` 字段，**只能通过 `ThreeLoop_Tuning_Apply()` 写入**；
> 不要直接改 `U.par_c[2]` 等，否则会重演文档 §3 的时序错配。

### 3.4 函数签名（`three_loop_tuning.h`）

```c
typedef struct {
    real_T kp_i, ki_i, kp_w, ki_w, kp_p, w_max, i_max, max_duty;
} ThreeLoop_Tuning_T;

typedef enum {
    THREE_LOOP_OK = 0, READ_ONLY = 1, OUT_OF_RANGE = 2,
    WRITE_STATE_ERROR = 3, UNKNOWN_ID = 4
} ThreeLoop_ParamStatus_T;   /* 与 APP_PARAM_STATUS 数值对齐 */

void ThreeLoop_Tuning_Init(void);                 /* 上电一次：默认值 + 固定项 -> U */
ThreeLoop_ParamStatus_T ThreeLoop_Tuning_Validate(const ThreeLoop_Tuning_T *t);
void ThreeLoop_Tuning_Apply(const ThreeLoop_Tuning_T *t);   /* 原子展开 */
void ThreeLoop_Tuning_Get(ThreeLoop_Tuning_T *t);           /* 回读 */
ThreeLoop_ParamStatus_T ThreeLoop_ParamSet(uint8_t id, int32_t raw, uint8_t is_enabled);
ThreeLoop_ParamStatus_T ThreeLoop_ParamGet(uint8_t id, int32_t *raw);
real_T ThreeLoop_WrapPosRefEx(real_T pos_ref_multi, real_T theta_m, real_T dir_sign);
real_T ThreeLoop_WrapPosRef(real_T pos_ref_multi, real_T theta_m);   /* dir_sign=+1 */
```

---

## 4. database 桥接示例（按工程约定写）

```c
/* database/app_param.c —— 唯一参数通道；协议层不得直接改 U */
APP_PARAM_STATUS set_app_CurKp(int32_t raw)
{
    uint8_t en = (uint8_t)get_app_Enable();
    return (APP_PARAM_STATUS)ThreeLoop_ParamSet(TL_PID_CUR_KP, raw, en);
}
int32_t get_app_CurKp(void)
{
    int32_t r = 0;
    (void)ThreeLoop_ParamGet(TL_PID_CUR_KP, &r);
    return r;
}
/* CUR_KI / VEL_KP / VEL_KI / POS_KP / VEL_MAX / I_MAX / DUTY_MAX 同构 */
```

```c
/* app 初始化顺序（顺序不能反！） */
three_loop_deploy_initialize();      /* 清 DW 状态 */
AppParamInit();                      /* 从 FlashDB 读参数 */
ThreeLoop_Tuning_Init();             /* 先填默认值 + 固定项（含 dt/N_w/N_p） */
for (各已保存参数 id) {
    ThreeLoop_ParamSet(id, flash_raw[id], 0);   /* 再回放用户参数，只覆盖可调项 */
}
/* 之后才启动 TIM1 控制 ISR */
```

```c
/* 10 kHz 控制 ISR（只做信号，不解析协议） */
U.ia = ...; U.ib = ...; U.ic = ...; U.vbus = ...;
U.theta_m  = (real_T)theta_now;
U.omega_m  = (real_T)wm;
U.pos_ref  = ThreeLoop_WrapPosRefEx((real_T)s_pos_ref, (real_T)theta_now,
                                    (real_T)s_dir_sign);  /* 文档 §3.3 */
U.en       = (real_T)get_app_Enable();
/* U.par_* 不在这里写 */
three_loop_deploy_step();
TIM1->CCR1 = (uint16_t)(Y.duty[0] * PWM_ARR + 0.5);
```

**安全规则**
1. `ThreeLoop_ParamSet(..., is_enabled=1)` 返回 `WRITE_STATE_ERROR`，与协议层「使能态拒写」一致；
   权限总闸 30068 由 database 层先判。
2. 每次写入都从 `U` 回读完整影子结构体、只改一项、整体 `Validate` 通过后一次性 `Apply`，
   避免多字段半更新；`Apply` 内部调用弱函数 `ThreeLoop_EnterCritical/ExitCritical()`，
   若要在运行中不停车改参数，BSP 层应覆盖为 `__disable_irq()/__enable_irq()`。
3. 范围检查在 `Validate` 与 `ParamSet` 双重执行；`isfinite` 拦截 NaN/Inf。
4. 持久化由 database/FlashDB 负责；本层不碰 Flash。

---

## 5. 整定方法（参数怎么设）

**电流环（标准 IMC / 极点抵消）**

```
ωc = kp_i / Ld = 0.7131415 / 113.5e-6 = 6283 rad/s ≈ 1000 Hz
零点 = ki_i / kp_i = 3334.8 rad/s = R/L (0.3785Ω / 113.5µH)
=> 改 kp_i 时必须同步：ki_i = kp_i × (R/L) ≈ kp_i × 3334.8
```
例如带宽降到 500 Hz：`kp_i = 0.3566`，`ki_i = 1189.1`。**只改一项会破坏极点抵消**。

**速度环 / 位置环**

```
kp_w = (2ζωw·J − B)/Kt,  ki_w = ωw²·J/Kt      （ζ=0.9, ωw=2π·40）
kp_p ≈ 2π·f_p                                  （f_p=8 Hz -> 50.265）
```
提高 `kp_p` 会放大位置环带宽，须保持 `f_p ≲ f_w/5`，否则级联失稳。

**推荐整定顺序**：`en=0` → 电流环（先 Kp 后同步 Ki，阶跃看 iq 跟随）→ 速度环（空载→带载）
→ 位置环（小阶跃→大阶跃）→ 每步 `ParamGet` 回读确认 → 满意后由 database 保存 Flash。

---

## 6. 验证结果（本轮实测）

| 项目 | 命令 / 证据 | 结果 |
|---|---|---|
| 模型接口 | `model_io_check.txt` | 11/11 输入、5/5 输出、`参数可调=1`、`无固定 PID 字面量=1` |
| 4 处错配 | `patch_gen_code.py --check` | 4/4「模型侧已修正」，不再替换文件 |
| host 编译 | `gcc -std=c99 -O2 -Wall` ×2 | 0 error / 0 warning |
| SIL 数值一致性 | `sil_test.exe`（3000 拍） | `max|Δduty| = 7.17e-12` PASS；ticks 3000/600/300 |
| 参数接口自检 | `tuning_test.exe` | 14/14 PASS（默认值、定点、只读、使能拒写、越界、跨 2π wrap、固定项保护） |
| 符号 | `nm` | `step` / `initialize` / `U` / `Y` / `DW` 齐全 |

复现（本机 `C:\Windows\TEMP` 不可写，必须重定向）：

```bat
set TMP=C:\Users\92714\WorkBuddy\MBD开发\three_loop_pmsm\tmp
set TEMP=%TMP%
set TMPDIR=%TMP%
set MATLAB_PREFDIR=%TMP%\prefs
matlab -sd "G:\FOC\X72\Firmware\FM_Servo_V3\simulink_code_gen" -batch "generate_three_loop_code"

gcc -std=c99 -O2 -Wall -I. sil_test.c three_loop_deploy.c three_loop_tuning.c -o sil_test.exe -lm
sil_test.exe
gcc -std=c99 -O2 -Wall -I. tuning_test.c three_loop_deploy.c three_loop_tuning.c -o tuning_test.exe -lm
tuning_test.exe
python patch_gen_code.py --check
```

---

## 7. 局限与后续

1. **必须重测 DWT 周期数**：参数改成 Inport 后，编译器不再能把 `kp_i/ki_i/dec/N` 常量折叠，
   `step()` 里多了数组读取与运行时分支（如 `par_c[9]==3` 判断）。原实测 9282 cyc / 54.6%，
   新版本应在实机重新量一次，确认仍 < 100 µs。
2. **根本解法仍是 single 化回 20 kHz**（文档 §6.2，配置⑤实测 1813 cyc）。届时把
   `three_loop_params.m` 的 `Ts_i` 改回 `50e-6`，`dt/N_w/N_p/gain_comp` 会自动跟回，
   **可调参数默认值无需改动**。
3. **协议层尚未落地**：`database/` 当前为空，`set/get_app_*` 是接口契约（协议层文档 §9 明确 NO-GO）。
   本层提供 `ParamSet/ParamGet` 供后续回调接入；参数 ID 0x01–0x08 需同步进 MAVLink 消息表/上位机字段表。
4. **位置模式仍限 ±π**：wrap 只保证单圈内正确；大范围绝对定位需按文档 §6.3 给模型加多圈 `theta_pos` 输入。
5. **低速测速量化**（文档 §6.4）与电流环带宽实测（§6.5）仍未解决，不在本轮范围。
6. **固定项虽在 `U` 中可见但不可写**：若其他模块直接写 `U.par_c[2]` 等，会绕过 `Validate`，
   需在集成评审中列为禁止项。
7. 生成代码仍是 `real_T = double`（M4F 单精度 FPU），与文档 §6.2 的性能结论一致。

---

## 8. 本轮改动文件

| 文件 | 变更 |
|---|---|
| `build_three_loop_deploy.m` | `par_c/par_w/par_p` 从 Constant → 顶层 Inport（11 输入），维度 11/5/4 |
| `generate_three_loop_code.m` | SIL 外部输入扩到 29 列；核验改为「参数可调 + 无固定字面量」；同源复制修复 |
| `three_loop_deploy.slx` / `.c` / `.h` | 重新生成，PID 从 `U.par_*[]` 读取 |
| `three_loop_tuning.h/.c` | **新增**：参数结构体、默认值、范围、定点 ID、原子提交、位置 wrap |
| `tuning_test.c` | **新增**：参数接口 14 项自检 |
| `sil_test.c` | 增加 `ThreeLoop_Tuning_Init()`，链接 `three_loop_tuning.c` |
| `patch_gen_code.py` | 退役为核验脚本（兼容旧固化版） |
| `integration_example.c` / `README.md` | 同步新接口 |
| `PID参数对外接口说明.md`、`参数接口与修正报告.html` | 本说明与汇总报告 |
