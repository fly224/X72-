# 20 kHz 回归与三环参数化改造

> 2026-11-15 21:0x · 固件 `X72_FOC.axf` Code **69876 B** / 0 Error 0 Warning
> 实测：`rate=20.0kHz`、`step max=2279cyc (26.8%)`、`3.00 rpm（给定 3.00）`、`f=0x00`、`clamp=0`

---

## 1. 结论：模型在 18:37 被重新生成，三件事一起变了

今天 18:37 有一次模型重新生成（`three_loop_deploy.c/.h/_types.h` 与 `three_loop_params.m`
的 `Model version` 时间戳全部更新）。新版本带来三个结构性变化：

| 变化 | 之前 | 之后 |
|---|---|---|
| 数据类型 | `real_T = double` | **`real32_T`（单精度）** |
| 数学函数 | `sin/cos/sqrt`（双精度软浮点） | **`sinf/cosf/sqrtf`** |
| 三环参数 | 编译期字面量（`2378.18563877 * 5.0E-5`） | **顶层输入 `par_c[11]` / `par_w[5]` / `par_p[4]`** |
| 分频 | 硬编码 `cnt >= 10.0` | **`cnt >= par_p[3]`（运行时）** |

**这等于把一直想做的「单精度化」做完了**，而且顺带把参数从生成代码里挪了出来
——以后调参不用再手改生成物，也就没有「重新生成就丢补丁」的问题。

### 直接后果：控制环自动回到 20 kHz

```
[DIAG] rate=20.0kHz  dt_avg=8500cyc  dt_min=8496cyc
[DIAG] step max=2279cyc  26.8% of measured period
[DIAG] hot theta=206 readinj=274 model=859 duty=236 enc=675 cyc
[DIAG] ticks=60010 adc1=60010 adc2=60010 tx=299 drop=0 rec=0 fault=0x00 vbus=30.8V
```

| 指标 | 单精度前（double） | 现在 |
|---|---:|---:|
| 控制频率 | 10.0 kHz | **20.0 kHz** |
| 单拍峰值 | 9282 cyc（54.6%） | **2279 cyc（26.8%）** |
| model 段 | 7756 cyc | **859 cyc** |
| ADC1 : ADC2 | 2 : 1（丢一半触发） | **1 : 1** |

**此前"三条 dt 错配、坐标系补丁、单精度化"这一整串问题，根源都是 double，现在一并消失了。**

---

## 2. 踩的坑：参数全 0 导致电机不转

新代码把参数改成输入后，`servo_isr.c` 仍在按旧接口写（不填 `par_c`），
于是三个数组**全是 0**：

```
Kp = Ki = 0  ->  电流环没有任何输出  ->  电机有电流却不转
位置环误差持续累积 -> clamp 从 0 涨到 1809
```

**实测现象**（当时很容易误判成"代码丢了"）：

```
[CMD] IDENT ok ratio=13.77 pn=14 dir=+1 ... ok
平均转速 0.01 rpm（给定 3.00）
[CMD] st ... i=0.36/0.71 f=0x00 clamp=1809
```

同时 `Code` 从 76124 掉到 68344，看起来像"少了 7.8 KB"。**实际是单精度省掉了
整套 `__hardfp_*` 双精度数学库**，符号表里 `three_loop_deploy_step`、
`Servo_ControlTick`、double 数学库、ADC 回调都还在。

### 修复：新增 `model_params_init()`

位置：`Board/servo_isr.c`，在 `Servo_Init()` 里 `three_loop_deploy_initialize()` **之后**调用
（initialize 会清零 U/Y/DW）。

```c
/* par_c = [kp_i, ki_i, dt, Ld, Lq, psi, max_duty, overmod, mag_vd_max, dec_mode, i_max] */
three_loop_deploy_U.par_c[0]  = 0.71314153f;   /* kp_i = wc*Ld */
three_loop_deploy_U.par_c[1]  = 2378.1856f;    /* ki_i = wc*R_eff */
three_loop_deploy_U.par_c[2]  = 5.0e-5f;       /* dt = 50 us (20 kHz) */
...
/* par_w[4] = N_w = 10，par_p[3] = N_p = 20（Ts_w/Ts_i、Ts_p/Ts_i） */
```

两条硬约束（违反任何一条三环频率就会错一档）：
1. `par_c[2]`（dt）必须等于**实际控制周期**；
2. `par_w[4] = Ts_w/Ts_i`、`par_p[3] = Ts_p/Ts_i`。

---

## 3. 连带修复：测速低通要跟着改

回到 20 kHz 后出现 **~8 Hz 振荡**（`iq_tar` 0.002→0.494→0.14，`rpm` 0→12.8→−2.9，周期 ~130 ms），
平均转速 4.77 rpm。

根因：`BSP_SPEED_LPF_ALPHA = 0.06` 是按 10 kHz 定的（等效 95 Hz），
20 kHz 下同系数变成 **190 Hz**，低速量化噪声通过太多，速度环被噪声驱动。

```c
/* bsp_servo.h */
#define BSP_SPEED_LPF_ALPHA  (0.03f)    /* 20 kHz 下 ≈ 95 Hz */
```

改完立刻稳定：**3.00 rpm，clamp=0**。

> 教训：任何「按控制频率折算」的系数（低通 alpha、延时补偿、分频）换频率时都要重新算一遍。
> 这次 `Ts_i` 由模型参数自动跟随了，但 `alpha` 在 BSP 里，得手工改。

---

## 4. 死区补偿：移植过来了，但默认关闭

按 FM_XC `current_ctl_loop_task.c:31` 移植了 `Board/deadband_comp.c/.h`：
`Δd = t_dead × f_pwm ≈ 1.34%`，按电流方向补偿，`|i| < 0.6 A` 内线性过渡。

**实测结论：在 X72 上默认关闭**（`s_enable = 0U`）。

| | 平均转速（给定 3.00） | clamp |
|---|---:|---:|
| 补偿 ON | 0.10 rpm | 748 |
| 补偿 OFF | 3.00 rpm | 0 |

原因是**闭环电流环的积分器已经把死区的平均电压损失补掉了**：
理论补偿量 1.34% × 30 V = 0.40 V，而 3 rpm 空载所需相电压只有
`0.2 A × 0.378 Ω ≈ 0.076 V` —— 补偿量是需求的 5 倍，属于过补偿，
PI 反向拉回后又过冲，于是振荡。

**它真正有价值的场景**：纯电压开环、电流环带宽很低、或标定出真实 `comp_du`
后做过零脉动优化。模块与串口 `B` 开关都保留，随时可启用。

---

## 5. 现在的架构：参数在哪、怎么改

```
three_loop_params.m          ← 参数唯一源（Ts_i=50e-6，与板上 20 kHz 一致）
      ↓ build（当前被 Simulink 临时目录问题挡住，需管理员权限）
three_loop_deploy.slx        ← par_c/par_w/par_p 已固化
      ↓ generate
three_loop_deploy.c          ← single + 参数为顶层输入，不要再手改
      ↑
Board/servo_isr.c : model_params_init()   ← ★ 调参改这里
```

| 想改什么 | 改哪里 |
|---|---|
| 电流环 Kp/Ki | `model_params_init()` 的 `par_c[0]` / `par_c[1]` |
| 控制周期 | `par_c[2]` + `par_w[4]` + `par_p[3]` 三个一起 |
| 速度/位置环增益 | `par_w[0..1]` / `par_p[0..1]` |
| 限幅 | `par_c[10]`（电流）、`par_p[1]`（转速） |
| 测速滤波 | `bsp_servo.h` 的 `BSP_SPEED_LPF_ALPHA` |

**`patch_gen_code.py` 已废弃**（新代码里没有那些字面量了），文件头已标注。

---

## 6. 明确遗留

1. **18:37 那次重新生成需要你确认是谁跑的**。我只做过 `build_three_loop_deploy`
   的尝试且卡在 `save_system`（`C:\Windows\Temp` 不可写）。如果那是你按 §5.3 的
   `icacls` 授权后跑的，说明解锁成功，可以继续走正规路径；如果不是，需要查清来源。
2. **`three_loop_params.m` 与实际参数是否一致未逐项核对**：`Ts_i=50e-6` 已确认一致，
   但 `par_c` 里其余 10 项是照旧值填的（kp/ki/Ld/psi 等未变），只做了功能验证。
3. **极端工况未测**：3 rpm 空载稳定，但高速、带载、正反转、急停都未验证。
4. **死区补偿如果要启用**，必须先标定真实 `comp_du`（用示波器看相电压误差），
   不能直接用理论值 1.34%。
