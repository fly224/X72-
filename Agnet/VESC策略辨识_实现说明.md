# VESC 策略辨识落地说明（极对数 / 方向 / 电角度零点一次跑出）

> 2026-11-15 16:5x · 固件 `X72_FOC.axf` Code **75132 B** / 0 Error 0 Warning
> 数学复算 **7 组工况全 PASS** · **实机辨识跑通**：`pn=14` / `dir=+1` / `zero` 两次一致 / **使能不飞车**
>
> 🔴 **过程中查出并修复一个比辨识本身更根本的 bug**：TIM1 配的是 `TIM_OCMODE_PWM2`，
> 而 `BspServo_SetDuty()` 按 PWM1 写 CCR → 三相输出整体反相 → 电流环从负反馈变正反馈。
> **这才是之前所有 20 / 40 / 43.8 / 47 A 过流与飞车的真正原因**，与硬件无关。
> 详见 §6.3。

---

## 1. 结论先行

按 VESC `mcpwm_foc_encoder_detect()` 的策略实现了三段式辨识，**一次输出三个参数**：

| 输出 | 含义 | 写入位置 |
|---|---|---|
| `ratio` / `ratio_int` | 极对数 | 回显（模型仍硬编码 14，见 §7） |
| `dir` | 编码器方向与相序是否同向 | `Servo_SetDirSign()`，影响送模型的 θ / ω |
| `offset_rad` / `zero_mech` | 电角度零点 | `BspServo_SetEncoderZero()`，**已含 π/2 模型补偿** |

原来 `Z` 只是"吸合到 0° + 单点读角"，现在 `Z` = 完整辨识（约 4.7 s）。

---

## 2. 文件架构：为什么放在 `Board/`

先看工程的实际约束（不是规划文档里的样子）：

```
IncludePath（uvprojx）: ../Core/Inc, ../Drivers/…, ../Board, ../../../simulink_code_gen
磁盘上存在的目录      : Board/（13 c/h）、Core/（23 c/h）
uvprojx 里的空分组    : app(0) motor_control(0) communication(0) database(0)
                        board_public/flash_db(0) utilties(0) software_scope(0)
```

`app/`、`motor_control/` 这些**在磁盘上并不存在**，只是 Keil 工程里 0 文件的分组；
把它们启用要先改 IncludePath，且沿 `Board → app` 调用会形成**反向依赖**。

所以辨识模块落在 Board 层，与 `servo_cmd.c` 同级：

```
Board/
├─ bsp_servo.c/.h        驱动层：电流、编码器、PWM、母线、NTC、故障
├─ servo_isr.c/.h        20 kHz 控制节拍 + Servo_* 接口（含虚拟角电流环）
├─ servo_cmd.c/.h        命令层：C/Z/I/E/D/F/R/P/S/X/?
├─ motor_ident.c/.h  ←   本次新增：VESC 三段式辨识（不依赖命令层，可单独复用）
├─ telemetry_tx.c/.h     100 Hz 遥测组帧
├─ uart_link.c/.h        ASCII 日志
├─ motion_profile.c/.h   梯形 / S 曲线
└─ vel_profile.c/.h      速度模式封装
```

依赖方向：`servo_cmd → motor_ident → bsp_servo / servo_isr`，单向，无环。

**将来启用 `app/Identify` 时**：整对文件平移 + 改 IncludePath 即可，API 不变。
`MotorIdent_*` 不引用 `servo_cmd` 的任何东西，也不引用 `uart_link`（回显由调用方做）。

---

## 3. 三段式状态机

```
IDLE ──Start()──> RAMP ──200ms──> STEP ──60 点──> SWEEP_F ──42 点──> SWEEP_R ──42 点──> DONE
                    │              │                  │                 │
                    └──────────────┴──────────────────┴─────────────────┴─> FAIL（故障/超时/卡死/ratio 非法）
```

| 状态 | tick 数 | 做什么 |
|---|---:|---|
| `RAMP` | 200 | 电流 0 → `id_ref` 线性上升，转子被吸到虚拟电角度 0 |
| `STEP` | 60 × 31 = 1860 | 虚拟角每次 ±120°：正 30 次 + 反 30 次，每点稳 30 ms |
| `SWEEP_F` | 42 × 31 = 1302 | 按 ratio 每电周期 3 点，正向走满一个机械圈 |
| `SWEEP_R` | 42 × 31 = 1302 | 反向走回来，累积不清零 → 两遍等权合并 |

总计 **≈ 4.7 s**，硬超时 20 s。

三个策略要点（都是 VESC 的，也是 FM_XC 缺的）：

1. **极对数用角度差，不用计数差**
   `ratio = 120° / mean(|Δθm|)`。丢 1~2 个编码器脉冲只让 ratio 动 0.1%；
   FM_XC 的 `pn = enc_line / Δcount` 会在低速时把取整误差放大。
2. **圆形平均 `atan2(Σsin, Σcos)`**
   天然处理 0/2π 环绕。FM_XC 要写 `if (|b1−b0| >= π) 则 += 2π` 的特判，容易漏。
3. **正反各扫一遍**
   齿槽与摩擦让转子"从正逼近"和"从负逼近"停在**不同**位置，两遍平均正好抵消这段滞后。
   FM_XC 只在 0° 附近取 2 个点。

---

## 4. 五个关键公式（含两个必须处理、否则白干的坑）

### 4.1 极对数

```
dmech_avg = atan2( Σ sin|Δθm| , Σ cos|Δθm| )     ← 只累积幅值
ratio     = (2π/3) / dmech_avg
dir       = sign( Σ_正向组 Δθm )
```

> **只累积幅值**是必须的：正向组与反向组的 `Δθm` 符号相反，带符号累加会互相抵消成 0。

### 4.2 电角度零点

```
θm     = theta_start + (pos_sum - pos_sum_start) · 2π/2²¹      ← 用 int64 累积脉冲，多圈安全
φ_i    = wrap_pi( θv - dir · ratio · θm )
φ      = atan2( Σ sin φ_i , Σ cos φ_i )
```

> 不能用 `BspServo_GetThetaCached()`：它返回的是**单圈归一化**角（0~2π），跨圈会跳变。
> 必须用 `BspServo_GetPosSum()`（int64 累积脉冲）做差。

### 4.3 坑 A：模型里的固定 π/2

`three_loop_deploy.c:48` 是：

```c
rtb_theta_sum = (14.0 * three_loop_deploy_U.theta_m + 1.57079632679) + 0.00105 * omega_m;
```

模型认为 `theta_m = 0` 时电角度是 **+90°**。所以零点必须把这段 π/2 一起吃掉：

```
Δz = dir · (φ − π/2) / ratio
```

**不处理会怎样**：辨识得再准，闭环电角度恒差 90° —— 力矩要么退化成 0（电流矢量与磁链正交），
要么反向。这很可能就是之前 `Z`（`zero = -θ`）之后仍然飞车的真正原因之一。

### 4.4 坑 B：dir = −1 时只平移零点救不回来

模型是 `ratio·theta_m`，斜率写死为正；`dir = −1` 意味着真实关系是 `−ratio·theta_m + φ`。
**平移零点只能改截距，改不了斜率。**

处理办法（VESC 同款思路）：在**送模型的入口**把 θ / ω 取反，

```c
/* servo_isr.c，模型输入处 */
three_loop_deploy_U.theta_m = (real_T)((float)s_dir_sign * theta_now);
three_loop_deploy_U.omega_m = (real_T)((float)s_dir_sign * BspServo_GetOmegaMech());
```

于是模型算的是 `ratio·(dir·θm) + π/2 = dir·ratio·θm + π/2`，与真实同号，4.3 的公式才成立。

`pos_ref` 必须同坐标系，否则使能瞬间误差翻倍（14 极对下直接顶满力矩）：

```c
Servo_SetPosRef((float)Servo_GetDirSign() * BspServo_GetThetaCached());
```

### 4.5 写零点

```c
z_old = theta_start - raw_start · 2π/2²¹;    /* 由 GetThetaCached() 与 GetEncoderRaw() 反推 */
z_new = z_old + Δz;                          /* BspServo_SetEncoderZero 内部会 fmod 归一化 */
```

顺序固定：**先封锁 MOE → 再 SetDirSign → 最后 SetEncoderZero**。
写零点会立刻改变 `theta`，若控制律还在跑会用旧角度继续。

---

## 5. 改动清单

### 新增

| 文件 | 行数 | 内容 |
|---|---:|---|
| `Board/motor_ident.h` | 113 | 状态机枚举、错误码、结果结构、5 个公开函数 |
| `Board/motor_ident.c` | 383 | 三段式状态机、圆形平均、零点换算、故障/超时保护 |
| `Agnet/verify_ident_math.py` | 225 | C 实现的同构复算，7 组工况 + 噪声 + 卡死 |

### 修改

| 文件 | 改动 |
|---|---|
| `Board/servo_cmd.c` | `Z` 改为调 `MotorIdent_Start()`；新增 `I` 查辨识结果；`do_enable` 的 `pos_ref` 乘 `dir_sign`；`do_disable` 调 `MotorIdent_Abort()`；删掉旧的 `s_align/s_align_t` 状态机（约 −840 B） |
| `Board/servo_isr.c` | 新增 `s_dir_sign` 与 `Servo_SetDirSign/GetDirSign`；模型输入 `theta_m`/`omega_m` 乘符号 |
| `Board/bsp_servo.h` | 声明 `Servo_SetDirSign` / `Servo_GetDirSign` |
| `MDK-ARM/X72_FOC.uvprojx` | Board 组加入 `motor_ident.c` |

命令层新增/变更：

```
C  电流零点标定（不变）
Z  电机参数辨识（VESC 三段式，~4.7 s）   ← 原来是"吸合到 0° 单点"
I  查看上次辨识结果（ratio/ resid/ dir/ off/ 样本数）   ← 新增
E/D/F/R/P/S/X/?  不变
```

`Z` 的回显：

```
[CMD] IDENT ok ratio=14.0000 pn=14 dir=+1 off=-89.98deg zero=123.45deg n=60/84 bad=0 moe=0
[CMD] IDENT FAIL stuck n=0/0 moe=0
```

> `off` 是 φ（相对基准的电角度偏移），`zero` 是实际写进编码器的机械角，两者差一个 π/2 与 1/ratio。

---

## 6. 验证

### 6.1 数学复算（不接硬件）

`python Agnet/verify_ident_math.py` → **全部 PASS**

| 工况 | ratio | dir | 闭环电角度最大偏差 |
|---|---|---:|---|
| 14 对 / 正向 / φ=0 | 14.0000 | +1 | 6.7e−11 rad（0.00000°） |
| 14 对 / 正向 / φ=0.837 | 14.0000 | +1 | 2.4e−07 rad（0.00001°） |
| 14 对 / **反向** / φ=0.837 | 14.0000 | −1 | 2.4e−07 rad |
| 14 对 / 反向 / φ=−2.9（跨 −π） | 14.0000 | −1 | 6.7e−07 rad |
| 7 对 / 正向 / φ=3.0（贴近 π） | 7.0000 | +1 | 3.5e−08 rad |
| 21 对 / 正向 | 21.0000 | +1 | 4.1e−07 rad |
| 4 对 / 反向 | 4.0000 | −1 | 3.7e−07 rad |
| 含 ±0.3 mrad 噪声 | 13.9992 | +1 | 1.5e−05 rad（0.00088°） |
| 转子卡死 | — | — | 正确判 stuck，不硬算 ratio |

"闭环电角度"指：写零点后模型算出的 `ratio·(dir·θm) + π/2` 与真实电角度之差。
**这一列才真正验证了 4.3 与 4.4 两个坑都被堵住。**

> 复算过程中先抓到一个**脚本自身**的错误：虚拟角 wrap 到 [−π,π] 后用它算
> `θm = (θv−φ)/ratio`，会让机械角瞬移，ratio 被拉偏成 10.5。
> 真实转子是连续跟随的，必须区分"物理位置用连续角、模运算用 wrap 角"。
> C 代码只保存 wrapped 值（等价），但验证脚本必须按物理连续来建模。

### 6.2 编译产物（`X72_FOC.map`）

```
motor_ident.o  18 568 B   step_step 592 B / step_sweep 424 B
MotorIdent_Start / Task1ms / Abort / GetResult / GetState / StateStr / ErrStr  全部进映像

数学库调用（仅 1 ms 上下文）：
  step_step  → __hardfp_sinf / __hardfp_cosf / __hardfp_atan2f / __hardfp_floorf
  step_sweep → 同上
```

Code 63984 → **72892 B（+8908）**，主要是 `sinf/cosf/atan2f` 的有理逼近实现；
Flash 占用 74756/131072 = **57%**，余量仍充足。

> **顺带发现（既有问题，非本次引入）**：`servo_isr.o(Servo_ControlTick)` 也引用
> `__hardfp_sinf/cosf` —— 那是开环分支的 `cosf(te)/sinf(te)`，位于 20 kHz 路径。
> 辨识期间每拍调用 2 次，约 +400 cyc；当前节拍 10 kHz（模型仍是 double，单拍 8862 cyc），
> 加上后仍在 8500 cyc 预算内，不影响辨识执行。若要彻底消掉，可改用
> `arm_sin_f32/arm_cos_f32`（配置⑤实测可从 8847 降到 1813 cyc）。

---

### 6.3 实机结果（COM27，2026-11-15 16:5x）

#### 第一次跑：MOE 一开就过流

```
Z → [CMD] IDENT FAIL fault_run n=0/0 moe=0
    ovc=-20.88/-9.48   raw=1261/1642
```

`raw=1261` → 1.02 V，明确偏离 2048 偏置，是**真实电流**而非噪声尖峰。
而 MOE=0 时电流只有 0.01 A。

#### 根因：PWM2 与 PWM1 语义错配（比辨识本身更根本）

```c
/* Core/Src/tim.c（CubeMX 生成；.ioc 里 4 个通道都是 PWM2） */
sConfigOC.OCMode      = TIM_OCMODE_PWM2;
sConfigOC.OCPolarity  = TIM_OCPOLARITY_HIGH;
sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;   /* 互补正常 */

/* Board/bsp_servo.c（按 PWM1 语义写的） */
ccr[i] = x * BSP_PWM_ARR;        /* 期望：上管导通比例 = x */
```

PWM2 下 `CNT < CCR` 时 `OCxREF = 0`，所以**上管导通时间 = ARR − CCR**，
实际输出是 `1 − x`。三相同步反相 → 线电压符号翻转 → 电流环从负反馈变**正反馈**。

`x = 0.5` 时 `1−0.5 = 0.5` 完全对称，**静止时看不出来**；一旦闭环给出非 50% 的占空比
就立刻发散。这正好解释"MOE=0 电流干净、MOE=1 立刻过流"，极易误判成上下桥直通。

修复（只改 Board 层，不动 TIM1 初始化，避免影响 CH4 的 ADC 触发时机）：

```c
ccr[i] = (uint16_t)((1.0f - x) * (float)BSP_PWM_ARR + 0.5f);
```

> 若将来把 `.ioc` 改成 PWM1，这里必须同步改回 `x * ARR`，否则二次反相。

#### 第二次跑：辨识成功

```
[CMD] IDENT ok ratio=13.98 pn=14 dir=+1 off=90.93deg zero=-1.88deg n=60/84 bad=0 moe=0
[CMD] id st=DONE ratio=13.98 pn=14 resid=0.02 dir=+1 dmin=4.36 dmax=12.74 navg=8.58deg n=60/84 bad=0
```

- `pn = 14` 与硬件一致，`resid = 0.02`
- `dir = +1`：相序与编码器同向，无需翻转
- `navg = 8.58°`，理论值 `360/(3×14) = 8.571°`，误差 0.1%
- 两次独立辨识的 `zero`：**−1.94° / −1.88°**，差 0.06°，自洽

#### 第三次跑：辨识后立即使能（关键验证）

| 指标 | 修复前（旧 `Z` 之后 `E`） | 本次 |
|---|---|---|
| `iq_tar` | **±9.000**（顶到限幅） | **+0.03** |
| 转速 | −301 → **−436 rpm** | **−0.4 rpm** |
| 位置 | 退 **2.7 圈** | 0.4 s 内 **−108 counts** |
| 故障 | `f=0x01` 过流 | **`f=0x00`** |
| 电流 | 尖峰 16 A、峰值 43.8 A | iu/iw ≈ 0.1 A |

**零点正确 → 位置误差≈0 → 位置环不要求力矩 → 电机静止。**

#### 一个必须记录的调参事实

`MI_SETTLE_MS` 从 30 → 60 ms：`ratio` 由 **12.66（resid 0.34，判 ratio_bad）**
变为 **13.82（resid 0.18）→ 13.98（resid 0.02）**，`dPulse` 净位移由 **527978 → 110 counts**。

30 ms 时转子在步进后仍处于爬行/过冲状态，读数取决于到达方向（齿槽 + 摩擦滞回），
正反两组系统性不等，平均后偏大 10.6%。**60 ms 是这套减速器负载下的下限**，
再短会直接让 `resid` 超容差。

---

## 7. 操作流程

```
C   →  标定电流零点（MOE=0，电机静止，200 ms）
X   →  清故障（若 f != 0）
Z   →  辨识（约 4.7 s，期间转子会正反各转约 1.5 圈机械角）
I   →  看结果，重点看三项：
        resid < 0.05       极对数是干净的整数
        bad = 0            每一拍都真的推动了转子
        dir = +1 / -1      若为 -1，说明相序与编码器反向（已由软件符号处理）
E   →  使能（建议先低压限流）
```

**安全约束（与之前一致，未放松）**
- `Z` 要求：`s_armed = 0`（先 `D`）、已完成 `C`、`fault = 0`
- 全程走虚拟角 dq 电流环，`id` 限幅在 [0.5, 5.0] A，`iq = 0`
- 每 1 ms 检查故障位，任何故障立即封锁 MOE
- 硬超时 20 s
- 结束一律 `MOE = 0`，要继续转必须重新 `E`

---

## 8. 局限与后续

1. ~~未实机验证~~ → **已实机跑通**，见 §6.3。板上固件 Code 75132 B，
   `C → Z → I → E → D` 全流程验证通过。
2. **步进角差的离散度仍偏大**：`dmin=4.36° / dmax=12.74°`，理论值 8.57°。
   均值准（8.58°）但单点散布 ±4°，说明 60 ms 只是"够用"。要继续改善可以
   ① 再延长到 100 ms；② 电流 2 A → 3 A；③ 把圆形平均换成"去掉两端 10% 后再平均"。
3. **极对数没有回写到模型**：`three_loop_deploy.c:48` 硬编码 `14.0`。
   辨识出的 `pn` 目前只用于回显与零点换算。若实测不是 14，必须改模型重新生成
   （或在模型入口加一个 `ratio_actual/14` 的增益层）。
3. **`dir` 为 −1 时的速度模式语义**：`F/R` 给的是"signed 坐标系"下的转向，
   与物理转向相反。位置保持不受影响。要不要在 `Servo_SetVelRef` 里也乘 `dir_sign`
   需要单独决定（涉及用户接口语义）。
4. **零点 / 标定 / dir 都只在 RAM**：每次复位要重跑 `C` + `Z`。量产应存 Flash
   （`board_public/flash_db` 那个空分组正是为这个准备的）。
5. **没有合理性交叉校验**：`ratio` 只检查"接近整数 + 在 1~64 内"。
   若硬件异常导致转子被拖动到别的稳定点，`ratio` 可能是个"漂亮的错误值"。
   建议后续加一条"ratio 应等于名义值 ±1"的告警（不阻断）。
6. **耗时 4.7 s 偏长**：VESC 每点 100 ms 是因为它跑在 1 kHz 级循环上；
   X72 的电流环是 20 kHz，30 ms 稳定已经足够。若嫌慢可降到 20 ms（约 3.2 s），
   但要先确认机械端在 20 ms 内确实停稳。
