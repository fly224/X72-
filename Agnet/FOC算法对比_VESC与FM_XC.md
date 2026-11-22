# FOC 算法对比：VESC 与 FM_XC

> 2026-11-15 · VESC = `bldc-master`（vedderb/bldc，通用无感驱动器）
> FM_XC = `FM_XC-master/FM_Servo_V3`（工业伺服）
> 只比 FOC 主干，辨识部分见 `对比_三种辨识方案_FM_XC与VESC.md`。

---

## 1. 结论先行：这不是"谁更好"，是两个物种

| | VESC | FM_XC |
|---|---|---|
| 目标场景 | **不知道电机参数、可能没有编码器**的通用驱动（滑板/平衡车/机器人关节） | **电机已知、编码器确定**的工业伺服（刚性定位、轨迹跟踪） |
| 核心能力 | 在线辨识 + 无感观测器 + 弱磁 | 刚性三环 + 轨迹规划 + 振动抑制 |
| 角度从哪来 | **观测器算的**（flux observer + PLL），编码器只是可选校正 | **编码器直接读**（MT6835 / 多摩川绝对值） |
| 零速怎么办 | HFI 高频注入 | 不需要，编码器本来就知道 |

一句话：**VESC 花大力气"猜转子在哪"，FM_XC 直接"问编码器"**。
这个分歧决定了后面所有的算法差异。

---

## 2. 逐项对比

| 维度 | VESC | FM_XC |
|---|---|---|
| FOC 主体位置 | `motor/foc_math.c`(815) + `mcpwm_foc.c`(5435) | `simulink_code_gen/current_ctl_loop_task.c`(501) |
| 角度来源 | `foc_observer_update()` + `foc_pll_run()` | 编码器 → `elec_theta_rad` |
| 零速/低速 | `foc_hfi_adjust_angle()` 高频注入 | 编码器直读，无此问题 |
| 弱磁 | **有**，`foc_run_fw()`，含 `i_fw_max` 电流回退 | **无**（伺服不需要高速弱磁） |
| MTPA | 有（`foc_mtpa_mode`） | 无（表贴式 Ld=Lq，不需要） |
| SVPWM | **六扇区查表**（`foc_svm()`，`switch(sector)` 算 tA/tB/tC） | **max-min 零序注入**（`svm`，求三相 min/max 再注入中点） |
| 电流环 | `foc_current_kp` / `foc_current_ki`（`mc_configuration`） | `kp_ld/ki_ld` × `bandwidth_percentage` |
| 参数从哪来 | detect 时由 R/L 自动算 → 写入 config | 人工整定 + 上位机 MAVLink 写入 |
| 电机模型 | `foc_motor_r` / `foc_motor_flux_linkage` / `foc_motor_ld_lq_diff` 都在配置里 | 只在生成代码字面量里（`Ld=Lq=113.5µH`） |
| PWM 频率 | `foc_f_zv` 可配，采样模式 `foc_control_sample_mode` 可切换（单/双采样） | 20 kHz 固定（`Ts_i=50µs`） |
| 外环 | 位置 P + 速度 PID（`foc_run_pid_control_pos/speed`） | PIP |
| 数据类型 | `float`（编译加 `-fsingle-precision-constant`） | `real32_T` |
| 在线调参通道 | VESC Tool（USB / UART / CAN） | MAVLink（ServoHub 协议） |

---

## 3. 三个根本差异

### 3.1 角度：观测器 vs 编码器

```c
/* VESC  foc_math.c:35 —— 反电动势积分出磁链，再 PLL 出角度 */
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, ...) {
    // flux = ∫(v - R·i)dt - L·i
    // 再由磁链矢量求角度，PLL 平滑
}
```

VESC 的 `foc_observer_update()` 是它最核心的代码：用 `v - R·i` 积分出定子磁链，
减去 `L·i` 得转子磁链方向，再用 PLL 跟踪。配套还有：
- `foc_hfi_adjust_angle()`：零速时反电动势为零、观测器失效，改用高频电压注入、
  靠 `Ld≠Lq` 的凸极效应判方向
- `foc_correct_encoder()` / `foc_correct_hall()`：有传感器时用观测器做**互补滤波**，
  兼顾低速精度与高速平滑

FM_XC 完全没有这一层——`elec_theta_rad` 就是编码器读数乘极对数，
加一个 `ω·dt·comp` 的延时补偿（`current_ctl_loop_task.c:130`）。

**代价对比**：
- VESC 能在"没传感器"时工作，但要处理观测器失锁、HFI 噪声、参数漂移
- FM_XC 的精度直接等于编码器精度（MT6835 是 21 位），但**没有传感器就完全不能转**

### 3.2 弱磁：VESC 有，FM_XC 不用

```c
/* VESC  foc_math.c:717 */
void foc_run_fw(motor_all_state_t *motor, float dt) {
    // 母线电压不足时，给负 id 削弱磁链，把转速推过基速
    // i_fw_max 会根据 iq 实际跟踪情况回退（见 foc_math.c:746 的 i_err_backoff）
}
```

VESC 面向的是"电池电压会掉、要跑高速"的场景（滑板/电动车的恒功率区）。
FM_XC 是伺服，工作区在基速以下，且 `id` 恒为 0（`err_d = 0.0 - id`），
根本没有弱磁的必要。

**这不是缺陷，是场景不同**。X72 同理，做关节模组不需要弱磁。

### 3.3 参数：自动辨识 vs 带宽百分比

| | VESC | FM_XC |
|---|---|---|
| 电流环增益 | `foc_current_kp/ki`，detect 时按 `R`、`L` 算好写进 config | `kp_ld/ki_ld` 人工整定，再乘 `bandwidth_percentage` |
| 电机参数 | R、λ、Ld−Lq **都在运行时配置里** | 只在生成代码字面量里 |
| 现场调整 | 改 `foc_current_kp` 直接生效 | 改 `bandwidth_percentage`，等价于整体缩放带宽 |

两条路各有道理：
- VESC 必须自动算，因为**它不知道你接的是什么电机**
- FM_XC 可以人工整定，因为**它是配自己的电机卖的**

FM_XC 的 `bandwidth_percentage` 是一个很聪明的折中：保留人工整定的基线，
又给现场留一个"整体调软/调硬"的旋钮，不用改 Kp/Ki 两个数。

---

## 4. 相同的地方（比想象中多）

1. **Clarke 都是幅值不变**：`ibeta = (ia + 2·ib) / √3`
2. **都是 dq PI + 解耦**：`vd = ... - iq·ω·Lq`、`vq = ... + id·ω·Ld + ω·ψf`
3. **都做电压圆限幅 + 积分器抗饱和**
4. **都用 float**（VESC 靠编译选项，FM_XC 用 `real32_T`）
5. **都是参数化 + 上位机可调**，只是通道不同（VESC Tool vs MAVLink）
6. **SVPWM 在 VESC 里也是六扇区**（`foc_math.c:255`，和 X72 的 `three_svm_deploy` 判据一字不差）

第 6 点值得单独说：**六扇区是绝对主流**（VESC 这种跑在百万台设备上的固件都用它），
FM_XC 的 max-min 是另一派写法，胜在代码短、无分支，但不代表六扇区"不成熟"。

---

## 5. 对 X72 的启示

X72 是**有编码器的伺服关节**，定位上更接近 FM_XC，所以：

| 建议 | 依据 |
|---|---|
| ✅ 走 FM_XC 的路线（编码器 + 三环 + 刚性） | X72 有 MT6835，不需要观测器 |
| ✅ 学 FM_XC 的**参数化**（`dt_s`、带宽百分比进配置） | X72 现在全是生成代码字面量 |
| ✅ 学 FM_XC 的**死区补偿** | VESC 也有类似的电压补偿，两边都做，X72 独缺 |
| ⚠️ SVPWM **不必**改成 max-min | VESC 也用六扇区，X72 现在的实现没问题 |
| ❌ 不要引入观测器/HFI/弱磁 | 有编码器、工作在基速以下，纯属增加复杂度 |
| 💡 可借鉴 VESC 的**相序处理** | 交换 `ia/ib` 而不是翻 θ，影响面最小 |

**唯一想从 VESC 拿的东西**：它的 `foc_current_kp/ki` 是**由 R 和 L 算出来的**，
而不是拍脑袋给的。X72 现在的 `kp_i=0.713 / ki_i=2378` 也是这么来的
（`ωc·L` 与 `ωc·R`），但**这个推导过程没有写进代码**。
建议在 `three_loop_params.m` 里用公式表达而不是写死数字：

```matlab
wc = 2*pi*1000;              % 目标带宽 1000 Hz
P.kp_i = wc * P.Ld;          % 0.713
P.ki_i = wc * R_winding;     % 2378（R=0.378）
```

这样换电机时改 `Ld` 和 `R` 就行，不用重新整定。

---

## 6. 附：两边 FOC 参数项对比

| 配置项 | VESC（`mc_configuration`） | FM_XC（`CurrentCtlConfig`） |
|---|---|---|
| 电流环增益 | `foc_current_kp` / `foc_current_ki` | `kp_ld` / `ki_ld` / `kp_lq` / `ki_lq` |
| 带宽缩放 | — | `bandwidth_percentage` |
| 控制周期 | `foc_f_zv`（PWM 频率） | `dt_s` |
| 电机电阻 | `foc_motor_r` | — |
| 磁链 | `foc_motor_flux_linkage` | — |
| 凸极差 | `foc_motor_ld_lq_diff` | —（Ld=Lq 硬编码） |
| 编码器标定 | `foc_encoder_offset` / `_ratio` / `_inverted` | 由辨识写入 `BspServo_SetEncoderZero` |
| 角度补偿 | — | `elec_angle_compensation` |
| 相序 | `foc_encoder_inverted` | `phase_dir` |
| 最大占空比 | `l_max_duty` | `pwm_duty_cycle_max` |
| 弱磁 | `foc_fw_*` 一组 | — |
| HFI | `foc_hfi_*` 十来个 | — |
| MTPA | `foc_mtpa_mode` | — |
