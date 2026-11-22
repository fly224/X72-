# FOC 算法对比：X72 与 FM_XC

> 2026-11-15 · X72 = `G:/FOC/X72`（STM32G474，当前在调）
> FM_XC = `FM_XC-master/FM_Servo_V3`（H743，产品级伺服参考）
> 两边都读源码逐行核对，行号可复核。

---

## 1. 结论先行

**两边的控制律在数学上是同一套东西**：Clarke（幅值不变）→ Park → dq PI + 解耦 →
电压限幅 → 反 Park → SVPWM。差异不在公式，在**五个工程决策**：

| # | 决策 | X72 | FM_XC | 后果 |
|---|---|---|---|---|
| 1 | 数据类型 | `real_T = double` | `real32_T` | X72 单拍 9282 cyc，**只能跑 10 kHz**；FM_XC 20 kHz |
| 2 | `dt` 来源 | 硬编码在生成代码 | 运行时 `config.dt_s` | X72 改频率要改 4 处并手改生成物 |
| 3 | PI 形式 | 并联（P、I 独立） | **串联**（I 的输入是 P 的输出） | FM_XC 改带宽不动 `ki`，调参更安全 |
| 4 | 死区补偿 | **无** | **有**（线性 + 饱和） | X72 低速有小幅力矩脉动 |
| 5 | SVPWM | 六扇区查表 + 时间计算 | **max-min 零序注入** | FM_XC 代码短一半、无扇区判断 |

前两条是**根因级**：X72 的 10 kHz、三条 dt 错配、坐标系 bug，全都源于 `double`。

---

## 2. 两个工程的定位差异

| | X72 | FM_XC |
|---|---|---|
| MCU | STM32G474CEU6（M4F，170 MHz，单精度 FPU） | STM32H743（M7） |
| 代码来源 | Simulink `fcn_*_deploy.m` → 生成 | 同样 Simulink 生成，但更细分 |
| 三环源文件 | 3 个 `.m`（237 + 60 + 47 行） | `current_ctl_loop_task.c`(501) + `pos_speed_ctl_pip.c`(411) |
| 业务层 | `Board/` 13 个文件、约 3100 行，全部堆在一起 | `app/ board_public/ communication/ database/ motor_control/` 分层，0 空目录 |
| 功能面 | 三环 + 辨识 + 遥测 | 三环 + 辨识 + **陷波** + **输入整形** + MIT 模式 + CiA402 CSP/CST/CSV + 保护表 |

FM_XC 是"已经跑完认证阶段的产品"，X72 是"刚把功率级点亮的样机"。
下面的对比只看**算法与工程实现**，不比功能多少。

---

## 3. 主链路逐级对比

### 3.1 Clarke

| | 实现 | 说明 |
|---|---|---|
| X72 | `i_alpha = ia; i_beta = 0.5774·ia + 1.1547·ib` | 双电阻采样，`ic` 由 `-(ia+ib)` 重构 |
| FM_XC | `ibeta = (ia + 2·ib) · 0.5774` | 同一个式子，`(ia+2ib)/√3` |

**完全等价**（`0.5774·ia + 1.1547·ib == (ia+2ib)·0.5774`），只是写作习惯不同。

### 3.2 Park 与角度补偿

```c
/* X72  three_loop_deploy.c:48 */
theta_sum = 14.0*theta_m + 1.57079632679 + 0.0021*omega_m;
                ↑极对数        ↑固定π/2        ↑延时补偿

/* FM_XC  current_ctl_loop_task.c:130 */
u_tar_comp_a = elec_angle_speed_rad_s * config->dt_s * config->elec_angle_compensation
             + elec_theta_rad;
```

**结构一样，可调性差一个量级**：
- X72 的 `14`、`π/2`、`0.0021` 全是**生成代码里的字面量**，改一个要动模型或打补丁
- FM_XC 的 `dt_s`、`elec_angle_compensation` 都在 `CurrentCtlConfig` 里，**上位机可改**

> X72 上一轮就因为 `Ts_i` 从 50 µs 变 100 µs，这个补偿要从 0.00105 改成 0.0021，
> 差点漏掉（`Agnet/电流环整定与三环频率修正.md` §5.1）。

### 3.3 dq 电流环：并联 vs 串联

```c
/* X72：并联（P 与 I 各自独立累加）*/
vd_int += (0.0 - id) * 2378.18563877 * 1.0E-4;   /* I 直接吃误差 */
vd     = (0.0 - id) * 0.713141532365 + vd_int;    /* P 再加进来 */

/* FM_XC：串联（I 吃的是 P 的输出）*/
p = (id_tar - id) * bandwidth_percentage*0.01F * kp_ld;   // ① 先算 P
i = ki_ld * p * dt_s + id_ki_int;                          // ② I 积分 P
u = i + p;                                                 // ③ 相加
```

两者在数学上可以互相换算，但**调参手感完全不同**：

| | 改 Kp 时 | 改带宽时 |
|---|---|---|
| 并联（X72） | 零点 `Ki/Kp` **跟着漂**，可能破坏极点抵消 | 要同时改 Kp 和 Ki |
| 串联（FM_XC） | `ki` 是积分时间倒数，零点不变 | `bandwidth_percentage` 一个数搞定 |

这就是 FM_XC 敢把 `bandwidth_percentage` 暴露给上位机的原因。

### 3.4 限幅与抗饱和

```c
/* X72：先每轴粗限，再做圆形限幅（d 轴优先），积分器同步钳位 */
lim_d = max_v_mag * mag_vd_max;
if (vd > lim_d)  vd = lim_d;  if (vd_int > lim_d) vd_int = lim_d;
max_vq = sqrt(max_v_mag² - vd²);
if (vq > max_vq) vq = max_vq; if (vq_int > max_vq) vq_int = max_vq;

/* FM_XC：每轴先限到 ±vbus·max_duty·2/3，再走 udq_max_lim 做圆形限幅 */
rtb_Gain = vbus * pwm_duty_cycle_max * 0.6667;
if (add3 > rtb_Gain) add3 = rtb_Gain;  ...  // I 项限幅
u_tar = add3 + p;                            // I + P
if (u_tar > rtb_Gain) u_tar = rtb_Gain;  ...  // 合成后限幅
→ udq_max_lim()                              // 圆形限幅
```

**X72 的抗饱和更细**：`mag_vd_max=0.98` 给 d 轴留 2% 余量，且积分器与输出**同步钳位**，
退出饱和时不会积分回吐。FM_XC 是"先限后算圆"，结构更简单但 d 轴没有单独优先级。

### 3.5 SVPWM：这是最明显的算法差异

| | 方法 | 代码量 | 特点 |
|---|---|---|---|
| X72 | **六扇区 + 相邻矢量时间计算**（`three_svm_deploy`） | 约 60 行 | 需判扇区、算 t1~t6，边界多 |
| FM_XC | **max-min 零序注入**（`svpwm`，注释写"最大值最小值法"） | 约 25 行 | 无扇区判断，先求三相 min/max 再注入中点 |

max-min 法等价于 SVPWM（都是注入零序把线性区扩到 `2/√3`），但：
- 代码短一半，**没有 6 个分支**，跑在 20 kHz 里更省周期
- 不会因为扇区边界判断错误而输出跳变

X72 用六扇区是因为它是从仿真模型直接生成、沿用了教学式写法。

### 3.6 死区补偿（X72 完全没有）

```c
/* FM_XC  current_ctl_loop_task.c:31 */
if (i_noise < 1e-5)        { du = 0; }                    // 电流太小，不补
else {
    line_range = i_noise * 2;                             // 线性区宽度
    if (|i_now| <= line_range) du = (comp_du/line_range) * i_now;   // 线性插值
    else if (i_now > 0)        du = +comp_du;             // 饱和
    else                       du = -comp_du;
}
u_tar_comp = u_tar + du;
```

按电流符号补偿，`±2·i_noise` 内线性过渡，避免过零抖动。
**作用**：低速小电流时抵消死区造成的电压损失，减小力矩脉动和转速波动。

X72 的 `DeadTime = 50`（约 670 ns），在 3 rpm 这种低速下，死区导致的电压误差
占相电压的比例明显变大——这是 X72 低速转速波动的一个未处理来源。

---

## 4. 外环结构差异

| | X72 | FM_XC |
|---|---|---|
| 位置环 | 纯 P，`w_ref = kp_p·(pos_ref − theta)`，限幅 ±120 rad/s | 并入 `pos_speed_ctl_pip` |
| 速度环 | PI，`i_term += ki·err·Ts`，输出限幅 ±9 A | 同一个 PIP 函数 |
| 结构 | **串级**（位置 → 速度 → 电流，三个独立函数） | **PIP**（位置-积分-位置），带 `dynamic_integration_enable` |
| 分频 | 各自 `cnt` 计数，20/10 → 1 kHz / 2 kHz | 由 `mode` 与任务调度决定 |

FM_XC 的 `dynamic_integration_enable`（`pos_speed_ctl_pip.c:138`）是**动态积分/抗饱和**：
输出饱和时自动减弱积分，比 X72 的"积分钳位到同一个限幅"更平顺。

---

## 5. 工程化差异（这才是真正的差距）

| 项 | X72 | FM_XC |
|---|---|---|
| 参数存放 | 生成代码字面量 | `CurrentCtlConfig` / `pos_speed_ctl_pip_Config` 结构体 |
| 在线调参 | 无 | MAVLink `AppMotorConfig` 可改 |
| 滤波 | 测速一阶低通，`alpha=0.06` 硬编码 | `math_lib_first_order_lpf_iir`（截止频率可配）+ `math_lib_median_filter`（中值） |
| 陷波 | 无 | `notch_filter.c` 363 行，抑制机械共振 |
| 输入整形 | 无 | `input_shaping_ei.c`，抑制残余振动 |
| 相序适配 | `Servo_SetDirSign()` 翻转 θ/ω | `phase_dir > 0` 时交换 `ia`/`ib`（`current_ctl_loop_task.c:127`） |
| 开环模式 | 有（虚拟角 dq 电流环，用于辨识） | 有（`mode<=0` 直接走 `uabc_tar_V`） |
| 保护 | 过流 + 编码器 + ADC 同步 | `protect_check.c` 192 行，表驱动 |

**相序处理两种思路都成立**：
- X72 翻角度 → 编码器零点语义跟着变，位置环坐标系要一起翻（复杂度外溢）
- FM_XC 交换电流采样 → 只影响 Clarke 输入，编码器侧不动，**影响面更小**

这一条值得 X72 采纳：把 `dir_sign` 从"翻 θ/ω + pos_ref"改成"交换 ia/ib"，
可以一次性消掉之前那个 `pos_ref` 与 `theta_m` 坐标系对齐的补丁。

---

## 6. 对 X72 的可借鉴清单（按 ROI 排序）

| 优先级 | 改什么 | 预期收益 | 成本 |
|---|---|---|---|
| **P0** | `real_T` 单精度化 → 回到 20 kHz | 单拍 9282 → 1813 cyc，三处 dt 错配自动消失，相位裕度 ~30° → ~55° | 改 `rtwtypes.h` + 约 100 处字面量 + 3 个 `arm_*_f32`，重跑 SIL |
| **P1** | `dt` 参数化（从 config 读） | 改频率不用碰生成代码 | 中，需要引入配置结构 |
| **P1** | 死区补偿 | 低速力矩脉动、转速波动改善 | 小，约 30 行（照抄 FM_XC 的线性+饱和） |
| **P2** | SVPWM 换 max-min | 代码少一半、少 6 个分支 | 小，约 25 行 |
| **P2** | 相序改用交换 `ia/ib` | 消掉 `pos_ref` 坐标系补丁 | 小，3 行 + 去掉 `dir_sign` 外溢 |
| **P3** | 电流环改串联型 + 带宽百分比 | 调参直观、零点不漂 | 中，要改模型重新生成 |
| **P3** | 陷波滤波器 | 抑制机械共振（若实测有共振） | 大，需要辨识共振频率 |

**不建议照抄的**：PIP 外环（X72 现在的串级 P+PI 已经跑出 3.00 rpm 精确跟踪，
换结构收益不明）；输入整形（先确认机械真有残余振动）。

---

## 7. 附：关键参数对照

| 参数 | X72 | FM_XC |
|---|---|---|
| 控制周期 | **100 µs（实际 10 kHz）** | 50 µs（20 kHz，`database_init.c:30`） |
| 电流环 Kp | 0.713142 V/A | `kp_ld` / `kp_lq`（可配） |
| 电流环 Ki | 2378.19 V/(A·s) | `ki_ld` / `ki_lq`（积分时间倒数形式） |
| 带宽 | 固定 1000 Hz | `bandwidth_percentage` 可调 |
| 电感 | Ld=Lq=113.5 µH | 同量级（`kp_ld` 与之匹配） |
| 磁链 | 7.593 mWb | — |
| 极对数 | 14（`gain_p_theta`） | `pn`（辨识写入，运行时） |
| 电流限幅 | ±9 A | `i_max`（可配） |
| 最大占空比 | 0.95（`max_duty`） | `pwm_duty_cycle_max` |
| 电压基准 | `0.5484·vbus` | `vbus·max_duty·0.6667` |
| 死区 | 670 ns，**不补偿** | 670 ns 量级，**线性+饱和补偿** |
