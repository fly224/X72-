# 两套辨识方案对比：FM_XC（自研） vs VESC（开源）

> FM_XC：`FM_Servo_V3`（STM32H743 / Y01），三个独立辨识流程
> VESC：`bldc-master`（vedderb/bldc），一个函数同时出 offset / ratio / inverted
> 对比日期：2026-11-15

---

## 0. 结论先行

| | FM_XC | VESC |
|---|---|---|
| **组织方式** | 3 个独立流程，有隐含依赖顺序 | **1 个流程（`mcpwm_foc_encoder_detect`）同时出 3 个结果** |
| **极对数** | `pn = enc_line / Δcount`（计数差） | `ratio = round(120° / 角度差)`（**角度差**） |
| **方向** | 单独跑一圈看 Δcount 符号 | 就是 `ratio` 的符号（`inverted`） |
| **零点** | 0° 附近正反各偏 0.5 rad，**2 个样本** | **满机械圈**正反各一遍，**2×ratio×3 个样本** |
| **角度平均** | 算术平均 + 跨零特判 | **圆形平均 `atan2(Σsin, Σcos)`** |
| **耗时** | 约 3~5 s | 约 30~60 s |
| **成熟度** | 算法完整但**参数全 0、未调通** | 量产验证，还有 `enc_corr[360]` 二级修正 |

一句话：**VESC 用"扫满一圈 + 圆形平均"换取精度，FM_XC 用"局部两点"换速度，但两者都建立在同一套机制上——覆盖电角度、给 d 轴电流、看编码器。**

---

## 1. 共同的底层机制

两个工程**完全一致**的三个动作：

```c
/* 1. 覆盖电角度（不给编码器算，直接注入） */
FM_XC : current_ctl_input.elec_theta_rad = 辨识模块生成的虚拟角度
VESC  : motor->m_phase_now_override = i;

/* 2. d 轴给电流，q 轴给 0 */
FM_XC : idq_tar_A[0] = 额定电流;  idq_tar_A[1] = 0;
VESC  : m_id_set = current;      m_iq_set = 0.0;

/* 3. 进电流模式，让转子被磁场"拖住" */
FM_XC : current_ctl_input.mode = 1;
VESC  : m_control_mode = CONTROL_MODE_CURRENT;
```

所以"能不能做辨识"的分水岭不是算法，而是**电流环是否支持外部注入 θ**。X72 今天补的 `Servo_SetOpenLoop()` 就是补这一层。

---

## 2. VESC 的完整流程（`motor/mcpwm_foc.c:1519`）

### 2.1 前置：找编码器 index（L1550）

```c
while (!encoder_index_found()) {
    for (i = 0 .. 2π, 步长 2π/500) { phase_override = i; delay 1ms; }
    if (++cnt > 30) break;       /* 转 30 圈还找不到就放弃 */
}
```

增量编码器的 Z 信号定位。**MT6835 是 21 位绝对值编码器，这一步 X72 不需要。**

### 2.2 测 ratio 与 inverted（L1590–1664）

```c
for (i = 0; i < 30; i++) {
    phase_old = m_phase_now_encoder;
    /* 电角度前进 120°（2π/3），每步 2π/500，每步 sleep 1 ms */
    for (j = cur; j < cur + 2π/3; j += 2π/500) { override = j; delay 1ms; }
    norm(override);  delay 300ms;              /* 等转子停稳 */

    diff = angle_diff(m_phase_now_encoder, phase_old);   /* 编码器电角度实际走了多少 */
    s_sum += sinf(diff);  c_sum += cosf(diff);           /* 圆形累加 */
}
/* 反向再走一遍 30 次，同样累加 */

diff = RAD2DEG(atan2f(s_sum, c_sum));
*inverted = (diff < 0);                       /* 方向：120° 走成负的 */
*ratio    = roundf(120.0 / fabsf(diff));      /* 比例：120° 实际走了 diff 度 */
```

**关键点**：

- 每次只走 **120° 电角度**，不是整圈 → 单次动作小、冲击小。
- `ratio` 用 **角度差** 而不是计数差：`ratio = 120° / |diff|`。14 极对时 `diff ≈ 8.57°`，`ratio = 14`。
- 正反各 30 次，共 60 个样本，**用圆形平均合并**。
- 自适应提前退出：`if (i > 3 && |enc − first| < |diff/2|) break;` —— 编码器转满一圈就停。

### 2.3 测 offset（L1686–1750）

```c
it_ofs = ratio * 3;                       /* 采样点数正比于极对数 */
step   = 2π * ratio / it_ofs = 2π / 3;    /* 每步 120° 电角度 */

/* 正转一整圈（it_ofs 步 × 120° = ratio 圈电角度 = 1 圈机械） */
for (i = 0; i < it_ofs; i++) {
    override = i * step;
    step_towards(m_phase_now_override, override, step/100);   /* 平滑逼近，每步 4ms */
    delay 100ms;
    diff = angle_diff(m_phase_now_encoder, m_phase_now_override);
    s_sum += sinf(diff);  c_sum += cosf(diff);
}

/* 反转一整圈，同样累加 */
for (i = it_ofs; i > 0; i--) { ... }

*offset = RAD2DEG(atan2f(s_sum, c_sum));
norm(offset);                             /* 归一化到 [0,360) */
```

**与 FM_XC 的根本差别**：

| | FM_XC | VESC |
|---|---|---|
| 采样范围 | 只在 0° 附近 ±0.5 rad | **整个机械圈**（每个电周期采 3 点） |
| 样本数 | 2（正反各 1） | `2 × ratio × 3`（14 极对 = 84 个） |
| 平均方式 | 算术平均（还要特判跨零） | **圆形平均**，天生处理 0/2π 环绕 |
| 抗局部误差 | 差（只看一个位置） | 好（齿槽/偏心被平均掉） |

### 2.4 角度换算（`mcpwm_foc.c:3275`）

```c
phase = enc_deg;                          /* 编码器机械角 */
if (inverted) phase = 360 - phase;        /* 方向 */
phase *= ratio;                           /* 机械 → 电角度 */
phase -= offset;                          /* 零点 */
if (enc_corr_en) phase -= enc_corr[(int)enc_deg];   /* 每 1° 误差修正表 */
norm(phase);
elec_angle = deg2rad(phase);
```

三个参数各司其职，与 FM_XC 的 `elec_angle = enc/enc_line*2π*pn − bias` 在数学上等价，只是 VESC 用度、FM_XC 用弧度。

### 2.5 附带：霍尔表（`mcpwm_foc.c:2384`）

同一套 `atan2(Σsin, Σcos)` 再用一次：正反各转 3 圈（2160 个点），把每个霍尔状态对应的电角度分别累加，最后每个状态取圆形平均得到一个中心角。

---

## 3. FM_XC 的流程回顾

| 流程 | 判据 | 样本 |
|---|---|---|
| 方向 | 电角度走 1 圈，`Δcount` 的符号 | 2 次回绕（1 次建基准 + 1 次判定） |
| 极对数 | `pn = enc_line / Δcount`，每个电周期一次 | 4 个电周期取平均 |
| 电角度 | 0° 吸合 → 正/反各偏 0.5 rad → 回零 → 取两点平均 | **2 个** |

隐含依赖：**必须先方向**（否则 `Δcount < 0` 会让 `pn` 算出负数）→ **再极对数** → **最后电角度**（offset 需要 ratio 才能换算）。

VESC 把这三次序**内化到一个函数**里：先用 120° 步进同时定 ratio 与方向，再用 ratio 算出采样点数去测 offset。

---

## 4. 逐项打分

| 维度 | FM_XC | VESC | 说明 |
|---|---|---|---|
| 极对数精度 | 中 | 高 | VESC 用角度差，不受编码器丢脉冲影响；FM_XC 的 `Δcount` 小的时候取整误差大 |
| 方向可靠性 | 高 | 高 | 都是看符号，原理相同 |
| 零点精度 | 低~中 | 高 | FM_XC 只有 2 个点，局部齿槽误差直接进结果 |
| 抗角度环绕 | 需特判（`|b1-b0| ≥ π`） | 天然（atan2） | 圆形平均是更正确的数学工具 |
| 耗时 | 3~5 s | 30~60 s | VESC 每点 100 ms 稳定，慢但稳 |
| 对电机冲击 | 小 | 小 | 都只用额定电流吸合 |
| 代码完整度 | 算法在，**参数全 0** | 完整 | FM_XC 实际跑不起来 |
| 额外能力 | 无 | 霍尔表、每度误差修正 | |

---

## 5. 两套都没解决、但都该有的东西

1. **结果持久化**：两者都只写 RAM（VESC 靠 `conf_general_store_config()` 另存 EEPROM，FM_XC 完全没有）。
2. **辨识前置校验**：都应该先确认"电流环是稳的、编码器在动"，否则辨识出垃圾值也不报错。
3. **失败判定**：FM_XC 的 `pn=1` 兜底、VESC 的 `cnt>30` 放弃，都只是超时，没有"结果合理性检查"（如 `ratio` 应接近整数、`|diff|` 应在合理范围）。

---

## 6. 对 X72 的结论：融合两者

X72 现在是"单点吸合"（等于 FM_XC 的简化版，只吸到 0° 就完事）。建议：

### 6.1 电角度零点：学 VESC 的"整圈扫 + 圆形平均"（但不必这么慢）

```c
/* 不用每点 100 ms，20 kHz 电流环下 30 ms 就够停稳 */
it_ofs = (uint32_t)roundf(pn * 3.0f);          /* 每电周期 3 点 */
step   = 6.28318548f / 3.0f;                    /* 120° */

for (pass = 0; pass < 2; pass++) {              /* 正一遍、反一遍 */
    for (i = 0; i < it_ofs; i++) {
        uint32_t idx = pass ? (it_ofs - i) : i;
        set_override(idx * step);               /* 平滑逼近 */
        wait_stable(30);                        /* 30 ms */
        float d = angle_diff(elec_angle_now, override);
        s_sum += sinf(d);  c_sum += cosf(d);
    }
}
bias = atan2f(s_sum, c_sum);
```

14 极对 × 3 × 2 = 84 点 × 30 ms ≈ **2.5 s**，精度接近 VESC，速度接近 FM_XC。

### 6.2 极对数：用角度差代替计数差

```c
/* 走 120° 电角度，量编码器电角度实际变化 */
ratio = roundf((float)M_PI * 2.0f / 3.0f / fabsf(diff));
```

好处：丢一两个脉冲不会让结果跳变（`Δcount` 法会）。

### 6.3 方向：直接用 `bias` 或 `ratio` 的符号

不需要单独的"方向辨识流程"——`diff < 0` 就是反了，跟 VESC 一样。

### 6.4 三步合并成一个 `Z` 流程

```
① 120° 步进 60 次 → ratio + inverted（约 8 s）
② 按 ratio 扫整圈 → offset（约 2.5 s）
③ 写 flash
```

这样 `Z` 一次就把 VESC 的三个参数全拿到，比 FM_XC 的三个独立命令更好用。

---

## 12. 2026-11-15 16:40 更新：已按 VESC 策略落地

本文的对比结论已实现为 `Board/motor_ident.c/.h`（`Z` 命令），要点：

- 采用 VESC 的三段式（120° 步进 → ratio/dir；扫整圈 ×2 → offset），
  而不是 FM_XC 的三个独立流程；
- 圆形平均 `atan2(Σsin, Σcos)`，不做 `|b1-b0| >= π` 特判；
- 极对数用**角度差**而非计数差；
- 补上了两边都没处理的两件事：**模型固定 π/2 补偿** 与 **dir=−1 时的符号翻转**。

验证与局限见 `Agnet/VESC策略辨识_实现说明.md`。
