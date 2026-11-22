﻿# 删除 ASCII 通道，USART1 只跑 MAVLink

**日期** 2026-11-21 23:5x · **固件** Code=106732 · **串口** COM27 @921600

---

## 0. 结果

| 项 | 修改前 | 修改后 | 变化 |
|---|---|---|---|
| **Code** | 119204 | **106732** | **−12472 B（−12.2 KB）** |
| RO-data | 5240 | **2224** | −3016 B |
| ZI-data | 27952 | 27432 | −520 B |
| `servo_cmd.c` | 3744 行 | **1291 行** | −2453 行（−65%） |
| `uart_link.c` | 568 行 | **325 行** | −243 行 |
| `servo_cmd.h` | 127 行 | **104 行** | −23 行 |

> 为什么之前 `X72_ASCII_DEBUG=0` 也省了 13.6 KB，这次还能再省 12.2 KB：
> 那个宏只包住了 `handle()` 的 57 个 case 与回显字符串，**`UartLink_Trace` 那套
> 完全不受宏控制**（20 处启动打点 + 一整套 `put_*` 格式化 + `ReportStatus` /
> `ReportDiag` / `ReportRegs`），一直编在固件里。

编译：`0 Error(s), 0 Warning(s)`。

---

## 1. 删了什么

### 1.1 `servo_cmd.c`（控制命令层）

| 删除对象 | 行数 | 说明 |
|---|---|---|
| `handle(uint8_t c)` | 524 | 57 条单字符命令的总分发 |
| `cmd_num_parse / apply / feed / begin` | 388 | 数字输入状态机（`W1.23` 这类带参数的写法） |
| `do_status()` | 153 | `S` 行的全部状态打印 |
| `do_step() / fmt_step_result / fmt_trace` | 107 | 阶跃响应测试的 ASCII 输出 |
| `do_ident_info() / do_help() / do_spd_info()` | 107 | 查询类命令 |
| `put_str / put_u / put_hex* / put_f*` | 109 | 极简格式化（只服务 ASCII） |
| `emit()` | 20 | 回显出口 |
| `ascii_pump / rec_pump / trace_pump` | 96 | 三套逐行发送泵 |
| `refuse_if_speed_loop / rec_ch_name` | 34 | 只用于拼提示语 |
| `do_mech_start()` | 22 | ASCII 侧的机械辨识入口（MAVLink 直调 `MechIdent_Start`） |
| `servo_cmd.c` 内 51 处打印块 + 7 处孤立 `emit` | — | 混合函数里的 `p = put_str(...) → emit(p)` 序列 |
| MIT 半帧 / POSF 半帧状态机 | 60 | 0x02 / 0x04 私有二进制帧 |
| `s_out` / `s_out_pending` / 健康计数 / `s_num_*` / `s_tr_*` / `s_rec_*` | — | ASCII 专属状态 |

### 1.2 `uart_link.c`（链路层）

| 删除对象 | 说明 |
|---|---|
| `UartLink_Init` 的 ASCII 横幅 | 上电不再输出 "X72 FOC Servo Driver / banner sent" |
| `UartLink_SendAscii` / `TrySendAscii` / `AsciiBusy` | ASCII 发送 |
| `UartLink_ReportStatus` | `[BOOT] bsp=OK tim6=OK ...` 启动状态行 |
| `UartLink_ReportDiag` | 控制节拍预算诊断行（含 `UartLinkDiag_t` 的使用） |
| `UartLink_ReportRegs` | ADC 寄存器快照 |
| `UartLink_Trace` | 20 处启动打点（`bsp:opamp ok` 之类） |
| `UartLink_GetAsciiCount` | ASCII 计数 |

### 1.3 调用点

| 文件 | 删除 |
|---|---|
| `bsp_servo.c` | 9 处 `UartLink_Trace` + 1 处 `ReportRegs`（连同只读寄存器的 `uint32_t r[7]` 块） |
| `servo_isr.c` | 8 处 `Trace` + 2 处 `ReportStatus` + 整个 `Servo_DiagTask1ms()` 及其调用 |
| `servo_cmd.h` | `X72_ASCII_DEBUG` 宏、`ServoCmd_GetDiag` / `ServoCmd_GetHealth` 声明 |

---

## 2. 保留了什么（以及为什么）

| 保留项 | 理由 |
|---|---|
| `ServoCmd_Feed()` 的 256 B 环形队列 | MAVLink 也走它；解析要动 CRC 与 156 B 缓冲，放 20 kHz 同优先级会超时 |
| `ServoCmd_Poll()` 的 deadman / 故障收尾 | 与控制安全相关，与通讯无关 |
| `UartLink_TrySendBinary()` | **MAVLink 应答帧的发送通道**，删了就没有 ACK |
| `UartLink_FaultDump()` + `RawPutChar` + `put_*`（`uart_link.c` 内私有） | **HardFault 崩溃取证**。系统已挂时 MAVLink 不可能工作，只剩寄存器轮询这一条路。它不属于"正常运行时的通讯" |
| `mit_engage()` | MAVLink 30065 与 `ServoCmd_CtrlMit` 共用 |
| `s_mit_bad` / `s_mit_why` | `mit_engage` 的联锁拒绝原因，仍写入供诊断 |
| `Servo_DiagTask1ms` 依赖的 DWT 测量 | 保留在 ISR 里（开销极小），只是不再有输出 |

> **如果你要连 `FaultDump` 也删**：说一声，`uart_link.c` 可再降到约 80 行
> （只剩 `TrySendBinary` + `Init` + `GetErrorCount`）。代价是 HardFault 时
> 串口一个字都没有，只能靠调试器接 SWD 看 CFSR。

---

## 3. 验证

### 3.1 上电输出（关键）

```
2 s 收到 65536 字节 (32.0 KB/s)
MAVLink 帧头 0xFD 出现 1274 次
  文本 X72      : 无
  文本 [BOOT]   : 无
  文本 ====     : 无
  文本 [CMD]    : 无
  文本 [T]      : 无
  文本 [REG]    : 无
  文本 bsp:     : 无
→ 纯二进制 MAVLink 流，无任何 ASCII 文本
```

**上位机不再需要跳过 banner**：复位后第一个字节就是 `0xFD`。

### 3.2 功能

| 项 | 结果 |
|---|---|
| 遥测 30090 / 30091 / 30100 | ✅ `fault=0`、`cal=1 z=1 cog=1` |
| 命令通道（30056 模式切换） | ✅ |
| `action(3)` 清故障 | ✅ `status=0` |
| `action(7)` R/L 辨识 | ✅ `ok=1 R=288.4 mΩ L=92.1 µH` |
| 位置模式使能 | ✅ 坐标对齐正常：`p_now` 变化 **−2π**（进入即单圈角），`err=+1.5 mrad` |
| 位置移动 +0.3 rad | ✅ 到位误差 **+0.83 mrad**，`st=DONE` |

> 验证中曾看到"使能后 p_now 跳 0.49 rad / err=−0.49"，排查后确认是
> **电机在 MOE=0 期间被外力（重力/手）转动**，两次会话间隔几分钟。
> 同一时刻连续测量时，`p_now` 只变化整 2π，`err` 为 1.5 mrad。
> 位置环本身无回归。

### 3.3 参数保全

烧录后 `cal=1 z=1 cog=1` 全部保留（本次未擦参数页）。

---

## 4. 行为变化（上位机需知）

| 变化 | 影响 |
|---|---|
| **上电无横幅** | 原来要跳过 `X72 FOC Servo Driver` 那几行；现在直接是帧 |
| **失败无 ASCII 提示** | 已全部改为 30092 `ok=0` + `v[0]=错误码`（上一轮已补） |
| **0x02 MIT 帧不再识别** | MIT 只能用 30065（带完整联锁）；0x02 本来就没有应答 |
| **0x04 PCMP 帧不再识别** | 补偿参数改用 30082 整数通道 6/7/8 |
| **`!` 复位命令失效** | 用 30069 `action=2`（软复位） |
| **`S`/`K`/`I` 等查询命令失效** | 用 30090 / 30091 / 30100 / 30082 读 |

---

## 5. 遗留（未处理，如实说明）

| 项 | 状态 |
|---|---|
| `s_mi_phase` / `s_mi_i` / `s_fi_*` | 变量还在，但只被已删的逐行输出用过 —— 现在无引用（静态变量不报警告） |
| `CurStep_*`（阶跃采集） | ISR 里仍在采集，但触发入口（ASCII `T`）已删，**目前无法触发** |
| `Servo_Record`（录波） | 采集保留、30094 可回读；但启动入口原来在 ASCII（`&`），需确认 30094 的 op=1 能独立启动 |
| `do_disable(const char *why)` 的 `why` | 参数已无人使用，保留只为调用点可读性 |
| `uart_link.c` 的 `put_*` | 只服务 `FaultDump`，约 100 行 |
| `X72_ASCII_DEBUG` 字样 | 仅存在于历史注释里（说明当时为什么那样做），不再是宏 |

**建议下一步**：`CurStep` 和录波如果还要用，给它们加 30069 动作号（现在 4~14 已用，可用 15/16）。

---

# 追加：删除引入的一个 regression（已修，Code=106732）

## R1. 现象

`+`（磁链）与机械四项 `(` `)` `<` `}` **第一次能出结果，之后每次都"卡住"** ——
上位机等到超时，30092 一条都没有。`@`（R/L）不受影响。

## R2. 根因：边沿检测把 `s_prev` 停在 DONE

重写 `flux_task1ms` / `mech_task1ms` 时，只保留"终结态推送"，写成了：

```c
if ((st == s_prev) || ((st != FI_DONE) && (st != FI_FAIL))) { return; }  // ← 不更新 s_prev
s_prev = st;
```

`s_prev` 只有在进入 DONE/FAIL 时才被更新。于是：

| 轮次 | st 序列 | s_prev | 结果 |
|---|---|---|---|
| 第 1 次 | IDLE → RUN… → **DONE** | IDLE → **DONE** | ✅ 推送 |
| 第 2 次 | IDLE → RUN… → **DONE** | **DONE 不变** | ❌ 首条 `st == s_prev` 命中，直接 return |

第二次结束时 `st` 和 `s_prev` 都是 `DONE` → 第一个条件成立 → 永远推不出来。

**这正是"电气参数辨识和机械参数辨识会一直卡住"的来源**：
`@` 走的 `rl_ident_task1ms()` 是原写法（`return` 前有 `s_prev = st;`），所以正常；
而 `+` 和机械四项走的是我重写的这两个函数。

## R3. 修法：先跟踪状态，再判终结态

```c
if (st == s_prev) { return; }        /* 状态没变：不动 */
s_prev = st;                          /* ★ 每个状态都记录 */
if ((st != FI_DONE) && (st != FI_FAIL)) { return; }   /* 只关心终结态 */
```

这样 `s_prev` 会跟着 IDLE → RUN → DONE 一路更新，
每次进入 DONE 都是一个真边沿。

## R4. 核对：其余四个本来就是安全写法

| 函数 | 写法 | 结论 |
|---|---|---|
| `rl_ident_task1ms` | `if (st==s_prev \|\| 非终结) { s_prev = st; return; }` | ✅ 安全 |
| `fric_task1ms` | 同上 | ✅ |
| `enc_cal_task1ms` | 同上 | ✅ |
| `ident_task1ms`（Z） | 同上 | ✅ |
| **`flux_task1ms`** | 被改成"不更新 s_prev" | ❌ → 已修 |
| **`mech_task1ms`** | 同上 | ❌ → 已修 |

## R5. 验证（Code=106732，全部连测）

| 动作 | 次数 | 结果 |
|---|---|---|
| `@` R/L | 3 | ok=1，0.39 s，R=290.8/287.0/289.1 mΩ，L=91.2/90.4/90.5 µH |
| `+` 磁链 | 2 | ok=1，15.0 s，ψ=4.364/4.361 mWb，kt=91.6 mN·m/A，dev=3.8% |
| `}` 齿槽 | 2 | ok=1，12.8 s，n=512，pp=1.240/1.223 A |
| `<` 惯量 | 1 | ok=1，1.3 s，J=0.6835e-4 |
| `(` 扫频 | 1 | ok=1，3.5 s，n=11 |
| `)` 摩擦曲线 | 1 | ok=1，28.1 s，n=8，fric=0.1078 A，visc=0.00281 |

`fault` 全程 0，`cog=1`（齿槽表已装）。

## R6. 教训

**"只在终结态动作"的边沿检测，必须把 `s_prev = st` 放在判断终结态之前。**
否则 `s_prev` 会停在终结态，下一次"同样的终结态"不构成边沿 —— 而且第一次
总是好的，最容易骗过验收（我第一次测 `@` 三次全过，因为 `@` 根本没走这条路径）。

验收这类改动必须**每类动作连测两次以上**，且要覆盖所有分支，不能只测一种。
