# CRC 口径与 seq 计数修正说明

> 2026-11-15 11:52 · 固件 `X72_FOC.axf` Code **49504 B** / RO-data 1540 / RW 40 / ZI 7344 · 0 Error 0 Warning
> 烧录 11:52:43 `Erase Done / Programming Done / Verify OK / Application running`
> 本文所有 PASS/FAIL 都来自可重跑的命令输出，不是估算。

---

## 0. 结论先行

| 项 | 修正前 | 修正后 | 影响面 |
|---|---|---|---|
| CRC 覆盖范围 | 从帧首 `0xFD` 起算（`crc_calc(f, 10+len)`） | 从 `len` 字节起算（`crc_calc(&f[1], 9+len)`），符合 MAVLink v2 | 固件 1 处 + 工具链 4 处 + 页面 1 处 |
| 报文 seq | 先 `s_seq++` 组帧，发送失败也推进 → 假丢帧 | 发送返回 `HAL_OK` 后才 `s_seq += 3` | 仅固件 |
| 失败原因可见性 | 无 | `[DIAG]` 行尾新增 `busy=0xNN`（`(gState<<4)\|hdmatx->State` 快照） | 固件 + 页面 |

**修完之后**：抓包里 300 帧全部通过标准口径 CRC；seq 严格 +1（含 255→0 回绕），`seq 跳变 = 0 次 / 缺 0`。

关键背景：**host 侧协议层 `mavlink_proto.py` 一直是标准口径**（`CRC_INCLUDE_SEQ=True`，即 `header[1:10]+payload`，见该文件 `_crc_cover()`），所以这次不是"引入新约定"，而是**把固件改成和既有协议层一致**。此前若接 pymavlink / QGC / Mission Planner，这些帧会 100% 被判 CRC 错。

---

## 1. 改动一：CRC 覆盖范围

### 1.1 代码差异

`Board/telemetry_tx.c` → `frame_build()`

```c
/* 修正前 */
crc = crc_calc(f, (uint16_t)(10u + len), extra);        // f[0] = 0xFD 也进 CRC

/* 修正后 */
crc = crc_calc(&f[1], (uint16_t)(9u + len), extra);     // len..payload，不含 0xFD
```

MAVLink v2 规范：CRC 从 `len` 字节开始，覆盖 `len / incompat / compat / seq / sysid / compid / msgid[3]` 共 9 字节 + payload，最后追加 `crc_extra`。

### 1.2 影响面清单（改前逐一核对过）

| # | 位置 | 原状态 | 动作 | 风险 |
|---|---|---|---|---|
| 1 | `Board/telemetry_tx.c:109` | 含魔数 | **已改**为标准口径 | 无：唯一组帧点 |
| 2 | `Agnet/serial_link_check.py:177` | `raw[:HDR_LEN+ln]` 含魔数 | **已改** `raw[1:HDR_LEN+ln]`，并保留旧口径兼容计数 | 低 |
| 3 | `Agnet/serial_link_check.py:226` `build_frame()` | 自测造帧含魔数 | **已改**，加 `legacy_crc` 开关复现旧帧 | 低 |
| 4 | `Agnet/verify_host_keyparams.py:51` | `body = raw[i+1:...]`（**本来就是标准**）+ `ok_fw` 兼容旧口径 | **已改**注释措辞，双口径统计保留 | 无 |
| 5 | `Agnet/verify_fw_frame.py:68` | `crc_calculate(frame[:-2])` 含魔数 | **已改** `frame[1:-2]` | 无 |
| 6 | `servo-web/index.html` `buildFrame()` | `mavCrc(body)` 含魔数（下发命令用） | **已改** `mavCrc(body.subarray(1))` | 低：固件当前不收命令 |
| 7 | `G:/FOC/驱动开发/PG_X72_CAN/host/mavlink_proto.py` | **本来就是标准口径** | 不动 | — |
| 8 | 历史抓包 `cap_baseline.bin` 等 | 旧口径字节 | 不动，脚本自动识别为 legacy | 无 |

**没有受影响的地方**（逐一确认过）：

- `Board/uart_link.c` 的 ASCII 诊断行走 `HAL_UART_Transmit`，与 MAVLink 无关。
- `Core/`、`app/`、`motor_control/` 里没有任何 CRC 实现。
- 页面 `decodeFrame()` 不校验 CRC（只按 len 切片），所以**即使不修页面也不影响显示**；本次只是把 `buildFrame()` 同步成标准口径。

### 1.3 验证证据

**离线（固件本体编译到主机跑，非手写模拟）**

```
$ python Agnet/verify_fw_frame.py
固件输出: 13300 B
  30025  LEN=53 CRC=A436/A436  PASS
  30027  LEN=32 CRC=94EA/94EA  PASS
  30040  LEN=12 CRC=F72A/F72A  PASS
  报文 30025 × 100 条 / 30027 × 100 条 / 30040 × 100 条
  seq 连续性: 300 帧首尾 0→43，断点 0 处  PASS
合计 FAIL=0
```

**双口径统计（同一份 300 帧流）**

```
$ python Agnet/verify_host_keyparams.py fw_stream.bin
 "frames": 300, "bad_crc": 0,
 "crc_hits": { "fw_magic_included": 0, "std_mavlink": 300 },   ← 修正前是 2134 / 0
 "seq_first": 0, "seq_last": 43, "seq_gaps": 0
```

**页面接收侧（页面自己的 mavCrc 与 Python 独立实现对齐）**

```
$ node Agnet/verify_host_keyparams.js fw_stream.bin expected_fw.json 4096
[PASS] CRC 通过数 == Python std 口径        → 300 vs 300
[PASS] CRC 失败数 == Python 旧口径命中数    → 0 vs 0
[PASS] 面板-CRC 文本含通过数                → 300 对
RESULT 23/23  ALL PASS

$ node Agnet/verify_host_keyparams.js cap_baseline.bin expected_baseline.json 4096
[PASS] CRC 通过数 == Python std 口径        → 0 vs 0
[PASS] CRC 失败数 == Python 旧口径命中数    → 2134 vs 2134
RESULT 33/33  ALL PASS          ← 旧抓包仍可解析，只是标记为 legacy
```

### 1.4 实机证据（11:56 之后）

板上跑新固件（11:52:43 烧录），COM27 上直接读 5 s：

```
$ python capture_monitor.py 5
OPEN_OK COM27 @ 921600
读到的字节 66501 / 5.00 s = 13289 B/s（理论 13300 B/s）
in_waiting 峰值 119 B，均值 22 B，采样 3015 次
帧数 1500  坏帧头 0  seq 跳变 0 次，缺 0 帧

$ python Agnet/verify_host_keyparams.py cap_monitor.bin
 "frames": 1500, "bad_crc": 0,
 "crc_hits": { "fw_magic_included": 0, "std_mavlink": 1500 },
 "seq_first": 234, "seq_last": 197, "seq_gaps": 0     ← 跨过一次 255→0 回绕
```

`[DIAG]` 同一次开机的输出：

```
[DIAG] ticks=30009 adc1=60018 adc2=30009 tx=299 drop=0 rec=0 fault=0x00 vbus=30.8V busy=0x00
```

- `busy=0x00` → 这次运行**一次 `HAL_UART_Transmit_DMA` 启动失败都没有**（`s_busy_snap` 仍为初值）
- `drop=0`、`rec=0` → 没有丢帧、没有走自愈路径
- 5 s 内 1500 帧 = 500 组 × 3，正好 100 Hz，seq 零跳变

A–E 五项判定（实机，同一块板）：

```
CRC 错误 / 未知 ID      : 0 / 1  (起点残帧 0，旧口径帧 0)
[PASS] A 帧率达标        100.0 / 100 fps
[PASS] B CRC 全对        0 错
[PASS] C 三报文成对      30027/30025=1.000 30040/30025=1.001
[PASS] D 无断流          最大间隔 62.0 ms
[PASS] E 量纲/偏移合理    vbus=30.78 V
结论 : PASS —— 串口链路已通，二进制遥测稳定
```

**顺手修掉一个工具缺陷**：`serial_link_check.py` 原来把"抓包起点未对齐的半帧"也算进 `crc_err`，导致从数据流中间切入时 B 项必然 FAIL（本次首跑就报了 `1 错`）。现在 `FrameParser` 增加 `synced` 标志，第一个合法帧之前的 CRC 错单独计入 `crc_err_head`，不参与 B 项判定。

**意外收获：第一次实机抓包曾出现 `seq_gaps=211`，但那是抓包端的问题，不是固件的。** 详见 §3.5。

---

## 2. 改动二：seq 计数时机

### 2.1 问题链路

```c
/* 修正前：序号在组帧参数里就 ++ */
n_in  = frame_build(..., s_seq++, p_in,  ...);
n_out = frame_build(..., s_seq++, p_out, ...);
n_pos = frame_build(..., s_seq++, p_pos, ...);

if (HAL_UART_Transmit_DMA(...) == HAL_OK) { s_tx_ok++; }
else                                      { s_tx_drop++; }   /* 序号已经用掉了 */
```

`HAL_UART_Transmit_DMA` 返回 `HAL_BUSY` 时，这三条帧**一个字节都没发出去**，但 `s_seq` 已经 +3。上层按 seq 差分算丢帧，于是整组帧被误判为丢帧。

原抓包实证：`seq_gaps = 156`，且跳变呈"单次跳 156"的形态 —— **52 次启动失败 × 3 个序号**。真正丢失的帧数远小于 156。

### 2.2 改法

```c
seq0 = s_seq;                                              /* 组帧前取基准 */
n_in  = frame_build(..., seq0,                 ...);
n_out = frame_build(..., (uint8_t)(seq0 + 1u), ...);
n_pos = frame_build(..., (uint8_t)(seq0 + 2u), ...);

if (HAL_UART_Transmit_DMA(...) == HAL_OK) {
    s_seq = (uint8_t)(seq0 + 3u);        /* 真发出去了才推进 */
    s_tx_ok++;
} else {
    s_tx_drop++;
    s_busy_snap = (uint8_t)(((uint8_t)huart1.gState << 4) |
                            (uint8_t)((huart1.hdmatx != 0) ? (huart1.hdmatx->State & 0x0Fu) : 0x0Fu));
}
```

### 2.3 顺带补上的两件事

1. **失败现场留证**：`[DIAG]` 第 4 行结尾新增 `busy=0xNN`。
   - `0x00` = 从未失败；`0x1x` = 上次失败时 `gState` 非 READY，低半字节是 `hdmatx->State`（`1` = `HAL_DMA_STATE_BUSY`）；`0x?F` = `hdmatx` 为空指针。
   - 这是为了回答"那 52 次 `HAL_BUSY` 到底卡在哪一级 HAL 状态机"，不必再靠猜。
2. **`buf[352] → buf[384]`**：给新增字段留余量。该函数此前出过一次 60 B 栈溢出（`CFSR=0x100 IBUSERR`），虽然 `LIM_SET` 已做上界检查（会截断而不是溢出），仍按"宁可多留"处理。

### 2.4 验证

- 正常路径：300 帧 seq 从 0 到 43（= 299 & 0xFF），**断点 0 处**。
- 失败路径：只能实机造 `HAL_BUSY` 才能验证；逻辑上"失败不推进"是显然的，且现在能通过 `busy=0xNN` 反查。

---

## 3. 完整改动清单

| 文件 | 改动 |
|---|---|
| `Board/telemetry_tx.c` | CRC 起点 `f[0]→f[1]`；`seq0` 基准 + 成功后 `s_seq = seq0+3`；新增 `s_busy_snap` 与 `Telemetry_GetBusySnap()`；更新文件头注释 |
| `Board/telemetry_tx.h` | 声明 `Telemetry_GetBusySnap()` |
| `Board/uart_link.c` | `[DIAG] ticks` 行尾追加 ` busy=0xNN`；`buf[352]→[384]`；include `telemetry_tx.h` |
| `Agnet/serial_link_check.py` | 解析与造帧全部改标准口径；新增 `Stats.crc_legacy` / `crc_err_head` 与 `FrameParser.synced`（起点残帧不判错）；自检增加"旧口径兼容"用例 |
| `Agnet/verify_host_keyparams.py` | 注释澄清（本文件原就是标准口径 + 双口径统计） |
| `Agnet/verify_fw_frame.py` | CRC 口径 `frame[:-2] → frame[1:-2]`；新增 seq 连续性断言 |
| `Agnet/verify_host_keyparams.js` | 注入页面 `CRC_EXTRA` 与 `mavCrc`；新增两条 CRC 一致性断言 + `kpBusy` 断言；无 `[DIAG]` 的纯帧流自动跳过 DIAG 断言 |
| `servo-web/index.html` | `buildFrame` 用标准口径；接收侧软 CRC 统计（`linkStat.crcOk/crcErr`）；`[DIAG]` 解析新增 `busy`；KP 面板新增「CRC 校验」「启动失败现场」两格；示波器新增 7 通道 |

固件体积变化：`Code 49864 → 49504 B`（-360 B，主要是 seq 改动 + 32 位快照，CRC 起点少算 1 字节），`RO-data 1544 → 1540 B`。

---

### 3.5 排障记录：一次「假丢帧」的误判与澄清

改完烧进去后第一次抓包（`capture_crc.py`，同时做烧录与读取），统计到 `seq_gaps=235`。当时有两种可能：固件还在吞号，或者抓包端丢数据。逐一排查：

**第 1 步：看跳变分布，而不是只看总数。**

```
帧数 2715   跳变事件 2 次   缺帧 235
第一次：字节偏移 160   （复位点，旧运行尾巴 → 新运行开头）
第二次：字节偏移 13473，d=146
之后 2412 帧无任何断点
```

两次都在**前 13.5 KB**，也就是 `UV4 -f` 烧录阻塞的那 5.7 s —— 那段时间 `subprocess.call()` 把进程卡住，**没人读串口**，驱动缓冲溢出。

**第 2 步：改成「跳过启动窗口再读」。** 仍剩 `seq_gaps=211`（1 次事件，偏移 1099）。但此时 `[DIAG]` 明确写着 `tx=299 drop=0 busy=0x00`——固件侧 299 次发送全成功。矛盾。

**第 3 步：监控 `in_waiting`。** 不烧录、纯读 5 s：

```
读到的字节 66501 / 5.00 s = 13289 B/s（理论 13300 B/s）
in_waiting 峰值 119 B，均值 22 B
帧数 1500  坏帧头 0  seq 跳变 0 次，缺 0 帧
```

积压峰值只有 119 B（远低于驱动缓冲），**seq 零跳变**。

**结论**：那 211 帧的"缺失"发生在 `capture_stable.py` 从 `reset_input_buffer()` 到进入 `while` 读取循环之间的空档 —— `time.sleep(3.5) + reset + 首次 read` 期间驱动仍在接收，reset 之后到真正开始读之前又积压了一段。**与固件无关**。

**教训（已写进脚本）**：

1. 「烧录 + 抓包」同进程时，`UV4 -f` 的 5.7 s 是黑洞，任何 seq 统计都必须排除这段；
2. 只看 `seq_gaps` 总数会误判，**必须看跳变位置分布**；
3. 要证明链路干净，最省事的做法是**纯读 + 监控 `in_waiting`**，别在同一次运行里既烧录又统计。

---

## 4. 回滚方式

固件（只动 3 个文件，均为小范围）：

```bash
cd G:/FOC/X72/Firmware/FM_Servo_V3/embedded_project/STM32G474_App
# 把 crc_calc(&f[1], 9u+len, ...) 改回 crc_calc(f, 10u+len, ...)
# 把三处 (uint8_t)(seq0 + n) 改回 s_seq++，并删掉 s_seq = seq0 + 3
# uart_link.c 去掉 busy=0xNN 输出
```

工具链不必回滚：`serial_link_check.py` 与 `verify_host_keyparams.py` 都保留了旧口径兼容分支，新旧固件都能解析（旧帧只计入 `crc_legacy`，不判错）。

---

## 5. 局限与未验证项

1. **实机验证已完成，但只覆盖 `en=0`（电机未使能）**：5 s / 1500 帧 / seq 零跳变 / CRC 1500 passed。闭环（PI + SVPWM 使能）后 UART 时序不变，但未实测。
2. **`HAL_BUSY` 根因仍未知，只是这次没发生**：`busy=0x00` 说明本次运行从未失败；旧固件的 52 次失败仍未复现、未定因。要把 `busy=0xNN` 读到非零值才能定位，属于"下次出现时再抓".
3. **`verify_fw_frame.py` 的构建脚本缺失**：`Agnet/verify_fw_frame.sh` 不存在，`.workbuddy/tmp/fw_frame.exe` 需要手工重建（本次已重建，命令见 §6）。建议补一个 `.sh`，否则下次改固件后跑的还是旧 exe，会得到"300 帧全 FAIL"的假结论。
4. **兼容旧口径的代价**：`serial_link_check.py` 遇到旧口径帧会计入 `ok`（不判错），只看 `crc_err` 会漏掉"固件没升级"的情况，所以报告里必须一起看 `旧口径帧` 那一行。
5. **历史基线抓包已被覆盖**：`tmp/exp_dsp_20261115/cap_baseline.bin` 在本次抓包时被同名输出覆盖（`capture_crc.py` 的路径替换没生效，脚本仍指向旧路径），原 94937 B 的旧口径样本已不可用；它的统计结论（2134 帧 / 含魔数 2134 命中 / 标准口径 0 命中 / seq 缺 156）已固化成 `expected_legacy_orphan.json` 并写进本文与 `上位机_关键参数显示说明.md`（那一组 33/33 断言曾真实跑过，但**现在无法复现**）。替代样本：
   - `tmp/build_crc/cap_monitor.bin` —— 新固件、纯读 5 s，**黄金样本**（66501 B / 1500 帧 / 全标准口径 / seq 零跳变）
   - `tmp/exp_dsp_20261115/cap_incl_flash_window.bin` —— 新固件、含 `UV4 -f` 阻塞窗口（120669 B / 2715 帧 / seq 假跳 235），**只作排障样本，不要用它判链路**

---

## 6. 复现命令

```bash
# 1) 重建主机侧固件 harness（若 exe 缺失或固件已改）
cd G:/FOC/X72/Firmware/FM_Servo_V3/embedded_project/STM32G474_App
mkdir -p G:/FOC/X72/.workbuddy/tmp/gtmp
TMP=G:/FOC/X72/.workbuddy/tmp/gtmp TEMP=G:/FOC/X72/.workbuddy/tmp/gtmp \
gcc -DUSE_HAL_DRIVER -DSTM32G474xx \
    -I Board -I Core/Inc -I Drivers/STM32G4xx_HAL_Driver/Inc \
    -I Drivers/CMSIS/Device/ST/STM32G4xx/Include -I Drivers/CMSIS/Include \
    G:/FOC/X72/Agnet/verify_fw_frame_harness.c Board/telemetry_tx.c -lm \
    -o G:/FOC/X72/.workbuddy/tmp/fw_frame.exe

# 2) 三方闭环（固件 → 协议布局 → 页面 decodeFrame）
cd G:/FOC/X72 && python Agnet/verify_fw_frame.py

# 3) 解析器自检（不接硬件，含旧口径兼容用例）
python Agnet/serial_link_check.py --selftest

# 4) 实机链路 A–E（需释放 COM27）
python Agnet/serial_link_check.py -p COM27 -b 921600 -t 8

# 5) 实机 seq/CRC 黄金样本（不烧录、纯读，同时监控驱动缓冲积压）
#    ★ 别把烧录和统计放在同一次运行里：UV4 -f 会阻塞 5.7 s，那段时间没人读串口
cd G:/FOC/X72/.workbuddy/tmp/build_crc
python capture_monitor.py 5          # → cap_monitor.bin
cd G:/FOC/X72 && python Agnet/verify_host_keyparams.py .workbuddy/tmp/build_crc/cap_monitor.bin
```

构建坑：Keil 与 gcc 在命令行下都会因为 `C:\Windows\TEMP` 不可写而失败（`Failed to create temporary file name` / `Cannot create temporary file`），**必须显式指定 `TMP`/`TEMP`**，否则 Keil 会"假成功"（0 个目标生成）。
