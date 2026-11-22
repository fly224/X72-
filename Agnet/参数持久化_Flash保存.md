# 辨识/标定参数的 Flash 持久化

> 2026-11-16 · X72 / STM32G474CEU6 · 固件 `Code=63408 RO=3624 RW=104 ZI=9400`，0 Error 0 Warning

## 1. 结论

`C`（电流零点）与 `Z`（极对数/方向/电角度零点）的结果，加上速度环整定增益，现在会自动写进片内 Flash 最后一页。**冷启动后直接可用，不用再跑 9.3 s 的 Z**。

实测（软复位验证，等价于掉电重启）：

```
复位前：cal=1 z=1 fl=1
复位后：cal=1 z=1 fl=1     <- 没发过 C / Z
速度环：复位前 kp=4667.05e-3 → 复位后 kp=4667.05e-3（8x 档保住了）
```

判据是 `S` 状态行里的 `fl=`：1 = Flash 里有通过 CRC 校验的参数。`fl=0` 且 `cal=0 z=0` 就说明参数页是空的（例如刚烧录过固件）。

## 2. 存了什么

48 字节，正好 6 个 doubleword：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | `magic` | `0x58373250`（'X72P'） |
| 4 | `version` | 1 |
| 6 | `ratio_int` | 极对数（辨识值取整） |
| 8 | `ratio` | 实测比值，保留小数用于判可信度 |
| 12 | `dir` | 编码器方向 ±1 |
| 13 | `flags` | bit0 = bias 有效，bit1 = zero 有效 |
| 16 | `zero_rad` | 编码器零点（已归一化到 [0,2π)） |
| 20/24 | `bias_u` / `bias_w` | U/W 相电流零点（ADC 输入侧电压） |
| 28/32 | `spd_kp` / `spd_ki_ts` | 速度环整定值（掉电不丢，省得每次重扫 6 个档） |
| 36 | `crc32` | 覆盖前 36 字节 |
| 40 | `pad[2]` | 补到 48 |

`magic + version + CRC32` 三重校验，任何一项不符就当作"没存过"——**绝不半套应用**。零点错一半比完全没有零点更危险：Park 变换的 θ 与磁场无关，力矩方向是随机的（这个坑 11-15 已经踩过，飞车 −436 rpm）。

## 3. 放在哪、为什么

`0x0807F000`，512 KB Flash 的最后一页。代码当前 63.4 KB，离这一页还有 440+ KB。

链接脚本 `stm32g474xx_flash.sct` 把 ROM 映射到 `0x08000000 + 0x80000`（全片），没有为参数预留区域——因为代码永远长不到那里，加预留反而要改 sct 并重启工程。

**页大小不是固定的**，这是本次第一个坑：

| DBANK 选项 | 页大小 | 0x0807F000 所在 | 页号 |
|---|---|---|---|
| `DBANK=0`（单 bank） | 4 KB | 唯一 bank | 127 |
| `DBANK=1`（双 bank） | **2 KB** | bank2（起始 0x08040000） | **126** |

实机 `FLASH->OPTR` 读出来是 `DBANK=1`，所以按 4 KB 算出来的页号 63 擦的是 `0x0805F800` —— 目标页根本没擦，编程却返回 OK，回读 CRC 自然不对。代码现在从 OPTR 实时判断，并把 `dbank/bank/page/page_size` 记进诊断结构，写失败时 `A` 命令会打出：

```
[CMD] PS FAIL db=1 bk=2 pg=126 sz=2048 er=0 pr=0 rb=0 al=0
```

`er=0` 擦除成功、`pr=0` 编程成功、`rb=0` 回读失败——这三个字段把"哪一步坏了"分得很清。

## 4. 三个坑（按踩到的顺序）

### 4.1 `HAL_FLASH_Program()` 第三个参数：值，不是指针

**这是最隐蔽的一个。** STM32G4 的签名是：

```c
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
```

`Data` 是**数据本身**，按值传。而 H7/H5 那几代是 `uint32_t Data` 且内部当指针解引用：

```c
// G4：值
*(__IO uint32_t *)FLASH_NBDOUBLEWORDS_ADDR = (uint32_t)Data;
// H7：指针
*(__IO uint32_t *)FLASH_NBDOUBLEWORDS_ADDR = *(__IO uint32_t *)Data;
```

按 H7 的习惯写 `HAL_FLASH_Program(..., (uint32_t)&buf)`，等于把**栈地址**写进 Flash。现象是：擦除返回 OK、编程返回 OK、回读 CRC 不对——和"没擦干净"一模一样，光看返回值根本分不出来。

正确写法是组装成 `uint64_t` 按值传：

```c
uint64_t dw = 0U;
memcpy(&dw, ((const uint8_t *)&s_wbuf) + (i * PS_DW), PS_DW);
HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PS_FLASH_ADDR + (i * PS_DW), dw);
```

### 4.2 栈上局部变量只有 4 字节对齐

`ParamBlob` 成员最大对齐是 4（`uint32_t`），所以即使给结构体加 `__attribute__((aligned(8)))`，栈上实例仍可能落在 4 的倍数而非 8 的倍数上。原来那段 `if (((uint32_t)&buf & 7u) != 0u) ok = 0;` 直接把流程挡在编程循环之前，`pr` 一直是 0。

现在用 `static ParamBlob s_wbuf __attribute__((aligned(8)))`，并把 `&s_wbuf & 7` 记进 `al` 字段（实测 0）。

顺带一提：改成 memcpy 组装 `uint64_t` 之后，其实对源缓冲的对齐已经不敏感了，静态缓冲只是让"为什么这么写"更直白。

### 4.3 擦写期间会误报 ADC 同步故障

单 bank 模式下擦一页会让 CPU 取指停顿 20~40 ms，20 kHz 控制节拍连丢几百拍。`Servo_SlowTask1ms` 里那条"连续 3 ms 没有控制节拍 → `BSP_FAULT_ADC_SYNC`"的判据会立刻命中，于是每次保存参数都留下一个假故障。

现在 `ParamStore_IsBusy()` 为 1 时跳过该判据：

```c
if (ParamStore_IsBusy() != 0U) {
    s_inj_miss = 0U;
    s_last_tick = s_tick_count;
} else if (s_tick_count == s_last_tick) { ... }
```

并且 `ParamStore_Save()` 自身要求 `MOE=0`——擦除期间占空比寄存器停在最后一次写入值，等于桥臂输出被冻结在某个固定矢量上。

## 5. 什么时候写

不是"命令执行完就写"，而是**值真的变了才写**：

- `C` 是异步的：`ReadInjected` 在 20 kHz 里累加 200 ms 才把 `cal` 置 1；
- `Z` 是 9 s 状态机。

所以在命令处理里立刻保存只会存到旧值。实际做法是 1 ms 任务每 500 ms 调一次 `Servo_ParamSaveNow()`，`ParamStore_Save()` 内部再与 Flash 内容 `memcmp`，相同就直接返回——稳态下不产生任何擦写，调试时反复按 `C`/`Z` 也不会白耗 10k 次寿命。

手动路径是 `A` 命令，走同一个函数。

## 6. 命令

| 键 | 作用 |
|---|---|
| `A` | 立即保存（回显 `PS saved`，失败时回显 `db/bk/pg/sz/er/pr/rb/al` 诊断） |
| `!` | 软复位（先走完整封锁流程再 `NVIC_SystemReset`，用于验证冷启动恢复） |
| `S` | 状态行新增 `fl=`（1 = Flash 里有有效参数） |

`!` 的实现有个细节：`do_disable()` 只是把回显挂进 `s_out_pending`，真正发送在下次 `ServoCmd_Poll` 的 `ascii_pump()`。如果直接 `HAL_Delay()` 再复位，那行永远发不出去。所以复位前手动 pump 一次：

```c
do_disable("reset");
while (s_out_pending != 0U) { ascii_pump(); }
HAL_Delay(50);
NVIC_SystemReset();
```

## 7. 验证方法

`Agnet/param_flash_test.py` 走完整链路并逐步取证：

```
[1] 烧录后首次上电      -> fl=0 cal=0 z=0
[2] C 标定              -> cal=1
[3] Z 辨识              -> z=1（此时 fl 已变 1，自动保存生效）
[4] A 手动确认          -> PS saved
[5] ! 软复位
[6] 复位后 S            -> cal=1 z=1 fl=1  => PASS
```

第 6 步是关键判据。如果复位后 `cal/z` 又回 0，说明 `param_load_and_apply()` 没生效，按这个顺序查：CRC 两边算法是否一致 → DBANK/页号是否算对 → 编程是否真按值传。

单独验证档位恢复：

```
L L (1x->2x->4x) -> 等 1.2 s -> ! 复位 -> K 看 kp 是否仍是 4x
```

实测 `kp=4667.05e-3` 复位后不变，且再按 `L` 直接到 16x（说明档位索引也同步了，不是从 1x 重跳）。

## 8. 局限

| 项 | 现状 | 影响 |
|---|---|---|
| **Keil 烧录会擦掉参数** | `Erase Full Chip` 覆盖整个 512 KB | 每次烧固件都要重做 C+Z。要保留需把工程擦除设置改成 `Erase Sectors` 并排除最后一页，或用 `!` 复位（不烧录）验证 |
| 电流零点温漂 | 恢复的是上次标定值 | 冷机标定、热机使用会偏。温漂大的场合仍应重标一次 `C` |
| Flash 寿命 | 内容不变不擦写 | 正常使用一次上电最多写一次；按 10k 次算足够 |
| 未涵盖 R/L/psi | 这三个量还没有辨识流程 | `psi` 实测比模型小 41%（见《速度闭环_实现与实测.md》§9），需要单独的辨识才能一并存进来 |

## 9. 涉及文件

| 文件 | 内容 |
|---|---|
| `Board/param_store.c/.h`（新增） | 布局、CRC、DBANK 判断、擦写、诊断 |
| `Board/bsp_servo.c/.h` | `BspServo_SetCurrentBias()`（恢复）、`BspServo_GetEncoderZero()`（读取） |
| `Board/motor_ident.c/.h` | `MotorIdent_SetCached()`（恢复 ratio/dir 供回显） |
| `Board/servo_isr.c` | `Servo_ParamSaveNow()`、1 ms 自动保存、上电 `param_load_and_apply()`、busy 时跳过 ADC 同步判据 |
| `Board/servo_cmd.c` | `A` / `!` 命令、`fl=` 字段、档位索引与恢复值同步 |
| `Agnet/param_flash_test.py` | 端到端验证脚本 |
