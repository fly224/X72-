/*
 * param_store.h —— 辨识/标定参数的掉电保存（片内 Flash 最后一页）
 *
 * 为什么需要：C（电流零点）与 Z（电角度/极对数/方向）的结果原本只存在 RAM，
 * 每次上电都要重做一遍，其中 Z 要 9 s 且转子会转 —— 台架上调试时很浪费时间，
 * 做成整机后更不可能每次上电都转一圈。
 *
 * 存在哪：0x0807F000，STM32G474CE 的最后一页（512 KB / 4 KB 页 = 128 页）。
 *   代码当前 59.5 KB，离 0x7F000 还有 450+ KB，不会冲突。
 *   DBANK 选项决定这页属于 bank1 还是 bank2，擦除时按 FLASH->OPTR 判断。
 *
 * 一致性：magic + version + CRC32 三重校验。任何一项不符就当作"没有存过"，
 *   绝不半套应用 —— 零点错一半比完全没有零点更危险（力矩方向随机）。
 *
 * 擦写代价：一页擦除 20~40 ms，期间单 bank 模式下 CPU 取指停顿，
 *   20 kHz 控制节拍会连丢数百拍。所以：
 *     1) 只允许在 MOE=0 时调用（Save 内部会检查）；
 *     2) 擦写期间置 busy 标志，让 1 ms 任务跳过"ADC 同步丢失"判据，
 *        否则会误报 BSP_FAULT_ADC_SYNC；
 *     3) 内容未变化时直接返回，不擦页 —— 调试时反复按 C/Z 不会白耗寿命。
 */
#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>

#define PS_FLASH_ADDR    (0x0807F000u)
#define PS_MAGIC         (0x58373250u)   /* 'X''7''2''P' 小端 */
#define PS_VERSION       (3u)   /* v3: 增加磁链 psi_wb（56B -> 64B）
                                 * v2: 摩擦前馈 fric_a / fric_visc（48B -> 56B）
                                 * 版本不符一律当"没存过"——字段布局变了就不能半套应用。 */

/* 有效位：告诉上层"哪些参数是可信的" */
#define PS_FLAG_BIAS     (1u << 0)       /* 电流零点来自标定 */
#define PS_FLAG_ZERO     (1u << 1)       /* 编码器零点来自辨识 */
#define PS_FLAG_RL       (1u << 2)       /* R/L 来自在线辨识 */
#define PS_FLAG_FRIC     (1u << 3)       /* 摩擦前馈来自 $ 两点标定 */
#define PS_FLAG_PSI      (1u << 4)       /* 磁链来自 + 联合辨识 */
/* psi_t_ref 是否有效（2026-11-20 加）。
 *
 * 为什么必须单独一位而不是看 psi_t_ref 的数值：
 *   v3 的老参数里这个字段是 rsv1=0，若按"数值在 −20~150 内即有效"判据，
 *   0 会被当成"0 °C 辨识的"，补偿就从 0 °C 起算 —— 28 °C 时 ψ 被压到 96.6%，
 *   比不补更糟。用 flags 位判断则老参数自然退回 20 °C 基准，向后兼容。
 *   这样也不必升 PS_VERSION，现有零点/齿槽表全部保留。 */
#define PS_FLAG_PSI_T    (1u << 5)

/* aligned(8)：STM32G4 的 FLASH_TYPEPROGRAM_DOUBLEWORD 要求源地址 8 字节对齐，
 * 否则 HAL 会把非对齐地址直接喂给 FLASH->DATA0，写进去的是垃圾。
 * 只给局部变量加 __attribute__((aligned(8))) 不够 —— 结构体本身对齐要求是 4，
 * 栈上实例仍可能落在 4 的倍数而非 8 的倍数上（实测就是这么失败的：
 * 擦除/编程都返回 OK，但回读 CRC 不对，因为根本没进编程循环）。 */
typedef struct __attribute__((aligned(8))) {
    uint32_t magic;        /* PS_MAGIC */
    uint16_t version;      /* PS_VERSION */
    uint16_t ratio_int;    /* 极对数（辨识的 ratio 取整） */
    float    ratio;        /* 实测比值，保留小数用于判可信度 */
    int8_t   dir;          /* 编码器方向 +1/-1 */
    uint8_t  flags;        /* PS_FLAG_* */
    uint16_t rsv0;
    float    zero_rad;     /* 编码器零点（rad，已归一化到 [0,2π)） */
    float    bias_u;       /* U 相电流零点（V，ADC 输入侧） */
    float    bias_w;       /* W 相电流零点（V） */
    float    spd_kp;       /* 速度环 Kp（实测整定值，掉电不丢） */
    float    spd_ki_ts;    /* 速度环 Ki*Ts */
    float    r_ohm;        /* 相电阻，来自 @ 在线辨识 */
    float    l_h;          /* dq 电感，来自 @ 在线辨识（阶跃法） */
    float    fric_a;       /* 摩擦前馈幅值 A（= 3 rpm 稳态 iq），来自 $ 两点标定 */
    float    fric_visc;    /* 黏滞前馈 A/(rad/s) */
    float    psi_wb;       /* 永磁磁链 Wb，来自 + 联合辨识。
                            * 这个量决定 Kt = 1.5·p·ψ，进而决定 J 与所有转矩换算，
                            * 之前只能从"电压饱和点"反推（4 种注入幅值扫出来的曲线外推），
                            * 现在有 + 命令直接测，值得掉电保留。 */
    float    psi_t_ref;    /* ψ 辨识那一刻的电机温度 °C（PS_FLAG_PSI_T 有效时）。
                            * 为什么必须存：ψ 是**带温度**的量。2026-11-20 实测
                            * 84 °C 辨识得 3.894 mWb，若事后把 20 °C 当基准补偿，
                            * 冷机时会被二次压价 8%，Kt 与 BEMF 前馈同时偏小。
                            * 占原来的 rsv1（v3 里恒为 0），结构大小不变 = 64 B。 */
    uint32_t crc32;        /* 覆盖 crc32 之前的全部字节 */
} ParamBlob;                                 /* 64 B = 8 个 doubleword */

/* 读出并校验。返回 1 = 有效且已填入 *out（out 可为 NULL 只看有效性） */
uint8_t ParamStore_Load(ParamBlob *out);

/* 上一次 Load 失败的原因（0=成功 1=magic 2=version 3=crc 4=flags 未置位），
 * 以及 Flash 里读到的 version。只读诊断，见 param_store.c 的说明。 */
uint8_t  ParamStore_GetLoadWhy(void);
uint16_t ParamStore_GetLoadVer(void);

/* 校验字段（magic/version）并计算 CRC，然后与已存内容比较；
 * 不同才擦写。返回 1 = 成功（含"无需写"），0 = 失败。
 * 要求 MOE=0，否则直接返回 0。
 */
uint8_t ParamStore_Save(ParamBlob *in);

/* 擦除参数页（恢复出厂）。返回 1 = 成功 */
uint8_t ParamStore_Erase(void);

/* 1 = 正在擦写 Flash（供 1 ms 任务跳过实时性判据） */
uint8_t ParamStore_IsBusy(void);

/* 最近一次擦写的现场，给命令层回显用。
 * 存在的意义：写失败时"readback mismatch"这一句话无法定位问题，
 * 必须能看到实际用的是哪个 bank、哪个页、页多大 —— G4 的页大小随 DBANK 变
 * （DBANK=1 时 2 KB、DBANK=0 时 4 KB），算错就会擦到别的页去。 */
typedef struct {
    uint32_t dbank;      /* FLASH->OPTR 的 DBANK 位（0/1） */
    uint32_t bank;       /* 实际传给 HAL 的 FLASH_BANK_x */
    uint32_t page;       /* 实际传给 HAL 的页号 */
    uint32_t page_size;  /* 2 KB 或 4 KB */
    uint32_t erase_ret;  /* HAL_FLASHEx_Erase 返回值 */
    uint32_t prog_ret;   /* 第一个失败的 HAL_FLASH_Program 返回值，0 = 全部成功 */
    uint32_t readback;   /* 回读是否通过 */
    uint32_t align;      /* 编程源缓冲的地址低 3 位（必须为 0） */
} ParamStoreDiag;

const ParamStoreDiag *ParamStore_GetDiag(void);

/* ---------------- 齿槽表（单独占一页，不塞进 ParamBlob） ----------------
 *
 * 为什么不塞进 ParamBlob：128 点 int16 = 256 B，会让 ParamBlob 从 64 B 涨到
 * 320 B，每次保存要写 40 个 doubleword、memcmp 也要比 320 B，而齿槽表是
 * "标定一次用很久"的数据，没必要跟零漂/增益一起每 500 ms 比对。
 * 单独一页也让 PS_VERSION 不用再升一级（避免现有参数全部作废）。
 *
 * 地址 0x0807E000（倒数第二页）；ParamBlob 在 0x0807F000（最后一页）。
 * 双 bank 2 KB/页时分别是 bank2 的第 124 与 126 页，不冲突。
 */
#define COG_BINS_MAX     (512U)      /* 与 mech_ident.h 的 MI2_COG_BINS_MAX 必须一致；
                                      * 512 箱 = 1 KB 表，仍远小于 2 KB 页 */

/* 返回 1 = 成功（含"内容未变、无需写"），0 = 被拒绝（MOE=1）或写失败
 *
 * zero_ref：测量当时的编码器零点（rad）—— 加载时比对，不一致判失效
 * phase   ：表的 0 点对应 θ_multi 的哪个角度（rad）—— 查表时的相位基准
 * 两个角度缺一不可：只存 phase 就没法检出"重做 Z 之后相位失效"，
 * 而错相位的前馈会把齿槽扰动放大一倍。
 *
 * en：用户开关。为什么要落盘——表在、零点在，但开关只在 RAM 里，
 * 每次上电/复位都退回关闭，现场等于"标定过却从来不生效"。
 * 存入后冷启动自动恢复；加载时仍要求表有效（零点一致），
 * 所以"重做 Z 让表失效"这条保护不受影响。
 */
uint8_t ParamStore_SaveCog(const int16_t *tab, uint16_t n_bins,
                           float zero_ref, float phase, float pp, uint8_t en);
/* 返回 1 = 读到且 CRC 通过。*n_bins 返回实际点数；*en 返回开关（可为 NULL） */
uint8_t ParamStore_LoadCog(int16_t *tab, uint16_t *n_bins,
                           float *zero_ref, float *phase, float *pp,
                           uint8_t *en);
/* 1 = 页内有通过 CRC 的表 */
uint8_t ParamStore_HasCog(void);

#endif /* PARAM_STORE_H */
