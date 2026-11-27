/*
 * cog_ff.h —— 齿槽转矩前馈
 *
 * 为什么需要它：
 *   齿槽是**位置相关**的周期扰动，实测峰峰 1.169 A（双向平均后），
 *   而 30 rpm 空载运行电流只有 0.13 A —— 扰动是信号的 9 倍。
 *   但速度环是**速度反馈**：齿槽造成的位移不产生持续速度，环"看不见"它，
 *   所以无论怎么调 Kp/Ki 都压不掉这部分误差（实测增益调度在 3~500 rpm
 *   完全测不出差异，就是这个原因）。
 *
 *   唯一有效的办法是**按角度查表直接抵消**：把 `}` 测出的 128 点表
 *   （单位 A，已经是"维持匀速所需的 iq"）直接叠到 iq 给定上。
 *
 * 三个必须讲清的细节：
 *
 *  ① 表的相位参考不是编码器零点，而是**测量起点** s_cog_th0。
 *     齿槽表按 rel = θ_multi − s_cog_th0 分箱，bin 0 对应 s_cog_th0。
 *     所以表里存 cog_phase = s_cog_th0 mod 2π，查表时用
 *     (θ_multi − cog_phase) mod 2π 索引。
 *
 *  ② 相位错了比不补更糟。θ_multi 的零点由 Z 决定，重做 Z 之后
 *     cog_phase 就失效了，继续用会把扰动**放大**一倍。
 *     所以表里同时存 zero_ref（测量时的编码器零点），加载时比对，
 *     差超过 1° 直接判定表失效。
 *
 *  ③ 高速要关掉。齿槽频率 = 24 阶 × 转速；1500 rpm 时 600 Hz，
 *     已经接近电流环带宽（1.4 kHz）的三分之一，前馈跟不上反而引入噪声。
 *     而且高速时转子惯性把齿槽滤掉了，本来就不需要补。
 *     所以按 |w_ramp| 门控：< 20 rad/s（191 rpm）全量，> 52 rad/s（496 rpm）关闭，
 *     中间线性过渡。
 *
 *  ④ **极低速也要减量**。实测 3 rpm 开全量前馈，纹波 σ 从 0.397 涨到 0.449
 *     （恶化 13%）：3 rpm 时转子在黏滑（stick-slip）爬行区，1.05 A 峰峰的
 *     前馈在静摩擦拐点反复激发，力矩补偿直接变成速度抖动。
 *     低速段的问题不是"补得不够"而是"补得不对"，所以 10 rpm 以下按 ω/ω_low
 *     线性缩放到 0：3 rpm 只补 30%，既保留齿槽抵消的收益，又不激化爬行。
 *
 * 叠加位置：`s_spd_iq = PI + 摩擦前馈 + 加速度前馈 + cog_ff`，
 * 然后再限幅。cog_ff 不进入积分（积分是独立变量），所以不会被积分抵消。
 *
 * 开关（s_en）随表一起落盘：表在、零点没变，但开关只在 RAM 里，
 * 每次上电都退回关闭，现场等于"标定过却从来不生效"。
 */
#ifndef COG_FF_H
#define COG_FF_H

#include <stdint.h>

/* 速度门控（rad/s）：< LOW 线性起量，LOW~FULL 全量，> ZERO 关闭 */
#define COG_FF_W_LOW    (1.0472f)   /* 10 rpm：以下按比例缩放 */
#define COG_FF_W_FULL   (20.0f)     /* 191 rpm */
#define COG_FF_W_ZERO   (52.0f)     /* 496 rpm */

/* 零点校验门限（rad）：超过就尝试"相位对齐"。
 * 2026-11-21：原来超过 1° 直接判表失效 —— 重做一次 Z 就白丢 30 s 的辨识。
 * 齿槽表是**机械位置**的函数：编码器零点平移 δ，同一位置读到的 θ_multi
 * 就大 δ，而查表索引是 (θ_multi − phase)。所以只要把 phase 加 δ，
 * 索引自动回到原处 —— 不需要移动表里任何一个点，没有量化损失，
 * 也不需要 1 KB 的临时缓冲。对齐后 zero_ref 同步更新为当前零点。
 * 超过半圈说明不是零点平移（换装配/换电机），仍判失效。 */
#define COG_FF_ZERO_TOL (0.0175f)   /* 1°：以下直接用 */
#define COG_FF_ALIGN_MAX (3.14159f) /* 半圈：以内旋转对齐，以外判失效 */

typedef struct {
    uint8_t valid;        /* 表可用（已装载且零点一致） */
    uint8_t enabled;      /* 用户开关 */
    uint16_t n_bins;
    float   phase_deg;    /* 表 0 点的 θ_multi（deg，诊断） */
    float   zero_ref;     /* 测量时的编码器零点（rad） */
    float   zero_now;     /* 当前编码器零点（rad） */
    float   pp;           /* 峰峰值 A（来自辨识） */
    float   zero_dev_deg; /* 对齐后的残余偏差（deg，正常≈0） */
    float   align_deg;    /* 本次装载时自动对齐的角度（deg，0 = 没动） */
    uint8_t aligned;      /* 1 = 装载时做过相位对齐 */
} CogFfInfo;

void    CogFf_Init(void);

/* 从 mech_ident 的上一次 `}` 结果装载（不写 Flash） */
uint8_t CogFf_LoadFromIdent(void);
/* 从 Flash 装载 */
uint8_t CogFf_LoadFromFlash(void);
/* 写 Flash（要求 MOE=0） */
uint8_t CogFf_SaveToFlash(void);

void    CogFf_SetEnable(uint8_t en);
uint8_t CogFf_IsEnabled(void);
/* 1 = 有表且零点一致（不代表用户已开启） */
uint8_t CogFf_IsValid(void);

/* 本拍前馈电流（A）。非活动/无效/超出速度门控时返回 0 */
float   CogFf_Get(float theta_multi, float w_ramp);

void    CogFf_GetInfo(CogFfInfo *o);

/* 逐点回读：把表区间 [from, from+n) 拷到 dst（单位 mA，与内部 s_tab 一致），
 * 返回实际拷贝点数（0 = 表无效或参数越界）。
 *
 * 存在的意义：`}` 辨识完会自己装表，所以"装表"不需要上位机传数据；
 * 但**核对**需要逐点表 —— 之前只能靠 ASCII 的 8 行 `[MI] CG` 拼，
 * 那是唯一通道。飞控协议化后由 30093 分块拉取。 */
uint8_t CogFf_CopyTableMv(int16_t *dst, uint16_t from, uint8_t n);

#endif /* COG_FF_H */
