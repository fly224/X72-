/**
  ******************************************************************************
  * @file    mech_ident.h
  * @brief   机械环节辨识：谐振扫频 / 摩擦曲线 / 折算惯量 / 齿槽转矩
  ******************************************************************************
  * 为什么这四项放一个模块：
  *   它们的激励源、测量通道、安全联锁完全一样，只是"怎么扫、扫完算什么"不同。
  *   分开写会有四套重复的进环/退环/超时/故障判断，任何一处漏改就是一次飞车。
  *
  * 共同前提（四项都一样）：
  *   1) 必须在 D 状态启动（本模块自己进电流环、自己封锁）
  *   2) 必须有 C（电流零点）+ Z（电角度对齐）
  *   3) 全程用**未滤波**的单拍速度 s_omega_fast：
  *      正常测速是 8 拍累计 + 95 Hz 低通（BSP_SPEED_LPF_ALPHA=0.03），
  *      100 Hz 以上的谐振会被它直接抹掉 —— 那正是要测的东西。
  *
  * 四种模式（用 `_<n>` 选择，复用数值命令解析器）：
  *   _1  谐振扫频：iq 正弦扫 20..1000 Hz，同步解调出 |w/i| 与相位 → Bode
  *   _2  摩擦曲线：8 个转速点的稳态 iq，线性回归出 fric/visc，同时输出全表供 Stribeck 拟合
  *   _3  折算惯量：两个转矩幅值下的加速度差分，消去摩擦项
  *   _4  齿槽转矩：低速匀速转 1 圈，iq 按机械角分 128 箱平均
  ******************************************************************************
  */

#ifndef __MECH_IDENT_H
#define __MECH_IDENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    MI2_IDLE = 0,
    MI2_SETTLE,      /* 进环/换频点后的稳定等待 */
    MI2_MEAS,        /* 正在累加 */
    MI2_DONE,
    MI2_FAIL
} MechIdentState;

typedef enum {
    MI2_OK = 0,
    MI2_ERR_BUSY,
    MI2_ERR_CAL,
    MI2_ERR_ZERO,
    MI2_ERR_FAULT,
    MI2_ERR_MODE,
    MI2_ERR_MOTION,  /* 没转起来 / 速度跟不上 */
    MI2_ERR_DATA     /* 数据不可信（幅值太小、相位乱跳） */
} MechIdentErr;

#define MI2_SWEEP_PTS    (11U)
#define MI2_FRIC_PTS     (8U)
/* 齿槽表箱数：容量与默认值分开（2026-11-21 加密到 256/512）。
 * 128 箱在 14 对极下每齿槽周期只有 9 个点，线性插值把峰顶削掉一大截；
 * 实测 100 rpm 速度波动 σ=3.96 rpm，齿槽前馈只砍掉 36% 的扰动。
 * 512 箱 → 每周期 36 点、机械分辨率 0.7°，代价是 Flash 1 KB + 测量内存 5 KB。 */
#define MI2_COG_BINS_MAX  (512U)
#define MI2_COG_BINS_DFLT (512U)
#define MI2_COG_BINS     (MI2_COG_BINS_DFLT)

typedef struct {
    float freq;      /* Hz */
    float gain;      /* |w| / |i|，rad/s per A */
    float phase;     /* deg，已补偿采样延迟 */
} MechSweepPt;

typedef struct {
    float rpm;       /* 目标转速 */
    float rpm_act;   /* 实测 */
    float iq;        /* 稳态 iq（A） */
} MechFricPt;

typedef struct {
    uint8_t  mode;   /* 1..4 */
    uint8_t  err;

    /* _1 扫频 */
    uint8_t  sweep_n;
    MechSweepPt sweep[MI2_SWEEP_PTS];

    /* _2 摩擦 */
    uint8_t  fric_n;
    MechFricPt  fric[MI2_FRIC_PTS];
    float    fric_a;      /* 线性回归截距（A）= 库仑项 */
    float    visc;        /* 线性回归斜率（A/(rad/s)） */
    /* 诊断量（**不参与标定**，2026-11-21）
     * 这组数据来自单向 8 点扫描，重复性 ±12%~21%，只适合看特性、不适合定参数；
     * 标定参数请用 `$`（action 9，三点正反，重复性 0.3%）。
     *   fric_iq_lo：最低速三点（1/2/3 rpm）里 |iq| 的最大值。
     *     注意它**不是**严格意义上的静摩擦：1/2/3 rpm 落在粘滑区，同一个点
     *     不同次能测出 0.61 / 0.256 / 0.077 A（历史值），量级都不稳定。
     *     它能说明的是"最低速维持电流远大于动摩擦"这件事，不能当定量常数。
     *   fric_ratio：fric_iq_lo / 动摩擦区最小 iq。实测 2~6 倍之间跳，
     *     只能当"低速性能是否明显恶化"的粗指标。
     * 真正可靠的 ff_boost 依据是"从静止起转的超调量"（见前馈接入报告），
     * 不是这里的比值。 */
    float    fric_iq_lo;
    float    fric_ratio;

    /* _1 扫频的诊断量（只对第一个频点记录）
     * 用来区分"响应真的没有"和"解调通道坏了"：
     *   wmean=0        -> 差分通道有问题（theta 没推进 / 索引算错）
     *   wmean>0 但 amp 小 -> 真的没响应 */
    float    dbg_wmean;   /* Σ|w|/n，差分的平均绝对值 rad/s */
    float    dbg_wamp;    /* 解调得到的速度幅值 rad/s */
    float    dbg_iamp;    /* 解调得到的电流幅值 A */

    /* _3 惯量 */
    float    j_eq;        /* kg·m^2（基于 psi 实测值，psi 偏多少 J 就偏多少） */
    float    acc1, acc2;  /* rad/s^2 */
    float    tq1, tq2;    /* N·m */

    /* _4 齿槽（正反各一圈后取平均）
     * 为什么要两向：摩擦的**常值偏置**在正转时是 +Tc、反转时是 -Tc，
     * 单向测量会把 Tc 当成齿槽的直流分量留在表里；两向平均后 Tc 抵消，
     * 只剩真正与角度相关的齿槽分量。 */
    uint16_t cog_n;
    int16_t  cog[MI2_COG_BINS_MAX];   /* 平均后的 iq 表，缩放 1000（±32 A 不溢出） */
    float    cog_pp;              /* 平均后的峰峰值 A */
    float    cog_pp_fwd;          /* 正向单圈的峰峰值 A（诊断：与反向差多少） */
    float    cog_pp_rev;          /* 反向单圈的峰峰值 A */
    float    cog_dc;              /* 正向表均值 A（应接近摩擦常值；平均后应≈0 的偏移量） */
    /* 表的相位参考：bin 0 对应 theta_multi 的哪个角度（rad，已归一化到 [0,2π)）。
     * 齿槽表是按 rel = theta_multi − s_cog_th0 分箱的，所以 0 点不是编码器零点
     * 而是**测量起点**。做前馈时必须把它一起存下来，否则查表会整体偏移。
     * 而 s_cog_th0 本身是相对的（θ_multi 从编码器零点起算），所以重做 Z 之后
     * 这个参考就失效了 —— 这一点由 CogFf 侧的 zero_ref 校验兜住。 */
    float    cog_phase;
} MechIdentResult;

MechIdentErr          MechIdent_Start(uint8_t mode);

/* 齿槽表箱数（只允许 128/256/512，默认 256）。必须在 `}` 测量前设置。 */
void                  MechIdent_SetCogBins(uint16_t n);
uint16_t              MechIdent_GetCogBins(void);
void                  MechIdent_Abort(void);
MechIdentState        MechIdent_GetState(void);
uint8_t               MechIdent_IsBusy(void);
const MechIdentResult *MechIdent_GetResult(void);
void                  MechIdent_Task1ms(void);
void                  MechIdent_Init(void);
const char           *MechIdent_ErrStr(MechIdentErr e);

/* ISR 侧：频率推进 + 同步解调。只在电流环分支里调用一次/拍。 */
void                  MechIdent_Step(float omega_fast, float iq_now, float theta_multi);
/* ISR 侧：本拍是否由辨识接管 iq 给定（是则返回给定值） */
uint8_t               MechIdent_Refs(float *iq_ref);

#ifdef __cplusplus
}
#endif

#endif /* __MECH_IDENT_H */
