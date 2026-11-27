/*
 * motor_flux_ident.h —— 磁链 ψ 在线辨识（配合 R/L，消除 J 的 ψ 耦合）
 *
 * ============================ 要解决的问题 ============================
 *
 * 折算惯量 J 由转矩算：T = Kt·iq = 1.5·p·ψ·iq。所以 **J ∝ 1/ψ**。
 * 之前 J 用 Kt=0.0936（ψ=4.455 mWb，由 3000 rpm 电压饱和点反推）算出
 * 5.32e-5 kg·m²；若 ψ 真值是模型的 7.593 mWb，J 就是 9.07e-5，差 41%。
 * 这个误差会 1:1 进到加速度前馈系数 J/Kt，也进到所有转矩换算。
 *
 * 单编码器 + 无相电压采样时，最干净的办法是"断开 PWM 让转子滑行、示波器量
 * 相电压"。但 X72 的 ua/ub/uc 是 (duty−0.5)·vbus 的**指令值**（telemetry_tx.c:217），
 * 网表里只有 VBUS_ADC，没有相电压分压网络 —— 这条路要改板。
 *
 * ============================ 采用的办法 ============================
 *
 * 稳态 dq 电压方程（表贴式 Ld=Lq=L，稳态下 di/dt=0）：
 *
 *     vd = R·id − ωe·L·iq
 *     vq = R·iq + ωe·L·id + ωe·ψ
 *
 * 关键洞察：**在同一个转速下扫 id，两个方程对 id 的斜率分别是**
 *
 *     ∂vd/∂id = R                      ← R 直接出来，与摩擦/iq 无关
 *     ∂vq/∂id = ωe·L                   ← L = 斜率/ωe
 *
 * 这两个斜率只依赖 id，不依赖 iq。而空载时 iq 是被摩擦"锁"在 ωe 上的
 * （iq ≈ 摩擦曲线），如果用转速扫描去解 R 和 ψ，会得到
 *     vq = R·a + (R·b + ψ)·ωe
 * 只能解出 (R·b + ψ) 这个组合 —— **完全共线，原理上无法分离**。
 * id 是我们能独立给定的第二个维度，这才让方程满秩。
 *
 * 所以流程是：
 *   ① 主转速 1000 rpm，id = 0 / −1 / −2 / −3 A 四点
 *        · vd 对 id 线性回归 → R
 *        · vq 对 id 线性回归 → L = 斜率/ωe
 *        · 每点算 ψ_k = (vq − R·iq − ωe·L·id)/ωe，平均 → ψ
 *   ② 副转速 300 rpm，id = 0 / −2 A 两点
 *        · 用已求的 R、L 反算 ψ_2 → 与 ψ 比对，偏差 >15% 判不可信
 *
 * 为什么主转速选 1000 rpm：vq 对 id 的斜率 = ωe·L；
 *   L=80 µH 时 1000 rpm → 0.117 V/A，id 扫 3 A 共 0.35 V，ADC 能分辨；
 *   300 rpm 只有 0.035 V/A，3 A 才 0.1 V，接近死区电压量级。
 *
 * 为什么副转速选 300 rpm 仍能验 ψ：vq 里 ωe·ψ 项在 300 rpm 是
 *   440×0.0045 = 2.0 V，而 R·iq 只有 0.05 V —— ψ 的贡献占 97%，很好测。
 *
 * ============================ 与 @ 的关系 ============================
 *
 * `@`（motor_rl_ident）测的 R/L 是在**虚拟角锁死、ωe=0** 下测的：
 *   · 好处：不受 ωe 项污染，R 很准（实测 ±1.2%）
 *   · 限制：L 只能用注入法（时域阶跃分辨率不够），且测的是小信号 Ld
 * 本模块测的是**实际工作点、实际转速**下的 R/L/ψ：
 *   · 好处：直接服务于转矩换算与电压预算
 *   · 限制：ωe 项与死区电压混在 vq 里，L 的精度不如注入法
 * 两者互为交叉验证：R 应接近（±10%），L 应接近（±25%）。
 *
 * ============================ 安全 ============================
 *   · 要求 C（电流零点）、Z（电角度）、无故障、MOE=0 启动
 *   · 全程走速度环，iq 由速度环给出（限 3 A），id 由本模块给（限 3 A）
 *   · 辨识期间**强制关闭 5/7 次谐波抑制**（它会给 αβ 叠电压，
 *     而本模块读的 vd/vq 是限幅后、谐波叠加前的值，开着会让方程多一项）
 *   · 结束必然封锁 MOE
 */
#ifndef MOTOR_FLUX_IDENT_H
#define MOTOR_FLUX_IDENT_H

#include <stdint.h>

#define FI_PTS_MAX   (6U)

typedef enum {
    FI_IDLE = 0,
    FI_SETTLE,      /* 稳速 + 电流建立 */
    FI_SAMPLE,      /* 累加 vd/vq/id/iq/we */
    FI_DONE,
    FI_FAIL,
} FluxIdentState;

typedef enum {
    FI_OK = 0,
    FI_ERR_BUSY = 1,
    FI_ERR_CAL = 2,
    FI_ERR_ZERO = 3,
    FI_ERR_FAULT = 4,
    FI_ERR_PWM = 5,
    FI_ERR_DATA = 6,        /* 回归退化了（斜率不显著 / 残差过大） */
    FI_ERR_RANGE = 7,       /* 结果超出物理范围 */
    FI_ERR_PSI = 8,         /* 两个转速算出的 ψ 不一致 */
} FluxIdentErr;

typedef struct {
    float id;
    float iq;
    float vd;
    float vq;
    float we;               /* 电角速度 rad/s（实测，带符号） */
} FluxPt;

typedef struct {
    /* ---- 结果 ---- */
    float r_ohm;
    float l_h;
    float psi_wb;
    float kt;               /* 1.5·p·ψ，派生值，便于直接换转矩 */
    float ke_v_s_rad;       /* p·ψ，反电势常数 */

    /* ---- 拟合质量 ---- */
    float r_r2;             /* vd-id 回归的 R² */
    float l_r2;             /* vq-id 回归的 R² */
    float psi_dev_pct;      /* |ψ_2 − ψ_1| / ψ_1 */
    float psi1;             /* 1000 rpm 组算出的 ψ */
    float psi2;             /* 300 rpm 组算出的 ψ（用 R/L 反算） */
    float we1;
    float we2;

    /* ---- 失败现场（只在 FAIL 时有意义）----
     * 只回一句 "fault present" 无法定位是超速、过流还是编码器。
     * 这三个量能把"速度环方向反了"和"目标转速本身不可达"区分开。 */
    float dbg_om;        /* 失败瞬间的机械角速度 rad/s */
    float dbg_wref;      /* 失败瞬间的速度给定 rad/s */
    uint8_t dbg_spd_en;
    uint8_t dbg_cur_en;
    uint8_t dbg_fault;

    /* ---- 原始数据（逐点回显用） ---- */
    uint8_t   n_pts;      /* 计划点数（FI_N_PTS） */
    uint8_t   n_done;     /* 实际已采到的点数：失败时也能把前面的点吐出来，
                           * 否则"副组才失败"和"第一点就没数据"看起来一模一样 */
    FluxPt    pt[FI_PTS_MAX];
    FluxIdentErr err;
} FluxIdentResult;

void           FluxIdent_Init(void);
FluxIdentErr   FluxIdent_Start(void);
void           FluxIdent_Abort(void);
uint8_t        FluxIdent_IsBusy(void);
FluxIdentState FluxIdent_GetState(void);
const FluxIdentResult *FluxIdent_GetResult(void);
const char    *FluxIdent_ErrStr(FluxIdentErr e);

/* 从 Flash 恢复上次结果（只用于回显与对照，不改任何控制参数） */
void           FluxIdent_SetCached(float r_ohm, float l_h, float psi_wb);

/* ---- ISR 侧 ---- */
/* 本拍 id 给定（非活动时返回 0）。速度环分支里用它替代 id_ref=0 */
float          FluxIdent_GetIdRef(void);
/* 记录本拍实测，供累加。非活动时立即返回 */
void           FluxIdent_Step(float id_now, float iq_now, float vd, float vq, float we);

/* ---- 1 ms ---- */
uint8_t        FluxIdent_Task1ms(void);

#endif /* MOTOR_FLUX_IDENT_H */
