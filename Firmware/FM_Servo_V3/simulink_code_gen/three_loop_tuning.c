/*
 * three_loop_tuning.c —— 三环 PID 参数对外接口实现（手工层，不随 codegen 覆盖）
 *
 * 数值来源：three_loop_params.m（Ts_i = 50 us / 20 kHz 设计节拍）。
 * 固定项 dt / N_w / N_p 由 Ts_i 派生：5e-5 / 10 / 20（速度环 2 kHz、位置环 1 kHz）。
 * 生成代码中的 U.par_c[11] / U.par_w[5] / U.par_p[4] 由本文件在初始化时填入。
 * 周期相关字段 (dt, Ts_w, Ts_p, N_w, N_p) 与电机标定字段 (Ld,Lq,psi,dec,...)
 * 属于固定项，只允许从本文件派生，不开放给上位机，防止写坏控制时序。
 */

#include "three_loop_tuning.h"
#include <stddef.h>
#include <math.h>

/* ---------------- 固定项（禁止对外改写） ---------------- */
#define TL_TS_I        (50.0e-6)           /* 控制 ISR 周期 [s]，20 kHz */
#define TL_TS_W        (500.0e-6)          /* 速度环 2 kHz */
#define TL_TS_P        (1.0e-3)            /* 位置环 1 kHz */
#define TL_N_W         (10.0)              /* 20 kHz / 10 = 2 kHz */
#define TL_N_P         (20.0)              /* 20 kHz / 20 = 1 kHz */
#define TL_LD          (113.5e-6)
#define TL_LQ          (113.5e-6)
#define TL_PSI         (7.59297995988e-3)
#define TL_OVERMOD     (1.0)
#define TL_MAG_VD_MAX  (0.98)
#define TL_DEC         (3.0)               /* 交叉耦合 + BEMF 全解耦 */
#define TL_POLE_PAIRS  (14.0)
#define TL_GAIN_COMP   (TL_POLE_PAIRS * 1.5 * TL_TS_I)   /* 0.00105 */
#define TL_TWO_PI      (6.28318530717958647692)

/* 默认值：与 three_loop_params.m / 生成模型 SIL 输入逐项一致 */
const ThreeLoop_Tuning_T THREE_LOOP_TUNING_DEFAULT = {
    0.713141532365,      /* kp_i  [V/A]          */
    2378.18563877,       /* ki_i  [V/(A*s)]      */
    0.0729227202682,     /* kp_w  [A/(rad/s)]    */
    10.1906891515,       /* ki_w  [A/(rad/s)/s]  */
    50.2654824574,       /* kp_p  [(rad/s)/rad]  */
    120.0,               /* w_max [rad/s]        */
    9.0,                 /* i_max [A]            */
    0.95                 /* max_duty             */
};

/* ---------------- 参数描述表（可写项） ---------------- */
typedef struct {
    uint8_t id;
    double  scale;      /* raw = value * scale */
    double  lo, hi;
    size_t  off;        /* 在 ThreeLoop_Tuning_T 内的偏移 */
} TL_Desc_T;

static const TL_Desc_T TL_DESC[] = {
    { TL_PID_CUR_KP,  1.0e3,    0.05,     3.0,    offsetof(ThreeLoop_Tuning_T, kp_i)     },
    { TL_PID_CUR_KI,  1.0e3,  100.0,  10000.0,    offsetof(ThreeLoop_Tuning_T, ki_i)     },
    { TL_PID_VEL_KP,  1.0e6,    0.0,      1.0,    offsetof(ThreeLoop_Tuning_T, kp_w)     },
    { TL_PID_VEL_KI,  1.0e3,    0.0,    100.0,    offsetof(ThreeLoop_Tuning_T, ki_w)     },
    { TL_PID_POS_KP,  1.0e3,    0.0,    200.0,    offsetof(ThreeLoop_Tuning_T, kp_p)     },
    { TL_PID_VEL_MAX, 1.0e3,    1.0,    200.0,    offsetof(ThreeLoop_Tuning_T, w_max)    },
    { TL_PID_I_MAX,   1.0e3,    0.5,      9.0,    offsetof(ThreeLoop_Tuning_T, i_max)    },
    { TL_PID_DUTY_MAX,1.0e3,    0.5,     0.98,    offsetof(ThreeLoop_Tuning_T, max_duty) }
};
#define TL_DESC_N (sizeof(TL_DESC) / sizeof(TL_DESC[0]))

static double field_get(const ThreeLoop_Tuning_T *t, size_t off)
{
    /* 单精度结构体：必须按 4 字节 real32_T 读取，再提升为 double 计算 */
    return (double)(*(const real32_T *)((const uint8_t *)t + off));
}
static void field_set(ThreeLoop_Tuning_T *t, size_t off, double v)
{
    *(real32_T *)((uint8_t *)t + off) = (real32_T)v;
}

/* ---------------- 临界区（可选弱钩子） ----------------
 * 控制 ISR 会抢占 1 ms 参数任务。默认空实现，靠「使能态拒写 + 一次性展开」
 * 保证安全；若要在运行中不停车改参数，请在 BSP 层覆盖这两个弱函数
 * （ARM: __disable_irq() / __enable_irq()，注意临界区必须极短）。 */
#if defined(__GNUC__) || defined(__ARMCC_VERSION)
#define TL_WEAK __attribute__((weak))
#else
#define TL_WEAK
#endif
TL_WEAK void ThreeLoop_EnterCritical(void) { }
TL_WEAK void ThreeLoop_ExitCritical(void)  { }

/* 不引入 string.h 的小循环，避免部分裸机工程裁剪 libc 后链接失败 */
static void memcpy_u(real32_T *dst, const real32_T *src, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; ++i) {
        dst[i] = src[i];
    }
}

/* ---------------- 展开到 U ---------------- */
static void expand(const ThreeLoop_Tuning_T *t,
                   real32_T *pc, real32_T *pw, real32_T *pp)
{
    /* par_c = [kp ki dt Ld Lq psi max_duty overmod mag_vd_max dec imax] */
    pc[0]  = t->kp_i;
    pc[1]  = t->ki_i;
    pc[2]  = TL_TS_I;
    pc[3]  = TL_LD;
    pc[4]  = TL_LQ;
    pc[5]  = TL_PSI;
    pc[6]  = t->max_duty;
    pc[7]  = TL_OVERMOD;
    pc[8]  = TL_MAG_VD_MAX;
    pc[9]  = TL_DEC;
    pc[10] = t->i_max;

    /* par_w = [kp_w ki_w Ts_w i_max N_w] */
    pw[0] = t->kp_w;
    pw[1] = t->ki_w;
    pw[2] = TL_TS_W;
    pw[3] = t->i_max;          /* 与 par_c[10] 同步 */
    pw[4] = TL_N_W;

    /* par_p = [kp_p w_max Ts_p N_p] */
    pp[0] = t->kp_p;
    pp[1] = t->w_max;
    pp[2] = TL_TS_P;
    pp[3] = TL_N_P;
}

void ThreeLoop_Tuning_Init(void)
{
    ThreeLoop_Tuning_Apply(&THREE_LOOP_TUNING_DEFAULT);
}

void ThreeLoop_Tuning_Apply(const ThreeLoop_Tuning_T *t)
{
    real32_T pc[11], pw[5], pp[4];
    if (t == NULL) {
        return;
    }
    expand(t, pc, pw, pp);

    ThreeLoop_EnterCritical();
    memcpy_u(&three_loop_deploy_U.par_c[0], pc, 11);
    memcpy_u(&three_loop_deploy_U.par_w[0], pw, 5);
    memcpy_u(&three_loop_deploy_U.par_p[0], pp, 4);
    ThreeLoop_ExitCritical();
}

void ThreeLoop_Tuning_Get(ThreeLoop_Tuning_T *t)
{
    if (t == NULL) {
        return;
    }
    t->kp_i     = three_loop_deploy_U.par_c[0];
    t->ki_i     = three_loop_deploy_U.par_c[1];
    t->max_duty = three_loop_deploy_U.par_c[6];
    t->i_max    = three_loop_deploy_U.par_c[10];
    t->kp_w     = three_loop_deploy_U.par_w[0];
    t->ki_w     = three_loop_deploy_U.par_w[1];
    t->kp_p     = three_loop_deploy_U.par_p[0];
    t->w_max    = three_loop_deploy_U.par_p[1];
}

ThreeLoop_ParamStatus_T ThreeLoop_Tuning_Validate(const ThreeLoop_Tuning_T *t)
{
    size_t i;
    double v;
    if (t == NULL) {
        return THREE_LOOP_OUT_OF_RANGE;
    }
    for (i = 0; i < TL_DESC_N; ++i) {
        v = field_get(t, TL_DESC[i].off);
        if (!isfinite(v) || v < TL_DESC[i].lo || v > TL_DESC[i].hi) {
            return THREE_LOOP_OUT_OF_RANGE;
        }
    }
    /* 交叉约束：电流环零点 ki/kp 应落在合理区间，避免写成纯积分或纯比例 */
    if (t->kp_i <= 0.0 || t->ki_i <= 0.0) {
        return THREE_LOOP_OUT_OF_RANGE;
    }
    return THREE_LOOP_OK;
}

/* ---------------- database 回调入口 ---------------- */
ThreeLoop_ParamStatus_T ThreeLoop_ParamSet(uint8_t id, int32_t raw, uint8_t is_enabled)
{
    size_t i;
    double v;
    ThreeLoop_Tuning_T shadow;

    /* 只读 ID */
    if (id >= TL_RO_CTRL_TS_US && id <= TL_RO_POLE_PAIRS) {
        return THREE_LOOP_READ_ONLY;
    }
    /* 使能态拒写：与协议层 APP_PARAM_WRITE_STATE_ERROR 对齐 */
    if (is_enabled) {
        return THREE_LOOP_WRITE_STATE_ERROR;
    }
    for (i = 0; i < TL_DESC_N; ++i) {
        if (TL_DESC[i].id != id) {
            continue;
        }
        v = (double)raw / TL_DESC[i].scale;
        if (!isfinite(v) || v < TL_DESC[i].lo || v > TL_DESC[i].hi) {
            return THREE_LOOP_OUT_OF_RANGE;
        }
        ThreeLoop_Tuning_Get(&shadow);          /* 影子 = 当前生效值 */
        field_set(&shadow, TL_DESC[i].off, v);
        if (ThreeLoop_Tuning_Validate(&shadow) != THREE_LOOP_OK) {
            return THREE_LOOP_OUT_OF_RANGE;
        }
        ThreeLoop_Tuning_Apply(&shadow);        /* 一次性原子提交 */
        return THREE_LOOP_OK;
    }
    return THREE_LOOP_UNKNOWN_ID;
}

ThreeLoop_ParamStatus_T ThreeLoop_ParamGet(uint8_t id, int32_t *raw)
{
    size_t i;
    double v;
    ThreeLoop_Tuning_T cur;

    if (raw == NULL) {
        return THREE_LOOP_OUT_OF_RANGE;
    }
    ThreeLoop_Tuning_Get(&cur);
    for (i = 0; i < TL_DESC_N; ++i) {
        if (TL_DESC[i].id == id) {
            v = field_get(&cur, TL_DESC[i].off);
            *raw = (int32_t)(v * TL_DESC[i].scale + (v >= 0.0 ? 0.5 : -0.5));
            return THREE_LOOP_OK;
        }
    }
    switch (id) {
    case TL_RO_CTRL_TS_US: *raw = (int32_t)(TL_TS_I * 1.0e6);          break;  /* 100 */
    case TL_RO_N_W:        *raw = (int32_t)TL_N_W;                      break;  /* 5   */
    case TL_RO_N_P:        *raw = (int32_t)TL_N_P;                      break;  /* 10  */
    case TL_RO_GAIN_COMP:  *raw = (int32_t)(TL_GAIN_COMP * 1.0e6);     break;  /* 2100 */
    case TL_RO_POLE_PAIRS: *raw = (int32_t)TL_POLE_PAIRS;               break;  /* 14  */
    default: return THREE_LOOP_UNKNOWN_ID;
    }
    return THREE_LOOP_OK;
}

/* ---------------- 位置坐标系折叠 ----------------
 * 文档 §3.3：pos_ref 是多圈累积，theta_m 是单圈 0~2π；直接相减跨圈会跳 2π，
 * 位置环输出瞬间饱和到 ±120 rad/s -> iq 限幅 9 A -> 过流跳闸。
 * 返回折叠后的 pos_ref，使模型内误差 = wrap_pi(真实误差)。 */
real32_T ThreeLoop_WrapPosRefEx(real32_T pos_ref_multi, real32_T theta_m, real32_T dir_sign)
{
    real32_T ds = (dir_sign >= 0.0) ? 1.0 : -1.0;
    real32_T th = ds * theta_m;
    real32_T e  = ds * (pos_ref_multi - theta_m);
    e -= TL_TWO_PI * floor(e * (1.0 / TL_TWO_PI) + 0.5);
    return th + e;
}

real32_T ThreeLoop_WrapPosRef(real32_T pos_ref_multi, real32_T theta_m)
{
    return ThreeLoop_WrapPosRefEx(pos_ref_multi, theta_m, 1.0);
}
