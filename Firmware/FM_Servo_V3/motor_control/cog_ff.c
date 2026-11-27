/*
 * cog_ff.c —— 齿槽转矩前馈（实现）
 * 原理与三个细节见 cog_ff.h。
 */
#include "cog_ff.h"

#include "bsp_servo.h"
#include "mech_ident.h"
#include "param_store.h"

#include <math.h>
#include <string.h>

#define COG_FF_TWO_PI   (6.28318530718f)
#define COG_FF_SCALE    (1000.0f)       /* 表存的是 iq×1000（int16，±32 A） */

static int16_t  s_tab[COG_BINS_MAX];
static uint16_t s_n;
static uint8_t  s_valid;
static uint8_t  s_en;
static float    s_phase;      /* 表 0 点的 θ_multi（rad，[0,2π)） */
static float    s_zero_ref;   /* 测量时的编码器零点（rad） */
static float    s_pp;
static float    s_align;      /* 最近一次自动相位对齐的角度（rad，诊断） */

/* 角度差归一到 ±π */
static float cog_dwrap(float a)
{
    a -= COG_FF_TWO_PI * floorf((a * (1.0f / COG_FF_TWO_PI)) + 0.5f);
    return a;
}

/* 校验当前零点与表基准；偏差在"半圈以内"就**平移相位基准**把表救回来。
 *
 * 为什么不旋转表数据：查表索引是 (θ_multi − phase)。零点平移 δ 之后，
 * 同一机械位置的 θ_multi 大 δ；只要 phase 也加 δ，索引自动回到原处。
 * 纯标量运算、零量化损失、不需要临时缓冲 —— 比逐点旋转好一个量级。
 * 对齐后 zero_ref 跟着更新，所以下一次调用的残差是 0，天然幂等。 */
static void cog_check_zero(void)
{
    float dz;

    if (s_n == 0U) { s_valid = 0U; return; }

    dz = cog_dwrap(BspServo_GetEncoderZero() - s_zero_ref);

    if (fabsf(dz) <= COG_FF_ZERO_TOL) { s_valid = 1U; return; }

    if (fabsf(dz) <= COG_FF_ALIGN_MAX) {
        s_phase    += dz;              /* 索引自动回到原处 */
        s_phase     = s_phase - COG_FF_TWO_PI
                    * floorf(s_phase / COG_FF_TWO_PI);   /* 归一化到 [0,2π) */
        s_zero_ref  = BspServo_GetEncoderZero();
        s_align     = dz;
        s_valid     = 1U;
        return;
    }

    s_valid = 0U;                      /* 超过半圈：不是零点平移，判失效 */
    s_align = 0.0f;
}

void CogFf_Init(void)
{
    memset(s_tab, 0, sizeof(s_tab));
    s_n        = 0U;
    s_valid    = 0U;
    s_en       = 0U;      /* 初值关闭：错的相位比不补更糟。
                           * 若 Flash 里存过 en=1 且零点仍一致，
                           * 下面的 LoadFromFlash 会把它恢复成 1。 */
    s_phase    = 0.0f;
    s_zero_ref = 0.0f;
    s_pp       = 0.0f;

    (void)CogFf_LoadFromFlash();
}

/* 把一份表装进缓存（内部用） */
static void cog_install(const int16_t *tab, uint16_t n, float phase,
                        float zero_ref, float pp)
{
    if ((tab == NULL) || (n == 0U) || (n > COG_BINS_MAX)) { return; }

    memcpy(s_tab, tab, (size_t)n * sizeof(int16_t));
    s_n        = n;
    s_phase    = phase;
    s_zero_ref = zero_ref;
    s_pp       = pp;
    cog_check_zero();
}

uint8_t CogFf_LoadFromIdent(void)
{
    const MechIdentResult *r = MechIdent_GetResult();

    if ((r->cog_n == 0U) || (r->cog_n > COG_BINS_MAX)) { return 0U; }
    if (r->cog_phase < 0.0f)                          { return 0U; }

    cog_install(r->cog, r->cog_n, r->cog_phase,
                BspServo_GetEncoderZero(), r->cog_pp);
    return s_valid;
}

uint8_t CogFf_LoadFromFlash(void)
{
    static int16_t tab[COG_BINS_MAX];   /* 1 KB：不能放栈上（本函数在 Init 里调） */
    uint16_t n = 0U;
    uint8_t  en = 0U;
    float   zero_ref = 0.0f, phase = 0.0f, pp = 0.0f;

    /* 两个角度都要读出来：phase 用来查表，zero_ref 用来判断"这张表还是不是
     * 当前零点下测的"。只读 phase 的话，重做 Z 之后表会静默失效并开始放大扰动。 */
    if (ParamStore_LoadCog(tab, &n, &zero_ref, &phase, &pp, &en) == 0U) { return 0U; }

    cog_install(tab, n, phase, zero_ref, pp);

    /* 开关随表恢复，但**不能在这里用 s_valid 压制**：
     * CogFf_Init 在 BspServo_Init 之前调用（servo_isr.c 的 Servo_Init 里
     * three_loop_deploy_initialize 之后就排到了 1707 行，而 BSP 初始化在 1723 行），
     * 此刻 BspServo_GetEncoderZero() 还是 0，cog_install 里的零点比对必然失败，
     * s_valid=0。若在加载时就把 en 压成 0，"上电自动恢复开关"永远不成立 ——
     * 这正是第一版实测到的现象：复位后 v=1 但 en=0。
     *
     * 真正的安全门在 CogFf_Get 里：s_en=1 但 s_valid=0 时它返回 0，
     * 而 s_valid 由运行时（Get 内分频）重新校验。 */
    s_en = (en != 0U) ? 1U : 0U;
    return s_valid;
}

uint8_t CogFf_SaveToFlash(void)
{
    if (s_n == 0U) { return 0U; }

    /* en 一起写：`;` 打开后发 A 就能跨复位保持 */
    return ParamStore_SaveCog(s_tab, s_n, s_zero_ref, s_phase, s_pp, s_en);
}

void CogFf_SetEnable(uint8_t en)
{
    /* 开启时再校验一次零点：从 Init 到现在零点可能已经变了（做过 Z） */
    if (en != 0U) { cog_check_zero(); }
    s_en = (en != 0U) ? 1U : 0U;
}

uint8_t CogFf_IsEnabled(void) { return s_en; }
uint8_t CogFf_IsValid(void)   { return s_valid; }

float CogFf_Get(float theta_multi, float w_ramp)
{
    static uint16_t s_tick;
    float a, g, ph, x, t, v;
    uint16_t i0, i1;

    /* 零点校验分频（256 拍 ≈ 128 ms @2 kHz 速度环）。
     * 为什么不能只在开/关时校验：加载发生在 BSP 初始化之前，那时零点还是 0，
     * s_valid 必然是 0；若运行时不重算，"上电自动恢复开关"会一直配着一张
     * 被判失效的表，等于开关白存。开销只是一次减法 + fabsf。 */
    if ((s_en != 0U) && (((++s_tick) & 0xFFU) == 0U)) { cog_check_zero(); }

    if ((s_en == 0U) || (s_valid == 0U) || (s_n < 2U)) { return 0.0f; }

    /* ---- 速度门控：两端都要收 ----
     * 高速端：齿槽频率超出电流环能力，且惯性已把它滤掉；
     * 低速端：黏滑爬行区，全量前馈会把力矩补偿变成速度抖动（实测 3 rpm +13%）。 */
    a = fabsf(w_ramp);
    if (a >= COG_FF_W_ZERO) { return 0.0f; }
    if (a <= COG_FF_W_LOW) {
        g = a * (1.0f / COG_FF_W_LOW);                       /* 0~10 rpm：线性起量 */
    } else if (a <= COG_FF_W_FULL) {
        g = 1.0f;                                            /* 10~191 rpm：全量 */
    } else {
        g = (COG_FF_W_ZERO - a) / (COG_FF_W_ZERO - COG_FF_W_FULL);
    }

    /* ---- 相位：相对表的 0 点，归一化到 [0,2π) ---- */
    ph = theta_multi - s_phase;
    ph -= COG_FF_TWO_PI * floorf(ph * (1.0f / COG_FF_TWO_PI));

    /* ---- 线性插值查表 ---- */
    x  = ph * ((float)s_n / COG_FF_TWO_PI);
    i0 = (uint16_t)x;
    if (i0 >= s_n) { i0 = (uint16_t)(s_n - 1U); }
    i1 = (uint16_t)((i0 + 1U) % s_n);
    t  = x - (float)i0;

    v = ((1.0f - t) * (float)s_tab[i0]) + (t * (float)s_tab[i1]);

    return (v / COG_FF_SCALE) * g;
}

void CogFf_GetInfo(CogFfInfo *o)
{
    if (o == NULL) { return; }

    cog_check_zero();      /* 每次查询都重新比对，便于诊断"做了 Z 之后表失效" */

    o->valid        = s_valid;
    o->enabled      = s_en;
    o->n_bins       = s_n;
    o->phase_deg    = s_phase * (180.0f / 3.14159265f);
    o->zero_ref     = s_zero_ref;
    o->zero_now     = BspServo_GetEncoderZero();
    o->pp           = s_pp;
    o->align_deg    = s_align * (180.0f / 3.14159265f);
    o->aligned      = (fabsf(s_align) > 1.0e-6f) ? 1U : 0U;
    o->zero_dev_deg = fabsf(cog_dwrap(o->zero_now - s_zero_ref))
                    * (180.0f / 3.14159265f);
}

uint8_t CogFf_CopyTableMv(int16_t *dst, uint16_t from, uint8_t n)
{
    uint16_t k;

    if ((dst == 0) || (s_n == 0U) || (n == 0U)) { return 0U; }
    if (from >= s_n) { return 0U; }
    if ((uint32_t)from + (uint32_t)n > (uint32_t)s_n) {
        n = (uint8_t)(s_n - from);          /* 末块余数，最多 8 */
    }
    for (k = 0U; k < (uint16_t)n; k++) {
        dst[k] = s_tab[from + k];
    }
    return n;
}
