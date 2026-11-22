/*
 * enc_comp.c —— 编码器角度误差的软件补偿（实现见 enc_comp.h 的说明）
 */
#include "enc_comp.h"

#include <math.h>
#include <string.h>

#define ENC_LUT_N       (256U)
#define ENC_TWO_PI      (6.283185307179586f)
#define ENC_COEF_LIM    (0.10f)        /* 单系数上限 0.1 rad ≈ 5.7°，防野值 */
#define ENC_LUT_SCALE   ((float)ENC_LUT_N / ENC_TWO_PI)

static float             s_lut[ENC_LUT_N + 1U];   /* 末尾复制首点，省取模 */
static float             s_a[ENC_COMP_MAX_H];
static float             s_b[ENC_COMP_MAX_H];
static volatile uint8_t  s_n;                     /* 0 = 无补偿 */
static volatile uint8_t  s_en;
static volatile uint8_t  s_lut_ok;

void EncComp_Init(void)
{
    memset(s_lut, 0, sizeof(s_lut));
    memset(s_a, 0, sizeof(s_a));
    memset(s_b, 0, sizeof(s_b));
    s_n      = 0U;
    s_en     = 0U;
    s_lut_ok = 0U;
}

static void enc_build_lut(void)
{
    uint16_t i;
    uint8_t  k;

    for (i = 0U; i <= ENC_LUT_N; i++) {
        const float x = ((float)i) * (ENC_TWO_PI / (float)ENC_LUT_N);
        float s = 0.0f;

        for (k = 0U; k < s_n; k++) {
            const float kx = ((float)(k + 1U)) * x;
            s += (s_a[k] * sinf(kx)) + (s_b[k] * cosf(kx));
        }
        s_lut[i] = s;
    }
    s_lut[ENC_LUT_N] = s_lut[0];      /* 首尾一致，插值跨 2π 不跳 */
    s_lut_ok = (s_n != 0U) ? 1U : 0U;
}

uint8_t EncComp_SetHarm(uint8_t n, const float *a, const float *b)
{
    uint8_t k;

    if (n > ENC_COMP_MAX_H) { return 0U; }
    if ((n != 0U) && ((a == 0) || (b == 0))) { return 0U; }

    for (k = 0U; k < n; k++) {
        if ((a[k] != a[k]) || (b[k] != b[k])) { return 0U; }      /* NaN */
        if ((fabsf(a[k]) > ENC_COEF_LIM) ||
            (fabsf(b[k]) > ENC_COEF_LIM))     { return 0U; }      /* 野值 */
    }

    for (k = 0U; k < ENC_COMP_MAX_H; k++) {
        s_a[k] = (k < n) ? a[k] : 0.0f;
        s_b[k] = (k < n) ? b[k] : 0.0f;
    }
    s_n = n;
    if (n == 0U) { s_en = 0U; }
    enc_build_lut();
    return 1U;
}

void EncComp_GetHarm(float *a, float *b, uint8_t *n)
{
    uint8_t k;

    for (k = 0U; k < ENC_COMP_MAX_H; k++) {
        if (a) { a[k] = s_a[k]; }
        if (b) { b[k] = s_b[k]; }
    }
    if (n) { *n = s_n; }
}

void EncComp_SetEnable(uint8_t en)
{
    s_en = ((en != 0U) && (s_lut_ok != 0U)) ? 1U : 0U;
}

uint8_t EncComp_GetEnable(void) { return s_en; }

float EncComp_Eval(float theta_m)
{
    float    x;
    uint16_t i;
    float    f;

    if (s_en == 0U) { return 0.0f; }

    x = theta_m * ENC_LUT_SCALE;
    if (x < 0.0f) { x += (float)ENC_LUT_N; }
    /* theta_m 已归一到 [-2π, 2π)，但保留一次兜底，避免调用方给多圈角 */
    if (x >= (float)ENC_LUT_N) { x -= (float)ENC_LUT_N; }
    if (x < 0.0f) { x = 0.0f; }

    i = (uint16_t)x;
    f = x - (float)i;
    return s_lut[i] + (f * (s_lut[i + 1U] - s_lut[i]));
}
