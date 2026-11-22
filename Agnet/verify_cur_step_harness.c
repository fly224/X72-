/*
 * verify_cur_step_harness.c —— 主机侧驱动真实的 cur_step.c，验证指标算法
 *
 * 用理想对象（R/L + PI）产生一次 1 A 阶跃，逐拍按 ISR 的顺序调用
 *   CurStep_Refs() -> 控制律 -> CurStep_Push()
 * 然后打印模块算出的 tr / os / ts / ss，与解析预期对比。
 *
 * 编译（Windows Git Bash）：
 *   gcc -I Board Agnet/verify_cur_step_harness.c Board/cur_step.c -lm -o fw_step.exe
 */
#include <math.h>
#include <stdio.h>

#include "cur_step.h"

#define KP     0.71314153f
#define KITS   0.11890653f
#define TS     5e-5
#define L      113.5e-6
#define R      0.378481f
#define VBUS   30.78f
#define VLIM   (0.5375f * VBUS)

int main(void)
{
    float i = 0.0f, integ = 0.0f;
    int   k;
    int   fails = 0;

    CurStep_Start(CUR_STEP_ID, 1.0f);

    for (k = 0; k < 2000; k++) {
        float id_ref = 0.0f, iq_ref = 0.0f;
        float e, v, i_next;
        float id_now, iq_now, vd, vq;

        if (CurStep_IsActive() == 0U) { break; }
        (void)CurStep_Refs(&id_ref, &iq_ref);

        e = id_ref - i;
        integ += e * KITS;
        v = KP * e + integ;
        if (v > VLIM)  { v = VLIM;  integ *= 0.98f; }
        if (v < -VLIM) { v = -VLIM; integ *= 0.98f; }

        id_now = i;
        iq_now = 0.0f;
        vd = v;
        vq = 0.0f;

        CurStep_Push(id_now, iq_now, vd, vq);

        i_next = i + (v - R * i) / L * TS;
        i = i_next;
    }

    {
        const CurStepResult *r = CurStep_GetResult(0);
        printf("模块输出: valid=%u amp=%.2f tr=%.0f us os=%+.2f%% ts=%.0f us ss=%+.2f%% pk=%.4f vp=%.2f\n",
               r->valid, r->amp, r->rise_us, r->os_pct, r->settle_us, r->ss_err_pct, r->peak, r->vpeak);

        /* 解析预期：一阶 + 极点抵消 PI -> 近似一阶，时间常数 L/Kp = 159 us
         * 10%->90% = ln(9)*tau = 2.197*159 = 349 us（含离散与限幅略有偏差） */
        if (r->valid == 0U) { printf("  [FAIL] valid=0\n"); fails++; }
        if (fabsf(r->rise_us - 349.0f) > 120.0f) {
            printf("  [FAIL] tr=%.0f us，预期约 349+-120 us\n", r->rise_us); fails++;
        } else {
            printf("  [PASS] tr=%.0f us（解析 349 us +-120）\n", r->rise_us);
        }
        if (fabsf(r->os_pct) > 2.0f) {
            printf("  [FAIL] os=%.2f%%，极点抵消后应无超调\n", r->os_pct); fails++;
        } else {
            printf("  [PASS] os=%+.2f%%（应≈0）\n", r->os_pct);
        }
        if (fabsf(r->ss_err_pct) > 0.5f) {
            printf("  [FAIL] ss=%.2f%%\n", r->ss_err_pct); fails++;
        } else {
            printf("  [PASS] ss=%+.2f%%\n", r->ss_err_pct);
        }
        if (r->peak < 0.98f || r->peak > 1.02f) {
            printf("  [FAIL] pk=%.4f\n", r->peak); fails++;
        } else {
            printf("  [PASS] pk=%.4f\n", r->peak);
        }
        if (fabsf(r->settle_us - 349.0f) > 400.0f) {
            printf("  [FAIL] ts=%.0f us 偏离过大\n", r->settle_us); fails++;
        } else {
            printf("  [PASS] ts=%.0f us\n", r->settle_us);
        }
    }

    printf("\n波形（每点 %.1f ms）:\n", 50.0f * 18.0f / 1000.0f);
    {
        uint16_t n = CurStep_GetTraceN();
        uint16_t j;
        printf("  n=%u\n", n);
        for (j = 0; j < n && j < 6; j++) {
            float t, ir, in, qr, qn;
            CurStep_GetTracePoint(j, &t, &ir, &in, &qr, &qn);
            printf("  t=%5.2f id_ref=%.3f id=%.3f iq_ref=%.3f iq=%.3f\n", t, ir, in, qr, qn);
        }
        if (n < 30) { printf("  [FAIL] 波形点数 %u < 30\n", n); fails++; }
        else        { printf("  [PASS] 波形点数 %u\n", n); }
    }

    /* 一致性：追加大幅阶跃，验证限幅与 os 计算 */
    {
        float i2 = 0.0f, integ2 = 0.0f;
        int m;
        CurStep_Start(CUR_STEP_IQ, 3.0f);
        for (m = 0; m < 2000; m++) {
            float id_ref = 0.0f, iq_ref = 0.0f;
            float e, v;
            if (CurStep_IsActive() == 0U) { break; }
            (void)CurStep_Refs(&id_ref, &iq_ref);
            e = iq_ref - i2;
            integ2 += e * KITS;
            v = KP * e + integ2;
            if (v > VLIM)  { v = VLIM;  integ2 *= 0.98f; }
            if (v < -VLIM) { v = -VLIM; integ2 *= 0.98f; }
            CurStep_Push(0.0f, i2, 0.0f, v);
            i2 = i2 + (v - R * i2) / L * TS;
        }
        {
            const CurStepResult *q = CurStep_GetResult(1);
            printf("\n3 A iq 阶跃: valid=%u tr=%.0f us os=%+.2f%% ss=%+.2f%% pk=%.3f vp=%.2f\n",
                   q->valid, q->rise_us, q->os_pct, q->ss_err_pct, q->peak, q->vpeak);
            if (q->valid == 0U || fabsf(q->peak - 3.0f) > 0.06f) {
                printf("  [FAIL] 3 A 阶跃峰值异常\n"); fails++;
            } else {
                printf("  [PASS] 峰值跟踪 3 A\n");
            }
        }
    }

    printf("\n%s（fails=%d）\n", (fails == 0) ? "全部 PASS" : "有 FAIL", fails);
    return (fails == 0) ? 0 : 1;
}
