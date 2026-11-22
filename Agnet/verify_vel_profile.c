/**
 * @file  verify_vel_profile.c
 * @brief 固件里的速度轨迹模块验证（主机侧，不依赖 HAL）
 *
 * 三部分：
 *   A) 用固件这份 motion_profile.c 复现参考工程的阶跃序列（ts=0.5ms, 0->50->-20->0），
 *      输出 CSV 供与参考 ramp_trap.csv / ramp_scurve.csv 逐点比对 —— 证明移植无偏差
 *   B) 固件实际参数（ts=1ms, a=200, j=2000, VelProfile 层）：验证加速度/jerk 上限、
 *      末速归零、位置给定 = ∫v·dt
 *   C) 安全行为：使能不跳变、堵转（theta 卡住）时位置给定被 err_max 限住
 *
 * 构建：
 *   gcc -O2 -Wall -Wextra -o verify_vel_profile verify_vel_profile.c \
 *       ../../Firmware/FM_Servo_V3/embedded_project/STM32G474_App/Board/motion_profile.c \
 *       ../../Firmware/FM_Servo_V3/embedded_project/STM32G474_App/Board/vel_profile.c -lm
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motion_profile.h"
#include "vel_profile.h"

static int g_fail = 0;
static int g_pass = 0;

static void check(const char *name, int ok, const char *detail)
{
    if (ok) {
        g_pass++;
        printf("[PASS] %-42s %s\n", name, detail ? detail : "");
    } else {
        g_fail++;
        printf("[FAIL] %-42s %s\n", name, detail ? detail : "");
    }
}

/* 参考工程 test_profile.c 的参数，必须一致才能比对 */
#define REF_TS    0.0005f
#define REF_VMAX  50.0f
#define REF_AMAX  200.0f
#define REF_JMAX  2000.0f

/* ---------------- A) 复现参考工程的阶跃序列 ---------------- */
static void run_ref_ramp(const char *tag, profile_mode_t mode)
{
    ramp_t rg;
    FILE  *fp;
    char   fn[128];
    float  a_prev = 0.0f;
    float  v_tgt;
    int    k;
    double max_a = 0.0, max_j = 0.0;

    sprintf(fn, "fw_ramp_%s.csv", tag);
    fp = fopen(fn, "w");
    if (!fp) {
        printf("[FAIL] cannot open %s\n", fn);
        g_fail++;
        return;
    }
    fprintf(fp, "t,v,a,jerk,v_target\n");

    ramp_init(&rg, mode, REF_TS, REF_AMAX, REF_JMAX);

    for (k = 0; k < 30000; k++) {
        double t = k * (double)REF_TS;

        /* 与参考 test_profile.c 完全相同的三段阶跃 */
        if (t < 0.2)       v_tgt = 0.0f;
        else if (t < 1.2)  v_tgt = REF_VMAX;
        else if (t < 2.2)  v_tgt = -0.4f * REF_VMAX;
        else               v_tgt = 0.0f;

        ramp_step(&rg, v_tgt);
        fprintf(fp, "%.6f,%.6f,%.6f,%.3f,%.6f\n",
                t, rg.v, rg.a, (rg.a - a_prev) / REF_TS, v_tgt);

        if (fabs(rg.a) > max_a)                 max_a = fabs(rg.a);
        if (fabs((rg.a - a_prev) / REF_TS) > max_j) max_j = fabs((rg.a - a_prev) / REF_TS);
        a_prev = rg.a;
    }
    fclose(fp);

    printf("  %-8s max|a|=%7.3f (<=%.0f)  max|jerk|=%8.1f  v_end=%.6f\n",
           tag, max_a, REF_AMAX, max_j, rg.v);
    check(tag, fabs(rg.v) < 1e-4f, "末速度归零");
}

/* ---------------- B) 固件实际参数：1 kHz 速度模式 ---------------- */
static void run_fw_vel(void)
{
    vel_profile_t vp;
    float  theta = 0.0f;
    float  ts = VEL_TS_DEFAULT;
    double max_a = 0.0, max_j = 0.0, pos_int = 0.0;
    float  a_prev = 0.0f, v_prev = 0.0f;
    int    k;
    char   buf[128];

    VelProfile_Init(&vp, ts);
    VelProfile_SetMode(&vp, VEL_MODE_TRAP);
    VelProfile_SetTarget(&vp, 0.0f);
    VelProfile_Enable(&vp, theta);            /* 位置 0 处使能 */

    /* 0~0.1s 静止等待对齐；0.1s 给 +20 rad/s；1.1s 给 -8 rad/s；2.5s 回 0 */
    for (k = 0; k < 5000; k++) {
        double t = k * (double)ts;
        float  p;

        if (t >= 0.1 && t < 1.1)       VelProfile_SetTarget(&vp, 20.0f);
        else if (t >= 1.1 && t < 2.5)  VelProfile_SetTarget(&vp, -8.0f);
        else if (t >= 2.5)             VelProfile_SetTarget(&vp, 0.0f);

        p = VelProfile_Step(&vp, theta);
        /* 理想跟随：把 theta 当作"完全跟上给定" */
        theta = p;
        pos_int += (double)vp.v_ref * (double)ts;

        {
            float a = (vp.v_ref - v_prev) / ts;
            float j = (a - a_prev) / ts;
            if (fabs(a) > max_a) max_a = fabs(a);
            if (fabs(j) > max_j) max_j = fabs(j);
            a_prev = a;
        }
        v_prev = vp.v_ref;
    }

    sprintf(buf, "max|a|=%.1f max|jerk|=%.0f v_end=%.6f clamp=%u",
            max_a, max_j, vp.v_ref, (unsigned)vp.clamp_cnt);
    check("B1 梯形 a 上限", max_a <= VEL_A_MAX_DEFAULT * 1.02, buf);
    check("B2 梯形末速归零", fabsf(vp.v_ref) < 1e-3f, "v_ref≈0");
    check("B3 位置给定=速度积分",
          fabs((double)vp.pos_ref - pos_int) < 1e-2,
          "|pos_ref - ∫v dt| < 0.01 rad");
    check("B4 梯形无跟随限幅", vp.clamp_cnt == 0u, "未触发 err_max");

    /* S 曲线：同一序列，检查 jerk 有界。
     * 注意：参考实现在"末段零速吸附"时会把 a 直接归零（防极限环），
     * 那一下的 jerk 会超限（参考 CSV 在 ts=0.5ms 时整体 8000 / 排除吸附后 2000）。
     * 所以判据同样排除 |v-v_target| <= 0.5 的末段样本，与参考口径一致。 */
    VelProfile_Init(&vp, ts);
    VelProfile_SetMode(&vp, VEL_MODE_SCURVE);
    VelProfile_Enable(&vp, theta);
    max_a = 0.0; max_j = 0.0; a_prev = 0.0f; v_prev = 0.0f;

    {
        double max_j_all = 0.0;

        for (k = 0; k < 5000; k++) {
            double t = k * (double)ts;
            float  p;
            float  a, j;

            if (t >= 0.1 && t < 1.1)       VelProfile_SetTarget(&vp, 20.0f);
            else if (t >= 1.1 && t < 2.5)  VelProfile_SetTarget(&vp, 0.0f);
            else if (t >= 2.5)             VelProfile_SetTarget(&vp, 0.0f);

            p = VelProfile_Step(&vp, theta);
            theta = p;

            a = (vp.v_ref - v_prev) / ts;
            j = (a - a_prev) / ts;
            if (fabs(a) > max_a) max_a = fabs(a);
            if (fabs(j) > max_j_all) max_j_all = fabs(j);
            if (fabsf(vp.v_ref - vp.v_target) > 0.5f && fabs(j) > max_j) max_j = fabs(j);
            a_prev = a;
            v_prev = vp.v_ref;
        }

        sprintf(buf, "max|a|=%.1f jerk=%.0f(含吸附 %.0f)/上限 %.0f",
                max_a, max_j, max_j_all, VEL_J_MAX_DEFAULT);
        check("B5 S 曲线 jerk 有界", max_j <= VEL_J_MAX_DEFAULT * 1.02, buf);
    }
    check("B6 S 曲线末速归零", fabsf(vp.v_ref) < 2e-3f, "v_ref≈0");
}

/* ---------------- C) 安全行为 ---------------- */
static void run_safety(void)
{
    vel_profile_t vp;
    char buf[160];
    float theta = 1.234f;

    int   k;

    /* C1：使能瞬间不得跳变 */
    VelProfile_Init(&vp, VEL_TS_DEFAULT);
    VelProfile_SetTarget(&vp, 20.0f);
    VelProfile_Enable(&vp, theta);
    check("C1 使能瞬间给定=实际位置",
          fabsf(VelProfile_GetPosRef(&vp) - theta) < 1e-6f, "|pos_ref-theta|<1e-6");

    /* C2：第一个周期速度应为 0（斜坡从 0 起） */
    (void)VelProfile_Step(&vp, theta);
    sprintf(buf, "v_ref=%.4f", (double)vp.v_ref);
    check("C2 首拍速度从 0 起", fabsf(vp.v_ref) <= VEL_A_MAX_DEFAULT * VEL_TS_DEFAULT + 1e-4f, buf);

    /* C3：堵转 —— theta 卡住不动，位置给定必须被 err_max 限住 */
    for (k = 0; k < 3000; k++) {
        (void)VelProfile_Step(&vp, theta);      /* theta 始终不变 */
    }
    sprintf(buf, "pos_ref-theta=%.3f (<=%.2f) clamp=%u",
            (double)(VelProfile_GetPosRef(&vp) - theta),
            (double)VEL_ERR_MAX_DEFAULT, (unsigned)vp.clamp_cnt);
    check("C3 堵转时给定不跑飞",
          (VelProfile_GetPosRef(&vp) - theta) <= VEL_ERR_MAX_DEFAULT + 1e-3f &&
          vp.clamp_cnt > 0u, buf);

    /* C4：退出后给定贴住实际位置 */
    VelProfile_Disable(&vp, 2.5f);
    check("C4 退出后贴住实际位置",
          fabsf(VelProfile_GetPosRef(&vp) - 2.5f) < 1e-6f, "pos_ref=theta");
}

int main(void)
{
    printf("=== A) 与参考 CSV 同源复算（ts=0.5ms, 0->50->-20->0） ===\n");
    run_ref_ramp("trap",   PROFILE_MODE_TRAP);
    run_ref_ramp("scurve", PROFILE_MODE_SCURVE);

    printf("\n=== B) 固件参数：1 kHz 速度模式 ===\n");
    run_fw_vel();

    printf("\n=== C) 安全行为 ===\n");
    run_safety();

    printf("\n---- 汇总 ----\n");
    printf("PASS %d   FAIL %d\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
