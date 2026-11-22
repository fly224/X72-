/*
 * tuning_test.c —— PID 参数对外接口 / 位置 wrap 的 host 自检（不依赖 SIL CSV）
 *
 * gcc -std=c99 -O2 -Wall -I. tuning_test.c three_loop_deploy.c three_loop_tuning.c \
 *     -o tuning_test.exe -lm
 *
 * 验证：
 *   1) 初始化后 U.par_* 的固定项与默认值正确（20 kHz 的 dt/N_w/N_p）；
 *   2) ParamGet/ParamSet 定点换算、范围检查、只读项、使能态拒写；
 *   3) 改 kp_i 后确实写进 U.par_c[0]（参数不是固化常量）；
 *   4) 位置 wrap 跨 2π 不跳变（文档 §3.3 的过流根因）。
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "three_loop_tuning.h"

static int g_fail = 0;

static void ck(const char *name, int cond, const char *detail)
{
    printf("[%s] %-46s %s\n", cond ? "PASS" : "FAIL", name, detail);
    if (!cond) {
        g_fail++;
    }
}

int main(void)
{
    ThreeLoop_Tuning_T t;
    int32_t raw = 0;
    real_T  wrapped, err;
    char    buf[128];

    three_loop_deploy_initialize();
    ThreeLoop_Tuning_Init();

    /* ---- 1) 固定项：全部由 Ts_i=50us 派生 ---- */
    snprintf(buf, sizeof(buf), "dt=%.6g Ts_w=%.6g Ts_p=%.6g",
             three_loop_deploy_U.par_c[2], three_loop_deploy_U.par_w[2],
             three_loop_deploy_U.par_p[2]);
    ck("电流/速度/位置环周期 = 5e-5 / 5e-4 / 1e-3",
       fabs(three_loop_deploy_U.par_c[2] - 5.0e-5) < 1e-9 &&
       fabs(three_loop_deploy_U.par_w[2] - 5.0e-4) < 1e-9 &&
       fabs(three_loop_deploy_U.par_p[2] - 1.0e-3) < 1e-9, buf);

    snprintf(buf, sizeof(buf), "N_w=%.0f N_p=%.0f",
             three_loop_deploy_U.par_w[4], three_loop_deploy_U.par_p[3]);
    ck("速度环 /10 = 2kHz，位置环 /20 = 1kHz",
       three_loop_deploy_U.par_w[4] == 10.0 &&
       three_loop_deploy_U.par_p[3] == 20.0, buf);

    /* ---- 2) 默认值与定点回读 ---- */
    ThreeLoop_Tuning_Get(&t);
    snprintf(buf, sizeof(buf), "kp_i=%.9f ki_i=%.6f kp_w=%.9f ki_w=%.6f",
             t.kp_i, t.ki_i, t.kp_w, t.ki_w);
    ck("默认 PID 与 three_loop_params.m 一致（single 容差）",
       fabs(t.kp_i - 0.713141532365) < 1e-6 &&
       fabs(t.ki_i - 2378.18563877) < 1e-3  &&
       fabs(t.kp_w - 0.0729227202682) < 1e-6 &&
       fabs(t.ki_w - 10.1906891515) < 1e-4, buf);

    ck("ParamGet(CUR_KP) = 713 (x1000)",
       ThreeLoop_ParamGet(TL_PID_CUR_KP, &raw) == THREE_LOOP_OK && raw == 713,
       "raw=713");
    ck("ParamGet(只读 GAIN_COMP) = 1050 (x1e6)",
       ThreeLoop_ParamGet(TL_RO_GAIN_COMP, &raw) == THREE_LOOP_OK && raw == 1050,
       "0.00105 = 14*1.5*50us");
    ck("只读项写入被拒 (READ_ONLY)",
       ThreeLoop_ParamSet(TL_RO_N_W, 999, 0) == THREE_LOOP_READ_ONLY, "id=0x82");
    ck("使能态拒写 (WRITE_STATE_ERROR)",
       ThreeLoop_ParamSet(TL_PID_CUR_KP, 1426, 1) == THREE_LOOP_WRITE_STATE_ERROR,
       "is_enabled=1");
    ck("越界写入被拒 (OUT_OF_RANGE)",
       ThreeLoop_ParamSet(TL_PID_CUR_KP, 99999999, 0) == THREE_LOOP_OUT_OF_RANGE,
       "raw=99999999");
    ck("未知 ID 被拒 (UNKNOWN_ID)",
       ThreeLoop_ParamSet(0x7F, 1, 0) == THREE_LOOP_UNKNOWN_ID, "id=0x7F");

    /* ---- 3) 参数确实可调（不是固化常量） ---- */
    ck("ParamSet(CUR_KP,1426) 成功",
       ThreeLoop_ParamSet(TL_PID_CUR_KP, 1426, 0) == THREE_LOOP_OK, "kp_i 0.713 -> 1.426");
    snprintf(buf, sizeof(buf), "U.par_c[0]=%.3f", three_loop_deploy_U.par_c[0]);
    ck("新 Kp 已写入 U.par_c[0]",
       fabs(three_loop_deploy_U.par_c[0] - 1.426) < 1e-6, buf);
    ck("ParamGet 回读 = 1426",
       ThreeLoop_ParamGet(TL_PID_CUR_KP, &raw) == THREE_LOOP_OK && raw == 1426, "raw=1426");
    ck("固定项未被参数写入破坏",
       fabs(three_loop_deploy_U.par_c[2] - 5.0e-5) < 1e-9 &&
       three_loop_deploy_U.par_w[4] == 10.0 &&
       three_loop_deploy_U.par_p[3] == 20.0, "dt/N_w/N_p 不变");

    /* ---- 4) 位置坐标系折叠（文档 §3.3 过流根因） ---- */
    wrapped = ThreeLoop_WrapPosRef(6.283185307179586 + 0.15, 0.05);
    err = wrapped - 0.05;
    snprintf(buf, sizeof(buf), "pos_ref=%.6f  err=%.6f rad", wrapped, err);
    ck("跨 2pi 时位置误差不跳变 (期望 0.10 rad)",
       fabs(err - 0.10) < 1e-5, buf);

    printf("\n%s (%d 项失败)\n", g_fail ? "[FAIL] tuning interface" : "[PASS] tuning interface all", g_fail);
    return g_fail ? 1 : 0;
}
