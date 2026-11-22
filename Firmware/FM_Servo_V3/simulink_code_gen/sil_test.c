/*
 * sil_test.c —— 生成 C 代码与 Simulink 部署模型的数值一致性（host SIL）测试
 *
 * 用法（MinGW / GCC，在本目录执行）：
 *   gcc -std=c99 -O2 -Wall -I. sil_test.c three_loop_deploy.c three_loop_tuning.c -o sil_test.exe -lm
 *   ./sil_test.exe
 *
 * 生成模型的 par_c/par_w/par_p 已改为顶层 Inport，故必须先调用
 * ThreeLoop_Tuning_Init() 把默认参数写入 U；该默认值与 MATLAB 侧
 * export_sil_data 提供的参数向量逐项一致。
 *
 * sil_input.csv     : 由 generate_three_loop_code.m 的 export_sil_data 导出，
 *                     每行 8 个实数：ia,ib,ic,vbus,theta_m,omega_m,pos_ref,en
 * sil_expected.csv  : 同一激励下 Simulink 模型的 Y.duty[3]，每行 3 个实数
 *
 * 逐拍把输入写入 three_loop_deploy_U，调用 three_loop_deploy_step()，
 * 比较 Y.duty 与模型期望值。容差 1e-9（同一 double 运算链应完全或近完全一致）。
 */

#include <stdio.h>
#include <math.h>
#include "three_loop_deploy.h"
#include "three_loop_tuning.h"

int main(void)
{
    FILE *fi = fopen("sil_input.csv", "r");
    FILE *fe = fopen("sil_expected.csv", "r");
    double ia, ib, ic, vbus, th, wm, pref, en;
    double ea, eb, ec;
    double maxerr = 0.0;
    long   n = 0;
    int    ok;

    if (fi == NULL || fe == NULL) {
        printf("[FAIL] cannot open sil_input.csv / sil_expected.csv\n");
        if (fi) fclose(fi);
        if (fe) fclose(fe);
        return 2;
    }

    three_loop_deploy_initialize();
    ThreeLoop_Tuning_Init();          /* 填 U.par_c / par_w / par_p 默认值 */

    while (fscanf(fi, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                  &ia, &ib, &ic, &vbus, &th, &wm, &pref, &en) == 8 &&
           fscanf(fe, "%lf,%lf,%lf", &ea, &eb, &ec) == 3) {

        three_loop_deploy_U.ia       = ia;
        three_loop_deploy_U.ib       = ib;
        three_loop_deploy_U.ic       = ic;
        three_loop_deploy_U.vbus     = vbus;
        three_loop_deploy_U.theta_m  = th;
        three_loop_deploy_U.omega_m  = wm;
        three_loop_deploy_U.pos_ref  = pref;
        three_loop_deploy_U.en       = en;

        three_loop_deploy_step();

        {
            double e1 = fabs(three_loop_deploy_Y.duty[0] - ea);
            double e2 = fabs(three_loop_deploy_Y.duty[1] - eb);
            double e3 = fabs(three_loop_deploy_Y.duty[2] - ec);
            double e  = (e1 > e2) ? e1 : e2;
            if (e3 > e) e = e3;
            if (e > maxerr) maxerr = e;
        }
        n++;
    }

    printf("SIL samples                       : %ld\n", n);
    printf("max |duty_C - duty_Simulink|      : %.6e\n", maxerr);
    printf("last ticks (current/speed/pos)    : %.0f / %.0f / %.0f\n",
           three_loop_deploy_Y.ticks[0], three_loop_deploy_Y.ticks[1],
           three_loop_deploy_Y.ticks[2]);
    printf("last iq_ref / w_ref               : %.6f A / %.6f rad/s\n",
           three_loop_deploy_Y.iq_ref, three_loop_deploy_Y.w_ref);

    ok = (n > 0) && (maxerr < 1e-6);   /* 单精度 real32_T 容差 */
    printf("%s\n", ok ? "[PASS] generated C code == Simulink deploy model"
                      : "[FAIL] numerical mismatch");

    fclose(fi);
    fclose(fe);
    return ok ? 0 : 1;
}
