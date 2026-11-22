/*
 * File: three_loop_deploy.c
 *
 * Code generated for Simulink model 'three_loop_deploy'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Sat Nov 14 17:42:46 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "three_loop_deploy.h"
#include <math.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_three_loop_deploy_T three_loop_deploy_DW;

/* External inputs (root inport signals with default storage) */
ExtU_three_loop_deploy_T three_loop_deploy_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_three_loop_deploy_T three_loop_deploy_Y;

/* Model step function */
void three_loop_deploy_step(void)
{
  real_T err;
  real_T err_q;
  real_T i_beta;
  real_T id;
  real_T rtb_gain_p_w;
  real_T rtb_theta_sum;
  int32_T tmp;

  /* Outputs for Atomic SubSystem: '<Root>/ThreeLoop_Controller' */
  /* Sum: '<S1>/theta_sum' incorporates:
   *  Constant: '<S1>/theta_off'
   *  Gain: '<S1>/gain_comp'
   *  Gain: '<S1>/gain_p_theta'
   *  Inport: '<Root>/omega_m'
   *  Inport: '<Root>/theta_m'
   */
  rtb_theta_sum = (14.0 * three_loop_deploy_U.theta_m + 1.57079632679) + 0.0021
    * three_loop_deploy_U.omega_m;

  /* Gain: '<S1>/gain_p_w' incorporates:
   *  Inport: '<Root>/omega_m'
   */
  rtb_gain_p_w = 14.0 * three_loop_deploy_U.omega_m;

  /* MATLAB Function: '<S1>/Position_Loop' incorporates:
   *  Constant: '<S1>/par_p'
   *  Constant: '<S1>/par_w'
   *  Inport: '<Root>/en'
   *  Inport: '<Root>/omega_m'
   *  Inport: '<Root>/pos_ref'
   *  Inport: '<Root>/theta_m'
   *  MATLAB Function: '<S1>/Speed_Loop'
   */
  if (three_loop_deploy_U.en < 0.5) {
    three_loop_deploy_DW.hold_w = 0.0;
    three_loop_deploy_DW.i_term = 0.0;
    three_loop_deploy_DW.hold_iq = 0.0;
  } else {
    /* ⚠ 分频：位置环设计 1 kHz = 20 kHz / 20。实际电流环只有 10 kHz，
     *   所以这里必须是 10 才能维持 1 kHz。与下方电流环 dt 是同一个根因。 */
    three_loop_deploy_DW.cnt_i++;
    if (three_loop_deploy_DW.cnt_i >= 10.0) {
      three_loop_deploy_DW.cnt_i = 0.0;
      three_loop_deploy_DW.n_exec_n++;
      three_loop_deploy_DW.hold_w = (three_loop_deploy_U.pos_ref -
        three_loop_deploy_U.theta_m) * 50.2654824574;
      if (three_loop_deploy_DW.hold_w > 120.0) {
        three_loop_deploy_DW.hold_w = 120.0;
      } else if (three_loop_deploy_DW.hold_w < -120.0) {
        three_loop_deploy_DW.hold_w = -120.0;
      }
    }

    /* ⚠ 分频：速度环设计 2 kHz = 20 kHz / 10。实际 10 kHz 下必须是 5，
     *   否则速度环掉到 1 kHz，ki_w=10.19 的积分等效只有设计值一半
     *   （i_term 用的是固定 Ts_w=500 us，见下面 0.0005）。 */
    three_loop_deploy_DW.cnt++;
    if (three_loop_deploy_DW.cnt >= 5.0) {
      three_loop_deploy_DW.cnt = 0.0;
      three_loop_deploy_DW.n_exec++;
      err = three_loop_deploy_DW.hold_w - three_loop_deploy_U.omega_m;
      three_loop_deploy_DW.i_term += 10.1906891515 * err * 0.0005;
      if (three_loop_deploy_DW.i_term > 9.0) {
        three_loop_deploy_DW.i_term = 9.0;
      } else if (three_loop_deploy_DW.i_term < -9.0) {
        three_loop_deploy_DW.i_term = -9.0;
      }

      three_loop_deploy_DW.hold_iq = 0.0729227202682 * err +
        three_loop_deploy_DW.i_term;
      if (three_loop_deploy_DW.hold_iq > 9.0) {
        three_loop_deploy_DW.hold_iq = 9.0;
      } else if (three_loop_deploy_DW.hold_iq < -9.0) {
        three_loop_deploy_DW.hold_iq = -9.0;
      }
    }
  }

  /* MATLAB Function: '<S1>/Current_Loop' incorporates:
   *  Constant: '<S1>/par_c'
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  MATLAB Function: '<S1>/Speed_Loop'
   */
  three_loop_deploy_DW.n_exec_b++;
  i_beta = 0.57735026918962584 * three_loop_deploy_U.ia + 1.1547005383792515 *
    three_loop_deploy_U.ib;
  err = sin(rtb_theta_sum);
  rtb_theta_sum = cos(rtb_theta_sum);
  id = rtb_theta_sum * three_loop_deploy_U.ia + err * i_beta;
  i_beta = rtb_theta_sum * i_beta - err * three_loop_deploy_U.ia;
  three_loop_deploy_Y.iq_used = three_loop_deploy_DW.hold_iq;
  if (three_loop_deploy_DW.hold_iq > 9.0) {
    three_loop_deploy_Y.iq_used = 9.0;
  } else if (three_loop_deploy_DW.hold_iq < -9.0) {
    three_loop_deploy_Y.iq_used = -9.0;
  }

  /* Outport: '<Root>/duty' incorporates:
   *  MATLAB Function: '<S1>/Current_Loop'
   */
  three_loop_deploy_Y.duty[0] = 0.5;
  three_loop_deploy_Y.duty[1] = 0.5;
  three_loop_deploy_Y.duty[2] = 0.5;

  /* MATLAB Function: '<S1>/Current_Loop' incorporates:
   *  Constant: '<S1>/par_c'
   *  Inport: '<Root>/en'
   *  Inport: '<Root>/vbus'
   */
  if (three_loop_deploy_U.en < 0.5) {
    three_loop_deploy_DW.vd_int = 0.0;
    three_loop_deploy_DW.vq_int = 0.0;
  } else {
    err_q = three_loop_deploy_Y.iq_used - i_beta;
    /* ⚠⚠ dt 必须等于**实际控制周期**，不是设计周期。
     *
     * 原值 5.0E-5 是按 20 kHz 设计的；但本板控制环实际跑 10 kHz
     * （模型 double 导致单拍 8862 cyc > 50 us，ADC2 每两次触发丢一次，
     *   见 Agnet/控制节拍与预算诊断.md §3）。两者不一致的后果：
     *   Ki_effective = 2378.19 × 5e-5 × 10000 = 1189，只有设计值的**一半**，
     *   积分零点从 3334 rad/s 掉到 1667 rad/s，稳态收敛变慢、抗扰变差。
     *
     * 这里改成 1.0E-4 让 Ki 回到设计值。副作用：相位裕度比 20 kHz 时低
     * （采样延迟 150 us vs 75 us），若实测有超调再降 kp。
     *
     * 正式做法改 .slx 的 par_c 常量第 3 项（build_three_loop_deploy.m:119），
     * 本文件是 Simulink 生成物，重新生成会覆盖这里。
     * 回到 20 kHz 时必须把这两处改回 5.0E-5。 */
    three_loop_deploy_DW.vd_int += (0.0 - id) * 2378.18563877 * 1.0E-4;
    three_loop_deploy_DW.vq_int += err_q * 2378.18563877 * 1.0E-4;
    i_beta = ((0.0 - id) * 0.713141532365 + three_loop_deploy_DW.vd_int) -
      i_beta * rtb_gain_p_w * 0.0001135;
    rtb_gain_p_w = ((err_q * 0.713141532365 + three_loop_deploy_DW.vq_int) + id *
                    rtb_gain_p_w * 0.0001135) + rtb_gain_p_w * 0.00759297995988;
    id = 0.54848275573014449 * three_loop_deploy_U.vbus;
    err_q = id * 0.98;
    if (i_beta > err_q) {
      i_beta = err_q;
    } else if (i_beta < -err_q) {
      i_beta = -err_q;
    }

    if (three_loop_deploy_DW.vd_int > err_q) {
      three_loop_deploy_DW.vd_int = err_q;
    } else if (three_loop_deploy_DW.vd_int < -err_q) {
      three_loop_deploy_DW.vd_int = -err_q;
    }

    id = id * id - i_beta * i_beta;
    if (id > 0.0) {
      id = sqrt(id);
    } else {
      id = 0.0;
    }

    if (rtb_gain_p_w > id) {
      rtb_gain_p_w = id;
    } else if (rtb_gain_p_w < -id) {
      rtb_gain_p_w = -id;
    }

    if (three_loop_deploy_DW.vq_int > id) {
      three_loop_deploy_DW.vq_int = id;
    } else if (three_loop_deploy_DW.vq_int < -id) {
      three_loop_deploy_DW.vq_int = -id;
    }

    err_q = 1.5 / three_loop_deploy_U.vbus;
    id = i_beta * err_q;
    rtb_gain_p_w *= err_q;
    i_beta = rtb_theta_sum * id - err * rtb_gain_p_w;
    err = rtb_theta_sum * rtb_gain_p_w + err * id;
    if (err >= 0.0) {
      if (i_beta >= 0.0) {
        if (0.57735026918962584 * err > i_beta) {
          tmp = 2;
        } else {
          tmp = 1;
        }
      } else if (-0.57735026918962584 * err > i_beta) {
        tmp = 3;
      } else {
        tmp = 2;
      }
    } else if (i_beta >= 0.0) {
      if (-0.57735026918962584 * err > i_beta) {
        tmp = 5;
      } else {
        tmp = 6;
      }
    } else if (0.57735026918962584 * err > i_beta) {
      tmp = 4;
    } else {
      tmp = 5;
    }

    switch (tmp) {
     case 1:
      i_beta -= 0.57735026918962584 * err;
      rtb_theta_sum = 1.1547005383792515 * err;
      err = ((i_beta + 1.0) + rtb_theta_sum) * 0.5;
      rtb_gain_p_w = err - i_beta;
      i_beta = rtb_gain_p_w - rtb_theta_sum;
      break;

     case 2:
      rtb_theta_sum = 0.57735026918962584 * err + i_beta;
      id = 0.57735026918962584 * err - i_beta;
      rtb_gain_p_w = ((rtb_theta_sum + 1.0) + id) * 0.5;
      err = rtb_gain_p_w - id;
      i_beta = err - rtb_theta_sum;
      break;

     case 3:
      id = 1.1547005383792515 * err;
      rtb_theta_sum = -i_beta - 0.57735026918962584 * err;
      rtb_gain_p_w = ((id + 1.0) + rtb_theta_sum) * 0.5;
      i_beta = rtb_gain_p_w - id;
      err = i_beta - rtb_theta_sum;
      break;

     case 4:
      rtb_theta_sum = 0.57735026918962584 * err - i_beta;
      rtb_gain_p_w = -1.1547005383792515 * err;
      i_beta = ((rtb_theta_sum + 1.0) + rtb_gain_p_w) * 0.5;
      rtb_gain_p_w = i_beta - rtb_gain_p_w;
      err = rtb_gain_p_w - rtb_theta_sum;
      break;

     case 5:
      rtb_gain_p_w = -i_beta - 0.57735026918962584 * err;
      rtb_theta_sum = i_beta - 0.57735026918962584 * err;
      i_beta = ((rtb_gain_p_w + 1.0) + rtb_theta_sum) * 0.5;
      err = i_beta - rtb_gain_p_w;
      rtb_gain_p_w = err - rtb_theta_sum;
      break;

     default:
      rtb_theta_sum = -1.1547005383792515 * err;
      i_beta += 0.57735026918962584 * err;
      err = ((i_beta + 1.0) + rtb_theta_sum) * 0.5;
      i_beta = err - i_beta;
      rtb_gain_p_w = i_beta - rtb_theta_sum;
      break;
    }

    if (err > 0.975) {
      err = 0.975;
    } else if (err < 0.0) {
      err = 0.0;
    }

    if (rtb_gain_p_w > 0.975) {
      rtb_gain_p_w = 0.975;
    } else if (rtb_gain_p_w < 0.0) {
      rtb_gain_p_w = 0.0;
    }

    if (i_beta > 0.975) {
      i_beta = 0.975;
    } else if (i_beta < 0.0) {
      i_beta = 0.0;
    }

    /* Outport: '<Root>/duty' incorporates:
     *  Constant: '<S1>/par_c'
     *  Inport: '<Root>/vbus'
     */
    three_loop_deploy_Y.duty[0] = err;
    three_loop_deploy_Y.duty[1] = rtb_gain_p_w;
    three_loop_deploy_Y.duty[2] = i_beta;
  }

  /* Outport: '<Root>/ticks' incorporates:
   *  MATLAB Function: '<S1>/Current_Loop'
   *  MATLAB Function: '<S1>/Position_Loop'
   *  MATLAB Function: '<S1>/Speed_Loop'
   */
  three_loop_deploy_Y.ticks[0] = three_loop_deploy_DW.n_exec_b;
  three_loop_deploy_Y.ticks[1] = three_loop_deploy_DW.n_exec;
  three_loop_deploy_Y.ticks[2] = three_loop_deploy_DW.n_exec_n;

  /* Outport: '<Root>/iq_ref' incorporates:
   *  MATLAB Function: '<S1>/Speed_Loop'
   */
  three_loop_deploy_Y.iq_ref = three_loop_deploy_DW.hold_iq;

  /* Outport: '<Root>/w_ref' incorporates:
   *  MATLAB Function: '<S1>/Position_Loop'
   */
  three_loop_deploy_Y.w_ref = three_loop_deploy_DW.hold_w;

  /* End of Outputs for SubSystem: '<Root>/ThreeLoop_Controller' */
}

/* Model initialize function */
void three_loop_deploy_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
