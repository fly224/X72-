/*
 * File: three_loop_deploy.c
 *
 * Code generated for Simulink model 'three_loop_deploy'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Sun Nov 15 18:37:03 2026
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
  int32_T tmp;
  real32_T dec_bemf;
  real32_T dec_vd;
  real32_T dec_vq;
  real32_T err;
  real32_T i_beta;
  real32_T id;
  real32_T iq;
  real32_T rtb_gain_p_w;
  real32_T rtb_theta_sum;

  /* Outputs for Atomic SubSystem: '<Root>/ThreeLoop_Controller' */
  /* Sum: '<S1>/theta_sum' incorporates:
   *  Constant: '<S1>/theta_off'
   *  Gain: '<S1>/gain_comp'
   *  Gain: '<S1>/gain_p_theta'
   *  Inport: '<Root>/omega_m'
   *  Inport: '<Root>/theta_m'
   */
  rtb_theta_sum = (14.0F * three_loop_deploy_U.theta_m + 1.57079637F) + 0.00105F
    * three_loop_deploy_U.omega_m;

  /* Gain: '<S1>/gain_p_w' incorporates:
   *  Inport: '<Root>/omega_m'
   */
  rtb_gain_p_w = 14.0F * three_loop_deploy_U.omega_m;

  /* MATLAB Function: '<S1>/Position_Loop' incorporates:
   *  Inport: '<Root>/en'
   *  Inport: '<Root>/omega_m'
   *  Inport: '<Root>/par_p'
   *  Inport: '<Root>/par_w'
   *  Inport: '<Root>/pos_ref'
   *  Inport: '<Root>/theta_m'
   *  MATLAB Function: '<S1>/Speed_Loop'
   */
  if (three_loop_deploy_U.en < 0.5F) {
    three_loop_deploy_DW.hold_w = 0.0F;
    three_loop_deploy_DW.i_term = 0.0F;
    three_loop_deploy_DW.hold_iq = 0.0F;
  } else {
    three_loop_deploy_DW.cnt_b++;
    if (three_loop_deploy_DW.cnt_b >= three_loop_deploy_U.par_p[3]) {
      three_loop_deploy_DW.cnt_b = 0.0F;
      three_loop_deploy_DW.n_exec_p++;
      three_loop_deploy_DW.hold_w = (three_loop_deploy_U.pos_ref -
        three_loop_deploy_U.theta_m) * three_loop_deploy_U.par_p[0];
      if (three_loop_deploy_DW.hold_w > three_loop_deploy_U.par_p[1]) {
        three_loop_deploy_DW.hold_w = three_loop_deploy_U.par_p[1];
      } else if (three_loop_deploy_DW.hold_w < -three_loop_deploy_U.par_p[1]) {
        three_loop_deploy_DW.hold_w = -three_loop_deploy_U.par_p[1];
      }
    }

    three_loop_deploy_DW.cnt++;
    if (three_loop_deploy_DW.cnt >= three_loop_deploy_U.par_w[4]) {
      three_loop_deploy_DW.cnt = 0.0F;
      three_loop_deploy_DW.n_exec++;
      err = three_loop_deploy_DW.hold_w - three_loop_deploy_U.omega_m;
      three_loop_deploy_DW.i_term += three_loop_deploy_U.par_w[1] * err *
        three_loop_deploy_U.par_w[2];
      if (three_loop_deploy_DW.i_term > three_loop_deploy_U.par_w[3]) {
        three_loop_deploy_DW.i_term = three_loop_deploy_U.par_w[3];
      } else if (three_loop_deploy_DW.i_term < -three_loop_deploy_U.par_w[3]) {
        three_loop_deploy_DW.i_term = -three_loop_deploy_U.par_w[3];
      }

      three_loop_deploy_DW.hold_iq = three_loop_deploy_U.par_w[0] * err +
        three_loop_deploy_DW.i_term;
      if (three_loop_deploy_DW.hold_iq > three_loop_deploy_U.par_w[3]) {
        three_loop_deploy_DW.hold_iq = three_loop_deploy_U.par_w[3];
      } else if (three_loop_deploy_DW.hold_iq < -three_loop_deploy_U.par_w[3]) {
        three_loop_deploy_DW.hold_iq = -three_loop_deploy_U.par_w[3];
      }
    }
  }

  /* MATLAB Function: '<S1>/Current_Loop' incorporates:
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/par_c'
   *  MATLAB Function: '<S1>/Speed_Loop'
   */
  three_loop_deploy_DW.n_exec_k++;
  i_beta = 0.577350259F * three_loop_deploy_U.ia + 1.15470052F *
    three_loop_deploy_U.ib;
  err = sinf(rtb_theta_sum);
  rtb_theta_sum = cosf(rtb_theta_sum);
  id = rtb_theta_sum * three_loop_deploy_U.ia + err * i_beta;
  iq = rtb_theta_sum * i_beta - err * three_loop_deploy_U.ia;
  three_loop_deploy_Y.iq_used = three_loop_deploy_DW.hold_iq;
  if (three_loop_deploy_DW.hold_iq > three_loop_deploy_U.par_c[10]) {
    three_loop_deploy_Y.iq_used = three_loop_deploy_U.par_c[10];
  } else if (three_loop_deploy_DW.hold_iq < -three_loop_deploy_U.par_c[10]) {
    three_loop_deploy_Y.iq_used = -three_loop_deploy_U.par_c[10];
  }

  /* Outport: '<Root>/duty' incorporates:
   *  MATLAB Function: '<S1>/Current_Loop'
   */
  three_loop_deploy_Y.duty[0] = 0.5F;
  three_loop_deploy_Y.duty[1] = 0.5F;
  three_loop_deploy_Y.duty[2] = 0.5F;

  /* MATLAB Function: '<S1>/Current_Loop' incorporates:
   *  Inport: '<Root>/en'
   *  Inport: '<Root>/par_c'
   *  Inport: '<Root>/vbus'
   */
  if (three_loop_deploy_U.en < 0.5F) {
    three_loop_deploy_DW.vd_int = 0.0F;
    three_loop_deploy_DW.vq_int = 0.0F;
  } else {
    i_beta = three_loop_deploy_Y.iq_used - iq;
    three_loop_deploy_DW.vd_int += (0.0F - id) * three_loop_deploy_U.par_c[1] *
      three_loop_deploy_U.par_c[2];
    three_loop_deploy_DW.vq_int += i_beta * three_loop_deploy_U.par_c[1] *
      three_loop_deploy_U.par_c[2];
    dec_vd = 0.0F;
    dec_vq = 0.0F;
    dec_bemf = 0.0F;
    if ((three_loop_deploy_U.par_c[9] == 1.0F) || (three_loop_deploy_U.par_c[9] ==
         3.0F)) {
      dec_vd = iq * rtb_gain_p_w * three_loop_deploy_U.par_c[4];
      dec_vq = id * rtb_gain_p_w * three_loop_deploy_U.par_c[3];
    }

    if (three_loop_deploy_U.par_c[9] == 2.0F) {
      dec_bemf = rtb_gain_p_w * three_loop_deploy_U.par_c[5];
    } else if (three_loop_deploy_U.par_c[9] == 3.0F) {
      dec_bemf = rtb_gain_p_w * three_loop_deploy_U.par_c[5];
    }

    rtb_gain_p_w = ((0.0F - id) * three_loop_deploy_U.par_c[0] +
                    three_loop_deploy_DW.vd_int) - dec_vd;
    id = ((i_beta * three_loop_deploy_U.par_c[0] + three_loop_deploy_DW.vq_int)
          + dec_vq) + dec_bemf;
    i_beta = 0.577350259F * three_loop_deploy_U.par_c[6] *
      three_loop_deploy_U.vbus * three_loop_deploy_U.par_c[7];
    dec_vq = i_beta * three_loop_deploy_U.par_c[8];
    if (rtb_gain_p_w > dec_vq) {
      rtb_gain_p_w = dec_vq;
    } else if (rtb_gain_p_w < -dec_vq) {
      rtb_gain_p_w = -dec_vq;
    }

    if (three_loop_deploy_DW.vd_int > dec_vq) {
      three_loop_deploy_DW.vd_int = dec_vq;
    } else if (three_loop_deploy_DW.vd_int < -dec_vq) {
      three_loop_deploy_DW.vd_int = -dec_vq;
    }

    i_beta = i_beta * i_beta - rtb_gain_p_w * rtb_gain_p_w;
    if (i_beta > 0.0F) {
      i_beta = sqrtf(i_beta);
    } else {
      i_beta = 0.0F;
    }

    if (id > i_beta) {
      id = i_beta;
    } else if (id < -i_beta) {
      id = -i_beta;
    }

    if (three_loop_deploy_DW.vq_int > i_beta) {
      three_loop_deploy_DW.vq_int = i_beta;
    } else if (three_loop_deploy_DW.vq_int < -i_beta) {
      three_loop_deploy_DW.vq_int = -i_beta;
    }

    dec_vq = 1.5F / three_loop_deploy_U.vbus;
    i_beta = rtb_gain_p_w * dec_vq;
    id *= dec_vq;
    rtb_gain_p_w = rtb_theta_sum * i_beta - err * id;
    err = rtb_theta_sum * id + err * i_beta;
    if (err >= 0.0F) {
      if (rtb_gain_p_w >= 0.0F) {
        if (0.577350259F * err > rtb_gain_p_w) {
          tmp = 2;
        } else {
          tmp = 1;
        }
      } else if (-0.577350259F * err > rtb_gain_p_w) {
        tmp = 3;
      } else {
        tmp = 2;
      }
    } else if (rtb_gain_p_w >= 0.0F) {
      if (-0.577350259F * err > rtb_gain_p_w) {
        tmp = 5;
      } else {
        tmp = 6;
      }
    } else if (0.577350259F * err > rtb_gain_p_w) {
      tmp = 4;
    } else {
      tmp = 5;
    }

    switch (tmp) {
     case 1:
      rtb_gain_p_w -= 0.577350259F * err;
      id = 1.15470052F * err;
      err = ((rtb_gain_p_w + 1.0F) + id) * 0.5F;
      rtb_theta_sum = err - rtb_gain_p_w;
      rtb_gain_p_w = rtb_theta_sum - id;
      break;

     case 2:
      id = 0.577350259F * err + rtb_gain_p_w;
      i_beta = 0.577350259F * err - rtb_gain_p_w;
      rtb_theta_sum = ((id + 1.0F) + i_beta) * 0.5F;
      err = rtb_theta_sum - i_beta;
      rtb_gain_p_w = err - id;
      break;

     case 3:
      i_beta = 1.15470052F * err;
      id = -rtb_gain_p_w - 0.577350259F * err;
      rtb_theta_sum = ((i_beta + 1.0F) + id) * 0.5F;
      rtb_gain_p_w = rtb_theta_sum - i_beta;
      err = rtb_gain_p_w - id;
      break;

     case 4:
      id = 0.577350259F * err - rtb_gain_p_w;
      i_beta = -1.15470052F * err;
      rtb_gain_p_w = ((id + 1.0F) + i_beta) * 0.5F;
      rtb_theta_sum = rtb_gain_p_w - i_beta;
      err = rtb_theta_sum - id;
      break;

     case 5:
      i_beta = -rtb_gain_p_w - 0.577350259F * err;
      rtb_theta_sum = rtb_gain_p_w - 0.577350259F * err;
      rtb_gain_p_w = ((i_beta + 1.0F) + rtb_theta_sum) * 0.5F;
      err = rtb_gain_p_w - i_beta;
      rtb_theta_sum = err - rtb_theta_sum;
      break;

     default:
      rtb_theta_sum = -1.15470052F * err;
      rtb_gain_p_w += 0.577350259F * err;
      err = ((rtb_gain_p_w + 1.0F) + rtb_theta_sum) * 0.5F;
      rtb_gain_p_w = err - rtb_gain_p_w;
      rtb_theta_sum = rtb_gain_p_w - rtb_theta_sum;
      break;
    }

    id = (three_loop_deploy_U.par_c[6] + 1.0F) * 0.5F;
    if (err > id) {
      err = id;
    } else if (err < 0.0F) {
      err = 0.0F;
    }

    if (rtb_theta_sum > id) {
      rtb_theta_sum = id;
    } else if (rtb_theta_sum < 0.0F) {
      rtb_theta_sum = 0.0F;
    }

    if (rtb_gain_p_w > id) {
      rtb_gain_p_w = id;
    } else if (rtb_gain_p_w < 0.0F) {
      rtb_gain_p_w = 0.0F;
    }

    /* Outport: '<Root>/duty' incorporates:
     *  Inport: '<Root>/par_c'
     *  Inport: '<Root>/vbus'
     */
    three_loop_deploy_Y.duty[0] = err;
    three_loop_deploy_Y.duty[1] = rtb_theta_sum;
    three_loop_deploy_Y.duty[2] = rtb_gain_p_w;
  }

  /* Outport: '<Root>/ticks' incorporates:
   *  MATLAB Function: '<S1>/Current_Loop'
   *  MATLAB Function: '<S1>/Position_Loop'
   *  MATLAB Function: '<S1>/Speed_Loop'
   */
  three_loop_deploy_Y.ticks[0] = three_loop_deploy_DW.n_exec_k;
  three_loop_deploy_Y.ticks[1] = three_loop_deploy_DW.n_exec;
  three_loop_deploy_Y.ticks[2] = three_loop_deploy_DW.n_exec_p;

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
