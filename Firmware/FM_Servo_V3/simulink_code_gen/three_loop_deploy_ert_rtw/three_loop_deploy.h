/*
 * File: three_loop_deploy.h
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

#ifndef RTW_HEADER_three_loop_deploy_h_
#define RTW_HEADER_three_loop_deploy_h_
#ifndef three_loop_deploy_COMMON_INCLUDES_
#define three_loop_deploy_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* three_loop_deploy_COMMON_INCLUDES_ */

#include "three_loop_deploy_types.h"

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T i_term;                     /* '<S1>/Speed_Loop' */
  real32_T cnt;                        /* '<S1>/Speed_Loop' */
  real32_T hold_iq;                    /* '<S1>/Speed_Loop' */
  real32_T n_exec;                     /* '<S1>/Speed_Loop' */
  real32_T cnt_b;                      /* '<S1>/Position_Loop' */
  real32_T hold_w;                     /* '<S1>/Position_Loop' */
  real32_T n_exec_p;                   /* '<S1>/Position_Loop' */
  real32_T vd_int;                     /* '<S1>/Current_Loop' */
  real32_T vq_int;                     /* '<S1>/Current_Loop' */
  real32_T n_exec_k;                   /* '<S1>/Current_Loop' */
} DW_three_loop_deploy_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ia;                         /* '<Root>/ia' */
  real32_T ib;                         /* '<Root>/ib' */
  real32_T ic;                         /* '<Root>/ic' */
  real32_T vbus;                       /* '<Root>/vbus' */
  real32_T theta_m;                    /* '<Root>/theta_m' */
  real32_T omega_m;                    /* '<Root>/omega_m' */
  real32_T pos_ref;                    /* '<Root>/pos_ref' */
  real32_T en;                         /* '<Root>/en' */
  real32_T par_c[11];                  /* '<Root>/par_c' */
  real32_T par_w[5];                   /* '<Root>/par_w' */
  real32_T par_p[4];                   /* '<Root>/par_p' */
} ExtU_three_loop_deploy_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T duty[3];                    /* '<Root>/duty' */
  real32_T iq_ref;                     /* '<Root>/iq_ref' */
  real32_T w_ref;                      /* '<Root>/w_ref' */
  real32_T iq_used;                    /* '<Root>/iq_used' */
  real32_T ticks[3];                   /* '<Root>/ticks' */
} ExtY_three_loop_deploy_T;

/* Block states (default storage) */
extern DW_three_loop_deploy_T three_loop_deploy_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_three_loop_deploy_T three_loop_deploy_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_three_loop_deploy_T three_loop_deploy_Y;

/* Model entry point functions */
extern void three_loop_deploy_initialize(void);
extern void three_loop_deploy_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'three_loop_deploy'
 * '<S1>'   : 'three_loop_deploy/ThreeLoop_Controller'
 * '<S2>'   : 'three_loop_deploy/ThreeLoop_Controller/Current_Loop'
 * '<S3>'   : 'three_loop_deploy/ThreeLoop_Controller/Position_Loop'
 * '<S4>'   : 'three_loop_deploy/ThreeLoop_Controller/Speed_Loop'
 */
#endif                                 /* RTW_HEADER_three_loop_deploy_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
