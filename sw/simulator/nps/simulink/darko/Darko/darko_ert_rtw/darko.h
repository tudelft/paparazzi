/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: darko.h
 *
 * Code generated for Simulink model 'darko'.
 *
 * Model version                  : 1.20
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Wed Mar  1 17:24:17 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_darko_h_
#define RTW_HEADER_darko_h_
#ifndef darko_COMMON_INCLUDES_
#define darko_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* darko_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef DEFINED_TYPEDEF_FOR_struct_YysP9KtKw43hFSdxSlefvD_
#define DEFINED_TYPEDEF_FOR_struct_YysP9KtKw43hFSdxSlefvD_

typedef struct {
  real_T MASS;
  real_T INERTIA[9];
  real_T G;
  real_T P_P1_CG[3];
  real_T P_P2_CG[3];
  real_T P_A1_CG[3];
  real_T P_A2_CG[3];
  real_T INERTIA_PROP_X;
  real_T INERTIA_PROP_N;
  real_T PHI[36];
  real_T RHO;
  real_T WET_SURFACE;
  real_T DRY_SURFACE;
  real_T PHI_n;
  real_T CHORD;
  real_T WINGSPAN;
  real_T PROP_RADIUS;
  real_T ELEVON_MEFFICIENCY[3];
  real_T ELEVON_FEFFICIENCY[3];
  real_T PROP_KP;
  real_T THICKNESS;
  real_T PROP_KM;
} struct_YysP9KtKw43hFSdxSlefvD;

#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE[13];/* '<Root>/Discrete-Time Integrator' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: initial_state
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  real_T DiscreteTimeIntegrator_IC[13];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u[4];                         /* '<Root>/u' */
  real_T w[3];                         /* '<Root>/w' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T p[3];                         /* '<Root>/p' */
  real_T v[3];                         /* '<Root>/v' */
  real_T q[4];                         /* '<Root>/q' */
  real_T omega[3];                     /* '<Root>/omega' */
  real_T accel[3];                     /* '<Root>/accel' */
  real_T rotaccel[3];                  /* '<Root>/rotaccel' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void darko_initialize(void);
extern void darko_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

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
 * '<Root>' : 'darko'
 * '<S1>'   : 'darko/MATLAB Function'
 * '<S2>'   : 'darko/MATLAB Function1'
 * '<S3>'   : 'darko/down to up'
 * '<S4>'   : 'darko/down to up1'
 */
#endif                                 /* RTW_HEADER_darko_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
