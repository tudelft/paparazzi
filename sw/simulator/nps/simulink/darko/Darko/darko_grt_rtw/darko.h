/*
 * darko.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "darko".
 *
 * Model version              : 1.6
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Tue Dec  6 10:09:29 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_darko_h_
#define RTW_HEADER_darko_h_
#ifndef darko_COMMON_INCLUDES_
#define darko_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* darko_COMMON_INCLUDES_ */

#include "darko_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE[13];/* '<Root>/Discrete-Time Integrator' */
} DW_darko_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: initial_state
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  real_T DiscreteTimeIntegrator_IC[13];
} ConstP_darko_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u[4];                         /* '<Root>/u' */
  real_T w[3];                         /* '<Root>/w' */
} ExtU_darko_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T p[3];                         /* '<Root>/p' */
  real_T v[3];                         /* '<Root>/v' */
  real_T q[4];                         /* '<Root>/q' */
  real_T omega[3];                     /* '<Root>/omega' */
} ExtY_darko_T;

/* Real-time Model Data Structure */
struct tag_RTM_darko_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block states (default storage) */
extern DW_darko_T darko_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_darko_T darko_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_darko_T darko_Y;

/* Constant parameters (default storage) */
extern const ConstP_darko_T darko_ConstP;

/* Model entry point functions */
extern void darko_initialize(void);
extern void darko_step(void);
extern void darko_terminate(void);

/* Real-time Model object */
extern RT_MODEL_darko_T *const darko_M;

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
 */
#endif                                 /* RTW_HEADER_darko_h_ */
