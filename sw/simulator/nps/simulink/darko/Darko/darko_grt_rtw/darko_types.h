/*
 * darko_types.h
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

#ifndef RTW_HEADER_darko_types_h_
#define RTW_HEADER_darko_types_h_
#include "rtwtypes.h"
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

#ifndef SS_UINT64
#define SS_UINT64                      18
#endif

#ifndef SS_INT64
#define SS_INT64                       19
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_darko_T RT_MODEL_darko_T;

#endif                                 /* RTW_HEADER_darko_types_h_ */
