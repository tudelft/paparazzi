/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: step.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef STEP_H
#define STEP_H

/* Include Files */
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
bool step(int *STEP_TYPE, double Hessian[324], c_struct_T *TrialState,
          struct_T *MeritFunction, e_struct_T *memspace, f_struct_T *WorkingSet,
          g_struct_T *QRManager, h_struct_T *CholManager,
          d_struct_T *QPObjective, j_struct_T *qpoptions);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for step.h
 *
 * [EOF]
 */
