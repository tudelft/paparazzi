/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: driver1.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Jun-2024 11:13:45
 */

#ifndef DRIVER1_H
#define DRIVER1_H

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_driver(const double lb[15], const double ub[15], h_struct_T *TrialState,
              b_struct_T *MeritFunction,
              i_coder_internal_stickyStruct *FcnEvaluator, f_struct_T *memspace,
              i_struct_T *WorkingSet, double Hessian[225],
              d_struct_T *QRManager, e_struct_T *CholManager,
              struct_T *QPObjective);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for driver1.h
 *
 * [EOF]
 */
