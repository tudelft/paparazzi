/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: driver1.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

#ifndef DRIVER1_H
#define DRIVER1_H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_driver(const double lb[15], const double ub[15], n_struct_T *TrialState,
              b_struct_T *MeritFunction,
              i_coder_internal_stickyStruct *FcnEvaluator, h_struct_T *memspace,
              o_struct_T *WorkingSet, double Hessian[225],
              f_struct_T *QRManager, g_struct_T *CholManager,
              struct_T *QPObjective);

void d_driver(const double lb[13], const double ub[13], p_struct_T *TrialState,
              b_struct_T *MeritFunction,
              r_coder_internal_stickyStruct *FcnEvaluator, k_struct_T *memspace,
              q_struct_T *WorkingSet, double Hessian[169],
              i_struct_T *QRManager, j_struct_T *CholManager,
              c_struct_T *QPObjective);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for driver1.h
 *
 * [EOF]
 */
