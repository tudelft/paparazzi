/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RemoveDependentIneq_.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-May-2024 00:26:53
 */

#ifndef REMOVEDEPENDENTINEQ__H
#define REMOVEDEPENDENTINEQ__H

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void RemoveDependentIneq_(i_struct_T *workingset, d_struct_T *qrmanager,
                          f_struct_T *memspace, double tolfactor);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for RemoveDependentIneq_.h
 *
 * [EOF]
 */
