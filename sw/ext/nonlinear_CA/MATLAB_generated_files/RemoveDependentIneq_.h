/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RemoveDependentIneq_.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

#ifndef REMOVEDEPENDENTINEQ__H
#define REMOVEDEPENDENTINEQ__H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void RemoveDependentIneq_(o_struct_T *workingset, f_struct_T *qrmanager,
                          h_struct_T *memspace, double tolfactor);

void b_RemoveDependentIneq_(q_struct_T *workingset, i_struct_T *qrmanager,
                            k_struct_T *memspace, double tolfactor);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for RemoveDependentIneq_.h
 *
 * [EOF]
 */
