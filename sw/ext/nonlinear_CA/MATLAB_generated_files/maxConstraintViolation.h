/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: maxConstraintViolation.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

#ifndef MAXCONSTRAINTVIOLATION_H
#define MAXCONSTRAINTVIOLATION_H

/* Include Files */
#include "Nonlinear_controller_w_ail_basic_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double b_maxConstraintViolation(const i_struct_T *obj, const double x[16]);

double maxConstraintViolation(const i_struct_T *obj, const double x[496],
                              int ix0);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for maxConstraintViolation.h
 *
 * [EOF]
 */
