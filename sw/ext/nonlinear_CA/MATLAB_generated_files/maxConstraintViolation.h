/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: maxConstraintViolation.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

#ifndef MAXCONSTRAINTVIOLATION_H
#define MAXCONSTRAINTVIOLATION_H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double b_maxConstraintViolation(const q_struct_T *obj, const double x[378],
                                int ix0);

double c_maxConstraintViolation(const o_struct_T *obj, const double x[16]);

double d_maxConstraintViolation(const q_struct_T *obj, const double x[14]);

double maxConstraintViolation(const o_struct_T *obj, const double x[496],
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
