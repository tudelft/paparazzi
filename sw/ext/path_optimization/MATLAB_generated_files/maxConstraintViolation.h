/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: maxConstraintViolation.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef MAXCONSTRAINTVIOLATION_H
#define MAXCONSTRAINTVIOLATION_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double b_maxConstraintViolation(f_struct_T *obj, const double x_data[]);

double c_maxConstraintViolation(f_struct_T *obj, const emxArray_real_T *x);

double maxConstraintViolation(f_struct_T *obj, const emxArray_real_T *x);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for maxConstraintViolation.h
 *
 * [EOF]
 */
