/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef COMPUTEFVAL_H
#define COMPUTEFVAL_H

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
double computeFval(const d_struct_T *obj, emxArray_real_T *workspace,
                   const double H[324], const double f_data[],
                   const double x_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeFval.h
 *
 * [EOF]
 */
