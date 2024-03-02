/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval_ReuseHx.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Mar-2024 00:35:29
 */

#ifndef COMPUTEFVAL_REUSEHX_H
#define COMPUTEFVAL_REUSEHX_H

/* Include Files */
#include "Nonlinear_controller_w_ailerons_rebuttal_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double computeFval_ReuseHx(const struct_T *obj, double workspace[496],
                           const double f[16], const double x[16]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeFval_ReuseHx.h
 *
 * [EOF]
 */
