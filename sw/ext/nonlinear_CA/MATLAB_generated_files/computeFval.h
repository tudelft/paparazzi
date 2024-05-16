/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

#ifndef COMPUTEFVAL_H
#define COMPUTEFVAL_H

/* Include Files */
#include "Nonlinear_controller_w_ail_basic_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double computeFval(const struct_T *obj, double workspace[496],
                   const double H[225], const double f[16], const double x[16]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeFval.h
 *
 * [EOF]
 */
