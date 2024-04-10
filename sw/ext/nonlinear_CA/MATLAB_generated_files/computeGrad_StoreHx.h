/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGrad_StoreHx.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

#ifndef COMPUTEGRAD_STOREHX_H
#define COMPUTEGRAD_STOREHX_H

/* Include Files */
#include "Nonlinear_CA_w_ail_approach_ext_acc_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void computeGrad_StoreHx(struct_T *obj, const double H[225], const double f[16],
                         const double x[16]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeGrad_StoreHx.h
 *
 * [EOF]
 */
