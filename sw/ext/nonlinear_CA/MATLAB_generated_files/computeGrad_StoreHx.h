/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGrad_StoreHx.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

#ifndef COMPUTEGRAD_STOREHX_H
#define COMPUTEGRAD_STOREHX_H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_computeGrad_StoreHx(c_struct_T *obj, const double H[169],
                           const double f[14], const double x[14]);

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
