/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: factorQR.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef FACTORQR_H
#define FACTORQR_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_factorQR(i_struct_T *obj, const double A[378], int mrows, int ncols);

void factorQR(f_struct_T *obj, const double A[496], int mrows, int ncols);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for factorQR.h
 *
 * [EOF]
 */
