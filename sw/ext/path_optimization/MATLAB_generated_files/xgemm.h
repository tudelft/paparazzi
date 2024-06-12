/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemm.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef XGEMM_H
#define XGEMM_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xgemm(int m, int n, int k, const emxArray_real_T *A, int ia0,
             const emxArray_real_T *B, emxArray_real_T *C);

void xgemm(int m, int n, int k, const double A[324], int lda,
           const emxArray_real_T *B, int ib0, emxArray_real_T *C);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgemm.h
 *
 * [EOF]
 */
