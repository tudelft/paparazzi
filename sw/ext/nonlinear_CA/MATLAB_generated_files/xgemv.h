/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef XGEMV_H
#define XGEMV_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xgemv(int m, int n, const double A[169], int lda, const double x[14],
             double y[13]);

void c_xgemv(int m, int n, const double A[961], const double x[16],
             double y[496]);

void d_xgemv(int m, int n, const double A[729], const double x[14],
             double y[378]);

void xgemv(int m, int n, const double A[225], int lda, const double x[16],
           double y[15]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgemv.h
 *
 * [EOF]
 */
