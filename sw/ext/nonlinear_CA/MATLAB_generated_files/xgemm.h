/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemm.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 07-May-2024 15:18:42
 */

#ifndef XGEMM_H
#define XGEMM_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xgemm(int m, int n, int k, const double A[961], int ia0,
             const double B[496], double C[961]);

void c_xgemm(int m, int n, int k, const double A[169], int lda,
             const double B[729], int ib0, double C[378]);

void d_xgemm(int m, int n, int k, const double A[729], int ia0,
             const double B[378], double C[729]);

void xgemm(int m, int n, int k, const double A[225], int lda,
           const double B[961], int ib0, double C[496]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgemm.h
 *
 * [EOF]
 */
