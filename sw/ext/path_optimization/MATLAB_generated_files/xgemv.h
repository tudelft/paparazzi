/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef XGEMV_H
#define XGEMV_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xgemv(int m, int n, const emxArray_real_T *A, const double x_data[],
             emxArray_real_T *y);

void c_xgemv(int m, int n, const emxArray_real_T *A, const emxArray_real_T *x,
             double y_data[]);

void d_xgemv(int m, int n, const emxArray_real_T *A, const emxArray_real_T *x,
             double y_data[]);

void e_xgemv(int m, int n, const emxArray_real_T *A, const double x_data[],
             double y_data[]);

void f_xgemv(int m, int n, const emxArray_real_T *A, const double x_data[],
             emxArray_real_T *y);

void xgemv(int m, int n, const double A[324], int lda, const double x_data[],
           double y_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgemv.h
 *
 * [EOF]
 */
