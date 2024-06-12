/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xgemv.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                const emxArray_real_T *A
 *                const double x_data[]
 *                emxArray_real_T *y
 * Return Type  : void
 */
void b_xgemv(int m, int n, const emxArray_real_T *A, const double x_data[],
             emxArray_real_T *y)
{
  const double *A_data;
  double *y_data;
  int ia;
  int iac;
  int iy;
  y_data = y->data;
  A_data = A->data;
  if (m != 0) {
    for (iy = 0; iy < n; iy++) {
      y_data[iy] = 0.0;
    }
    iy = 311 * (n - 1) + 1;
    for (iac = 1; iac <= iy; iac += 311) {
      double c;
      int i;
      c = 0.0;
      i = (iac + m) - 1;
      for (ia = iac; ia <= i; ia++) {
        c += A_data[ia - 1] * x_data[ia - iac];
      }
      i = div_nde_s32_floor(iac - 1, 311);
      y_data[i] += c;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const emxArray_real_T *A
 *                const emxArray_real_T *x
 *                double y_data[]
 * Return Type  : void
 */
void c_xgemv(int m, int n, const emxArray_real_T *A, const emxArray_real_T *x,
             double y_data[])
{
  const double *A_data;
  const double *x_data;
  int i;
  int ia;
  int iac;
  int iy;
  x_data = x->data;
  A_data = A->data;
  i = (unsigned char)n;
  for (iy = 0; iy < i; iy++) {
    y_data[iy] = -y_data[iy];
  }
  i = 183 * (n - 1) + 1;
  for (iac = 1; iac <= i; iac += 183) {
    double c;
    c = 0.0;
    iy = (iac + m) - 1;
    for (ia = iac; ia <= iy; ia++) {
      c += A_data[ia - 1] * x_data[(ia - iac) + 311];
    }
    iy = div_nde_s32_floor(iac - 1, 183);
    y_data[iy] += c;
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const emxArray_real_T *A
 *                const emxArray_real_T *x
 *                double y_data[]
 * Return Type  : void
 */
void d_xgemv(int m, int n, const emxArray_real_T *A, const emxArray_real_T *x,
             double y_data[])
{
  const double *A_data;
  const double *x_data;
  int i;
  int ia;
  int iac;
  int iy;
  x_data = x->data;
  A_data = A->data;
  i = (unsigned char)n;
  for (iy = 0; iy < i; iy++) {
    y_data[iy] = -y_data[iy];
  }
  i = 183 * (n - 1) + 1;
  for (iac = 1; iac <= i; iac += 183) {
    double c;
    c = 0.0;
    iy = (iac + m) - 1;
    for (ia = iac; ia <= iy; ia++) {
      c += A_data[ia - 1] * x_data[ia - iac];
    }
    iy = div_nde_s32_floor(iac - 1, 183);
    y_data[iy] += c;
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const emxArray_real_T *A
 *                const double x_data[]
 *                double y_data[]
 * Return Type  : void
 */
void e_xgemv(int m, int n, const emxArray_real_T *A, const double x_data[],
             double y_data[])
{
  const double *A_data;
  int i;
  int ia;
  int iac;
  int iy;
  A_data = A->data;
  i = (unsigned char)n;
  for (iy = 0; iy < i; iy++) {
    y_data[iy] = -y_data[iy];
  }
  i = 183 * (n - 1) + 1;
  for (iac = 1; iac <= i; iac += 183) {
    double c;
    c = 0.0;
    iy = (iac + m) - 1;
    for (ia = iac; ia <= iy; ia++) {
      c += A_data[ia - 1] * x_data[ia - iac];
    }
    iy = div_nde_s32_floor(iac - 1, 183);
    y_data[iy] += c;
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const emxArray_real_T *A
 *                const double x_data[]
 *                emxArray_real_T *y
 * Return Type  : void
 */
void f_xgemv(int m, int n, const emxArray_real_T *A, const double x_data[],
             emxArray_real_T *y)
{
  const double *A_data;
  double *y_data;
  int i;
  int ia;
  int iac;
  int iy;
  y_data = y->data;
  A_data = A->data;
  i = (unsigned char)n;
  for (iy = 0; iy < i; iy++) {
    y_data[iy] = -y_data[iy];
  }
  i = 183 * (n - 1) + 1;
  for (iac = 1; iac <= i; iac += 183) {
    double c;
    c = 0.0;
    iy = (iac + m) - 1;
    for (ia = iac; ia <= iy; ia++) {
      c += A_data[ia - 1] * x_data[ia - iac];
    }
    iy = div_nde_s32_floor(iac - 1, 183);
    y_data[iy] += c;
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[324]
 *                int lda
 *                const double x_data[]
 *                double y_data[]
 * Return Type  : void
 */
void xgemv(int m, int n, const double A[324], int lda, const double x_data[],
           double y_data[])
{
  int ia;
  int iac;
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = (unsigned char)m;
    memset(&y_data[0], 0, (unsigned int)i * sizeof(double));
    ix = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y_data[i2] += A[ia - 1] * x_data[ix];
      }
      ix++;
    }
  }
}

/*
 * File trailer for xgemv.c
 *
 * [EOF]
 */
