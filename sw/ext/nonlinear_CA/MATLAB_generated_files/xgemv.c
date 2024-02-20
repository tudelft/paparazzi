/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

/* Include Files */
#include "xgemv.h"
#include "Cascaded_nonlinear_TestFlight_rtwutil.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                const double A[169]
 *                int lda
 *                const double x[14]
 *                double y[13]
 * Return Type  : void
 */
void b_xgemv(int m, int n, const double A[169], int lda, const double x[14],
             double y[13])
{
  int ia;
  int iac;
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = (unsigned char)m;
    if (i - 1 >= 0) {
      memset(&y[0], 0, (unsigned int)i * sizeof(double));
    }
    ix = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y[i2] += A[ia - 1] * x[ix];
      }
      ix++;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[961]
 *                const double x[16]
 *                double y[496]
 * Return Type  : void
 */
void c_xgemv(int m, int n, const double A[961], const double x[16],
             double y[496])
{
  int ia;
  int iac;
  if (m != 0) {
    int i;
    memset(&y[0], 0, (unsigned int)n * sizeof(double));
    i = 31 * (n - 1) + 1;
    for (iac = 1; iac <= i; iac += 31) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      i1 = div_nde_s32_floor(iac - 1, 31);
      y[i1] += c;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[729]
 *                const double x[14]
 *                double y[378]
 * Return Type  : void
 */
void d_xgemv(int m, int n, const double A[729], const double x[14],
             double y[378])
{
  int ia;
  int iac;
  if (m != 0) {
    int i;
    memset(&y[0], 0, (unsigned int)n * sizeof(double));
    i = 27 * (n - 1) + 1;
    for (iac = 1; iac <= i; iac += 27) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      i1 = div_nde_s32_floor(iac - 1, 27);
      y[i1] += c;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[225]
 *                int lda
 *                const double x[16]
 *                double y[15]
 * Return Type  : void
 */
void xgemv(int m, int n, const double A[225], int lda, const double x[16],
           double y[15])
{
  int ia;
  int iac;
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = (unsigned char)m;
    memset(&y[0], 0, (unsigned int)i * sizeof(double));
    ix = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y[i2] += A[ia - 1] * x[ix];
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
