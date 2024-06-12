/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzgeqp3.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xzgeqp3.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *A
 *                int m
 *                int n
 *                int nfxd
 *                double tau_data[]
 * Return Type  : void
 */
void qrf(emxArray_real_T *A, int m, int n, int nfxd, double tau_data[])
{
  double work_data[311];
  double atmp;
  double *A_data;
  int b_i;
  int i;
  A_data = A->data;
  memset(&work_data[0], 0, 311U * sizeof(double));
  i = (unsigned char)nfxd;
  for (b_i = 0; b_i < i; b_i++) {
    double d;
    int ii;
    int mmi;
    ii = b_i * 311 + b_i;
    mmi = m - b_i;
    if (b_i + 1 < m) {
      atmp = A_data[ii];
      d = xzlarfg(mmi, &atmp, A, ii + 2);
      A_data = A->data;
      tau_data[b_i] = d;
      A_data[ii] = atmp;
    } else {
      d = 0.0;
      tau_data[b_i] = 0.0;
    }
    if (b_i + 1 < n) {
      atmp = A_data[ii];
      A_data[ii] = 1.0;
      xzlarf(mmi, (n - b_i) - 1, ii + 1, d, A, ii + 312, work_data);
      A_data = A->data;
      A_data[ii] = atmp;
    }
  }
}

/*
 * File trailer for xzgeqp3.c
 *
 * [EOF]
 */
