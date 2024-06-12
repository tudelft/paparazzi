/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xtrsv.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xtrsv.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int n
 *                const emxArray_real_T *A
 *                emxArray_real_T *x
 * Return Type  : void
 */
void xtrsv(int n, const emxArray_real_T *A, emxArray_real_T *x)
{
  const double *A_data;
  double *x_data;
  int i;
  int j;
  x_data = x->data;
  A_data = A->data;
  if (n != 0) {
    for (j = n; j >= 1; j--) {
      int jjA;
      jjA = (j + (j - 1) * 311) - 1;
      x_data[j - 1] /= A_data[jjA];
      for (i = 0; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        x_data[ix] -= x_data[j - 1] * A_data[(jjA - i) - 1];
      }
    }
  }
}

/*
 * File trailer for xtrsv.c
 *
 * [EOF]
 */
