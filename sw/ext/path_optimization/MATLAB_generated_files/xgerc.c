/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgerc.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xgerc.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                double alpha1
 *                int ix0
 *                const double y_data[]
 *                emxArray_real_T *A
 *                int ia0
 * Return Type  : void
 */
void xgerc(int m, int n, double alpha1, int ix0, const double y_data[],
           emxArray_real_T *A, int ia0)
{
  double *A_data;
  int ijA;
  int j;
  A_data = A->data;
  if (!(alpha1 == 0.0)) {
    int jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      double temp;
      temp = y_data[j];
      if (temp != 0.0) {
        int i;
        temp *= alpha1;
        i = m + jA;
        for (ijA = jA; ijA < i; ijA++) {
          A_data[ijA - 1] += A_data[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += 311;
    }
  }
}

/*
 * File trailer for xgerc.c
 *
 * [EOF]
 */
