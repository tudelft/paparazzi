/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: solve.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "solve.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const h_struct_T *obj
 *                double rhs_data[]
 * Return Type  : void
 */
void solve(const h_struct_T *obj, double rhs_data[])
{
  int i;
  int j;
  int jA;
  int n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    for (j = 0; j < n_tmp; j++) {
      double temp;
      jA = j * 311;
      temp = rhs_data[j];
      for (i = 0; i < j; i++) {
        temp -= obj->FMat->data[jA + i] * rhs_data[i];
      }
      rhs_data[j] = temp / obj->FMat->data[jA + j];
    }
  }
  if (obj->ndims != 0) {
    for (j = n_tmp; j >= 1; j--) {
      jA = (j + (j - 1) * 311) - 1;
      rhs_data[j - 1] /= obj->FMat->data[jA];
      for (i = 0; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        rhs_data[ix] -= rhs_data[j - 1] * obj->FMat->data[(jA - i) - 1];
      }
    }
  }
}

/*
 * File trailer for solve.c
 *
 * [EOF]
 */
