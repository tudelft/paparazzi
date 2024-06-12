/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: checkMatrixNonFinite.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "checkMatrixNonFinite.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int ncols
 *                const emxArray_real_T *mat
 * Return Type  : int
 */
int checkMatrixNonFinite(int ncols, const emxArray_real_T *mat)
{
  const double *mat_data;
  int col;
  int idx_mat;
  int row;
  int status;
  bool allFinite;
  mat_data = mat->data;
  status = 1;
  allFinite = true;
  row = -1;
  col = -1;
  while (allFinite && (col + 2 <= ncols)) {
    row = -1;
    while (allFinite && (row + 2 <= 18)) {
      idx_mat = (row + 183 * (col + 1)) + 1;
      allFinite =
          ((!rtIsInf(mat_data[idx_mat])) && (!rtIsNaN(mat_data[idx_mat])));
      row++;
    }
    col++;
  }
  if (!allFinite) {
    idx_mat = row + 183 * col;
    if (rtIsNaN(mat_data[idx_mat])) {
      status = -3;
    } else if (mat_data[idx_mat] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  return status;
}

/*
 * File trailer for checkMatrixNonFinite.c
 *
 * [EOF]
 */
