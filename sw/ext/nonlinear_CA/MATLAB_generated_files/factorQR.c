/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: factorQR.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

/* Include Files */
#include "factorQR.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : d_struct_T *obj
 *                const double A[496]
 *                int mrows
 *                int ncols
 * Return Type  : void
 */
void factorQR(d_struct_T *obj, const double A[496], int mrows, int ncols)
{
  int i;
  int idx;
  int k;
  bool guard1;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    for (idx = 0; idx < ncols; idx++) {
      int ix0;
      int iy0;
      ix0 = idx << 4;
      iy0 = 31 * idx;
      i = (unsigned char)mrows;
      for (k = 0; k < i; k++) {
        obj->QR[iy0 + k] = A[ix0 + k];
      }
    }
    guard1 = true;
  } else if (i == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt[idx] = idx + 1;
    }
    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }
    obj->minRowCol = i;
    memset(&obj->tau[0], 0, 31U * sizeof(double));
    if (i >= 1) {
      qrf(obj->QR, mrows, ncols, i, obj->tau);
    }
  }
}

/*
 * File trailer for factorQR.c
 *
 * [EOF]
 */
