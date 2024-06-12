/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: factorQR.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "factorQR.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : g_struct_T *obj
 *                const emxArray_real_T *A
 *                int mrows
 *                int ncols
 * Return Type  : void
 */
void factorQR(g_struct_T *obj, const emxArray_real_T *A, int mrows, int ncols)
{
  emxArray_real_T *y;
  double tau_data[311];
  const double *A_data;
  double *y_data;
  int i;
  int idx;
  int k;
  bool guard1;
  A_data = A->data;
  i = mrows * ncols;
  emxInit_real_T(&y, 2);
  guard1 = false;
  if (i > 0) {
    for (idx = 0; idx < ncols; idx++) {
      int ix0;
      int iy0;
      ix0 = 183 * idx;
      iy0 = 311 * idx;
      i = y->size[0] * y->size[1];
      y->size[0] = 311;
      y->size[1] = 311;
      emxEnsureCapacity_real_T(y, i);
      y_data = y->data;
      for (i = 0; i < 96721; i++) {
        y_data[i] = obj->QR->data[i];
      }
      i = (unsigned char)mrows;
      for (k = 0; k < i; k++) {
        y_data[iy0 + k] = A_data[ix0 + k];
      }
      i = obj->QR->size[0] * obj->QR->size[1];
      obj->QR->size[0] = 311;
      obj->QR->size[1] = 311;
      emxEnsureCapacity_real_T(obj->QR, i);
      for (i = 0; i < 96721; i++) {
        obj->QR->data[i] = y_data[i];
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
      obj->jpvt.data[idx] = idx + 1;
    }
    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }
    obj->minRowCol = i;
    memset(&tau_data[0], 0, 311U * sizeof(double));
    if (i >= 1) {
      qrf(obj->QR, mrows, ncols, i, tau_data);
    }
    obj->tau.size[0] = 311;
    memcpy(&obj->tau.data[0], &tau_data[0], 311U * sizeof(double));
  }
  emxFree_real_T(&y);
}

/*
 * File trailer for factorQR.c
 *
 * [EOF]
 */
