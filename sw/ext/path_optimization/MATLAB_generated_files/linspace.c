/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: linspace.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "linspace.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double d2
 *                double n
 *                emxArray_real_T *y
 * Return Type  : void
 */
void linspace(double d2, double n, emxArray_real_T *y)
{
  double *y_data;
  int k;
  if (!(n >= 0.0)) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    double delta1;
    int y_tmp_tmp;
    delta1 = floor(n);
    y_tmp_tmp = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)delta1;
    emxEnsureCapacity_real_T(y, y_tmp_tmp);
    y_data = y->data;
    if ((int)delta1 >= 1) {
      y_tmp_tmp = (int)delta1 - 1;
      y_data[(int)floor(n) - 1] = d2;
      if (y->size[1] >= 2) {
        y_data[0] = 0.0;
        if (y->size[1] >= 3) {
          if (-d2 == 0.0) {
            delta1 = d2 / ((double)y->size[1] - 1.0);
            for (k = 2; k <= y_tmp_tmp; k++) {
              y_data[k - 1] = (double)(((k << 1) - y->size[1]) - 1) * delta1;
            }
            if ((y->size[1] & 1) == 1) {
              y_data[y->size[1] >> 1] = 0.0;
            }
          } else if ((d2 < 0.0) && (fabs(d2) > 8.9884656743115785E+307)) {
            delta1 = d2 / ((double)y->size[1] - 1.0);
            y_tmp_tmp = y->size[1];
            for (k = 0; k <= y_tmp_tmp - 3; k++) {
              y_data[k + 1] = delta1 * ((double)k + 1.0);
            }
          } else {
            delta1 = d2 / ((double)y->size[1] - 1.0);
            y_tmp_tmp = y->size[1];
            for (k = 0; k <= y_tmp_tmp - 3; k++) {
              y_data[k + 1] = ((double)k + 1.0) * delta1;
            }
          }
        }
      }
    }
  }
}

/*
 * File trailer for linspace.c
 *
 * [EOF]
 */
