/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval_ReuseHx.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "computeFval_ReuseHx.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const d_struct_T *obj
 *                emxArray_real_T *workspace
 *                const double f_data[]
 *                const double x_data[]
 * Return Type  : double
 */
double computeFval_ReuseHx(const d_struct_T *obj, emxArray_real_T *workspace,
                           const double f_data[], const double x_data[])
{
  double val;
  double *workspace_data;
  int k;
  workspace_data = workspace->data;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x_data[obj->nvar - 1];
    break;
  case 3: {
    if (obj->hasLinear) {
      int i;
      i = (unsigned char)obj->nvar;
      for (k = 0; k < i; k++) {
        workspace_data[k] = 0.5 * obj->Hx.data[k] + f_data[k];
      }
      val = 0.0;
      if (obj->nvar >= 1) {
        for (k = 0; k < i; k++) {
          val += x_data[k] * workspace_data[k];
        }
      }
    } else {
      val = 0.0;
      if (obj->nvar >= 1) {
        int i;
        i = (unsigned char)obj->nvar;
        for (k = 0; k < i; k++) {
          val += x_data[k] * obj->Hx.data[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    if (obj->hasLinear) {
      int i;
      i = (unsigned char)obj->nvar;
      for (k = 0; k < i; k++) {
        workspace_data[k] = f_data[k];
      }
      i = 181 - obj->nvar;
      for (k = 0; k <= i; k++) {
        workspace_data[obj->nvar + k] = obj->rho;
      }
      val = 0.0;
      for (k = 0; k < 182; k++) {
        double d;
        d = workspace_data[k] + 0.5 * obj->Hx.data[k];
        workspace_data[k] = d;
        val += x_data[k] * d;
      }
    } else {
      int i;
      val = 0.0;
      for (k = 0; k < 182; k++) {
        val += x_data[k] * obj->Hx.data[k];
      }
      val *= 0.5;
      i = obj->nvar + 1;
      for (k = i; k < 183; k++) {
        val += x_data[k - 1] * obj->rho;
      }
    }
  } break;
  }
  return val;
}

/*
 * File trailer for computeFval_ReuseHx.c
 *
 * [EOF]
 */
