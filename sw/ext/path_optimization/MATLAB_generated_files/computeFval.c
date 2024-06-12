/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "computeFval.h"
#include "linearForm_.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const d_struct_T *obj
 *                emxArray_real_T *workspace
 *                const double H[324]
 *                const double f_data[]
 *                const double x_data[]
 * Return Type  : double
 */
double computeFval(const d_struct_T *obj, emxArray_real_T *workspace,
                   const double H[324], const double f_data[],
                   const double x_data[])
{
  double val;
  double *workspace_data;
  int idx;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x_data[obj->nvar - 1];
    break;
  case 3: {
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f_data, x_data);
    workspace_data = workspace->data;
    val = 0.0;
    if (obj->nvar >= 1) {
      int i;
      i = (unsigned char)obj->nvar;
      for (idx = 0; idx < i; idx++) {
        val += x_data[idx] * workspace_data[idx];
      }
    }
  } break;
  default: {
    int i;
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f_data, x_data);
    workspace_data = workspace->data;
    i = obj->nvar + 1;
    for (idx = i; idx < 183; idx++) {
      workspace_data[idx - 1] = 0.5 * obj->beta * x_data[idx - 1] + obj->rho;
    }
    val = 0.0;
    for (idx = 0; idx < 182; idx++) {
      val += x_data[idx] * workspace_data[idx];
    }
  } break;
  }
  return val;
}

/*
 * File trailer for computeFval.c
 *
 * [EOF]
 */
