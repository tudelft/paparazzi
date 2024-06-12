/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: linearForm_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "linearForm_.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : bool obj_hasLinear
 *                int obj_nvar
 *                emxArray_real_T *workspace
 *                const double H[324]
 *                const double f_data[]
 *                const double x_data[]
 * Return Type  : void
 */
void linearForm_(bool obj_hasLinear, int obj_nvar, emxArray_real_T *workspace,
                 const double H[324], const double f_data[],
                 const double x_data[])
{
  double *workspace_data;
  int i;
  int ia;
  int iac;
  int ix;
  workspace_data = workspace->data;
  ix = 0;
  if (obj_hasLinear) {
    i = (unsigned char)obj_nvar;
    for (ix = 0; ix < i; ix++) {
      workspace_data[ix] = f_data[ix];
    }
    ix = 1;
  }
  if (obj_nvar != 0) {
    if (ix != 1) {
      i = (unsigned char)obj_nvar;
      for (ix = 0; ix < i; ix++) {
        workspace_data[ix] = 0.0;
      }
    }
    ix = 0;
    i = obj_nvar * (obj_nvar - 1) + 1;
    for (iac = 1; obj_nvar < 0 ? iac >= i : iac <= i; iac += obj_nvar) {
      double c;
      int i1;
      c = 0.5 * x_data[ix];
      i1 = (iac + obj_nvar) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        workspace_data[i2] += H[ia - 1] * c;
      }
      ix++;
    }
  }
}

/*
 * File trailer for linearForm_.c
 *
 * [EOF]
 */
