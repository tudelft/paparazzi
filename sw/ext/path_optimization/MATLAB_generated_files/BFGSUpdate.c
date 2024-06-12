/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: BFGSUpdate.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "BFGSUpdate.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int nvar
 *                double Bk[324]
 *                const double sk_data[]
 *                double yk_data[]
 *                emxArray_real_T *workspace
 * Return Type  : bool
 */
bool BFGSUpdate(int nvar, double Bk[324], const double sk_data[],
                double yk_data[], emxArray_real_T *workspace)
{
  double curvatureS;
  double dotSY;
  double theta;
  double *workspace_data;
  int i;
  int i1;
  int i2;
  int ia;
  int iac;
  int ix;
  int k;
  bool success;
  workspace_data = workspace->data;
  dotSY = 0.0;
  if (nvar >= 1) {
    i = (unsigned char)nvar;
    for (k = 0; k < i; k++) {
      dotSY += sk_data[k] * yk_data[k];
    }
  }
  i = (unsigned char)nvar;
  for (ix = 0; ix < i; ix++) {
    workspace_data[ix] = 0.0;
  }
  ix = 0;
  i1 = 18 * (nvar - 1) + 1;
  for (iac = 1; iac <= i1; iac += 18) {
    i2 = (iac + nvar) - 1;
    for (ia = iac; ia <= i2; ia++) {
      k = ia - iac;
      workspace_data[k] += Bk[ia - 1] * sk_data[ix];
    }
    ix++;
  }
  curvatureS = 0.0;
  if (nvar >= 1) {
    for (k = 0; k < i; k++) {
      curvatureS += sk_data[k] * workspace_data[k];
    }
  }
  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    for (k = 0; k < i; k++) {
      yk_data[k] *= theta;
    }
    if (!(1.0 - theta == 0.0)) {
      ix = nvar - 1;
      for (k = 0; k <= ix; k++) {
        yk_data[k] += (1.0 - theta) * workspace_data[k];
      }
    }
    dotSY = 0.0;
    if (nvar >= 1) {
      for (k = 0; k < i; k++) {
        dotSY += sk_data[k] * yk_data[k];
      }
    }
  }
  if ((curvatureS > 2.2204460492503131E-16) &&
      (dotSY > 2.2204460492503131E-16)) {
    success = true;
  } else {
    success = false;
  }
  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      for (k = 0; k < i; k++) {
        theta = workspace_data[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += workspace_data[(iac - ix) - 1] * theta;
          }
        }
        ix += 18;
      }
    }
    curvatureS = 1.0 / dotSY;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      for (k = 0; k < i; k++) {
        theta = yk_data[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += yk_data[(iac - ix) - 1] * theta;
          }
        }
        ix += 18;
      }
    }
  }
  return success;
}

/*
 * File trailer for BFGSUpdate.c
 *
 * [EOF]
 */
