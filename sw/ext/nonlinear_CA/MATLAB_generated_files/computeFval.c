/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Jun-2024 11:13:45
 */

/* Include Files */
#include "computeFval.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "linearForm_.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const struct_T *obj
 *                double workspace[496]
 *                const double H[225]
 *                const double f[16]
 *                const double x[16]
 * Return Type  : double
 */
double computeFval(const struct_T *obj, double workspace[496],
                   const double H[225], const double f[16], const double x[16])
{
  double val;
  int idx;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3: {
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    val = 0.0;
    if (obj->nvar >= 1) {
      int i;
      i = (unsigned char)obj->nvar;
      for (idx = 0; idx < i; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
  } break;
  default: {
    int i;
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    i = obj->nvar + 1;
    for (idx = i; idx < 16; idx++) {
      workspace[idx - 1] = 0.5 * obj->beta * x[idx - 1] + obj->rho;
    }
    val = 0.0;
    for (idx = 0; idx < 15; idx++) {
      val += x[idx] * workspace[idx];
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
