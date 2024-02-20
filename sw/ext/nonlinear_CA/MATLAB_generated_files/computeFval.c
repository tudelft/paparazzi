/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

/* Include Files */
#include "computeFval.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "linearForm_.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const c_struct_T *obj
 *                double workspace[378]
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : double
 */
double b_computeFval(const c_struct_T *obj, double workspace[378],
                     const double H[169], const double f[14],
                     const double x[14])
{
  double val;
  int idx;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3: {
    b_linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
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
    b_linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    i = obj->nvar + 1;
    for (idx = i; idx < 14; idx++) {
      workspace[idx - 1] = 0.5 * obj->beta * x[idx - 1] + obj->rho;
    }
    val = 0.0;
    for (idx = 0; idx < 13; idx++) {
      val += x[idx] * workspace[idx];
    }
  } break;
  }
  return val;
}

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
