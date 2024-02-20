/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeFval_ReuseHx.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

/* Include Files */
#include "computeFval_ReuseHx.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const c_struct_T *obj
 *                double workspace[378]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : double
 */
double b_computeFval_ReuseHx(const c_struct_T *obj, double workspace[378],
                             const double f[14], const double x[14])
{
  double val;
  int k;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3: {
    if (obj->hasLinear) {
      int i;
      i = (unsigned char)obj->nvar;
      for (k = 0; k < i; k++) {
        workspace[k] = 0.5 * obj->Hx[k] + f[k];
      }
      val = 0.0;
      if (obj->nvar >= 1) {
        for (k = 0; k < i; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      val = 0.0;
      if (obj->nvar >= 1) {
        int i;
        i = (unsigned short)obj->nvar;
        for (k = 0; k < i; k++) {
          val += x[k] * obj->Hx[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    if (obj->hasLinear) {
      int i;
      i = (unsigned char)obj->nvar;
      if (i - 1 >= 0) {
        memcpy(&workspace[0], &f[0], (unsigned int)i * sizeof(double));
      }
      i = 12 - obj->nvar;
      for (k = 0; k <= i; k++) {
        workspace[obj->nvar + k] = obj->rho;
      }
      val = 0.0;
      for (k = 0; k < 13; k++) {
        double d;
        d = workspace[k] + 0.5 * obj->Hx[k];
        workspace[k] = d;
        val += x[k] * d;
      }
    } else {
      int i;
      val = 0.0;
      for (k = 0; k < 13; k++) {
        val += x[k] * obj->Hx[k];
      }
      val *= 0.5;
      i = obj->nvar + 1;
      for (k = i; k < 14; k++) {
        val += x[k - 1] * obj->rho;
      }
    }
  } break;
  }
  return val;
}

/*
 * Arguments    : const struct_T *obj
 *                double workspace[496]
 *                const double f[16]
 *                const double x[16]
 * Return Type  : double
 */
double computeFval_ReuseHx(const struct_T *obj, double workspace[496],
                           const double f[16], const double x[16])
{
  double val;
  int k;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3: {
    if (obj->hasLinear) {
      int i;
      i = (unsigned char)obj->nvar;
      for (k = 0; k < i; k++) {
        workspace[k] = 0.5 * obj->Hx[k] + f[k];
      }
      val = 0.0;
      if (obj->nvar >= 1) {
        for (k = 0; k < i; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      val = 0.0;
      if (obj->nvar >= 1) {
        int i;
        i = (unsigned char)obj->nvar;
        for (k = 0; k < i; k++) {
          val += x[k] * obj->Hx[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    if (obj->hasLinear) {
      int i;
      i = (unsigned char)obj->nvar;
      if (i - 1 >= 0) {
        memcpy(&workspace[0], &f[0], (unsigned int)i * sizeof(double));
      }
      i = 14 - obj->nvar;
      for (k = 0; k <= i; k++) {
        workspace[obj->nvar + k] = obj->rho;
      }
      val = 0.0;
      for (k = 0; k < 15; k++) {
        double d;
        d = workspace[k] + 0.5 * obj->Hx[k];
        workspace[k] = d;
        val += x[k] * d;
      }
    } else {
      int i;
      val = 0.0;
      for (k = 0; k < 15; k++) {
        val += x[k] * obj->Hx[k];
      }
      val *= 0.5;
      i = obj->nvar + 1;
      for (k = i; k < 16; k++) {
        val += x[k - 1] * obj->rho;
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
