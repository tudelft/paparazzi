/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGrad_StoreHx.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

/* Include Files */
#include "computeGrad_StoreHx.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : c_struct_T *obj
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : void
 */
void b_computeGrad_StoreHx(c_struct_T *obj, const double H[169],
                           const double f[14], const double x[14])
{
  int ixlast;
  int k;
  switch (obj->objtype) {
  case 5: {
    int i;
    i = obj->nvar;
    if (i - 2 >= 0) {
      memset(&obj->grad[0], 0, (unsigned int)(i - 1) * sizeof(double));
    }
    obj->grad[obj->nvar - 1] = obj->gammaScalar;
  } break;
  case 3: {
    int i;
    b_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    i = (unsigned char)obj->nvar;
    if (i - 1 >= 0) {
      memcpy(&obj->grad[0], &obj->Hx[0], (unsigned int)i * sizeof(double));
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      ixlast = obj->nvar - 1;
      for (k = 0; k <= ixlast; k++) {
        obj->grad[k] += f[k];
      }
    }
  } break;
  default: {
    int i;
    b_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    i = obj->nvar + 1;
    for (ixlast = i; ixlast < 14; ixlast++) {
      obj->Hx[ixlast - 1] = obj->beta * x[ixlast - 1];
    }
    memcpy(&obj->grad[0], &obj->Hx[0], 13U * sizeof(double));
    if (obj->hasLinear && (obj->nvar >= 1)) {
      ixlast = obj->nvar - 1;
      for (k = 0; k <= ixlast; k++) {
        obj->grad[k] += f[k];
      }
    }
    if (13 - obj->nvar >= 1) {
      ixlast = obj->nvar;
      i = 12 - obj->nvar;
      for (k = 0; k <= i; k++) {
        int i1;
        i1 = ixlast + k;
        obj->grad[i1] += obj->rho;
      }
    }
  } break;
  }
}

/*
 * Arguments    : struct_T *obj
 *                const double H[225]
 *                const double f[16]
 *                const double x[16]
 * Return Type  : void
 */
void computeGrad_StoreHx(struct_T *obj, const double H[225], const double f[16],
                         const double x[16])
{
  int ixlast;
  int k;
  switch (obj->objtype) {
  case 5: {
    int i;
    i = obj->nvar;
    if (i - 2 >= 0) {
      memset(&obj->grad[0], 0, (unsigned int)(i - 1) * sizeof(double));
    }
    obj->grad[obj->nvar - 1] = obj->gammaScalar;
  } break;
  case 3: {
    int i;
    xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    i = (unsigned char)obj->nvar;
    if (i - 1 >= 0) {
      memcpy(&obj->grad[0], &obj->Hx[0], (unsigned int)i * sizeof(double));
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      ixlast = obj->nvar - 1;
      for (k = 0; k <= ixlast; k++) {
        obj->grad[k] += f[k];
      }
    }
  } break;
  default: {
    int i;
    xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    i = obj->nvar + 1;
    for (ixlast = i; ixlast < 16; ixlast++) {
      obj->Hx[ixlast - 1] = obj->beta * x[ixlast - 1];
    }
    memcpy(&obj->grad[0], &obj->Hx[0], 15U * sizeof(double));
    if (obj->hasLinear && (obj->nvar >= 1)) {
      ixlast = obj->nvar - 1;
      for (k = 0; k <= ixlast; k++) {
        obj->grad[k] += f[k];
      }
    }
    if (15 - obj->nvar >= 1) {
      ixlast = obj->nvar;
      i = 14 - obj->nvar;
      for (k = 0; k <= i; k++) {
        int i1;
        i1 = ixlast + k;
        obj->grad[i1] += obj->rho;
      }
    }
  } break;
  }
}

/*
 * File trailer for computeGrad_StoreHx.c
 *
 * [EOF]
 */
