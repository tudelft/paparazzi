/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGrad_StoreHx.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

/* Include Files */
#include "computeGrad_StoreHx.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include <string.h>

/* Function Definitions */
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
