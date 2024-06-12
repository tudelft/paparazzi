/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGrad_StoreHx.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "computeGrad_StoreHx.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : d_struct_T *obj
 *                const double H[324]
 *                const double f_data[]
 *                const double x_data[]
 * Return Type  : void
 */
void computeGrad_StoreHx(d_struct_T *obj, const double H[324],
                         const double f_data[], const double x_data[])
{
  double y_data[183];
  int ixlast;
  int k;
  switch (obj->objtype) {
  case 5: {
    int i;
    i = obj->nvar;
    if (i - 2 >= 0) {
      memset(&obj->grad.data[0], 0, (unsigned int)(i - 1) * sizeof(double));
    }
    obj->grad.data[obj->nvar - 1] = obj->gammaScalar;
  } break;
  case 3: {
    int i;
    xgemv(obj->nvar, obj->nvar, H, obj->nvar, x_data, obj->Hx.data);
    i = (unsigned char)obj->nvar;
    if (i - 1 >= 0) {
      memcpy(&obj->grad.data[0], &obj->Hx.data[0],
             (unsigned int)i * sizeof(double));
    }
    if (obj->hasLinear) {
      memcpy(&y_data[0], &obj->grad.data[0], 183U * sizeof(double));
      if (obj->nvar >= 1) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          y_data[k] += f_data[k];
        }
      }
      obj->grad.size[0] = 183;
      memcpy(&obj->grad.data[0], &y_data[0], 183U * sizeof(double));
    }
  } break;
  default: {
    int i;
    xgemv(obj->nvar, obj->nvar, H, obj->nvar, x_data, obj->Hx.data);
    i = obj->nvar + 1;
    for (ixlast = i; ixlast < 183; ixlast++) {
      obj->Hx.data[ixlast - 1] = obj->beta * x_data[ixlast - 1];
    }
    memcpy(&obj->grad.data[0], &obj->Hx.data[0], 182U * sizeof(double));
    if (obj->hasLinear) {
      memcpy(&y_data[0], &obj->grad.data[0], 183U * sizeof(double));
      if (obj->nvar >= 1) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          y_data[k] += f_data[k];
        }
      }
      obj->grad.size[0] = 183;
      memcpy(&obj->grad.data[0], &y_data[0], 183U * sizeof(double));
    }
    if (182 - obj->nvar >= 1) {
      ixlast = obj->nvar;
      i = 181 - obj->nvar;
      for (k = 0; k <= i; k++) {
        int i1;
        i1 = ixlast + k;
        obj->grad.data[i1] += obj->rho;
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
