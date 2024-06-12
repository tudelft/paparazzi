/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: maxConstraintViolation.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "maxConstraintViolation.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 *                const double x_data[]
 * Return Type  : double
 */
double b_maxConstraintViolation(f_struct_T *obj, const double x_data[])
{
  emxArray_real_T b_obj;
  double v;
  int i;
  int idx;
  int idxLB;
  if (obj->probType == 2) {
    double y_data[311];
    double d;
    v = 0.0;
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 128; idxLB++) {
      y_data[idxLB] = obj->bineq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    e_xgemv(18, 128, obj->Aineq, x_data, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 128; idx++) {
      d = obj->maxConstrWorkspace.data[idx] - x_data[idx + 18];
      obj->maxConstrWorkspace.data[idx] = d;
      v = fmax(v, d);
    }
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 18; idxLB++) {
      y_data[idxLB] = obj->beq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    b_obj.data = &obj->Aeq.data[0];
    b_obj.size = &obj->Aeq.size[0];
    b_obj.allocatedSize = 3294;
    b_obj.numDimensions = 1;
    b_obj.canFreeData = false;
    e_xgemv(18, 18, &b_obj, x_data, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 18; idx++) {
      d = (obj->maxConstrWorkspace.data[idx] - x_data[idx + 146]) +
          x_data[idx + 164];
      obj->maxConstrWorkspace.data[idx] = d;
      v = fmax(v, fabs(d));
    }
  } else {
    double y_data[311];
    v = 0.0;
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 128; idxLB++) {
      y_data[idxLB] = obj->bineq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    e_xgemv(obj->nVar, 128, obj->Aineq, x_data, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 128; idx++) {
      v = fmax(v, obj->maxConstrWorkspace.data[idx]);
    }
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 18; idxLB++) {
      y_data[idxLB] = obj->beq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    b_obj.data = &obj->Aeq.data[0];
    b_obj.size = &obj->Aeq.size[0];
    b_obj.allocatedSize = 3294;
    b_obj.numDimensions = 1;
    b_obj.canFreeData = false;
    e_xgemv(obj->nVar, 18, &b_obj, x_data, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 18; idx++) {
      v = fmax(v, fabs(obj->maxConstrWorkspace.data[idx]));
    }
  }
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB.data[idx] - 1;
      v = fmax(v, -x_data[idxLB] - obj->lb.data[idxLB]);
    }
  }
  return v;
}

/*
 * Arguments    : f_struct_T *obj
 *                const emxArray_real_T *x
 * Return Type  : double
 */
double c_maxConstraintViolation(f_struct_T *obj, const emxArray_real_T *x)
{
  emxArray_real_T b_obj;
  const double *x_data;
  double v;
  int i;
  int idx;
  int idxLB;
  x_data = x->data;
  if (obj->probType == 2) {
    double y_data[311];
    double d;
    v = 0.0;
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 128; idxLB++) {
      y_data[idxLB] = obj->bineq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    c_xgemv(18, 128, obj->Aineq, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 128; idx++) {
      d = obj->maxConstrWorkspace.data[idx] - x_data[idx + 329];
      obj->maxConstrWorkspace.data[idx] = d;
      v = fmax(v, d);
    }
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 18; idxLB++) {
      y_data[idxLB] = obj->beq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    b_obj.data = &obj->Aeq.data[0];
    b_obj.size = &obj->Aeq.size[0];
    b_obj.allocatedSize = 3294;
    b_obj.numDimensions = 1;
    b_obj.canFreeData = false;
    c_xgemv(18, 18, &b_obj, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 18; idx++) {
      d = (obj->maxConstrWorkspace.data[idx] - x_data[idx + 457]) +
          x_data[idx + 475];
      obj->maxConstrWorkspace.data[idx] = d;
      v = fmax(v, fabs(d));
    }
  } else {
    double y_data[311];
    v = 0.0;
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 128; idxLB++) {
      y_data[idxLB] = obj->bineq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    c_xgemv(obj->nVar, 128, obj->Aineq, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 128; idx++) {
      v = fmax(v, obj->maxConstrWorkspace.data[idx]);
    }
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 18; idxLB++) {
      y_data[idxLB] = obj->beq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    b_obj.data = &obj->Aeq.data[0];
    b_obj.size = &obj->Aeq.size[0];
    b_obj.allocatedSize = 3294;
    b_obj.numDimensions = 1;
    b_obj.canFreeData = false;
    c_xgemv(obj->nVar, 18, &b_obj, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 18; idx++) {
      v = fmax(v, fabs(obj->maxConstrWorkspace.data[idx]));
    }
  }
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB.data[idx];
      v = fmax(v, -x_data[idxLB + 310] - obj->lb.data[idxLB - 1]);
    }
  }
  return v;
}

/*
 * Arguments    : f_struct_T *obj
 *                const emxArray_real_T *x
 * Return Type  : double
 */
double maxConstraintViolation(f_struct_T *obj, const emxArray_real_T *x)
{
  emxArray_real_T b_obj;
  const double *x_data;
  double v;
  int i;
  int idx;
  int idxLB;
  x_data = x->data;
  if (obj->probType == 2) {
    double y_data[311];
    double d;
    v = 0.0;
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 128; idxLB++) {
      y_data[idxLB] = obj->bineq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    d_xgemv(18, 128, obj->Aineq, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 128; idx++) {
      d = obj->maxConstrWorkspace.data[idx] - x_data[idx + 18];
      obj->maxConstrWorkspace.data[idx] = d;
      v = fmax(v, d);
    }
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 18; idxLB++) {
      y_data[idxLB] = obj->beq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    b_obj.data = &obj->Aeq.data[0];
    b_obj.size = &obj->Aeq.size[0];
    b_obj.allocatedSize = 3294;
    b_obj.numDimensions = 1;
    b_obj.canFreeData = false;
    d_xgemv(18, 18, &b_obj, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 18; idx++) {
      d = (obj->maxConstrWorkspace.data[idx] - x_data[idx + 146]) +
          x_data[idx + 164];
      obj->maxConstrWorkspace.data[idx] = d;
      v = fmax(v, fabs(d));
    }
  } else {
    double y_data[311];
    v = 0.0;
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 128; idxLB++) {
      y_data[idxLB] = obj->bineq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    d_xgemv(obj->nVar, 128, obj->Aineq, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 128; idx++) {
      v = fmax(v, obj->maxConstrWorkspace.data[idx]);
    }
    for (i = 0; i < 311; i++) {
      y_data[i] = obj->maxConstrWorkspace.data[i];
    }
    for (idxLB = 0; idxLB < 18; idxLB++) {
      y_data[idxLB] = obj->beq.data[idxLB];
    }
    obj->maxConstrWorkspace.size[0] = 311;
    for (i = 0; i < 311; i++) {
      obj->maxConstrWorkspace.data[i] = y_data[i];
    }
    b_obj.data = &obj->Aeq.data[0];
    b_obj.size = &obj->Aeq.size[0];
    b_obj.allocatedSize = 3294;
    b_obj.numDimensions = 1;
    b_obj.canFreeData = false;
    d_xgemv(obj->nVar, 18, &b_obj, x, obj->maxConstrWorkspace.data);
    for (idx = 0; idx < 18; idx++) {
      v = fmax(v, fabs(obj->maxConstrWorkspace.data[idx]));
    }
  }
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB.data[idx] - 1;
      v = fmax(v, -x_data[idxLB] - obj->lb.data[idxLB]);
    }
  }
  return v;
}

/*
 * File trailer for maxConstraintViolation.c
 *
 * [EOF]
 */
