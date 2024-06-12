/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: saveJacobian.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "saveJacobian.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : c_struct_T *obj
 *                int nVar
 *                const emxArray_real_T *JacCineqTrans
 *                const double JacCeqTrans_data[]
 * Return Type  : void
 */
void saveJacobian(c_struct_T *obj, int nVar,
                  const emxArray_real_T *JacCineqTrans,
                  const double JacCeqTrans_data[])
{
  emxArray_real_T *y;
  const double *JacCineqTrans_data;
  double *y_data;
  int i;
  int i1;
  int iCol;
  int idx_col;
  int k;
  int loop_ub_tmp;
  JacCineqTrans_data = JacCineqTrans->data;
  iCol = -1;
  i = (unsigned char)nVar;
  emxInit_real_T(&y, 2);
  for (idx_col = 0; idx_col < 128; idx_col++) {
    i1 = y->size[0] * y->size[1];
    y->size[0] = 183;
    loop_ub_tmp = obj->JacCineqTrans_old->size[1];
    y->size[1] = loop_ub_tmp;
    emxEnsureCapacity_real_T(y, i1);
    y_data = y->data;
    loop_ub_tmp *= 183;
    for (i1 = 0; i1 < loop_ub_tmp; i1++) {
      y_data[i1] = obj->JacCineqTrans_old->data[i1];
    }
    for (k = 0; k < i; k++) {
      i1 = iCol + k;
      y_data[i1 + 1] = JacCineqTrans_data[i1 + 1];
    }
    i1 = obj->JacCineqTrans_old->size[0] * obj->JacCineqTrans_old->size[1];
    obj->JacCineqTrans_old->size[0] = 183;
    obj->JacCineqTrans_old->size[1] = y->size[1];
    emxEnsureCapacity_real_T(obj->JacCineqTrans_old, i1);
    for (i1 = 0; i1 < loop_ub_tmp; i1++) {
      obj->JacCineqTrans_old->data[i1] = y_data[i1];
    }
    iCol += 183;
  }
  iCol = -1;
  for (idx_col = 0; idx_col < 18; idx_col++) {
    i1 = y->size[0] * y->size[1];
    y->size[0] = 183;
    loop_ub_tmp = obj->JacCeqTrans_old->size[1];
    y->size[1] = loop_ub_tmp;
    emxEnsureCapacity_real_T(y, i1);
    y_data = y->data;
    loop_ub_tmp *= 183;
    for (i1 = 0; i1 < loop_ub_tmp; i1++) {
      y_data[i1] = obj->JacCeqTrans_old->data[i1];
    }
    for (k = 0; k < i; k++) {
      i1 = iCol + k;
      y_data[i1 + 1] = JacCeqTrans_data[i1 + 1];
    }
    i1 = obj->JacCeqTrans_old->size[0] * obj->JacCeqTrans_old->size[1];
    obj->JacCeqTrans_old->size[0] = 183;
    obj->JacCeqTrans_old->size[1] = y->size[1];
    emxEnsureCapacity_real_T(obj->JacCeqTrans_old, i1);
    for (i1 = 0; i1 < loop_ub_tmp; i1++) {
      obj->JacCeqTrans_old->data[i1] = y_data[i1];
    }
    iCol += 183;
  }
  emxFree_real_T(&y);
}

/*
 * File trailer for saveJacobian.c
 *
 * [EOF]
 */
