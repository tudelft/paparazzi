/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: addBoundToActiveSetMatrix_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "addBoundToActiveSetMatrix_.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 *                int TYPE
 *                int idx_local
 * Return Type  : void
 */
void addBoundToActiveSetMatrix_(f_struct_T *obj, int TYPE, int idx_local)
{
  int colOffset;
  int i;
  int idx;
  int idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  i = obj->nActiveConstr - 1;
  obj->Wid.data[i] = TYPE;
  obj->Wlocalidx.data[i] = idx_local;
  colOffset = 183 * i - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB.data[idx_local - 1];
    obj->bwset.data[i] = obj->ub.data[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB.data[idx_local - 1];
    obj->bwset.data[i] = obj->lb.data[idx_bnd_local - 1];
  }
  for (idx = 0; idx <= idx_bnd_local - 2; idx++) {
    obj->ATwset->data[(idx + colOffset) + 1] = 0.0;
  }
  obj->ATwset->data[idx_bnd_local + colOffset] =
      2.0 * (double)(TYPE == 5) - 1.0;
  i = idx_bnd_local + 1;
  idx_bnd_local = obj->nVar;
  for (idx = i; idx <= idx_bnd_local; idx++) {
    obj->ATwset->data[idx + colOffset] = 0.0;
  }
  switch (obj->probType) {
  case 3:
  case 2:
    break;
  default:
    obj->ATwset->data[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/*
 * File trailer for addBoundToActiveSetMatrix_.c
 *
 * [EOF]
 */
