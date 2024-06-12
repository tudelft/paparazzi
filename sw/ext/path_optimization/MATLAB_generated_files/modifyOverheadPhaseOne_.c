/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: modifyOverheadPhaseOne_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "modifyOverheadPhaseOne_.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 * Return Type  : void
 */
void modifyOverheadPhaseOne_(f_struct_T *obj)
{
  int i;
  int idx;
  int idxEq;
  for (idx = 0; idx < 18; idx++) {
    idxEq = (obj->nVar + 183 * idx) - 1;
    obj->Aeq.data[idxEq] = 0.0;
    obj->ATwset->data[idxEq + 183 * (obj->isActiveIdx[1] - 1)] = 0.0;
  }
  for (idx = 0; idx < 128; idx++) {
    obj->Aineq->data[(obj->nVar + 183 * idx) - 1] = -1.0;
  }
  obj->indexLB.data[obj->sizes[3] - 1] = obj->nVar;
  obj->lb.data[obj->nVar - 1] = 1.0E-5;
  idxEq = obj->isActiveIdx[2];
  i = obj->nActiveConstr;
  for (idx = idxEq; idx <= i; idx++) {
    obj->ATwset->data[(obj->nVar + 183 * (idx - 1)) - 1] = -1.0;
  }
  if (obj->nWConstr[4] > 0) {
    obj->isActiveConstr.data[obj->isActiveIdx[4] - 1] = false;
  }
  obj->isActiveConstr.data[obj->isActiveIdx[4] - 2] = false;
}

/*
 * File trailer for modifyOverheadPhaseOne_.c
 *
 * [EOF]
 */
