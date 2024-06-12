/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: addAeqConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "addAeqConstr.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 *                int idx_local
 * Return Type  : void
 */
void addAeqConstr(f_struct_T *obj, int idx_local)
{
  int idx;
  int totalEq;
  totalEq = obj->nWConstr[0] + obj->nWConstr[1];
  if ((obj->nActiveConstr == totalEq) && (idx_local > obj->nWConstr[1])) {
    int i;
    int i1;
    int iAeq0;
    int iAw0;
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->nActiveConstr++;
    i = obj->nActiveConstr - 1;
    obj->Wid.data[i] = 2;
    obj->Wlocalidx.data[i] = idx_local;
    iAeq0 = 183 * (idx_local - 1);
    iAw0 = 183 * (obj->nActiveConstr - 1);
    i1 = (unsigned char)obj->nVar;
    for (idx = 0; idx < i1; idx++) {
      obj->ATwset->data[iAw0 + idx] = obj->Aeq.data[iAeq0 + idx];
    }
    obj->bwset.data[i] = obj->beq.data[idx_local - 1];
  } else {
    int i;
    int i1;
    int iAeq0;
    int iAw0;
    obj->nActiveConstr++;
    i = obj->nActiveConstr - 1;
    obj->Wid.data[i] = obj->Wid.data[totalEq];
    obj->Wlocalidx.data[i] = obj->Wlocalidx.data[totalEq];
    i1 = (unsigned char)obj->nVar;
    for (idx = 0; idx < i1; idx++) {
      obj->ATwset->data[idx + 183 * i] = obj->ATwset->data[idx + 183 * totalEq];
    }
    obj->bwset.data[i] = obj->bwset.data[totalEq];
    obj->nWConstr[1]++;
    obj->isActiveConstr.data[(obj->isActiveIdx[1] + idx_local) - 2] = true;
    obj->Wid.data[totalEq] = 2;
    obj->Wlocalidx.data[totalEq] = idx_local;
    iAeq0 = 183 * (idx_local - 1);
    iAw0 = 183 * totalEq;
    for (idx = 0; idx < i1; idx++) {
      obj->ATwset->data[iAw0 + idx] = obj->Aeq.data[iAeq0 + idx];
    }
    obj->bwset.data[totalEq] = obj->beq.data[idx_local - 1];
  }
}

/*
 * File trailer for addAeqConstr.c
 *
 * [EOF]
 */
