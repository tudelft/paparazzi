/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: addAineqConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "addAineqConstr.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 *                int idx_local
 * Return Type  : void
 */
void addAineqConstr(f_struct_T *obj, int idx_local)
{
  int i;
  int i1;
  int iAineq0;
  int iAw0;
  int idx;
  obj->nWConstr[2]++;
  obj->isActiveConstr.data[(obj->isActiveIdx[2] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  i = obj->nActiveConstr - 1;
  obj->Wid.data[i] = 3;
  obj->Wlocalidx.data[i] = idx_local;
  iAineq0 = 183 * (idx_local - 1);
  iAw0 = 183 * i;
  i1 = obj->nVar - 1;
  for (idx = 0; idx <= i1; idx++) {
    obj->ATwset->data[iAw0 + idx] = obj->Aineq->data[iAineq0 + idx];
  }
  obj->bwset.data[i] = obj->bineq.data[idx_local - 1];
}

/*
 * File trailer for addAineqConstr.c
 *
 * [EOF]
 */
