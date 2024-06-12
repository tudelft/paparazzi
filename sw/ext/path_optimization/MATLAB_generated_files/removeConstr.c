/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: removeConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "removeConstr.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 *                int idx_global
 * Return Type  : void
 */
void removeConstr(f_struct_T *obj, int idx_global)
{
  int TYPE_tmp;
  int i;
  int i1;
  int idx;
  TYPE_tmp = obj->Wid.data[idx_global - 1] - 1;
  obj->isActiveConstr
      .data[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx.data[idx_global - 1]) -
            2] = false;
  i = obj->nActiveConstr - 1;
  obj->Wid.data[idx_global - 1] = obj->Wid.data[i];
  obj->Wlocalidx.data[idx_global - 1] = obj->Wlocalidx.data[i];
  i1 = (unsigned char)obj->nVar;
  for (idx = 0; idx < i1; idx++) {
    obj->ATwset->data[idx + 183 * (idx_global - 1)] =
        obj->ATwset->data[idx + 183 * i];
  }
  obj->bwset.data[idx_global - 1] = obj->bwset.data[i];
  obj->nActiveConstr = i;
  obj->nWConstr[TYPE_tmp]--;
}

/*
 * File trailer for removeConstr.c
 *
 * [EOF]
 */
