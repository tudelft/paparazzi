/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: updateWorkingSetForNewQP.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "updateWorkingSetForNewQP.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *WorkingSet
 *                const double cIneq_data[]
 *                const double cEq_data[]
 * Return Type  : void
 */
void updateWorkingSetForNewQP(f_struct_T *WorkingSet, const double cIneq_data[],
                              const double cEq_data[])
{
  int b_i;
  int i;
  int iEq0;
  int idx;
  int iw0;
  int nVar;
  nVar = WorkingSet->nVar;
  iw0 = 0;
  iEq0 = 0;
  i = nVar - 1;
  for (idx = 0; idx < 18; idx++) {
    WorkingSet->beq.data[idx] = -cEq_data[idx];
    WorkingSet->bwset.data[idx] = WorkingSet->beq.data[idx];
    for (b_i = 0; b_i <= i; b_i++) {
      WorkingSet->ATwset->data[iw0 + b_i] = WorkingSet->Aeq.data[iEq0 + b_i];
    }
    iEq0 = iw0 + 183;
    iw0 += 183;
  }
  for (idx = 0; idx < 128; idx++) {
    WorkingSet->bineq.data[idx] = -cIneq_data[idx];
  }
  if (WorkingSet->nActiveConstr > 18) {
    i = WorkingSet->nActiveConstr;
    for (idx = 19; idx <= i; idx++) {
      switch (WorkingSet->Wid.data[idx - 1]) {
      case 4:
        WorkingSet->bwset.data[idx - 1] =
            WorkingSet->lb
                .data[WorkingSet->indexLB
                          .data[WorkingSet->Wlocalidx.data[idx - 1] - 1] -
                      1];
        break;
      case 5:
        WorkingSet->bwset.data[idx - 1] =
            WorkingSet->ub
                .data[WorkingSet->indexUB
                          .data[WorkingSet->Wlocalidx.data[idx - 1] - 1] -
                      1];
        break;
      default:
        iw0 = WorkingSet->Wlocalidx.data[idx - 1];
        WorkingSet->bwset.data[idx - 1] = WorkingSet->bineq.data[iw0 - 1];
        if (iw0 >= 128) {
          iEq0 = 183 * (idx - 1);
          iw0 = (unsigned char)nVar;
          for (b_i = 0; b_i < iw0; b_i++) {
            WorkingSet->ATwset->data[iEq0 + b_i] =
                WorkingSet->Aineq->data[b_i + 23241];
          }
        }
        break;
      }
    }
  }
}

/*
 * File trailer for updateWorkingSetForNewQP.c
 *
 * [EOF]
 */
