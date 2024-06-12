/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: PresolveWorkingSet.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "PresolveWorkingSet.h"
#include "RemoveDependentEq_.h"
#include "RemoveDependentIneq_.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : c_struct_T *solution
 *                e_struct_T *memspace
 *                f_struct_T *workingset
 *                g_struct_T *qrmanager
 * Return Type  : void
 */
void PresolveWorkingSet(c_struct_T *solution, e_struct_T *memspace,
                        f_struct_T *workingset, g_struct_T *qrmanager)
{
  int idxStartIneq_tmp;
  int idx_global;
  solution->state = 82;
  idxStartIneq_tmp = RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((idxStartIneq_tmp != -1) && (workingset->nActiveConstr <= 311)) {
    bool guard1;
    bool okWorkingSet;
    RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet =
        feasibleX0ForWorkingSet(memspace->workspace_double,
                                solution->xstar.data, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      b_RemoveDependentIneq_(workingset, qrmanager, memspace);
      okWorkingSet =
          feasibleX0ForWorkingSet(memspace->workspace_double,
                                  solution->xstar.data, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      double constrViolation;
      constrViolation =
          b_maxConstraintViolation(workingset, solution->xstar.data);
      if (constrViolation > 0.001) {
        solution->state = -2;
      }
    }
  } else {
    int idxEndIneq;
    int idxStartIneq;
    solution->state = -3;
    idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
    idxStartIneq = idxStartIneq_tmp + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
      workingset->isActiveConstr.data
          [(workingset->isActiveIdx[workingset->Wid.data[idx_global - 1] - 1] +
            workingset->Wlocalidx.data[idx_global - 1]) -
           2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = idxStartIneq_tmp;
  }
}

/*
 * File trailer for PresolveWorkingSet.c
 *
 * [EOF]
 */
