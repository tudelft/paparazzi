/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: soc.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "soc.h"
#include "addAeqConstr.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double Hessian[324]
 *                const double grad_data[]
 *                c_struct_T *TrialState
 *                e_struct_T *memspace
 *                f_struct_T *WorkingSet
 *                g_struct_T *QRManager
 *                h_struct_T *CholManager
 *                d_struct_T *QPObjective
 *                const j_struct_T *qpoptions
 * Return Type  : bool
 */
bool soc(const double Hessian[324], const double grad_data[],
         c_struct_T *TrialState, e_struct_T *memspace, f_struct_T *WorkingSet,
         g_struct_T *QRManager, h_struct_T *CholManager,
         d_struct_T *QPObjective, const j_struct_T *qpoptions)
{
  j_struct_T b_qpoptions;
  j_struct_T c_qpoptions;
  double y_data[128];
  double c;
  int i;
  int i1;
  int idx;
  int idxIneqOffset;
  int idx_Aineq;
  int idx_Partition;
  int idx_lower;
  int idx_upper;
  int nVar;
  int nWIneq_old;
  int nWLower_old;
  int nWUpper_old;
  bool success;
  nWIneq_old = WorkingSet->nWConstr[2];
  nWLower_old = WorkingSet->nWConstr[3];
  nWUpper_old = WorkingSet->nWConstr[4];
  nVar = WorkingSet->nVar;
  i = (unsigned char)WorkingSet->nVar;
  for (idx_Aineq = 0; idx_Aineq < i; idx_Aineq++) {
    TrialState->xstarsqp[idx_Aineq] = TrialState->xstarsqp_old[idx_Aineq];
    TrialState->socDirection.data[idx_Aineq] =
        TrialState->xstar.data[idx_Aineq];
  }
  TrialState->lambdaStopTest.size[0] = 311;
  memcpy(&TrialState->lambdaStopTest.data[0], &TrialState->lambda.data[0],
         311U * sizeof(double));
  idxIneqOffset = WorkingSet->isActiveIdx[2];
  for (idx = 0; idx < 18; idx++) {
    c = -TrialState->cEq.data[idx];
    WorkingSet->beq.data[idx] = c;
    y_data[idx] = c;
  }
  for (idx_lower = 0; idx_lower <= 3111; idx_lower += 183) {
    c = 0.0;
    i1 = idx_lower + WorkingSet->nVar;
    for (idx_Aineq = idx_lower + 1; idx_Aineq <= i1; idx_Aineq++) {
      c += WorkingSet->Aeq.data[idx_Aineq - 1] *
           TrialState->searchDir.data[(idx_Aineq - idx_lower) - 1];
    }
    idx_Aineq = div_nde_s32_floor(idx_lower, 183);
    y_data[idx_Aineq] += c;
  }
  WorkingSet->beq.size[0] = 18;
  memcpy(&WorkingSet->beq.data[0], &y_data[0], 18U * sizeof(double));
  memcpy(&WorkingSet->bwset.data[0], &y_data[0], 18U * sizeof(double));
  for (idx = 0; idx < 128; idx++) {
    c = -TrialState->cIneq.data[idx];
    WorkingSet->bineq.data[idx] = c;
    y_data[idx] = c;
  }
  for (idx_lower = 0; idx_lower <= 23241; idx_lower += 183) {
    c = 0.0;
    i1 = idx_lower + WorkingSet->nVar;
    for (idx_Aineq = idx_lower + 1; idx_Aineq <= i1; idx_Aineq++) {
      c += WorkingSet->Aineq->data[idx_Aineq - 1] *
           TrialState->searchDir.data[(idx_Aineq - idx_lower) - 1];
    }
    idx_Aineq = div_nde_s32_floor(idx_lower, 183);
    y_data[idx_Aineq] += c;
  }
  WorkingSet->bineq.size[0] = 128;
  memcpy(&WorkingSet->bineq.data[0], &y_data[0], 128U * sizeof(double));
  idx_Aineq = 1;
  idx_lower = 129;
  idx_upper = WorkingSet->sizes[3] + 129;
  i1 = WorkingSet->nActiveConstr;
  for (idx = idxIneqOffset; idx <= i1; idx++) {
    switch (WorkingSet->Wid.data[idx - 1]) {
    case 3:
      idx_Partition = idx_Aineq;
      idx_Aineq++;
      WorkingSet->bwset.data[idx - 1] =
          y_data[WorkingSet->Wlocalidx.data[idx - 1] - 1];
      break;
    case 4:
      idx_Partition = idx_lower;
      idx_lower++;
      break;
    default:
      idx_Partition = idx_upper;
      idx_upper++;
      break;
    }
    TrialState->workingset_old.data[idx_Partition - 1] =
        WorkingSet->Wlocalidx.data[idx - 1];
  }
  memcpy(&TrialState->xstar.data[0], &TrialState->xstarsqp[0],
         (unsigned int)i * sizeof(double));
  b_qpoptions = *qpoptions;
  c_qpoptions = *qpoptions;
  b_driver(Hessian, grad_data, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, &c_qpoptions);
  while ((WorkingSet->mEqRemoved > 0) &&
         (WorkingSet->indexEqRemoved.data[WorkingSet->mEqRemoved - 1] >= 1)) {
    addAeqConstr(WorkingSet,
                 WorkingSet->indexEqRemoved.data[WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }
  i = (unsigned char)nVar;
  for (idx = 0; idx < i; idx++) {
    double oldDirIdx;
    c = TrialState->socDirection.data[idx];
    oldDirIdx = c;
    c = TrialState->xstar.data[idx] - c;
    TrialState->socDirection.data[idx] = c;
    TrialState->xstar.data[idx] = oldDirIdx;
  }
  success = (b_xnrm2(nVar, TrialState->socDirection.data) <=
             2.0 * b_xnrm2(nVar, TrialState->xstar.data));
  idx_Partition = WorkingSet->sizes[3];
  for (idx = 0; idx < 18; idx++) {
    WorkingSet->beq.data[idx] = -TrialState->cEq.data[idx];
    WorkingSet->bwset.data[idx] = WorkingSet->beq.data[idx];
  }
  for (idx = 0; idx < 128; idx++) {
    WorkingSet->bineq.data[idx] = -TrialState->cIneq.data[idx];
  }
  if (!success) {
    idx_Aineq = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
    idx_lower = idx_Aineq + 1;
    i = WorkingSet->nActiveConstr;
    for (idx_upper = idx_lower; idx_upper <= i; idx_upper++) {
      WorkingSet->isActiveConstr.data
          [(WorkingSet->isActiveIdx[WorkingSet->Wid.data[idx_upper - 1] - 1] +
            WorkingSet->Wlocalidx.data[idx_upper - 1]) -
           2] = false;
    }
    WorkingSet->nWConstr[2] = 0;
    WorkingSet->nWConstr[3] = 0;
    WorkingSet->nWConstr[4] = 0;
    WorkingSet->nActiveConstr = idx_Aineq;
    for (idx = 0; idx < nWIneq_old; idx++) {
      idx_Aineq = TrialState->workingset_old.data[idx];
      WorkingSet->nWConstr[2]++;
      WorkingSet->isActiveConstr
          .data[(WorkingSet->isActiveIdx[2] + idx_Aineq) - 2] = true;
      WorkingSet->nActiveConstr++;
      i = WorkingSet->nActiveConstr - 1;
      WorkingSet->Wid.data[i] = 3;
      WorkingSet->Wlocalidx.data[i] = idx_Aineq;
      idx_lower = 183 * (idx_Aineq - 1);
      idx_upper = 183 * i;
      i1 = WorkingSet->nVar - 1;
      for (idxIneqOffset = 0; idxIneqOffset <= i1; idxIneqOffset++) {
        WorkingSet->ATwset->data[idx_upper + idxIneqOffset] =
            WorkingSet->Aineq->data[idx_lower + idxIneqOffset];
      }
      WorkingSet->bwset.data[i] = WorkingSet->bineq.data[idx_Aineq - 1];
    }
    for (idx = 0; idx < nWLower_old; idx++) {
      addBoundToActiveSetMatrix_(WorkingSet, 4,
                                 TrialState->workingset_old.data[idx + 128]);
    }
    for (idx = 0; idx < nWUpper_old; idx++) {
      addBoundToActiveSetMatrix_(
          WorkingSet, 5,
          TrialState->workingset_old.data[(idx + idx_Partition) + 128]);
    }
    TrialState->lambda.size[0] = 311;
    memcpy(&TrialState->lambda.data[0], &TrialState->lambdaStopTest.data[0],
           311U * sizeof(double));
  } else {
    sortLambdaQP(TrialState->lambda.data, &TrialState->lambda.size[0],
                 WorkingSet->nActiveConstr, WorkingSet->sizes,
                 WorkingSet->isActiveIdx, WorkingSet->Wid.data,
                 WorkingSet->Wlocalidx.data, memspace->workspace_double);
  }
  return success;
}

/*
 * File trailer for soc.c
 *
 * [EOF]
 */
