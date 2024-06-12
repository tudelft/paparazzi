/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: relaxed.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "relaxed.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "xgemv.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double Hessian[324]
 *                const double grad_data[]
 *                c_struct_T *TrialState
 *                struct_T *MeritFunction
 *                e_struct_T *memspace
 *                f_struct_T *WorkingSet
 *                g_struct_T *QRManager
 *                h_struct_T *CholManager
 *                d_struct_T *QPObjective
 *                j_struct_T *qpoptions
 * Return Type  : void
 */
void relaxed(const double Hessian[324], const double grad_data[],
             c_struct_T *TrialState, struct_T *MeritFunction,
             e_struct_T *memspace, f_struct_T *WorkingSet,
             g_struct_T *QRManager, h_struct_T *CholManager,
             d_struct_T *QPObjective, j_struct_T *qpoptions)
{
  emxArray_real_T b_WorkingSet;
  j_struct_T b_qpoptions;
  j_struct_T c_qpoptions;
  double beta;
  double d;
  double s;
  double smax;
  int i;
  int idx;
  int idx_max;
  int mLBOrig;
  int nActiveLBArtificial;
  int nVarOrig;
  bool tf;
  nVarOrig = WorkingSet->nVar;
  beta = 0.0;
  i = (unsigned char)WorkingSet->nVar;
  for (idx = 0; idx < i; idx++) {
    beta += Hessian[idx + 18 * idx];
  }
  beta /= (double)WorkingSet->nVar;
  if (TrialState->sqpIterations <= 1) {
    mLBOrig = QPObjective->nvar;
    if (QPObjective->nvar < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (QPObjective->nvar > 1) {
        smax = fabs(grad_data[0]);
        for (idx = 2; idx <= mLBOrig; idx++) {
          s = fabs(grad_data[idx - 1]);
          if (s > smax) {
            idx_max = idx;
            smax = s;
          }
        }
      }
    }
    smax = 100.0 * fmax(1.0, fabs(grad_data[idx_max - 1]));
  } else {
    mLBOrig = WorkingSet->mConstr;
    if (WorkingSet->mConstr < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (WorkingSet->mConstr > 1) {
        smax = fabs(TrialState->lambdasqp.data[0]);
        for (idx = 2; idx <= mLBOrig; idx++) {
          s = fabs(TrialState->lambdasqp.data[idx - 1]);
          if (s > smax) {
            idx_max = idx;
            smax = s;
          }
        }
      }
    }
    smax = fabs(TrialState->lambdasqp.data[idx_max - 1]);
  }
  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = smax;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  setProblemType(WorkingSet, 2);
  mLBOrig = WorkingSet->sizes[3] - 35;
  for (idx = 0; idx < 128; idx++) {
    memspace->workspace_double->data[idx] = WorkingSet->bineq.data[idx];
  }
  f_xgemv(nVarOrig, 128, WorkingSet->Aineq, TrialState->xstar.data,
          memspace->workspace_double);
  for (idx = 0; idx < 128; idx++) {
    d = memspace->workspace_double->data[idx];
    TrialState->xstar.data[nVarOrig + idx] = (double)(d > 0.0) * d;
  }
  for (idx = 0; idx < 18; idx++) {
    memspace->workspace_double->data[idx] = WorkingSet->beq.data[idx];
  }
  b_WorkingSet.data = &WorkingSet->Aeq.data[0];
  b_WorkingSet.size = &WorkingSet->Aeq.size[0];
  b_WorkingSet.allocatedSize = 3294;
  b_WorkingSet.numDimensions = 1;
  b_WorkingSet.canFreeData = false;
  f_xgemv(nVarOrig, 18, &b_WorkingSet, TrialState->xstar.data,
          memspace->workspace_double);
  for (idx = 0; idx < 18; idx++) {
    d = memspace->workspace_double->data[idx];
    if (d <= 0.0) {
      i = nVarOrig + idx;
      TrialState->xstar.data[i + 128] = 0.0;
      TrialState->xstar.data[i + 146] = -d;
      i = mLBOrig + idx;
      addBoundToActiveSetMatrix_(WorkingSet, 4, i);
      if (d >= -0.001) {
        addBoundToActiveSetMatrix_(WorkingSet, 4, i + 18);
      }
    } else {
      i = nVarOrig + idx;
      TrialState->xstar.data[i + 128] = d;
      TrialState->xstar.data[i + 146] = 0.0;
      i = mLBOrig + idx;
      addBoundToActiveSetMatrix_(WorkingSet, 4, i + 18);
      if (d <= 0.001) {
        addBoundToActiveSetMatrix_(WorkingSet, 4, i);
      }
    }
  }
  mLBOrig = qpoptions->MaxIterations;
  qpoptions->MaxIterations =
      (qpoptions->MaxIterations + WorkingSet->nVar) - nVarOrig;
  b_qpoptions = *qpoptions;
  c_qpoptions = *qpoptions;
  b_driver(Hessian, grad_data, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, &c_qpoptions);
  qpoptions->MaxIterations = mLBOrig;
  idx_max = WorkingSet->sizes[3] - 165;
  nActiveLBArtificial = 0;
  for (idx = 0; idx < 18; idx++) {
    bool b_tf;
    mLBOrig = (WorkingSet->isActiveIdx[3] + idx_max) + idx;
    tf = WorkingSet->isActiveConstr.data[mLBOrig + 128];
    b_tf = WorkingSet->isActiveConstr.data[mLBOrig + 146];
    memspace->workspace_int.data[idx] = tf;
    memspace->workspace_int.data[idx + 18] = b_tf;
    nActiveLBArtificial = (nActiveLBArtificial + tf) + b_tf;
  }
  for (idx = 0; idx < 128; idx++) {
    tf = WorkingSet->isActiveConstr
             .data[(WorkingSet->isActiveIdx[3] + idx_max) + idx];
    memspace->workspace_int.data[idx + 36] = tf;
    nActiveLBArtificial += tf;
  }
  if (TrialState->state != -6) {
    double penaltyParamTrial;
    double qpfvalQuadExcess;
    idx_max = nVarOrig + 1;
    s = 0.0;
    qpfvalQuadExcess = 0.0;
    if (182 - nVarOrig >= 1) {
      for (idx = idx_max; idx < 183; idx++) {
        s += fabs(TrialState->xstar.data[idx - 1]);
      }
    }
    if (182 - nVarOrig >= 1) {
      i = (unsigned char)(182 - nVarOrig);
      for (idx = 0; idx < i; idx++) {
        d = TrialState->xstar.data[nVarOrig + idx];
        qpfvalQuadExcess += d * d;
      }
    }
    beta = (TrialState->fstar - smax * s) - beta / 2.0 * qpfvalQuadExcess;
    penaltyParamTrial = MeritFunction->penaltyParam;
    smax = 0.0;
    for (idx = 0; idx < 18; idx++) {
      smax += fabs(TrialState->cEq.data[idx]);
    }
    s = 0.0;
    for (idx = 0; idx < 128; idx++) {
      d = TrialState->cIneq.data[idx];
      if (d > 0.0) {
        s += d;
      }
    }
    qpfvalQuadExcess = smax + s;
    smax = MeritFunction->linearizedConstrViol;
    s = 0.0;
    if (182 - nVarOrig >= 1) {
      for (idx = idx_max; idx < 183; idx++) {
        s += fabs(TrialState->xstar.data[idx - 1]);
      }
    }
    MeritFunction->linearizedConstrViol = s;
    smax = (qpfvalQuadExcess + smax) - s;
    if ((smax > 2.2204460492503131E-16) && (beta > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        d = 1.0;
      } else {
        d = 1.5;
      }
      penaltyParamTrial = d * beta / smax;
    }
    if (penaltyParamTrial < MeritFunction->penaltyParam) {
      MeritFunction->phi =
          TrialState->sqpFval + penaltyParamTrial * qpfvalQuadExcess;
      if ((MeritFunction->initFval +
           penaltyParamTrial * (MeritFunction->initConstrViolationEq +
                                MeritFunction->initConstrViolationIneq)) -
              MeritFunction->phi >
          (double)MeritFunction->nPenaltyDecreases * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) >
            TrialState->sqpIterations) {
          MeritFunction->threshold *= 10.0;
        }
        MeritFunction->penaltyParam = fmax(penaltyParamTrial, 1.0E-10);
      } else {
        MeritFunction->phi = TrialState->sqpFval +
                             MeritFunction->penaltyParam * qpfvalQuadExcess;
      }
    } else {
      MeritFunction->penaltyParam = fmax(penaltyParamTrial, 1.0E-10);
      MeritFunction->phi =
          TrialState->sqpFval + MeritFunction->penaltyParam * qpfvalQuadExcess;
    }
    MeritFunction->phiPrimePlus =
        fmin(beta - MeritFunction->penaltyParam * qpfvalQuadExcess, 0.0);
    idx_max = WorkingSet->isActiveIdx[1] - 1;
    for (idx = 0; idx < 18; idx++) {
      if ((memspace->workspace_int.data[idx] != 0) &&
          (memspace->workspace_int.data[idx + 18] != 0)) {
        tf = true;
      } else {
        tf = false;
      }
      i = idx_max + idx;
      TrialState->lambda.data[i] *= (double)tf;
    }
    idx_max = WorkingSet->isActiveIdx[2];
    mLBOrig = WorkingSet->nActiveConstr;
    for (idx = idx_max; idx <= mLBOrig; idx++) {
      if (WorkingSet->Wid.data[idx - 1] == 3) {
        TrialState->lambda.data[idx - 1] *=
            (double)memspace->workspace_int
                .data[WorkingSet->Wlocalidx.data[idx - 1] + 35];
      }
    }
  }
  idx_max = WorkingSet->sizes[3] - 164;
  idx = WorkingSet->nActiveConstr;
  while ((idx > 18) && (nActiveLBArtificial > 0)) {
    if ((WorkingSet->Wid.data[idx - 1] == 4) &&
        (WorkingSet->Wlocalidx.data[idx - 1] > idx_max)) {
      mLBOrig = WorkingSet->nActiveConstr - 1;
      smax = TrialState->lambda.data[mLBOrig];
      TrialState->lambda.data[mLBOrig] = 0.0;
      TrialState->lambda.data[idx - 1] = smax;
      removeConstr(WorkingSet, idx);
      nActiveLBArtificial--;
    }
    idx--;
  }
  QPObjective->nvar = nVarOrig;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  setProblemType(WorkingSet, 3);
  sortLambdaQP(TrialState->lambda.data, &TrialState->lambda.size[0],
               WorkingSet->nActiveConstr, WorkingSet->sizes,
               WorkingSet->isActiveIdx, WorkingSet->Wid.data,
               WorkingSet->Wlocalidx.data, memspace->workspace_double);
}

/*
 * File trailer for relaxed.c
 *
 * [EOF]
 */
