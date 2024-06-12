/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: normal.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "normal.h"
#include "addAeqConstr.h"
#include "driver1.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
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
 *                const j_struct_T *qpoptions
 * Return Type  : void
 */
void normal(const double Hessian[324], const double grad_data[],
            c_struct_T *TrialState, struct_T *MeritFunction,
            e_struct_T *memspace, f_struct_T *WorkingSet, g_struct_T *QRManager,
            h_struct_T *CholManager, d_struct_T *QPObjective,
            const j_struct_T *qpoptions)
{
  j_struct_T b_qpoptions;
  j_struct_T c_qpoptions;
  int k;
  bool nonlinEqRemoved;
  b_qpoptions = *qpoptions;
  c_qpoptions = *qpoptions;
  b_driver(Hessian, grad_data, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, &c_qpoptions);
  if (TrialState->state > 0) {
    double constrViolationEq;
    double constrViolationIneq;
    double d;
    double penaltyParamTrial;
    penaltyParamTrial = MeritFunction->penaltyParam;
    constrViolationEq = 0.0;
    for (k = 0; k < 18; k++) {
      constrViolationEq += fabs(TrialState->cEq.data[k]);
    }
    constrViolationIneq = 0.0;
    for (k = 0; k < 128; k++) {
      d = TrialState->cIneq.data[k];
      if (d > 0.0) {
        constrViolationIneq += d;
      }
    }
    constrViolationIneq += constrViolationEq;
    constrViolationEq = MeritFunction->linearizedConstrViol;
    MeritFunction->linearizedConstrViol = 0.0;
    constrViolationEq += constrViolationIneq;
    if ((constrViolationEq > 2.2204460492503131E-16) &&
        (TrialState->fstar > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        d = 1.0;
      } else {
        d = 1.5;
      }
      penaltyParamTrial = d * TrialState->fstar / constrViolationEq;
    }
    if (penaltyParamTrial < MeritFunction->penaltyParam) {
      MeritFunction->phi =
          TrialState->sqpFval + penaltyParamTrial * constrViolationIneq;
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
                             MeritFunction->penaltyParam * constrViolationIneq;
      }
    } else {
      MeritFunction->penaltyParam = fmax(penaltyParamTrial, 1.0E-10);
      MeritFunction->phi = TrialState->sqpFval +
                           MeritFunction->penaltyParam * constrViolationIneq;
    }
    MeritFunction->phiPrimePlus = fmin(
        TrialState->fstar - MeritFunction->penaltyParam * constrViolationIneq,
        0.0);
  }
  sortLambdaQP(TrialState->lambda.data, &TrialState->lambda.size[0],
               WorkingSet->nActiveConstr, WorkingSet->sizes,
               WorkingSet->isActiveIdx, WorkingSet->Wid.data,
               WorkingSet->Wlocalidx.data, memspace->workspace_double);
  nonlinEqRemoved = (WorkingSet->mEqRemoved > 0);
  while ((WorkingSet->mEqRemoved > 0) &&
         (WorkingSet->indexEqRemoved.data[WorkingSet->mEqRemoved - 1] >= 1)) {
    addAeqConstr(WorkingSet,
                 WorkingSet->indexEqRemoved.data[WorkingSet->mEqRemoved - 1]);
    WorkingSet->mEqRemoved--;
  }
  if (nonlinEqRemoved) {
    for (k = 0; k < 18; k++) {
      WorkingSet->Wlocalidx.data[k] = k + 1;
    }
  }
}

/*
 * File trailer for normal.c
 *
 * [EOF]
 */
