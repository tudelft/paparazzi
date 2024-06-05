/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: step.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Jun-2024 11:13:45
 */

/* Include Files */
#include "step.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "driver.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int *STEP_TYPE
 *                double Hessian[225]
 *                const double lb[15]
 *                const double ub[15]
 *                h_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                f_struct_T *memspace
 *                i_struct_T *WorkingSet
 *                d_struct_T *QRManager
 *                e_struct_T *CholManager
 *                struct_T *QPObjective
 *                j_struct_T *qpoptions
 * Return Type  : bool
 */
bool step(int *STEP_TYPE, double Hessian[225], const double lb[15],
          const double ub[15], h_struct_T *TrialState,
          b_struct_T *MeritFunction, f_struct_T *memspace,
          i_struct_T *WorkingSet, d_struct_T *QRManager,
          e_struct_T *CholManager, struct_T *QPObjective, j_struct_T *qpoptions)
{
  j_struct_T b_qpoptions;
  double dv[16];
  double oldDirIdx;
  double s;
  int idxEndIneq_tmp_tmp;
  int idxStartIneq;
  int k;
  int nVar_tmp_tmp;
  bool checkBoundViolation;
  bool stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0],
           (unsigned int)idxEndIneq_tmp_tmp * sizeof(double));
  } else {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    if (idxEndIneq_tmp_tmp - 1 >= 0) {
      memcpy(&TrialState->searchDir[0], &TrialState->xstar[0],
             (unsigned int)idxEndIneq_tmp_tmp * sizeof(double));
    }
  }
  int exitg1;
  bool guard1;
  do {
    int temp;
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
    case 1:
      b_qpoptions = *qpoptions;
      driver(Hessian, TrialState->grad, TrialState, memspace, WorkingSet,
             QRManager, CholManager, QPObjective, &b_qpoptions,
             qpoptions->MaxIterations);
      if (TrialState->state > 0) {
        MeritFunction->phi = TrialState->sqpFval;
        MeritFunction->linearizedConstrViol = 0.0;
        MeritFunction->penaltyParam = 1.0;
        MeritFunction->phiPrimePlus = fmin(TrialState->fstar, 0.0);
      }
      sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                   WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                   WorkingSet->Wlocalidx, memspace->workspace_double);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0],
                 (unsigned int)idxEndIneq_tmp_tmp * sizeof(double));
        }
        guard1 = true;
      }
      break;
    case 2: {
      double beta;
      temp = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idxStartIneq = temp + 1;
      idxEndIneq_tmp_tmp = WorkingSet->nActiveConstr;
      for (k = idxStartIneq; k <= idxEndIneq_tmp_tmp; k++) {
        WorkingSet->isActiveConstr
            [(WorkingSet->isActiveIdx[WorkingSet->Wid[k - 1] - 1] +
              WorkingSet->Wlocalidx[k - 1]) -
             2] = false;
      }
      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = temp;
      memcpy(&dv[0], &TrialState->xstar[0], 16U * sizeof(double));
      idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
      for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
        s = WorkingSet->lb[WorkingSet->indexLB[k] - 1];
        if (-dv[WorkingSet->indexLB[k] - 1] > s) {
          if (rtIsInf(ub[WorkingSet->indexLB[k] - 1])) {
            dv[WorkingSet->indexLB[k] - 1] = -s + fabs(s);
          } else {
            dv[WorkingSet->indexLB[k] - 1] =
                (WorkingSet->ub[WorkingSet->indexLB[k] - 1] - s) / 2.0;
          }
        }
      }
      idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
      for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
        s = WorkingSet->ub[WorkingSet->indexUB[k] - 1];
        if (dv[WorkingSet->indexUB[k] - 1] > s) {
          if (rtIsInf(lb[WorkingSet->indexUB[k] - 1])) {
            dv[WorkingSet->indexUB[k] - 1] = s - fabs(s);
          } else {
            dv[WorkingSet->indexUB[k] - 1] =
                (s - WorkingSet->lb[WorkingSet->indexUB[k] - 1]) / 2.0;
          }
        }
      }
      memcpy(&TrialState->xstar[0], &dv[0], 16U * sizeof(double));
      idxEndIneq_tmp_tmp = WorkingSet->nVar;
      beta = 0.0;
      idxStartIneq = (unsigned char)WorkingSet->nVar;
      for (k = 0; k < idxStartIneq; k++) {
        beta += Hessian[k + 15 * k];
      }
      beta /= (double)WorkingSet->nVar;
      if (TrialState->sqpIterations <= 1) {
        temp = QPObjective->nvar;
        if (QPObjective->nvar < 1) {
          idxStartIneq = 0;
        } else {
          idxStartIneq = 1;
          if (QPObjective->nvar > 1) {
            oldDirIdx = fabs(TrialState->grad[0]);
            for (k = 2; k <= temp; k++) {
              s = fabs(TrialState->grad[k - 1]);
              if (s > oldDirIdx) {
                idxStartIneq = k;
                oldDirIdx = s;
              }
            }
          }
        }
        s = 100.0 * fmax(1.0, fabs(TrialState->grad[idxStartIneq - 1]));
      } else {
        temp = WorkingSet->mConstr;
        if (WorkingSet->mConstr < 1) {
          idxStartIneq = 0;
        } else {
          idxStartIneq = 1;
          if (WorkingSet->mConstr > 1) {
            oldDirIdx = fabs(TrialState->lambdasqp[0]);
            for (k = 2; k <= temp; k++) {
              s = fabs(TrialState->lambdasqp[k - 1]);
              if (s > oldDirIdx) {
                idxStartIneq = k;
                oldDirIdx = s;
              }
            }
          }
        }
        s = fabs(TrialState->lambdasqp[idxStartIneq - 1]);
      }
      QPObjective->nvar = WorkingSet->nVar;
      QPObjective->beta = beta;
      QPObjective->rho = s;
      QPObjective->hasLinear = true;
      QPObjective->objtype = 4;
      setProblemType(WorkingSet, 2);
      temp = qpoptions->MaxIterations;
      qpoptions->MaxIterations =
          (qpoptions->MaxIterations + WorkingSet->nVar) - idxEndIneq_tmp_tmp;
      memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
      b_qpoptions = *qpoptions;
      driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
             CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
      qpoptions->MaxIterations = temp;
      if (TrialState->state != -6) {
        MeritFunction->phi = TrialState->sqpFval;
        MeritFunction->linearizedConstrViol = 0.0;
        MeritFunction->penaltyParam = 1.0;
        MeritFunction->phiPrimePlus =
            fmin((TrialState->fstar - s * 0.0) - beta / 2.0 * 0.0, 0.0);
        temp = WorkingSet->isActiveIdx[2];
        idxStartIneq = WorkingSet->nActiveConstr;
        for (k = temp; k <= idxStartIneq; k++) {
          if (WorkingSet->Wid[k - 1] == 3) {
            TrialState->lambda[k - 1] *=
                (double)
                    memspace->workspace_int[WorkingSet->Wlocalidx[k - 1] - 1];
          }
        }
      }
      QPObjective->nvar = idxEndIneq_tmp_tmp;
      QPObjective->hasLinear = true;
      QPObjective->objtype = 3;
      setProblemType(WorkingSet, 3);
      sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                   WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                   WorkingSet->Wlocalidx, memspace->workspace_double);
      idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
      if (idxEndIneq_tmp_tmp - 1 >= 0) {
        memcpy(&TrialState->delta_x[0], &TrialState->xstar[0],
               (unsigned int)idxEndIneq_tmp_tmp * sizeof(double));
      }
      guard1 = true;
    } break;
    default:
      idxEndIneq_tmp_tmp = WorkingSet->nVar;
      idxStartIneq = (unsigned char)WorkingSet->nVar;
      for (k = 0; k < idxStartIneq; k++) {
        TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k];
        TrialState->socDirection[k] = TrialState->xstar[k];
      }
      memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0],
             31U * sizeof(double));
      memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0],
             (unsigned int)idxStartIneq * sizeof(double));
      memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
      b_qpoptions = *qpoptions;
      driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
             CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
      idxStartIneq = (unsigned char)idxEndIneq_tmp_tmp;
      for (k = 0; k < idxStartIneq; k++) {
        s = TrialState->socDirection[k];
        oldDirIdx = s;
        s = TrialState->xstar[k] - s;
        TrialState->socDirection[k] = s;
        TrialState->xstar[k] = oldDirIdx;
      }
      stepSuccess = (b_xnrm2(idxEndIneq_tmp_tmp, TrialState->socDirection) <=
                     2.0 * b_xnrm2(idxEndIneq_tmp_tmp, TrialState->xstar));
      if (!stepSuccess) {
        memcpy(&TrialState->lambda[0], &TrialState->lambdaStopTest[0],
               31U * sizeof(double));
      } else {
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx,
                     WorkingSet->Wid, WorkingSet->Wlocalidx,
                     memspace->workspace_double);
      }
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          TrialState->delta_x[k] =
              TrialState->xstar[k] + TrialState->socDirection[k];
        }
      }
      guard1 = true;
      break;
    }
    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        oldDirIdx = 0.0;
        s = 1.0;
        for (k = 0; k < 15; k++) {
          oldDirIdx = fmax(oldDirIdx, fabs(TrialState->grad[k]));
          s = fmax(s, fabs(TrialState->xstar[k]));
        }
        s = fmax(2.2204460492503131E-16, oldDirIdx / s);
        for (idxStartIneq = 0; idxStartIneq < 15; idxStartIneq++) {
          temp = 15 * idxStartIneq;
          for (k = 0; k < idxStartIneq; k++) {
            Hessian[temp + k] = 0.0;
          }
          temp = idxStartIneq + 15 * idxStartIneq;
          Hessian[temp] = s;
          idxEndIneq_tmp_tmp = 13 - idxStartIneq;
          if (idxEndIneq_tmp_tmp >= 0) {
            memset(&Hessian[temp + 1], 0,
                   (unsigned int)(((idxEndIneq_tmp_tmp + temp) - temp) + 1) *
                       sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    memcpy(&dv[0], &TrialState->delta_x[0], 16U * sizeof(double));
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexLB[k] - 1];
      s = (TrialState->xstarsqp[WorkingSet->indexLB[k] - 1] + oldDirIdx) -
          lb[WorkingSet->indexLB[k] - 1];
      if (s < 0.0) {
        dv[WorkingSet->indexLB[k] - 1] = oldDirIdx - s;
        TrialState->xstar[WorkingSet->indexLB[k] - 1] -= s;
      }
    }
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexUB[k] - 1];
      s = (ub[WorkingSet->indexUB[k] - 1] -
           TrialState->xstarsqp[WorkingSet->indexUB[k] - 1]) -
          oldDirIdx;
      if (s < 0.0) {
        dv[WorkingSet->indexUB[k] - 1] = oldDirIdx + s;
        TrialState->xstar[WorkingSet->indexUB[k] - 1] += s;
      }
    }
    memcpy(&TrialState->delta_x[0], &dv[0], 16U * sizeof(double));
  }
  return stepSuccess;
}

/*
 * File trailer for step.c
 *
 * [EOF]
 */
