/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: driver.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "driver.h"
#include "BFGSUpdate.h"
#include "evalObjAndConstr.h"
#include "evalObjAndConstrAndDerivatives.h"
#include "linesearch.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "saveJacobian.h"
#include "step.h"
#include "test_exit.h"
#include "updateWorkingSetForNewQP.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : c_struct_T *TrialState
 *                struct_T *MeritFunction
 *                const h_coder_internal_stickyStruct *FcnEvaluator
 *                e_struct_T *memspace
 *                f_struct_T *WorkingSet
 *                g_struct_T *QRManager
 *                h_struct_T *CholManager
 *                d_struct_T *QPObjective
 *                double Hessian[324]
 * Return Type  : void
 */
void driver(c_struct_T *TrialState, struct_T *MeritFunction,
            const h_coder_internal_stickyStruct *FcnEvaluator,
            e_struct_T *memspace, f_struct_T *WorkingSet, g_struct_T *QRManager,
            h_struct_T *CholManager, d_struct_T *QPObjective,
            double Hessian[324])
{
  static const signed char iv[324] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const char qpoptions_SolverName[7] = {'f', 'm', 'i', 'n',
                                               'c', 'o', 'n'};
  i_struct_T Flags;
  j_struct_T b_expl_temp;
  j_struct_T expl_temp;
  double y_data[311];
  double b_y_data[183];
  int i;
  int ia;
  int iac;
  int ix;
  int ixlast;
  int mLB;
  int nVar;
  int qpoptions_MaxIterations;
  for (i = 0; i < 324; i++) {
    Hessian[i] = iv[i];
  }
  nVar = WorkingSet->nVar;
  mLB = WorkingSet->sizes[3];
  ixlast = WorkingSet->nVar;
  ix = WorkingSet->sizes[3] + 128;
  if (ixlast >= ix) {
    ix = ixlast;
  }
  qpoptions_MaxIterations = 10 * ix;
  TrialState->steplength = 1.0;
  test_exit(MeritFunction, WorkingSet, TrialState, &Flags.fevalOK, &Flags.done,
            &Flags.stepAccepted, &Flags.failedLineSearch, &Flags.stepType);
  saveJacobian(TrialState, nVar, WorkingSet->Aineq, WorkingSet->Aeq.data);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (ix = 0; ix < 18; ix++) {
    TrialState->xstarsqp_old[ix] = TrialState->xstarsqp[ix];
    TrialState->grad_old.data[ix] = TrialState->grad.data[ix];
  }
  ixlast = TrialState->cIneq_old.size[0];
  ix = TrialState->cIneq_old.size[0];
  if (ix - 1 >= 0) {
    memcpy(&y_data[0], &TrialState->cIneq_old.data[0],
           (unsigned int)ix * sizeof(double));
  }
  memcpy(&y_data[0], &TrialState->cIneq.data[0], 128U * sizeof(double));
  if (ixlast - 1 >= 0) {
    memcpy(&TrialState->cIneq_old.data[0], &y_data[0],
           (unsigned int)ixlast * sizeof(double));
  }
  TrialState->cEq_old.size[0] = 18;
  memcpy(&TrialState->cEq_old.data[0], &TrialState->cEq.data[0],
         18U * sizeof(double));
  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }
  while (!Flags.done) {
    double d;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        updateWorkingSetForNewQP(WorkingSet, TrialState->cIneq.data,
                                 TrialState->cEq.data);
      }
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }
      b_expl_temp = expl_temp;
      Flags.stepAccepted =
          step(&Flags.stepType, Hessian, TrialState, MeritFunction, memspace,
               WorkingSet, QRManager, CholManager, QPObjective, &b_expl_temp);
      if (Flags.stepAccepted) {
        i = (unsigned char)nVar;
        for (ixlast = 0; ixlast < i; ixlast++) {
          TrialState->xstarsqp[ixlast] += TrialState->delta_x.data[ixlast];
        }
        TrialState->sqpFval = evalObjAndConstr(
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace.P0,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace.V0,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace.A0,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .v_max,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .v_min,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .a_max,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .a_min,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .landing_time,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .num_points,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .coeffs_ship_prediction,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .pos_gain,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .speed_gain,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .acc_gain,
            TrialState->xstarsqp, TrialState->cIneq.data, TrialState->cEq.data,
            &ixlast);
        Flags.fevalOK = (ixlast == 1);
        TrialState->FunctionEvaluations++;
        if (Flags.fevalOK) {
          double constrViolationEq;
          double constrViolationIneq;
          constrViolationEq = 0.0;
          for (ix = 0; ix < 18; ix++) {
            constrViolationEq += fabs(TrialState->cEq.data[ix]);
          }
          constrViolationIneq = 0.0;
          for (ixlast = 0; ixlast < 128; ixlast++) {
            d = TrialState->cIneq.data[ixlast];
            if (d > 0.0) {
              constrViolationIneq += d;
            }
          }
          MeritFunction->phiFullStep =
              TrialState->sqpFval +
              MeritFunction->penaltyParam *
                  (constrViolationEq + constrViolationIneq);
        } else {
          MeritFunction->phiFullStep = rtInf;
        }
      }
      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        bool socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }
        d = linesearch(
            &Flags.fevalOK, WorkingSet->nVar, TrialState,
            MeritFunction->penaltyParam, MeritFunction->phi,
            MeritFunction->phiPrimePlus, MeritFunction->phiFullStep,
            &FcnEvaluator->next.next.next.next.next.next.next.value.workspace,
            socTaken, &ixlast);
        TrialState->steplength = d;
        if (ixlast > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }
    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      i = (unsigned char)nVar;
      for (ixlast = 0; ixlast < i; ixlast++) {
        TrialState->xstarsqp[ixlast] =
            TrialState->xstarsqp_old[ixlast] + TrialState->delta_x.data[ixlast];
      }
      i = (unsigned short)(mLB + 146);
      for (ixlast = 0; ixlast < i; ixlast++) {
        d = TrialState->lambdasqp.data[ixlast];
        d += TrialState->steplength * (TrialState->lambda.data[ixlast] - d);
        TrialState->lambdasqp.data[ixlast] = d;
      }
      TrialState->sqpFval_old = TrialState->sqpFval;
      for (ix = 0; ix < 18; ix++) {
        TrialState->xstarsqp_old[ix] = TrialState->xstarsqp[ix];
        TrialState->grad_old.data[ix] = TrialState->grad.data[ix];
      }
      ixlast = TrialState->cIneq_old.size[0];
      ix = TrialState->cIneq_old.size[0];
      if (ix - 1 >= 0) {
        memcpy(&y_data[0], &TrialState->cIneq_old.data[0],
               (unsigned int)ix * sizeof(double));
      }
      memcpy(&y_data[0], &TrialState->cIneq.data[0], 128U * sizeof(double));
      if (ixlast - 1 >= 0) {
        memcpy(&TrialState->cIneq_old.data[0], &y_data[0],
               (unsigned int)ixlast * sizeof(double));
      }
      TrialState->cEq_old.size[0] = 18;
      memcpy(&TrialState->cEq_old.data[0], &TrialState->cEq.data[0],
             18U * sizeof(double));
      TrialState->sqpFval = evalObjAndConstrAndDerivatives(
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace.P0,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace.V0,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace.A0,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .v_max,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .v_min,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .a_max,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .a_min,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .landing_time,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .num_points,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .coeffs_ship_prediction,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .pos_gain,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .speed_gain,
          FcnEvaluator->next.next.next.next.next.next.next.value.workspace
              .acc_gain,
          TrialState->xstarsqp, TrialState->grad.data, TrialState->cIneq.data,
          TrialState->cEq.data, &TrialState->cEq.size[0], WorkingSet->Aineq,
          WorkingSet->Aeq.data, &WorkingSet->Aeq.size[0], &ixlast);
      TrialState->FunctionEvaluations++;
      Flags.fevalOK = (ixlast == 1);
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0],
             18U * sizeof(double));
      ixlast = TrialState->cIneq.size[0];
      ix = TrialState->cIneq.size[0];
      if (ix - 1 >= 0) {
        memcpy(&y_data[0], &TrialState->cIneq.data[0],
               (unsigned int)ix * sizeof(double));
      }
      memcpy(&y_data[0], &TrialState->cIneq_old.data[0], 128U * sizeof(double));
      if (ixlast - 1 >= 0) {
        memcpy(&TrialState->cIneq.data[0], &y_data[0],
               (unsigned int)ixlast * sizeof(double));
      }
      TrialState->cEq.size[0] = 18;
      memcpy(&TrialState->cEq.data[0], &TrialState->cEq_old.data[0],
             18U * sizeof(double));
    }
    b_test_exit(&Flags, memspace, MeritFunction, WorkingSet, TrialState,
                QRManager);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      i = (unsigned char)nVar;
      memcpy(&TrialState->delta_gradLag.data[0], &TrialState->grad.data[0],
             (unsigned int)i * sizeof(double));
      memcpy(&b_y_data[0], &TrialState->delta_gradLag.data[0],
             183U * sizeof(double));
      if (nVar >= 1) {
        ixlast = nVar - 1;
        for (ix = 0; ix <= ixlast; ix++) {
          b_y_data[ix] -= TrialState->grad_old.data[ix];
        }
      }
      TrialState->delta_gradLag.size[0] = 183;
      memcpy(&TrialState->delta_gradLag.data[0], &b_y_data[0],
             183U * sizeof(double));
      ix = 0;
      for (iac = 0; iac <= 3111; iac += 183) {
        i = iac + nVar;
        for (ia = iac + 1; ia <= i; ia++) {
          ixlast = (ia - iac) - 1;
          TrialState->delta_gradLag.data[ixlast] +=
              WorkingSet->Aeq.data[ia - 1] * TrialState->lambdasqp.data[ix];
        }
        ix++;
      }
      ix = 0;
      for (iac = 0; iac <= 3111; iac += 183) {
        i = iac + nVar;
        for (ia = iac + 1; ia <= i; ia++) {
          ixlast = (ia - iac) - 1;
          TrialState->delta_gradLag.data[ixlast] +=
              TrialState->JacCeqTrans_old->data[ia - 1] *
              -TrialState->lambdasqp.data[ix];
        }
        ix++;
      }
      ix = 18;
      for (iac = 0; iac <= 23241; iac += 183) {
        i = iac + nVar;
        for (ia = iac + 1; ia <= i; ia++) {
          ixlast = (ia - iac) - 1;
          TrialState->delta_gradLag.data[ixlast] +=
              WorkingSet->Aineq->data[ia - 1] * TrialState->lambdasqp.data[ix];
        }
        ix++;
      }
      ix = 18;
      for (iac = 0; iac <= 23241; iac += 183) {
        i = iac + nVar;
        for (ia = iac + 1; ia <= i; ia++) {
          ixlast = (ia - iac) - 1;
          TrialState->delta_gradLag.data[ixlast] +=
              TrialState->JacCineqTrans_old->data[ia - 1] *
              -TrialState->lambdasqp.data[ix];
        }
        ix++;
      }
      saveJacobian(TrialState, nVar, WorkingSet->Aineq, WorkingSet->Aeq.data);
      BFGSUpdate(nVar, Hessian, TrialState->delta_x.data,
                 TrialState->delta_gradLag.data, memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/*
 * File trailer for driver.c
 *
 * [EOF]
 */
