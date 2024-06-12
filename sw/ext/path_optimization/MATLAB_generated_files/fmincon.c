/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fmincon.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "fmincon.h"
#include "checkNonlinearInputs.h"
#include "driver.h"
#include "evalObjAndConstrAndDerivatives.h"
#include "loadProblem.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x0[18]
 *                const double nonlcon_workspace_P0[3]
 *                const double nonlcon_workspace_V0[3]
 *                const double nonlcon_workspace_A0[3]
 *                const double nonlcon_workspace_v_max[3]
 *                const double nonlcon_workspace_v_min[3]
 *                const double nonlcon_workspace_a_max[3]
 *                const double nonlcon_workspace_a_min[3]
 *                double nonlcon_workspace_landing_time
 *                double nonlcon_workspace_num_points
 *                const double c_nonlcon_workspace_coeffs_ship[18]
 *                double nonlcon_workspace_pos_gain
 *                double nonlcon_workspace_speed_gain
 *                double nonlcon_workspace_acc_gain
 *                double x[18]
 *                double *exitflag
 *                double *output_iterations
 *                double *output_funcCount
 *                char output_algorithm[3]
 *                double *output_constrviolation
 *                double *output_stepsize
 *                double *output_lssteplength
 *                double *output_firstorderopt
 * Return Type  : double
 */
double fmincon(const double x0[18], const double nonlcon_workspace_P0[3],
               const double nonlcon_workspace_V0[3],
               const double nonlcon_workspace_A0[3],
               const double nonlcon_workspace_v_max[3],
               const double nonlcon_workspace_v_min[3],
               const double nonlcon_workspace_a_max[3],
               const double nonlcon_workspace_a_min[3],
               double nonlcon_workspace_landing_time,
               double nonlcon_workspace_num_points,
               const double c_nonlcon_workspace_coeffs_ship[18],
               double nonlcon_workspace_pos_gain,
               double nonlcon_workspace_speed_gain,
               double nonlcon_workspace_acc_gain, double x[18],
               double *exitflag, double *output_iterations,
               double *output_funcCount, char output_algorithm[3],
               double *output_constrviolation, double *output_stepsize,
               double *output_lssteplength, double *output_firstorderopt)
{
  static f_struct_T WorkingSet;
  c_struct_T TrialState;
  d_struct_T QPObjective;
  e_struct_T memspace;
  g_struct_T obj;
  h_coder_internal_stickyStruct FcnEvaluator;
  h_struct_T b_obj;
  struct_T MeritFunction;
  double unusedExpr[324];
  double fval;
  double normResid;
  double scale;
  int i;
  int iEq0;
  int idx;
  int iw0;
  output_algorithm[0] = 's';
  output_algorithm[1] = 'q';
  output_algorithm[2] = 'p';
  iEq0 = checkNonlinearInputs(
      x0, nonlcon_workspace_P0, nonlcon_workspace_V0, nonlcon_workspace_A0,
      nonlcon_workspace_v_max, nonlcon_workspace_v_min, nonlcon_workspace_a_max,
      nonlcon_workspace_a_min, nonlcon_workspace_landing_time,
      nonlcon_workspace_num_points, c_nonlcon_workspace_coeffs_ship,
      nonlcon_workspace_pos_gain, nonlcon_workspace_speed_gain,
      nonlcon_workspace_acc_gain, &iw0);
  emxInitStruct_struct_T(&TrialState);
  TrialState.nVarMax = 183;
  TrialState.mNonlinIneq = 128;
  TrialState.mNonlinEq = 18;
  TrialState.mIneq = 128;
  TrialState.mEq = 18;
  TrialState.iNonIneq0 = 1;
  TrialState.iNonEq0 = 1;
  TrialState.sqpFval_old = 0.0;
  TrialState.cIneq.size[0] = 128;
  TrialState.cIneq_old.size[0] = 128;
  TrialState.cEq.size[0] = 18;
  TrialState.cEq_old.size[0] = 18;
  TrialState.grad.size[0] = 183;
  TrialState.grad_old.size[0] = 183;
  TrialState.sqpIterations = 0;
  TrialState.sqpExitFlag = 0;
  TrialState.lambdasqp.size[0] = 311;
  memset(&TrialState.lambdasqp.data[0], 0, 311U * sizeof(double));
  TrialState.lambdaStopTest.size[0] = 311;
  TrialState.lambdaStopTestPrev.size[0] = 311;
  TrialState.steplength = 1.0;
  TrialState.delta_x.size[0] = 183;
  memset(&TrialState.delta_x.data[0], 0, 183U * sizeof(double));
  TrialState.socDirection.size[0] = 183;
  TrialState.workingset_old.size[0] = 311;
  iw0 = TrialState.JacCineqTrans_old->size[0] *
        TrialState.JacCineqTrans_old->size[1];
  TrialState.JacCineqTrans_old->size[0] = 183;
  TrialState.JacCineqTrans_old->size[1] = 128;
  emxEnsureCapacity_real_T(TrialState.JacCineqTrans_old, iw0);
  iw0 =
      TrialState.JacCeqTrans_old->size[0] * TrialState.JacCeqTrans_old->size[1];
  TrialState.JacCeqTrans_old->size[0] = 183;
  TrialState.JacCeqTrans_old->size[1] = 18;
  emxEnsureCapacity_real_T(TrialState.JacCeqTrans_old, iw0);
  TrialState.gradLag.size[0] = 183;
  TrialState.delta_gradLag.size[0] = 183;
  TrialState.xstar.size[0] = 183;
  TrialState.fstar = 0.0;
  TrialState.firstorderopt = 0.0;
  TrialState.lambda.size[0] = 311;
  memset(&TrialState.lambda.data[0], 0, 311U * sizeof(double));
  TrialState.state = 0;
  TrialState.maxConstr = 0.0;
  TrialState.iterations = 0;
  TrialState.searchDir.size[0] = 183;
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.P0[0] =
      nonlcon_workspace_P0[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.V0[0] =
      nonlcon_workspace_V0[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.A0[0] =
      nonlcon_workspace_A0[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_max[0] =
      nonlcon_workspace_v_max[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_min[0] =
      nonlcon_workspace_v_min[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_max[0] =
      nonlcon_workspace_a_max[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_min[0] =
      nonlcon_workspace_a_min[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.P0[1] =
      nonlcon_workspace_P0[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.V0[1] =
      nonlcon_workspace_V0[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.A0[1] =
      nonlcon_workspace_A0[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_max[1] =
      nonlcon_workspace_v_max[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_min[1] =
      nonlcon_workspace_v_min[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_max[1] =
      nonlcon_workspace_a_max[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_min[1] =
      nonlcon_workspace_a_min[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.P0[2] =
      nonlcon_workspace_P0[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.V0[2] =
      nonlcon_workspace_V0[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.A0[2] =
      nonlcon_workspace_A0[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_max[2] =
      nonlcon_workspace_v_max[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_min[2] =
      nonlcon_workspace_v_min[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_max[2] =
      nonlcon_workspace_a_max[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_min[2] =
      nonlcon_workspace_a_min[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.landing_time =
      nonlcon_workspace_landing_time;
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.num_points =
      nonlcon_workspace_num_points;
  memcpy(&TrialState.xstarsqp[0], &x0[0], 18U * sizeof(double));
  memcpy(&FcnEvaluator.next.next.next.next.next.next.next.value.workspace
              .coeffs_ship_prediction[0],
         &c_nonlcon_workspace_coeffs_ship[0], 18U * sizeof(double));
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pos_gain =
      nonlcon_workspace_pos_gain;
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.speed_gain =
      nonlcon_workspace_speed_gain;
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.acc_gain =
      nonlcon_workspace_acc_gain;
  FcnEvaluator.next.next.next.next.next.value = 128;
  FcnEvaluator.next.next.next.next.value = 18;
  QPObjective.grad.size[0] = 183;
  QPObjective.Hx.size[0] = 182;
  QPObjective.maxVar = 183;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.hasLinear = true;
  QPObjective.nvar = 18;
  QPObjective.objtype = 3;
  emxInitStruct_struct_T1(&memspace);
  iw0 = memspace.workspace_double->size[0] * memspace.workspace_double->size[1];
  memspace.workspace_double->size[0] = 311;
  memspace.workspace_double->size[1] = 183;
  emxEnsureCapacity_real_T(memspace.workspace_double, iw0);
  memspace.workspace_int.size[0] = 311;
  memspace.workspace_sort.size[0] = 311;
  emxInitStruct_struct_T2(&WorkingSet);
  WorkingSet.mConstr = 0;
  WorkingSet.mConstrOrig = 0;
  WorkingSet.mConstrMax = 311;
  WorkingSet.nVar = 18;
  WorkingSet.nVarOrig = 18;
  WorkingSet.nVarMax = 183;
  WorkingSet.ldA = 183;
  iw0 = WorkingSet.Aineq->size[0];
  WorkingSet.Aineq->size[0] = 23424;
  emxEnsureCapacity_real_T(WorkingSet.Aineq, iw0);
  WorkingSet.bineq.size[0] = 128;
  WorkingSet.Aeq.size[0] = 3294;
  WorkingSet.beq.size[0] = 18;
  WorkingSet.lb.size[0] = 183;
  WorkingSet.ub.size[0] = 183;
  WorkingSet.indexLB.size[0] = 183;
  WorkingSet.indexUB.size[0] = 183;
  WorkingSet.indexFixed.size[0] = 183;
  WorkingSet.mEqRemoved = 0;
  WorkingSet.indexEqRemoved.size[0] = 18;
  iw0 = WorkingSet.ATwset->size[0];
  WorkingSet.ATwset->size[0] = 56913;
  emxEnsureCapacity_real_T(WorkingSet.ATwset, iw0);
  WorkingSet.bwset.size[0] = 311;
  WorkingSet.nActiveConstr = 0;
  WorkingSet.maxConstrWorkspace.size[0] = 311;
  for (i = 0; i < 5; i++) {
    WorkingSet.sizes[i] = 0;
    WorkingSet.sizesNormal[i] = 0;
    WorkingSet.sizesPhaseOne[i] = 0;
    WorkingSet.sizesRegularized[i] = 0;
    WorkingSet.sizesRegPhaseOne[i] = 0;
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdx[i] = 0;
    WorkingSet.isActiveIdxNormal[i] = 0;
    WorkingSet.isActiveIdxPhaseOne[i] = 0;
    WorkingSet.isActiveIdxRegularized[i] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[i] = 0;
  }
  WorkingSet.isActiveConstr.size[0] = 311;
  WorkingSet.Wid.size[0] = 311;
  WorkingSet.Wlocalidx.size[0] = 311;
  for (i = 0; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  loadProblem(&WorkingSet);
  TrialState.sqpFval = evalObjAndConstrAndDerivatives(
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace.P0,
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace.V0,
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace.A0,
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_max,
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace.v_min,
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_max,
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace.a_min,
      nonlcon_workspace_landing_time, nonlcon_workspace_num_points,
      FcnEvaluator.next.next.next.next.next.next.next.value.workspace
          .coeffs_ship_prediction,
      nonlcon_workspace_pos_gain, nonlcon_workspace_speed_gain,
      nonlcon_workspace_acc_gain, TrialState.xstarsqp, TrialState.grad.data,
      TrialState.cIneq.data, TrialState.cEq.data, &TrialState.cEq.size[0],
      WorkingSet.Aineq, WorkingSet.Aeq.data, &WorkingSet.Aeq.size[0], &iEq0);
  TrialState.FunctionEvaluations = 1;
  iw0 = 0;
  iEq0 = 0;
  for (idx = 0; idx < 18; idx++) {
    WorkingSet.beq.data[idx] = -TrialState.cEq.data[idx];
    WorkingSet.bwset.data[idx] = WorkingSet.beq.data[idx];
    for (i = 0; i < 18; i++) {
      WorkingSet.ATwset->data[iw0 + i] = WorkingSet.Aeq.data[iEq0 + i];
    }
    iEq0 = iw0 + 183;
    iw0 += 183;
  }
  for (idx = 0; idx < 128; idx++) {
    WorkingSet.bineq.data[idx] = -TrialState.cIneq.data[idx];
  }
  setProblemType(&WorkingSet, 3);
  iEq0 = WorkingSet.isActiveIdx[2];
  for (idx = iEq0; idx < 312; idx++) {
    WorkingSet.isActiveConstr.data[idx - 1] = false;
  }
  WorkingSet.nWConstr[0] = 0;
  WorkingSet.nWConstr[1] = 18;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = 18;
  MeritFunction.initFval = TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  normResid = 0.0;
  iw0 = WorkingSet.nVar - 1;
  for (iEq0 = 0; iEq0 < 18; iEq0++) {
    WorkingSet.Wid.data[iEq0] = 2;
    WorkingSet.Wlocalidx.data[iEq0] = iEq0 + 1;
    WorkingSet.isActiveConstr.data[iEq0] = true;
    idx = 183 * iEq0;
    for (i = 0; i <= iw0; i++) {
      int b_i;
      b_i = idx + i;
      WorkingSet.ATwset->data[b_i] = WorkingSet.Aeq.data[b_i];
    }
    WorkingSet.bwset.data[iEq0] = WorkingSet.beq.data[iEq0];
    normResid += fabs(TrialState.cEq.data[iEq0]);
  }
  MeritFunction.initConstrViolationEq = normResid;
  normResid = 0.0;
  for (idx = 0; idx < 128; idx++) {
    scale = TrialState.cIneq.data[idx];
    if (scale > 0.0) {
      normResid += scale;
    }
  }
  MeritFunction.initConstrViolationIneq = normResid;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  emxInitStruct_struct_T3(&obj);
  obj.ldq = 311;
  iw0 = obj.QR->size[0] * obj.QR->size[1];
  obj.QR->size[0] = 311;
  obj.QR->size[1] = 311;
  emxEnsureCapacity_real_T(obj.QR, iw0);
  iw0 = obj.Q->size[0] * obj.Q->size[1];
  obj.Q->size[0] = 311;
  obj.Q->size[1] = 311;
  emxEnsureCapacity_real_T(obj.Q, iw0);
  for (iw0 = 0; iw0 < 96721; iw0++) {
    obj.Q->data[iw0] = 0.0;
  }
  obj.jpvt.size[0] = 311;
  memset(&obj.jpvt.data[0], 0, 311U * sizeof(int));
  obj.mrows = 0;
  obj.ncols = 0;
  obj.tau.size[0] = 311;
  obj.minRowCol = 0;
  obj.usedPivoting = false;
  emxInitStruct_struct_T4(&b_obj);
  iw0 = b_obj.FMat->size[0] * b_obj.FMat->size[1];
  b_obj.FMat->size[0] = 311;
  b_obj.FMat->size[1] = 311;
  emxEnsureCapacity_real_T(b_obj.FMat, iw0);
  b_obj.ldm = 311;
  b_obj.ndims = 0;
  b_obj.info = 0;
  b_obj.scaleFactor = 0.0;
  b_obj.ConvexCheck = true;
  b_obj.regTol_ = rtInf;
  b_obj.workspace_ = rtInf;
  b_obj.workspace2_ = rtInf;
  driver(&TrialState, &MeritFunction, &FcnEvaluator, &memspace, &WorkingSet,
         &obj, &b_obj, &QPObjective, unusedExpr);
  emxFreeStruct_struct_T4(&b_obj);
  emxFreeStruct_struct_T3(&obj);
  emxFreeStruct_struct_T2(&WorkingSet);
  emxFreeStruct_struct_T1(&memspace);
  fval = TrialState.sqpFval;
  *exitflag = TrialState.sqpExitFlag;
  *output_iterations = TrialState.sqpIterations;
  *output_funcCount = TrialState.FunctionEvaluations;
  *output_constrviolation = MeritFunction.nlpPrimalFeasError;
  normResid = 0.0;
  scale = 3.3121686421112381E-170;
  for (iEq0 = 0; iEq0 < 18; iEq0++) {
    double absxk;
    x[iEq0] = TrialState.xstarsqp[iEq0];
    absxk = fabs(TrialState.delta_x.data[iEq0]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      normResid = normResid * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      normResid += t * t;
    }
  }
  *output_stepsize = scale * sqrt(normResid);
  *output_lssteplength = TrialState.steplength;
  emxFreeStruct_struct_T(&TrialState);
  *output_firstorderopt = MeritFunction.firstOrderOpt;
  return fval;
}

/*
 * File trailer for fmincon.c
 *
 * [EOF]
 */
