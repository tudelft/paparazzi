/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fmincon.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

/* Include Files */
#include "fmincon.h"
#include "Cascaded_nonlinear_TestFlight_data.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "driver1.h"
#include "evalObjAndConstrAndDerivatives.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : e_struct_T *fun_workspace
 *                double x0[13]
 *                const double lb[13]
 *                const double ub[13]
 *                double *exitflag
 *                double *output_iterations
 *                double *output_funcCount
 *                char output_algorithm[3]
 *                double *output_constrviolation
 *                double *output_stepsize
 *                double *output_lssteplength
 *                double *output_firstorderopt
 *                l_struct_T *lambda
 *                double grad[13]
 *                double Hessian[169]
 * Return Type  : double
 */
double b_fmincon(e_struct_T *fun_workspace, double x0[13], const double lb[13],
                 const double ub[13], double *exitflag,
                 double *output_iterations, double *output_funcCount,
                 char output_algorithm[3], double *output_constrviolation,
                 double *output_stepsize, double *output_lssteplength,
                 double *output_firstorderopt, l_struct_T *lambda,
                 double grad[13], double Hessian[169])
{
  b_struct_T MeritFunction;
  c_struct_T QPObjective;
  e_struct_T b_fun_workspace;
  e_struct_T c_FcnEvaluator_next_next_next_n;
  e_struct_T d_FcnEvaluator_next_next_next_n;
  i_struct_T QRManager;
  j_struct_T CholManager;
  k_struct_T memspace;
  m_struct_T unusedExpr;
  p_struct_T TrialState;
  q_struct_T WorkingSet;
  r_coder_internal_stickyStruct r;
  double fval;
  int b_i;
  int i;
  int idx;
  int mFixed;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  bool exitg1;
  *exitflag = rtInf;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 13)) {
    if (lb[i] > ub[i]) {
      *exitflag = -2.0;
      exitg1 = true;
    } else {
      i++;
    }
  }
  for (b_i = 0; b_i < 169; b_i++) {
    Hessian[b_i] = iv[b_i];
  }
  if (*exitflag == -2.0) {
    fval = rtInf;
    *output_iterations = 0.0;
    *output_funcCount = 0.0;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = rtInf;
    *output_stepsize = rtInf;
    *output_lssteplength = rtInf;
    *output_firstorderopt = rtInf;
    for (i = 0; i < 13; i++) {
      lambda->lower[i] = 0.0;
      lambda->upper[i] = 0.0;
      grad[i] = rtInf;
    }
  } else {
    double scale;
    double y;
    int mLB;
    int mUB;
    TrialState.nVarMax = 14;
    TrialState.mNonlinIneq = 0;
    TrialState.mNonlinEq = 0;
    TrialState.mIneq = 0;
    TrialState.mEq = 0;
    TrialState.iNonIneq0 = 1;
    TrialState.iNonEq0 = 1;
    TrialState.sqpFval_old = 0.0;
    TrialState.sqpIterations = 0;
    TrialState.sqpExitFlag = 0;
    memset(&TrialState.lambdasqp[0], 0, 27U * sizeof(double));
    TrialState.steplength = 1.0;
    memset(&TrialState.delta_x[0], 0, 14U * sizeof(double));
    TrialState.fstar = 0.0;
    TrialState.firstorderopt = 0.0;
    memset(&TrialState.lambda[0], 0, 27U * sizeof(double));
    TrialState.state = 0;
    TrialState.maxConstr = 0.0;
    TrialState.iterations = 0;
    memcpy(&TrialState.xstarsqp[0], &x0[0], 13U * sizeof(double));
    c_FcnEvaluator_next_next_next_n = *fun_workspace;
    b_fun_workspace = *fun_workspace;
    factoryConstruct(&b_fun_workspace, lb, ub, &unusedExpr);
    WorkingSet.nVar = 13;
    WorkingSet.nVarOrig = 13;
    WorkingSet.nVarMax = 14;
    WorkingSet.ldA = 14;
    memset(&WorkingSet.lb[0], 0, 14U * sizeof(double));
    memset(&WorkingSet.ub[0], 0, 14U * sizeof(double));
    WorkingSet.mEqRemoved = 0;
    memset(&WorkingSet.ATwset[0], 0, 378U * sizeof(double));
    WorkingSet.nActiveConstr = 0;
    memset(&WorkingSet.bwset[0], 0, 27U * sizeof(double));
    memset(&WorkingSet.maxConstrWorkspace[0], 0, 27U * sizeof(double));
    memset(&WorkingSet.Wid[0], 0, 27U * sizeof(int));
    memset(&WorkingSet.Wlocalidx[0], 0, 27U * sizeof(int));
    for (i = 0; i < 27; i++) {
      WorkingSet.isActiveConstr[i] = false;
    }
    for (i = 0; i < 5; i++) {
      WorkingSet.nWConstr[i] = 0;
    }
    WorkingSet.probType = 3;
    WorkingSet.SLACK0 = 1.0E-5;
    for (i = 0; i < 14; i++) {
      WorkingSet.indexLB[i] = 0;
      WorkingSet.indexUB[i] = 0;
      WorkingSet.indexFixed[i] = 0;
    }
    mLB = 0;
    mUB = 0;
    mFixed = 0;
    for (idx = 0; idx < 13; idx++) {
      bool guard1;
      y = lb[idx];
      guard1 = false;
      if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
        if (fabs(y - ub[idx]) < 1.0E-6) {
          mFixed++;
          WorkingSet.indexFixed[mFixed - 1] = idx + 1;
        } else {
          mLB++;
          WorkingSet.indexLB[mLB - 1] = idx + 1;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        y = ub[idx];
        if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
          mUB++;
          WorkingSet.indexUB[mUB - 1] = idx + 1;
        }
      }
    }
    i = (mLB + mUB) + mFixed;
    WorkingSet.mConstr = i;
    WorkingSet.mConstrOrig = i;
    WorkingSet.mConstrMax = 27;
    obj_tmp[0] = (signed char)mFixed;
    obj_tmp[1] = 0;
    obj_tmp[2] = 0;
    obj_tmp[3] = (signed char)mLB;
    obj_tmp[4] = (signed char)mUB;
    b_obj_tmp[0] = (signed char)mFixed;
    b_obj_tmp[1] = 0;
    b_obj_tmp[2] = 0;
    b_obj_tmp[3] = (signed char)(mLB + 1);
    b_obj_tmp[4] = (signed char)mUB;
    WorkingSet.isActiveIdx[0] = 1;
    WorkingSet.isActiveIdx[1] = mFixed;
    WorkingSet.isActiveIdx[2] = 0;
    WorkingSet.isActiveIdx[3] = 0;
    WorkingSet.isActiveIdx[4] = mLB;
    WorkingSet.isActiveIdx[5] = mUB;
    for (i = 0; i < 5; i++) {
      signed char i1;
      signed char i2;
      i1 = obj_tmp[i];
      WorkingSet.sizes[i] = i1;
      WorkingSet.sizesNormal[i] = i1;
      i2 = b_obj_tmp[i];
      WorkingSet.sizesPhaseOne[i] = i2;
      WorkingSet.sizesRegularized[i] = i1;
      WorkingSet.sizesRegPhaseOne[i] = i2;
      WorkingSet.isActiveIdx[i + 1] += WorkingSet.isActiveIdx[i];
    }
    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxNormal[b_i] = WorkingSet.isActiveIdx[b_i];
    }
    WorkingSet.isActiveIdxPhaseOne[0] = 1;
    WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxPhaseOne[2] = 0;
    WorkingSet.isActiveIdxPhaseOne[3] = 0;
    WorkingSet.isActiveIdxPhaseOne[4] = mLB + 1;
    WorkingSet.isActiveIdxPhaseOne[5] = mUB;
    for (i = 0; i < 5; i++) {
      WorkingSet.isActiveIdxPhaseOne[i + 1] +=
          WorkingSet.isActiveIdxPhaseOne[i];
    }
    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
      WorkingSet.isActiveIdxRegPhaseOne[b_i] =
          WorkingSet.isActiveIdxPhaseOne[b_i];
    }
    for (idx = 0; idx < mLB; idx++) {
      b_i = WorkingSet.indexLB[idx];
      TrialState.xstarsqp[b_i - 1] =
          fmax(TrialState.xstarsqp[b_i - 1], lb[b_i - 1]);
    }
    for (idx = 0; idx < mUB; idx++) {
      b_i = WorkingSet.indexUB[idx];
      TrialState.xstarsqp[b_i - 1] =
          fmin(TrialState.xstarsqp[b_i - 1], ub[b_i - 1]);
    }
    for (idx = 0; idx < mFixed; idx++) {
      b_i = WorkingSet.indexFixed[idx];
      TrialState.xstarsqp[b_i - 1] = ub[b_i - 1];
    }
    d_FcnEvaluator_next_next_next_n = c_FcnEvaluator_next_next_next_n;
    TrialState.sqpFval = b_evalObjAndConstrAndDerivative(
        &d_FcnEvaluator_next_next_next_n, TrialState.xstarsqp, TrialState.grad,
        &i);
    TrialState.FunctionEvaluations = 1;
    for (idx = 0; idx < mLB; idx++) {
      WorkingSet.lb[WorkingSet.indexLB[idx] - 1] =
          -lb[WorkingSet.indexLB[idx] - 1] + x0[WorkingSet.indexLB[idx] - 1];
    }
    for (idx = 0; idx < mUB; idx++) {
      WorkingSet.ub[WorkingSet.indexUB[idx] - 1] =
          ub[WorkingSet.indexUB[idx] - 1] - x0[WorkingSet.indexUB[idx] - 1];
    }
    for (idx = 0; idx < mFixed; idx++) {
      y = ub[WorkingSet.indexFixed[idx] - 1] -
          x0[WorkingSet.indexFixed[idx] - 1];
      WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = y;
      WorkingSet.bwset[idx] = y;
    }
    b_setProblemType(&WorkingSet, 3);
    i = WorkingSet.isActiveIdx[2];
    for (idx = i; idx < 28; idx++) {
      WorkingSet.isActiveConstr[idx - 1] = false;
    }
    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    b_i = (unsigned char)WorkingSet.sizes[0];
    for (mFixed = 0; mFixed < b_i; mFixed++) {
      WorkingSet.Wid[mFixed] = 1;
      WorkingSet.Wlocalidx[mFixed] = mFixed + 1;
      WorkingSet.isActiveConstr[mFixed] = true;
      i = 14 * mFixed;
      mLB = WorkingSet.indexFixed[mFixed];
      if (mLB - 2 >= 0) {
        memset(&WorkingSet.ATwset[i], 0,
               (unsigned int)(((mLB + i) - i) - 1) * sizeof(double));
      }
      WorkingSet.ATwset[(WorkingSet.indexFixed[mFixed] + i) - 1] = 1.0;
      mLB = WorkingSet.indexFixed[mFixed] + 1;
      mUB = WorkingSet.nVar;
      if (mLB <= mUB) {
        memset(&WorkingSet.ATwset[(mLB + i) + -1], 0,
               (unsigned int)((((mUB + i) - mLB) - i) + 1) * sizeof(double));
      }
      WorkingSet.bwset[mFixed] =
          WorkingSet.ub[WorkingSet.indexFixed[mFixed] - 1];
    }
    b_factoryConstruct(TrialState.sqpFval, &MeritFunction);
    r.next.next.next.next.next.next.next.next.value.workspace =
        c_FcnEvaluator_next_next_next_n;
    d_driver(lb, ub, &TrialState, &MeritFunction, &r, &memspace, &WorkingSet,
             Hessian, &QRManager, &CholManager, &QPObjective);
    fval = TrialState.sqpFval;
    *exitflag = TrialState.sqpExitFlag;
    *output_iterations = TrialState.sqpIterations;
    *output_funcCount = TrialState.FunctionEvaluations;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = MeritFunction.nlpPrimalFeasError;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (i = 0; i < 13; i++) {
      double absxk;
      x0[i] = TrialState.xstarsqp[i];
      absxk = fabs(TrialState.delta_x[i]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        y += t * t;
      }
    }
    *output_stepsize = scale * sqrt(y);
    *output_lssteplength = TrialState.steplength;
    *output_firstorderopt = MeritFunction.firstOrderOpt;
    memset(&lambda->lower[0], 0, 13U * sizeof(double));
    memset(&lambda->upper[0], 0, 13U * sizeof(double));
    b_i = (unsigned char)WorkingSet.sizes[0];
    for (idx = 0; idx < b_i; idx++) {
      lambda->lower[WorkingSet.indexFixed[idx] - 1] =
          -TrialState.lambdaStopTest[idx];
    }
    b_i = (unsigned char)WorkingSet.sizes[3];
    for (idx = 0; idx < b_i; idx++) {
      lambda->lower[WorkingSet.indexLB[idx] - 1] =
          TrialState.lambdaStopTest[WorkingSet.sizes[0] + idx];
    }
    if ((unsigned char)WorkingSet.sizes[3] - 1 < 0) {
      i = WorkingSet.sizes[0];
    } else {
      i = WorkingSet.sizes[0] + (unsigned char)WorkingSet.sizes[3];
    }
    b_i = (unsigned char)WorkingSet.sizes[4];
    for (idx = 0; idx < b_i; idx++) {
      lambda->upper[WorkingSet.indexUB[idx] - 1] =
          TrialState.lambdaStopTest[i + idx];
    }
    memcpy(&grad[0], &TrialState.grad[0], 13U * sizeof(double));
  }
  return fval;
}

/*
 * Arguments    : d_struct_T *fun_workspace
 *                double x0[15]
 *                const double lb[15]
 *                const double ub[15]
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
double fmincon(d_struct_T *fun_workspace, double x0[15], const double lb[15],
               const double ub[15], double *exitflag, double *output_iterations,
               double *output_funcCount, char output_algorithm[3],
               double *output_constrviolation, double *output_stepsize,
               double *output_lssteplength, double *output_firstorderopt)
{
  b_struct_T MeritFunction;
  d_struct_T b_fun_workspace;
  f_struct_T QRManager;
  g_struct_T CholManager;
  h_struct_T memspace;
  i_coder_internal_stickyStruct r;
  n_struct_T TrialState;
  o_struct_T WorkingSet;
  struct_T QPObjective;
  double fval;
  int b_i;
  int i;
  int idx;
  int mUB;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  bool exitg1;
  *exitflag = rtInf;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 15)) {
    if (lb[i] > ub[i]) {
      *exitflag = -2.0;
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (*exitflag == -2.0) {
    fval = rtInf;
    *output_iterations = 0.0;
    *output_funcCount = 0.0;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = rtInf;
    *output_stepsize = rtInf;
    *output_lssteplength = rtInf;
    *output_firstorderopt = rtInf;
  } else {
    double scale;
    double y;
    int mFixed;
    int mLB;
    TrialState.nVarMax = 16;
    TrialState.mNonlinIneq = 0;
    TrialState.mNonlinEq = 0;
    TrialState.mIneq = 0;
    TrialState.mEq = 0;
    TrialState.iNonIneq0 = 1;
    TrialState.iNonEq0 = 1;
    TrialState.sqpFval_old = 0.0;
    TrialState.sqpIterations = 0;
    TrialState.sqpExitFlag = 0;
    memset(&TrialState.lambdasqp[0], 0, 31U * sizeof(double));
    TrialState.steplength = 1.0;
    memset(&TrialState.delta_x[0], 0, 16U * sizeof(double));
    TrialState.fstar = 0.0;
    TrialState.firstorderopt = 0.0;
    memset(&TrialState.lambda[0], 0, 31U * sizeof(double));
    TrialState.state = 0;
    TrialState.maxConstr = 0.0;
    TrialState.iterations = 0;
    memcpy(&TrialState.xstarsqp[0], &x0[0], 15U * sizeof(double));
    WorkingSet.nVar = 15;
    WorkingSet.nVarOrig = 15;
    WorkingSet.nVarMax = 16;
    WorkingSet.ldA = 16;
    memset(&WorkingSet.lb[0], 0, 16U * sizeof(double));
    memset(&WorkingSet.ub[0], 0, 16U * sizeof(double));
    WorkingSet.mEqRemoved = 0;
    memset(&WorkingSet.ATwset[0], 0, 496U * sizeof(double));
    WorkingSet.nActiveConstr = 0;
    memset(&WorkingSet.bwset[0], 0, 31U * sizeof(double));
    memset(&WorkingSet.maxConstrWorkspace[0], 0, 31U * sizeof(double));
    memset(&WorkingSet.Wid[0], 0, 31U * sizeof(int));
    memset(&WorkingSet.Wlocalidx[0], 0, 31U * sizeof(int));
    for (i = 0; i < 31; i++) {
      WorkingSet.isActiveConstr[i] = false;
    }
    for (i = 0; i < 5; i++) {
      WorkingSet.nWConstr[i] = 0;
    }
    WorkingSet.probType = 3;
    WorkingSet.SLACK0 = 1.0E-5;
    memset(&WorkingSet.indexLB[0], 0, 16U * sizeof(int));
    memset(&WorkingSet.indexUB[0], 0, 16U * sizeof(int));
    memset(&WorkingSet.indexFixed[0], 0, 16U * sizeof(int));
    mLB = 0;
    mUB = 0;
    mFixed = 0;
    for (idx = 0; idx < 15; idx++) {
      bool guard1;
      y = lb[idx];
      guard1 = false;
      if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
        if (fabs(y - ub[idx]) < 1.0E-6) {
          mFixed++;
          WorkingSet.indexFixed[mFixed - 1] = idx + 1;
        } else {
          mLB++;
          WorkingSet.indexLB[mLB - 1] = idx + 1;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        y = ub[idx];
        if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
          mUB++;
          WorkingSet.indexUB[mUB - 1] = idx + 1;
        }
      }
    }
    i = (mLB + mUB) + mFixed;
    WorkingSet.mConstr = i;
    WorkingSet.mConstrOrig = i;
    WorkingSet.mConstrMax = 31;
    obj_tmp[0] = (signed char)mFixed;
    obj_tmp[1] = 0;
    obj_tmp[2] = 0;
    obj_tmp[3] = (signed char)mLB;
    obj_tmp[4] = (signed char)mUB;
    b_obj_tmp[0] = (signed char)mFixed;
    b_obj_tmp[1] = 0;
    b_obj_tmp[2] = 0;
    b_obj_tmp[3] = (signed char)(mLB + 1);
    b_obj_tmp[4] = (signed char)mUB;
    WorkingSet.isActiveIdx[0] = 1;
    WorkingSet.isActiveIdx[1] = mFixed;
    WorkingSet.isActiveIdx[2] = 0;
    WorkingSet.isActiveIdx[3] = 0;
    WorkingSet.isActiveIdx[4] = mLB;
    WorkingSet.isActiveIdx[5] = mUB;
    for (i = 0; i < 5; i++) {
      signed char i1;
      signed char i2;
      i1 = obj_tmp[i];
      WorkingSet.sizes[i] = i1;
      WorkingSet.sizesNormal[i] = i1;
      i2 = b_obj_tmp[i];
      WorkingSet.sizesPhaseOne[i] = i2;
      WorkingSet.sizesRegularized[i] = i1;
      WorkingSet.sizesRegPhaseOne[i] = i2;
      WorkingSet.isActiveIdx[i + 1] += WorkingSet.isActiveIdx[i];
    }
    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxNormal[b_i] = WorkingSet.isActiveIdx[b_i];
    }
    WorkingSet.isActiveIdxPhaseOne[0] = 1;
    WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxPhaseOne[2] = 0;
    WorkingSet.isActiveIdxPhaseOne[3] = 0;
    WorkingSet.isActiveIdxPhaseOne[4] = mLB + 1;
    WorkingSet.isActiveIdxPhaseOne[5] = mUB;
    for (i = 0; i < 5; i++) {
      WorkingSet.isActiveIdxPhaseOne[i + 1] +=
          WorkingSet.isActiveIdxPhaseOne[i];
    }
    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
      WorkingSet.isActiveIdxRegPhaseOne[b_i] =
          WorkingSet.isActiveIdxPhaseOne[b_i];
    }
    for (idx = 0; idx < mLB; idx++) {
      b_i = WorkingSet.indexLB[idx];
      TrialState.xstarsqp[b_i - 1] =
          fmax(TrialState.xstarsqp[b_i - 1], lb[b_i - 1]);
    }
    for (idx = 0; idx < mUB; idx++) {
      b_i = WorkingSet.indexUB[idx];
      TrialState.xstarsqp[b_i - 1] =
          fmin(TrialState.xstarsqp[b_i - 1], ub[b_i - 1]);
    }
    for (idx = 0; idx < mFixed; idx++) {
      b_i = WorkingSet.indexFixed[idx];
      TrialState.xstarsqp[b_i - 1] = ub[b_i - 1];
    }
    b_fun_workspace = *fun_workspace;
    TrialState.sqpFval = evalObjAndConstrAndDerivatives(
        &b_fun_workspace, TrialState.xstarsqp, TrialState.grad, &i);
    TrialState.FunctionEvaluations = 1;
    for (idx = 0; idx < mLB; idx++) {
      WorkingSet.lb[WorkingSet.indexLB[idx] - 1] =
          -lb[WorkingSet.indexLB[idx] - 1] + x0[WorkingSet.indexLB[idx] - 1];
    }
    for (idx = 0; idx < mUB; idx++) {
      WorkingSet.ub[WorkingSet.indexUB[idx] - 1] =
          ub[WorkingSet.indexUB[idx] - 1] - x0[WorkingSet.indexUB[idx] - 1];
    }
    for (idx = 0; idx < mFixed; idx++) {
      y = ub[WorkingSet.indexFixed[idx] - 1] -
          x0[WorkingSet.indexFixed[idx] - 1];
      WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = y;
      WorkingSet.bwset[idx] = y;
    }
    setProblemType(&WorkingSet, 3);
    i = WorkingSet.isActiveIdx[2];
    for (idx = i; idx < 32; idx++) {
      WorkingSet.isActiveConstr[idx - 1] = false;
    }
    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    b_i = (unsigned char)WorkingSet.sizes[0];
    for (mUB = 0; mUB < b_i; mUB++) {
      WorkingSet.Wid[mUB] = 1;
      WorkingSet.Wlocalidx[mUB] = mUB + 1;
      WorkingSet.isActiveConstr[mUB] = true;
      i = mUB << 4;
      idx = WorkingSet.indexFixed[mUB];
      if (idx - 2 >= 0) {
        memset(&WorkingSet.ATwset[i], 0,
               (unsigned int)(((idx + i) - i) - 1) * sizeof(double));
      }
      WorkingSet.ATwset[(WorkingSet.indexFixed[mUB] + i) - 1] = 1.0;
      idx = WorkingSet.indexFixed[mUB] + 1;
      mLB = WorkingSet.nVar;
      if (idx <= mLB) {
        memset(&WorkingSet.ATwset[(idx + i) + -1], 0,
               (unsigned int)((((mLB + i) - idx) - i) + 1) * sizeof(double));
      }
      WorkingSet.bwset[mUB] = WorkingSet.ub[WorkingSet.indexFixed[mUB] - 1];
    }
    double Hessian[225];
    b_factoryConstruct(TrialState.sqpFval, &MeritFunction);
    r.next.next.next.next.next.next.next.next.value.workspace = *fun_workspace;
    c_driver(lb, ub, &TrialState, &MeritFunction, &r, &memspace, &WorkingSet,
             Hessian, &QRManager, &CholManager, &QPObjective);
    fval = TrialState.sqpFval;
    *exitflag = TrialState.sqpExitFlag;
    *output_iterations = TrialState.sqpIterations;
    *output_funcCount = TrialState.FunctionEvaluations;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = MeritFunction.nlpPrimalFeasError;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (i = 0; i < 15; i++) {
      double absxk;
      x0[i] = TrialState.xstarsqp[i];
      absxk = fabs(TrialState.delta_x[i]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        y += t * t;
      }
    }
    *output_stepsize = scale * sqrt(y);
    *output_lssteplength = TrialState.steplength;
    *output_firstorderopt = MeritFunction.firstOrderOpt;
  }
  return fval;
}

/*
 * File trailer for fmincon.c
 *
 * [EOF]
 */
