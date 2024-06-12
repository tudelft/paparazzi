/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: test_exit.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "test_exit.h"
#include "computeGradLag.h"
#include "computeQ_.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "xgemv.h"
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : i_struct_T *Flags
 *                e_struct_T *memspace
 *                struct_T *MeritFunction
 *                f_struct_T *WorkingSet
 *                c_struct_T *TrialState
 *                g_struct_T *QRManager
 * Return Type  : void
 */
void b_test_exit(i_struct_T *Flags, e_struct_T *memspace,
                 struct_T *MeritFunction, f_struct_T *WorkingSet,
                 c_struct_T *TrialState, g_struct_T *QRManager)
{
  emxArray_real_T *y;
  double b_y_data[311];
  double y_data[311];
  double optimRelativeFactor;
  double s;
  double smax;
  double *c_y_data;
  int b_i;
  int i;
  int idx;
  int idx_max;
  int k;
  int nVar;
  bool dxTooSmall;
  bool exitg1;
  bool isFeasible;
  nVar = WorkingSet->nVar;
  memcpy(&y_data[0], &TrialState->lambdaStopTest.data[0],
         311U * sizeof(double));
  i = (unsigned short)(WorkingSet->sizes[3] + 146);
  if (i - 1 >= 0) {
    memcpy(&y_data[0], &TrialState->lambdasqp.data[0],
           (unsigned int)i * sizeof(double));
  }
  TrialState->lambdaStopTest.size[0] = 311;
  memcpy(&TrialState->lambdaStopTest.data[0], &y_data[0],
         311U * sizeof(double));
  b_computeGradLag(TrialState->gradLag.data, WorkingSet->nVar,
                   TrialState->grad.data, WorkingSet->Aineq,
                   WorkingSet->Aeq.data, WorkingSet->indexLB.data,
                   WorkingSet->sizes[3], y_data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad.data[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad.data[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  optimRelativeFactor = fmax(1.0, fabs(TrialState->grad.data[idx_max - 1]));
  if (rtIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  smax = 0.0;
  for (idx = 0; idx < 18; idx++) {
    smax = fmax(smax, fabs(TrialState->cEq.data[idx]));
  }
  for (idx = 0; idx < 128; idx++) {
    smax = fmax(smax, TrialState->cIneq.data[idx]);
  }
  MeritFunction->nlpPrimalFeasError = smax;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  }
  isFeasible = (smax <= 0.001 * MeritFunction->feasRelativeFactor);
  dxTooSmall = true;
  smax = 0.0;
  idx_max = (unsigned char)WorkingSet->nVar;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= idx_max - 1)) {
    dxTooSmall = ((!rtIsInf(TrialState->gradLag.data[idx])) &&
                  (!rtIsNaN(TrialState->gradLag.data[idx])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag.data[idx]));
      idx++;
    }
  }
  MeritFunction->nlpDualFeasError = smax;
  emxInit_real_T(&y, 2);
  if (!dxTooSmall) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    double d;
    double nlpComplErrorTmp;
    s = 0.0;
    for (idx = 0; idx < 128; idx++) {
      d = TrialState->cIneq.data[idx];
      nlpComplErrorTmp = y_data[idx + 18];
      s = fmax(
          s, fmin(fabs(d * nlpComplErrorTmp), fmin(fabs(d), nlpComplErrorTmp)));
    }
    MeritFunction->nlpComplError = s;
    d = fmax(smax, s);
    MeritFunction->firstOrderOpt = d;
    if (TrialState->sqpIterations > 1) {
      computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                     TrialState->grad.data, WorkingSet->Aineq,
                     WorkingSet->Aeq.data, WorkingSet->indexLB.data,
                     WorkingSet->sizes[3], TrialState->lambdaStopTestPrev.data);
      s = 0.0;
      idx = 0;
      while ((idx <= idx_max - 1) &&
             ((!rtIsInf(memspace->workspace_double->data[idx])) &&
              (!rtIsNaN(memspace->workspace_double->data[idx])))) {
        s = fmax(s, fabs(memspace->workspace_double->data[idx]));
        idx++;
      }
      nlpComplErrorTmp = 0.0;
      for (idx = 0; idx < 128; idx++) {
        smax = TrialState->lambdaStopTestPrev.data[idx + 18];
        nlpComplErrorTmp =
            fmax(nlpComplErrorTmp,
                 fmin(fabs(TrialState->cIneq.data[idx] * smax),
                      fmin(fabs(TrialState->cIneq.data[idx]), smax)));
      }
      smax = fmax(s, nlpComplErrorTmp);
      if (smax < d) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = smax;
        if (i - 1 >= 0) {
          memcpy(&y_data[0], &TrialState->lambdaStopTestPrev.data[0],
                 (unsigned int)i * sizeof(double));
        }
        TrialState->lambdaStopTest.size[0] = 311;
        memcpy(&TrialState->lambdaStopTest.data[0], &y_data[0],
               311U * sizeof(double));
      } else {
        memcpy(&b_y_data[0], &TrialState->lambdaStopTestPrev.data[0],
               311U * sizeof(double));
        if (i - 1 >= 0) {
          memcpy(&b_y_data[0], &y_data[0], (unsigned int)i * sizeof(double));
        }
        TrialState->lambdaStopTestPrev.size[0] = 311;
        memcpy(&TrialState->lambdaStopTestPrev.data[0], &b_y_data[0],
               311U * sizeof(double));
      }
    } else {
      memcpy(&b_y_data[0], &TrialState->lambdaStopTestPrev.data[0],
             311U * sizeof(double));
      if (i - 1 >= 0) {
        memcpy(&b_y_data[0], &y_data[0], (unsigned int)i * sizeof(double));
      }
      TrialState->lambdaStopTestPrev.size[0] = 311;
      memcpy(&TrialState->lambdaStopTestPrev.data[0], &b_y_data[0],
             311U * sizeof(double));
    }
    if (isFeasible &&
        (MeritFunction->nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        bool guard1;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          idx = 0;
          exitg1 = false;
          while ((!exitg1) && (idx <= idx_max - 1)) {
            if (1.0E-6 * fmax(1.0, fabs(TrialState->xstarsqp[idx])) <=
                fabs(TrialState->delta_x.data[idx])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx++;
            }
          }
          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              int nActiveConstr;
              nActiveConstr = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr > 0) {
                int ix;
                int rankR;
                updateWorkingSetForNewQP(WorkingSet, TrialState->cIneq.data,
                                         TrialState->cEq.data);
                memcpy(&y_data[0], &TrialState->lambda.data[0],
                       311U * sizeof(double));
                memset(&y_data[0], 0,
                       (unsigned int)nActiveConstr * sizeof(double));
                TrialState->lambda.size[0] = 311;
                memcpy(&TrialState->lambda.data[0], &y_data[0],
                       311U * sizeof(double));
                for (idx = 0; idx < nActiveConstr; idx++) {
                  ix = 183 * idx;
                  rankR = 311 * idx;
                  k = y->size[0] * y->size[1];
                  y->size[0] = 311;
                  y->size[1] = 311;
                  emxEnsureCapacity_real_T(y, k);
                  c_y_data = y->data;
                  for (k = 0; k < 96721; k++) {
                    c_y_data[k] = QRManager->QR->data[k];
                  }
                  for (k = 0; k < idx_max; k++) {
                    c_y_data[rankR + k] = WorkingSet->ATwset->data[ix + k];
                  }
                  k = QRManager->QR->size[0] * QRManager->QR->size[1];
                  QRManager->QR->size[0] = 311;
                  QRManager->QR->size[1] = 311;
                  emxEnsureCapacity_real_T(QRManager->QR, k);
                  for (k = 0; k < 96721; k++) {
                    QRManager->QR->data[k] = c_y_data[k];
                  }
                }
                QRManager->usedPivoting = true;
                QRManager->mrows = nVar;
                QRManager->ncols = nActiveConstr;
                if (nVar <= nActiveConstr) {
                  k = nVar;
                } else {
                  k = nActiveConstr;
                }
                QRManager->minRowCol = k;
                QRManager->tau.size[0] =
                    xgeqp3(QRManager->QR, nVar, nActiveConstr,
                           QRManager->jpvt.data, QRManager->tau.data);
                computeQ_(QRManager, nVar);
                b_xgemv(nVar, nVar, QRManager->Q, TrialState->grad.data,
                        memspace->workspace_double);
                if (nVar >= nActiveConstr) {
                  idx_max = nVar;
                } else {
                  idx_max = nActiveConstr;
                }
                smax = fabs(QRManager->QR->data[0]) *
                       fmin(1.4901161193847656E-8,
                            (double)idx_max * 2.2204460492503131E-16);
                rankR = 0;
                idx_max = 0;
                while ((rankR < k) &&
                       (fabs(QRManager->QR->data[idx_max]) > smax)) {
                  rankR++;
                  idx_max += 312;
                }
                if (rankR != 0) {
                  for (idx = rankR; idx >= 1; idx--) {
                    idx_max = (idx + (idx - 1) * 311) - 1;
                    memspace->workspace_double->data[idx - 1] /=
                        QRManager->QR->data[idx_max];
                    for (b_i = 0; b_i <= idx - 2; b_i++) {
                      ix = (idx - b_i) - 2;
                      memspace->workspace_double->data[ix] -=
                          memspace->workspace_double->data[idx - 1] *
                          QRManager->QR->data[(idx_max - b_i) - 1];
                    }
                  }
                }
                if (nActiveConstr <= k) {
                  k = nActiveConstr;
                }
                idx_max = (unsigned char)k;
                for (idx = 0; idx < idx_max; idx++) {
                  TrialState->lambda.data[QRManager->jpvt.data[idx] - 1] =
                      memspace->workspace_double->data[idx];
                }
                for (idx = 0; idx < 18; idx++) {
                  TrialState->lambda.data[idx] = -TrialState->lambda.data[idx];
                }
                sortLambdaQP(
                    TrialState->lambda.data, &TrialState->lambda.size[0],
                    WorkingSet->nActiveConstr, WorkingSet->sizes,
                    WorkingSet->isActiveIdx, WorkingSet->Wid.data,
                    WorkingSet->Wlocalidx.data, memspace->workspace_double);
                computeGradLag(memspace->workspace_double, nVar,
                               TrialState->grad.data, WorkingSet->Aineq,
                               WorkingSet->Aeq.data, WorkingSet->indexLB.data,
                               WorkingSet->sizes[3], TrialState->lambda.data);
                smax = 0.0;
                idx = 0;
                while ((idx <= (unsigned char)nVar - 1) &&
                       ((!rtIsInf(memspace->workspace_double->data[idx])) &&
                        (!rtIsNaN(memspace->workspace_double->data[idx])))) {
                  smax =
                      fmax(smax, fabs(memspace->workspace_double->data[idx]));
                  idx++;
                }
                s = 0.0;
                for (idx = 0; idx < 128; idx++) {
                  s = fmax(s, fmin(fabs(TrialState->cIneq.data[idx] *
                                        TrialState->lambda.data[idx]),
                                   fmin(fabs(TrialState->cIneq.data[idx]),
                                        TrialState->lambda.data[idx])));
                }
                if ((smax <= 1.0E-6 * optimRelativeFactor) &&
                    (s <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = smax;
                  MeritFunction->nlpComplError = s;
                  MeritFunction->firstOrderOpt = fmax(smax, s);
                  memcpy(&y_data[0], &TrialState->lambdaStopTest.data[0],
                         311U * sizeof(double));
                  if (i - 1 >= 0) {
                    memcpy(&y_data[0], &TrialState->lambda.data[0],
                           (unsigned int)i * sizeof(double));
                  }
                  TrialState->lambdaStopTest.size[0] = 311;
                  memcpy(&TrialState->lambdaStopTest.data[0], &y_data[0],
                         311U * sizeof(double));
                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (TrialState->sqpIterations >= 50) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 1800) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
  emxFree_real_T(&y);
}

/*
 * Arguments    : struct_T *MeritFunction
 *                const f_struct_T *WorkingSet
 *                c_struct_T *TrialState
 *                bool *Flags_fevalOK
 *                bool *Flags_done
 *                bool *Flags_stepAccepted
 *                bool *Flags_failedLineSearch
 *                int *Flags_stepType
 * Return Type  : bool
 */
bool test_exit(struct_T *MeritFunction, const f_struct_T *WorkingSet,
               c_struct_T *TrialState, bool *Flags_fevalOK, bool *Flags_done,
               bool *Flags_stepAccepted, bool *Flags_failedLineSearch,
               int *Flags_stepType)
{
  double b_y_data[311];
  double y_data[311];
  double s;
  double smax;
  int i;
  int idx_max;
  int k;
  int nVar;
  bool Flags_gradOK;
  bool exitg1;
  bool isFeasible;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  memcpy(&y_data[0], &TrialState->lambdaStopTest.data[0],
         311U * sizeof(double));
  i = (unsigned short)(WorkingSet->sizes[3] + 146);
  if (i - 1 >= 0) {
    memcpy(&y_data[0], &TrialState->lambdasqp.data[0],
           (unsigned int)i * sizeof(double));
  }
  TrialState->lambdaStopTest.size[0] = 311;
  memcpy(&TrialState->lambdaStopTest.data[0], &y_data[0],
         311U * sizeof(double));
  b_computeGradLag(TrialState->gradLag.data, WorkingSet->nVar,
                   TrialState->grad.data, WorkingSet->Aineq,
                   WorkingSet->Aeq.data, WorkingSet->indexLB.data,
                   WorkingSet->sizes[3], y_data);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad.data[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad.data[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  s = fmax(1.0, fabs(TrialState->grad.data[idx_max - 1]));
  if (rtIsInf(s)) {
    s = 1.0;
  }
  smax = 0.0;
  for (idx_max = 0; idx_max < 18; idx_max++) {
    smax = fmax(smax, fabs(TrialState->cEq.data[idx_max]));
  }
  for (idx_max = 0; idx_max < 128; idx_max++) {
    smax = fmax(smax, TrialState->cIneq.data[idx_max]);
  }
  MeritFunction->nlpPrimalFeasError = smax;
  MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  isFeasible = (smax <= 0.001 * MeritFunction->feasRelativeFactor);
  Flags_gradOK = true;
  smax = 0.0;
  nVar = (unsigned char)WorkingSet->nVar;
  idx_max = 0;
  exitg1 = false;
  while ((!exitg1) && (idx_max <= nVar - 1)) {
    Flags_gradOK = ((!rtIsInf(TrialState->gradLag.data[idx_max])) &&
                    (!rtIsNaN(TrialState->gradLag.data[idx_max])));
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag.data[idx_max]));
      idx_max++;
    }
  }
  MeritFunction->nlpDualFeasError = smax;
  if (!Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = smax;
    memcpy(&b_y_data[0], &TrialState->lambdaStopTestPrev.data[0],
           311U * sizeof(double));
    if (i - 1 >= 0) {
      memcpy(&b_y_data[0], &y_data[0], (unsigned int)i * sizeof(double));
    }
    TrialState->lambdaStopTestPrev.size[0] = 311;
    memcpy(&TrialState->lambdaStopTestPrev.data[0], &b_y_data[0],
           311U * sizeof(double));
    if (isFeasible && (smax <= 1.0E-6 * s)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }
  return Flags_gradOK;
}

/*
 * File trailer for test_exit.c
 *
 * [EOF]
 */
