/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: iterate.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "iterate.h"
#include "addAineqConstr.h"
#include "addBoundToActiveSetMatrix_.h"
#include "checkStoppingAndUpdateFval.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "factorQR.h"
#include "feasibleratiotest.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "xrotg.h"
#include "xtrsv.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double H[324]
 *                const double f_data[]
 *                c_struct_T *solution
 *                e_struct_T *memspace
 *                f_struct_T *workingset
 *                g_struct_T *qrmanager
 *                h_struct_T *cholmanager
 *                d_struct_T *objective
 *                const char options_SolverName[7]
 *                double options_StepTolerance
 *                double options_ObjectiveLimit
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
void iterate(const double H[324], const double f_data[], c_struct_T *solution,
             e_struct_T *memspace, f_struct_T *workingset,
             g_struct_T *qrmanager, h_struct_T *cholmanager,
             d_struct_T *objective, const char options_SolverName[7],
             double options_StepTolerance, double options_ObjectiveLimit,
             int runTimeOptions_MaxIterations)
{
  static const char b[7] = {'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  double y_data[311];
  double d;
  double s;
  int TYPE;
  int activeSetChangeID;
  int b_k;
  int globalActiveConstrIdx;
  int i;
  int idx;
  int iy;
  int iyend;
  int k;
  int nVar;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double,
                                        f_data, solution->xstar.data);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }
  solution->lambda.size[0] = 311;
  memset(&solution->lambda.data[0], 0, 311U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      double temp;
      int Qk0;
      bool guard1;
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1: {
          double c;
          int ix0;
          ix0 = 183 * (workingset->nActiveConstr - 1);
          Qk0 = qrmanager->mrows;
          iyend = qrmanager->ncols + 1;
          if (Qk0 <= iyend) {
            iyend = Qk0;
          }
          qrmanager->minRowCol = iyend;
          Qk0 = 311 * qrmanager->ncols;
          if (qrmanager->mrows != 0) {
            iyend = Qk0 + qrmanager->mrows;
            for (iy = Qk0 + 1; iy <= iyend; iy++) {
              qrmanager->QR->data[iy - 1] = 0.0;
            }
            i = 311 * (qrmanager->mrows - 1) + 1;
            for (iyend = 1; iyend <= i; iyend += 311) {
              c = 0.0;
              iy = (iyend + qrmanager->mrows) - 1;
              for (idx = iyend; idx <= iy; idx++) {
                c += qrmanager->Q->data[idx - 1] *
                     workingset->ATwset->data[(ix0 + idx) - iyend];
              }
              iy = Qk0 + div_nde_s32_floor(iyend - 1, 311);
              qrmanager->QR->data[iy] += c;
            }
          }
          qrmanager->ncols++;
          qrmanager->jpvt.data[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            i = idx + 311 * (qrmanager->ncols - 1);
            d = qrmanager->QR->data[i + 1];
            c = xrotg(&qrmanager->QR->data[i], &d, &s);
            qrmanager->QR->data[i + 1] = d;
            Qk0 = 311 * idx;
            ix0 = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              for (k = 0; k < ix0; k++) {
                iy = Qk0 + k;
                temp = c * qrmanager->Q->data[iy] +
                       s * qrmanager->Q->data[iy + 311];
                qrmanager->Q->data[iy + 311] =
                    c * qrmanager->Q->data[iy + 311] -
                    s * qrmanager->Q->data[iy];
                qrmanager->Q->data[iy] = temp;
              }
            }
          }
        } break;
        case -1: {
          idx = globalActiveConstrIdx;
          if (qrmanager->usedPivoting) {
            idx = 1;
            while ((idx <= qrmanager->ncols) &&
                   (qrmanager->jpvt.data[idx - 1] != globalActiveConstrIdx)) {
              idx++;
            }
          }
          if (idx >= qrmanager->ncols) {
            qrmanager->ncols--;
          } else {
            i = qrmanager->ncols - 1;
            qrmanager->jpvt.data[idx - 1] = qrmanager->jpvt.data[i];
            iy = qrmanager->minRowCol;
            for (k = 0; k < iy; k++) {
              qrmanager->QR->data[k + 311 * (idx - 1)] =
                  qrmanager->QR->data[k + 311 * i];
            }
            qrmanager->ncols = i;
            Qk0 = qrmanager->mrows;
            iyend = qrmanager->ncols;
            if (Qk0 <= iyend) {
              iyend = Qk0;
            }
            qrmanager->minRowCol = iyend;
            if (idx < qrmanager->mrows) {
              double c;
              int endIdx;
              int ix0;
              Qk0 = qrmanager->mrows - 1;
              endIdx = qrmanager->ncols;
              if (Qk0 <= endIdx) {
                endIdx = Qk0;
              }
              k = endIdx;
              iyend = 311 * (idx - 1);
              while (k >= idx) {
                i = k + iyend;
                d = qrmanager->QR->data[i];
                c = xrotg(&qrmanager->QR->data[i - 1], &d, &s);
                qrmanager->QR->data[i] = d;
                i = 311 * (k - 1);
                qrmanager->QR->data[k + i] = 0.0;
                Qk0 = k + 311 * idx;
                ix0 = qrmanager->ncols - idx;
                if (ix0 >= 1) {
                  for (b_k = 0; b_k < ix0; b_k++) {
                    iy = Qk0 + b_k * 311;
                    temp = c * qrmanager->QR->data[iy - 1] +
                           s * qrmanager->QR->data[iy];
                    qrmanager->QR->data[iy] = c * qrmanager->QR->data[iy] -
                                              s * qrmanager->QR->data[iy - 1];
                    qrmanager->QR->data[iy - 1] = temp;
                  }
                }
                ix0 = qrmanager->mrows;
                for (b_k = 0; b_k < ix0; b_k++) {
                  iy = i + b_k;
                  temp = c * qrmanager->Q->data[iy] +
                         s * qrmanager->Q->data[iy + 311];
                  qrmanager->Q->data[iy + 311] =
                      c * qrmanager->Q->data[iy + 311] -
                      s * qrmanager->Q->data[iy];
                  qrmanager->Q->data[iy] = temp;
                }
                k--;
              }
              i = idx + 1;
              for (k = i; k <= endIdx; k++) {
                iyend = 311 * (k - 1);
                iy = k + iyend;
                d = qrmanager->QR->data[iy];
                c = xrotg(&qrmanager->QR->data[iy - 1], &d, &s);
                qrmanager->QR->data[iy] = d;
                Qk0 = k * 312;
                ix0 = qrmanager->ncols - k;
                if (ix0 >= 1) {
                  for (b_k = 0; b_k < ix0; b_k++) {
                    iy = Qk0 + b_k * 311;
                    temp = c * qrmanager->QR->data[iy - 1] +
                           s * qrmanager->QR->data[iy];
                    qrmanager->QR->data[iy] = c * qrmanager->QR->data[iy] -
                                              s * qrmanager->QR->data[iy - 1];
                    qrmanager->QR->data[iy - 1] = temp;
                  }
                }
                ix0 = qrmanager->mrows;
                for (b_k = 0; b_k < ix0; b_k++) {
                  iy = iyend + b_k;
                  temp = c * qrmanager->Q->data[iy] +
                         s * qrmanager->Q->data[iy + 311];
                  qrmanager->Q->data[iy + 311] =
                      c * qrmanager->Q->data[iy + 311] -
                      s * qrmanager->Q->data[iy];
                  qrmanager->Q->data[iy] = temp;
                }
              }
            }
          }
        } break;
        default:
          factorQR(qrmanager, workingset->ATwset, nVar,
                   workingset->nActiveConstr);
          computeQ_(qrmanager, qrmanager->mrows);
          break;
        }
        iyend = memcmp(&options_SolverName[0], &b[0], 7);
        compute_deltax(H, solution, memspace, qrmanager, cholmanager, objective,
                       iyend == 0);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((b_xnrm2(nVar, solution->searchDir.data) <
                    options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar)) {
          guard1 = true;
        } else {
          temp = feasibleratiotest(
              solution->xstar.data, solution->searchDir.data,
              memspace->workspace_double, workingset->nVar, workingset->Aineq,
              workingset->bineq.data, workingset->lb.data,
              workingset->indexLB.data, workingset->sizes,
              workingset->isActiveIdx, workingset->isActiveConstr.data,
              workingset->nWConstr, TYPE == 5, &updateFval, &i, &iyend);
          if (updateFval) {
            switch (i) {
            case 3:
              addAineqConstr(workingset, iyend);
              break;
            case 4:
              addBoundToActiveSetMatrix_(workingset, 4, iyend);
              break;
            default:
              addBoundToActiveSetMatrix_(workingset, 5, iyend);
              break;
            }
            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (b_xnrm2(objective->nvar, solution->searchDir.data) >
                  100.0 * (double)objective->nvar * 1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }
          if ((nVar >= 1) && (!(temp == 0.0))) {
            iyend = nVar - 1;
            for (k = 0; k <= iyend; k++) {
              solution->xstar.data[k] += temp * solution->searchDir.data[k];
            }
          }
          computeGrad_StoreHx(objective, H, f_data, solution->xstar.data);
          updateFval = true;
          checkStoppingAndUpdateFval(&activeSetChangeID, f_data, solution,
                                     memspace, objective, workingset, qrmanager,
                                     options_ObjectiveLimit,
                                     runTimeOptions_MaxIterations, updateFval);
        }
      } else {
        iyend = solution->searchDir.size[0];
        Qk0 = solution->searchDir.size[0];
        if (Qk0 - 1 >= 0) {
          memcpy(&y_data[0], &solution->searchDir.data[0],
                 (unsigned int)Qk0 * sizeof(double));
        }
        memset(&y_data[0], 0, (unsigned int)nVar * sizeof(double));
        if (iyend - 1 >= 0) {
          memcpy(&solution->searchDir.data[0], &y_data[0],
                 (unsigned int)iyend * sizeof(double));
        }
        guard1 = true;
      }
      if (guard1) {
        iyend = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          bool b_guard1;
          b_guard1 = false;
          if (objective->objtype != 4) {
            temp = 100.0 * (double)qrmanager->mrows * 2.2204460492503131E-16;
            if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }
            if (updateFval) {
              bool guard2;
              idx = qrmanager->ncols;
              guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                Qk0 = qrmanager->mrows + 311 * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) &&
                       (fabs(qrmanager->QR->data[Qk0 - 1]) >= temp)) {
                  idx--;
                  Qk0 -= 311;
                }
                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
              if (guard2) {
                Qk0 = idx + 311 * (idx - 1);
                while ((idx >= 1) &&
                       (fabs(qrmanager->QR->data[Qk0 - 1]) >= temp)) {
                  idx--;
                  Qk0 -= 312;
                }
                updateFval = (idx == 0);
              }
            }
            if (!updateFval) {
              solution->state = -7;
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }
          if (b_guard1) {
            b_xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q,
                    objective->grad.data, memspace->workspace_double);
            xtrsv(qrmanager->ncols, qrmanager->QR, memspace->workspace_double);
            for (idx = 0; idx < iyend; idx++) {
              solution->lambda.data[idx] =
                  -memspace->workspace_double->data[idx];
            }
          }
        }
        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          Qk0 = 0;
          temp = 0.0;
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          iy = workingset->nActiveConstr;
          for (idx = i; idx <= iy; idx++) {
            d = solution->lambda.data[idx - 1];
            if (d < temp) {
              temp = d;
              Qk0 = idx;
            }
          }
          if (Qk0 == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = Qk0;
            subProblemChanged = true;
            removeConstr(workingset, Qk0);
            solution->lambda.data[Qk0 - 1] = 0.0;
          }
        } else {
          Qk0 = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda.data[Qk0 - 1] = 0.0;
        }
        updateFval = false;
        checkStoppingAndUpdateFval(&activeSetChangeID, f_data, solution,
                                   memspace, objective, workingset, qrmanager,
                                   options_ObjectiveLimit,
                                   runTimeOptions_MaxIterations, updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar =
            computeFval_ReuseHx(objective, memspace->workspace_double, f_data,
                                solution->xstar.data);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/*
 * File trailer for iterate.c
 *
 * [EOF]
 */
