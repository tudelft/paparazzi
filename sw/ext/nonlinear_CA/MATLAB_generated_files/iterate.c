/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: iterate.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

/* Include Files */
#include "iterate.h"
#include "Nonlinear_controller_w_ail_basic_aero_sl_internal_types.h"
#include "Nonlinear_controller_w_ail_basic_aero_sl_rtwutil.h"
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "xrotg.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double H[225]
 *                const double f[16]
 *                h_struct_T *solution
 *                f_struct_T *memspace
 *                i_struct_T *workingset
 *                d_struct_T *qrmanager
 *                e_struct_T *cholmanager
 *                struct_T *objective
 *                const char options_SolverName[7]
 *                double options_StepTolerance
 *                double options_ObjectiveLimit
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
void iterate(const double H[225], const double f[16], h_struct_T *solution,
             f_struct_T *memspace, i_struct_T *workingset,
             d_struct_T *qrmanager, e_struct_T *cholmanager,
             struct_T *objective, const char options_SolverName[7],
             double options_StepTolerance, double options_ObjectiveLimit,
             int runTimeOptions_MaxIterations)
{
  static const char b[7] = {'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  double s;
  double temp;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int i;
  int idx;
  int iyend;
  int nActiveConstr;
  int nVar;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double,
                                        f, solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }
  memset(&solution->lambda[0], 0, 31U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      double minLambda;
      int idxQR;
      int ix0;
      bool guard1;
      bool guard2;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1: {
          double c;
          ix0 = (workingset->nActiveConstr - 1) << 4;
          iyend = qrmanager->mrows;
          idxQR = qrmanager->ncols + 1;
          if (iyend <= idxQR) {
            idxQR = iyend;
          }
          qrmanager->minRowCol = idxQR;
          idxQR = 31 * qrmanager->ncols;
          if (qrmanager->mrows != 0) {
            iyend = idxQR + qrmanager->mrows;
            if (idxQR + 1 <= iyend) {
              memset(&qrmanager->QR[idxQR], 0,
                     (unsigned int)(iyend - idxQR) * sizeof(double));
            }
            i = 31 * (qrmanager->mrows - 1) + 1;
            for (iyend = 1; iyend <= i; iyend += 31) {
              c = 0.0;
              nActiveConstr = (iyend + qrmanager->mrows) - 1;
              for (idx = iyend; idx <= nActiveConstr; idx++) {
                c += qrmanager->Q[idx - 1] *
                     workingset->ATwset[(ix0 + idx) - iyend];
              }
              nActiveConstr = idxQR + div_nde_s32_floor(iyend - 1);
              qrmanager->QR[nActiveConstr] += c;
            }
          }
          qrmanager->ncols++;
          i = qrmanager->ncols - 1;
          qrmanager->jpvt[i] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            nActiveConstr = idx + 31 * i;
            temp = qrmanager->QR[nActiveConstr + 1];
            c = xrotg(&qrmanager->QR[nActiveConstr], &temp, &s);
            qrmanager->QR[nActiveConstr + 1] = temp;
            iyend = 31 * idx;
            ix0 = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              for (nActiveConstr = 0; nActiveConstr < ix0; nActiveConstr++) {
                idxQR = iyend + nActiveConstr;
                minLambda = qrmanager->Q[idxQR + 31];
                temp = c * qrmanager->Q[idxQR] + s * minLambda;
                qrmanager->Q[idxQR + 31] =
                    c * minLambda - s * qrmanager->Q[idxQR];
                qrmanager->Q[idxQR] = temp;
              }
            }
          }
        } break;
        case -1:
          deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;
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
        } else if ((b_xnrm2(nVar, solution->searchDir) <
                    options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar)) {
          guard2 = true;
        } else {
          minLambda = feasibleratiotest(
              solution->xstar, solution->searchDir, workingset->nVar,
              workingset->lb, workingset->ub, workingset->indexLB,
              workingset->indexUB, workingset->sizes, workingset->isActiveIdx,
              workingset->isActiveConstr, workingset->nWConstr, TYPE == 5,
              &updateFval, &i, &iyend);
          if (updateFval) {
            switch (i) {
            case 3:
              workingset->nWConstr[2]++;
              workingset
                  ->isActiveConstr[(workingset->isActiveIdx[2] + iyend) - 2] =
                  true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] = iyend;
              /* A check that is always false is detected at compile-time.
               * Eliminating code that follows. */
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
              if (b_xnrm2(objective->nvar, solution->searchDir) >
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
          if ((nVar >= 1) && (!(minLambda == 0.0))) {
            iyend = nVar - 1;
            for (nActiveConstr = 0; nActiveConstr <= iyend; nActiveConstr++) {
              solution->xstar[nActiveConstr] +=
                  minLambda * solution->searchDir[nActiveConstr];
            }
          }
          computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
          guard1 = true;
        }
      } else {
        i = (unsigned char)nVar;
        memset(&solution->searchDir[0], 0, (unsigned int)i * sizeof(double));
        guard2 = true;
      }
      if (guard2) {
        nActiveConstr = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          bool b_guard1;
          b_guard1 = false;
          if (objective->objtype != 4) {
            minLambda =
                100.0 * (double)qrmanager->mrows * 2.2204460492503131E-16;
            if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }
            if (updateFval) {
              bool b_guard2;
              idx = qrmanager->ncols;
              b_guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                idxQR = qrmanager->mrows + 31 * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) &&
                       (fabs(qrmanager->QR[idxQR - 1]) >= minLambda)) {
                  idx--;
                  idxQR -= 31;
                }
                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }
              if (b_guard2) {
                idxQR = idx + 31 * (idx - 1);
                while ((idx >= 1) &&
                       (fabs(qrmanager->QR[idxQR - 1]) >= minLambda)) {
                  idx--;
                  idxQR -= 32;
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
            ix0 = qrmanager->ncols;
            b_xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q,
                    objective->grad, memspace->workspace_double);
            if (qrmanager->ncols != 0) {
              for (idx = ix0; idx >= 1; idx--) {
                iyend = (idx + (idx - 1) * 31) - 1;
                memspace->workspace_double[idx - 1] /= qrmanager->QR[iyend];
                for (i = 0; i <= idx - 2; i++) {
                  idxQR = (idx - i) - 2;
                  memspace->workspace_double[idxQR] -=
                      memspace->workspace_double[idx - 1] *
                      qrmanager->QR[(iyend - i) - 1];
                }
              }
            }
            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }
        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          iyend = 0;
          minLambda = 0.0;
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          nActiveConstr = workingset->nActiveConstr;
          for (idx = i; idx <= nActiveConstr; idx++) {
            temp = solution->lambda[idx - 1];
            if (temp < minLambda) {
              minLambda = temp;
              iyend = idx;
            }
          }
          if (iyend == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iyend;
            subProblemChanged = true;
            removeConstr(workingset, iyend);
            solution->lambda[iyend - 1] = 0.0;
          }
        } else {
          iyend = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda[iyend - 1] = 0.0;
        }
        updateFval = false;
        guard1 = true;
      }
      if (guard1) {
        solution->iterations++;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }
        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr =
              b_maxConstraintViolation(workingset, solution->xstar);
          minLambda = solution->maxConstr;
          if (objective->objtype == 5) {
            minLambda =
                solution->maxConstr - solution->xstar[objective->nvar - 1];
          }
          if (minLambda > 1.0E-6) {
            bool nonDegenerateWset;
            i = (unsigned char)objective->nvar;
            if (i - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0],
                     (unsigned int)i * sizeof(double));
            }
            nonDegenerateWset = feasibleX0ForWorkingSet(
                memspace->workspace_double, solution->searchDir, workingset,
                qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }
            activeSetChangeID = 0;
            minLambda =
                b_maxConstraintViolation(workingset, solution->searchDir);
            if (minLambda < solution->maxConstr) {
              if (i - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0],
                       (unsigned int)i * sizeof(double));
              }
              solution->maxConstr = minLambda;
            }
          }
        }
        if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
          solution->fstar = computeFval_ReuseHx(
              objective, memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) &&
              ((solution->state != 0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = computeFval_ReuseHx(
            objective, memspace->workspace_double, f, solution->xstar);
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
