/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_deltax.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "compute_deltax.h"
#include "fullColLDL2_.h"
#include "partialColLDL3_.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include "xpotrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double H[324]
 *                c_struct_T *solution
 *                e_struct_T *memspace
 *                const g_struct_T *qrmanager
 *                h_struct_T *cholmanager
 *                const d_struct_T *objective
 *                bool alwaysPositiveDef
 * Return Type  : void
 */
void compute_deltax(const double H[324], c_struct_T *solution,
                    e_struct_T *memspace, const g_struct_T *qrmanager,
                    h_struct_T *cholmanager, const d_struct_T *objective,
                    bool alwaysPositiveDef)
{
  int idx;
  int idx_row;
  int ix;
  int mNull_tmp;
  int nVar_tmp;
  int nVars;
  int order;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      memset(&solution->searchDir.data[0], 0,
             (unsigned int)(nVar_tmp + 1) * sizeof(double));
    }
  } else {
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir.data[idx] = -objective->grad.data[idx];
    }
    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
      case 5:
        break;
      case 3: {
        double smax;
        if (alwaysPositiveDef) {
          cholmanager->ndims = qrmanager->mrows;
          for (idx = 0; idx <= nVar_tmp; idx++) {
            order = qrmanager->mrows * idx;
            nVars = 311 * idx;
            for (ix = 0; ix <= nVar_tmp; ix++) {
              cholmanager->FMat->data[nVars + ix] = H[order + ix];
            }
          }
          cholmanager->info = xpotrf(qrmanager->mrows, cholmanager->FMat);
        } else {
          idx_row = qrmanager->mrows;
          cholmanager->ndims = qrmanager->mrows;
          for (idx = 0; idx < idx_row; idx++) {
            order = qrmanager->mrows * idx;
            nVars = 311 * idx;
            for (ix = 0; ix < idx_row; ix++) {
              cholmanager->FMat->data[nVars + ix] = H[order + ix];
            }
          }
          if (qrmanager->mrows < 1) {
            nVars = -1;
          } else {
            nVars = 0;
            if (qrmanager->mrows > 1) {
              smax = fabs(cholmanager->FMat->data[0]);
              for (ix = 2; ix <= idx_row; ix++) {
                double s;
                s = fabs(cholmanager->FMat->data[(ix - 1) * 312]);
                if (s > smax) {
                  nVars = ix - 1;
                  smax = s;
                }
              }
            }
          }
          cholmanager->regTol_ =
              fmax(fabs(cholmanager->FMat->data[nVars + 311 * nVars]) *
                       2.2204460492503131E-16,
                   0.0);
          if (qrmanager->mrows > 128) {
            bool exitg1;
            ix = 0;
            exitg1 = false;
            while ((!exitg1) && (ix < idx_row)) {
              nVars = 312 * ix + 1;
              order = idx_row - ix;
              if (ix + 48 <= idx_row) {
                partialColLDL3_(cholmanager, nVars, order);
                ix += 48;
              } else {
                fullColLDL2_(cholmanager, nVars, order);
                exitg1 = true;
              }
            }
          } else {
            fullColLDL2_(cholmanager, 1, qrmanager->mrows);
          }
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg2;
            do {
              exitg2 = 0;
              if (idx <= idx_row - 1) {
                if (cholmanager->FMat->data[idx + 311 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg2 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else if (alwaysPositiveDef) {
          solve(cholmanager, solution->searchDir.data);
        } else {
          int i;
          order = cholmanager->ndims - 2;
          if (cholmanager->ndims != 0) {
            for (idx_row = 0; idx_row <= order + 1; idx_row++) {
              nVars = idx_row + idx_row * 311;
              i = order - idx_row;
              for (idx = 0; idx <= i; idx++) {
                ix = (idx_row + idx) + 1;
                solution->searchDir.data[ix] -=
                    solution->searchDir.data[idx_row] *
                    cholmanager->FMat->data[(nVars + idx) + 1];
              }
            }
          }
          order = cholmanager->ndims;
          for (idx = 0; idx < order; idx++) {
            solution->searchDir.data[idx] /=
                cholmanager->FMat->data[idx + 311 * idx];
          }
          if (cholmanager->ndims != 0) {
            for (idx_row = order; idx_row >= 1; idx_row--) {
              nVars = (idx_row - 1) * 311;
              smax = solution->searchDir.data[idx_row - 1];
              i = idx_row + 1;
              for (idx = order; idx >= i; idx--) {
                smax -= cholmanager->FMat->data[(nVars + idx) - 1] *
                        solution->searchDir.data[idx - 1];
              }
              solution->searchDir.data[idx_row - 1] = smax;
            }
          }
        }
      } break;
      default: {
        if (alwaysPositiveDef) {
          idx_row = objective->nvar - 1;
          cholmanager->ndims = objective->nvar;
          for (idx = 0; idx <= idx_row; idx++) {
            order = objective->nvar * idx;
            nVars = 311 * idx;
            for (ix = 0; ix <= idx_row; ix++) {
              cholmanager->FMat->data[nVars + ix] = H[order + ix];
            }
          }
          cholmanager->info = xpotrf(objective->nvar, cholmanager->FMat);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            double smax;
            int i;
            solve(cholmanager, solution->searchDir.data);
            smax = 1.0 / objective->beta;
            order = objective->nvar + 1;
            i = qrmanager->mrows;
            for (ix = order; ix <= i; ix++) {
              solution->searchDir.data[ix - 1] *= smax;
            }
          }
        }
      } break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = 311 * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double->data[idx] =
              -qrmanager->Q->data[nVar_tmp + 311 * (qrmanager->ncols + idx)];
        }
        if (qrmanager->mrows != 0) {
          int i;
          memset(&solution->searchDir.data[0], 0,
                 (unsigned int)(nVar_tmp + 1) * sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 311 * (mNull_tmp - 1);
          for (idx_row = nullStartIdx_tmp; idx_row <= i; idx_row += 311) {
            order = idx_row + nVar_tmp;
            for (idx = idx_row; idx <= order; idx++) {
              nVars = idx - idx_row;
              solution->searchDir.data[nVars] +=
                  qrmanager->Q->data[idx - 1] *
                  memspace->workspace_double->data[ix];
            }
            ix++;
          }
        }
      } else {
        double smax;
        int i;
        if (objective->objtype == 3) {
          xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp,
                memspace->workspace_double);
          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager->mrows;
          xgemm(objective->nvar, mNull_tmp, objective->nvar, H, objective->nvar,
                qrmanager->Q, nullStartIdx_tmp, memspace->workspace_double);
          i = objective->nvar + 1;
          for (order = 0; order < mNull_tmp; order++) {
            for (idx_row = i; idx_row <= nVars; idx_row++) {
              memspace->workspace_double->data[(idx_row + 311 * order) - 1] =
                  objective->beta *
                  qrmanager->Q
                      ->data[(idx_row + 311 * (order + qrmanager->ncols)) - 1];
            }
          }
          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        }
        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          cholmanager->info = xpotrf(mNull_tmp, cholmanager->FMat);
        } else {
          cholmanager->ndims = mNull_tmp;
          nVars = 0;
          if (mNull_tmp > 1) {
            smax = fabs(cholmanager->FMat->data[0]);
            for (ix = 2; ix <= mNull_tmp; ix++) {
              double s;
              s = fabs(cholmanager->FMat->data[(ix - 1) * 312]);
              if (s > smax) {
                nVars = ix - 1;
                smax = s;
              }
            }
          }
          cholmanager->regTol_ =
              fmax(fabs(cholmanager->FMat->data[nVars + 311 * nVars]) *
                       2.2204460492503131E-16,
                   0.0);
          if (mNull_tmp > 128) {
            bool exitg1;
            ix = 0;
            exitg1 = false;
            while ((!exitg1) && (ix < mNull_tmp)) {
              nVars = 312 * ix + 1;
              order = mNull_tmp - ix;
              if (ix + 48 <= mNull_tmp) {
                partialColLDL3_(cholmanager, nVars, order);
                ix += 48;
              } else {
                fullColLDL2_(cholmanager, nVars, order);
                exitg1 = true;
              }
            }
          } else {
            fullColLDL2_(cholmanager, 1, mNull_tmp);
          }
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg2;
            do {
              exitg2 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager->FMat->data[idx + 311 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg2 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            for (nVars = 0; nVars < mNull_tmp; nVars++) {
              memspace->workspace_double->data[nVars] = 0.0;
            }
            i = nullStartIdx_tmp + 311 * (mNull_tmp - 1);
            for (idx_row = nullStartIdx_tmp; idx_row <= i; idx_row += 311) {
              smax = 0.0;
              order = idx_row + nVar_tmp;
              for (idx = idx_row; idx <= order; idx++) {
                smax += qrmanager->Q->data[idx - 1] *
                        objective->grad.data[idx - idx_row];
              }
              order = div_nde_s32_floor(idx_row - nullStartIdx_tmp, 311);
              memspace->workspace_double->data[order] -= smax;
            }
          }
          if (alwaysPositiveDef) {
            order = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx_row = 0; idx_row < order; idx_row++) {
                nVars = idx_row * 311;
                smax = memspace->workspace_double->data[idx_row];
                for (idx = 0; idx < idx_row; idx++) {
                  smax -= cholmanager->FMat->data[nVars + idx] *
                          memspace->workspace_double->data[idx];
                }
                memspace->workspace_double->data[idx_row] =
                    smax / cholmanager->FMat->data[nVars + idx_row];
              }
            }
            if (cholmanager->ndims != 0) {
              for (idx_row = order; idx_row >= 1; idx_row--) {
                nVars = (idx_row + (idx_row - 1) * 311) - 1;
                memspace->workspace_double->data[idx_row - 1] /=
                    cholmanager->FMat->data[nVars];
                for (idx = 0; idx <= idx_row - 2; idx++) {
                  ix = (idx_row - idx) - 2;
                  memspace->workspace_double->data[ix] -=
                      memspace->workspace_double->data[idx_row - 1] *
                      cholmanager->FMat->data[(nVars - idx) - 1];
                }
              }
            }
          } else {
            order = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx_row = 0; idx_row <= order + 1; idx_row++) {
                nVars = idx_row + idx_row * 311;
                i = order - idx_row;
                for (idx = 0; idx <= i; idx++) {
                  ix = (idx_row + idx) + 1;
                  memspace->workspace_double->data[ix] -=
                      memspace->workspace_double->data[idx_row] *
                      cholmanager->FMat->data[(nVars + idx) + 1];
                }
              }
            }
            order = cholmanager->ndims;
            for (idx = 0; idx < order; idx++) {
              memspace->workspace_double->data[idx] /=
                  cholmanager->FMat->data[idx + 311 * idx];
            }
            if (cholmanager->ndims != 0) {
              for (idx_row = order; idx_row >= 1; idx_row--) {
                nVars = (idx_row - 1) * 311;
                smax = memspace->workspace_double->data[idx_row - 1];
                i = idx_row + 1;
                for (idx = order; idx >= i; idx--) {
                  smax -= cholmanager->FMat->data[(nVars + idx) - 1] *
                          memspace->workspace_double->data[idx - 1];
                }
                memspace->workspace_double->data[idx_row - 1] = smax;
              }
            }
          }
          if (qrmanager->mrows != 0) {
            memset(&solution->searchDir.data[0], 0,
                   (unsigned int)(nVar_tmp + 1) * sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + 311 * (mNull_tmp - 1);
            for (idx_row = nullStartIdx_tmp; idx_row <= i; idx_row += 311) {
              order = idx_row + nVar_tmp;
              for (idx = idx_row; idx <= order; idx++) {
                nVars = idx - idx_row;
                solution->searchDir.data[nVars] +=
                    qrmanager->Q->data[idx - 1] *
                    memspace->workspace_double->data[ix];
              }
              ix++;
            }
          }
        }
      }
    }
  }
}

/*
 * File trailer for compute_deltax.c
 *
 * [EOF]
 */
