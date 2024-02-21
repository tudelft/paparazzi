/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_deltax.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:30:21
 */

/* Include Files */
#include "compute_deltax.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "Cascaded_nonlinear_TestFlight_rtwutil.h"
#include "fullColLDL2_.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include "xpotrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double H[169]
 *                p_struct_T *solution
 *                k_struct_T *memspace
 *                const i_struct_T *qrmanager
 *                j_struct_T *cholmanager
 *                const c_struct_T *objective
 *                bool alwaysPositiveDef
 * Return Type  : void
 */
void b_compute_deltax(const double H[169], p_struct_T *solution,
                      k_struct_T *memspace, const i_struct_T *qrmanager,
                      j_struct_T *cholmanager, const c_struct_T *objective,
                      bool alwaysPositiveDef)
{
  int b_i;
  int idx;
  int ix;
  int jA;
  int jjA;
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      memset(&solution->searchDir[0], 0,
             (unsigned int)(nVar_tmp + 1) * sizeof(double));
    }
  } else {
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = -objective->grad[idx];
    }
    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
      case 5:
        break;
      case 3: {
        double smax;
        int nVars;
        if (alwaysPositiveDef) {
          cholmanager->ndims = qrmanager->mrows;
          for (idx = 0; idx <= nVar_tmp; idx++) {
            jjA = (nVar_tmp + 1) * idx;
            jA = 27 * idx;
            for (ix = 0; ix <= nVar_tmp; ix++) {
              cholmanager->FMat[jA + ix] = H[jjA + ix];
            }
          }
          cholmanager->info = b_xpotrf(qrmanager->mrows, cholmanager->FMat);
        } else {
          cholmanager->ndims = qrmanager->mrows;
          for (idx = 0; idx <= nVar_tmp; idx++) {
            jjA = qrmanager->mrows * idx;
            jA = 27 * idx;
            for (ix = 0; ix <= nVar_tmp; ix++) {
              cholmanager->FMat[jA + ix] = H[jjA + ix];
            }
          }
          if (qrmanager->mrows < 1) {
            nVars = -1;
          } else {
            nVars = 0;
            if (qrmanager->mrows > 1) {
              smax = fabs(cholmanager->FMat[0]);
              for (ix = 2; ix <= nVar_tmp + 1; ix++) {
                double s;
                s = fabs(cholmanager->FMat[(ix - 1) * 28]);
                if (s > smax) {
                  nVars = ix - 1;
                  smax = s;
                }
              }
            }
          }
          cholmanager->regTol_ =
              fmax(fabs(cholmanager->FMat[nVars + 27 * nVars]) *
                       2.2204460492503131E-16,
                   0.0);
          b_fullColLDL2_(cholmanager, qrmanager->mrows);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= nVar_tmp) {
                if (cholmanager->FMat[idx + 27 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else if (alwaysPositiveDef) {
          b_solve(cholmanager, solution->searchDir);
        } else {
          int i;
          nVars = cholmanager->ndims - 2;
          if (cholmanager->ndims != 0) {
            for (idx = 0; idx <= nVars + 1; idx++) {
              jjA = idx + idx * 27;
              i = nVars - idx;
              for (b_i = 0; b_i <= i; b_i++) {
                ix = (idx + b_i) + 1;
                solution->searchDir[ix] -= solution->searchDir[idx] *
                                           cholmanager->FMat[(jjA + b_i) + 1];
              }
            }
          }
          nVars = cholmanager->ndims;
          for (idx = 0; idx < nVars; idx++) {
            solution->searchDir[idx] /= cholmanager->FMat[idx + 27 * idx];
          }
          if (cholmanager->ndims != 0) {
            for (idx = nVars; idx >= 1; idx--) {
              jA = (idx - 1) * 27;
              smax = solution->searchDir[idx - 1];
              i = idx + 1;
              for (b_i = nVars; b_i >= i; b_i--) {
                smax -= cholmanager->FMat[(jA + b_i) - 1] *
                        solution->searchDir[b_i - 1];
              }
              solution->searchDir[idx - 1] = smax;
            }
          }
        }
      } break;
      default: {
        if (alwaysPositiveDef) {
          int nVars;
          nVars = objective->nvar;
          cholmanager->ndims = objective->nvar;
          for (idx = 0; idx < nVars; idx++) {
            jjA = nVars * idx;
            jA = 27 * idx;
            for (ix = 0; ix < nVars; ix++) {
              cholmanager->FMat[jA + ix] = H[jjA + ix];
            }
          }
          cholmanager->info = b_xpotrf(objective->nvar, cholmanager->FMat);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            double smax;
            int i;
            b_solve(cholmanager, solution->searchDir);
            smax = 1.0 / objective->beta;
            jjA = objective->nvar + 1;
            i = qrmanager->mrows;
            for (ix = jjA; ix <= i; ix++) {
              solution->searchDir[ix - 1] *= smax;
            }
          }
        }
      } break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = 27 * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] =
              -qrmanager->Q[nVar_tmp + 27 * (qrmanager->ncols + idx)];
        }
        if (qrmanager->mrows != 0) {
          int i;
          memset(&solution->searchDir[0], 0,
                 (unsigned int)(nVar_tmp + 1) * sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
          for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
            int nVars;
            nVars = jjA + nVar_tmp;
            for (idx = jjA; idx <= nVars; idx++) {
              jA = idx - jjA;
              solution->searchDir[jA] +=
                  qrmanager->Q[idx - 1] * memspace->workspace_double[ix];
            }
            ix++;
          }
        }
      } else {
        double smax;
        int i;
        int nVars;
        if (objective->objtype == 3) {
          c_xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                  qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp,
                  memspace->workspace_double);
          d_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager->mrows;
          c_xgemm(objective->nvar, mNull_tmp, objective->nvar, H,
                  objective->nvar, qrmanager->Q, nullStartIdx_tmp,
                  memspace->workspace_double);
          for (jA = 0; jA < mNull_tmp; jA++) {
            i = objective->nvar + 1;
            for (jjA = i; jjA <= nVars; jjA++) {
              memspace->workspace_double[(jjA + 27 * jA) - 1] =
                  objective->beta *
                  qrmanager->Q[(jjA + 27 * (jA + qrmanager->ncols)) - 1];
            }
          }
          d_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        }
        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          cholmanager->info = b_xpotrf(mNull_tmp, cholmanager->FMat);
        } else {
          cholmanager->ndims = mNull_tmp;
          nVars = 0;
          if (mNull_tmp > 1) {
            smax = fabs(cholmanager->FMat[0]);
            for (ix = 2; ix <= mNull_tmp; ix++) {
              double s;
              s = fabs(cholmanager->FMat[(ix - 1) * 28]);
              if (s > smax) {
                nVars = ix - 1;
                smax = s;
              }
            }
          }
          cholmanager->regTol_ =
              fmax(fabs(cholmanager->FMat[nVars + 27 * nVars]) *
                       2.2204460492503131E-16,
                   0.0);
          b_fullColLDL2_(cholmanager, mNull_tmp);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager->FMat[idx + 27 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            memset(&memspace->workspace_double[0], 0,
                   (unsigned int)mNull_tmp * sizeof(double));
            i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
              smax = 0.0;
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                smax += qrmanager->Q[idx - 1] * objective->grad[idx - jjA];
              }
              nVars = div_nde_s32_floor(jjA - nullStartIdx_tmp, 27);
              memspace->workspace_double[nVars] -= smax;
            }
          }
          if (alwaysPositiveDef) {
            nVars = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx < nVars; idx++) {
                jA = idx * 27;
                smax = memspace->workspace_double[idx];
                for (b_i = 0; b_i < idx; b_i++) {
                  smax -= cholmanager->FMat[jA + b_i] *
                          memspace->workspace_double[b_i];
                }
                memspace->workspace_double[idx] =
                    smax / cholmanager->FMat[jA + idx];
              }
            }
            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jjA = (idx + (idx - 1) * 27) - 1;
                memspace->workspace_double[idx - 1] /= cholmanager->FMat[jjA];
                for (b_i = 0; b_i <= idx - 2; b_i++) {
                  ix = (idx - b_i) - 2;
                  memspace->workspace_double[ix] -=
                      memspace->workspace_double[idx - 1] *
                      cholmanager->FMat[(jjA - b_i) - 1];
                }
              }
            }
          } else {
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 27;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  memspace->workspace_double[ix] -=
                      memspace->workspace_double[idx] *
                      cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }
            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              memspace->workspace_double[idx] /=
                  cholmanager->FMat[idx + 27 * idx];
            }
            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 27;
                smax = memspace->workspace_double[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                          memspace->workspace_double[b_i - 1];
                }
                memspace->workspace_double[idx - 1] = smax;
              }
            }
          }
          if (qrmanager->mrows != 0) {
            memset(&solution->searchDir[0], 0,
                   (unsigned int)(nVar_tmp + 1) * sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                jA = idx - jjA;
                solution->searchDir[jA] +=
                    qrmanager->Q[idx - 1] * memspace->workspace_double[ix];
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
 * Arguments    : const double H[225]
 *                n_struct_T *solution
 *                h_struct_T *memspace
 *                const f_struct_T *qrmanager
 *                g_struct_T *cholmanager
 *                const struct_T *objective
 *                bool alwaysPositiveDef
 * Return Type  : void
 */
void compute_deltax(const double H[225], n_struct_T *solution,
                    h_struct_T *memspace, const f_struct_T *qrmanager,
                    g_struct_T *cholmanager, const struct_T *objective,
                    bool alwaysPositiveDef)
{
  int b_i;
  int idx;
  int ix;
  int jA;
  int jjA;
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      memset(&solution->searchDir[0], 0,
             (unsigned int)(nVar_tmp + 1) * sizeof(double));
    }
  } else {
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = -objective->grad[idx];
    }
    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
      case 5:
        break;
      case 3: {
        double smax;
        int nVars;
        if (alwaysPositiveDef) {
          cholmanager->ndims = qrmanager->mrows;
          for (idx = 0; idx <= nVar_tmp; idx++) {
            jjA = (nVar_tmp + 1) * idx;
            jA = 31 * idx;
            for (ix = 0; ix <= nVar_tmp; ix++) {
              cholmanager->FMat[jA + ix] = H[jjA + ix];
            }
          }
          cholmanager->info = xpotrf(qrmanager->mrows, cholmanager->FMat);
        } else {
          cholmanager->ndims = qrmanager->mrows;
          for (idx = 0; idx <= nVar_tmp; idx++) {
            jjA = qrmanager->mrows * idx;
            jA = 31 * idx;
            for (ix = 0; ix <= nVar_tmp; ix++) {
              cholmanager->FMat[jA + ix] = H[jjA + ix];
            }
          }
          if (qrmanager->mrows < 1) {
            nVars = -1;
          } else {
            nVars = 0;
            if (qrmanager->mrows > 1) {
              smax = fabs(cholmanager->FMat[0]);
              for (ix = 2; ix <= nVar_tmp + 1; ix++) {
                double s;
                s = fabs(cholmanager->FMat[(ix - 1) << 5]);
                if (s > smax) {
                  nVars = ix - 1;
                  smax = s;
                }
              }
            }
          }
          cholmanager->regTol_ =
              fmax(fabs(cholmanager->FMat[nVars + 31 * nVars]) *
                       2.2204460492503131E-16,
                   0.0);
          fullColLDL2_(cholmanager, qrmanager->mrows);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= nVar_tmp) {
                if (cholmanager->FMat[idx + 31 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else if (alwaysPositiveDef) {
          solve(cholmanager, solution->searchDir);
        } else {
          int i;
          nVars = cholmanager->ndims - 2;
          if (cholmanager->ndims != 0) {
            for (idx = 0; idx <= nVars + 1; idx++) {
              jjA = idx + idx * 31;
              i = nVars - idx;
              for (b_i = 0; b_i <= i; b_i++) {
                ix = (idx + b_i) + 1;
                solution->searchDir[ix] -= solution->searchDir[idx] *
                                           cholmanager->FMat[(jjA + b_i) + 1];
              }
            }
          }
          nVars = cholmanager->ndims;
          for (idx = 0; idx < nVars; idx++) {
            solution->searchDir[idx] /= cholmanager->FMat[idx + 31 * idx];
          }
          if (cholmanager->ndims != 0) {
            for (idx = nVars; idx >= 1; idx--) {
              jA = (idx - 1) * 31;
              smax = solution->searchDir[idx - 1];
              i = idx + 1;
              for (b_i = nVars; b_i >= i; b_i--) {
                smax -= cholmanager->FMat[(jA + b_i) - 1] *
                        solution->searchDir[b_i - 1];
              }
              solution->searchDir[idx - 1] = smax;
            }
          }
        }
      } break;
      default: {
        if (alwaysPositiveDef) {
          int nVars;
          nVars = objective->nvar;
          cholmanager->ndims = objective->nvar;
          for (idx = 0; idx < nVars; idx++) {
            jjA = nVars * idx;
            jA = 31 * idx;
            for (ix = 0; ix < nVars; ix++) {
              cholmanager->FMat[jA + ix] = H[jjA + ix];
            }
          }
          cholmanager->info = xpotrf(objective->nvar, cholmanager->FMat);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            double smax;
            int i;
            solve(cholmanager, solution->searchDir);
            smax = 1.0 / objective->beta;
            jjA = objective->nvar + 1;
            i = qrmanager->mrows;
            for (ix = jjA; ix <= i; ix++) {
              solution->searchDir[ix - 1] *= smax;
            }
          }
        }
      } break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = 31 * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] =
              -qrmanager->Q[nVar_tmp + 31 * (qrmanager->ncols + idx)];
        }
        if (qrmanager->mrows != 0) {
          int i;
          memset(&solution->searchDir[0], 0,
                 (unsigned int)(nVar_tmp + 1) * sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
          for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
            int nVars;
            nVars = jjA + nVar_tmp;
            for (idx = jjA; idx <= nVars; idx++) {
              jA = idx - jjA;
              solution->searchDir[jA] +=
                  qrmanager->Q[idx - 1] * memspace->workspace_double[ix];
            }
            ix++;
          }
        }
      } else {
        double smax;
        int i;
        int nVars;
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
          for (jA = 0; jA < mNull_tmp; jA++) {
            i = objective->nvar + 1;
            for (jjA = i; jjA <= nVars; jjA++) {
              memspace->workspace_double[(jjA + 31 * jA) - 1] =
                  objective->beta *
                  qrmanager->Q[(jjA + 31 * (jA + qrmanager->ncols)) - 1];
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
            smax = fabs(cholmanager->FMat[0]);
            for (ix = 2; ix <= mNull_tmp; ix++) {
              double s;
              s = fabs(cholmanager->FMat[(ix - 1) << 5]);
              if (s > smax) {
                nVars = ix - 1;
                smax = s;
              }
            }
          }
          cholmanager->regTol_ =
              fmax(fabs(cholmanager->FMat[nVars + 31 * nVars]) *
                       2.2204460492503131E-16,
                   0.0);
          fullColLDL2_(cholmanager, mNull_tmp);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager->FMat[idx + 31 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            memset(&memspace->workspace_double[0], 0,
                   (unsigned int)mNull_tmp * sizeof(double));
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
              smax = 0.0;
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                smax += qrmanager->Q[idx - 1] * objective->grad[idx - jjA];
              }
              nVars = div_nde_s32_floor(jjA - nullStartIdx_tmp, 31);
              memspace->workspace_double[nVars] -= smax;
            }
          }
          if (alwaysPositiveDef) {
            nVars = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx < nVars; idx++) {
                jA = idx * 31;
                smax = memspace->workspace_double[idx];
                for (b_i = 0; b_i < idx; b_i++) {
                  smax -= cholmanager->FMat[jA + b_i] *
                          memspace->workspace_double[b_i];
                }
                memspace->workspace_double[idx] =
                    smax / cholmanager->FMat[jA + idx];
              }
            }
            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jjA = (idx + (idx - 1) * 31) - 1;
                memspace->workspace_double[idx - 1] /= cholmanager->FMat[jjA];
                for (b_i = 0; b_i <= idx - 2; b_i++) {
                  ix = (idx - b_i) - 2;
                  memspace->workspace_double[ix] -=
                      memspace->workspace_double[idx - 1] *
                      cholmanager->FMat[(jjA - b_i) - 1];
                }
              }
            }
          } else {
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 31;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  memspace->workspace_double[ix] -=
                      memspace->workspace_double[idx] *
                      cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }
            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              memspace->workspace_double[idx] /=
                  cholmanager->FMat[idx + 31 * idx];
            }
            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 31;
                smax = memspace->workspace_double[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                          memspace->workspace_double[b_i - 1];
                }
                memspace->workspace_double[idx - 1] = smax;
              }
            }
          }
          if (qrmanager->mrows != 0) {
            memset(&solution->searchDir[0], 0,
                   (unsigned int)(nVar_tmp + 1) * sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                jA = idx - jjA;
                solution->searchDir[jA] +=
                    qrmanager->Q[idx - 1] * memspace->workspace_double[ix];
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
