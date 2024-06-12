/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: feasibleX0ForWorkingSet.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "maxConstraintViolation.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *workspace
 *                double xCurrent_data[]
 *                f_struct_T *workingset
 *                g_struct_T *qrmanager
 * Return Type  : bool
 */
bool feasibleX0ForWorkingSet(emxArray_real_T *workspace, double xCurrent_data[],
                             f_struct_T *workingset, g_struct_T *qrmanager)
{
  emxArray_real_T *B;
  double tau_data[311];
  double *B_data;
  double *workspace_data;
  int b_i;
  int br;
  int i;
  int iAcol;
  int j;
  int jBcol;
  int k;
  int mWConstr;
  int nVar;
  bool nonDegenerateWset;
  workspace_data = workspace->data;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  emxInit_real_T(&B, 2);
  if (mWConstr != 0) {
    double c;
    int i1;
    for (iAcol = 0; iAcol < mWConstr; iAcol++) {
      c = workingset->bwset.data[iAcol];
      workspace_data[iAcol] = c;
      workspace_data[iAcol + 311] = c;
    }
    if (mWConstr != 0) {
      i = 183 * (mWConstr - 1) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 183) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (jBcol = iAcol; jBcol <= i1; jBcol++) {
          c += workingset->ATwset->data[jBcol - 1] *
               xCurrent_data[jBcol - iAcol];
        }
        i1 = div_nde_s32_floor(iAcol - 1, 183);
        workspace_data[i1] -= c;
      }
    }
    if (mWConstr >= nVar) {
      i = (unsigned char)nVar;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (jBcol = 0; jBcol < i; jBcol++) {
        iAcol = 311 * jBcol;
        for (br = 0; br < mWConstr; br++) {
          qrmanager->QR->data[br + iAcol] =
              workingset->ATwset->data[jBcol + 183 * br];
        }
        qrmanager->jpvt.data[jBcol] = jBcol + 1;
      }
      if (mWConstr <= nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }
      qrmanager->minRowCol = i;
      memset(&tau_data[0], 0, 311U * sizeof(double));
      if (i >= 1) {
        qrf(qrmanager->QR, mWConstr, nVar, i, tau_data);
      }
      qrmanager->tau.size[0] = 311;
      memcpy(&qrmanager->tau.data[0], &tau_data[0], 311U * sizeof(double));
      computeQ_(qrmanager, mWConstr);
      i = B->size[0] * B->size[1];
      B->size[0] = 311;
      B->size[1] = 183;
      emxEnsureCapacity_real_T(B, i);
      B_data = B->data;
      for (i = 0; i < 56913; i++) {
        B_data[i] = workspace_data[i];
      }
      for (b_i = 0; b_i <= 311; b_i += 311) {
        i = b_i + 1;
        i1 = b_i + nVar;
        for (k = i; k <= i1; k++) {
          workspace_data[k - 1] = 0.0;
        }
      }
      br = -1;
      for (b_i = 0; b_i <= 311; b_i += 311) {
        jBcol = -1;
        i = b_i + 1;
        i1 = b_i + nVar;
        for (k = i; k <= i1; k++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q->data[(iAcol + jBcol) + 1] *
                 B_data[(iAcol + br) + 1];
          }
          workspace_data[k - 1] += c;
          jBcol += 311;
        }
        br += 311;
      }
      for (j = 0; j < 2; j++) {
        jBcol = 311 * j - 1;
        for (k = nVar; k >= 1; k--) {
          iAcol = 311 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace_data[i];
          if (c != 0.0) {
            workspace_data[i] = c / qrmanager->QR->data[k + iAcol];
            i1 = (unsigned char)(k - 1);
            for (b_i = 0; b_i < i1; b_i++) {
              int i2;
              i2 = (b_i + jBcol) + 1;
              workspace_data[i2] -=
                  workspace_data[i] * qrmanager->QR->data[(b_i + iAcol) + 1];
            }
          }
        }
      }
    } else {
      factorQR(qrmanager, workingset->ATwset, nVar, mWConstr);
      computeQ_(qrmanager, qrmanager->minRowCol);
      for (j = 0; j < 2; j++) {
        jBcol = 311 * j;
        for (b_i = 0; b_i < mWConstr; b_i++) {
          iAcol = 311 * b_i;
          br = b_i + jBcol;
          c = workspace_data[br];
          i = (unsigned char)b_i;
          for (k = 0; k < i; k++) {
            c -= qrmanager->QR->data[k + iAcol] * workspace_data[k + jBcol];
          }
          workspace_data[br] = c / qrmanager->QR->data[b_i + iAcol];
        }
      }
      i = B->size[0] * B->size[1];
      B->size[0] = 311;
      B->size[1] = 183;
      emxEnsureCapacity_real_T(B, i);
      B_data = B->data;
      for (i = 0; i < 56913; i++) {
        B_data[i] = workspace_data[i];
      }
      for (b_i = 0; b_i <= 311; b_i += 311) {
        i = b_i + 1;
        i1 = b_i + nVar;
        for (k = i; k <= i1; k++) {
          workspace_data[k - 1] = 0.0;
        }
      }
      br = 0;
      for (b_i = 0; b_i <= 311; b_i += 311) {
        jBcol = -1;
        i = br + 1;
        i1 = br + mWConstr;
        for (j = i; j <= i1; j++) {
          int i2;
          i2 = b_i + 1;
          iAcol = b_i + nVar;
          for (k = i2; k <= iAcol; k++) {
            workspace_data[k - 1] +=
                B_data[j - 1] * qrmanager->Q->data[(jBcol + k) - b_i];
          }
          jBcol += 311;
        }
        br += 311;
      }
    }
    iAcol = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iAcol <= (unsigned char)nVar - 1) {
        if (rtIsInf(workspace_data[iAcol]) || rtIsNaN(workspace_data[iAcol]) ||
            (rtIsInf(workspace_data[iAcol + 311]) ||
             rtIsNaN(workspace_data[iAcol + 311]))) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          iAcol++;
        }
      } else {
        double constrViolation_basicX;
        iAcol = nVar - 1;
        for (k = 0; k <= iAcol; k++) {
          workspace_data[k] += xCurrent_data[k];
        }
        c = maxConstraintViolation(workingset, workspace);
        constrViolation_basicX =
            c_maxConstraintViolation(workingset, workspace);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          i = (unsigned char)nVar;
          for (k = 0; k < i; k++) {
            xCurrent_data[k] = workspace_data[k];
          }
        } else {
          i = (unsigned char)nVar;
          for (k = 0; k < i; k++) {
            xCurrent_data[k] = workspace_data[k + 311];
          }
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  emxFree_real_T(&B);
  return nonDegenerateWset;
}

/*
 * File trailer for feasibleX0ForWorkingSet.c
 *
 * [EOF]
 */
