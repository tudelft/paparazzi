/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RemoveDependentEq_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "RemoveDependentEq_.h"
#include "computeQ_.h"
#include "countsort.h"
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : e_struct_T *memspace
 *                f_struct_T *workingset
 *                g_struct_T *qrmanager
 * Return Type  : int
 */
int RemoveDependentEq_(e_struct_T *memspace, f_struct_T *workingset,
                       g_struct_T *qrmanager)
{
  emxArray_real_T *y;
  double *y_data;
  int idxDiag;
  int idx_col;
  int k;
  int mTotalWorkingEq_tmp_tmp;
  int nDepInd;
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    double tol;
    int i;
    int i1;
    int u0;
    i = (unsigned char)workingset->nVar;
    for (idxDiag = 0; idxDiag < mTotalWorkingEq_tmp_tmp; idxDiag++) {
      for (idx_col = 0; idx_col < i; idx_col++) {
        qrmanager->QR->data[idxDiag + 311 * idx_col] =
            workingset->ATwset->data[idx_col + 183 * idxDiag];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }
    memset(&qrmanager->jpvt.data[0], 0, (unsigned int)i * sizeof(int));
    i1 = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i1 == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      idxDiag = workingset->nVar;
      if (mTotalWorkingEq_tmp_tmp <= idxDiag) {
        idxDiag = mTotalWorkingEq_tmp_tmp;
      }
      qrmanager->minRowCol = idxDiag;
      qrmanager->tau.size[0] =
          xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp, workingset->nVar,
                 qrmanager->jpvt.data, qrmanager->tau.data);
    }
    tol = 100.0 * (double)workingset->nVar * 2.2204460492503131E-16;
    u0 = workingset->nVar;
    if (u0 > mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }
    idxDiag = u0 + 311 * (u0 - 1);
    while ((idxDiag > 0) && (fabs(qrmanager->QR->data[idxDiag - 1]) < tol)) {
      idxDiag -= 312;
      nDepInd++;
    }
    if (nDepInd > 0) {
      bool exitg1;
      computeQ_(qrmanager, qrmanager->mrows);
      idx_col = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_col <= nDepInd - 1)) {
        double qtb;
        idxDiag = 311 * ((mTotalWorkingEq_tmp_tmp - idx_col) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q->data[idxDiag + k] * workingset->bwset.data[k];
        }
        if (fabs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx_col++;
        }
      }
    }
    if (nDepInd > 0) {
      int ix0;
      emxInit_real_T(&y, 2);
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = 311 * idx_col;
        ix0 = 183 * idx_col;
        k = y->size[0] * y->size[1];
        y->size[0] = 311;
        y->size[1] = 311;
        emxEnsureCapacity_real_T(y, k);
        y_data = y->data;
        for (k = 0; k < 96721; k++) {
          y_data[k] = qrmanager->QR->data[k];
        }
        for (k = 0; k < i; k++) {
          y_data[idxDiag + k] = workingset->ATwset->data[ix0 + k];
        }
        k = qrmanager->QR->size[0] * qrmanager->QR->size[1];
        qrmanager->QR->size[0] = 311;
        qrmanager->QR->size[1] = 311;
        emxEnsureCapacity_real_T(qrmanager->QR, k);
        for (k = 0; k < 96721; k++) {
          qrmanager->QR->data[k] = y_data[k];
        }
      }
      emxFree_real_T(&y);
      idxDiag = workingset->nWConstr[0];
      for (idx_col = 0; idx_col < idxDiag; idx_col++) {
        qrmanager->jpvt.data[idx_col] = 1;
      }
      k = workingset->nWConstr[0] + 1;
      if (k <= mTotalWorkingEq_tmp_tmp) {
        memset(&qrmanager->jpvt.data[k + -1], 0,
               (unsigned int)((mTotalWorkingEq_tmp_tmp - k) + 1) * sizeof(int));
      }
      if (i1 == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = u0;
        qrmanager->tau.size[0] =
            xgeqp3(qrmanager->QR, workingset->nVar, mTotalWorkingEq_tmp_tmp,
                   qrmanager->jpvt.data, qrmanager->tau.data);
      }
      for (idx_col = 0; idx_col < nDepInd; idx_col++) {
        memspace->workspace_int.data[idx_col] =
            qrmanager->jpvt.data[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx_col];
      }
      countsort(memspace->workspace_int.data, nDepInd,
                memspace->workspace_sort.data, 1, mTotalWorkingEq_tmp_tmp);
      for (idx_col = nDepInd; idx_col >= 1; idx_col--) {
        i1 = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (i1 != 0) {
          k = memspace->workspace_int.data[idx_col - 1];
          if (k <= i1) {
            if ((workingset->nActiveConstr == i1) || (k == i1)) {
              workingset->mEqRemoved++;
              workingset->indexEqRemoved.data[workingset->mEqRemoved - 1] =
                  workingset->Wlocalidx.data[k - 1];
              removeConstr(workingset, k);
            } else {
              workingset->mEqRemoved++;
              ix0 = workingset->Wid.data[k - 1] - 1;
              idxDiag = workingset->Wlocalidx.data[k - 1];
              workingset->indexEqRemoved.data[workingset->mEqRemoved - 1] =
                  idxDiag;
              workingset->isActiveConstr
                  .data[(workingset->isActiveIdx[ix0] + idxDiag) - 2] = false;
              workingset->Wid.data[k - 1] = workingset->Wid.data[i1 - 1];
              workingset->Wlocalidx.data[k - 1] =
                  workingset->Wlocalidx.data[i1 - 1];
              for (idxDiag = 0; idxDiag < i; idxDiag++) {
                workingset->ATwset->data[idxDiag + 183 * (k - 1)] =
                    workingset->ATwset->data[idxDiag + 183 * (i1 - 1)];
              }
              workingset->bwset.data[k - 1] = workingset->bwset.data[i1 - 1];
              k = workingset->nActiveConstr - 1;
              workingset->Wid.data[i1 - 1] = workingset->Wid.data[k];
              workingset->Wlocalidx.data[i1 - 1] =
                  workingset->Wlocalidx.data[k];
              for (idxDiag = 0; idxDiag < i; idxDiag++) {
                workingset->ATwset->data[idxDiag + 183 * (i1 - 1)] =
                    workingset->ATwset->data[idxDiag + 183 * k];
              }
              workingset->bwset.data[i1 - 1] = workingset->bwset.data[k];
              workingset->nActiveConstr = k;
              workingset->nWConstr[ix0]--;
            }
          }
        }
      }
    }
  }
  return nDepInd;
}

/*
 * File trailer for RemoveDependentEq_.c
 *
 * [EOF]
 */
