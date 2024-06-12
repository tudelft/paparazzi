/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RemoveDependentIneq_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "RemoveDependentIneq_.h"
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
 * Arguments    : f_struct_T *workingset
 *                g_struct_T *qrmanager
 *                e_struct_T *memspace
 * Return Type  : void
 */
void RemoveDependentIneq_(f_struct_T *workingset, g_struct_T *qrmanager,
                          e_struct_T *memspace)
{
  emxArray_real_T *y;
  double *y_data;
  int idx;
  int idx_col;
  int k;
  int nActiveConstr_tmp;
  int nFixedConstr;
  int nVar;
  nActiveConstr_tmp = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
          workingset->nWConstr[4] >
      0) {
    double tol;
    int idxDiag;
    int nDepIneq;
    tol = 100.0 * (double)workingset->nVar * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt.data[idx] = 1;
    }
    idx = nFixedConstr + 1;
    if (idx <= nActiveConstr_tmp) {
      memset(&qrmanager->jpvt.data[idx + -1], 0,
             (unsigned int)((nActiveConstr_tmp - idx) + 1) * sizeof(int));
    }
    emxInit_real_T(&y, 2);
    for (idx_col = 0; idx_col < nActiveConstr_tmp; idx_col++) {
      idxDiag = 311 * idx_col;
      nDepIneq = 183 * idx_col;
      idx = y->size[0] * y->size[1];
      y->size[0] = 311;
      y->size[1] = 311;
      emxEnsureCapacity_real_T(y, idx);
      y_data = y->data;
      for (idx = 0; idx < 96721; idx++) {
        y_data[idx] = qrmanager->QR->data[idx];
      }
      idx = (unsigned char)nVar;
      for (k = 0; k < idx; k++) {
        y_data[idxDiag + k] = workingset->ATwset->data[nDepIneq + k];
      }
      idx = qrmanager->QR->size[0] * qrmanager->QR->size[1];
      qrmanager->QR->size[0] = 311;
      qrmanager->QR->size[1] = 311;
      emxEnsureCapacity_real_T(qrmanager->QR, idx);
      for (idx = 0; idx < 96721; idx++) {
        qrmanager->QR->data[idx] = y_data[idx];
      }
    }
    emxFree_real_T(&y);
    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      idxDiag = workingset->nVar;
      nDepIneq = workingset->nActiveConstr;
      if (idxDiag <= nDepIneq) {
        nDepIneq = idxDiag;
      }
      qrmanager->minRowCol = nDepIneq;
      qrmanager->tau.size[0] =
          xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
                 qrmanager->jpvt.data, qrmanager->tau.data);
    }
    nDepIneq = 0;
    for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar; idx--) {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[idx];
    }
    if (idx + 1 <= workingset->nVar) {
      idxDiag = idx + 311 * idx;
      while ((idx + 1 > nFixedConstr) &&
             (fabs(qrmanager->QR->data[idxDiag]) < tol)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[idx];
        idx--;
        idxDiag -= 312;
      }
    }
    countsort(memspace->workspace_int.data, nDepIneq,
              memspace->workspace_sort.data, nFixedConstr + 1,
              workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      removeConstr(workingset, memspace->workspace_int.data[idx - 1]);
    }
  }
}

/*
 * Arguments    : f_struct_T *workingset
 *                g_struct_T *qrmanager
 *                e_struct_T *memspace
 * Return Type  : void
 */
void b_RemoveDependentIneq_(f_struct_T *workingset, g_struct_T *qrmanager,
                            e_struct_T *memspace)
{
  emxArray_real_T *y;
  double *y_data;
  int idx;
  int idx_col;
  int k;
  int nActiveConstr_tmp;
  int nFixedConstr;
  int nVar;
  nActiveConstr_tmp = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
          workingset->nWConstr[4] >
      0) {
    double tol;
    int idxDiag;
    int nDepIneq;
    tol = 1000.0 * (double)workingset->nVar * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt.data[idx] = 1;
    }
    idx = nFixedConstr + 1;
    if (idx <= nActiveConstr_tmp) {
      memset(&qrmanager->jpvt.data[idx + -1], 0,
             (unsigned int)((nActiveConstr_tmp - idx) + 1) * sizeof(int));
    }
    emxInit_real_T(&y, 2);
    for (idx_col = 0; idx_col < nActiveConstr_tmp; idx_col++) {
      idxDiag = 311 * idx_col;
      nDepIneq = 183 * idx_col;
      idx = y->size[0] * y->size[1];
      y->size[0] = 311;
      y->size[1] = 311;
      emxEnsureCapacity_real_T(y, idx);
      y_data = y->data;
      for (idx = 0; idx < 96721; idx++) {
        y_data[idx] = qrmanager->QR->data[idx];
      }
      idx = (unsigned char)nVar;
      for (k = 0; k < idx; k++) {
        y_data[idxDiag + k] = workingset->ATwset->data[nDepIneq + k];
      }
      idx = qrmanager->QR->size[0] * qrmanager->QR->size[1];
      qrmanager->QR->size[0] = 311;
      qrmanager->QR->size[1] = 311;
      emxEnsureCapacity_real_T(qrmanager->QR, idx);
      for (idx = 0; idx < 96721; idx++) {
        qrmanager->QR->data[idx] = y_data[idx];
      }
    }
    emxFree_real_T(&y);
    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      idxDiag = workingset->nVar;
      nDepIneq = workingset->nActiveConstr;
      if (idxDiag <= nDepIneq) {
        nDepIneq = idxDiag;
      }
      qrmanager->minRowCol = nDepIneq;
      qrmanager->tau.size[0] =
          xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
                 qrmanager->jpvt.data, qrmanager->tau.data);
    }
    nDepIneq = 0;
    for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar; idx--) {
      nDepIneq++;
      memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[idx];
    }
    if (idx + 1 <= workingset->nVar) {
      idxDiag = idx + 311 * idx;
      while ((idx + 1 > nFixedConstr) &&
             (fabs(qrmanager->QR->data[idxDiag]) < tol)) {
        nDepIneq++;
        memspace->workspace_int.data[nDepIneq - 1] = qrmanager->jpvt.data[idx];
        idx--;
        idxDiag -= 312;
      }
    }
    countsort(memspace->workspace_int.data, nDepIneq,
              memspace->workspace_sort.data, nFixedConstr + 1,
              workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      removeConstr(workingset, memspace->workspace_int.data[idx - 1]);
    }
  }
}

/*
 * File trailer for RemoveDependentIneq_.c
 *
 * [EOF]
 */
