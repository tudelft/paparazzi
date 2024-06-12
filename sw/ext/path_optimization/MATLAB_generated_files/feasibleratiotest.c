/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: feasibleratiotest.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "feasibleratiotest.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xnrm2.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double solution_xstar_data[]
 *                const double solution_searchDir_data[]
 *                emxArray_real_T *workspace
 *                int workingset_nVar
 *                const emxArray_real_T *workingset_Aineq
 *                const double workingset_bineq_data[]
 *                const double workingset_lb_data[]
 *                const int workingset_indexLB_data[]
 *                const int workingset_sizes[5]
 *                const int workingset_isActiveIdx[6]
 *                const bool workingset_isActiveConstr_data[]
 *                const int workingset_nWConstr[5]
 *                bool isPhaseOne
 *                bool *newBlocking
 *                int *constrType
 *                int *constrIdx
 * Return Type  : double
 */
double feasibleratiotest(const double solution_xstar_data[],
                         const double solution_searchDir_data[],
                         emxArray_real_T *workspace, int workingset_nVar,
                         const emxArray_real_T *workingset_Aineq,
                         const double workingset_bineq_data[],
                         const double workingset_lb_data[],
                         const int workingset_indexLB_data[],
                         const int workingset_sizes[5],
                         const int workingset_isActiveIdx[6],
                         const bool workingset_isActiveConstr_data[],
                         const int workingset_nWConstr[5], bool isPhaseOne,
                         bool *newBlocking, int *constrType, int *constrIdx)
{
  const double *workingset_Aineq_data;
  double alpha;
  double c;
  double denomTol;
  double phaseOneCorrectionX;
  double *workspace_data;
  int ia;
  int iac;
  int k;
  workingset_Aineq_data = workingset_Aineq->data;
  workspace_data = workspace->data;
  alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 *
             b_xnrm2(workingset_nVar, solution_searchDir_data);
  if (workingset_nWConstr[2] < 128) {
    for (k = 0; k < 128; k++) {
      workspace_data[k] = workingset_bineq_data[k];
    }
    f_xgemv(workingset_nVar, 128, workingset_Aineq, solution_xstar_data,
            workspace);
    workspace_data = workspace->data;
    for (k = 0; k < 128; k++) {
      workspace_data[k + 311] = 0.0;
    }
    for (iac = 0; iac <= 23241; iac += 183) {
      c = 0.0;
      k = iac + workingset_nVar;
      for (ia = iac + 1; ia <= k; ia++) {
        c += workingset_Aineq_data[ia - 1] *
             solution_searchDir_data[(ia - iac) - 1];
      }
      k = div_nde_s32_floor(iac, 183) + 311;
      workspace_data[k] += c;
    }
    for (iac = 0; iac < 128; iac++) {
      phaseOneCorrectionX = workspace_data[iac + 311];
      if ((phaseOneCorrectionX > denomTol) &&
          (!workingset_isActiveConstr_data[(workingset_isActiveIdx[2] + iac) -
                                           1])) {
        c = workspace_data[iac];
        c = fmin(fabs(c), 0.001 - c) / phaseOneCorrectionX;
        if (c < alpha) {
          alpha = c;
          *constrType = 3;
          *constrIdx = iac + 1;
          *newBlocking = true;
        }
      }
    }
  }
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    double phaseOneCorrectionP;
    double ratio;
    phaseOneCorrectionX =
        (double)isPhaseOne * solution_xstar_data[workingset_nVar - 1];
    phaseOneCorrectionP =
        (double)isPhaseOne * solution_searchDir_data[workingset_nVar - 1];
    k = workingset_sizes[3];
    for (iac = 0; iac <= k - 2; iac++) {
      c = -solution_searchDir_data[workingset_indexLB_data[iac] - 1] -
          phaseOneCorrectionP;
      if ((c > denomTol) &&
          (!workingset_isActiveConstr_data[(workingset_isActiveIdx[3] + iac) -
                                           1])) {
        ratio = (-solution_xstar_data[workingset_indexLB_data[iac] - 1] -
                 workingset_lb_data[workingset_indexLB_data[iac] - 1]) -
                phaseOneCorrectionX;
        c = fmin(fabs(ratio), 0.001 - ratio) / c;
        if (c < alpha) {
          alpha = c;
          *constrType = 4;
          *constrIdx = iac + 1;
          *newBlocking = true;
        }
      }
    }
    k = workingset_indexLB_data[workingset_sizes[3] - 1] - 1;
    phaseOneCorrectionX = -solution_searchDir_data[k];
    if ((phaseOneCorrectionX > denomTol) &&
        (!workingset_isActiveConstr_data
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar_data[k] - workingset_lb_data[k];
      c = fmin(fabs(ratio), 0.001 - ratio) / phaseOneCorrectionX;
      if (c < alpha) {
        alpha = c;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }
  if (!isPhaseOne) {
    if ((*newBlocking) && (alpha > 1.0)) {
      *newBlocking = false;
    }
    alpha = fmin(alpha, 1.0);
  }
  return alpha;
}

/*
 * File trailer for feasibleratiotest.c
 *
 * [EOF]
 */
