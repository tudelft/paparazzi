/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortLambdaQP.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "sortLambdaQP.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double lambda_data[]
 *                int *lambda_size
 *                int WorkingSet_nActiveConstr
 *                const int WorkingSet_sizes[5]
 *                const int WorkingSet_isActiveIdx[6]
 *                const int WorkingSet_Wid_data[]
 *                const int WorkingSet_Wlocalidx_data[]
 *                emxArray_real_T *workspace
 * Return Type  : void
 */
void sortLambdaQP(double lambda_data[], int *lambda_size,
                  int WorkingSet_nActiveConstr, const int WorkingSet_sizes[5],
                  const int WorkingSet_isActiveIdx[6],
                  const int WorkingSet_Wid_data[],
                  const int WorkingSet_Wlocalidx_data[],
                  emxArray_real_T *workspace)
{
  double y_data[311];
  double *workspace_data;
  int idxOffset;
  workspace_data = workspace->data;
  if (WorkingSet_nActiveConstr != 0) {
    int idx;
    int mAll;
    mAll = WorkingSet_sizes[3] + 145;
    idx = (unsigned short)(WorkingSet_sizes[3] + 146);
    for (idxOffset = 0; idxOffset < idx; idxOffset++) {
      workspace_data[idxOffset] = lambda_data[idxOffset];
    }
    memcpy(&y_data[0], &lambda_data[0], 311U * sizeof(double));
    memset(&y_data[0], 0, (unsigned int)(mAll + 1) * sizeof(double));
    *lambda_size = 311;
    memcpy(&lambda_data[0], &y_data[0], 311U * sizeof(double));
    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) &&
           (WorkingSet_Wid_data[idx] <= 2)) {
      if (WorkingSet_Wid_data[idx] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }
      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[idx]) - 2] =
          workspace_data[mAll];
      mAll++;
      idx++;
    }
    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid_data[idx]) {
      case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;
      case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;
      default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }
      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[idx]) - 2] =
          workspace_data[mAll];
      mAll++;
      idx++;
    }
  }
}

/*
 * File trailer for sortLambdaQP.c
 *
 * [EOF]
 */
