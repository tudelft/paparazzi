/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortLambdaQP.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

/* Include Files */
#include "sortLambdaQP.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double lambda[27]
 *                int WorkingSet_nActiveConstr
 *                const int WorkingSet_sizes[5]
 *                const int WorkingSet_isActiveIdx[6]
 *                const int WorkingSet_Wid[27]
 *                const int WorkingSet_Wlocalidx[27]
 *                double workspace[378]
 * Return Type  : void
 */
void b_sortLambdaQP(double lambda[27], int WorkingSet_nActiveConstr,
                    const int WorkingSet_sizes[5],
                    const int WorkingSet_isActiveIdx[6],
                    const int WorkingSet_Wid[27],
                    const int WorkingSet_Wlocalidx[27], double workspace[378])
{
  if (WorkingSet_nActiveConstr != 0) {
    int idx;
    int idxOffset;
    int mAll;
    mAll = (WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4];
    idx = (unsigned char)mAll;
    if (idx - 1 >= 0) {
      memcpy(&workspace[0], &lambda[0], (unsigned int)idx * sizeof(double));
    }
    if (mAll - 1 >= 0) {
      memset(&lambda[0], 0, (unsigned int)mAll * sizeof(double));
    }
    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) &&
           (WorkingSet_Wid[idx] <= 2)) {
      if (WorkingSet_Wid[idx] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }
      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }
    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid[idx]) {
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
      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }
  }
}

/*
 * Arguments    : double lambda[31]
 *                int WorkingSet_nActiveConstr
 *                const int WorkingSet_sizes[5]
 *                const int WorkingSet_isActiveIdx[6]
 *                const int WorkingSet_Wid[31]
 *                const int WorkingSet_Wlocalidx[31]
 *                double workspace[496]
 * Return Type  : void
 */
void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr,
                  const int WorkingSet_sizes[5],
                  const int WorkingSet_isActiveIdx[6],
                  const int WorkingSet_Wid[31],
                  const int WorkingSet_Wlocalidx[31], double workspace[496])
{
  if (WorkingSet_nActiveConstr != 0) {
    int idx;
    int idxOffset;
    int mAll;
    mAll = (WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4];
    idx = (unsigned char)mAll;
    if (idx - 1 >= 0) {
      memcpy(&workspace[0], &lambda[0], (unsigned int)idx * sizeof(double));
    }
    if (mAll - 1 >= 0) {
      memset(&lambda[0], 0, (unsigned int)mAll * sizeof(double));
    }
    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) &&
           (WorkingSet_Wid[idx] <= 2)) {
      if (WorkingSet_Wid[idx] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }
      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }
    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid[idx]) {
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
      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
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
