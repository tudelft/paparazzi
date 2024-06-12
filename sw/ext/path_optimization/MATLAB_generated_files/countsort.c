/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: countsort.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "countsort.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int x_data[]
 *                int xLen
 *                int workspace_data[]
 *                int xMin
 *                int xMax
 * Return Type  : void
 */
void countsort(int x_data[], int xLen, int workspace_data[], int xMin, int xMax)
{
  int idx;
  int idxFill;
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    if (idxStart >= 0) {
      memset(&workspace_data[0], 0, (unsigned int)(idxStart + 1) * sizeof(int));
    }
    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x_data[idx] - xMin;
      workspace_data[idxStart]++;
    }
    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace_data[idx - 1] += workspace_data[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace_data[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x_data[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace_data[idx] + 1;
      idxEnd = workspace_data[idx + 1];
    }
    for (idx = idxStart; idx <= idxEnd; idx++) {
      x_data[idx - 1] = xMax;
    }
  }
}

/*
 * File trailer for countsort.c
 *
 * [EOF]
 */
