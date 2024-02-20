/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: countsort.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

/* Include Files */
#include "countsort.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int x[27]
 *                int xLen
 *                int workspace[27]
 *                int xMin
 *                int xMax
 * Return Type  : void
 */
void b_countsort(int x[27], int xLen, int workspace[27], int xMin, int xMax)
{
  int idx;
  int idxFill;
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    if (idxStart >= 0) {
      memset(&workspace[0], 0, (unsigned int)(idxStart + 1) * sizeof(int));
    }
    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart]++;
    }
    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] += workspace[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }
    for (idx = idxStart; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

/*
 * Arguments    : int x[31]
 *                int xLen
 *                int workspace[31]
 *                int xMin
 *                int xMax
 * Return Type  : void
 */
void countsort(int x[31], int xLen, int workspace[31], int xMin, int xMax)
{
  int idx;
  int idxFill;
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    if (idxStart >= 0) {
      memset(&workspace[0], 0, (unsigned int)(idxStart + 1) * sizeof(int));
    }
    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart]++;
    }
    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] += workspace[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }
    for (idx = idxStart; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

/*
 * File trailer for countsort.c
 *
 * [EOF]
 */
