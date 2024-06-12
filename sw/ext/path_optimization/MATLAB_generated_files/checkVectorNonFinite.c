/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: checkVectorNonFinite.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "checkVectorNonFinite.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int N
 *                const double vec_data[]
 * Return Type  : int
 */
int checkVectorNonFinite(int N, const double vec_data[])
{
  int idx_current;
  int status;
  bool allFinite;
  status = 1;
  allFinite = true;
  idx_current = 0;
  while (allFinite && (idx_current + 1 <= N)) {
    allFinite = ((!rtIsInf(vec_data[idx_current])) &&
                 (!rtIsNaN(vec_data[idx_current])));
    idx_current++;
  }
  if (!allFinite) {
    idx_current--;
    if (rtIsNaN(vec_data[idx_current])) {
      status = -3;
    } else if (vec_data[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  return status;
}

/*
 * File trailer for checkVectorNonFinite.c
 *
 * [EOF]
 */
