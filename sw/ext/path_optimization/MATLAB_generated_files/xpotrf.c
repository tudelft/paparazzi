/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xpotrf.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xpotrf.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int n
 *                emxArray_real_T *A
 * Return Type  : int
 */
int xpotrf(int n, emxArray_real_T *A)
{
  double *A_data;
  int ia;
  int iac;
  int info;
  int j;
  int nmj;
  bool exitg1;
  A_data = A->data;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= n - 1)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    idxA1j = j * 311;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (nmj = 0; nmj < j; nmj++) {
        c = A_data[idxA1j + nmj];
        ssq += c * c;
      }
    }
    ssq = A_data[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      A_data[idxAjj] = ssq;
      if (j + 1 < n) {
        int i;
        int ia0;
        int idxAjjp1;
        nmj = (n - j) - 2;
        ia0 = idxA1j + 312;
        idxAjjp1 = idxAjj + 312;
        if ((j != 0) && (nmj + 1 != 0)) {
          i = (idxA1j + 311 * nmj) + 312;
          for (iac = ia0; iac <= i; iac += 311) {
            int i1;
            c = 0.0;
            i1 = (iac + j) - 1;
            for (ia = iac; ia <= i1; ia++) {
              c += A_data[ia - 1] * A_data[(idxA1j + ia) - iac];
            }
            i1 = (idxAjj + div_nde_s32_floor((iac - idxA1j) - 312, 311) * 311) +
                 311;
            A_data[i1] -= c;
          }
        }
        ssq = 1.0 / ssq;
        i = (idxAjj + 311 * nmj) + 312;
        for (nmj = idxAjjp1; nmj <= i; nmj += 311) {
          A_data[nmj - 1] *= ssq;
        }
      }
      j++;
    } else {
      A_data[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

/*
 * File trailer for xpotrf.c
 *
 * [EOF]
 */
