/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlarf.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xzlarf.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xgerc.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                int iv0
 *                double tau
 *                emxArray_real_T *C
 *                int ic0
 *                double work_data[]
 * Return Type  : void
 */
void xzlarf(int m, int n, int iv0, double tau, emxArray_real_T *C, int ic0,
            double work_data[])
{
  double *C_data;
  int i;
  int ia;
  int iac;
  int lastc;
  int lastv;
  C_data = C->data;
  if (tau != 0.0) {
    bool exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C_data[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int exitg1;
      i = ic0 + (lastc - 1) * 311;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }
  if (lastv > 0) {
    if (lastc != 0) {
      if (lastc - 1 >= 0) {
        memset(&work_data[0], 0, (unsigned int)lastc * sizeof(double));
      }
      i = ic0 + 311 * (lastc - 1);
      for (iac = ic0; iac <= i; iac += 311) {
        double c;
        int b_i;
        c = 0.0;
        b_i = (iac + lastv) - 1;
        for (ia = iac; ia <= b_i; ia++) {
          c += C_data[ia - 1] * C_data[((iv0 + ia) - iac) - 1];
        }
        b_i = div_nde_s32_floor(iac - ic0, 311);
        work_data[b_i] += c;
      }
    }
    xgerc(lastv, lastc, -tau, iv0, work_data, C, ic0);
  }
}

/*
 * File trailer for xzlarf.c
 *
 * [EOF]
 */
