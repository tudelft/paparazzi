/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgeqp3.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xgeqp3.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzgeqp3.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *A
 *                int m
 *                int n
 *                int jpvt_data[]
 *                double tau_data[]
 * Return Type  : int
 */
int xgeqp3(emxArray_real_T *A, int m, int n, int jpvt_data[], double tau_data[])
{
  double vn1_data[311];
  double vn2_data[311];
  double work_data[311];
  double temp;
  double *A_data;
  int b_i;
  int k;
  int minmn_tmp;
  int pvt;
  int tau_size;
  A_data = A->data;
  if (m <= n) {
    minmn_tmp = m;
  } else {
    minmn_tmp = n;
  }
  tau_size = 311;
  memset(&tau_data[0], 0, 311U * sizeof(double));
  if (minmn_tmp < 1) {
    for (pvt = 0; pvt < n; pvt++) {
      jpvt_data[pvt] = pvt + 1;
    }
  } else {
    int i;
    int ix;
    int iy;
    int nfxd;
    int temp_tmp;
    nfxd = 0;
    for (pvt = 0; pvt < n; pvt++) {
      if (jpvt_data[pvt] != 0) {
        nfxd++;
        if (pvt + 1 != nfxd) {
          ix = pvt * 311;
          iy = (nfxd - 1) * 311;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A_data[temp_tmp];
            i = iy + k;
            A_data[temp_tmp] = A_data[i];
            A_data[i] = temp;
          }
          jpvt_data[pvt] = jpvt_data[nfxd - 1];
          jpvt_data[nfxd - 1] = pvt + 1;
        } else {
          jpvt_data[pvt] = pvt + 1;
        }
      } else {
        jpvt_data[pvt] = pvt + 1;
      }
    }
    if (nfxd > minmn_tmp) {
      nfxd = minmn_tmp;
    }
    qrf(A, m, n, nfxd, tau_data);
    A_data = A->data;
    if (nfxd < minmn_tmp) {
      double d;
      memset(&work_data[0], 0, 311U * sizeof(double));
      memset(&vn1_data[0], 0, 311U * sizeof(double));
      memset(&vn2_data[0], 0, 311U * sizeof(double));
      i = nfxd + 1;
      for (pvt = i; pvt <= n; pvt++) {
        d = xnrm2(m - nfxd, A, (nfxd + (pvt - 1) * 311) + 1);
        vn1_data[pvt - 1] = d;
        vn2_data[pvt - 1] = d;
      }
      for (b_i = i; b_i <= minmn_tmp; b_i++) {
        double s;
        int ii;
        int ip1;
        int mmi;
        int nmi;
        ip1 = b_i + 1;
        iy = (b_i - 1) * 311;
        ii = (iy + b_i) - 1;
        nmi = (n - b_i) + 1;
        mmi = m - b_i;
        if (nmi < 1) {
          nfxd = -2;
        } else {
          nfxd = -1;
          if (nmi > 1) {
            temp = fabs(vn1_data[b_i - 1]);
            for (k = 2; k <= nmi; k++) {
              s = fabs(vn1_data[(b_i + k) - 2]);
              if (s > temp) {
                nfxd = k - 2;
                temp = s;
              }
            }
          }
        }
        pvt = b_i + nfxd;
        if (pvt + 1 != b_i) {
          ix = pvt * 311;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A_data[temp_tmp];
            nfxd = iy + k;
            A_data[temp_tmp] = A_data[nfxd];
            A_data[nfxd] = temp;
          }
          nfxd = jpvt_data[pvt];
          jpvt_data[pvt] = jpvt_data[b_i - 1];
          jpvt_data[b_i - 1] = nfxd;
          vn1_data[pvt] = vn1_data[b_i - 1];
          vn2_data[pvt] = vn2_data[b_i - 1];
        }
        if (b_i < m) {
          temp = A_data[ii];
          d = xzlarfg(mmi + 1, &temp, A, ii + 2);
          A_data = A->data;
          tau_data[b_i - 1] = d;
          A_data[ii] = temp;
        } else {
          d = 0.0;
          tau_data[b_i - 1] = 0.0;
        }
        if (b_i < n) {
          temp = A_data[ii];
          A_data[ii] = 1.0;
          xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, ii + 312, work_data);
          A_data = A->data;
          A_data[ii] = temp;
        }
        for (pvt = ip1; pvt <= n; pvt++) {
          nfxd = b_i + (pvt - 1) * 311;
          d = vn1_data[pvt - 1];
          if (d != 0.0) {
            temp = fabs(A_data[nfxd - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }
            s = d / vn2_data[pvt - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < m) {
                d = xnrm2(mmi, A, nfxd + 1);
                vn1_data[pvt - 1] = d;
                vn2_data[pvt - 1] = d;
              } else {
                vn1_data[pvt - 1] = 0.0;
                vn2_data[pvt - 1] = 0.0;
              }
            } else {
              vn1_data[pvt - 1] = d * sqrt(temp);
            }
          }
        }
      }
    }
  }
  return tau_size;
}

/*
 * File trailer for xgeqp3.c
 *
 * [EOF]
 */
