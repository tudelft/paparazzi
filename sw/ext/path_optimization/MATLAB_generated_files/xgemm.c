/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemm.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "xgemm.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const emxArray_real_T *A
 *                int ia0
 *                const emxArray_real_T *B
 *                emxArray_real_T *C
 * Return Type  : void
 */
void b_xgemm(int m, int n, int k, const emxArray_real_T *A, int ia0,
             const emxArray_real_T *B, emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int cr;
  int ic;
  int w;
  C_data = C->data;
  B_data = B->data;
  A_data = A->data;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = 311 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 311) {
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }
    br = -1;
    for (cr = 0; cr <= lastColC; cr += 311) {
      int ar;
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (w = 0; w < k; w++) {
          temp += A_data[(w + ar) - 1] * B_data[(w + br) + 1];
        }
        C_data[ic - 1] += temp;
        ar += 311;
      }
      br += 311;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const double A[324]
 *                int lda
 *                const emxArray_real_T *B
 *                int ib0
 *                emxArray_real_T *C
 * Return Type  : void
 */
void xgemm(int m, int n, int k, const double A[324], int lda,
           const emxArray_real_T *B, int ib0, emxArray_real_T *C)
{
  const double *B_data;
  double *C_data;
  int cr;
  int ib;
  int ic;
  C_data = C->data;
  B_data = B->data;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    br = ib0;
    lastColC = 311 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 311) {
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        C_data[ic - 1] = 0.0;
      }
    }
    for (cr = 0; cr <= lastColC; cr += 311) {
      int ar;
      ar = -1;
      i = br + k;
      for (ib = br; ib < i; ib++) {
        int i2;
        i1 = cr + 1;
        i2 = cr + m;
        for (ic = i1; ic <= i2; ic++) {
          C_data[ic - 1] += B_data[ib - 1] * A[(ar + ic) - cr];
        }
        ar += lda;
      }
      br += 311;
    }
  }
}

/*
 * File trailer for xgemm.c
 *
 * [EOF]
 */
