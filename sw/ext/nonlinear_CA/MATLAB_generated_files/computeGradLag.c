/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGradLag.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:26:49
 */

/* Include Files */
#include "computeGradLag.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double workspace[496]
 *                int nVar
 *                const double grad[16]
 *                const int finiteFixed[16]
 *                int mFixed
 *                const int finiteLB[16]
 *                int mLB
 *                const int finiteUB[16]
 *                int mUB
 *                const double lambda[31]
 * Return Type  : void
 */
void b_computeGradLag(double workspace[496], int nVar, const double grad[16],
                      const int finiteFixed[16], int mFixed,
                      const int finiteLB[16], int mLB, const int finiteUB[16],
                      int mUB, const double lambda[31])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }
  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }
  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }
  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : double workspace[14]
 *                int nVar
 *                const double grad[14]
 *                const int finiteFixed[14]
 *                int mFixed
 *                const int finiteLB[14]
 *                int mLB
 *                const int finiteUB[14]
 *                int mUB
 *                const double lambda[27]
 * Return Type  : void
 */
void c_computeGradLag(double workspace[14], int nVar, const double grad[14],
                      const int finiteFixed[14], int mFixed,
                      const int finiteLB[14], int mLB, const int finiteUB[14],
                      int mUB, const double lambda[27])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }
  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }
  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }
  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : double workspace[16]
 *                int nVar
 *                const double grad[16]
 *                const int finiteFixed[16]
 *                int mFixed
 *                const int finiteLB[16]
 *                int mLB
 *                const int finiteUB[16]
 *                int mUB
 *                const double lambda[31]
 * Return Type  : void
 */
void computeGradLag(double workspace[16], int nVar, const double grad[16],
                    const int finiteFixed[16], int mFixed,
                    const int finiteLB[16], int mLB, const int finiteUB[16],
                    int mUB, const double lambda[31])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }
  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }
  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }
  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : double workspace[378]
 *                int nVar
 *                const double grad[14]
 *                const int finiteFixed[14]
 *                int mFixed
 *                const int finiteLB[14]
 *                int mLB
 *                const int finiteUB[14]
 *                int mUB
 *                const double lambda[27]
 * Return Type  : void
 */
void d_computeGradLag(double workspace[378], int nVar, const double grad[14],
                      const int finiteFixed[14], int mFixed,
                      const int finiteLB[14], int mLB, const int finiteUB[14],
                      int mUB, const double lambda[27])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }
  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }
  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }
  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * File trailer for computeGradLag.c
 *
 * [EOF]
 */
