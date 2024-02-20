/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeComplError.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

/* Include Files */
#include "computeComplError.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double xCurrent[13]
 *                const int finiteLB[14]
 *                int mLB
 *                const double lb[13]
 *                const int finiteUB[14]
 *                int mUB
 *                const double ub[13]
 *                const double lambda[27]
 *                int iL0
 * Return Type  : double
 */
double b_computeComplError(const double xCurrent[13], const int finiteLB[14],
                           int mLB, const double lb[13], const int finiteUB[14],
                           int mUB, const double ub[13],
                           const double lambda[27], int iL0)
{
  double nlpComplError;
  int idx;
  nlpComplError = 0.0;
  if (mLB + mUB > 0) {
    double lbDelta;
    double lbLambda;
    int i;
    int i1;
    int ubOffset;
    ubOffset = (iL0 + mLB) - 1;
    i = (unsigned char)mLB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteLB[idx];
      lbDelta = xCurrent[i1 - 1] - lb[i1 - 1];
      lbLambda = lambda[(iL0 + idx) - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda),
                                               fmin(fabs(lbDelta), lbLambda)));
    }
    i = (unsigned char)mUB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteUB[idx];
      lbDelta = ub[i1 - 1] - xCurrent[i1 - 1];
      lbLambda = lambda[ubOffset + idx];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda),
                                               fmin(fabs(lbDelta), lbLambda)));
    }
  }
  return nlpComplError;
}

/*
 * Arguments    : const double xCurrent[15]
 *                const int finiteLB[16]
 *                int mLB
 *                const double lb[15]
 *                const int finiteUB[16]
 *                int mUB
 *                const double ub[15]
 *                const double lambda[31]
 *                int iL0
 * Return Type  : double
 */
double computeComplError(const double xCurrent[15], const int finiteLB[16],
                         int mLB, const double lb[15], const int finiteUB[16],
                         int mUB, const double ub[15], const double lambda[31],
                         int iL0)
{
  double nlpComplError;
  int idx;
  nlpComplError = 0.0;
  if (mLB + mUB > 0) {
    double lbDelta;
    double lbLambda;
    int i;
    int i1;
    int ubOffset;
    ubOffset = (iL0 + mLB) - 1;
    i = (unsigned char)mLB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteLB[idx];
      lbDelta = xCurrent[i1 - 1] - lb[i1 - 1];
      lbLambda = lambda[(iL0 + idx) - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda),
                                               fmin(fabs(lbDelta), lbLambda)));
    }
    i = (unsigned char)mUB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteUB[idx];
      lbDelta = ub[i1 - 1] - xCurrent[i1 - 1];
      lbLambda = lambda[ubOffset + idx];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda),
                                               fmin(fabs(lbDelta), lbLambda)));
    }
  }
  return nlpComplError;
}

/*
 * File trailer for computeComplError.c
 *
 * [EOF]
 */
