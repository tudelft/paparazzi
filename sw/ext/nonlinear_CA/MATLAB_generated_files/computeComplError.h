/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeComplError.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-May-2024 01:26:38
 */

#ifndef COMPUTECOMPLERROR_H
#define COMPUTECOMPLERROR_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double b_computeComplError(const double xCurrent[13], const int finiteLB[14],
                           int mLB, const double lb[13], const int finiteUB[14],
                           int mUB, const double ub[13],
                           const double lambda[27], int iL0);

double computeComplError(const double xCurrent[15], const int finiteLB[16],
                         int mLB, const double lb[15], const int finiteUB[16],
                         int mUB, const double ub[15], const double lambda[31],
                         int iL0);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeComplError.h
 *
 * [EOF]
 */
