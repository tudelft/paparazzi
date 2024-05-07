/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGradLag.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-May-2024 00:26:53
 */

#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_computeGradLag(double workspace[496], int nVar, const double grad[16],
                      const int finiteFixed[16], int mFixed,
                      const int finiteLB[16], int mLB, const int finiteUB[16],
                      int mUB, const double lambda[31]);

void computeGradLag(double workspace[16], int nVar, const double grad[16],
                    const int finiteFixed[16], int mFixed,
                    const int finiteLB[16], int mLB, const int finiteUB[16],
                    int mUB, const double lambda[31]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeGradLag.h
 *
 * [EOF]
 */
