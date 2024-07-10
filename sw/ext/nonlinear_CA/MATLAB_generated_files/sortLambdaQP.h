/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortLambdaQP.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Jun-2024 11:13:45
 */

#ifndef SORTLAMBDAQP_H
#define SORTLAMBDAQP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr,
                  const int WorkingSet_sizes[5],
                  const int WorkingSet_isActiveIdx[6],
                  const int WorkingSet_Wid[31],
                  const int WorkingSet_Wlocalidx[31], double workspace[496]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sortLambdaQP.h
 *
 * [EOF]
 */
