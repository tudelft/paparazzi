/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: feasibleratiotest.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double feasibleratiotest(
    const double solution_xstar[16], const double solution_searchDir[16],
    int workingset_nVar, const double workingset_lb[16],
    const double workingset_ub[16], const int workingset_indexLB[16],
    const int workingset_indexUB[16], const int workingset_sizes[5],
    const int workingset_isActiveIdx[6],
    const bool workingset_isActiveConstr[31], const int workingset_nWConstr[5],
    bool isPhaseOne, bool *newBlocking, int *constrType, int *constrIdx);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for feasibleratiotest.h
 *
 * [EOF]
 */
