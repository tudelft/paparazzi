/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: feasibleratiotest.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double feasibleratiotest(const double solution_xstar_data[],
                         const double solution_searchDir_data[],
                         emxArray_real_T *workspace, int workingset_nVar,
                         const emxArray_real_T *workingset_Aineq,
                         const double workingset_bineq_data[],
                         const double workingset_lb_data[],
                         const int workingset_indexLB_data[],
                         const int workingset_sizes[5],
                         const int workingset_isActiveIdx[6],
                         const bool workingset_isActiveConstr_data[],
                         const int workingset_nWConstr[5], bool isPhaseOne,
                         bool *newBlocking, int *constrType, int *constrIdx);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for feasibleratiotest.h
 *
 * [EOF]
 */
