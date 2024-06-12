/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: linesearch.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef LINESEARCH_H
#define LINESEARCH_H

/* Include Files */
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double linesearch(bool *evalWellDefined, int WorkingSet_nVar,
                  c_struct_T *TrialState, double MeritFunction_penaltyParam,
                  double MeritFunction_phi, double MeritFunction_phiPrimePlus,
                  double MeritFunction_phiFullStep,
                  const b_struct_T *c_FcnEvaluator_next_next_next_n,
                  bool socTaken, int *exitflag);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for linesearch.h
 *
 * [EOF]
 */
