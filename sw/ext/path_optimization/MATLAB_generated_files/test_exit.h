/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: test_exit.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef TEST_EXIT_H
#define TEST_EXIT_H

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
void b_test_exit(i_struct_T *Flags, e_struct_T *memspace,
                 struct_T *MeritFunction, f_struct_T *WorkingSet,
                 c_struct_T *TrialState, g_struct_T *QRManager);

bool test_exit(struct_T *MeritFunction, const f_struct_T *WorkingSet,
               c_struct_T *TrialState, bool *Flags_fevalOK, bool *Flags_done,
               bool *Flags_stepAccepted, bool *Flags_failedLineSearch,
               int *Flags_stepType);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for test_exit.h
 *
 * [EOF]
 */
