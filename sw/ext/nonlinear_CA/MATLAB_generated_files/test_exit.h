/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: test_exit.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

#ifndef TEST_EXIT_H
#define TEST_EXIT_H

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_test_exit(k_struct_T *Flags, f_struct_T *memspace,
                 b_struct_T *MeritFunction, const i_struct_T *WorkingSet,
                 h_struct_T *TrialState, d_struct_T *QRManager,
                 const double lb[15], const double ub[15]);

bool test_exit(b_struct_T *MeritFunction, const i_struct_T *WorkingSet,
               h_struct_T *TrialState, const double lb[15], const double ub[15],
               bool *Flags_fevalOK, bool *Flags_done, bool *Flags_stepAccepted,
               bool *Flags_failedLineSearch, int *Flags_stepType);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for test_exit.h
 *
 * [EOF]
 */
