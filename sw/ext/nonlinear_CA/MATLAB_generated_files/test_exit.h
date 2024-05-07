/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: test_exit.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 07-May-2024 15:18:42
 */

#ifndef TEST_EXIT_H
#define TEST_EXIT_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_test_exit(q_struct_T *Flags, h_struct_T *memspace,
                 b_struct_T *MeritFunction, const m_struct_T *WorkingSet,
                 l_struct_T *TrialState, f_struct_T *QRManager,
                 const double lb[15], const double ub[15]);

bool c_test_exit(b_struct_T *MeritFunction, const o_struct_T *WorkingSet,
                 n_struct_T *TrialState, const double lb[13],
                 const double ub[13], bool *Flags_fevalOK, bool *Flags_done,
                 bool *Flags_stepAccepted, bool *Flags_failedLineSearch,
                 int *Flags_stepType);

void d_test_exit(q_struct_T *Flags, k_struct_T *memspace,
                 b_struct_T *MeritFunction, const o_struct_T *WorkingSet,
                 n_struct_T *TrialState, i_struct_T *QRManager,
                 const double lb[13], const double ub[13]);

bool test_exit(b_struct_T *MeritFunction, const m_struct_T *WorkingSet,
               l_struct_T *TrialState, const double lb[15], const double ub[15],
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
