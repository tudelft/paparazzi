/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: iterate.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

#ifndef ITERATE_H
#define ITERATE_H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_iterate(const double H[169], const double f[14], p_struct_T *solution,
               k_struct_T *memspace, q_struct_T *workingset,
               i_struct_T *qrmanager, j_struct_T *cholmanager,
               c_struct_T *objective, const char options_SolverName[7],
               double options_StepTolerance, double options_ObjectiveLimit,
               int runTimeOptions_MaxIterations);

void iterate(const double H[225], const double f[16], n_struct_T *solution,
             h_struct_T *memspace, o_struct_T *workingset,
             f_struct_T *qrmanager, g_struct_T *cholmanager,
             struct_T *objective, const char options_SolverName[7],
             double options_StepTolerance, double options_ObjectiveLimit,
             int runTimeOptions_MaxIterations);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for iterate.h
 *
 * [EOF]
 */
