/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: iterate.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

#ifndef ITERATE_H
#define ITERATE_H

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void iterate(const double H[225], const double f[16], h_struct_T *solution,
             f_struct_T *memspace, i_struct_T *workingset,
             d_struct_T *qrmanager, e_struct_T *cholmanager,
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
