/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: iterate.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef ITERATE_H
#define ITERATE_H

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
void iterate(const double H[324], const double f_data[], c_struct_T *solution,
             e_struct_T *memspace, f_struct_T *workingset,
             g_struct_T *qrmanager, h_struct_T *cholmanager,
             d_struct_T *objective, const char options_SolverName[7],
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
