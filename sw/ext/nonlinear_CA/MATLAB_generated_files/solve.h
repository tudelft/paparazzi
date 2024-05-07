/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: solve.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 07-May-2024 15:18:42
 */

#ifndef SOLVE_H
#define SOLVE_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_solve(const j_struct_T *obj, double rhs[14]);

void solve(const g_struct_T *obj, double rhs[16]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for solve.h
 *
 * [EOF]
 */
