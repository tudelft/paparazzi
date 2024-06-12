/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_deltax.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

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
void compute_deltax(const double H[324], c_struct_T *solution,
                    e_struct_T *memspace, const g_struct_T *qrmanager,
                    h_struct_T *cholmanager, const d_struct_T *objective,
                    bool alwaysPositiveDef);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_deltax.h
 *
 * [EOF]
 */
