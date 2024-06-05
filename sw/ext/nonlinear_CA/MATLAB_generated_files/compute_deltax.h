/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_deltax.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Jun-2024 11:13:45
 */

#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void compute_deltax(const double H[225], h_struct_T *solution,
                    f_struct_T *memspace, const d_struct_T *qrmanager,
                    e_struct_T *cholmanager, const struct_T *objective,
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
