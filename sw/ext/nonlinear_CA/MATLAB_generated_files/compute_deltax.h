/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_deltax.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:26:49
 */

#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_compute_deltax(const double H[169], p_struct_T *solution,
                      k_struct_T *memspace, const i_struct_T *qrmanager,
                      j_struct_T *cholmanager, const c_struct_T *objective,
                      bool alwaysPositiveDef);

void compute_deltax(const double H[225], n_struct_T *solution,
                    h_struct_T *memspace, const f_struct_T *qrmanager,
                    g_struct_T *cholmanager, const struct_T *objective,
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
