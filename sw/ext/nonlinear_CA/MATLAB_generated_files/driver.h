/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: driver.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

#ifndef DRIVER_H
#define DRIVER_H

/* Include Files */
#include "Nonlinear_controller_w_ail_basic_aero_sl_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void driver(const double H[225], const double f[16], h_struct_T *solution,
            f_struct_T *memspace, i_struct_T *workingset, d_struct_T *qrmanager,
            e_struct_T *cholmanager, struct_T *objective, j_struct_T *options,
            int runTimeOptions_MaxIterations);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for driver.h
 *
 * [EOF]
 */
