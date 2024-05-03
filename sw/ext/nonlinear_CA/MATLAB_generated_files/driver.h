/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: driver.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef DRIVER_H
#define DRIVER_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_driver(const double H[169], const double f[14], n_struct_T *solution,
              k_struct_T *memspace, o_struct_T *workingset,
              i_struct_T *qrmanager, j_struct_T *cholmanager,
              c_struct_T *objective, p_struct_T *options,
              int runTimeOptions_MaxIterations);

void driver(const double H[225], const double f[16], l_struct_T *solution,
            h_struct_T *memspace, m_struct_T *workingset, f_struct_T *qrmanager,
            g_struct_T *cholmanager, struct_T *objective, p_struct_T *options,
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
