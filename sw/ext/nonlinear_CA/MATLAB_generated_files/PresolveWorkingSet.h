/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: PresolveWorkingSet.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void PresolveWorkingSet(l_struct_T *solution, h_struct_T *memspace,
                        m_struct_T *workingset, f_struct_T *qrmanager);

void b_PresolveWorkingSet(n_struct_T *solution, k_struct_T *memspace,
                          o_struct_T *workingset, i_struct_T *qrmanager);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for PresolveWorkingSet.h
 *
 * [EOF]
 */
