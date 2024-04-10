/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: PresolveWorkingSet.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

/* Include Files */
#include "Nonlinear_CA_w_ail_approach_ext_acc_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void PresolveWorkingSet(h_struct_T *solution, f_struct_T *memspace,
                        i_struct_T *workingset, d_struct_T *qrmanager);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for PresolveWorkingSet.h
 *
 * [EOF]
 */
