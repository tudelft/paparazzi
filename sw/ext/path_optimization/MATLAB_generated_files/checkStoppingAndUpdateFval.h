/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: checkStoppingAndUpdateFval.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef CHECKSTOPPINGANDUPDATEFVAL_H
#define CHECKSTOPPINGANDUPDATEFVAL_H

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
void checkStoppingAndUpdateFval(int *activeSetChangeID, const double f_data[],
                                c_struct_T *solution, e_struct_T *memspace,
                                const d_struct_T *objective,
                                f_struct_T *workingset, g_struct_T *qrmanager,
                                double options_ObjectiveLimit,
                                int runTimeOptions_MaxIterations,
                                bool updateFval);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for checkStoppingAndUpdateFval.h
 *
 * [EOF]
 */
