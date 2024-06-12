/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: feasibleX0ForWorkingSet.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef FEASIBLEX0FORWORKINGSET_H
#define FEASIBLEX0FORWORKINGSET_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
bool feasibleX0ForWorkingSet(emxArray_real_T *workspace, double xCurrent_data[],
                             f_struct_T *workingset, g_struct_T *qrmanager);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for feasibleX0ForWorkingSet.h
 *
 * [EOF]
 */
