/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: feasibleX0ForWorkingSet.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

#ifndef FEASIBLEX0FORWORKINGSET_H
#define FEASIBLEX0FORWORKINGSET_H

/* Include Files */
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
bool b_feasibleX0ForWorkingSet(double workspace[378], double xCurrent[14],
                               const q_struct_T *workingset,
                               i_struct_T *qrmanager);

bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
                             const o_struct_T *workingset,
                             f_struct_T *qrmanager);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for feasibleX0ForWorkingSet.h
 *
 * [EOF]
 */
