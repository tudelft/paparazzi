/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeObjectiveAndUserGradient_.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef COMPUTEOBJECTIVEANDUSERGRADIENT__H
#define COMPUTEOBJECTIVEANDUSERGRADIENT__H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double c_computeObjectiveAndUserGradie(
    const d_struct_T *c_obj_next_next_next_next_next_, const double x[15],
    double grad_workspace[16], int *status);

double d_computeObjectiveAndUserGradie(
    const e_struct_T *c_obj_next_next_next_next_next_, const double x[13],
    double grad_workspace[14], int *status);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeObjectiveAndUserGradient_.h
 *
 * [EOF]
 */
