/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstrAndDerivatives.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

#ifndef EVALOBJANDCONSTRANDDERIVATIVES_H
#define EVALOBJANDCONSTRANDDERIVATIVES_H

/* Include Files */
#include "Nonlinear_CA_w_ail_approach_ext_acc_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double evalObjAndConstrAndDerivatives(
    const c_struct_T *c_obj_next_next_next_next_next_, const double x[15],
    double grad_workspace[16], int *status);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for evalObjAndConstrAndDerivatives.h
 *
 * [EOF]
 */
