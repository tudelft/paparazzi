/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstr.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef EVALOBJANDCONSTR_H
#define EVALOBJANDCONSTR_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double b_evalObjAndConstr(const e_struct_T *c_obj_next_next_next_next_next_,
                          const double x[13], int *status);

double evalObjAndConstr(const d_struct_T *c_obj_next_next_next_next_next_,
                        const double x[15], int *status);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for evalObjAndConstr.h
 *
 * [EOF]
 */
