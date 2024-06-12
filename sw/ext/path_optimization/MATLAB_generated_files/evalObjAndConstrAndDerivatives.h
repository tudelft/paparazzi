/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstrAndDerivatives.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef EVALOBJANDCONSTRANDDERIVATIVES_H
#define EVALOBJANDCONSTRANDDERIVATIVES_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double evalObjAndConstrAndDerivatives(
    const double c_obj_next_next_next_next_next_[3],
    const double d_obj_next_next_next_next_next_[3],
    const double e_obj_next_next_next_next_next_[3],
    const double f_obj_next_next_next_next_next_[3],
    const double g_obj_next_next_next_next_next_[3],
    const double h_obj_next_next_next_next_next_[3],
    const double i_obj_next_next_next_next_next_[3],
    double j_obj_next_next_next_next_next_,
    double k_obj_next_next_next_next_next_,
    const double l_obj_next_next_next_next_next_[18],
    double m_obj_next_next_next_next_next_,
    double n_obj_next_next_next_next_next_,
    double o_obj_next_next_next_next_next_, const double x[18],
    double grad_workspace_data[], double Cineq_workspace_data[],
    double Ceq_workspace_data[], int *Ceq_workspace_size,
    emxArray_real_T *JacIneqTrans_workspace, double JacEqTrans_workspace_data[],
    int *JacEqTrans_workspace_size, int *status);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for evalObjAndConstrAndDerivatives.h
 *
 * [EOF]
 */
