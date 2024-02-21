/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_acc_cascaded_nonlinear_CA_w_ailerons_internal.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:29:34
 */

#ifndef COMPUTE_ACC_CASCADED_NONLINEAR_CA_W_AILERONS_INTERNAL_H
#define COMPUTE_ACC_CASCADED_NONLINEAR_CA_W_AILERONS_INTERNAL_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_compute_acc_cascaded_nonlinea(
    const double u_in[15], double p, double q, double r, double K_p_T,
    double K_p_M, double m, double I_xx, double I_yy, double I_zz, double l_1,
    double l_2, double l_3, double l_4, double l_z, double Cl_alpha,
    double Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero,
    double CL_aileron, double rho, double V, double S, double wing_chord,
    double flight_path_angle, double Beta, double accelerations_array[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_acc_cascaded_nonlinear_CA_w_ailerons_internal.h
 *
 * [EOF]
 */
