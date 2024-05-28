/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_acc_nonlinear_control_rf_w_Mx_noah.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

#ifndef COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_MX_NOAH_H
#define COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_MX_NOAH_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_compute_acc_nonlinear_control(
    double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
    double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz,
    double K_Cd, double Omega_1, double Omega_2, double Omega_3, double Omega_4,
    double Omega_1_scaled, double Omega_2_scaled, double Phi, double S,
    double Theta, double V, double V_scaled, double b_1, double b_2, double b_3,
    double b_4, double delta_ailerons, double flight_path_angle, double g_1,
    double g_2, double g_3, double g_4, double l_1, double l_2, double l_3,
    double l_4, double l_z, double m, double p, double prop_R, double prop_Cd_0,
    double prop_Cl_0, double prop_Cd_a, double prop_Cl_a, double prop_delta,
    double prop_sigma, double prop_theta, double q, double r, double rho,
    double wing_span, double wing_chord, double accelerations_array[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_acc_nonlinear_control_rf_w_Mx_noah.h
 *
 * [EOF]
 */
