/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_cost_and_gradient_w_Mx_noah.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Jun-2024 11:13:45
 */

#ifndef COMPUTE_COST_AND_GRADIENT_W_MX_NOAH_H
#define COMPUTE_COST_AND_GRADIENT_W_MX_NOAH_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double c_compute_cost_and_gradient_w_M(
    double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
    double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz,
    double K_Cd, double Omega_1_scaled, double Omega_2_scaled,
    double Omega_3_scaled, double Omega_4_scaled, double Phi_scaled, double S,
    double Theta_scaled, double V, double V_scaled, double W_act_phi,
    double W_act_theta, double W_act_motor, double W_dv_1, double W_dv_2,
    double W_dv_3, double W_dv_4, double W_dv_5, double W_dv_6,
    double W_act_tilt_el, double W_act_tilt_az, double W_act_ailerons,
    double b_1_scaled, double b_2_scaled, double b_3_scaled, double b_4_scaled,
    double delta_ailerons_scaled, double desired_el_value,
    double desired_az_value, double desired_phi_value,
    double desired_theta_value, double desired_motor_value,
    double desired_ailerons_value, double dv_global_1, double dv_global_2,
    double dv_global_3, double dv_global_4, double dv_global_5,
    double dv_global_6, double flight_path_angle, double g_1_scaled,
    double g_2_scaled, double g_3_scaled, double g_4_scaled, double gain_el,
    double gain_az, double gain_phi, double gain_theta, double gain_motor,
    double gain_ailerons, double gamma_quadratic_du, double l_1, double l_2,
    double l_3, double l_4, double l_z, double m, double p, double power_Cd_0,
    double power_Cd_a, double prop_R, double prop_Cd_0, double prop_Cl_0,
    double prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma,
    double prop_theta, double q, double r, double rho, double wing_span,
    double wing_chord, double gradient_data[], int *gradient_size);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_cost_and_gradient_w_Mx_noah.h
 *
 * [EOF]
 */
