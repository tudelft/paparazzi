/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Cascaded_nonlinear_controller_w_ail_new_aero.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef CASCADED_NONLINEAR_CONTROLLER_W_AIL_NEW_AERO_H
#define CASCADED_NONLINEAR_CONTROLLER_W_AIL_NEW_AERO_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void Cascaded_nonlinear_controller_w_ail_new_aero(
    double m, double I_xx, double I_yy, double I_zz, double l_1, double l_2,
    double l_3, double l_4, double l_z, double Phi, double Theta,
    double Omega_1, double Omega_2, double Omega_3, double Omega_4, double b_1,
    double b_2, double b_3, double b_4, double g_1, double g_2, double g_3,
    double g_4, double delta_ailerons, double W_act_motor_const,
    double W_act_motor_speed, double W_act_tilt_el_const,
    double W_act_tilt_el_speed, double W_act_tilt_az_const,
    double W_act_tilt_az_speed, double W_act_theta_const,
    double W_act_theta_speed, double W_act_phi_const, double W_act_phi_speed,
    double W_act_ailerons_const, double W_act_ailerons_speed, double W_dv_1,
    double W_dv_2, double W_dv_3, double W_dv_4, double W_dv_5, double W_dv_6,
    double max_omega, double min_omega, double max_b, double min_b,
    double max_g, double min_g, double max_theta, double min_theta,
    double max_phi, double max_delta_ailerons, double min_delta_ailerons,
    double dv[6], double p, double q, double r, double Cm_zero, double Cl_alpha,
    double Cd_zero, double K_Cd, double Cm_alpha, double CL_aileron, double rho,
    double V, double S, double wing_chord, double wing_span,
    double flight_path_angle, double max_alpha, double min_alpha,
    double max_airspeed, double Beta, double gamma_quadratic_du,
    double desired_motor_value, double desired_el_value,
    double desired_az_value, double desired_theta_value,
    double desired_phi_value, double desired_ailerons_value,
    double k_alt_tilt_constraint, double min_alt_tilt_constraint,
    double lidar_alt_corrected, double approach_mode, double verbose,
    double aoa_protection_speed, double transition_speed, double k_d_airspeed,
    double p_body_current, double q_body_current, double r_body_current,
    double p_dot_current, double q_dot_current, double r_dot_current,
    double phi_current, double theta_current, double theta_gain,
    double phi_gain, double p_body_gain, double q_body_gain, double r_body_gain,
    double des_psi_dot, double min_theta_hard, double max_theta_hard,
    double min_phi_hard, double max_phi_hard,
    double c_disable_acc_decrement_inner_l, double vert_acc_margin,
    const double current_accelerations[6],
    const double current_lin_acc_aero_only[3], double power_Cd_0,
    double power_Cd_a, double prop_R, double prop_Cd_0, double prop_Cl_0,
    double prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma,
    double prop_theta, double u_out[15], double residuals[6],
    double *elapsed_time, double *N_iterations_inner_loop,
    double *N_iterations_outer_loop, double *N_evaluations_inner_loop,
    double *N_evaluations_outer_loop);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Cascaded_nonlinear_controller_w_ail_new_aero.h
 *
 * [EOF]
 */
