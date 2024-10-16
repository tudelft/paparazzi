/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_controller_w_ail_basic_aero_outer_loop.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 10-Oct-2024 12:23:54
 */

#ifndef NONLINEAR_CONTROLLER_W_AIL_BASIC_AERO_OUTER_LOOP_H
#define NONLINEAR_CONTROLLER_W_AIL_BASIC_AERO_OUTER_LOOP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void Nonlinear_controller_w_ail_basic_aero_outer_loop(
    double K_p_T, double K_p_M, double m, double I_xx, double I_yy, double I_zz,
    double l_1, double l_2, double l_3, double l_4, double l_z, double Phi,
    double Theta, double Omega_1, double Omega_2, double Omega_3,
    double Omega_4, double b_1, double b_2, double b_3, double b_4, double g_1,
    double g_2, double g_3, double g_4, double delta_ailerons,
    double W_act_motor_const, double W_act_motor_speed,
    double W_act_tilt_el_const, double W_act_tilt_el_speed,
    double W_act_tilt_az_const, double W_act_tilt_az_speed,
    double W_act_theta_const, double W_act_theta_speed, double W_act_phi_const,
    double W_act_phi_speed, double W_act_ailerons_const,
    double W_act_ailerons_speed, double W_dv_1, double W_dv_2, double W_dv_3,
    double W_dv_4, double W_dv_5, double W_dv_6, double max_omega,
    double min_omega, double max_b, double min_b, double max_g, double min_g,
    double max_theta, double min_theta, double max_phi,
    double max_delta_ailerons, double min_delta_ailerons, const double dv[6],
    double p, double q, double r, double Cm_zero, double Cl_alpha,
    double Cd_zero, double K_Cd, double Cm_alpha, double CL_aileron, double rho,
    double V, double S, double wing_chord, double flight_path_angle,
    double max_alpha, double min_alpha, double Beta, double gamma_quadratic_du,
    double desired_motor_value, double desired_el_value,
    double desired_az_value, double desired_theta_value,
    double desired_phi_value, double desired_ailerons_value,
    double k_alt_tilt_constraint, double min_alt_tilt_constraint,
    double lidar_alt_corrected, double approach_mode, double verbose,
    double aoa_protection_speed, double vert_acc_margin, double p_body_current,
    double q_body_current, double r_body_current, double p_dot_current,
    double q_dot_current, double r_dot_current, double phi_current,
    double theta_current, double angular_proportional_gains[3],
    double angular_derivative_gains[3], double theta_hard_max,
    double theta_hard_min, double phi_hard_max, double phi_hard_min,
    double k_d_airspeed, double des_psi_dot,
    const double current_accelerations[6], const double u_init[15],
    double use_u_init, double W_act_motor_du, double W_act_tilt_el_du,
    double W_act_tilt_az_du, double W_act_theta_du, double W_act_phi_du,
    double W_act_ailerons_du, double gamma_quadratic_du2,
    double induced_failure, double W_act_motor_failure,
    double W_act_tilt_el_failure, double W_act_tilt_az_failure,
    double W_act_theta_failure, double W_act_phi_failure,
    double W_act_ailerons_failure, double W_dv_1_failure, double W_dv_2_failure,
    double W_dv_3_failure, double W_dv_4_failure, double W_dv_5_failure,
    double W_dv_6_failure, double gamma_quadratic_du_failure, double u_out[15],
    double dv_pqr_dot_target[3], double acc_decrement_aero[6],
    double residuals[6], double *elapsed_time, double *N_iterations,
    double *N_evaluation, double *exitflag);

extern void Nonlinear_controller_w_ail_basic_aero_outer_loop_initialize(void);

extern void Nonlinear_controller_w_ail_basic_aero_outer_loop_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Nonlinear_controller_w_ail_basic_aero_outer_loop.h
 *
 * [EOF]
 */