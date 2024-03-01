/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_controller_w_ailerons_rebuttal.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Mar-2024 00:35:29
 */

#ifndef NONLINEAR_CONTROLLER_W_AILERONS_REBUTTAL_H
#define NONLINEAR_CONTROLLER_W_AILERONS_REBUTTAL_H

/* Include Files */
#include "Nonlinear_controller_w_ailerons_rebuttal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void Nonlinear_controller_w_ailerons_rebuttal(
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
    double desired_phi_value, double desired_ailerons_value, double verbose,
    double aoa_protection_speed, double transition_airspeed,
    const double modeled_acc_array[6], double u_out[15], double residuals[6],
    double *elapsed_time, double *N_iterations, double *N_evaluation,
    double *cost_value, double *exitflag);

double compute_cost_and_gradient_fcn(
    const b_captured_var *dv_global, const captured_var *gain_theta,
    const captured_var *Cl_alpha, const captured_var *S, const captured_var *V,
    const captured_var *rho, const captured_var *flight_path_angle,
    const captured_var *Beta, const captured_var *K_Cd,
    const captured_var *Cd_zero, const captured_var *gain_phi,
    const captured_var *K_p_T, const captured_var *gain_motor,
    const captured_var *gain_el, const captured_var *gain_az,
    const captured_var *m, const captured_var *I_zz, const captured_var *p,
    const captured_var *r, const captured_var *I_xx, const captured_var *I_yy,
    const captured_var *l_z, const captured_var *K_p_M,
    const captured_var *Cm_zero, const captured_var *wing_chord,
    const captured_var *l_4, const captured_var *l_3,
    const captured_var *Cm_alpha, const captured_var *q,
    const captured_var *l_1, const captured_var *l_2,
    const captured_var *CL_aileron, const captured_var *gain_ailerons,
    const captured_var *W_dv_4, const captured_var *W_dv_6,
    const captured_var *W_act_motor, const captured_var *gamma_quadratic_du,
    const captured_var *desired_motor_value, const captured_var *W_dv_5,
    const captured_var *W_dv_3, const captured_var *W_dv_1,
    const captured_var *W_dv_2, const captured_var *W_act_tilt_el,
    const captured_var *desired_el_value, const captured_var *W_act_tilt_az,
    const captured_var *desired_az_value, const captured_var *W_act_theta,
    const captured_var *desired_theta_value, const captured_var *W_act_phi,
    const captured_var *desired_phi_value, const captured_var *W_act_ailerons,
    const captured_var *desired_ailerons_value, const double u_in[15],
    double gradient[15]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Nonlinear_controller_w_ailerons_rebuttal.h
 *
 * [EOF]
 */
