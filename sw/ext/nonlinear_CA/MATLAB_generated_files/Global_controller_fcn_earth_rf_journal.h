/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Global_controller_fcn_earth_rf_journal.h
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 20-Nov-2023 22:27:20
 */

#ifndef GLOBAL_CONTROLLER_FCN_EARTH_RF_JOURNAL_H
#define GLOBAL_CONTROLLER_FCN_EARTH_RF_JOURNAL_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void Global_controller_fcn_earth_rf_journal(
    double K_p_T, double K_p_M, double m, double I_xx, double I_yy, double I_zz,
    double l_1, double l_2, double l_3, double l_4, double l_z, double Phi,
    double Theta, double Psi, double Omega_1, double Omega_2, double Omega_3,
    double Omega_4, double b_1, double b_2, double b_3, double b_4, double g_1,
    double g_2, double g_3, double g_4, double W_act_motor_const,
    double W_act_motor_speed, double W_act_tilt_el_const,
    double W_act_tilt_el_speed, double W_act_tilt_az_const,
    double W_act_tilt_az_speed, double W_act_theta_const,
    double W_act_theta_speed, double W_act_phi_const, double W_act_phi_speed,
    double W_dv_1, double W_dv_2, double W_dv_3, double W_dv_4, double W_dv_5,
    double W_dv_6, double max_omega, double min_omega, double max_b,
    double min_b, double max_g, double min_g, double max_theta,
    double min_theta, double max_phi, double dv[6], double p, double q,
    double r, double Cm_zero, double Cl_alpha, double Cd_zero, double K_Cd,
    double Cm_alpha, double rho, double V, double S, double wing_chord,
    double flight_path_angle, double max_alpha, double min_alpha, double Beta,
    double gamma_quadratic_du, double gamma_quadratic_wls,
    double desired_motor_value, double desired_el_value,
    double desired_az_value, double desired_phi_value,
    double desired_theta_value, double controller_id, double verbose,
    double u_out[14], double residuals[6], double *elapsed_time,
    double *N_iterations, double *N_evaluation, double *exitflag);

extern void Global_controller_fcn_earth_rf_journal_initialize(void);

extern void Global_controller_fcn_earth_rf_journal_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Global_controller_fcn_earth_rf_journal.h
 *
 * [EOF]
 */
