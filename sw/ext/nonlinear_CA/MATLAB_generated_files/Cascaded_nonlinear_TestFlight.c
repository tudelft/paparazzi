/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Cascaded_nonlinear_TestFlight.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

/* Include Files */
#include "Cascaded_nonlinear_TestFlight.h"
#include "Cascaded_nonlinear_TestFlight_data.h"
#include "Cascaded_nonlinear_TestFlight_initialize.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "compute_acc_cascaded_nonlinear_CA_w_ailerons_internal.h"
#include "fmincon.h"
#include "mean.h"
#include "minOrMax.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Function Definitions */
/*
 * Create variables necessary for the optimization
 *
 * Arguments    : double m_failure_ID
 *                double K_p_T
 *                double K_p_M
 *                double m
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double Phi
 *                double Theta
 *                double Omega_1
 *                double Omega_2
 *                double Omega_3
 *                double Omega_4
 *                double b_1
 *                double b_2
 *                double b_3
 *                double b_4
 *                double g_1
 *                double g_2
 *                double g_3
 *                double g_4
 *                double delta_ailerons
 *                double W_act_theta_du
 *                double W_act_phi_du
 *                double W_act_motor_const
 *                double W_act_motor_speed
 *                double W_act_tilt_el_const
 *                double W_act_tilt_el_speed
 *                double W_act_tilt_az_const
 *                double W_act_tilt_az_speed
 *                double W_act_theta_const
 *                double W_act_theta_speed
 *                double W_act_phi_const
 *                double W_act_phi_speed
 *                double W_act_ailerons_const
 *                double W_act_ailerons_speed
 *                double W_dv_1
 *                double W_dv_2
 *                double W_dv_3
 *                double W_dv_4
 *                double W_dv_5
 *                double W_dv_6
 *                double max_omega
 *                double min_omega
 *                double max_b
 *                double min_b
 *                double max_g
 *                double min_g
 *                double max_theta
 *                double min_theta
 *                double max_phi
 *                double max_delta_ailerons
 *                double min_delta_ailerons
 *                double dv[6]
 *                double p
 *                double q
 *                double r
 *                double Cm_zero
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double CL_aileron
 *                double rho
 *                double V
 *                double W_act_motor_du
 *                double W_act_tilt_el_du
 *                double W_act_tilt_az_du
 *                double W_act_ailerons_du
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double max_alpha
 *                double min_alpha
 *                double Beta
 *                double gamma_quadratic_du
 *                double gamma_quadratic_du2
 *                double desired_motor_value
 *                double desired_el_value
 *                double desired_az_value
 *                double desired_theta_value
 *                double desired_phi_value
 *                double desired_ailerons_value
 *                double k_alt_tilt_constraint
 *                double min_alt_tilt_constraint
 *                double lidar_alt_corrected
 *                double approach_mode
 *                double verbose
 *                double aoa_protection_speed
 *                double transition_speed
 *                double p_body_current
 *                double q_body_current
 *                double r_body_current
 *                double p_dot_current
 *                double q_dot_current
 *                double r_dot_current
 *                double phi_current
 *                double theta_current
 *                double theta_gain
 *                double phi_gain
 *                double p_body_gain
 *                double q_body_gain
 *                double r_body_gain
 *                double des_psi_dot
 *                const double current_accelerations[6]
 *                double u_out_local[15]
 *                double residuals[6]
 *                double gradient[13]
 *                double *cost
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluation
 *                double *exitflag
 * Return Type  : void
 */
void Cascaded_nonlinear_TestFlight(
    double m_failure_ID, double K_p_T, double K_p_M, double m, double I_xx,
    double I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4,
    double l_z, double Phi, double Theta, double Omega_1, double Omega_2,
    double Omega_3, double Omega_4, double b_1, double b_2, double b_3,
    double b_4, double g_1, double g_2, double g_3, double g_4,
    double delta_ailerons, double W_act_theta_du, double W_act_phi_du,
    double W_act_motor_const, double W_act_motor_speed,
    double W_act_tilt_el_const, double W_act_tilt_el_speed,
    double W_act_tilt_az_const, double W_act_tilt_az_speed,
    double W_act_theta_const, double W_act_theta_speed, double W_act_phi_const,
    double W_act_phi_speed, double W_act_ailerons_const,
    double W_act_ailerons_speed, double W_dv_1, double W_dv_2, double W_dv_3,
    double W_dv_4, double W_dv_5, double W_dv_6, double max_omega,
    double min_omega, double max_b, double min_b, double max_g, double min_g,
    double max_theta, double min_theta, double max_phi,
    double max_delta_ailerons, double min_delta_ailerons, double dv[6],
    double p, double q, double r, double Cm_zero, double Cl_alpha,
    double Cd_zero, double K_Cd, double Cm_alpha, double CL_aileron, double rho,
    double V, double W_act_motor_du, double W_act_tilt_el_du,
    double W_act_tilt_az_du, double W_act_ailerons_du, double S,
    double wing_chord, double flight_path_angle, double max_alpha,
    double min_alpha, double Beta, double gamma_quadratic_du,
    double gamma_quadratic_du2, double desired_motor_value,
    double desired_el_value, double desired_az_value,
    double desired_theta_value, double desired_phi_value,
    double desired_ailerons_value, double k_alt_tilt_constraint,
    double min_alt_tilt_constraint, double lidar_alt_corrected,
    double approach_mode, double verbose, double aoa_protection_speed,
    double transition_speed, double p_body_current, double q_body_current,
    double r_body_current, double p_dot_current, double q_dot_current,
    double r_dot_current, double phi_current, double theta_current,
    double theta_gain, double phi_gain, double p_body_gain, double q_body_gain,
    double r_body_gain, double des_psi_dot,
    const double current_accelerations[6], double u_out_local[15],
    double residuals[6], double gradient[13], double *cost,
    double *elapsed_time, double *N_iterations, double *N_evaluation,
    double *exitflag)
{
  b_captured_var dv_global;
  c_captured_var actual_u;
  captured_var W_act_ailerons;
  captured_var W_act_motor;
  captured_var W_act_phi;
  captured_var W_act_theta;
  captured_var W_act_tilt_az;
  captured_var W_act_tilt_el;
  captured_var b_Beta;
  captured_var b_CL_aileron;
  captured_var b_Cd_zero;
  captured_var b_Cl_alpha;
  captured_var b_Cm_alpha;
  captured_var b_Cm_zero;
  captured_var b_I_xx;
  captured_var b_I_yy;
  captured_var b_I_zz;
  captured_var b_K_Cd;
  captured_var b_K_p_M;
  captured_var b_K_p_T;
  captured_var b_Phi;
  captured_var b_S;
  captured_var b_Theta;
  captured_var b_V;
  captured_var b_W_act_ailerons_du;
  captured_var b_W_act_motor_du;
  captured_var b_W_act_phi_du;
  captured_var b_W_act_theta_du;
  captured_var b_W_act_tilt_az_du;
  captured_var b_W_act_tilt_el_du;
  captured_var b_W_dv_1;
  captured_var b_W_dv_2;
  captured_var b_W_dv_3;
  captured_var b_W_dv_4;
  captured_var b_W_dv_5;
  captured_var b_W_dv_6;
  captured_var b_desired_ailerons_value;
  captured_var b_desired_az_value;
  captured_var b_desired_el_value;
  captured_var b_desired_motor_value;
  captured_var b_desired_phi_value;
  captured_var b_desired_theta_value;
  captured_var b_flight_path_angle;
  captured_var b_gamma_quadratic_du;
  captured_var b_gamma_quadratic_du2;
  captured_var b_l_1;
  captured_var b_l_2;
  captured_var b_l_3;
  captured_var b_l_4;
  captured_var b_l_z;
  captured_var b_m;
  captured_var b_p;
  captured_var b_q;
  captured_var b_r;
  captured_var b_rho;
  captured_var b_wing_chord;
  captured_var gain_ailerons;
  captured_var gain_az;
  captured_var gain_el;
  captured_var gain_motor;
  captured_var gain_phi;
  captured_var gain_theta;
  d_struct_T b_expl_temp;
  d_struct_T expl_temp;
  e_struct_T d_expl_temp;
  e_struct_T e_expl_temp;
  l_struct_T a__3;
  double u_max[15];
  double u_max_scaled[15];
  double u_min[15];
  double u_min_scaled[15];
  double b_u_max_scaled[13];
  double b_u_min_scaled[13];
  double u_out[13];
  double b_dv[9];
  double final_accelerations[6];
  double des_body_rates[3];
  double b_max_approach;
  double b_min_approach;
  double d;
  double des_phi_first_iteration;
  double des_theta_first_iteration;
  double g_max_approach;
  double g_min_approach;
  double max_theta_protection;
  double max_tilt_value_approach;
  double min_theta_protection;
  double output_funcCount;
  double output_iterations;
  int i;
  char c_expl_temp[3];
  if (!isInitialized_Cascaded_nonlinear_TestFlight) {
    Cascaded_nonlinear_TestFlight_initialize();
  }
  b_K_p_T.contents = K_p_T;
  b_K_p_M.contents = K_p_M;
  b_m.contents = m;
  b_I_xx.contents = I_xx;
  b_I_yy.contents = I_yy;
  b_I_zz.contents = I_zz;
  b_l_1.contents = l_1;
  b_l_2.contents = l_2;
  b_l_3.contents = l_3;
  b_l_4.contents = l_4;
  b_l_z.contents = l_z;
  b_Phi.contents = Phi;
  b_Theta.contents = Theta;
  b_W_act_theta_du.contents = W_act_theta_du;
  b_W_act_phi_du.contents = W_act_phi_du;
  b_W_dv_1.contents = W_dv_1;
  b_W_dv_2.contents = W_dv_2;
  b_W_dv_3.contents = W_dv_3;
  b_W_dv_4.contents = W_dv_4;
  b_W_dv_5.contents = W_dv_5;
  b_W_dv_6.contents = W_dv_6;
  b_p.contents = p;
  b_q.contents = q;
  b_r.contents = r;
  b_Cm_zero.contents = Cm_zero;
  b_Cl_alpha.contents = Cl_alpha;
  b_Cd_zero.contents = Cd_zero;
  b_K_Cd.contents = K_Cd;
  b_Cm_alpha.contents = Cm_alpha;
  b_CL_aileron.contents = CL_aileron;
  b_rho.contents = rho;
  b_V.contents = V;
  b_W_act_motor_du.contents = W_act_motor_du;
  b_W_act_tilt_el_du.contents = W_act_tilt_el_du;
  b_W_act_tilt_az_du.contents = W_act_tilt_az_du;
  b_W_act_ailerons_du.contents = W_act_ailerons_du;
  b_S.contents = S;
  b_wing_chord.contents = wing_chord;
  b_flight_path_angle.contents = flight_path_angle;
  b_Beta.contents = Beta;
  b_gamma_quadratic_du.contents = gamma_quadratic_du;
  b_gamma_quadratic_du2.contents = gamma_quadratic_du2;
  b_desired_motor_value.contents = desired_motor_value;
  b_desired_el_value.contents = desired_el_value;
  b_desired_az_value.contents = desired_az_value;
  b_desired_theta_value.contents = desired_theta_value;
  b_desired_phi_value.contents = desired_phi_value;
  b_desired_ailerons_value.contents = desired_ailerons_value;
  if (b_V.contents > aoa_protection_speed) {
    max_tilt_value_approach =
        (max_alpha + b_flight_path_angle.contents) * 180.0 / 3.1415926535897931;
    max_theta_protection = fmin(max_theta, max_tilt_value_approach);
    max_tilt_value_approach =
        (min_alpha + b_flight_path_angle.contents) * 180.0 / 3.1415926535897931;
    min_theta_protection = fmax(min_theta, max_tilt_value_approach);
  } else {
    max_theta_protection = max_theta;
    min_theta_protection = min_theta;
  }
  if (b_desired_motor_value.contents < min_omega) {
    double b_Omega_1[4];
    b_Omega_1[0] = Omega_1;
    b_Omega_1[1] = Omega_2;
    b_Omega_1[2] = Omega_3;
    b_Omega_1[3] = Omega_4;
    b_desired_motor_value.contents = mean(b_Omega_1);
  }
  gain_motor.contents = max_omega / 2.0;
  gain_el.contents = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az.contents = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
  gain_theta.contents = (max_theta_protection - min_theta_protection) *
                        3.1415926535897931 / 180.0 / 2.0;
  gain_phi.contents = max_phi * 3.1415926535897931 / 180.0;
  gain_ailerons.contents = (max_delta_ailerons - min_delta_ailerons) *
                           3.1415926535897931 / 180.0 / 2.0;
  u_max[0] = Omega_1;
  u_max[1] = Omega_2;
  u_max[2] = Omega_3;
  u_max[3] = Omega_4;
  u_max[4] = b_1;
  u_max[5] = b_2;
  u_max[6] = b_3;
  u_max[7] = b_4;
  u_max[8] = g_1;
  u_max[9] = g_2;
  u_max[10] = g_3;
  u_max[11] = g_4;
  u_max[12] = b_Theta.contents;
  u_max[13] = b_Phi.contents;
  u_max[14] = delta_ailerons;
  memcpy(&actual_u.contents[0], &u_max[0], 15U * sizeof(double));
  /* Build the max and minimum actuator array: */
  if (m_failure_ID == 3.0) {
    /*  u_max = [max_omega, max_omega, 155, max_omega, max_b, max_b, 1, max_b,
     * max_g, max_g, 1, max_g, max_theta_protection, max_phi,
     * max_delta_ailerons]; */
    /*  u_min = [min_omega, min_omega, min_omega, min_omega, min_b, min_b, -1,
     * min_b, min_g, min_g, -1, min_g, deg2rad(-2), deg2rad(-3),
     * min_delta_ailerons]; */
    u_max[0] = max_omega;
    u_max[1] = max_omega;
    u_max[2] = 155.0;
    u_max[3] = max_omega;
    u_max[4] = max_b;
    u_max[5] = max_b;
    u_max[6] = 1.0;
    u_max[7] = max_b;
    u_max[8] = max_g;
    u_max[9] = max_g;
    u_max[10] = 1.0;
    u_max[11] = max_g;
    u_max[12] = max_theta_protection;
    u_max[13] = max_phi;
    u_max[14] = max_delta_ailerons;
    u_min[0] = min_omega;
    u_min[1] = min_omega;
    u_min[2] = min_omega;
    u_min[3] = min_omega;
    u_min[4] = min_b;
    u_min[5] = min_b;
    u_min[6] = -1.0;
    u_min[7] = min_b;
    u_min[8] = min_g;
    u_min[9] = min_g;
    u_min[10] = -1.0;
    u_min[11] = min_g;
    u_min[12] = min_theta_protection;
    u_min[13] = -max_phi;
    u_min[14] = min_delta_ailerons;
    /*  elseif (m_failure_ID==4) */
    /*  u_max = [max_omega, max_omega, max_omega, 155, max_b, max_b, max_b, 0,
     * max_g, max_g, max_g, 0, max_theta_protection, max_phi,
     * max_delta_ailerons]; */
    /*  u_min = [min_omega, min_omega, min_omega, min_omega, min_b, min_b,
     * min_b, 0, min_g, min_g, min_g, 0, min_theta_protection, -max_phi,
     * min_delta_ailerons]; */
  } else {
    u_max[0] = max_omega;
    u_max[1] = max_omega;
    u_max[2] = max_omega;
    u_max[3] = max_omega;
    u_max[4] = max_b;
    u_max[5] = max_b;
    u_max[6] = max_b;
    u_max[7] = max_b;
    u_max[8] = max_g;
    u_max[9] = max_g;
    u_max[10] = max_g;
    u_max[11] = max_g;
    u_max[12] = max_theta_protection;
    u_max[13] = max_phi;
    u_max[14] = max_delta_ailerons;
    u_min[0] = min_omega;
    u_min[1] = min_omega;
    u_min[2] = min_omega;
    u_min[3] = min_omega;
    u_min[4] = min_b;
    u_min[5] = min_b;
    u_min[6] = min_b;
    u_min[7] = min_b;
    u_min[8] = min_g;
    u_min[9] = min_g;
    u_min[10] = min_g;
    u_min[11] = min_g;
    u_min[12] = min_theta_protection;
    u_min[13] = -max_phi;
    u_min[14] = min_delta_ailerons;
  }
  if (approach_mode != 0.0) {
    double b_max_tilt_value_approach[2];
    b_max_tilt_value_approach[0] = 0.0;
    b_max_tilt_value_approach[1] =
        k_alt_tilt_constraint * lidar_alt_corrected -
        min_alt_tilt_constraint * k_alt_tilt_constraint;
    max_tilt_value_approach = maximum(b_max_tilt_value_approach);
    /* Elevation angle */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_b;
    b_max_approach = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_b;
    b_min_approach = maximum(b_max_tilt_value_approach);
    /* Azimuth angle */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_g;
    g_max_approach = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_g;
    g_min_approach = maximum(b_max_tilt_value_approach);
    u_max[4] = b_max_approach;
    u_min[4] = b_min_approach;
    u_max[8] = g_max_approach;
    u_min[8] = g_min_approach;
    u_max[5] = b_max_approach;
    u_min[5] = b_min_approach;
    u_max[9] = g_max_approach;
    u_min[9] = g_min_approach;
    u_max[6] = b_max_approach;
    u_min[6] = b_min_approach;
    u_max[10] = g_max_approach;
    u_min[10] = g_min_approach;
    u_max[7] = b_max_approach;
    u_min[7] = b_min_approach;
    u_max[11] = g_max_approach;
    u_min[11] = g_min_approach;
    /* Pitch angle  */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_theta_protection;
    u_max[12] = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_theta_protection;
    u_min[12] = maximum(b_max_tilt_value_approach);
    /* Roll angle  */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_phi;
    u_max[13] = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = -max_phi;
    u_min[13] = maximum(b_max_tilt_value_approach);
  }
  for (i = 0; i < 11; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }
  memcpy(&u_max_scaled[0], &u_max[0], 15U * sizeof(double));
  memcpy(&u_min_scaled[0], &u_min[0], 15U * sizeof(double));
  u_max_scaled[0] = u_max[0] / gain_motor.contents;
  u_max_scaled[1] = u_max[1] / gain_motor.contents;
  u_max_scaled[2] = u_max[2] / gain_motor.contents;
  u_max_scaled[3] = u_max[3] / gain_motor.contents;
  u_min_scaled[0] = u_min[0] / gain_motor.contents;
  u_min_scaled[1] = u_min[1] / gain_motor.contents;
  u_min_scaled[2] = u_min[2] / gain_motor.contents;
  u_min_scaled[3] = u_min[3] / gain_motor.contents;
  u_max_scaled[4] /= gain_el.contents;
  u_max_scaled[5] /= gain_el.contents;
  u_max_scaled[6] /= gain_el.contents;
  u_max_scaled[7] /= gain_el.contents;
  u_min_scaled[4] /= gain_el.contents;
  u_min_scaled[5] /= gain_el.contents;
  u_min_scaled[6] /= gain_el.contents;
  u_min_scaled[7] /= gain_el.contents;
  u_max_scaled[8] /= gain_az.contents;
  u_max_scaled[9] /= gain_az.contents;
  u_max_scaled[10] /= gain_az.contents;
  u_max_scaled[11] /= gain_az.contents;
  u_min_scaled[8] /= gain_az.contents;
  u_min_scaled[9] /= gain_az.contents;
  u_min_scaled[10] /= gain_az.contents;
  u_min_scaled[11] /= gain_az.contents;
  u_max_scaled[12] /= gain_theta.contents;
  u_min_scaled[12] /= gain_theta.contents;
  u_max_scaled[13] /= gain_phi.contents;
  u_min_scaled[13] /= gain_phi.contents;
  u_max_scaled[14] /= gain_ailerons.contents;
  u_min_scaled[14] /= gain_ailerons.contents;
  memcpy(&u_max[0], &actual_u.contents[0], 15U * sizeof(double));
  u_max[0] /= gain_motor.contents;
  u_max[1] /= gain_motor.contents;
  u_max[2] /= gain_motor.contents;
  u_max[3] /= gain_motor.contents;
  u_max[4] /= gain_el.contents;
  u_max[5] /= gain_el.contents;
  u_max[6] /= gain_el.contents;
  u_max[7] /= gain_el.contents;
  u_max[8] /= gain_az.contents;
  u_max[9] /= gain_az.contents;
  u_max[10] /= gain_az.contents;
  u_max[11] /= gain_az.contents;
  u_max[12] /= gain_theta.contents;
  u_max[13] /= gain_phi.contents;
  u_max[14] /= gain_ailerons.contents;
  /*  Apply Nonlinear optimization algorithm: */
  /*  current_accelerations =
   * compute_acc_cascaded_nonlinear_CA_w_ailerons_internal(actual_u,p,q,r,K_p_T,K_p_M,m,I_xx,I_yy,I_zz,l_1,l_2,l_3,l_4,l_z,Cl_alpha,
   * Cd_zero, K_Cd, Cm_alpha, Cm_zero, CL_aileron,rho, V, S, wing_chord,
   * flight_path_angle, Beta); */
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = dv[i] + current_accelerations[i];
  }
  /* Pseudo-control hedging:  */
  if (b_V.contents > transition_speed) {
    /*  CL_max =  Cl_alpha * max_alpha;   */
    b_max_approach = b_Cl_alpha.contents * min_alpha;
    /*  max_vert_force_fwd = CL_max * 0.5 * rho * S * V^2 *
     * cos(max_theta_protection*pi/180); */
    max_tilt_value_approach = b_V.contents;
    b_max_approach = b_max_approach * 0.5 * b_rho.contents * b_S.contents *
                     (max_tilt_value_approach * max_tilt_value_approach) *
                     cos(min_theta_protection * 3.1415926535897931 / 180.0);
    b_max_approach = 9.81 - b_max_approach / b_m.contents;
    /*      min_vert_acc_fwd = 9.81 - max_vert_force_fwd / m;  */
    max_tilt_value_approach = 0.0;
    if (dv_global.contents[2] >= b_max_approach) {
      max_tilt_value_approach = dv_global.contents[2] - b_max_approach;
      /*      elseif(dv_global(3) <= min_vert_acc_fwd) */
      /*          vert_acc_cut = dv_global(3) - min_vert_acc_fwd;  */
    }
    dv_global.contents[2] -= max_tilt_value_approach;
  }
  /* Compute weights for individual actuators and make sure they are always
   * positive */
  /*  Individual weights were originally intended to be used to deter use of */
  /*  specific actuators, current implementation instead limits available */
  /*  optimisation space, but it remains a good option to individually tune the
   */
  /*  acutators. */
  max_tilt_value_approach =
      W_act_motor_const + W_act_motor_speed * b_V.contents;
  W_act_motor.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach =
      W_act_tilt_el_const + W_act_tilt_el_speed * b_V.contents;
  W_act_tilt_el.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach =
      W_act_tilt_az_const + W_act_tilt_az_speed * b_V.contents;
  W_act_tilt_az.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach =
      W_act_theta_const + W_act_theta_speed * b_V.contents;
  W_act_theta.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_phi_const + W_act_phi_speed * b_V.contents;
  W_act_phi.contents = fmax(0.0, max_tilt_value_approach);
  /*  W_act_motor4 = W_MOTOR_FAILURE_WEIGHT; */
  /*  W_act_theta = max(0,W_act_theta_const + W_act_theta_speed*V); */
  /*  W_act_phi = max(0,W_act_phi_const + W_act_phi_speed*V); */
  max_tilt_value_approach =
      W_act_ailerons_const + W_act_ailerons_speed * b_V.contents;
  W_act_ailerons.contents = fmax(0.0, max_tilt_value_approach);
  /* Default values for the optimizer: */
  /*  max_iterations = 120;  */
  /*  max_evaluations = 120;  */
  /*  This yielded similar, but not 100% matching results to the first
   * testflight */
  /*  OPTIONS2 =
   * optimoptions('fmincon','Algorithm','sqp','StepTolerance',step_tolerance_value,'MaxFunctionEvaluations',60,
   * ... */
  /*      'OptimalityTolerance',optimality_tolerance_value,'MaxIterations',60,'SpecifyObjectiveGradient',true,'UseParallel',true);
   */
  /*   */
  /*  OPTIONS1 =
   * optimoptions('fmincon','Algorithm','sqp','StepTolerance',step_tolerance_value,'MaxFunctionEvaluations',40,
   * ... */
  /*      'OptimalityTolerance',optimality_tolerance_value,'MaxIterations',40,'SpecifyObjectiveGradient',true,'UseParallel',true);
   */
  /*  There is still a difference in the Theta and motor response */
  /*  Below seems even more promising, also makes sense based on the premise
   * that the first is more complex and thus runs less iterations */
  /*  OPTIONS2 =
   * optimoptions('fmincon','Algorithm','sqp','StepTolerance',step_tolerance_value,'MaxFunctionEvaluations',90,
   * ... */
  /*      'OptimalityTolerance',optimality_tolerance_value,'MaxIterations',40,'SpecifyObjectiveGradient',true,'UseParallel',true);
   */
  /*   */
  /*  OPTIONS1 =
   * optimoptions('fmincon','Algorithm','sqp','StepTolerance',step_tolerance_value,'MaxFunctionEvaluations',90,
   * ... */
  /*      'OptimalityTolerance',optimality_tolerance_value,'MaxIterations',40,'SpecifyObjectiveGradient',true,'UseParallel',true);
   */
  /* Config as intended to be used for the testflight 80 80 */
  /*  OPTIONS2 =
   * optimoptions('fmincon','Algorithm','sqp','StepTolerance',step_tolerance_value,'MaxFunctionEvaluations',80,
   * ... */
  /*      'OptimalityTolerance',optimality_tolerance_value,'MaxIterations',80,'SpecifyObjectiveGradient',true,'UseParallel',true);
   */
  /*   */
  /*  OPTIONS1 =
   * optimoptions('fmincon','Algorithm','sqp','StepTolerance',step_tolerance_value,'MaxFunctionEvaluations',80,
   * ... */
  /*      'OptimalityTolerance',optimality_tolerance_value,'MaxIterations',80,'SpecifyObjectiveGradient',true,'UseParallel',true);
   */
  tic();
  /* First optimization run to identify the pitch and roll angles: */
  memcpy(&u_min[0], &u_max[0], 15U * sizeof(double));
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.rho = &b_rho;
  expl_temp.r = &b_r;
  expl_temp.q = &b_q;
  expl_temp.actual_u = &actual_u;
  expl_temp.p = &b_p;
  expl_temp.m = &b_m;
  expl_temp.l_z = &b_l_z;
  expl_temp.l_4 = &b_l_4;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_2 = &b_l_2;
  expl_temp.l_1 = &b_l_1;
  expl_temp.gamma_quadratic_du2 = &b_gamma_quadratic_du2;
  expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  expl_temp.gain_ailerons = &gain_ailerons;
  expl_temp.gain_motor = &gain_motor;
  expl_temp.gain_theta = &gain_theta;
  expl_temp.gain_phi = &gain_phi;
  expl_temp.gain_az = &gain_az;
  expl_temp.gain_el = &gain_el;
  expl_temp.flight_path_angle = &b_flight_path_angle;
  expl_temp.desired_ailerons_value = &b_desired_ailerons_value;
  expl_temp.desired_motor_value = &b_desired_motor_value;
  expl_temp.desired_theta_value = &b_desired_theta_value;
  expl_temp.desired_phi_value = &b_desired_phi_value;
  expl_temp.desired_az_value = &b_desired_az_value;
  expl_temp.desired_el_value = &b_desired_el_value;
  expl_temp.W_act_ailerons_du = &b_W_act_ailerons_du;
  expl_temp.W_act_tilt_az_du = &b_W_act_tilt_az_du;
  expl_temp.W_act_tilt_el_du = &b_W_act_tilt_el_du;
  expl_temp.W_act_motor_du = &b_W_act_motor_du;
  expl_temp.W_act_ailerons = &W_act_ailerons;
  expl_temp.W_act_theta_du = &b_W_act_theta_du;
  expl_temp.W_act_tilt_az = &W_act_tilt_az;
  expl_temp.W_act_tilt_el = &W_act_tilt_el;
  expl_temp.W_dv_6 = &b_W_dv_6;
  expl_temp.W_dv_5 = &b_W_dv_5;
  expl_temp.W_dv_4 = &b_W_dv_4;
  expl_temp.W_dv_3 = &b_W_dv_3;
  expl_temp.W_dv_2 = &b_W_dv_2;
  expl_temp.W_dv_1 = &b_W_dv_1;
  expl_temp.W_act_phi_du = &b_W_act_phi_du;
  expl_temp.W_act_motor = &W_act_motor;
  expl_temp.W_act_theta = &W_act_theta;
  expl_temp.W_act_phi = &W_act_phi;
  expl_temp.V = &b_V;
  expl_temp.S = &b_S;
  expl_temp.K_p_T = &b_K_p_T;
  expl_temp.K_p_M = &b_K_p_M;
  expl_temp.K_Cd = &b_K_Cd;
  expl_temp.I_zz = &b_I_zz;
  expl_temp.I_yy = &b_I_yy;
  expl_temp.I_xx = &b_I_xx;
  expl_temp.Cm_alpha = &b_Cm_alpha;
  expl_temp.Cm_zero = &b_Cm_zero;
  expl_temp.Cl_alpha = &b_Cl_alpha;
  expl_temp.Cd_zero = &b_Cd_zero;
  expl_temp.CL_aileron = &b_CL_aileron;
  expl_temp.Beta = &b_Beta;
  expl_temp.dv_global = &dv_global;
  b_expl_temp = expl_temp;
  max_tilt_value_approach = fmincon(
      &b_expl_temp, u_min, u_min_scaled, u_max_scaled, &b_max_approach,
      &output_iterations, &output_funcCount, c_expl_temp, &b_min_approach,
      &g_max_approach, &g_min_approach, &max_theta_protection);
  /*   */
  /*  if (m_failure_ID==3) */
  /*  des_theta_first_iteration = deg2rad(20); */
  /*  des_phi_first_iteration   = deg2rad(70); */
  /*  else */
  des_theta_first_iteration = u_min[12] * gain_theta.contents;
  des_phi_first_iteration = u_min[13] * gain_phi.contents;
  /*  end */
  /*  if (m_failure_ID==3) */
  /*      des_theta_first_iteration = deg2rad(14); */
  /*      des_phi_first_iteration = deg2rad(73); */
  /*  else */
  /*      des_theta_first_iteration = deg2rad(0); */
  /*      des_phi_first_iteration = deg2rad(0); */
  /*  end */
  /* with the desired theta and phi, let's compute the associated angular */
  /* accelerations through the EC gains and feedbacks:  */
  max_tilt_value_approach = b_Theta.contents;
  max_tilt_value_approach = sin(max_tilt_value_approach);
  b_max_approach = b_Phi.contents;
  b_max_approach = cos(b_max_approach);
  b_min_approach = b_Phi.contents;
  b_min_approach = sin(b_min_approach);
  g_max_approach = b_Theta.contents;
  g_max_approach = cos(g_max_approach);
  g_min_approach = b_Phi.contents;
  g_min_approach = sin(g_min_approach);
  max_theta_protection = b_Phi.contents;
  max_theta_protection = cos(max_theta_protection);
  min_theta_protection = b_Theta.contents;
  min_theta_protection = cos(min_theta_protection);
  b_dv[0] = 1.0;
  b_dv[3] = 0.0;
  b_dv[6] = -max_tilt_value_approach;
  b_dv[1] = 0.0;
  b_dv[4] = b_max_approach;
  b_dv[7] = b_min_approach * g_max_approach;
  b_dv[2] = 0.0;
  b_dv[5] = -g_min_approach;
  b_dv[8] = max_theta_protection * min_theta_protection;
  max_tilt_value_approach = (des_phi_first_iteration - phi_current) * phi_gain;
  b_max_approach = (des_theta_first_iteration - theta_current) * theta_gain;
  for (i = 0; i < 3; i++) {
    des_body_rates[i] =
        (b_dv[i] * max_tilt_value_approach + b_dv[i + 3] * b_max_approach) +
        b_dv[i + 6] * des_psi_dot;
  }
  double a__4[169];
  max_tilt_value_approach = dv_global.contents[3];
  b_max_approach = dv_global.contents[4];
  b_min_approach = dv_global.contents[5];
  dv_global.contents[3] =
      max_tilt_value_approach +
      ((des_body_rates[0] - p_body_current) * p_body_gain - p_dot_current);
  dv_global.contents[4] =
      b_max_approach +
      ((des_body_rates[1] - q_body_current) * q_body_gain - q_dot_current);
  dv_global.contents[5] =
      b_min_approach +
      ((des_body_rates[2] - r_body_current) * r_body_gain - r_dot_current);
  /* before entering the second optimization run, let's remove the extra inputs:
   */
  /* Second optimization run to identify the actuator commands: */
  memcpy(&u_out[0], &u_max[0], 12U * sizeof(double));
  u_out[12] = u_max[14];
  d_expl_temp.wing_chord = &b_wing_chord;
  d_expl_temp.rho = &b_rho;
  d_expl_temp.r = &b_r;
  d_expl_temp.q = &b_q;
  d_expl_temp.actual_u = &actual_u;
  d_expl_temp.p = &b_p;
  d_expl_temp.m = &b_m;
  d_expl_temp.l_z = &b_l_z;
  d_expl_temp.l_4 = &b_l_4;
  d_expl_temp.l_3 = &b_l_3;
  d_expl_temp.l_2 = &b_l_2;
  d_expl_temp.l_1 = &b_l_1;
  d_expl_temp.gamma_quadratic_du2 = &b_gamma_quadratic_du2;
  d_expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  d_expl_temp.gain_ailerons = &gain_ailerons;
  d_expl_temp.gain_motor = &gain_motor;
  d_expl_temp.gain_az = &gain_az;
  d_expl_temp.gain_el = &gain_el;
  d_expl_temp.flight_path_angle = &b_flight_path_angle;
  d_expl_temp.desired_ailerons_value = &b_desired_ailerons_value;
  d_expl_temp.desired_motor_value = &b_desired_motor_value;
  d_expl_temp.desired_az_value = &b_desired_az_value;
  d_expl_temp.desired_el_value = &b_desired_el_value;
  d_expl_temp.W_act_ailerons_du = &b_W_act_ailerons_du;
  d_expl_temp.W_act_tilt_az_du = &b_W_act_tilt_az_du;
  d_expl_temp.W_act_tilt_el_du = &b_W_act_tilt_el_du;
  d_expl_temp.W_act_motor_du = &b_W_act_motor_du;
  d_expl_temp.W_act_ailerons = &W_act_ailerons;
  d_expl_temp.W_act_tilt_az = &W_act_tilt_az;
  d_expl_temp.W_act_tilt_el = &W_act_tilt_el;
  d_expl_temp.W_dv_6 = &b_W_dv_6;
  d_expl_temp.W_dv_5 = &b_W_dv_5;
  d_expl_temp.W_dv_4 = &b_W_dv_4;
  d_expl_temp.W_dv_3 = &b_W_dv_3;
  d_expl_temp.W_dv_2 = &b_W_dv_2;
  d_expl_temp.W_dv_1 = &b_W_dv_1;
  d_expl_temp.W_act_motor = &W_act_motor;
  d_expl_temp.V = &b_V;
  d_expl_temp.Theta = &b_Theta;
  d_expl_temp.S = &b_S;
  d_expl_temp.Phi = &b_Phi;
  d_expl_temp.K_p_T = &b_K_p_T;
  d_expl_temp.K_p_M = &b_K_p_M;
  d_expl_temp.K_Cd = &b_K_Cd;
  d_expl_temp.I_zz = &b_I_zz;
  d_expl_temp.I_yy = &b_I_yy;
  d_expl_temp.I_xx = &b_I_xx;
  d_expl_temp.Cm_alpha = &b_Cm_alpha;
  d_expl_temp.Cm_zero = &b_Cm_zero;
  d_expl_temp.Cl_alpha = &b_Cl_alpha;
  d_expl_temp.Cd_zero = &b_Cd_zero;
  d_expl_temp.CL_aileron = &b_CL_aileron;
  d_expl_temp.Beta = &b_Beta;
  d_expl_temp.dv_global = &dv_global;
  b_u_min_scaled[12] = u_min_scaled[14];
  memcpy(&b_u_min_scaled[0], &u_min_scaled[0], 12U * sizeof(double));
  memcpy(&b_u_max_scaled[0], &u_max_scaled[0], 12U * sizeof(double));
  b_u_max_scaled[12] = u_max_scaled[14];
  e_expl_temp = d_expl_temp;
  *cost = b_fmincon(&e_expl_temp, u_out, b_u_min_scaled, b_u_max_scaled, &d,
                    &b_max_approach, &max_tilt_value_approach, c_expl_temp,
                    &b_min_approach, &g_max_approach, &g_min_approach,
                    &max_theta_protection, &a__3, gradient, a__4);
  *exitflag = d;
  *elapsed_time = toc();
  *N_evaluation = output_funcCount + max_tilt_value_approach;
  *N_iterations = output_iterations + b_max_approach;
  max_tilt_value_approach = gain_motor.contents;
  u_out_local[0] = u_out[0] * max_tilt_value_approach;
  u_out_local[1] = u_out[1] * max_tilt_value_approach;
  u_out_local[2] = u_out[2] * max_tilt_value_approach;
  u_out_local[3] = u_out[3] * max_tilt_value_approach;
  max_tilt_value_approach = gain_el.contents;
  u_out_local[4] = u_out[4] * max_tilt_value_approach;
  u_out_local[5] = u_out[5] * max_tilt_value_approach;
  u_out_local[6] = u_out[6] * max_tilt_value_approach;
  u_out_local[7] = u_out[7] * max_tilt_value_approach;
  max_tilt_value_approach = gain_az.contents;
  u_out_local[8] = u_out[8] * max_tilt_value_approach;
  u_out_local[9] = u_out[9] * max_tilt_value_approach;
  u_out_local[10] = u_out[10] * max_tilt_value_approach;
  u_out_local[11] = u_out[11] * max_tilt_value_approach;
  u_out_local[12] = b_Theta.contents;
  u_out_local[13] = b_Phi.contents;
  u_out_local[14] = u_out[12] * gain_ailerons.contents;
  c_compute_acc_cascaded_nonlinea(
      u_out_local, b_p.contents, b_q.contents, b_r.contents, b_K_p_T.contents,
      b_K_p_M.contents, b_m.contents, b_I_xx.contents, b_I_yy.contents,
      b_I_zz.contents, b_l_1.contents, b_l_2.contents, b_l_3.contents,
      b_l_4.contents, b_l_z.contents, b_Cl_alpha.contents, b_Cd_zero.contents,
      b_K_Cd.contents, b_Cm_alpha.contents, b_Cm_zero.contents,
      b_CL_aileron.contents, b_rho.contents, b_V.contents, b_S.contents,
      b_wing_chord.contents, b_flight_path_angle.contents, b_Beta.contents,
      final_accelerations);
  for (i = 0; i < 6; i++) {
    residuals[i] = dv_global.contents[i] - final_accelerations[i];
  }
  u_out_local[12] = des_theta_first_iteration;
  u_out_local[13] = des_phi_first_iteration;
  /*  Print infos */
  if (verbose != 0.0) {
    printf("\n Solution: \n");
    fflush(stdout);
    printf(" Motors [rad/s] =  ");
    fflush(stdout);
    printf(" %f ", u_out_local[0]);
    fflush(stdout);
    printf(" %f ", u_out_local[1]);
    fflush(stdout);
    printf(" %f ", u_out_local[2]);
    fflush(stdout);
    printf(" %f ", u_out_local[3]);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Elevator angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out_local[4] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out_local[5] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out_local[6] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out_local[7] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Azimuth angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out_local[8] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out_local[9] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out_local[10] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out_local[11] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Theta [deg] =  ");
    fflush(stdout);
    printf(" %f ", des_theta_first_iteration * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Phi [deg] =  ");
    fflush(stdout);
    printf(" %f ", des_phi_first_iteration * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Ailerons deflection [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out_local[14] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time = %f \n", *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations / evaluations = %f ", *N_iterations);
    fflush(stdout);
    printf("/ %f \n", *N_evaluation);
    fflush(stdout);
    printf("\n Exit flag optimizer = %f \n", *exitflag);
    fflush(stdout);
    printf("\n Modeled accelerations =   ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", current_accelerations[i]);
      fflush(stdout);
    }
    printf("\n");
    fflush(stdout);
    printf("\n desired acc increment =    ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", dv[i]);
      fflush(stdout);
    }
    printf("\n");
    fflush(stdout);
    printf("\n Requested accelerations =  ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      max_tilt_value_approach = dv_global.contents[i];
      printf(" %f ", max_tilt_value_approach);
      fflush(stdout);
    }
    printf("\n");
    fflush(stdout);
    printf("\n Achieved accelerations =   ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", final_accelerations[i]);
      fflush(stdout);
    }
    printf("\n");
    fflush(stdout);
    printf("\n Acc residuals / norm  =    ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", residuals[i]);
      fflush(stdout);
    }
    printf(" / ");
    fflush(stdout);
    printf(" %f \n", b_norm(residuals));
    fflush(stdout);
  }
}

/*
 * File trailer for Cascaded_nonlinear_TestFlight.c
 *
 * [EOF]
 */
