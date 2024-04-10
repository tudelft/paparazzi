/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_CA_w_ail_approach_ext_acc.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

/* Include Files */
#include "Nonlinear_CA_w_ail_approach_ext_acc.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc_data.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc_initialize.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc_internal_types.h"
#include "Nonlinear_CA_w_ail_approach_ext_acc_types.h"
#include "compute_acc_nonlinear_control_rf_w_ailerons.h"
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
 * Testing parameters:
 *  (0.5e-5, 2e-7, 2.4, 0.15, 0.15, 0.2, 0.25, 0.25, 0.35, 0.35, 0, ...
 *  0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...
 *  1, 0, 0, 0, ...
 *  0, 1.5, 100, -15, 100, -15, 0.5, 0, ...
 *  0.01, 0.01, 0.05, 0.1, 0.1, 0.1, ...
 *  1000, 100, 25, -130, 45, -45, 80, 15, 40, 25, -25, [0 0 -5 0 0 0]', 0, 0, 0,
 * 0.1, 3, 0.3, 0.15,... -0.1, 0.1, 1.15, 0, 0.3, 0.3, 0, 15, -5, 0, 1e-5, ...
 *  100, 0, 0, 0, 0, 0, 1, 7)
 *  or
 *  (0.5e-5, 2e-7, 2.4, 0.15, 0.15, 0.2, 0.25, 0.25, 0.35, 0.35, 0, 0, 0, 100,
 * 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1.5, 100, -15, 100,
 * -15, 0.5, 0, 0.01, 0.01, 0.05, 0.1, 0.1, 0.1, 1000, 100, 25, 130, 45, -45,
 * 80, 15, 40, 25, -25, [0 0 -5 0 0 0]', 0, 0, 0, 0.1, 3, 0.3, 0.15, -0.1,
 * 0.1, 1.15, 0, 0.3, 0.3, 0, 15, -5, 0, 1e-5, 100, 0, 0, 0, 0, 0, 1, 7)
 *
 * Arguments    : double K_p_T
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
 *                const double dv[6]
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
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double max_alpha
 *                double min_alpha
 *                double Beta
 *                double gamma_quadratic_du
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
 *                const double current_accelerations[6]
 *                double u_out[15]
 *                double residuals[6]
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluation
 *                double *exitflag
 * Return Type  : void
 */
void Nonlinear_CA_w_ail_approach_ext_acc(
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
    double aoa_protection_speed, double transition_speed,
    const double current_accelerations[6], double u_out[15],
    double residuals[6], double *elapsed_time, double *N_iterations,
    double *N_evaluation, double *exitflag)
{
  b_captured_var dv_global;
  c_struct_T b_expl_temp;
  c_struct_T expl_temp;
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
  captured_var b_S;
  captured_var b_V;
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
  double u_max[15];
  double u_min[15];
  double final_accelerations[6];
  double b_max_approach;
  double b_max_tilt_value_approach;
  double b_min_approach;
  double g_max_approach;
  double g_min_approach;
  double max_theta_protection;
  double min_theta_protection;
  int i;
  char c_expl_temp[3];
  if (!isInitialized_Nonlinear_CA_w_ail_approach_ext_acc) {
    Nonlinear_CA_w_ail_approach_ext_acc_initialize();
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
  b_S.contents = S;
  b_wing_chord.contents = wing_chord;
  b_flight_path_angle.contents = flight_path_angle;
  b_Beta.contents = Beta;
  b_gamma_quadratic_du.contents = gamma_quadratic_du;
  b_desired_motor_value.contents = desired_motor_value;
  b_desired_el_value.contents = desired_el_value;
  b_desired_az_value.contents = desired_az_value;
  b_desired_theta_value.contents = desired_theta_value;
  b_desired_phi_value.contents = desired_phi_value;
  b_desired_ailerons_value.contents = desired_ailerons_value;
  /*  Create variables necessary for the optimization */
  if (b_V.contents > aoa_protection_speed) {
    b_min_approach =
        (max_alpha + b_flight_path_angle.contents) * 180.0 / 3.1415926535897931;
    max_theta_protection = fmin(max_theta, b_min_approach);
    b_min_approach =
        (min_alpha + b_flight_path_angle.contents) * 180.0 / 3.1415926535897931;
    min_theta_protection = fmax(min_theta, b_min_approach);
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
  /* Build the max and minimum actuator array: */
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
  if (approach_mode != 0.0) {
    double max_tilt_value_approach[2];
    max_tilt_value_approach[0] = 0.0;
    max_tilt_value_approach[1] =
        k_alt_tilt_constraint * lidar_alt_corrected -
        min_alt_tilt_constraint * k_alt_tilt_constraint;
    b_max_tilt_value_approach = maximum(max_tilt_value_approach);
    /* Elevation angle */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_b;
    b_max_approach = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_b;
    b_min_approach = maximum(max_tilt_value_approach);
    /* Azimuth angle */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_g;
    g_max_approach = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_g;
    g_min_approach = maximum(max_tilt_value_approach);
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
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_theta_protection;
    u_max[12] = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_theta_protection;
    u_min[12] = maximum(max_tilt_value_approach);
    /* Roll angle  */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_phi;
    u_max[13] = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = -max_phi;
    u_min[13] = maximum(max_tilt_value_approach);
  }
  for (i = 0; i < 11; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }
  u_max[12] /= gain_theta.contents;
  u_min[12] /= gain_theta.contents;
  u_max[13] /= gain_phi.contents;
  u_min[13] /= gain_phi.contents;
  u_max[14] /= gain_ailerons.contents;
  u_min[14] /= gain_ailerons.contents;
  u_max[0] /= gain_motor.contents;
  u_min[0] /= gain_motor.contents;
  u_max[4] /= gain_el.contents;
  u_min[4] /= gain_el.contents;
  u_max[8] /= gain_az.contents;
  u_min[8] /= gain_az.contents;
  u_out[0] = Omega_1 / gain_motor.contents;
  u_out[4] = b_1 / gain_el.contents;
  u_out[8] = g_1 / gain_az.contents;
  u_max[1] /= gain_motor.contents;
  u_min[1] /= gain_motor.contents;
  u_max[5] /= gain_el.contents;
  u_min[5] /= gain_el.contents;
  u_max[9] /= gain_az.contents;
  u_min[9] /= gain_az.contents;
  u_out[1] = Omega_2 / gain_motor.contents;
  u_out[5] = b_2 / gain_el.contents;
  u_out[9] = g_2 / gain_az.contents;
  u_max[2] /= gain_motor.contents;
  u_min[2] /= gain_motor.contents;
  u_max[6] /= gain_el.contents;
  u_min[6] /= gain_el.contents;
  u_max[10] /= gain_az.contents;
  u_min[10] /= gain_az.contents;
  u_out[2] = Omega_3 / gain_motor.contents;
  u_out[6] = b_3 / gain_el.contents;
  u_out[10] = g_3 / gain_az.contents;
  u_max[3] /= gain_motor.contents;
  u_min[3] /= gain_motor.contents;
  u_max[7] /= gain_el.contents;
  u_min[7] /= gain_el.contents;
  u_max[11] /= gain_az.contents;
  u_min[11] /= gain_az.contents;
  u_out[3] = Omega_4 / gain_motor.contents;
  u_out[7] = b_4 / gain_el.contents;
  u_out[11] = g_4 / gain_az.contents;
  u_out[12] = Theta / gain_theta.contents;
  u_out[13] = Phi / gain_phi.contents;
  u_out[14] = delta_ailerons / gain_ailerons.contents;
  /*  Apply Nonlinear optimization algorithm: */
  /* current_accelerations =
   * compute_acc_nonlinear_control_rf_w_ailerons(actual_u,p,q,r,K_p_T,K_p_M,m,I_xx,I_yy,I_zz,l_1,l_2,l_3,l_4,l_z,Cl_alpha,
   * Cd_zero, K_Cd, Cm_alpha, Cm_zero, CL_aileron,rho, V, S, wing_chord,
   * flight_path_angle, Beta); */
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = dv[i] + current_accelerations[i];
  }
  /* Pseudo-control hedging:  */
  if (b_V.contents > transition_speed) {
    b_max_approach = b_Cl_alpha.contents * min_alpha;
    b_max_tilt_value_approach = b_V.contents;
    b_max_approach = b_max_approach * 0.5 * b_rho.contents * b_S.contents *
                     (b_max_tilt_value_approach * b_max_tilt_value_approach) *
                     cos(min_theta_protection * 3.1415926535897931 / 180.0);
    b_max_approach = 9.81 - b_max_approach / b_m.contents;
    /*      min_vert_acc_fwd = 9.81 - max_vert_force_fwd / m;  */
    b_max_tilt_value_approach = 0.0;
    if (dv_global.contents[2] >= b_max_approach) {
      b_max_tilt_value_approach = dv_global.contents[2] - b_max_approach;
      /*      elseif(dv_global(3) <= min_vert_acc_fwd) */
      /*          vert_acc_cut = dv_global(3) - min_vert_acc_fwd;  */
    }
    dv_global.contents[2] -= b_max_tilt_value_approach;
  }
  /* Compute weights for actuators and make sure they are always positive */
  b_min_approach = W_act_motor_const + W_act_motor_speed * b_V.contents;
  W_act_motor.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_tilt_el_const + W_act_tilt_el_speed * b_V.contents;
  W_act_tilt_el.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_tilt_az_const + W_act_tilt_az_speed * b_V.contents;
  W_act_tilt_az.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_theta_const + W_act_theta_speed * b_V.contents;
  W_act_theta.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_phi_const + W_act_phi_speed * b_V.contents;
  W_act_phi.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_ailerons_const + W_act_ailerons_speed * b_V.contents;
  W_act_ailerons.contents = fmax(0.0, b_min_approach);
  /* Default values for the optimizer: */
  tic();
  expl_temp.desired_ailerons_value = &b_desired_ailerons_value;
  expl_temp.W_act_ailerons = &W_act_ailerons;
  expl_temp.desired_phi_value = &b_desired_phi_value;
  expl_temp.W_act_phi = &W_act_phi;
  expl_temp.desired_theta_value = &b_desired_theta_value;
  expl_temp.W_act_theta = &W_act_theta;
  expl_temp.desired_az_value = &b_desired_az_value;
  expl_temp.W_act_tilt_az = &W_act_tilt_az;
  expl_temp.desired_el_value = &b_desired_el_value;
  expl_temp.W_act_tilt_el = &W_act_tilt_el;
  expl_temp.W_dv_2 = &b_W_dv_2;
  expl_temp.W_dv_1 = &b_W_dv_1;
  expl_temp.W_dv_3 = &b_W_dv_3;
  expl_temp.W_dv_5 = &b_W_dv_5;
  expl_temp.desired_motor_value = &b_desired_motor_value;
  expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  expl_temp.W_act_motor = &W_act_motor;
  expl_temp.W_dv_6 = &b_W_dv_6;
  expl_temp.W_dv_4 = &b_W_dv_4;
  expl_temp.gain_ailerons = &gain_ailerons;
  expl_temp.CL_aileron = &b_CL_aileron;
  expl_temp.l_2 = &b_l_2;
  expl_temp.l_1 = &b_l_1;
  expl_temp.q = &b_q;
  expl_temp.Cm_alpha = &b_Cm_alpha;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_4 = &b_l_4;
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.Cm_zero = &b_Cm_zero;
  expl_temp.K_p_M = &b_K_p_M;
  expl_temp.l_z = &b_l_z;
  expl_temp.I_yy = &b_I_yy;
  expl_temp.I_xx = &b_I_xx;
  expl_temp.r = &b_r;
  expl_temp.p = &b_p;
  expl_temp.I_zz = &b_I_zz;
  expl_temp.m = &b_m;
  expl_temp.gain_az = &gain_az;
  expl_temp.gain_el = &gain_el;
  expl_temp.gain_motor = &gain_motor;
  expl_temp.K_p_T = &b_K_p_T;
  expl_temp.gain_phi = &gain_phi;
  expl_temp.Cd_zero = &b_Cd_zero;
  expl_temp.K_Cd = &b_K_Cd;
  expl_temp.Beta = &b_Beta;
  expl_temp.flight_path_angle = &b_flight_path_angle;
  expl_temp.rho = &b_rho;
  expl_temp.V = &b_V;
  expl_temp.S = &b_S;
  expl_temp.Cl_alpha = &b_Cl_alpha;
  expl_temp.gain_theta = &gain_theta;
  expl_temp.dv_global = &dv_global;
  b_expl_temp = expl_temp;
  fmincon(&b_expl_temp, u_out, u_min, u_max, exitflag, N_iterations,
          N_evaluation, c_expl_temp, &b_max_approach, &b_min_approach,
          &g_max_approach, &g_min_approach);
  *elapsed_time = toc();
  b_min_approach = gain_motor.contents;
  b_max_tilt_value_approach = gain_el.contents;
  b_max_approach = gain_az.contents;
  u_out[0] *= b_min_approach;
  u_out[4] *= b_max_tilt_value_approach;
  u_out[8] *= b_max_approach;
  u_out[1] *= b_min_approach;
  u_out[5] *= b_max_tilt_value_approach;
  u_out[9] *= b_max_approach;
  u_out[2] *= b_min_approach;
  u_out[6] *= b_max_tilt_value_approach;
  u_out[10] *= b_max_approach;
  u_out[3] *= b_min_approach;
  u_out[7] *= b_max_tilt_value_approach;
  u_out[11] *= b_max_approach;
  u_out[12] *= gain_theta.contents;
  u_out[13] *= gain_phi.contents;
  u_out[14] *= gain_ailerons.contents;
  c_compute_acc_nonlinear_control(
      u_out, b_p.contents, b_q.contents, b_r.contents, b_K_p_T.contents,
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
  /*  Print infos */
  if (verbose != 0.0) {
    printf("\n Solution: \n");
    fflush(stdout);
    printf(" Motors [rad/s] =  ");
    fflush(stdout);
    printf(" %f ", u_out[0]);
    fflush(stdout);
    printf(" %f ", u_out[1]);
    fflush(stdout);
    printf(" %f ", u_out[2]);
    fflush(stdout);
    printf(" %f ", u_out[3]);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Elevator angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[4] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[5] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[6] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[7] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Azimuth angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[8] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[9] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[10] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[11] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Theta [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[12] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Phi [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[13] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Ailerons deflection [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[14] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time = %f \n", *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations / evaluations = %f ", *N_iterations);
    fflush(stdout);
    printf("/ %f \n", *N_evaluation);
    fflush(stdout);
    memcpy(&u_max[0], &u_out[0], 15U * sizeof(double));
    u_max[0] = u_out[0] / gain_motor.contents;
    u_max[4] /= gain_el.contents;
    u_max[8] /= gain_az.contents;
    u_max[1] = u_out[1] / gain_motor.contents;
    u_max[5] /= gain_el.contents;
    u_max[9] /= gain_az.contents;
    u_max[2] = u_out[2] / gain_motor.contents;
    u_max[6] /= gain_el.contents;
    u_max[10] /= gain_az.contents;
    u_max[3] = u_out[3] / gain_motor.contents;
    u_max[7] /= gain_el.contents;
    u_max[11] /= gain_az.contents;
    u_max[12] /= gain_theta.contents;
    u_max[13] /= gain_phi.contents;
    u_max[14] /= gain_ailerons.contents;
    printf("\n Solution scaled norm = %f \n", b_norm(u_max));
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
      b_max_tilt_value_approach = dv_global.contents[i];
      printf(" %f ", b_max_tilt_value_approach);
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
    printf(" %f \n", c_norm(residuals));
    fflush(stdout);
  }
}

/*
 * Arguments    : const b_captured_var *dv_global
 *                const captured_var *gain_theta
 *                const captured_var *Cl_alpha
 *                const captured_var *S
 *                const captured_var *V
 *                const captured_var *rho
 *                const captured_var *flight_path_angle
 *                const captured_var *Beta
 *                const captured_var *K_Cd
 *                const captured_var *Cd_zero
 *                const captured_var *gain_phi
 *                const captured_var *K_p_T
 *                const captured_var *gain_motor
 *                const captured_var *gain_el
 *                const captured_var *gain_az
 *                const captured_var *m
 *                const captured_var *I_zz
 *                const captured_var *p
 *                const captured_var *r
 *                const captured_var *I_xx
 *                const captured_var *I_yy
 *                const captured_var *l_z
 *                const captured_var *K_p_M
 *                const captured_var *Cm_zero
 *                const captured_var *wing_chord
 *                const captured_var *l_4
 *                const captured_var *l_3
 *                const captured_var *Cm_alpha
 *                const captured_var *q
 *                const captured_var *l_1
 *                const captured_var *l_2
 *                const captured_var *CL_aileron
 *                const captured_var *gain_ailerons
 *                const captured_var *W_dv_4
 *                const captured_var *W_dv_6
 *                const captured_var *W_act_motor
 *                const captured_var *gamma_quadratic_du
 *                const captured_var *desired_motor_value
 *                const captured_var *W_dv_5
 *                const captured_var *W_dv_3
 *                const captured_var *W_dv_1
 *                const captured_var *W_dv_2
 *                const captured_var *W_act_tilt_el
 *                const captured_var *desired_el_value
 *                const captured_var *W_act_tilt_az
 *                const captured_var *desired_az_value
 *                const captured_var *W_act_theta
 *                const captured_var *desired_theta_value
 *                const captured_var *W_act_phi
 *                const captured_var *desired_phi_value
 *                const captured_var *W_act_ailerons
 *                const captured_var *desired_ailerons_value
 *                const double u_in[15]
 *                double gradient[15]
 * Return Type  : double
 */
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
    double gradient[15])
{
  double gradient_fcn_15[16];
  double a;
  double a_tmp;
  double ab_sigma_1_tmp;
  double ab_sigma_1_tmp_tmp;
  double b_a;
  double b_a_tmp;
  double b_gradient_fcn_15_tmp;
  double b_gradient_fcn_15_tmp_tmp;
  double b_sigma_15_tmp;
  double b_sigma_1_tmp;
  double b_sigma_1_tmp_tmp;
  double b_sigma_3_tmp;
  double b_sigma_3_tmp_tmp;
  double b_sigma_5_tmp_tmp;
  double bb_sigma_1_tmp;
  double bb_sigma_1_tmp_tmp;
  double c_a;
  double c_a_tmp;
  double c_gradient_fcn_15_tmp;
  double c_gradient_fcn_15_tmp_tmp;
  double c_sigma_15_tmp;
  double c_sigma_1_tmp;
  double c_sigma_1_tmp_tmp;
  double c_sigma_3_tmp;
  double c_sigma_3_tmp_tmp;
  double c_sigma_5_tmp_tmp;
  double cb_sigma_1_tmp;
  double cb_sigma_1_tmp_tmp;
  double cost;
  double d_a;
  double d_a_tmp;
  double d_gradient_fcn_15_tmp;
  double d_gradient_fcn_15_tmp_tmp;
  double d_sigma_1_tmp;
  double d_sigma_1_tmp_tmp;
  double d_sigma_3_tmp;
  double d_sigma_3_tmp_tmp;
  double d_sigma_5_tmp_tmp;
  double db_sigma_1_tmp;
  double e_a;
  double e_a_tmp;
  double e_gradient_fcn_15_tmp;
  double e_gradient_fcn_15_tmp_tmp;
  double e_sigma_1_tmp;
  double e_sigma_1_tmp_tmp;
  double e_sigma_3_tmp;
  double eb_sigma_1_tmp;
  double f_a;
  double f_a_tmp;
  double f_gradient_fcn_15_tmp;
  double f_gradient_fcn_15_tmp_tmp;
  double f_sigma_1_tmp;
  double f_sigma_1_tmp_tmp;
  double f_sigma_3_tmp;
  double fb_sigma_1_tmp;
  double g_a;
  double g_a_tmp;
  double g_gradient_fcn_15_tmp;
  double g_gradient_fcn_15_tmp_tmp;
  double g_sigma_1_tmp;
  double g_sigma_1_tmp_tmp;
  double g_sigma_3_tmp;
  double gb_sigma_1_tmp;
  double gradient_fcn_15_tmp;
  double gradient_fcn_15_tmp_tmp;
  double h_a;
  double h_a_tmp;
  double h_gradient_fcn_15_tmp;
  double h_sigma_1_tmp;
  double h_sigma_1_tmp_tmp;
  double h_sigma_3_tmp;
  double hb_sigma_1_tmp;
  double i_a;
  double i_a_tmp;
  double i_gradient_fcn_15_tmp;
  double i_sigma_1_tmp;
  double i_sigma_1_tmp_tmp;
  double i_sigma_3_tmp;
  double ib_sigma_1_tmp;
  double j_a;
  double j_gradient_fcn_15_tmp;
  double j_sigma_1_tmp;
  double j_sigma_1_tmp_tmp;
  double j_sigma_3_tmp;
  double jb_sigma_1_tmp;
  double k_a;
  double k_gradient_fcn_15_tmp;
  double k_sigma_1_tmp;
  double k_sigma_1_tmp_tmp;
  double k_sigma_3_tmp;
  double kb_sigma_1_tmp;
  double l_a;
  double l_gradient_fcn_15_tmp;
  double l_sigma_1_tmp;
  double l_sigma_1_tmp_tmp;
  double lb_sigma_1_tmp;
  double m_a;
  double m_gradient_fcn_15_tmp;
  double m_sigma_1_tmp;
  double m_sigma_1_tmp_tmp;
  double n_a;
  double n_gradient_fcn_15_tmp;
  double n_sigma_1_tmp;
  double n_sigma_1_tmp_tmp;
  double o_a;
  double o_gradient_fcn_15_tmp;
  double o_sigma_1_tmp;
  double o_sigma_1_tmp_tmp;
  double p_gradient_fcn_15_tmp;
  double p_sigma_1_tmp;
  double p_sigma_1_tmp_tmp;
  double q_gradient_fcn_15_tmp;
  double q_sigma_1_tmp;
  double q_sigma_1_tmp_tmp;
  double r_gradient_fcn_15_tmp;
  double r_sigma_1_tmp;
  double r_sigma_1_tmp_tmp;
  double s_gradient_fcn_15_tmp;
  double s_sigma_1_tmp;
  double s_sigma_1_tmp_tmp;
  double sigma_1;
  double sigma_14;
  double sigma_15;
  double sigma_15_tmp;
  double sigma_1_tmp;
  double sigma_1_tmp_tmp;
  double sigma_1_tmp_tmp_tmp;
  double sigma_2;
  double sigma_3;
  double sigma_3_tmp;
  double sigma_3_tmp_tmp;
  double sigma_4;
  double sigma_4_tmp;
  double sigma_5;
  double sigma_5_tmp_tmp;
  double sigma_7;
  double sigma_7_tmp;
  double t_gradient_fcn_15_tmp;
  double t_sigma_1_tmp;
  double t_sigma_1_tmp_tmp;
  double u_gradient_fcn_15_tmp;
  double u_sigma_1_tmp;
  double u_sigma_1_tmp_tmp;
  double v_gradient_fcn_15_tmp;
  double v_sigma_1_tmp;
  double v_sigma_1_tmp_tmp;
  double w_gradient_fcn_15_tmp;
  double w_sigma_1_tmp;
  double w_sigma_1_tmp_tmp;
  double x_gradient_fcn_15_tmp;
  double x_sigma_1_tmp;
  double x_sigma_1_tmp_tmp;
  double y_sigma_1_tmp;
  double y_sigma_1_tmp_tmp;
  sigma_1_tmp_tmp_tmp = u_in[12] * gain_theta->contents;
  sigma_1_tmp_tmp = flight_path_angle->contents - sigma_1_tmp_tmp_tmp;
  b_sigma_1_tmp_tmp = cos(sigma_1_tmp_tmp);
  c_sigma_1_tmp_tmp = sin(sigma_1_tmp_tmp);
  sigma_1_tmp = cos(Beta->contents);
  d_sigma_1_tmp_tmp = cos(sigma_1_tmp_tmp_tmp);
  e_sigma_1_tmp_tmp = u_in[13] * gain_phi->contents;
  f_sigma_1_tmp_tmp = cos(e_sigma_1_tmp_tmp);
  g_sigma_1_tmp_tmp = sin(sigma_1_tmp_tmp_tmp);
  h_sigma_1_tmp_tmp = u_in[4] * gain_el->contents;
  b_sigma_1_tmp = cos(h_sigma_1_tmp_tmp);
  i_sigma_1_tmp_tmp = u_in[5] * gain_el->contents;
  c_sigma_1_tmp = cos(i_sigma_1_tmp_tmp);
  j_sigma_1_tmp_tmp = u_in[6] * gain_el->contents;
  d_sigma_1_tmp = cos(j_sigma_1_tmp_tmp);
  k_sigma_1_tmp_tmp = u_in[7] * gain_el->contents;
  e_sigma_1_tmp = cos(k_sigma_1_tmp_tmp);
  l_sigma_1_tmp_tmp = sin(e_sigma_1_tmp_tmp);
  f_sigma_1_tmp = sin(Beta->contents);
  g_sigma_1_tmp = sin(h_sigma_1_tmp_tmp);
  h_sigma_1_tmp = sin(i_sigma_1_tmp_tmp);
  i_sigma_1_tmp = sin(j_sigma_1_tmp_tmp);
  j_sigma_1_tmp = sin(k_sigma_1_tmp_tmp);
  m_sigma_1_tmp_tmp = u_in[8] * gain_az->contents;
  k_sigma_1_tmp = sin(m_sigma_1_tmp_tmp);
  n_sigma_1_tmp_tmp = u_in[9] * gain_az->contents;
  l_sigma_1_tmp = sin(n_sigma_1_tmp_tmp);
  o_sigma_1_tmp_tmp = u_in[10] * gain_az->contents;
  m_sigma_1_tmp = sin(o_sigma_1_tmp_tmp);
  p_sigma_1_tmp_tmp = u_in[11] * gain_az->contents;
  n_sigma_1_tmp = sin(p_sigma_1_tmp_tmp);
  o_sigma_1_tmp = cos(m_sigma_1_tmp_tmp);
  p_sigma_1_tmp = cos(n_sigma_1_tmp_tmp);
  q_sigma_1_tmp = cos(o_sigma_1_tmp_tmp);
  r_sigma_1_tmp = cos(p_sigma_1_tmp_tmp);
  a_tmp = V->contents;
  b_a_tmp = Cl_alpha->contents;
  c_a_tmp = gain_theta->contents;
  d_a_tmp = flight_path_angle->contents;
  e_a_tmp = gain_motor->contents;
  s_sigma_1_tmp = a_tmp * a_tmp;
  t_sigma_1_tmp = b_a_tmp * b_a_tmp;
  u_sigma_1_tmp = K_Cd->contents * t_sigma_1_tmp;
  q_sigma_1_tmp_tmp = Cl_alpha->contents * S->contents * s_sigma_1_tmp;
  v_sigma_1_tmp = q_sigma_1_tmp_tmp * rho->contents;
  r_sigma_1_tmp_tmp = S->contents * s_sigma_1_tmp;
  w_sigma_1_tmp = r_sigma_1_tmp_tmp * rho->contents;
  x_sigma_1_tmp =
      ((u_sigma_1_tmp * (u_in[12] * u_in[12]) * (c_a_tmp * c_a_tmp) -
        2.0 * K_Cd->contents * t_sigma_1_tmp * u_in[12] *
            flight_path_angle->contents * gain_theta->contents) +
       u_sigma_1_tmp * (d_a_tmp * d_a_tmp)) +
      Cd_zero->contents;
  s_sigma_1_tmp_tmp = e_a_tmp * e_a_tmp;
  y_sigma_1_tmp = K_p_T->contents * s_sigma_1_tmp_tmp;
  ab_sigma_1_tmp = u_in[0] * u_in[0];
  bb_sigma_1_tmp = u_in[1] * u_in[1];
  cb_sigma_1_tmp = u_in[2] * u_in[2];
  db_sigma_1_tmp = u_in[3] * u_in[3];
  t_sigma_1_tmp_tmp = v_sigma_1_tmp * b_sigma_1_tmp_tmp * sigma_1_tmp_tmp / 2.0;
  eb_sigma_1_tmp = t_sigma_1_tmp_tmp + w_sigma_1_tmp * c_sigma_1_tmp_tmp *
                                           sigma_1_tmp * x_sigma_1_tmp / 2.0;
  fb_sigma_1_tmp = y_sigma_1_tmp * l_sigma_1_tmp_tmp;
  u_sigma_1_tmp_tmp = b_sigma_1_tmp * o_sigma_1_tmp;
  v_sigma_1_tmp_tmp = c_sigma_1_tmp * p_sigma_1_tmp;
  w_sigma_1_tmp_tmp = d_sigma_1_tmp * q_sigma_1_tmp;
  x_sigma_1_tmp_tmp = e_sigma_1_tmp * r_sigma_1_tmp;
  gb_sigma_1_tmp = ((u_sigma_1_tmp_tmp * ab_sigma_1_tmp +
                     v_sigma_1_tmp_tmp * bb_sigma_1_tmp) +
                    w_sigma_1_tmp_tmp * cb_sigma_1_tmp) +
                   x_sigma_1_tmp_tmp * db_sigma_1_tmp;
  hb_sigma_1_tmp = y_sigma_1_tmp * f_sigma_1_tmp_tmp;
  y_sigma_1_tmp_tmp = b_sigma_1_tmp * k_sigma_1_tmp;
  ab_sigma_1_tmp_tmp = c_sigma_1_tmp * l_sigma_1_tmp;
  bb_sigma_1_tmp_tmp = d_sigma_1_tmp * m_sigma_1_tmp;
  cb_sigma_1_tmp_tmp = e_sigma_1_tmp * n_sigma_1_tmp;
  ib_sigma_1_tmp = ((y_sigma_1_tmp_tmp * ab_sigma_1_tmp +
                     ab_sigma_1_tmp_tmp * bb_sigma_1_tmp) +
                    bb_sigma_1_tmp_tmp * cb_sigma_1_tmp) +
                   cb_sigma_1_tmp_tmp * db_sigma_1_tmp;
  jb_sigma_1_tmp = v_sigma_1_tmp * c_sigma_1_tmp_tmp * sigma_1_tmp_tmp / 2.0;
  kb_sigma_1_tmp =
      ((g_sigma_1_tmp * ab_sigma_1_tmp + h_sigma_1_tmp * bb_sigma_1_tmp) +
       i_sigma_1_tmp * cb_sigma_1_tmp) +
      j_sigma_1_tmp * db_sigma_1_tmp;
  lb_sigma_1_tmp = y_sigma_1_tmp * d_sigma_1_tmp_tmp;
  sigma_1 = dv_global->contents[0] -
            (((((d_sigma_1_tmp_tmp *
                     (jb_sigma_1_tmp - w_sigma_1_tmp * b_sigma_1_tmp_tmp *
                                           sigma_1_tmp * x_sigma_1_tmp / 2.0) +
                 f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp * eb_sigma_1_tmp) -
                lb_sigma_1_tmp * kb_sigma_1_tmp) -
               hb_sigma_1_tmp * g_sigma_1_tmp_tmp * gb_sigma_1_tmp) +
              fb_sigma_1_tmp * g_sigma_1_tmp_tmp * ib_sigma_1_tmp) -
             w_sigma_1_tmp * l_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp *
                 f_sigma_1_tmp * x_sigma_1_tmp / 2.0) /
                m->contents;
  sigma_2 =
      dv_global->contents[1] +
      (((l_sigma_1_tmp_tmp * eb_sigma_1_tmp - fb_sigma_1_tmp * gb_sigma_1_tmp) -
        hb_sigma_1_tmp * ib_sigma_1_tmp) +
       w_sigma_1_tmp * f_sigma_1_tmp_tmp * f_sigma_1_tmp * x_sigma_1_tmp /
           2.0) /
          m->contents;
  sigma_3_tmp = 2.0 * K_p_T->contents;
  fb_sigma_1_tmp = 2.0 * K_p_M->contents;
  sigma_3_tmp_tmp = sigma_3_tmp * ab_sigma_1_tmp;
  b_sigma_3_tmp = sigma_3_tmp_tmp * s_sigma_1_tmp_tmp;
  b_sigma_3_tmp_tmp = sigma_3_tmp * bb_sigma_1_tmp;
  a_tmp = b_sigma_3_tmp_tmp * s_sigma_1_tmp_tmp;
  c_sigma_3_tmp_tmp = sigma_3_tmp * cb_sigma_1_tmp;
  b_a_tmp = c_sigma_3_tmp_tmp * s_sigma_1_tmp_tmp;
  d_sigma_3_tmp_tmp = sigma_3_tmp * db_sigma_1_tmp;
  c_a_tmp = d_sigma_3_tmp_tmp * s_sigma_1_tmp_tmp;
  eb_sigma_1_tmp = Cm_alpha->contents * S->contents;
  d_a_tmp = 2.0 * I_yy->contents;
  c_sigma_3_tmp = 2.0 * I_xx->contents;
  d_sigma_3_tmp = 2.0 * I_zz->contents;
  e_a_tmp = fb_sigma_1_tmp * ab_sigma_1_tmp * s_sigma_1_tmp_tmp;
  e_sigma_3_tmp = fb_sigma_1_tmp * bb_sigma_1_tmp * s_sigma_1_tmp_tmp;
  f_sigma_3_tmp = fb_sigma_1_tmp * cb_sigma_1_tmp * s_sigma_1_tmp_tmp;
  fb_sigma_1_tmp = fb_sigma_1_tmp * db_sigma_1_tmp * s_sigma_1_tmp_tmp;
  g_sigma_3_tmp = b_sigma_3_tmp * l_z->contents;
  h_sigma_3_tmp = a_tmp * l_z->contents;
  i_sigma_3_tmp = b_a_tmp * l_z->contents;
  j_sigma_3_tmp = c_a_tmp * l_z->contents;
  k_sigma_3_tmp = eb_sigma_1_tmp * s_sigma_1_tmp;
  sigma_3 =
      ((((((((((((((((d_sigma_3_tmp * p->contents * r->contents -
                      c_sigma_3_tmp * p->contents * r->contents) -
                     d_a_tmp * dv_global->contents[4]) +
                    g_sigma_3_tmp * g_sigma_1_tmp) +
                   h_sigma_3_tmp * h_sigma_1_tmp) +
                  i_sigma_3_tmp * i_sigma_1_tmp) +
                 j_sigma_3_tmp * j_sigma_1_tmp) -
                e_a_tmp * b_sigma_1_tmp * k_sigma_1_tmp) +
               e_sigma_3_tmp * c_sigma_1_tmp * l_sigma_1_tmp) -
              f_sigma_3_tmp * d_sigma_1_tmp * m_sigma_1_tmp) +
             fb_sigma_1_tmp * e_sigma_1_tmp * n_sigma_1_tmp) +
            Cm_zero->contents * S->contents * s_sigma_1_tmp * rho->contents *
                wing_chord->contents) +
           b_sigma_3_tmp * l_4->contents * b_sigma_1_tmp * o_sigma_1_tmp) +
          a_tmp * l_4->contents * c_sigma_1_tmp * p_sigma_1_tmp) -
         b_a_tmp * l_3->contents * d_sigma_1_tmp * q_sigma_1_tmp) -
        c_a_tmp * l_3->contents * e_sigma_1_tmp * r_sigma_1_tmp) -
       k_sigma_3_tmp * flight_path_angle->contents * rho->contents *
           wing_chord->contents) +
      eb_sigma_1_tmp * u_in[12] * s_sigma_1_tmp * gain_theta->contents *
          rho->contents * wing_chord->contents;
  sigma_4_tmp = CL_aileron->contents * S->contents * s_sigma_1_tmp;
  sigma_4 =
      ((((((((((((((d_a_tmp * q->contents * r->contents -
                    c_sigma_3_tmp * dv_global->contents[3]) -
                   d_sigma_3_tmp * q->contents * r->contents) +
                  e_a_tmp * g_sigma_1_tmp) -
                 e_sigma_3_tmp * h_sigma_1_tmp) +
                f_sigma_3_tmp * i_sigma_1_tmp) -
               fb_sigma_1_tmp * j_sigma_1_tmp) +
              b_sigma_3_tmp * l_1->contents * b_sigma_1_tmp * o_sigma_1_tmp) -
             a_tmp * l_1->contents * c_sigma_1_tmp * p_sigma_1_tmp) -
            b_a_tmp * l_2->contents * d_sigma_1_tmp * q_sigma_1_tmp) +
           c_a_tmp * l_2->contents * e_sigma_1_tmp * r_sigma_1_tmp) +
          g_sigma_3_tmp * b_sigma_1_tmp * k_sigma_1_tmp) +
         h_sigma_3_tmp * c_sigma_1_tmp * l_sigma_1_tmp) +
        i_sigma_3_tmp * d_sigma_1_tmp * m_sigma_1_tmp) +
       j_sigma_3_tmp * e_sigma_1_tmp * n_sigma_1_tmp) +
      sigma_4_tmp * u_in[14] * gain_ailerons->contents * rho->contents;
  sigma_5_tmp_tmp = K_p_T->contents * ab_sigma_1_tmp;
  eb_sigma_1_tmp = sigma_5_tmp_tmp * s_sigma_1_tmp_tmp;
  b_sigma_5_tmp_tmp = K_p_T->contents * bb_sigma_1_tmp;
  fb_sigma_1_tmp = b_sigma_5_tmp_tmp * s_sigma_1_tmp_tmp;
  c_sigma_5_tmp_tmp = K_p_T->contents * cb_sigma_1_tmp;
  b_sigma_3_tmp = c_sigma_5_tmp_tmp * s_sigma_1_tmp_tmp;
  d_sigma_5_tmp_tmp = K_p_T->contents * db_sigma_1_tmp;
  a_tmp = d_sigma_5_tmp_tmp * s_sigma_1_tmp_tmp;
  sigma_5 = (((((((((((((I_zz->contents * dv_global->contents[5] -
                         I_xx->contents * p->contents * q->contents) +
                        I_yy->contents * p->contents * q->contents) +
                       eb_sigma_1_tmp * l_1->contents * g_sigma_1_tmp) -
                      fb_sigma_1_tmp * l_1->contents * h_sigma_1_tmp) -
                     b_sigma_3_tmp * l_2->contents * i_sigma_1_tmp) +
                    a_tmp * l_2->contents * j_sigma_1_tmp) -
                   K_p_M->contents * ab_sigma_1_tmp * s_sigma_1_tmp_tmp *
                       b_sigma_1_tmp * o_sigma_1_tmp) +
                  K_p_M->contents * bb_sigma_1_tmp * s_sigma_1_tmp_tmp *
                      c_sigma_1_tmp * p_sigma_1_tmp) -
                 K_p_M->contents * cb_sigma_1_tmp * s_sigma_1_tmp_tmp *
                     d_sigma_1_tmp * q_sigma_1_tmp) +
                K_p_M->contents * db_sigma_1_tmp * s_sigma_1_tmp_tmp *
                    e_sigma_1_tmp * r_sigma_1_tmp) -
               eb_sigma_1_tmp * l_4->contents * b_sigma_1_tmp * k_sigma_1_tmp) -
              fb_sigma_1_tmp * l_4->contents * c_sigma_1_tmp * l_sigma_1_tmp) +
             b_sigma_3_tmp * l_3->contents * d_sigma_1_tmp * m_sigma_1_tmp) +
            a_tmp * l_3->contents * e_sigma_1_tmp * n_sigma_1_tmp;
  sigma_7_tmp = w_sigma_1_tmp * x_sigma_1_tmp;
  sigma_7 = (100.0 *
                 (((((g_sigma_1_tmp_tmp *
                          (sigma_7_tmp * b_sigma_1_tmp_tmp * sigma_1_tmp / 2.0 -
                           jb_sigma_1_tmp) +
                      f_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp *
                          (t_sigma_1_tmp_tmp + sigma_7_tmp * c_sigma_1_tmp_tmp *
                                                   sigma_1_tmp / 2.0)) +
                     y_sigma_1_tmp * g_sigma_1_tmp_tmp * kb_sigma_1_tmp) -
                    hb_sigma_1_tmp * d_sigma_1_tmp_tmp * gb_sigma_1_tmp) +
                   lb_sigma_1_tmp * l_sigma_1_tmp_tmp * ib_sigma_1_tmp) -
                  sigma_7_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp *
                      f_sigma_1_tmp / 2.0) /
                 m->contents -
             100.0 * dv_global->contents[2]) +
            981.0;
  sigma_14 = 1.0 / m->contents;
  eb_sigma_1_tmp = q_sigma_1_tmp_tmp * gain_theta->contents * rho->contents;
  sigma_15_tmp = r_sigma_1_tmp_tmp * gain_theta->contents * rho->contents;
  b_sigma_15_tmp = sigma_15_tmp * x_sigma_1_tmp;
  c_sigma_15_tmp = u_sigma_1_tmp * S->contents * s_sigma_1_tmp *
                   gain_theta->contents * rho->contents * sigma_1_tmp_tmp;
  sigma_15 = ((eb_sigma_1_tmp * b_sigma_1_tmp_tmp / 2.0 +
               b_sigma_15_tmp * sigma_1_tmp * b_sigma_1_tmp_tmp / 2.0) -
              eb_sigma_1_tmp * sigma_1_tmp_tmp * c_sigma_1_tmp_tmp / 2.0) +
             c_sigma_15_tmp * sigma_1_tmp * c_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp_tmp = e_sigma_1_tmp_tmp + m_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp = sin(gradient_fcn_15_tmp_tmp);
  b_gradient_fcn_15_tmp_tmp = e_sigma_1_tmp_tmp + n_sigma_1_tmp_tmp;
  b_gradient_fcn_15_tmp = sin(b_gradient_fcn_15_tmp_tmp);
  c_gradient_fcn_15_tmp_tmp = e_sigma_1_tmp_tmp + o_sigma_1_tmp_tmp;
  c_gradient_fcn_15_tmp = sin(c_gradient_fcn_15_tmp_tmp);
  d_gradient_fcn_15_tmp_tmp = e_sigma_1_tmp_tmp + p_sigma_1_tmp_tmp;
  d_gradient_fcn_15_tmp = sin(d_gradient_fcn_15_tmp_tmp);
  a_tmp = W_dv_4->contents;
  b_a_tmp = I_xx->contents;
  c_a_tmp = W_dv_6->contents;
  d_a_tmp = I_zz->contents;
  e_a_tmp = W_act_motor->contents;
  fb_sigma_1_tmp = W_dv_5->contents;
  b_sigma_3_tmp = I_yy->contents;
  c_sigma_3_tmp = W_dv_3->contents;
  d_sigma_3_tmp = W_dv_1->contents;
  e_sigma_3_tmp = W_dv_2->contents;
  f_sigma_3_tmp = W_act_tilt_el->contents;
  f_a_tmp = W_act_tilt_az->contents;
  g_a_tmp = W_act_theta->contents;
  h_a_tmp = W_act_phi->contents;
  i_a_tmp = W_act_ailerons->contents;
  eb_sigma_1_tmp = desired_motor_value->contents / gain_motor->contents;
  a = u_in[0] - eb_sigma_1_tmp;
  b_a = u_in[1] - eb_sigma_1_tmp;
  c_a = u_in[2] - eb_sigma_1_tmp;
  d_a = u_in[3] - eb_sigma_1_tmp;
  e_a = u_in[13] - desired_phi_value->contents / gain_phi->contents;
  f_a = u_in[12] - desired_theta_value->contents / gain_theta->contents;
  eb_sigma_1_tmp = desired_el_value->contents / gain_el->contents;
  g_a = u_in[4] - eb_sigma_1_tmp;
  h_a = u_in[5] - eb_sigma_1_tmp;
  i_a = u_in[6] - eb_sigma_1_tmp;
  j_a = u_in[7] - eb_sigma_1_tmp;
  k_a = u_in[14] - desired_ailerons_value->contents / gain_ailerons->contents;
  eb_sigma_1_tmp = desired_az_value->contents / gain_az->contents;
  l_a = u_in[8] - eb_sigma_1_tmp;
  m_a = u_in[9] - eb_sigma_1_tmp;
  n_a = u_in[10] - eb_sigma_1_tmp;
  o_a = u_in[11] - eb_sigma_1_tmp;
  e_gradient_fcn_15_tmp = K_p_T->contents * l_1->contents;
  f_gradient_fcn_15_tmp = K_p_T->contents * l_z->contents;
  g_gradient_fcn_15_tmp = K_p_M->contents * b_sigma_1_tmp;
  e_gradient_fcn_15_tmp_tmp = K_p_T->contents * l_4->contents;
  h_gradient_fcn_15_tmp = e_gradient_fcn_15_tmp_tmp * b_sigma_1_tmp;
  f_gradient_fcn_15_tmp_tmp = 4.0 * K_p_T->contents;
  i_gradient_fcn_15_tmp = f_gradient_fcn_15_tmp_tmp * u_in[0];
  j_gradient_fcn_15_tmp = fb_sigma_1_tmp * fb_sigma_1_tmp;
  k_gradient_fcn_15_tmp = b_sigma_3_tmp * b_sigma_3_tmp;
  l_gradient_fcn_15_tmp = c_a_tmp * c_a_tmp;
  m_gradient_fcn_15_tmp = d_a_tmp * d_a_tmp;
  n_gradient_fcn_15_tmp = a_tmp * a_tmp;
  o_gradient_fcn_15_tmp = b_a_tmp * b_a_tmp;
  g_gradient_fcn_15_tmp_tmp = e_a_tmp * e_a_tmp;
  p_gradient_fcn_15_tmp =
      2.0 * g_gradient_fcn_15_tmp_tmp * gamma_quadratic_du->contents;
  q_gradient_fcn_15_tmp = c_sigma_3_tmp * c_sigma_3_tmp;
  r_gradient_fcn_15_tmp = d_sigma_3_tmp * d_sigma_3_tmp;
  s_gradient_fcn_15_tmp = e_sigma_3_tmp * e_sigma_3_tmp;
  t_gradient_fcn_15_tmp = e_gradient_fcn_15_tmp * b_sigma_1_tmp;
  u_gradient_fcn_15_tmp = f_gradient_fcn_15_tmp * g_sigma_1_tmp;
  v_gradient_fcn_15_tmp = f_gradient_fcn_15_tmp * b_sigma_1_tmp;
  w_gradient_fcn_15_tmp = K_p_M->contents * g_sigma_1_tmp;
  gradient_fcn_15[0] =
      (((((2.0 * u_in[0] * n_gradient_fcn_15_tmp * sigma_4 * s_sigma_1_tmp_tmp *
               ((w_gradient_fcn_15_tmp +
                 t_gradient_fcn_15_tmp * o_sigma_1_tmp) +
                v_gradient_fcn_15_tmp * k_sigma_1_tmp) /
               o_gradient_fcn_15_tmp -
           4.0 * u_in[0] * l_gradient_fcn_15_tmp * sigma_5 * s_sigma_1_tmp_tmp *
               ((g_gradient_fcn_15_tmp * o_sigma_1_tmp -
                 e_gradient_fcn_15_tmp * g_sigma_1_tmp) +
                h_gradient_fcn_15_tmp * k_sigma_1_tmp) /
               m_gradient_fcn_15_tmp) -
          p_gradient_fcn_15_tmp *
              (desired_motor_value->contents - u_in[0] * gain_motor->contents) /
              gain_motor->contents) +
         2.0 * u_in[0] * j_gradient_fcn_15_tmp * sigma_3 * s_sigma_1_tmp_tmp *
             ((u_gradient_fcn_15_tmp - g_gradient_fcn_15_tmp * k_sigma_1_tmp) +
              h_gradient_fcn_15_tmp * o_sigma_1_tmp) /
             k_gradient_fcn_15_tmp) +
        K_p_T->contents * u_in[0] * q_gradient_fcn_15_tmp * sigma_7 *
            s_sigma_1_tmp_tmp * sigma_14 *
            ((g_sigma_1_tmp * g_sigma_1_tmp_tmp -
              u_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) +
             y_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
            25.0) +
       i_gradient_fcn_15_tmp * r_gradient_fcn_15_tmp * sigma_1 *
           s_sigma_1_tmp_tmp * sigma_14 *
           ((g_sigma_1_tmp * d_sigma_1_tmp_tmp +
             u_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) -
            y_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) -
      i_gradient_fcn_15_tmp * s_gradient_fcn_15_tmp * sigma_2 *
          s_sigma_1_tmp_tmp * sigma_14 * gradient_fcn_15_tmp * b_sigma_1_tmp;
  h_gradient_fcn_15_tmp = K_p_M->contents * c_sigma_1_tmp;
  i_gradient_fcn_15_tmp = e_gradient_fcn_15_tmp_tmp * c_sigma_1_tmp;
  d_a_tmp = f_gradient_fcn_15_tmp_tmp * u_in[1];
  e_a_tmp = e_gradient_fcn_15_tmp * c_sigma_1_tmp;
  u_sigma_1_tmp_tmp = f_gradient_fcn_15_tmp * h_sigma_1_tmp;
  y_sigma_1_tmp_tmp = K_p_M->contents * h_sigma_1_tmp;
  x_gradient_fcn_15_tmp = f_gradient_fcn_15_tmp * c_sigma_1_tmp;
  gradient_fcn_15[1] =
      (((((2.0 * u_in[1] * j_gradient_fcn_15_tmp * sigma_3 * s_sigma_1_tmp_tmp *
               ((u_sigma_1_tmp_tmp + h_gradient_fcn_15_tmp * l_sigma_1_tmp) +
                i_gradient_fcn_15_tmp * p_sigma_1_tmp) /
               k_gradient_fcn_15_tmp -
           4.0 * u_in[1] * l_gradient_fcn_15_tmp * sigma_5 * s_sigma_1_tmp_tmp *
               ((e_gradient_fcn_15_tmp * h_sigma_1_tmp -
                 h_gradient_fcn_15_tmp * p_sigma_1_tmp) +
                i_gradient_fcn_15_tmp * l_sigma_1_tmp) /
               m_gradient_fcn_15_tmp) -
          2.0 * u_in[1] * n_gradient_fcn_15_tmp * sigma_4 * s_sigma_1_tmp_tmp *
              ((y_sigma_1_tmp_tmp + e_a_tmp * p_sigma_1_tmp) -
               x_gradient_fcn_15_tmp * l_sigma_1_tmp) /
              o_gradient_fcn_15_tmp) -
         p_gradient_fcn_15_tmp *
             (desired_motor_value->contents - u_in[1] * gain_motor->contents) /
             gain_motor->contents) +
        K_p_T->contents * u_in[1] * q_gradient_fcn_15_tmp * sigma_7 *
            s_sigma_1_tmp_tmp * sigma_14 *
            ((h_sigma_1_tmp * g_sigma_1_tmp_tmp -
              v_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) +
             ab_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
            25.0) +
       d_a_tmp * r_gradient_fcn_15_tmp * sigma_1 * s_sigma_1_tmp_tmp *
           sigma_14 *
           ((h_sigma_1_tmp * d_sigma_1_tmp_tmp +
             v_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) -
            ab_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) -
      d_a_tmp * s_gradient_fcn_15_tmp * sigma_2 * s_sigma_1_tmp_tmp * sigma_14 *
          b_gradient_fcn_15_tmp * c_sigma_1_tmp;
  i_gradient_fcn_15_tmp = K_p_T->contents * l_2->contents;
  d_a_tmp = K_p_M->contents * d_sigma_1_tmp;
  q_sigma_1_tmp_tmp = K_p_T->contents * l_3->contents;
  w_sigma_1_tmp = q_sigma_1_tmp_tmp * d_sigma_1_tmp;
  r_sigma_1_tmp_tmp = f_gradient_fcn_15_tmp_tmp * u_in[2];
  u_sigma_1_tmp = f_gradient_fcn_15_tmp * i_sigma_1_tmp;
  a_tmp = i_gradient_fcn_15_tmp * d_sigma_1_tmp;
  b_a_tmp = f_gradient_fcn_15_tmp * d_sigma_1_tmp;
  c_a_tmp = K_p_M->contents * i_sigma_1_tmp;
  gradient_fcn_15[2] =
      (((((2.0 * u_in[2] * n_gradient_fcn_15_tmp * sigma_4 * s_sigma_1_tmp_tmp *
               ((c_a_tmp - a_tmp * q_sigma_1_tmp) + b_a_tmp * m_sigma_1_tmp) /
               o_gradient_fcn_15_tmp -
           4.0 * u_in[2] * l_gradient_fcn_15_tmp * sigma_5 * s_sigma_1_tmp_tmp *
               ((i_gradient_fcn_15_tmp * i_sigma_1_tmp +
                 d_a_tmp * q_sigma_1_tmp) -
                w_sigma_1_tmp * m_sigma_1_tmp) /
               m_gradient_fcn_15_tmp) -
          p_gradient_fcn_15_tmp *
              (desired_motor_value->contents - u_in[2] * gain_motor->contents) /
              gain_motor->contents) -
         2.0 * u_in[2] * j_gradient_fcn_15_tmp * sigma_3 * s_sigma_1_tmp_tmp *
             ((d_a_tmp * m_sigma_1_tmp - u_sigma_1_tmp) +
              w_sigma_1_tmp * q_sigma_1_tmp) /
             k_gradient_fcn_15_tmp) +
        K_p_T->contents * u_in[2] * q_gradient_fcn_15_tmp * sigma_7 *
            s_sigma_1_tmp_tmp * sigma_14 *
            ((i_sigma_1_tmp * g_sigma_1_tmp_tmp -
              w_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) +
             bb_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
            25.0) +
       r_sigma_1_tmp_tmp * r_gradient_fcn_15_tmp * sigma_1 * s_sigma_1_tmp_tmp *
           sigma_14 *
           ((i_sigma_1_tmp * d_sigma_1_tmp_tmp +
             w_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) -
            bb_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) -
      r_sigma_1_tmp_tmp * s_gradient_fcn_15_tmp * sigma_2 * s_sigma_1_tmp_tmp *
          sigma_14 * c_gradient_fcn_15_tmp * d_sigma_1_tmp;
  w_sigma_1_tmp = K_p_M->contents * e_sigma_1_tmp;
  r_sigma_1_tmp_tmp = q_sigma_1_tmp_tmp * e_sigma_1_tmp;
  eb_sigma_1_tmp = f_gradient_fcn_15_tmp_tmp * u_in[3];
  hb_sigma_1_tmp = f_gradient_fcn_15_tmp * e_sigma_1_tmp;
  jb_sigma_1_tmp = K_p_M->contents * j_sigma_1_tmp;
  lb_sigma_1_tmp = i_gradient_fcn_15_tmp * e_sigma_1_tmp;
  f_gradient_fcn_15_tmp *= j_sigma_1_tmp;
  gradient_fcn_15[3] =
      (((((4.0 * u_in[3] * l_gradient_fcn_15_tmp * sigma_5 * s_sigma_1_tmp_tmp *
               ((i_gradient_fcn_15_tmp * j_sigma_1_tmp +
                 w_sigma_1_tmp * r_sigma_1_tmp) +
                r_sigma_1_tmp_tmp * n_sigma_1_tmp) /
               m_gradient_fcn_15_tmp -
           p_gradient_fcn_15_tmp *
               (desired_motor_value->contents -
                u_in[3] * gain_motor->contents) /
               gain_motor->contents) +
          2.0 * u_in[3] * n_gradient_fcn_15_tmp * sigma_4 * s_sigma_1_tmp_tmp *
              ((lb_sigma_1_tmp * r_sigma_1_tmp - jb_sigma_1_tmp) +
               hb_sigma_1_tmp * n_sigma_1_tmp) /
              o_gradient_fcn_15_tmp) +
         2.0 * u_in[3] * j_gradient_fcn_15_tmp * sigma_3 * s_sigma_1_tmp_tmp *
             ((f_gradient_fcn_15_tmp + w_sigma_1_tmp * n_sigma_1_tmp) -
              r_sigma_1_tmp_tmp * r_sigma_1_tmp) /
             k_gradient_fcn_15_tmp) +
        K_p_T->contents * u_in[3] * q_gradient_fcn_15_tmp * sigma_7 *
            s_sigma_1_tmp_tmp * sigma_14 *
            ((j_sigma_1_tmp * g_sigma_1_tmp_tmp -
              x_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) +
             cb_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
            25.0) +
       eb_sigma_1_tmp * r_gradient_fcn_15_tmp * sigma_1 * s_sigma_1_tmp_tmp *
           sigma_14 *
           ((j_sigma_1_tmp * d_sigma_1_tmp_tmp +
             x_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) -
            cb_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) -
      eb_sigma_1_tmp * s_gradient_fcn_15_tmp * sigma_2 * s_sigma_1_tmp_tmp *
          sigma_14 * d_gradient_fcn_15_tmp * e_sigma_1_tmp;
  p_gradient_fcn_15_tmp = o_sigma_1_tmp * g_sigma_1_tmp;
  r_sigma_1_tmp_tmp = g_sigma_1_tmp * k_sigma_1_tmp;
  f_gradient_fcn_15_tmp_tmp = f_sigma_3_tmp * f_sigma_3_tmp;
  eb_sigma_1_tmp =
      2.0 * f_gradient_fcn_15_tmp_tmp * gamma_quadratic_du->contents;
  g_sigma_3_tmp = 2.0 * ab_sigma_1_tmp * l_gradient_fcn_15_tmp;
  h_sigma_3_tmp = e_gradient_fcn_15_tmp_tmp * o_sigma_1_tmp;
  i_sigma_3_tmp = ab_sigma_1_tmp * j_gradient_fcn_15_tmp;
  j_sigma_3_tmp = K_p_M->contents * o_sigma_1_tmp;
  y_sigma_1_tmp = sigma_3_tmp_tmp * s_gradient_fcn_15_tmp;
  t_sigma_1_tmp_tmp = sigma_5_tmp_tmp * q_gradient_fcn_15_tmp;
  f_sigma_3_tmp = sigma_3_tmp_tmp * r_gradient_fcn_15_tmp;
  gradient_fcn_15[4] =
      (((((g_sigma_3_tmp * gain_el->contents * sigma_5 * s_sigma_1_tmp_tmp *
               ((t_gradient_fcn_15_tmp + j_sigma_3_tmp * g_sigma_1_tmp) +
                e_gradient_fcn_15_tmp_tmp * g_sigma_1_tmp * k_sigma_1_tmp) /
               m_gradient_fcn_15_tmp -
           eb_sigma_1_tmp * (desired_el_value->contents - h_sigma_1_tmp_tmp) /
               gain_el->contents) -
          ab_sigma_1_tmp * n_gradient_fcn_15_tmp * gain_el->contents * sigma_4 *
              s_sigma_1_tmp_tmp *
              ((e_gradient_fcn_15_tmp * o_sigma_1_tmp * g_sigma_1_tmp -
                g_gradient_fcn_15_tmp) +
               u_gradient_fcn_15_tmp * k_sigma_1_tmp) /
              o_gradient_fcn_15_tmp) +
         i_sigma_3_tmp * gain_el->contents * sigma_3 * s_sigma_1_tmp_tmp *
             ((v_gradient_fcn_15_tmp + w_gradient_fcn_15_tmp * k_sigma_1_tmp) -
              h_sigma_3_tmp * g_sigma_1_tmp) /
             k_gradient_fcn_15_tmp) +
        t_sigma_1_tmp_tmp * gain_el->contents * sigma_7 * s_sigma_1_tmp_tmp *
            sigma_14 *
            ((b_sigma_1_tmp * g_sigma_1_tmp_tmp +
              p_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) -
             r_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
            50.0) +
       f_sigma_3_tmp * gain_el->contents * sigma_1 * s_sigma_1_tmp_tmp *
           sigma_14 *
           ((b_sigma_1_tmp * d_sigma_1_tmp_tmp -
             p_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) +
            r_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) +
      y_sigma_1_tmp * gain_el->contents * sigma_2 * s_sigma_1_tmp_tmp *
          sigma_14 * gradient_fcn_15_tmp * g_sigma_1_tmp;
  g_gradient_fcn_15_tmp = p_sigma_1_tmp * h_sigma_1_tmp;
  p_gradient_fcn_15_tmp = h_sigma_1_tmp * l_sigma_1_tmp;
  t_gradient_fcn_15_tmp = bb_sigma_1_tmp * j_gradient_fcn_15_tmp;
  u_gradient_fcn_15_tmp = K_p_M->contents * p_sigma_1_tmp;
  v_gradient_fcn_15_tmp = 2.0 * bb_sigma_1_tmp * l_gradient_fcn_15_tmp;
  w_gradient_fcn_15_tmp = e_gradient_fcn_15_tmp_tmp * p_sigma_1_tmp;
  r_sigma_1_tmp_tmp = b_sigma_3_tmp_tmp * s_gradient_fcn_15_tmp;
  e_sigma_3_tmp = b_sigma_5_tmp_tmp * q_gradient_fcn_15_tmp;
  d_sigma_3_tmp = b_sigma_3_tmp_tmp * r_gradient_fcn_15_tmp;
  gradient_fcn_15[5] =
      (((((e_sigma_3_tmp * gain_el->contents * sigma_7 * s_sigma_1_tmp_tmp *
               sigma_14 *
               ((c_sigma_1_tmp * g_sigma_1_tmp_tmp + g_gradient_fcn_15_tmp *
                                                         d_sigma_1_tmp_tmp *
                                                         f_sigma_1_tmp_tmp) -
                p_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
               50.0 -
           v_gradient_fcn_15_tmp * gain_el->contents * sigma_5 *
               s_sigma_1_tmp_tmp *
               ((e_a_tmp + u_gradient_fcn_15_tmp * h_sigma_1_tmp) -
                e_gradient_fcn_15_tmp_tmp * h_sigma_1_tmp * l_sigma_1_tmp) /
               m_gradient_fcn_15_tmp) -
          bb_sigma_1_tmp * n_gradient_fcn_15_tmp * gain_el->contents * sigma_4 *
              s_sigma_1_tmp_tmp *
              ((h_gradient_fcn_15_tmp -
                e_gradient_fcn_15_tmp * p_sigma_1_tmp * h_sigma_1_tmp) +
               u_sigma_1_tmp_tmp * l_sigma_1_tmp) /
              o_gradient_fcn_15_tmp) -
         t_gradient_fcn_15_tmp * gain_el->contents * sigma_3 *
             s_sigma_1_tmp_tmp *
             ((y_sigma_1_tmp_tmp * l_sigma_1_tmp - x_gradient_fcn_15_tmp) +
              w_gradient_fcn_15_tmp * h_sigma_1_tmp) /
             k_gradient_fcn_15_tmp) -
        eb_sigma_1_tmp * (desired_el_value->contents - i_sigma_1_tmp_tmp) /
            gain_el->contents) +
       d_sigma_3_tmp * gain_el->contents * sigma_1 * s_sigma_1_tmp_tmp *
           sigma_14 *
           ((c_sigma_1_tmp * d_sigma_1_tmp_tmp -
             g_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) +
            p_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) +
      r_sigma_1_tmp_tmp * gain_el->contents * sigma_2 * s_sigma_1_tmp_tmp *
          sigma_14 * b_gradient_fcn_15_tmp * h_sigma_1_tmp;
  e_gradient_fcn_15_tmp = q_sigma_1_tmp * i_sigma_1_tmp;
  g_gradient_fcn_15_tmp = i_sigma_1_tmp * m_sigma_1_tmp;
  h_gradient_fcn_15_tmp = 2.0 * cb_sigma_1_tmp * l_gradient_fcn_15_tmp;
  p_gradient_fcn_15_tmp = q_sigma_1_tmp_tmp * q_sigma_1_tmp;
  e_a_tmp = cb_sigma_1_tmp * j_gradient_fcn_15_tmp;
  u_sigma_1_tmp_tmp = K_p_M->contents * q_sigma_1_tmp;
  y_sigma_1_tmp_tmp = c_sigma_3_tmp_tmp * s_gradient_fcn_15_tmp;
  x_gradient_fcn_15_tmp = c_sigma_5_tmp_tmp * q_gradient_fcn_15_tmp;
  c_sigma_3_tmp = c_sigma_3_tmp_tmp * r_gradient_fcn_15_tmp;
  gradient_fcn_15[6] =
      (((((cb_sigma_1_tmp * n_gradient_fcn_15_tmp * gain_el->contents *
               sigma_4 * s_sigma_1_tmp_tmp *
               ((d_a_tmp +
                 i_gradient_fcn_15_tmp * q_sigma_1_tmp * i_sigma_1_tmp) -
                u_sigma_1_tmp * m_sigma_1_tmp) /
               o_gradient_fcn_15_tmp -
           h_gradient_fcn_15_tmp * gain_el->contents * sigma_5 *
               s_sigma_1_tmp_tmp *
               ((a_tmp - u_sigma_1_tmp_tmp * i_sigma_1_tmp) +
                q_sigma_1_tmp_tmp * i_sigma_1_tmp * m_sigma_1_tmp) /
               m_gradient_fcn_15_tmp) -
          eb_sigma_1_tmp * (desired_el_value->contents - j_sigma_1_tmp_tmp) /
              gain_el->contents) +
         e_a_tmp * gain_el->contents * sigma_3 * s_sigma_1_tmp_tmp *
             ((b_a_tmp + c_a_tmp * m_sigma_1_tmp) +
              p_gradient_fcn_15_tmp * i_sigma_1_tmp) /
             k_gradient_fcn_15_tmp) +
        x_gradient_fcn_15_tmp * gain_el->contents * sigma_7 *
            s_sigma_1_tmp_tmp * sigma_14 *
            ((d_sigma_1_tmp * g_sigma_1_tmp_tmp +
              e_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) -
             g_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
            50.0) +
       c_sigma_3_tmp * gain_el->contents * sigma_1 * s_sigma_1_tmp_tmp *
           sigma_14 *
           ((d_sigma_1_tmp * d_sigma_1_tmp_tmp -
             e_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) +
            g_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) +
      y_sigma_1_tmp_tmp * gain_el->contents * sigma_2 * s_sigma_1_tmp_tmp *
          sigma_14 * c_gradient_fcn_15_tmp * i_sigma_1_tmp;
  e_gradient_fcn_15_tmp = r_sigma_1_tmp * j_sigma_1_tmp;
  g_gradient_fcn_15_tmp = j_sigma_1_tmp * n_sigma_1_tmp;
  d_a_tmp = db_sigma_1_tmp * j_gradient_fcn_15_tmp;
  u_sigma_1_tmp = K_p_M->contents * r_sigma_1_tmp;
  a_tmp = 2.0 * db_sigma_1_tmp * l_gradient_fcn_15_tmp;
  b_a_tmp = q_sigma_1_tmp_tmp * r_sigma_1_tmp;
  c_a_tmp = d_sigma_3_tmp_tmp * s_gradient_fcn_15_tmp;
  b_sigma_3_tmp = d_sigma_5_tmp_tmp * q_gradient_fcn_15_tmp;
  fb_sigma_1_tmp = d_sigma_3_tmp_tmp * r_gradient_fcn_15_tmp;
  gradient_fcn_15[7] =
      (((((d_a_tmp * gain_el->contents * sigma_3 * s_sigma_1_tmp_tmp *
               ((hb_sigma_1_tmp - jb_sigma_1_tmp * n_sigma_1_tmp) +
                b_a_tmp * j_sigma_1_tmp) /
               k_gradient_fcn_15_tmp -
           a_tmp * gain_el->contents * sigma_5 * s_sigma_1_tmp_tmp *
               ((u_sigma_1_tmp * j_sigma_1_tmp - lb_sigma_1_tmp) +
                q_sigma_1_tmp_tmp * j_sigma_1_tmp * n_sigma_1_tmp) /
               m_gradient_fcn_15_tmp) -
          db_sigma_1_tmp * n_gradient_fcn_15_tmp * gain_el->contents * sigma_4 *
              s_sigma_1_tmp_tmp *
              ((w_sigma_1_tmp +
                i_gradient_fcn_15_tmp * r_sigma_1_tmp * j_sigma_1_tmp) +
               f_gradient_fcn_15_tmp * n_sigma_1_tmp) /
              o_gradient_fcn_15_tmp) -
         eb_sigma_1_tmp * (desired_el_value->contents - k_sigma_1_tmp_tmp) /
             gain_el->contents) +
        b_sigma_3_tmp * gain_el->contents * sigma_7 * s_sigma_1_tmp_tmp *
            sigma_14 *
            ((e_sigma_1_tmp * g_sigma_1_tmp_tmp +
              e_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) -
             g_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
            50.0) +
       fb_sigma_1_tmp * gain_el->contents * sigma_1 * s_sigma_1_tmp_tmp *
           sigma_14 *
           ((e_sigma_1_tmp * d_sigma_1_tmp_tmp -
             e_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) +
            g_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) +
      c_a_tmp * gain_el->contents * sigma_2 * s_sigma_1_tmp_tmp * sigma_14 *
          d_gradient_fcn_15_tmp * j_sigma_1_tmp;
  eb_sigma_1_tmp = f_a_tmp * f_a_tmp;
  e_gradient_fcn_15_tmp = 2.0 * eb_sigma_1_tmp * gamma_quadratic_du->contents;
  gradient_fcn_15[8] =
      (((((g_sigma_3_tmp * gain_az->contents * sigma_5 * s_sigma_1_tmp_tmp *
               b_sigma_1_tmp *
               (K_p_M->contents * k_sigma_1_tmp - h_sigma_3_tmp) /
               m_gradient_fcn_15_tmp -
           i_sigma_3_tmp * gain_az->contents * sigma_3 * s_sigma_1_tmp_tmp *
               b_sigma_1_tmp *
               (j_sigma_3_tmp + e_gradient_fcn_15_tmp_tmp * k_sigma_1_tmp) /
               k_gradient_fcn_15_tmp) -
          e_gradient_fcn_15_tmp *
              (desired_az_value->contents - m_sigma_1_tmp_tmp) /
              gain_az->contents) +
         sigma_5_tmp_tmp * n_gradient_fcn_15_tmp * gain_az->contents * sigma_4 *
             s_sigma_1_tmp_tmp * b_sigma_1_tmp *
             (l_z->contents * o_sigma_1_tmp - l_1->contents * k_sigma_1_tmp) /
             o_gradient_fcn_15_tmp) -
        y_sigma_1_tmp * gain_az->contents * sigma_2 * s_sigma_1_tmp_tmp *
            sigma_14 * cos(gradient_fcn_15_tmp_tmp) * b_sigma_1_tmp) +
       t_sigma_1_tmp_tmp * gain_az->contents * sigma_7 * s_sigma_1_tmp_tmp *
           sigma_14 * gradient_fcn_15_tmp * b_sigma_1_tmp * d_sigma_1_tmp_tmp /
           50.0) -
      f_sigma_3_tmp * gain_az->contents * sigma_1 * s_sigma_1_tmp_tmp *
          sigma_14 * gradient_fcn_15_tmp * b_sigma_1_tmp * g_sigma_1_tmp_tmp;
  gradient_fcn_15[9] =
      (((((t_gradient_fcn_15_tmp * gain_az->contents * sigma_3 *
               s_sigma_1_tmp_tmp * c_sigma_1_tmp *
               (u_gradient_fcn_15_tmp -
                e_gradient_fcn_15_tmp_tmp * l_sigma_1_tmp) /
               k_gradient_fcn_15_tmp -
           e_gradient_fcn_15_tmp *
               (desired_az_value->contents - n_sigma_1_tmp_tmp) /
               gain_az->contents) -
          v_gradient_fcn_15_tmp * gain_az->contents * sigma_5 *
              s_sigma_1_tmp_tmp * c_sigma_1_tmp *
              (K_p_M->contents * l_sigma_1_tmp + w_gradient_fcn_15_tmp) /
              m_gradient_fcn_15_tmp) +
         b_sigma_5_tmp_tmp * n_gradient_fcn_15_tmp * gain_az->contents *
             sigma_4 * s_sigma_1_tmp_tmp * c_sigma_1_tmp *
             (l_z->contents * p_sigma_1_tmp + l_1->contents * l_sigma_1_tmp) /
             o_gradient_fcn_15_tmp) -
        r_sigma_1_tmp_tmp * gain_az->contents * sigma_2 * s_sigma_1_tmp_tmp *
            sigma_14 * cos(b_gradient_fcn_15_tmp_tmp) * c_sigma_1_tmp) +
       e_sigma_3_tmp * gain_az->contents * sigma_7 * s_sigma_1_tmp_tmp *
           sigma_14 * b_gradient_fcn_15_tmp * c_sigma_1_tmp *
           d_sigma_1_tmp_tmp / 50.0) -
      d_sigma_3_tmp * gain_az->contents * sigma_1 * s_sigma_1_tmp_tmp *
          sigma_14 * b_gradient_fcn_15_tmp * c_sigma_1_tmp * g_sigma_1_tmp_tmp;
  gradient_fcn_15[10] =
      (((((h_gradient_fcn_15_tmp * gain_az->contents * sigma_5 *
               s_sigma_1_tmp_tmp * d_sigma_1_tmp *
               (K_p_M->contents * m_sigma_1_tmp + p_gradient_fcn_15_tmp) /
               m_gradient_fcn_15_tmp -
           e_a_tmp * gain_az->contents * sigma_3 * s_sigma_1_tmp_tmp *
               d_sigma_1_tmp *
               (u_sigma_1_tmp_tmp - q_sigma_1_tmp_tmp * m_sigma_1_tmp) /
               k_gradient_fcn_15_tmp) -
          e_gradient_fcn_15_tmp *
              (desired_az_value->contents - o_sigma_1_tmp_tmp) /
              gain_az->contents) +
         c_sigma_5_tmp_tmp * n_gradient_fcn_15_tmp * gain_az->contents *
             sigma_4 * s_sigma_1_tmp_tmp * d_sigma_1_tmp *
             (l_z->contents * q_sigma_1_tmp + l_2->contents * m_sigma_1_tmp) /
             o_gradient_fcn_15_tmp) -
        y_sigma_1_tmp_tmp * gain_az->contents * sigma_2 * s_sigma_1_tmp_tmp *
            sigma_14 * cos(c_gradient_fcn_15_tmp_tmp) * d_sigma_1_tmp) +
       x_gradient_fcn_15_tmp * gain_az->contents * sigma_7 * s_sigma_1_tmp_tmp *
           sigma_14 * c_gradient_fcn_15_tmp * d_sigma_1_tmp *
           d_sigma_1_tmp_tmp / 50.0) -
      c_sigma_3_tmp * gain_az->contents * sigma_1 * s_sigma_1_tmp_tmp *
          sigma_14 * c_gradient_fcn_15_tmp * d_sigma_1_tmp * g_sigma_1_tmp_tmp;
  gradient_fcn_15[11] =
      (((((d_a_tmp * gain_az->contents * sigma_3 * s_sigma_1_tmp_tmp *
               e_sigma_1_tmp *
               (u_sigma_1_tmp + q_sigma_1_tmp_tmp * n_sigma_1_tmp) /
               k_gradient_fcn_15_tmp -
           e_gradient_fcn_15_tmp *
               (desired_az_value->contents - p_sigma_1_tmp_tmp) /
               gain_az->contents) -
          a_tmp * gain_az->contents * sigma_5 * s_sigma_1_tmp_tmp *
              e_sigma_1_tmp * (K_p_M->contents * n_sigma_1_tmp - b_a_tmp) /
              m_gradient_fcn_15_tmp) +
         d_sigma_5_tmp_tmp * n_gradient_fcn_15_tmp * gain_az->contents *
             sigma_4 * s_sigma_1_tmp_tmp * e_sigma_1_tmp *
             (l_z->contents * r_sigma_1_tmp - l_2->contents * n_sigma_1_tmp) /
             o_gradient_fcn_15_tmp) -
        c_a_tmp * gain_az->contents * sigma_2 * s_sigma_1_tmp_tmp * sigma_14 *
            cos(d_gradient_fcn_15_tmp_tmp) * e_sigma_1_tmp) +
       b_sigma_3_tmp * gain_az->contents * sigma_7 * s_sigma_1_tmp_tmp *
           sigma_14 * d_gradient_fcn_15_tmp * e_sigma_1_tmp *
           d_sigma_1_tmp_tmp / 50.0) -
      fb_sigma_1_tmp * gain_az->contents * sigma_1 * s_sigma_1_tmp_tmp *
          sigma_14 * d_gradient_fcn_15_tmp * e_sigma_1_tmp * g_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp = K_p_T->contents * gain_theta->contents;
  b_gradient_fcn_15_tmp = x_sigma_1_tmp * sigma_1_tmp;
  c_gradient_fcn_15_tmp = Cl_alpha->contents * sigma_1_tmp_tmp;
  d_gradient_fcn_15_tmp = b_gradient_fcn_15_tmp * c_sigma_1_tmp_tmp;
  e_gradient_fcn_15_tmp = c_gradient_fcn_15_tmp * b_sigma_1_tmp_tmp;
  f_gradient_fcn_15_tmp = gradient_fcn_15_tmp * s_sigma_1_tmp_tmp;
  g_gradient_fcn_15_tmp = sigma_15_tmp * d_sigma_1_tmp_tmp;
  h_gradient_fcn_15_tmp =
      ((Cl_alpha->contents * c_sigma_1_tmp_tmp + d_gradient_fcn_15_tmp) +
       e_gradient_fcn_15_tmp) -
      2.0 * t_sigma_1_tmp * K_Cd->contents * sigma_1_tmp_tmp * sigma_1_tmp *
          b_sigma_1_tmp_tmp;
  i_gradient_fcn_15_tmp =
      gradient_fcn_15_tmp * gb_sigma_1_tmp * s_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp =
      gradient_fcn_15_tmp * ib_sigma_1_tmp * s_sigma_1_tmp_tmp;
  p_gradient_fcn_15_tmp = sigma_15_tmp * g_sigma_1_tmp_tmp;
  b_gradient_fcn_15_tmp = b_gradient_fcn_15_tmp * b_sigma_1_tmp_tmp -
                          c_gradient_fcn_15_tmp * c_sigma_1_tmp_tmp;
  c_gradient_fcn_15_tmp = d_gradient_fcn_15_tmp + e_gradient_fcn_15_tmp;
  d_gradient_fcn_15_tmp = b_sigma_15_tmp * f_sigma_1_tmp;
  e_gradient_fcn_15_tmp = c_sigma_15_tmp * f_sigma_1_tmp;
  t_gradient_fcn_15_tmp = g_a_tmp * g_a_tmp;
  gradient_fcn_15[12] =
      (((q_gradient_fcn_15_tmp * sigma_7 * sigma_14 *
             ((((((((f_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp *
                         kb_sigma_1_tmp -
                     sigma_15 * d_sigma_1_tmp_tmp * f_sigma_1_tmp_tmp) +
                    p_gradient_fcn_15_tmp * h_gradient_fcn_15_tmp / 2.0) +
                   i_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp *
                       g_sigma_1_tmp_tmp) +
                  g_gradient_fcn_15_tmp * b_gradient_fcn_15_tmp / 2.0) -
                 gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) -
                sigma_15_tmp * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp *
                    c_gradient_fcn_15_tmp / 2.0) +
               d_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp /
                   2.0) +
              e_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) /
             50.0 -
         2.0 * r_gradient_fcn_15_tmp * sigma_1 * sigma_14 *
             ((((((((f_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp *
                         kb_sigma_1_tmp -
                     sigma_15 * f_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) -
                    g_gradient_fcn_15_tmp * h_gradient_fcn_15_tmp / 2.0) -
                   i_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp *
                       f_sigma_1_tmp_tmp) +
                  gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp) +
                 p_gradient_fcn_15_tmp * b_gradient_fcn_15_tmp / 2.0) +
                g_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp *
                    c_gradient_fcn_15_tmp / 2.0) -
               d_gradient_fcn_15_tmp * d_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp /
                   2.0) +
              e_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp)) -
        2.0 * s_gradient_fcn_15_tmp * sigma_2 * sigma_14 *
            (K_Cd->contents * S->contents * gain_theta->contents *
                 rho->contents * sigma_1_tmp_tmp * f_sigma_1_tmp *
                 f_sigma_1_tmp_tmp * t_sigma_1_tmp * s_sigma_1_tmp +
             sigma_15 * l_sigma_1_tmp_tmp)) -
       2.0 * t_gradient_fcn_15_tmp * gamma_quadratic_du->contents *
           (desired_theta_value->contents - sigma_1_tmp_tmp_tmp) /
           gain_theta->contents) +
      k_sigma_3_tmp * j_gradient_fcn_15_tmp * gain_theta->contents *
          rho->contents * sigma_3 * wing_chord->contents /
          (2.0 * k_gradient_fcn_15_tmp);
  gradient_fcn_15_tmp = sigma_3_tmp * ib_sigma_1_tmp * s_sigma_1_tmp_tmp;
  b_gradient_fcn_15_tmp = sigma_3_tmp * gb_sigma_1_tmp * s_sigma_1_tmp_tmp;
  c_gradient_fcn_15_tmp = sigma_7_tmp * f_sigma_1_tmp;
  d_gradient_fcn_15_tmp = v_sigma_1_tmp * sigma_1_tmp_tmp * b_sigma_1_tmp_tmp;
  e_gradient_fcn_15_tmp = sigma_7_tmp * sigma_1_tmp;
  f_gradient_fcn_15_tmp =
      (((c_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp -
         b_gradient_fcn_15_tmp * l_sigma_1_tmp_tmp) -
        gradient_fcn_15_tmp * f_sigma_1_tmp_tmp) +
       d_gradient_fcn_15_tmp * l_sigma_1_tmp_tmp) +
      e_gradient_fcn_15_tmp * c_sigma_1_tmp_tmp * l_sigma_1_tmp_tmp;
  g_gradient_fcn_15_tmp = h_a_tmp * h_a_tmp;
  gradient_fcn_15[13] =
      ((gain_phi->contents * sigma_1 * sigma_14 * g_sigma_1_tmp_tmp *
            f_gradient_fcn_15_tmp * r_gradient_fcn_15_tmp +
        gain_phi->contents * sigma_2 * sigma_14 *
            ((((gradient_fcn_15_tmp * l_sigma_1_tmp_tmp -
                b_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp) -
               c_gradient_fcn_15_tmp * l_sigma_1_tmp_tmp) +
              d_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp) +
             e_gradient_fcn_15_tmp * f_sigma_1_tmp_tmp * c_sigma_1_tmp_tmp) *
            s_gradient_fcn_15_tmp) -
       2.0 * g_gradient_fcn_15_tmp * gamma_quadratic_du->contents *
           (desired_phi_value->contents - e_sigma_1_tmp_tmp) /
           gain_phi->contents) -
      q_gradient_fcn_15_tmp * gain_phi->contents * sigma_7 * sigma_14 *
          d_sigma_1_tmp_tmp * f_gradient_fcn_15_tmp / 100.0;
  gradient_fcn_15_tmp = i_a_tmp * i_a_tmp;
  gradient_fcn_15[14] =
      gradient_fcn_15_tmp * gamma_quadratic_du->contents *
          (2.0 * u_in[14] -
           2.0 * desired_ailerons_value->contents / gain_ailerons->contents) +
      sigma_4_tmp * n_gradient_fcn_15_tmp * gain_ailerons->contents *
          rho->contents * sigma_4 / (2.0 * o_gradient_fcn_15_tmp);
  gradient_fcn_15[15] =
      (((((gamma_quadratic_du->contents *
               ((((((((((((((g_gradient_fcn_15_tmp_tmp * (a * a) +
                             g_gradient_fcn_15_tmp_tmp * (b_a * b_a)) +
                            g_gradient_fcn_15_tmp_tmp * (c_a * c_a)) +
                           g_gradient_fcn_15_tmp_tmp * (d_a * d_a)) +
                          g_gradient_fcn_15_tmp * (e_a * e_a)) +
                         t_gradient_fcn_15_tmp * (f_a * f_a)) +
                        f_gradient_fcn_15_tmp_tmp * (g_a * g_a)) +
                       f_gradient_fcn_15_tmp_tmp * (h_a * h_a)) +
                      f_gradient_fcn_15_tmp_tmp * (i_a * i_a)) +
                     f_gradient_fcn_15_tmp_tmp * (j_a * j_a)) +
                    gradient_fcn_15_tmp * (k_a * k_a)) +
                   eb_sigma_1_tmp * (l_a * l_a)) +
                  eb_sigma_1_tmp * (m_a * m_a)) +
                 eb_sigma_1_tmp * (n_a * n_a)) +
                eb_sigma_1_tmp * (o_a * o_a)) +
           r_gradient_fcn_15_tmp * (sigma_1 * sigma_1)) +
          s_gradient_fcn_15_tmp * (sigma_2 * sigma_2)) +
         q_gradient_fcn_15_tmp * (sigma_7 * sigma_7) / 10000.0) +
        n_gradient_fcn_15_tmp * (sigma_4 * sigma_4) /
            (4.0 * o_gradient_fcn_15_tmp)) +
       j_gradient_fcn_15_tmp * (sigma_3 * sigma_3) /
           (4.0 * k_gradient_fcn_15_tmp)) +
      l_gradient_fcn_15_tmp * (sigma_5 * sigma_5) / m_gradient_fcn_15_tmp;
  cost = gradient_fcn_15[15];
  memcpy(&gradient[0], &gradient_fcn_15[0], 15U * sizeof(double));
  return cost;
}

/*
 * File trailer for Nonlinear_CA_w_ail_approach_ext_acc.c
 *
 * [EOF]
 */
