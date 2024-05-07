/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Nonlinear_controller_w_ail_new_aero_sl.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-May-2024 00:26:53
 */

/* Include Files */
#include "Nonlinear_controller_w_ail_new_aero_sl.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_data.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_initialize.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "compute_acc_nonlinear_control_rf_w_Mx_noah.h"
#include "fmincon.h"
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
 * Arguments    : double m
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
 *                double wing_span
 *                double flight_path_angle
 *                double max_alpha
 *                double min_alpha
 *                double max_airspeed
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
 *                double vert_acc_margin
 *                const double current_accelerations_filtered[6]
 *                double power_Cd_0
 *                double power_Cd_a
 *                double prop_R
 *                double prop_Cd_0
 *                double prop_Cl_0
 *                double prop_Cd_a
 *                double prop_Cl_a
 *                double prop_delta
 *                double prop_sigma
 *                double prop_theta
 *                double u_out[15]
 *                double residuals[6]
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluation
 *                double *exitflag
 * Return Type  : void
 */
void Nonlinear_controller_w_ail_new_aero_sl(
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
    const double dv[6], double p, double q, double r, double Cm_zero,
    double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha,
    double CL_aileron, double rho, double V, double S, double wing_chord,
    double wing_span, double flight_path_angle, double max_alpha,
    double min_alpha, double max_airspeed, double Beta,
    double gamma_quadratic_du, double desired_motor_value,
    double desired_el_value, double desired_az_value,
    double desired_theta_value, double desired_phi_value,
    double desired_ailerons_value, double k_alt_tilt_constraint,
    double min_alt_tilt_constraint, double lidar_alt_corrected,
    double approach_mode, double verbose, double aoa_protection_speed,
    double transition_speed, double vert_acc_margin,
    const double current_accelerations_filtered[6], double power_Cd_0,
    double power_Cd_a, double prop_R, double prop_Cd_0, double prop_Cl_0,
    double prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma,
    double prop_theta, double u_out[15], double residuals[6],
    double *elapsed_time, double *N_iterations, double *N_evaluation,
    double *exitflag)
{
  b_captured_var dv_global;
  c_struct_T b_expl_temp;
  c_struct_T expl_temp;
  captured_var Omega_1_scaled;
  captured_var Omega_2_scaled;
  captured_var V_scaled;
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
  captured_var b_power_Cd_0;
  captured_var b_power_Cd_a;
  captured_var b_prop_Cd_0;
  captured_var b_prop_Cd_a;
  captured_var b_prop_Cl_0;
  captured_var b_prop_Cl_a;
  captured_var b_prop_R;
  captured_var b_prop_delta;
  captured_var b_prop_sigma;
  captured_var b_prop_theta;
  captured_var b_q;
  captured_var b_r;
  captured_var b_rho;
  captured_var b_wing_chord;
  captured_var b_wing_span;
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
  double b_min_approach;
  double g_max_approach;
  double g_min_approach;
  double max_theta_protection;
  double max_tilt_value_approach;
  double min_theta_protection;
  int i;
  char c_expl_temp[3];
  if (!isInitialized_Nonlinear_controller_w_ail_new_aero_sl) {
    Nonlinear_controller_w_ail_new_aero_sl_initialize();
  }
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
  b_wing_span.contents = wing_span;
  b_flight_path_angle.contents = flight_path_angle;
  b_Beta.contents = Beta;
  b_gamma_quadratic_du.contents = gamma_quadratic_du;
  b_desired_motor_value.contents = desired_motor_value;
  b_desired_el_value.contents = desired_el_value;
  b_desired_az_value.contents = desired_az_value;
  b_desired_theta_value.contents = desired_theta_value;
  b_desired_phi_value.contents = desired_phi_value;
  b_desired_ailerons_value.contents = desired_ailerons_value;
  b_power_Cd_0.contents = power_Cd_0;
  b_power_Cd_a.contents = power_Cd_a;
  b_prop_R.contents = prop_R;
  b_prop_Cd_0.contents = prop_Cd_0;
  b_prop_Cl_0.contents = prop_Cl_0;
  b_prop_Cd_a.contents = prop_Cd_a;
  b_prop_Cl_a.contents = prop_Cl_a;
  b_prop_delta.contents = prop_delta;
  b_prop_sigma.contents = prop_sigma;
  b_prop_theta.contents = prop_theta;
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
  gain_motor.contents = max_omega / 2.0;
  gain_el.contents = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az.contents = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
  gain_theta.contents = (max_theta_protection - min_theta_protection) *
                        3.1415926535897931 / 180.0 / 2.0;
  gain_phi.contents = max_phi * 3.1415926535897931 / 180.0;
  gain_ailerons.contents = (max_delta_ailerons - min_delta_ailerons) *
                           3.1415926535897931 / 180.0 / 2.0;
  max_tilt_value_approach = b_V.contents / max_airspeed;
  V_scaled.contents = max_tilt_value_approach;
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
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = dv[i] + current_accelerations_filtered[i];
  }
  /* Pseudo-control hedging:  */
  if (b_V.contents > transition_speed) {
    max_tilt_value_approach = 0.0;
    if (dv_global.contents[2] >= 9.81 - vert_acc_margin) {
      max_tilt_value_approach =
          dv_global.contents[2] - (9.81 - vert_acc_margin);
    }
    dv_global.contents[2] -= max_tilt_value_approach;
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
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.wing_span = &b_wing_span;
  expl_temp.rho = &b_rho;
  expl_temp.r = &b_r;
  expl_temp.q = &b_q;
  expl_temp.prop_theta = &b_prop_theta;
  expl_temp.prop_sigma = &b_prop_sigma;
  expl_temp.prop_delta = &b_prop_delta;
  expl_temp.prop_Cl_a = &b_prop_Cl_a;
  expl_temp.prop_Cd_a = &b_prop_Cd_a;
  expl_temp.prop_Cl_0 = &b_prop_Cl_0;
  expl_temp.prop_Cd_0 = &b_prop_Cd_0;
  expl_temp.prop_R = &b_prop_R;
  expl_temp.power_Cd_a = &b_power_Cd_a;
  expl_temp.power_Cd_0 = &b_power_Cd_0;
  expl_temp.p = &b_p;
  expl_temp.m = &b_m;
  expl_temp.l_z = &b_l_z;
  expl_temp.l_4 = &b_l_4;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_2 = &b_l_2;
  expl_temp.l_1 = &b_l_1;
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
  expl_temp.W_act_ailerons = &W_act_ailerons;
  expl_temp.W_act_tilt_az = &W_act_tilt_az;
  expl_temp.W_act_tilt_el = &W_act_tilt_el;
  expl_temp.W_dv_6 = &b_W_dv_6;
  expl_temp.W_dv_5 = &b_W_dv_5;
  expl_temp.W_dv_4 = &b_W_dv_4;
  expl_temp.W_dv_3 = &b_W_dv_3;
  expl_temp.W_dv_2 = &b_W_dv_2;
  expl_temp.W_dv_1 = &b_W_dv_1;
  expl_temp.W_act_motor = &W_act_motor;
  expl_temp.W_act_theta = &W_act_theta;
  expl_temp.W_act_phi = &W_act_phi;
  expl_temp.V_scaled = &V_scaled;
  expl_temp.V = &b_V;
  expl_temp.S = &b_S;
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
  expl_temp.Omega_2_scaled = &Omega_2_scaled;
  expl_temp.Omega_1_scaled = &Omega_1_scaled;
  b_expl_temp = expl_temp;
  fmincon(&b_expl_temp, u_out, u_min, u_max, exitflag, N_iterations,
          N_evaluation, c_expl_temp, &b_max_approach, &b_min_approach,
          &g_max_approach, &g_min_approach);
  *elapsed_time = toc();
  Omega_1_scaled.contents = u_out[0];
  Omega_2_scaled.contents = u_out[1];
  b_min_approach = gain_motor.contents;
  max_tilt_value_approach = gain_el.contents;
  b_max_approach = gain_az.contents;
  u_out[0] *= b_min_approach;
  u_out[4] *= max_tilt_value_approach;
  u_out[8] *= b_max_approach;
  u_out[1] *= b_min_approach;
  u_out[5] *= max_tilt_value_approach;
  u_out[9] *= b_max_approach;
  u_out[2] *= b_min_approach;
  u_out[6] *= max_tilt_value_approach;
  u_out[10] *= b_max_approach;
  u_out[3] *= b_min_approach;
  u_out[7] *= max_tilt_value_approach;
  u_out[11] *= b_max_approach;
  u_out[12] *= gain_theta.contents;
  u_out[13] *= gain_phi.contents;
  u_out[14] *= gain_ailerons.contents;
  c_compute_acc_nonlinear_control(
      b_Beta.contents, b_CL_aileron.contents, b_Cd_zero.contents,
      b_Cl_alpha.contents, b_Cm_zero.contents, b_Cm_alpha.contents,
      b_I_xx.contents, b_I_yy.contents, b_I_zz.contents, b_K_Cd.contents,
      u_out[0], u_out[1], u_out[2], u_out[3], Omega_1_scaled.contents,
      Omega_2_scaled.contents, u_out[13], b_S.contents, u_out[12], b_V.contents,
      V_scaled.contents, u_out[4], u_out[5], u_out[6], u_out[7], u_out[14],
      b_flight_path_angle.contents, u_out[8], u_out[9], u_out[10], u_out[11],
      b_l_1.contents, b_l_2.contents, b_l_3.contents, b_l_4.contents,
      b_l_z.contents, b_m.contents, b_p.contents, b_prop_R.contents,
      b_prop_Cd_0.contents, b_prop_Cl_0.contents, b_prop_Cd_a.contents,
      b_prop_Cl_a.contents, b_prop_delta.contents, b_prop_sigma.contents,
      b_prop_theta.contents, b_q.contents, b_r.contents, b_rho.contents,
      b_wing_span.contents, b_wing_chord.contents, final_accelerations);
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
      printf(" %f ", current_accelerations_filtered[i]);
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
    printf(" %f \n", c_norm(residuals));
    fflush(stdout);
  }
}

/*
 * File trailer for Nonlinear_controller_w_ail_new_aero_sl.c
 *
 * [EOF]
 */
