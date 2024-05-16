/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_cost_and_gradient_basic_sl.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

/* Include Files */
#include "compute_cost_and_gradient_basic_sl.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * COMPUTE_COST_AND_GRADIENT_BASIC_SL
 *     [COST,GRADIENT] =
 * COMPUTE_COST_AND_GRADIENT_BASIC_SL(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD)
 *
 * Arguments    : double Beta
 *                double CL_aileron
 *                double Cd_zero
 *                double Cl_alpha
 *                double Cm_zero
 *                double Cm_alpha
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double K_Cd
 *                double K_p_M
 *                double K_p_T
 *                double Omega_1_scaled
 *                double Omega_2_scaled
 *                double Omega_3_scaled
 *                double Omega_4_scaled
 *                double Phi_scaled
 *                double S
 *                double Theta_scaled
 *                double V
 *                double W_act_phi
 *                double W_act_theta
 *                double W_act_motor
 *                double W_dv_1
 *                double W_dv_2
 *                double W_dv_3
 *                double W_dv_4
 *                double W_dv_5
 *                double W_dv_6
 *                double W_act_tilt_el
 *                double W_act_tilt_az
 *                double W_act_ailerons
 *                double b_1_scaled
 *                double b_2_scaled
 *                double b_3_scaled
 *                double b_4_scaled
 *                double delta_ailerons_scaled
 *                double desired_el_value
 *                double desired_az_value
 *                double desired_phi_value
 *                double desired_theta_value
 *                double desired_motor_value
 *                double desired_ailerons_value
 *                double dv_global_1
 *                double dv_global_2
 *                double dv_global_3
 *                double dv_global_4
 *                double dv_global_5
 *                double dv_global_6
 *                double flight_path_angle
 *                double g_1_scaled
 *                double g_2_scaled
 *                double g_3_scaled
 *                double g_4_scaled
 *                double gain_el
 *                double gain_az
 *                double gain_phi
 *                double gain_theta
 *                double gain_motor
 *                double gain_ailerons
 *                double gamma_quadratic_du
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double m
 *                double p
 *                double q
 *                double r
 *                double rho
 *                double wing_chord
 *                double gradient[15]
 * Return Type  : double
 */
double c_compute_cost_and_gradient_bas(
    double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
    double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz,
    double K_Cd, double K_p_M, double K_p_T, double Omega_1_scaled,
    double Omega_2_scaled, double Omega_3_scaled, double Omega_4_scaled,
    double Phi_scaled, double S, double Theta_scaled, double V,
    double W_act_phi, double W_act_theta, double W_act_motor, double W_dv_1,
    double W_dv_2, double W_dv_3, double W_dv_4, double W_dv_5, double W_dv_6,
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
    double l_3, double l_4, double l_z, double m, double p, double q, double r,
    double rho, double wing_chord, double gradient[15])
{
  double a;
  double a_tmp;
  double b_a;
  double b_a_tmp;
  double b_gradient_tmp;
  double b_t225_tmp;
  double c_a;
  double c_a_tmp;
  double c_gradient_tmp;
  double c_t225_tmp;
  double cost;
  double d_a;
  double gradient_tmp;
  double t10;
  double t102;
  double t103;
  double t104;
  double t105;
  double t106;
  double t107;
  double t108;
  double t109;
  double t11;
  double t115;
  double t116;
  double t117;
  double t118;
  double t119;
  double t12;
  double t120;
  double t121;
  double t122;
  double t123;
  double t124;
  double t125;
  double t126;
  double t127;
  double t128;
  double t129;
  double t13;
  double t130;
  double t131;
  double t133;
  double t135;
  double t137;
  double t14;
  double t140;
  double t141;
  double t143;
  double t15;
  double t155;
  double t156;
  double t16;
  double t169;
  double t17;
  double t172;
  double t173;
  double t18;
  double t180;
  double t181;
  double t182;
  double t184;
  double t185;
  double t186;
  double t188;
  double t19;
  double t192;
  double t194;
  double t195;
  double t197;
  double t199;
  double t199_tmp;
  double t2;
  double t20;
  double t200;
  double t200_tmp;
  double t203;
  double t203_tmp_tmp;
  double t204;
  double t209;
  double t21;
  double t212;
  double t212_tmp;
  double t216;
  double t22;
  double t220;
  double t225;
  double t225_tmp;
  double t226;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t32;
  double t33;
  double t34;
  double t37;
  double t4;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t53;
  double t54;
  double t55;
  double t56;
  double t57;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t69;
  double t7;
  double t72;
  double t73;
  double t74;
  double t8;
  double t80;
  double t84;
  double t86;
  double t88;
  double t9;
  double t92;
  double t93;
  double t94;
  double t95;
  double t96;
  double t97;
  double t98;
  double t99;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     15-May-2024 17:38:05 */
  t2 = cos(Beta);
  t3 = sin(Beta);
  t4 = Phi_scaled * gain_phi;
  t5 = Theta_scaled * gain_theta;
  t6 = b_1_scaled * gain_el;
  t7 = b_2_scaled * gain_el;
  t8 = b_3_scaled * gain_el;
  t9 = b_4_scaled * gain_el;
  t10 = g_1_scaled * gain_az;
  t11 = g_2_scaled * gain_az;
  t12 = g_3_scaled * gain_az;
  t13 = g_4_scaled * gain_az;
  t14 = Cl_alpha * Cl_alpha;
  t15 = Omega_1_scaled * Omega_1_scaled;
  t16 = Omega_2_scaled * Omega_2_scaled;
  t17 = Omega_3_scaled * Omega_3_scaled;
  t18 = Omega_4_scaled * Omega_4_scaled;
  t19 = V * V;
  t20 = W_act_phi * W_act_phi;
  t21 = W_act_theta * W_act_theta;
  t22 = W_act_motor * W_act_motor;
  t23 = W_dv_1 * W_dv_1;
  t24 = W_dv_2 * W_dv_2;
  t25 = W_dv_3 * W_dv_3;
  t26 = W_dv_4 * W_dv_4;
  t27 = W_dv_5 * W_dv_5;
  t28 = W_dv_6 * W_dv_6;
  t29 = W_act_tilt_el * W_act_tilt_el;
  t30 = W_act_tilt_az * W_act_tilt_az;
  t31 = W_act_ailerons * W_act_ailerons;
  t32 = gain_motor * gain_motor;
  t59 = 1.0 / I_xx;
  t60 = 1.0 / I_yy;
  t61 = 1.0 / I_zz;
  t69 = 1.0 / m;
  t33 = cos(t4);
  t34 = cos(t5);
  t37 = sin(t4);
  t42 = cos(t6);
  t43 = cos(t7);
  t44 = cos(t8);
  t45 = cos(t9);
  t46 = sin(t5);
  t47 = cos(t10);
  t48 = cos(t11);
  t49 = cos(t12);
  t50 = cos(t13);
  t51 = sin(t6);
  t52 = sin(t7);
  t53 = sin(t8);
  t54 = sin(t9);
  t55 = sin(t10);
  t56 = sin(t11);
  t57 = sin(t12);
  t58 = sin(t13);
  t72 = desired_el_value * (1.0 / gain_el);
  t73 = desired_az_value * (1.0 / gain_az);
  t74 = desired_motor_value * (1.0 / gain_motor);
  t80 = flight_path_angle - t5;
  t4 = K_p_M * t15 * t32;
  t92 = t4 * t51;
  t6 = K_p_M * t16 * t32;
  t93 = t6 * t52;
  t7 = K_p_M * t17 * t32;
  t94 = t7 * t53;
  t9 = K_p_M * t18 * t32;
  t95 = t9 * t54;
  t8 = K_p_T * t15 * t32;
  t96 = t8 * t51;
  t10 = K_p_T * t16 * t32;
  t97 = t10 * t52;
  t11 = K_p_T * t17 * t32;
  t98 = t11 * t53;
  t12 = K_p_T * t18 * t32;
  t99 = t12 * t54;
  t4 *= t42;
  t116 = t4 * t47;
  t6 *= t43;
  t117 = t6 * t48;
  t7 *= t44;
  t118 = t7 * t49;
  t8 *= t42;
  t119 = t8 * t47;
  t9 *= t45;
  t120 = t9 * t50;
  t10 *= t43;
  t121 = t10 * t48;
  t11 *= t44;
  t122 = t11 * t49;
  t12 *= t45;
  t123 = t12 * t50;
  t124 = t4 * t55;
  t125 = t6 * t56;
  t126 = t7 * t57;
  t127 = t8 * t55;
  t128 = t9 * t58;
  t129 = t10 * t56;
  t130 = t11 * t57;
  t131 = t12 * t58;
  t9 = cos(t80);
  t10 = sin(t80);
  t84 = -(t72 * 2.0);
  t86 = -(t73 * 2.0);
  t88 = -(t74 * 2.0);
  t102 = l_1 * t96;
  t103 = l_1 * t97;
  t104 = l_2 * t98;
  t105 = l_2 * t99;
  t106 = l_z * t96;
  t107 = l_z * t97;
  t108 = l_z * t98;
  t109 = l_z * t99;
  t133 = l_4 * t119;
  t135 = l_4 * t121;
  t137 = l_3 * t122;
  t140 = l_4 * t127;
  t141 = l_4 * t129;
  t143 = l_3 * t131;
  t4 = ((t96 + t97) + t98) + t99;
  t184 = ((t119 + t121) + t122) + t123;
  t185 = ((t127 + t129) + t130) + t131;
  t155 = -(l_3 * t123);
  t156 = -(l_3 * t130);
  t7 = Cl_alpha * S;
  t11 = t7 * gain_theta * rho * t19;
  t12 = t11 * t9 / 2.0;
  t180 = t34 * t4;
  t181 = t46 * t4;
  t194 = t37 * t184;
  t195 = t33 * t185;
  t197 = t34 * t37 * t185;
  t199_tmp = t33 * t34;
  t199 = t199_tmp * t184;
  t200_tmp = t33 * t46;
  t200 = t200_tmp * t184;
  t115 = Cd_zero + K_Cd * t14 * (t80 * t80);
  t13 = S * rho;
  t4 = t13 * t3 * t19;
  t169 = t4 * t33 * t115 / 2.0;
  t6 = t13 * t2 * t19;
  t8 = t6 * t10 * t115 / 2.0;
  t172 = t4 * t34 * t37 * t115 / 2.0;
  t173 = t4 * t37 * t46 * t115 / 2.0;
  t6 = t6 * t9 * t115 / 2.0;
  t4 = t7 * rho * t19 * t80;
  t182 = t4 * t9 / 2.0 + t8;
  t7 = t4 * t10 / 2.0 - t6;
  t186 = t37 * t182;
  t192 = t199_tmp * t182;
  t203_tmp_tmp = K_Cd * S * gain_theta * rho;
  t4 = t203_tmp_tmp * t2 * t14 * t19 * t80;
  t203 = ((t12 + t11 * t80 * t10 * -0.5) + t4 * t10) + gain_theta * t6;
  t204 = ((t11 * t10 / 2.0 + t80 * t12) - t4 * t9) + gain_theta * t8;
  t4 = I_xx * p;
  t209 =
      dv_global_6 -
      t61 *
          (((((((((((((t4 * q - I_yy * p * q) + t103) + t104) - t102) - t105) +
                  t116) +
                 t118) +
                t140) +
               t141) -
              t117) -
             t120) +
            t156) -
           t143);
  t212_tmp = CL_aileron * S;
  t212 = dv_global_4 -
         t59 * ((((((((((((((I_yy * q * r - I_zz * q * r) + t92) + t94) +
                          t212_tmp * delta_ailerons_scaled * gain_ailerons *
                              rho * t19 / 2.0) -
                         t93) -
                        t95) +
                       l_1 * t119) +
                      l_2 * t123) +
                     l_z * t127) +
                    l_z * t129) +
                   l_z * t130) +
                  l_z * t131) -
                 l_1 * t121) -
                l_2 * t122);
  t188 = t46 * t7;
  t216 =
      dv_global_5 -
      t60 *
          ((((((((((((((I_zz * p * r - t4 * r) + t106) + t107) + t108) + t109) +
                   t125) +
                  t128) +
                 t133) +
                t135) -
               t124) -
              t126) -
             t137) +
            t155) +
           t13 * t19 * (Cm_zero - Cm_alpha * t80) * wing_chord / 2.0);
  t220 = dv_global_2 + t69 * (((t169 + t186) - t194) - t195);
  t226 = (-dv_global_3 +
          -t69 * (((((t172 - t181) + t188) - t192) - t197) + t199)) +
         9.81;
  t225_tmp = t34 * t7;
  b_t225_tmp = t200_tmp * t182;
  c_t225_tmp = t37 * t46 * t185;
  t225 =
      dv_global_1 +
      t69 * (((((t173 + t180) - t225_tmp) - b_t225_tmp) + t200) - c_t225_tmp);
  a_tmp = desired_phi_value * (1.0 / gain_phi);
  t5 = Phi_scaled - a_tmp;
  b_a_tmp = desired_theta_value * (1.0 / gain_theta);
  t2 = Theta_scaled - b_a_tmp;
  c_a_tmp = desired_ailerons_value * (1.0 / gain_ailerons);
  a = delta_ailerons_scaled - c_a_tmp;
  b_a = Omega_1_scaled - t74;
  c_a = Omega_2_scaled - t74;
  d_a = Omega_3_scaled - t74;
  t10 = Omega_4_scaled - t74;
  t11 = b_1_scaled - t72;
  t12 = b_2_scaled - t72;
  t13 = b_3_scaled - t72;
  t6 = b_4_scaled - t72;
  t7 = g_1_scaled - t73;
  t8 = g_2_scaled - t73;
  t9 = g_3_scaled - t73;
  t4 = g_4_scaled - t73;
  cost = (((((gamma_quadratic_du *
                  ((((((((((((((t20 * (t5 * t5) + t21 * (t2 * t2)) +
                               t31 * (a * a)) +
                              t22 * (b_a * b_a)) +
                             t22 * (c_a * c_a)) +
                            t22 * (d_a * d_a)) +
                           t22 * (t10 * t10)) +
                          t29 * (t11 * t11)) +
                         t29 * (t12 * t12)) +
                        t29 * (t13 * t13)) +
                       t29 * (t6 * t6)) +
                      t30 * (t7 * t7)) +
                     t30 * (t8 * t8)) +
                    t30 * (t9 * t9)) +
                   t30 * (t4 * t4)) +
              t28 * (t209 * t209)) +
             t26 * (t212 * t212)) +
            t27 * (t216 * t216)) +
           t24 * (t220 * t220)) +
          t23 * (t225 * t225)) +
         t25 * (t226 * t226);
  a = K_p_T * Omega_1_scaled;
  b_a = K_p_M * Omega_1_scaled * t32;
  c_a = a * l_1 * t32;
  d_a = a * l_z * t32;
  t74 = b_a * t42;
  t72 = a * l_4 * t32 * t42;
  a *= t32;
  t73 = a * t33;
  gradient_tmp = a * t34;
  b_gradient_tmp = t73 * t42;
  c_gradient_tmp = a * t37 * t42;
  t2 = gamma_quadratic_du * t22;
  t12 = t28 * t61 * t209;
  t13 = t26 * t59 * t212;
  t5 = t27 * t60 * t216;
  t11 = t24 * t69 * t220;
  t10 = t25 * t69 * t226;
  t9 = t23 * t69 * t225;
  gradient[0] =
      (((((t2 * (Omega_1_scaled * 2.0 + t88) -
           t12 * ((c_a * t51 * -2.0 + t74 * t47 * 2.0) + t72 * t55 * 2.0) *
               2.0) -
          t13 *
              ((b_a * t51 * 2.0 + c_a * t42 * t47 * 2.0) +
               d_a * t42 * t55 * 2.0) *
              2.0) -
         t5 * ((d_a * t51 * 2.0 - t74 * t55 * 2.0) + t72 * t47 * 2.0) * 2.0) -
        t11 * (c_gradient_tmp * t47 * 2.0 + b_gradient_tmp * t55 * 2.0) * 2.0) +
       t10 *
           ((a * t46 * t51 * 2.0 - t73 * t34 * t42 * t47 * 2.0) +
            gradient_tmp * t37 * t42 * t55 * 2.0) *
           2.0) +
      t9 *
          ((gradient_tmp * t51 * 2.0 + b_gradient_tmp * t46 * t47 * 2.0) -
           c_gradient_tmp * t46 * t55 * 2.0) *
          2.0;
  a = K_p_T * Omega_2_scaled;
  b_a = K_p_M * Omega_2_scaled * t32;
  c_a = a * l_1 * t32;
  d_a = a * l_z * t32;
  t74 = b_a * t43;
  t72 = a * l_4 * t32 * t43;
  a *= t32;
  t73 = a * t33;
  gradient_tmp = a * t34;
  b_gradient_tmp = t73 * t43;
  c_gradient_tmp = a * t37 * t43;
  gradient[1] =
      (((((t2 * (Omega_2_scaled * 2.0 + t88) -
           t12 * ((c_a * t52 * 2.0 - t74 * t48 * 2.0) + t72 * t56 * 2.0) *
               2.0) +
          t13 *
              ((b_a * t52 * 2.0 + c_a * t43 * t48 * 2.0) -
               d_a * t43 * t56 * 2.0) *
              2.0) -
         t5 * ((d_a * t52 * 2.0 + t74 * t56 * 2.0) + t72 * t48 * 2.0) * 2.0) -
        t11 * (c_gradient_tmp * t48 * 2.0 + b_gradient_tmp * t56 * 2.0) * 2.0) +
       t10 *
           ((a * t46 * t52 * 2.0 - t73 * t34 * t43 * t48 * 2.0) +
            gradient_tmp * t37 * t43 * t56 * 2.0) *
           2.0) +
      t9 *
          ((gradient_tmp * t52 * 2.0 + b_gradient_tmp * t46 * t48 * 2.0) -
           c_gradient_tmp * t46 * t56 * 2.0) *
          2.0;
  a = K_p_T * Omega_3_scaled;
  b_a = K_p_M * Omega_3_scaled * t32;
  c_a = a * l_2 * t32;
  d_a = a * l_z * t32;
  t74 = b_a * t44;
  t72 = a * l_3 * t32 * t44;
  a *= t32;
  t73 = a * t33;
  gradient_tmp = a * t34;
  b_gradient_tmp = t73 * t44;
  c_gradient_tmp = a * t37 * t44;
  gradient[2] =
      (((((t2 * (Omega_3_scaled * 2.0 + t88) -
           t12 * ((c_a * t53 * 2.0 + t74 * t49 * 2.0) - t72 * t57 * 2.0) *
               2.0) -
          t13 *
              ((b_a * t53 * 2.0 - c_a * t44 * t49 * 2.0) +
               d_a * t44 * t57 * 2.0) *
              2.0) +
         t5 * ((d_a * t53 * -2.0 + t74 * t57 * 2.0) + t72 * t49 * 2.0) * 2.0) -
        t11 * (c_gradient_tmp * t49 * 2.0 + b_gradient_tmp * t57 * 2.0) * 2.0) +
       t10 *
           ((a * t46 * t53 * 2.0 - t73 * t34 * t44 * t49 * 2.0) +
            gradient_tmp * t37 * t44 * t57 * 2.0) *
           2.0) +
      t9 *
          ((gradient_tmp * t53 * 2.0 + b_gradient_tmp * t46 * t49 * 2.0) -
           c_gradient_tmp * t46 * t57 * 2.0) *
          2.0;
  a = K_p_T * Omega_4_scaled;
  b_a = K_p_M * Omega_4_scaled * t32;
  c_a = a * l_2 * t32;
  d_a = a * l_z * t32;
  t74 = b_a * t45;
  t72 = a * l_3 * t32 * t45;
  a *= t32;
  t73 = a * t33;
  gradient_tmp = a * t34;
  b_gradient_tmp = t73 * t45;
  c_gradient_tmp = a * t37 * t45;
  gradient[3] =
      (((((t2 * (Omega_4_scaled * 2.0 + t88) +
           t12 * ((c_a * t54 * 2.0 + t74 * t50 * 2.0) + t72 * t58 * 2.0) *
               2.0) -
          t13 *
              ((b_a * t54 * -2.0 + c_a * t45 * t50 * 2.0) +
               d_a * t45 * t58 * 2.0) *
              2.0) -
         t5 * ((d_a * t54 * 2.0 + t74 * t58 * 2.0) - t72 * t50 * 2.0) * 2.0) -
        t11 * (c_gradient_tmp * t50 * 2.0 + b_gradient_tmp * t58 * 2.0) * 2.0) +
       t10 *
           ((a * t46 * t54 * 2.0 - t73 * t34 * t45 * t50 * 2.0) +
            gradient_tmp * t37 * t45 * t58 * 2.0) *
           2.0) +
      t9 *
          ((gradient_tmp * t54 * 2.0 + b_gradient_tmp * t46 * t50 * 2.0) -
           c_gradient_tmp * t46 * t58 * 2.0) *
          2.0;
  t8 = K_p_T * gain_el;
  a = t8 * t15 * t32;
  b_a = gain_el * t37;
  c_a = gain_el * t33;
  d_a = gain_el * l_4;
  t74 = gain_el * t47;
  t72 = gain_el * t55;
  t73 = gamma_quadratic_du * t29;
  gradient_tmp = c_a * t34;
  b_gradient_tmp = gain_el * t34 * t37;
  c_gradient_tmp = -gain_el * t33 * t46;
  t2 = b_a * t46;
  t4 = t8 * l_1;
  t6 = t8 * l_z;
  t7 = K_p_M * gain_el;
  gradient[4] =
      (((((t73 * (b_1_scaled * 2.0 + t84) +
           t10 *
               ((gradient_tmp * t47 * t96 - b_gradient_tmp * t55 * t96) +
                a * t42 * t46) *
               2.0) +
          t9 * ((c_gradient_tmp * t47 * t96 + t2 * t55 * t96) + a * t34 * t42) *
              2.0) +
         t11 * (b_a * t47 * t96 + c_a * t55 * t96) * 2.0) +
        t12 * ((t74 * t92 + d_a * t55 * t96) + t4 * t15 * t32 * t42) * 2.0) -
       t5 * ((t72 * t92 - d_a * t47 * t96) + t6 * t15 * t32 * t42) * 2.0) +
      t13 * ((t74 * t102 + t72 * t106) - t7 * t15 * t32 * t42) * 2.0;
  a = t8 * t16 * t32;
  t74 = gain_el * t56;
  gradient[5] =
      (((((t73 * (b_2_scaled * 2.0 + t84) +
           t10 *
               ((gradient_tmp * t48 * t97 - b_gradient_tmp * t56 * t97) +
                a * t43 * t46) *
               2.0) +
          t9 * ((c_gradient_tmp * t48 * t97 + t2 * t56 * t97) + a * t34 * t43) *
              2.0) +
         t11 * (b_a * t48 * t97 + c_a * t56 * t97) * 2.0) -
        t12 * ((gain_el * t48 * t93 - d_a * t56 * t97) + t4 * t16 * t32 * t43) *
            2.0) +
       t5 * ((t74 * t93 + d_a * t48 * t97) - t6 * t16 * t32 * t43) * 2.0) +
      t13 * ((-gain_el * t48 * t103 + t74 * t107) + t7 * t16 * t32 * t43) * 2.0;
  a = t8 * t17 * t32;
  d_a = gain_el * l_3;
  t74 = gain_el * t57;
  t72 = t8 * l_2;
  gradient[6] =
      (((((t73 * (b_3_scaled * 2.0 + t84) +
           t10 *
               ((gradient_tmp * t49 * t98 - b_gradient_tmp * t57 * t98) +
                a * t44 * t46) *
               2.0) +
          t9 * ((c_gradient_tmp * t49 * t98 + t2 * t57 * t98) + a * t34 * t44) *
              2.0) +
         t11 * (b_a * t49 * t98 + c_a * t57 * t98) * 2.0) -
        t12 *
            ((-gain_el * t49 * t94 + d_a * t57 * t98) + t72 * t17 * t32 * t44) *
            2.0) -
       t5 * ((t74 * t94 + d_a * t49 * t98) + t6 * t17 * t32 * t44) * 2.0) -
      t13 * ((gain_el * t49 * t104 - t74 * t108) + t7 * t17 * t32 * t44) * 2.0;
  a = t8 * t18 * t32;
  t74 = gain_el * t50;
  t4 = gain_el * t58;
  gradient[7] =
      (((((t73 * (b_4_scaled * 2.0 + t84) +
           t10 *
               ((gradient_tmp * t50 * t99 - b_gradient_tmp * t58 * t99) +
                a * t45 * t46) *
               2.0) +
          t9 * ((c_gradient_tmp * t50 * t99 + t2 * t58 * t99) + a * t34 * t45) *
              2.0) +
         t11 * (b_a * t50 * t99 + c_a * t58 * t99) * 2.0) -
        t12 * ((t74 * t95 + d_a * t58 * t99) - t72 * t18 * t32 * t45) * 2.0) -
       t5 * ((t4 * -t95 + d_a * t50 * t99) + t6 * t18 * t32 * t45) * 2.0) +
      t13 * ((t74 * t105 + t4 * t109) + t7 * t18 * t32 * t45) * 2.0;
  a = gain_az * t33;
  b_a = gain_az * t37;
  c_a = gamma_quadratic_du * t30;
  d_a = gain_az * t34 * t37;
  t74 = a * t34;
  t72 = b_a * t46;
  t73 = a * t46;
  gradient_tmp = gain_az * l_1;
  b_gradient_tmp = gain_az * l_z;
  gradient[8] = (((((c_a * (g_1_scaled * 2.0 + t86) +
                     t10 * (d_a * t119 + t74 * t127) * 2.0) -
                    t9 * (t72 * t119 + t73 * t127) * 2.0) +
                   t13 * (gradient_tmp * t127 - b_gradient_tmp * t119) * 2.0) -
                  t11 * (a * t119 - b_a * t127) * 2.0) +
                 t12 * (gain_az * t124 - gain_az * t133) * 2.0) +
                t5 * (gain_az * t116 + gain_az * t140) * 2.0;
  gradient[9] = (((((c_a * (g_2_scaled * 2.0 + t86) +
                     t10 * (d_a * t121 + t74 * t129) * 2.0) -
                    t9 * (t72 * t121 + t73 * t129) * 2.0) -
                   t13 * (gradient_tmp * t129 + b_gradient_tmp * t121) * 2.0) -
                  t11 * (a * t121 - b_a * t129) * 2.0) -
                 t12 * (gain_az * t125 + gain_az * t135) * 2.0) -
                t5 * (gain_az * t117 - gain_az * t141) * 2.0;
  gradient_tmp = gain_az * l_2;
  gradient[10] = (((((c_a * (g_3_scaled * 2.0 + t86) +
                      t10 * (d_a * t122 + t74 * t130) * 2.0) -
                     t9 * (t72 * t122 + t73 * t130) * 2.0) -
                    t13 * (gradient_tmp * t130 + b_gradient_tmp * t122) * 2.0) -
                   t11 * (a * t122 - b_a * t130) * 2.0) +
                  t12 * (gain_az * t126 + gain_az * t137) * 2.0) +
                 t5 * (gain_az * t118 + gain_az * t156) * 2.0;
  gradient[11] = (((((c_a * (g_4_scaled * 2.0 + t86) +
                      t10 * (d_a * t123 + t74 * t131) * 2.0) -
                     t9 * (t72 * t123 + t73 * t131) * 2.0) +
                    t13 * (gradient_tmp * t131 - b_gradient_tmp * t123) * 2.0) -
                   t11 * (a * t123 - b_a * t131) * 2.0) -
                  t5 * (gain_az * t120 + gain_az * t143) * 2.0) -
                 t12 * (gain_az * t128 + gain_az * t155) * 2.0;
  a = t203_tmp_tmp * t3 * t14 * t19;
  gradient[12] =
      (((gamma_quadratic_du * t21 * (Theta_scaled * 2.0 - b_a_tmp * 2.0) +
         t10 *
             ((((((((gain_theta * t173 + gain_theta * t180) +
                    gain_theta * -t225_tmp) +
                   gain_theta * -b_t225_tmp) +
                  gain_theta * t200) +
                 gain_theta * -c_t225_tmp) +
                t46 * t204) -
               t199_tmp * t203) +
              a * t34 * t37 * t80) *
             2.0) +
        t9 *
            ((((((((gain_theta * t172 - gain_theta * t181) +
                   gain_theta * t188) -
                  gain_theta * t192) -
                 gain_theta * t197) +
                gain_theta * t199) +
               t34 * t204) +
              t200_tmp * t203) -
             a * t37 * t46 * t80) *
            2.0) -
       t11 * (t37 * t203 + a * t33 * t80) * 2.0) -
      Cm_alpha * S * gain_theta * rho * t19 * t27 * t60 * t216 * wing_chord;
  a = gain_phi * t33;
  b_a = gain_phi * t34;
  c_a = gain_phi * t46;
  gradient[13] =
      ((gamma_quadratic_du * t20 * (Phi_scaled * 2.0 - a_tmp * 2.0) +
        t11 *
            (((a * t182 - a * t184) + gain_phi * t37 * t185) -
             S * gain_phi * rho * t3 * t19 * t37 * t115 / 2.0) *
            2.0) -
       t10 * (((b_a * t169 + b_a * t186) - b_a * t194) - b_a * t195) * 2.0) +
      t9 * (((c_a * t169 + c_a * t186) - c_a * t194) - c_a * t195) * 2.0;
  gradient[14] =
      gamma_quadratic_du * t31 * (delta_ailerons_scaled * 2.0 - c_a_tmp * 2.0) -
      t212_tmp * gain_ailerons * rho * t19 * t26 * t59 * t212;
  return cost;
}

/*
 * File trailer for compute_cost_and_gradient_basic_sl.c
 *
 * [EOF]
 */
