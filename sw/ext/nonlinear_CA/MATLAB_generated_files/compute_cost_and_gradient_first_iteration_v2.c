/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_cost_and_gradient_first_iteration_v2.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 23:30:20
 */

/* Include Files */
#include "compute_cost_and_gradient_first_iteration_v2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2
 *     [COST,GRADIENT] =
 * COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_act_phi_du,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_theta_du,W_act_ailerons,W_act_motor_du,W_act_tilt_el_du,W_act_tilt_az_du,W_act_ailerons_du,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,GAMMA_QUADRATIC_DU2,L_1,L_2,L_3,L_4,L_Z,M,P,PREVIOUS_EL_VALUE1,PREVIOUS_EL_VALUE2,PREVIOUS_EL_VALUE3,PREVIOUS_EL_VALUE4,PREVIOUS_AZ_VALUE1,PREVIOUS_AZ_VALUE2,PREVIOUS_AZ_VALUE3,PREVIOUS_AZ_VALUE4,PREVIOUS_PHI_VALUE,PREVIOUS_THETA_VALUE,PREVIOUS_MOTOR_VALUE1,PREVIOUS_MOTOR_VALUE2,PREVIOUS_MOTOR_VALUE3,PREVIOUS_MOTOR_VALUE4,PREVIOUS_AILERONS_VALUE,Q,R,RHO,WING_CHORD)
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
 *                double W_act_phi_du
 *                double W_dv_1
 *                double W_dv_2
 *                double W_dv_3
 *                double W_dv_4
 *                double W_dv_5
 *                double W_dv_6
 *                double W_act_tilt_el
 *                double W_act_tilt_az
 *                double W_act_theta_du
 *                double W_act_ailerons
 *                double W_act_motor_du
 *                double W_act_tilt_el_du
 *                double W_act_tilt_az_du
 *                double W_act_ailerons_du
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
 *                double gamma_quadratic_du2
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double m
 *                double p
 *                double previous_el_value1
 *                double previous_el_value2
 *                double previous_el_value3
 *                double previous_el_value4
 *                double previous_az_value1
 *                double previous_az_value2
 *                double previous_az_value3
 *                double previous_az_value4
 *                double previous_phi_value
 *                double previous_theta_value
 *                double previous_motor_value1
 *                double previous_motor_value2
 *                double previous_motor_value3
 *                double previous_motor_value4
 *                double previous_ailerons_value
 *                double q
 *                double r
 *                double rho
 *                double wing_chord
 *                double gradient[15]
 * Return Type  : double
 */
double c_compute_cost_and_gradient_fir(
    double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
    double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz,
    double K_Cd, double K_p_M, double K_p_T, double Omega_1_scaled,
    double Omega_2_scaled, double Omega_3_scaled, double Omega_4_scaled,
    double Phi_scaled, double S, double Theta_scaled, double V,
    double W_act_phi, double W_act_theta, double W_act_motor,
    double W_act_phi_du, double W_dv_1, double W_dv_2, double W_dv_3,
    double W_dv_4, double W_dv_5, double W_dv_6, double W_act_tilt_el,
    double W_act_tilt_az, double W_act_theta_du, double W_act_ailerons,
    double W_act_motor_du, double W_act_tilt_el_du, double W_act_tilt_az_du,
    double W_act_ailerons_du, double b_1_scaled, double b_2_scaled,
    double b_3_scaled, double b_4_scaled, double delta_ailerons_scaled,
    double desired_el_value, double desired_az_value, double desired_phi_value,
    double desired_theta_value, double desired_motor_value,
    double desired_ailerons_value, double dv_global_1, double dv_global_2,
    double dv_global_3, double dv_global_4, double dv_global_5,
    double dv_global_6, double flight_path_angle, double g_1_scaled,
    double g_2_scaled, double g_3_scaled, double g_4_scaled, double gain_el,
    double gain_az, double gain_phi, double gain_theta, double gain_motor,
    double gain_ailerons, double gamma_quadratic_du, double gamma_quadratic_du2,
    double l_1, double l_2, double l_3, double l_4, double l_z, double m,
    double p, double previous_el_value1, double previous_el_value2,
    double previous_el_value3, double previous_el_value4,
    double previous_az_value1, double previous_az_value2,
    double previous_az_value3, double previous_az_value4,
    double previous_phi_value, double previous_theta_value,
    double previous_motor_value1, double previous_motor_value2,
    double previous_motor_value3, double previous_motor_value4,
    double previous_ailerons_value, double q, double r, double rho,
    double wing_chord, double gradient[15])
{
  double a;
  double a_tmp;
  double b_a;
  double b_a_tmp;
  double b_t246_tmp;
  double c_a;
  double c_a_tmp;
  double c_t246_tmp;
  double cost;
  double d_a;
  double d_a_tmp;
  double e_a;
  double e_a_tmp;
  double f_a;
  double f_a_tmp;
  double g_a;
  double g_a_tmp;
  double h_a;
  double h_a_tmp;
  double i_a;
  double i_a_tmp;
  double j_a;
  double j_a_tmp;
  double k_a;
  double k_a_tmp;
  double l_a;
  double l_a_tmp;
  double m_a;
  double m_a_tmp;
  double n_a;
  double n_a_tmp;
  double o_a;
  double o_a_tmp;
  double p_a_tmp;
  double q_a_tmp;
  double t10;
  double t101;
  double t105;
  double t107;
  double t109;
  double t11;
  double t113;
  double t114;
  double t115;
  double t116;
  double t117;
  double t118;
  double t119;
  double t12;
  double t120;
  double t123;
  double t124;
  double t125;
  double t126;
  double t127;
  double t128;
  double t129;
  double t13;
  double t130;
  double t136;
  double t137;
  double t138;
  double t139;
  double t14;
  double t140;
  double t141;
  double t142;
  double t143;
  double t144;
  double t145;
  double t146;
  double t147;
  double t148;
  double t149;
  double t15;
  double t150;
  double t151;
  double t152;
  double t154;
  double t156;
  double t158;
  double t16;
  double t161;
  double t162;
  double t164;
  double t17;
  double t176;
  double t177;
  double t18;
  double t19;
  double t190;
  double t193;
  double t194;
  double t2;
  double t20;
  double t201;
  double t202;
  double t203;
  double t205;
  double t206;
  double t207;
  double t209;
  double t21;
  double t213;
  double t215;
  double t216;
  double t218;
  double t22;
  double t220;
  double t220_tmp;
  double t221;
  double t221_tmp;
  double t224;
  double t224_tmp_tmp;
  double t225;
  double t23;
  double t230;
  double t233;
  double t233_tmp;
  double t237;
  double t24;
  double t241;
  double t246;
  double t246_tmp;
  double t247;
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
  double t35;
  double t36;
  double t37;
  double t38;
  double t39;
  double t4;
  double t40;
  double t41;
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
  double t58;
  double t6;
  double t63;
  double t64;
  double t65;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t71;
  double t72;
  double t73;
  double t74;
  double t75;
  double t76;
  double t77;
  double t78;
  double t79;
  double t8;
  double t80;
  double t81;
  double t82;
  double t84;
  double t85;
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t90;
  double t93;
  double t94;
  double t95;
  /*     This function was generated by the Symbolic Math Toolbox version 9.3.
   */
  /*     20-Feb-2024 16:03:43 */
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
  t15 = Omega_1_scaled * 2.0;
  t16 = Omega_2_scaled * 2.0;
  t17 = Omega_3_scaled * 2.0;
  t18 = Omega_4_scaled * 2.0;
  t19 = Omega_1_scaled * Omega_1_scaled;
  t20 = Omega_2_scaled * Omega_2_scaled;
  t21 = Omega_3_scaled * Omega_3_scaled;
  t22 = Omega_4_scaled * Omega_4_scaled;
  t23 = Phi_scaled * 2.0;
  t24 = Theta_scaled * 2.0;
  t25 = V * V;
  t26 = W_act_phi * W_act_phi;
  t27 = W_act_theta * W_act_theta;
  t28 = W_act_motor * W_act_motor;
  t29 = W_act_phi_du * W_act_phi_du;
  t30 = W_dv_1 * W_dv_1;
  t31 = W_dv_2 * W_dv_2;
  t32 = W_dv_3 * W_dv_3;
  t33 = W_dv_4 * W_dv_4;
  t34 = W_dv_5 * W_dv_5;
  t35 = W_dv_6 * W_dv_6;
  t36 = W_act_tilt_el * W_act_tilt_el;
  t37 = W_act_tilt_az * W_act_tilt_az;
  t38 = W_act_theta_du * W_act_theta_du;
  t39 = W_act_ailerons * W_act_ailerons;
  t40 = W_act_motor_du * W_act_motor_du;
  t41 = W_act_tilt_el_du * W_act_tilt_el_du;
  t42 = W_act_tilt_az_du * W_act_tilt_az_du;
  t43 = W_act_ailerons_du * W_act_ailerons_du;
  t44 = b_1_scaled * 2.0;
  t45 = b_2_scaled * 2.0;
  t46 = b_3_scaled * 2.0;
  t47 = b_4_scaled * 2.0;
  t48 = delta_ailerons_scaled * 2.0;
  t49 = g_1_scaled * 2.0;
  t50 = g_2_scaled * 2.0;
  t51 = g_3_scaled * 2.0;
  t52 = g_4_scaled * 2.0;
  t53 = gain_motor * gain_motor;
  t80 = 1.0 / I_xx;
  t81 = 1.0 / I_yy;
  t82 = 1.0 / I_zz;
  t84 = 1.0 / gain_el;
  t85 = 1.0 / gain_az;
  t86 = 1.0 / gain_phi;
  t87 = 1.0 / gain_theta;
  t88 = 1.0 / gain_motor;
  t89 = 1.0 / gain_ailerons;
  t90 = 1.0 / m;
  t54 = cos(t4);
  t55 = cos(t5);
  t58 = sin(t4);
  t63 = cos(t6);
  t64 = cos(t7);
  t65 = cos(t8);
  t66 = cos(t9);
  t67 = sin(t5);
  t68 = cos(t10);
  t69 = cos(t11);
  t70 = cos(t12);
  t71 = cos(t13);
  t72 = sin(t6);
  t73 = sin(t7);
  t74 = sin(t8);
  t75 = sin(t9);
  t76 = sin(t10);
  t77 = sin(t11);
  t78 = sin(t12);
  t79 = sin(t13);
  t93 = desired_el_value * t84;
  t94 = desired_az_value * t85;
  t95 = desired_motor_value * t88;
  t101 = flight_path_angle - t5;
  t4 = K_p_M * t19 * t53;
  t113 = t4 * t72;
  t6 = K_p_M * t20 * t53;
  t114 = t6 * t73;
  t7 = K_p_M * t21 * t53;
  t115 = t7 * t74;
  t9 = K_p_M * t22 * t53;
  t116 = t9 * t75;
  t8 = K_p_T * t19 * t53;
  t117 = t8 * t72;
  t10 = K_p_T * t20 * t53;
  t118 = t10 * t73;
  t11 = K_p_T * t21 * t53;
  t119 = t11 * t74;
  t12 = K_p_T * t22 * t53;
  t120 = t12 * t75;
  t4 *= t63;
  t137 = t4 * t68;
  t6 *= t64;
  t138 = t6 * t69;
  t7 *= t65;
  t139 = t7 * t70;
  t8 *= t63;
  t140 = t8 * t68;
  t9 *= t66;
  t141 = t9 * t71;
  t10 *= t64;
  t142 = t10 * t69;
  t11 *= t65;
  t143 = t11 * t70;
  t12 *= t66;
  t144 = t12 * t71;
  t145 = t4 * t76;
  t146 = t6 * t77;
  t147 = t7 * t78;
  t148 = t8 * t76;
  t149 = t9 * t79;
  t150 = t10 * t77;
  t151 = t11 * t78;
  t152 = t12 * t79;
  t9 = cos(t101);
  t10 = sin(t101);
  t105 = -(t93 * 2.0);
  t107 = -(t94 * 2.0);
  t109 = -(t95 * 2.0);
  t123 = l_1 * t117;
  t124 = l_1 * t118;
  t125 = l_2 * t119;
  t126 = l_2 * t120;
  t127 = l_z * t117;
  t128 = l_z * t118;
  t129 = l_z * t119;
  t130 = l_z * t120;
  t154 = l_4 * t140;
  t156 = l_4 * t142;
  t158 = l_3 * t143;
  t161 = l_4 * t148;
  t162 = l_4 * t150;
  t164 = l_3 * t152;
  t4 = ((t117 + t118) + t119) + t120;
  t205 = ((t140 + t142) + t143) + t144;
  t206 = ((t148 + t150) + t151) + t152;
  t176 = -(l_3 * t144);
  t177 = -(l_3 * t151);
  t7 = Cl_alpha * S;
  t11 = t7 * gain_theta * rho * t25;
  t12 = t11 * t9 / 2.0;
  t201 = t55 * t4;
  t202 = t67 * t4;
  t215 = t58 * t205;
  t216 = t54 * t206;
  t218 = t55 * t58 * t206;
  t220_tmp = t54 * t55;
  t220 = t220_tmp * t205;
  t221_tmp = t54 * t67;
  t221 = t221_tmp * t205;
  t136 = Cd_zero + K_Cd * t14 * (t101 * t101);
  t13 = S * rho;
  t4 = t13 * t3 * t25;
  t190 = t4 * t54 * t136 / 2.0;
  t6 = t13 * t2 * t25;
  t8 = t6 * t10 * t136 / 2.0;
  t193 = t4 * t55 * t58 * t136 / 2.0;
  t194 = t4 * t58 * t67 * t136 / 2.0;
  t6 = t6 * t9 * t136 / 2.0;
  t4 = t7 * rho * t25 * t101;
  t203 = t4 * t9 / 2.0 + t8;
  t7 = t4 * t10 / 2.0 - t6;
  t207 = t58 * t203;
  t213 = t220_tmp * t203;
  t224_tmp_tmp = K_Cd * S * gain_theta * rho;
  t4 = t224_tmp_tmp * t2 * t14 * t25 * t101;
  t224 = ((t12 + t11 * t101 * t10 * -0.5) + t4 * t10) + gain_theta * t6;
  t225 = ((t11 * t10 / 2.0 + t101 * t12) - t4 * t9) + gain_theta * t8;
  t4 = I_xx * p;
  t230 =
      dv_global_6 -
      t82 *
          (((((((((((((t4 * q - I_yy * p * q) + t124) + t125) - t123) - t126) +
                  t137) +
                 t139) +
                t161) +
               t162) -
              t138) -
             t141) +
            t177) -
           t164);
  t233_tmp = CL_aileron * S;
  t233 = dv_global_4 -
         t80 * ((((((((((((((I_yy * q * r - I_zz * q * r) + t113) + t115) +
                          t233_tmp * delta_ailerons_scaled * gain_ailerons *
                              rho * t25 / 2.0) -
                         t114) -
                        t116) +
                       l_1 * t140) +
                      l_2 * t144) +
                     l_z * t148) +
                    l_z * t150) +
                   l_z * t151) +
                  l_z * t152) -
                 l_1 * t142) -
                l_2 * t143);
  t209 = t67 * t7;
  t237 =
      dv_global_5 -
      t81 *
          ((((((((((((((I_zz * p * r - t4 * r) + t127) + t128) + t129) + t130) +
                   t146) +
                  t149) +
                 t154) +
                t156) -
               t145) -
              t147) -
             t158) +
            t176) +
           t13 * t25 * (Cm_zero - Cm_alpha * t101) * wing_chord / 2.0);
  t241 = dv_global_2 + t90 * (((t190 + t207) - t215) - t216);
  t247 = (-dv_global_3 +
          -t90 * (((((t193 - t202) + t209) - t213) - t218) + t220)) +
         9.81;
  t246_tmp = t55 * t7;
  b_t246_tmp = t221_tmp * t203;
  c_t246_tmp = t58 * t67 * t206;
  t246 =
      dv_global_1 +
      t90 * (((((t194 + t201) - t246_tmp) - b_t246_tmp) + t221) - c_t246_tmp);
  a_tmp = previous_motor_value1 * t88;
  a = Omega_1_scaled - a_tmp;
  b_a_tmp = previous_motor_value2 * t88;
  b_a = Omega_2_scaled - b_a_tmp;
  c_a_tmp = previous_motor_value3 * t88;
  c_a = Omega_3_scaled - c_a_tmp;
  d_a_tmp = previous_motor_value4 * t88;
  d_a = Omega_4_scaled - d_a_tmp;
  e_a_tmp = previous_phi_value * t86;
  e_a = Phi_scaled - e_a_tmp;
  f_a_tmp = previous_theta_value * t87;
  f_a = Theta_scaled - f_a_tmp;
  g_a_tmp = previous_el_value1 * t84;
  g_a = b_1_scaled - g_a_tmp;
  h_a_tmp = previous_el_value2 * t84;
  h_a = b_2_scaled - h_a_tmp;
  i_a_tmp = previous_el_value3 * t84;
  i_a = b_3_scaled - i_a_tmp;
  j_a_tmp = previous_el_value4 * t84;
  j_a = b_4_scaled - j_a_tmp;
  k_a_tmp = previous_ailerons_value * t89;
  k_a = delta_ailerons_scaled - k_a_tmp;
  l_a_tmp = previous_az_value1 * t85;
  l_a = g_1_scaled - l_a_tmp;
  m_a_tmp = previous_az_value2 * t85;
  m_a = g_2_scaled - m_a_tmp;
  n_a_tmp = previous_az_value3 * t85;
  n_a = g_3_scaled - n_a_tmp;
  o_a_tmp = previous_az_value4 * t85;
  o_a = g_4_scaled - o_a_tmp;
  p_a_tmp = desired_phi_value * t86;
  t86 = Phi_scaled - p_a_tmp;
  q_a_tmp = desired_theta_value * t87;
  t85 = Theta_scaled - q_a_tmp;
  t87 = desired_ailerons_value * t89;
  t5 = delta_ailerons_scaled - t87;
  t2 = Omega_1_scaled - t95;
  t88 = Omega_2_scaled - t95;
  t84 = Omega_3_scaled - t95;
  t10 = Omega_4_scaled - t95;
  t11 = b_1_scaled - t93;
  t12 = b_2_scaled - t93;
  t13 = b_3_scaled - t93;
  t6 = b_4_scaled - t93;
  t7 = g_1_scaled - t94;
  t8 = g_2_scaled - t94;
  t9 = g_3_scaled - t94;
  t4 = g_4_scaled - t94;
  cost =
      gamma_quadratic_du2 * ((((((((((((((t40 * (a * a) + t40 * (b_a * b_a)) +
                                         t40 * (c_a * c_a)) +
                                        t40 * (d_a * d_a)) +
                                       t29 * (e_a * e_a)) +
                                      t38 * (f_a * f_a)) +
                                     t41 * (g_a * g_a)) +
                                    t41 * (h_a * h_a)) +
                                   t41 * (i_a * i_a)) +
                                  t41 * (j_a * j_a)) +
                                 t43 * (k_a * k_a)) +
                                t42 * (l_a * l_a)) +
                               t42 * (m_a * m_a)) +
                              t42 * (n_a * n_a)) +
                             t42 * (o_a * o_a)) +
      ((((((gamma_quadratic_du *
                ((((((((((((((t26 * (t86 * t86) + t27 * (t85 * t85)) +
                             t39 * (t5 * t5)) +
                            t28 * (t2 * t2)) +
                           t28 * (t88 * t88)) +
                          t28 * (t84 * t84)) +
                         t28 * (t10 * t10)) +
                        t36 * (t11 * t11)) +
                       t36 * (t12 * t12)) +
                      t36 * (t13 * t13)) +
                     t36 * (t6 * t6)) +
                    t37 * (t7 * t7)) +
                   t37 * (t8 * t8)) +
                  t37 * (t9 * t9)) +
                 t37 * (t4 * t4)) +
            t35 * (t230 * t230)) +
           t33 * (t233 * t233)) +
          t34 * (t237 * t237)) +
         t31 * (t241 * t241)) +
        t30 * (t246 * t246)) +
       t32 * (t247 * t247));
  t86 = K_p_T * t15 * t53;
  o_a = t86 * t55;
  j_a = K_p_T * Omega_1_scaled;
  k_a = j_a * t53;
  t4 = K_p_T * l_4;
  l_a = t4 * t15 * t53 * t63;
  m_a = K_p_M * t15 * t53;
  t6 = K_p_T * l_z;
  n_a = t6 * t15 * t53;
  d_a = t86 * t54 * t63;
  t85 = gamma_quadratic_du2 * t40;
  t84 = gamma_quadratic_du * t28;
  t88 = t32 * t90 * t247;
  t2 = t30 * t90 * t246;
  t11 = t35 * t82 * t230;
  t12 = K_p_T * l_1;
  t13 = t33 * t80 * t233;
  t5 = t34 * t81 * t237;
  t10 = t31 * t90 * t241;
  gradient[0] =
      ((((((t85 * (t15 - a_tmp * 2.0) + t84 * (t15 + t109)) +
           t88 *
               ((t86 * t67 * t72 - k_a * t54 * t55 * t63 * t68 * 2.0) +
                o_a * t58 * t63 * t76) *
               2.0) +
          t2 *
              ((o_a * t72 - k_a * t58 * t63 * t67 * t76 * 2.0) +
               d_a * t67 * t68) *
              2.0) -
         t11 * ((j_a * l_1 * t53 * t72 * -2.0 + m_a * t63 * t68) + l_a * t76) *
             2.0) -
        t5 *
            ((K_p_M * Omega_1_scaled * t53 * t63 * t76 * -2.0 + n_a * t72) +
             l_a * t68) *
            2.0) -
       t13 * ((m_a * t72 + t12 * t15 * t53 * t63 * t68) + n_a * t63 * t76) *
           2.0) -
      t10 * (t86 * t58 * t63 * t68 + d_a * t76) * 2.0;
  t86 = K_p_T * t16 * t53;
  o_a = t86 * t55;
  j_a = K_p_T * Omega_2_scaled;
  k_a = j_a * t53;
  l_a = t12 * t16 * t53;
  m_a = K_p_M * t16 * t53;
  n_a = t4 * t16 * t53 * t64;
  d_a = t86 * t54 * t64;
  gradient[1] =
      ((((((t85 * (t16 - b_a_tmp * 2.0) + t84 * (t16 + t109)) +
           t88 *
               ((t86 * t67 * t73 - k_a * t54 * t55 * t64 * t69 * 2.0) +
                o_a * t58 * t64 * t77) *
               2.0) +
          t2 *
              ((o_a * t73 - k_a * t58 * t64 * t67 * t77 * 2.0) +
               d_a * t67 * t69) *
              2.0) -
         t11 *
             ((K_p_M * Omega_2_scaled * t53 * t64 * t69 * -2.0 + l_a * t73) +
              n_a * t77) *
             2.0) +
        t13 *
            ((m_a * t73 - j_a * l_z * t53 * t64 * t77 * 2.0) +
             l_a * t64 * t69) *
            2.0) -
       t5 * ((t6 * t16 * t53 * t73 + m_a * t64 * t77) + n_a * t69) * 2.0) -
      t10 * (t86 * t58 * t64 * t69 + d_a * t77) * 2.0;
  t86 = K_p_T * t17 * t53;
  o_a = t86 * t55;
  j_a = K_p_T * Omega_3_scaled;
  k_a = j_a * t53;
  l_a = K_p_M * t17 * t53;
  m_a = l_a * t65;
  n_a = t86 * t54 * t65;
  d_a = K_p_T * l_2;
  t12 = K_p_T * l_3;
  gradient[2] =
      ((((((t85 * (t17 - c_a_tmp * 2.0) + t84 * (t17 + t109)) +
           t88 *
               ((t86 * t67 * t74 - k_a * t54 * t55 * t65 * t70 * 2.0) +
                o_a * t58 * t65 * t78) *
               2.0) +
          t2 *
              ((o_a * t74 - k_a * t58 * t65 * t67 * t78 * 2.0) +
               n_a * t67 * t70) *
              2.0) -
         t11 *
             ((d_a * t17 * t53 * t74 + m_a * t70) -
              j_a * l_3 * t53 * t65 * t78 * 2.0) *
             2.0) -
        t13 *
            ((l_a * t74 - j_a * l_2 * t53 * t65 * t70 * 2.0) +
             t6 * t17 * t53 * t65 * t78) *
            2.0) +
       t5 *
           ((j_a * l_z * t53 * t74 * -2.0 + m_a * t78) +
            t12 * t17 * t53 * t65 * t70) *
           2.0) -
      t10 * (t86 * t58 * t65 * t70 + n_a * t78) * 2.0;
  t86 = K_p_T * t18 * t53;
  o_a = t86 * t55;
  j_a = K_p_T * Omega_4_scaled;
  k_a = j_a * t53;
  l_a = t6 * t18 * t53;
  m_a = d_a * t18 * t53;
  n_a = K_p_M * t18 * t53 * t66;
  d_a = t86 * t54 * t66;
  gradient[3] =
      ((((((t85 * (t18 - d_a_tmp * 2.0) + t84 * (t18 + t109)) +
           t88 *
               ((t86 * t67 * t75 - k_a * t54 * t55 * t66 * t71 * 2.0) +
                o_a * t58 * t66 * t79) *
               2.0) +
          t2 *
              ((o_a * t75 - k_a * t58 * t66 * t67 * t79 * 2.0) +
               d_a * t67 * t71) *
              2.0) -
         t13 *
             ((K_p_M * Omega_4_scaled * t53 * t75 * -2.0 + m_a * t66 * t71) +
              l_a * t66 * t79) *
             2.0) -
        t5 * ((l_a * t75 + n_a * t79) - j_a * l_3 * t53 * t66 * t71 * 2.0) *
            2.0) +
       t11 * ((m_a * t75 + n_a * t71) + t12 * t18 * t53 * t66 * t79) * 2.0) -
      t10 * (t86 * t58 * t66 * t71 + d_a * t79) * 2.0;
  j_a = K_p_T * gain_el;
  t86 = j_a * t19 * t53;
  o_a = gain_el * t58;
  k_a = gain_el * t54;
  l_a = gain_el * l_4;
  m_a = gain_el * t68;
  n_a = gain_el * t76;
  d_a = gamma_quadratic_du2 * t41;
  t85 = gamma_quadratic_du * t36;
  t84 = k_a * t55;
  t12 = gain_el * t55 * t58;
  t4 = -gain_el * t54 * t67;
  t6 = o_a * t67;
  t7 = j_a * l_1;
  t8 = j_a * l_z;
  t9 = K_p_M * gain_el;
  gradient[4] =
      ((((((d_a * (t44 - g_a_tmp * 2.0) + t85 * (t44 + t105)) +
           t88 * ((t84 * t68 * t117 - t12 * t76 * t117) + t86 * t63 * t67) *
               2.0) +
          t2 * ((t4 * t68 * t117 + t6 * t76 * t117) + t86 * t55 * t63) * 2.0) +
         t10 * (o_a * t68 * t117 + k_a * t76 * t117) * 2.0) +
        t11 * ((m_a * t113 + l_a * t76 * t117) + t7 * t19 * t53 * t63) * 2.0) -
       t5 * ((n_a * t113 - l_a * t68 * t117) + t8 * t19 * t53 * t63) * 2.0) +
      t13 * ((m_a * t123 + n_a * t127) - t9 * t19 * t53 * t63) * 2.0;
  t86 = j_a * t20 * t53;
  m_a = gain_el * t77;
  gradient[5] =
      ((((((d_a * (t45 - h_a_tmp * 2.0) + t85 * (t45 + t105)) +
           t88 * ((t84 * t69 * t118 - t12 * t77 * t118) + t86 * t64 * t67) *
               2.0) +
          t2 * ((t4 * t69 * t118 + t6 * t77 * t118) + t86 * t55 * t64) * 2.0) +
         t10 * (o_a * t69 * t118 + k_a * t77 * t118) * 2.0) -
        t11 *
            ((gain_el * t69 * t114 - l_a * t77 * t118) + t7 * t20 * t53 * t64) *
            2.0) +
       t5 * ((m_a * t114 + l_a * t69 * t118) - t8 * t20 * t53 * t64) * 2.0) +
      t13 * ((-gain_el * t69 * t124 + m_a * t128) + t9 * t20 * t53 * t64) * 2.0;
  t86 = j_a * t21 * t53;
  l_a = gain_el * l_3;
  m_a = gain_el * t78;
  n_a = j_a * l_2;
  gradient[6] =
      ((((((d_a * (t46 - i_a_tmp * 2.0) + t85 * (t46 + t105)) +
           t88 * ((t84 * t70 * t119 - t12 * t78 * t119) + t86 * t65 * t67) *
               2.0) +
          t2 * ((t4 * t70 * t119 + t6 * t78 * t119) + t86 * t55 * t65) * 2.0) +
         t10 * (o_a * t70 * t119 + k_a * t78 * t119) * 2.0) -
        t11 *
            ((-gain_el * t70 * t115 + l_a * t78 * t119) +
             n_a * t21 * t53 * t65) *
            2.0) -
       t5 * ((m_a * t115 + l_a * t70 * t119) + t8 * t21 * t53 * t65) * 2.0) -
      t13 * ((gain_el * t70 * t125 - m_a * t129) + t9 * t21 * t53 * t65) * 2.0;
  t86 = j_a * t22 * t53;
  m_a = gain_el * t71;
  t7 = gain_el * t79;
  gradient[7] =
      ((((((d_a * (t47 - j_a_tmp * 2.0) + t85 * (t47 + t105)) +
           t88 * ((t84 * t71 * t120 - t12 * t79 * t120) + t86 * t66 * t67) *
               2.0) +
          t2 * ((t4 * t71 * t120 + t6 * t79 * t120) + t86 * t55 * t66) * 2.0) +
         t10 * (o_a * t71 * t120 + k_a * t79 * t120) * 2.0) -
        t11 * ((m_a * t116 + l_a * t79 * t120) - n_a * t22 * t53 * t66) * 2.0) -
       t5 * ((t7 * -t116 + l_a * t71 * t120) + t8 * t22 * t53 * t66) * 2.0) +
      t13 * ((m_a * t126 + t7 * t130) + t9 * t22 * t53 * t66) * 2.0;
  t86 = gain_az * t54;
  o_a = gain_az * t58;
  k_a = gamma_quadratic_du2 * t42;
  l_a = gamma_quadratic_du * t37;
  m_a = gain_az * t55 * t58;
  n_a = t86 * t55;
  d_a = o_a * t67;
  t85 = t86 * t67;
  t84 = gain_az * l_1;
  t12 = gain_az * l_z;
  gradient[8] = ((((((k_a * (t49 - l_a_tmp * 2.0) + l_a * (t49 + t107)) +
                     t88 * (m_a * t140 + n_a * t148) * 2.0) -
                    t2 * (d_a * t140 + t85 * t148) * 2.0) +
                   t13 * (t84 * t148 - t12 * t140) * 2.0) -
                  t10 * (t86 * t140 - o_a * t148) * 2.0) +
                 t11 * (gain_az * t145 - gain_az * t154) * 2.0) +
                t5 * (gain_az * t137 + gain_az * t161) * 2.0;
  gradient[9] = ((((((k_a * (t50 - m_a_tmp * 2.0) + l_a * (t50 + t107)) +
                     t88 * (m_a * t142 + n_a * t150) * 2.0) -
                    t2 * (d_a * t142 + t85 * t150) * 2.0) -
                   t13 * (t84 * t150 + t12 * t142) * 2.0) -
                  t10 * (t86 * t142 - o_a * t150) * 2.0) -
                 t11 * (gain_az * t146 + gain_az * t156) * 2.0) -
                t5 * (gain_az * t138 - gain_az * t162) * 2.0;
  t84 = gain_az * l_2;
  gradient[10] = ((((((k_a * (t51 - n_a_tmp * 2.0) + l_a * (t51 + t107)) +
                      t88 * (m_a * t143 + n_a * t151) * 2.0) -
                     t2 * (d_a * t143 + t85 * t151) * 2.0) -
                    t13 * (t84 * t151 + t12 * t143) * 2.0) -
                   t10 * (t86 * t143 - o_a * t151) * 2.0) +
                  t11 * (gain_az * t147 + gain_az * t158) * 2.0) +
                 t5 * (gain_az * t139 + gain_az * t177) * 2.0;
  gradient[11] = ((((((k_a * (t52 - o_a_tmp * 2.0) + l_a * (t52 + t107)) +
                      t88 * (m_a * t144 + n_a * t152) * 2.0) -
                     t2 * (d_a * t144 + t85 * t152) * 2.0) +
                    t13 * (t84 * t152 - t12 * t144) * 2.0) -
                   t10 * (t86 * t144 - o_a * t152) * 2.0) -
                  t5 * (gain_az * t141 + gain_az * t164) * 2.0) -
                 t11 * (gain_az * t149 + gain_az * t176) * 2.0;
  t86 = t224_tmp_tmp * t3 * t14 * t25;
  gradient[12] =
      ((((gamma_quadratic_du * t27 * (t24 - q_a_tmp * 2.0) +
          gamma_quadratic_du2 * t38 * (t24 - f_a_tmp * 2.0)) +
         t88 *
             ((((((((gain_theta * t194 + gain_theta * t201) +
                    gain_theta * -t246_tmp) +
                   gain_theta * -b_t246_tmp) +
                  gain_theta * t221) +
                 gain_theta * -c_t246_tmp) +
                t67 * t225) -
               t220_tmp * t224) +
              t86 * t55 * t58 * t101) *
             2.0) +
        t2 *
            ((((((((gain_theta * t193 - gain_theta * t202) +
                   gain_theta * t209) -
                  gain_theta * t213) -
                 gain_theta * t218) +
                gain_theta * t220) +
               t55 * t225) +
              t221_tmp * t224) -
             t86 * t58 * t67 * t101) *
            2.0) -
       t10 * (t58 * t224 + t86 * t54 * t101) * 2.0) -
      Cm_alpha * S * gain_theta * rho * t25 * t34 * t81 * t237 * wing_chord;
  t86 = gain_phi * t54;
  o_a = gain_phi * t55;
  k_a = gain_phi * t67;
  gradient[13] =
      (((gamma_quadratic_du * t26 * (t23 - p_a_tmp * 2.0) +
         gamma_quadratic_du2 * t29 * (t23 - e_a_tmp * 2.0)) +
        t10 *
            (((t86 * t203 - t86 * t205) + gain_phi * t58 * t206) -
             S * gain_phi * rho * t3 * t25 * t58 * t136 / 2.0) *
            2.0) -
       t88 * (((o_a * t190 + o_a * t207) - o_a * t215) - o_a * t216) * 2.0) +
      t2 * (((k_a * t190 + k_a * t207) - k_a * t215) - k_a * t216) * 2.0;
  gradient[14] = (gamma_quadratic_du * t39 * (t48 - t87 * 2.0) +
                  gamma_quadratic_du2 * t43 * (t48 - k_a_tmp * 2.0)) -
                 t233_tmp * gain_ailerons * rho * t25 * t33 * t80 * t233;
  return cost;
}

/*
 * File trailer for compute_cost_and_gradient_first_iteration_v2.c
 *
 * [EOF]
 */
