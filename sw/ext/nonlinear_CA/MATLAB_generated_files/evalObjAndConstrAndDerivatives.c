/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstrAndDerivatives.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:26:49
 */

/* Include Files */
#include "evalObjAndConstrAndDerivatives.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "compute_cost_and_gradient_first_iteration_v2.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const e_struct_T *c_obj_next_next_next_next_next_
 *                const double x[13]
 *                double grad_workspace[14]
 *                int *status
 * Return Type  : double
 */
double b_evalObjAndConstrAndDerivative(
    const e_struct_T *c_obj_next_next_next_next_next_, const double x[13],
    double grad_workspace[14], int *status)
{
  double gradient[13];
  double Beta;
  double CL_aileron;
  double Cd_zero;
  double Cl_alpha;
  double Cm_zero;
  double I_xx;
  double I_yy;
  double I_zz;
  double K_Cd;
  double K_p_M;
  double K_p_T;
  double S;
  double Theta;
  double V;
  double W_act_motor;
  double W_dv_1;
  double W_dv_2;
  double W_dv_5;
  double W_dv_6;
  double desired_ailerons_value;
  double desired_az_value;
  double desired_el_value;
  double desired_motor_value;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double flight_path_angle;
  double fval;
  double gain_ailerons;
  double gain_az;
  double gain_el;
  double gain_motor;
  double gamma_quadratic_du;
  double gamma_quadratic_du2;
  double l_1;
  double l_2;
  double l_3;
  double l_4;
  double l_z;
  double m;
  double p;
  double previous_ailerons_value;
  double previous_az_value1;
  double previous_az_value2;
  double previous_az_value3;
  double previous_az_value4;
  double previous_el_value1;
  double previous_el_value2;
  double previous_el_value3;
  double previous_el_value4;
  double previous_motor_value1;
  double previous_motor_value2;
  double previous_motor_value3;
  double previous_motor_value4;
  double r;
  double rho;
  double t10;
  double t103;
  double t104;
  double t105;
  double t106;
  double t107;
  double t108;
  double t109;
  double t11;
  double t110;
  double t113;
  double t114;
  double t115;
  double t116;
  double t117;
  double t118;
  double t119;
  double t12;
  double t120;
  double t125;
  double t126;
  double t127;
  double t128;
  double t129;
  double t13;
  double t130;
  double t131;
  double t132;
  double t133;
  double t134;
  double t135;
  double t136;
  double t137;
  double t138;
  double t139;
  double t14;
  double t140;
  double t141;
  double t143;
  double t145;
  double t147;
  double t15;
  double t150;
  double t151;
  double t153;
  double t165;
  double t166;
  double t17;
  double t179;
  double t18;
  double t182;
  double t19;
  double t190;
  double t20;
  double t204;
  double t208;
  double t208_tmp;
  double t21;
  double t213;
  double t219;
  double t22;
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
  double t56;
  double t57;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t62;
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
  double t77;
  double t78;
  double t79;
  double t8;
  double t80;
  double t81;
  double t82;
  double t84;
  double t86;
  double t9;
  double t91;
  double t92;
  double t93;
  double t96;
  double t98;
  double wing_chord;
  bool allFinite;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  Beta = c_obj_next_next_next_next_next_->Beta->contents;
  CL_aileron = c_obj_next_next_next_next_next_->CL_aileron->contents;
  Cd_zero = c_obj_next_next_next_next_next_->Cd_zero->contents;
  Cl_alpha = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  Cm_zero = c_obj_next_next_next_next_next_->Cm_zero->contents;
  t213 = c_obj_next_next_next_next_next_->Cm_alpha->contents;
  I_xx = c_obj_next_next_next_next_next_->I_xx->contents;
  I_yy = c_obj_next_next_next_next_next_->I_yy->contents;
  I_zz = c_obj_next_next_next_next_next_->I_zz->contents;
  K_Cd = c_obj_next_next_next_next_next_->K_Cd->contents;
  K_p_M = c_obj_next_next_next_next_next_->K_p_M->contents;
  K_p_T = c_obj_next_next_next_next_next_->K_p_T->contents;
  t8 = c_obj_next_next_next_next_next_->Phi->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  Theta = c_obj_next_next_next_next_next_->Theta->contents;
  V = c_obj_next_next_next_next_next_->V->contents;
  W_act_motor = c_obj_next_next_next_next_next_->W_act_motor->contents;
  W_dv_1 = c_obj_next_next_next_next_next_->W_dv_1->contents;
  W_dv_2 = c_obj_next_next_next_next_next_->W_dv_2->contents;
  t182 = c_obj_next_next_next_next_next_->W_dv_3->contents;
  t125 = c_obj_next_next_next_next_next_->W_dv_4->contents;
  W_dv_5 = c_obj_next_next_next_next_next_->W_dv_5->contents;
  W_dv_6 = c_obj_next_next_next_next_next_->W_dv_6->contents;
  t92 = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  t93 = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  t82 = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  t179 = c_obj_next_next_next_next_next_->W_act_motor_du->contents;
  t190 = c_obj_next_next_next_next_next_->W_act_tilt_el_du->contents;
  t86 = c_obj_next_next_next_next_next_->W_act_tilt_az_du->contents;
  t84 = c_obj_next_next_next_next_next_->W_act_ailerons_du->contents;
  desired_el_value =
      c_obj_next_next_next_next_next_->desired_el_value->contents;
  desired_az_value =
      c_obj_next_next_next_next_next_->desired_az_value->contents;
  desired_motor_value =
      c_obj_next_next_next_next_next_->desired_motor_value->contents;
  desired_ailerons_value =
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents;
  flight_path_angle =
      c_obj_next_next_next_next_next_->flight_path_angle->contents;
  gain_el = c_obj_next_next_next_next_next_->gain_el->contents;
  gain_az = c_obj_next_next_next_next_next_->gain_az->contents;
  gain_motor = c_obj_next_next_next_next_next_->gain_motor->contents;
  gain_ailerons = c_obj_next_next_next_next_next_->gain_ailerons->contents;
  gamma_quadratic_du =
      c_obj_next_next_next_next_next_->gamma_quadratic_du->contents;
  gamma_quadratic_du2 =
      c_obj_next_next_next_next_next_->gamma_quadratic_du2->contents;
  l_1 = c_obj_next_next_next_next_next_->l_1->contents;
  l_2 = c_obj_next_next_next_next_next_->l_2->contents;
  l_3 = c_obj_next_next_next_next_next_->l_3->contents;
  l_4 = c_obj_next_next_next_next_next_->l_4->contents;
  l_z = c_obj_next_next_next_next_next_->l_z->contents;
  m = c_obj_next_next_next_next_next_->m->contents;
  p = c_obj_next_next_next_next_next_->p->contents;
  previous_el_value1 = c_obj_next_next_next_next_next_->actual_u->contents[4];
  previous_el_value2 = c_obj_next_next_next_next_next_->actual_u->contents[5];
  previous_el_value3 = c_obj_next_next_next_next_next_->actual_u->contents[6];
  previous_el_value4 = c_obj_next_next_next_next_next_->actual_u->contents[7];
  previous_az_value1 = c_obj_next_next_next_next_next_->actual_u->contents[8];
  previous_az_value2 = c_obj_next_next_next_next_next_->actual_u->contents[9];
  previous_az_value3 = c_obj_next_next_next_next_next_->actual_u->contents[10];
  previous_az_value4 = c_obj_next_next_next_next_next_->actual_u->contents[11];
  previous_motor_value1 =
      c_obj_next_next_next_next_next_->actual_u->contents[0];
  previous_motor_value2 =
      c_obj_next_next_next_next_next_->actual_u->contents[1];
  previous_motor_value3 =
      c_obj_next_next_next_next_next_->actual_u->contents[2];
  previous_motor_value4 =
      c_obj_next_next_next_next_next_->actual_u->contents[3];
  previous_ailerons_value =
      c_obj_next_next_next_next_next_->actual_u->contents[14];
  t219 = c_obj_next_next_next_next_next_->q->contents;
  r = c_obj_next_next_next_next_next_->r->contents;
  rho = c_obj_next_next_next_next_next_->rho->contents;
  wing_chord = c_obj_next_next_next_next_next_->wing_chord->contents;
  /* COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2 */
  /*     [COST,GRADIENT] =
   * COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi,S,Theta,V,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,W_act_motor_du,W_act_tilt_el_du,W_act_tilt_az_du,W_act_ailerons_du,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,GAMMA_QUADRATIC_DU2,L_1,L_2,L_3,L_4,L_Z,M,P,PREVIOUS_EL_VALUE1,PREVIOUS_EL_VALUE2,PREVIOUS_EL_VALUE3,PREVIOUS_EL_VALUE4,PREVIOUS_AZ_VALUE1,PREVIOUS_AZ_VALUE2,PREVIOUS_AZ_VALUE3,PREVIOUS_AZ_VALUE4,PREVIOUS_MOTOR_VALUE1,PREVIOUS_MOTOR_VALUE2,PREVIOUS_MOTOR_VALUE3,PREVIOUS_MOTOR_VALUE4,PREVIOUS_AILERONS_VALUE,Q,R,RHO,WING_CHORD)
   */
  /*     This function was generated by the Symbolic Math Toolbox version 9.3.
   */
  /*     20-Feb-2024 16:03:48 */
  t3 = cos(t8);
  t4 = sin(Beta);
  t5 = cos(Theta);
  t6 = sin(t8);
  t7 = sin(Theta);
  t8 = x[4] * gain_el;
  t9 = x[5] * gain_el;
  t10 = x[6] * gain_el;
  t11 = x[7] * gain_el;
  t12 = x[8] * gain_az;
  t13 = x[9] * gain_az;
  t14 = x[10] * gain_az;
  t15 = x[11] * gain_az;
  t17 = x[0] * 2.0;
  t18 = x[1] * 2.0;
  t19 = x[2] * 2.0;
  t20 = x[3] * 2.0;
  t21 = x[0] * x[0];
  t22 = x[1] * x[1];
  t23 = x[2] * x[2];
  t24 = x[3] * x[3];
  t25 = V * V;
  t26 = W_act_motor * W_act_motor;
  t27 = W_dv_1 * W_dv_1;
  t28 = W_dv_2 * W_dv_2;
  t29 = t182 * t182;
  t30 = t125 * t125;
  t31 = W_dv_5 * W_dv_5;
  t32 = W_dv_6 * W_dv_6;
  t33 = t92 * t92;
  t34 = t93 * t93;
  t35 = t82 * t82;
  t36 = t179 * t179;
  t37 = t190 * t190;
  t38 = t86 * t86;
  t39 = t84 * t84;
  t40 = x[4] * 2.0;
  t41 = x[5] * 2.0;
  t42 = x[6] * 2.0;
  t43 = x[7] * 2.0;
  t44 = x[12] * 2.0;
  t45 = x[8] * 2.0;
  t46 = x[9] * 2.0;
  t47 = x[10] * 2.0;
  t48 = x[11] * 2.0;
  t49 = gain_motor * gain_motor;
  t72 = 1.0 / I_xx;
  t73 = 1.0 / I_yy;
  t74 = 1.0 / I_zz;
  t77 = 1.0 / gain_el;
  t78 = 1.0 / gain_az;
  t79 = 1.0 / gain_motor;
  t80 = 1.0 / gain_ailerons;
  t81 = 1.0 / m;
  t56 = cos(t8);
  t57 = cos(t9);
  t58 = cos(t10);
  t59 = cos(t11);
  t60 = cos(t12);
  t61 = cos(t13);
  t62 = cos(t14);
  t63 = cos(t15);
  t64 = sin(t8);
  t65 = sin(t9);
  t66 = sin(t10);
  t67 = sin(t11);
  t68 = sin(t12);
  t69 = sin(t13);
  t70 = sin(t14);
  t71 = sin(t15);
  t82 = Theta - flight_path_angle;
  t84 = desired_el_value * t77;
  t15 = desired_az_value * t78;
  t86 = desired_motor_value * t79;
  t91 = t86 * 2.0;
  t92 = cos(t82);
  t93 = sin(t82);
  V = K_p_M * t21 * t49;
  t103 = V * t64;
  W_act_motor = K_p_M * t22 * t49;
  t104 = W_act_motor * t65;
  W_dv_1 = K_p_M * t23 * t49;
  t105 = W_dv_1 * t66;
  t182 = K_p_M * t24 * t49;
  t106 = t182 * t67;
  W_dv_2 = K_p_T * t21 * t49;
  t107 = W_dv_2 * t64;
  t125 = K_p_T * t22 * t49;
  t108 = t125 * t65;
  W_dv_5 = K_p_T * t23 * t49;
  t109 = W_dv_5 * t66;
  W_dv_6 = K_p_T * t24 * t49;
  t110 = W_dv_6 * t67;
  V *= t56;
  t126 = V * t60;
  W_act_motor *= t57;
  t127 = W_act_motor * t61;
  W_dv_1 *= t58;
  t128 = W_dv_1 * t62;
  W_dv_2 *= t56;
  t129 = W_dv_2 * t60;
  t182 *= t59;
  t130 = t182 * t63;
  t125 *= t57;
  t131 = t125 * t61;
  W_dv_5 *= t58;
  t132 = W_dv_5 * t62;
  W_dv_6 *= t59;
  t133 = W_dv_6 * t63;
  t134 = V * t68;
  t135 = W_act_motor * t69;
  t136 = W_dv_1 * t70;
  t137 = W_dv_2 * t68;
  t138 = t182 * t71;
  t139 = t125 * t69;
  t140 = W_dv_5 * t70;
  t141 = W_dv_6 * t71;
  t96 = -(t84 * 2.0);
  t98 = -(t15 * 2.0);
  t113 = l_1 * t107;
  t114 = l_1 * t108;
  t115 = l_2 * t109;
  t116 = l_2 * t110;
  t117 = l_z * t107;
  t118 = l_z * t108;
  t119 = l_z * t109;
  t120 = l_z * t110;
  t143 = l_4 * t129;
  t145 = l_4 * t131;
  t147 = l_3 * t132;
  t150 = l_4 * t137;
  t151 = l_4 * t139;
  t153 = l_3 * t141;
  W_dv_2 = Cl_alpha * S * rho * t25 * t82;
  t179 = ((t107 + t108) + t109) + t110;
  t190 = ((t129 + t131) + t132) + t133;
  t14 = ((t137 + t139) + t140) + t141;
  t125 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t82 * t82);
  t165 = -(l_3 * t133);
  t166 = -(l_3 * t140);
  W_dv_5 = t3 * t5;
  W_dv_1 = S * rho;
  W_dv_6 = W_dv_1 * t4;
  W_act_motor = W_dv_1 * cos(Beta) * t25;
  t182 = W_dv_2 * t92 / 2.0 + W_act_motor * t93 * t125 / 2.0;
  V = I_xx * p;
  t204 =
      dv_global_6 -
      t74 * (((((((((((((V * t219 - I_yy * p * t219) + t114) + t115) - t113) -
                     t116) +
                    t126) +
                   t128) +
                  t150) +
                 t151) -
                t127) -
               t130) +
              t166) -
             t153);
  t208_tmp = CL_aileron * S;
  t208 =
      dv_global_4 -
      t72 * ((((((((((((((I_yy * t219 * r - I_zz * t219 * r) + t103) + t105) +
                       t208_tmp * x[12] * gain_ailerons * rho * t25 / 2.0) -
                      t104) -
                     t106) +
                    l_1 * t129) +
                   l_2 * t133) +
                  l_z * t137) +
                 l_z * t139) +
                l_z * t140) +
               l_z * t141) -
              l_1 * t131) -
             l_2 * t132);
  t213 =
      dv_global_5 -
      t73 *
          ((((((((((((((I_zz * p * r - V * r) + t117) + t118) + t119) + t120) +
                   t135) +
                  t138) +
                 t143) +
                t145) -
               t134) -
              t136) -
             t147) +
            t165) +
           W_dv_1 * t25 * (Cm_zero + t213 * t82) * wing_chord / 2.0);
  V = t3 * t7;
  W_act_motor = W_dv_2 * t93 / 2.0 - W_act_motor * t92 * t125 / 2.0;
  t219 = dv_global_1 +
         t81 * (((((W_dv_6 * t6 * t7 * t25 * t125 / 2.0 + t5 * t179) +
                   -t5 * W_act_motor) +
                  V * t182) +
                 V * t190) -
                t6 * t7 * t14);
  dv_global_5 =
      dv_global_2 -
      t81 *
          (((-(W_dv_1 * t3 * t4 * t25 * t125 / 2.0) + t6 * t182) + t6 * t190) +
           t3 * t14);
  Cm_zero = desired_ailerons_value * t80;
  t13 = x[12] - Cm_zero;
  Theta = x[0] - t86;
  flight_path_angle = x[1] - t86;
  desired_el_value = x[2] - t86;
  t9 = x[3] - t86;
  t10 = x[4] - t84;
  t11 = x[5] - t84;
  t12 = x[6] - t84;
  t84 = x[7] - t84;
  gain_motor = x[8] - t15;
  m = x[9] - t15;
  t8 = x[10] - t15;
  t86 = x[11] - t15;
  I_yy = (dv_global_3 +
          t81 * (((((W_dv_6 * t5 * t6 * t25 * t125 / 2.0 - t7 * t179) +
                    W_dv_5 * t182) +
                   W_dv_5 * t190) -
                  t5 * t6 * t14) +
                 t7 * W_act_motor)) -
         9.81;
  dv_global_4 = previous_motor_value1 * t79;
  t190 = x[0] - dv_global_4;
  S = previous_motor_value2 * t79;
  t179 = x[1] - S;
  dv_global_6 = previous_motor_value3 * t79;
  t82 = x[2] - dv_global_6;
  K_Cd = previous_motor_value4 * t79;
  t93 = x[3] - K_Cd;
  CL_aileron = previous_el_value1 * t77;
  t92 = x[4] - CL_aileron;
  I_xx = previous_el_value2 * t77;
  W_dv_6 = x[5] - I_xx;
  Beta = previous_el_value3 * t77;
  W_dv_5 = x[6] - Beta;
  Cl_alpha = previous_el_value4 * t77;
  t125 = x[7] - Cl_alpha;
  Cd_zero = previous_ailerons_value * t80;
  t182 = x[12] - Cd_zero;
  desired_motor_value = previous_az_value1 * t78;
  W_dv_2 = x[8] - desired_motor_value;
  desired_az_value = previous_az_value2 * t78;
  W_dv_1 = x[9] - desired_az_value;
  t15 = previous_az_value3 * t78;
  W_act_motor = x[10] - t15;
  t14 = previous_az_value4 * t78;
  V = x[11] - t14;
  fval = ((((((gamma_quadratic_du *
                   ((((((((((((t35 * (t13 * t13) + t26 * (Theta * Theta)) +
                              t26 * (flight_path_angle * flight_path_angle)) +
                             t26 * (desired_el_value * desired_el_value)) +
                            t26 * (t9 * t9)) +
                           t33 * (t10 * t10)) +
                          t33 * (t11 * t11)) +
                         t33 * (t12 * t12)) +
                        t33 * (t84 * t84)) +
                       t34 * (gain_motor * gain_motor)) +
                      t34 * (m * m)) +
                     t34 * (t8 * t8)) +
                    t34 * (t86 * t86)) +
               t29 * (I_yy * I_yy)) +
              t32 * (t204 * t204)) +
             t30 * (t208 * t208)) +
            t28 * (dv_global_5 * dv_global_5)) +
           t31 * (t213 * t213)) +
          t27 * (t219 * t219)) +
         gamma_quadratic_du2 *
             ((((((((((((t36 * (t190 * t190) + t36 * (t179 * t179)) +
                        t36 * (t82 * t82)) +
                       t36 * (t93 * t93)) +
                      t37 * (t92 * t92)) +
                     t37 * (W_dv_6 * W_dv_6)) +
                    t37 * (W_dv_5 * W_dv_5)) +
                   t37 * (t125 * t125)) +
                  t39 * (t182 * t182)) +
                 t38 * (W_dv_2 * W_dv_2)) +
                t38 * (W_dv_1 * W_dv_1)) +
               t38 * (W_act_motor * W_act_motor)) +
              t38 * (V * V));
  t9 = K_p_T * t5;
  V = K_p_T * l_4;
  t10 = V * t17 * t49 * t56;
  t11 = K_p_M * t17 * t49;
  W_act_motor = K_p_T * l_z;
  t12 = W_act_motor * t17 * t49;
  t13 = K_p_T * t3;
  m = gamma_quadratic_du2 * t36;
  gain_motor = gamma_quadratic_du * t26;
  t179 = t29 * t81;
  t190 = t179 * I_yy;
  t86 = K_p_T * t7;
  t84 = t9 * t6;
  t93 = t27 * t81 * t219;
  t82 = t13 * t7;
  W_dv_5 = t32 * t74 * t204;
  W_dv_6 = K_p_T * l_1;
  t92 = t30 * t72 * t208;
  t125 = t31 * t73 * t213;
  W_dv_1 = t28 * t81 * dv_global_5;
  W_dv_2 = K_p_T * t6;
  t182 = K_p_T * x[0];
  gradient[0] =
      ((((((m * (t17 - dv_global_4 * 2.0) + gain_motor * (t17 - t91)) -
           t190 *
               ((t86 * t17 * t49 * t64 -
                 t182 * t3 * t5 * t49 * t56 * t60 * 2.0) +
                t84 * t17 * t49 * t56 * t68) *
               2.0) +
          t93 *
              ((t9 * t17 * t49 * t64 - t182 * t6 * t7 * t49 * t56 * t68 * 2.0) +
               t82 * t17 * t49 * t56 * t60) *
              2.0) -
         W_dv_5 *
             ((t182 * l_1 * t49 * t64 * -2.0 + t11 * t56 * t60) + t10 * t68) *
             2.0) -
        t125 *
            ((K_p_M * x[0] * t49 * t56 * t68 * -2.0 + t12 * t64) + t10 * t60) *
            2.0) -
       t92 * ((t11 * t64 + W_dv_6 * t17 * t49 * t56 * t60) + t12 * t56 * t68) *
           2.0) +
      W_dv_1 * (W_dv_2 * t17 * t49 * t56 * t60 + t13 * t17 * t49 * t56 * t68) *
          -2.0;
  t10 = W_dv_6 * t18 * t49;
  t11 = K_p_M * t18 * t49;
  t12 = V * t18 * t49 * t57;
  W_dv_6 = K_p_T * x[1];
  gradient[1] =
      ((((((m * (t18 - S * 2.0) + gain_motor * (t18 - t91)) -
           t190 *
               ((t86 * t18 * t49 * t65 -
                 W_dv_6 * t3 * t5 * t49 * t57 * t61 * 2.0) +
                t84 * t18 * t49 * t57 * t69) *
               2.0) +
          t93 *
              ((t9 * t18 * t49 * t65 -
                W_dv_6 * t6 * t7 * t49 * t57 * t69 * 2.0) +
               t82 * t18 * t49 * t57 * t61) *
              2.0) -
         W_dv_5 *
             ((K_p_M * x[1] * t49 * t57 * t61 * -2.0 + t10 * t65) + t12 * t69) *
             2.0) +
        t92 *
            ((t11 * t65 - W_dv_6 * l_z * t49 * t57 * t69 * 2.0) +
             t10 * t57 * t61) *
            2.0) -
       t125 * ((W_act_motor * t18 * t49 * t65 + t11 * t57 * t69) + t12 * t61) *
           2.0) +
      W_dv_1 * (W_dv_2 * t18 * t49 * t57 * t61 + t13 * t18 * t49 * t57 * t69) *
          -2.0;
  t10 = K_p_M * t19 * t49;
  t11 = t10 * t58;
  t12 = K_p_T * l_2;
  W_dv_6 = K_p_T * l_3;
  t182 = K_p_T * x[2];
  gradient[2] =
      ((((((m * (t19 - dv_global_6 * 2.0) + gain_motor * (t19 - t91)) -
           t190 *
               ((t86 * t19 * t49 * t66 -
                 t182 * t3 * t5 * t49 * t58 * t62 * 2.0) +
                t84 * t19 * t49 * t58 * t70) *
               2.0) +
          t93 *
              ((t9 * t19 * t49 * t66 - t182 * t6 * t7 * t49 * t58 * t70 * 2.0) +
               t82 * t19 * t49 * t58 * t62) *
              2.0) -
         W_dv_5 *
             ((t12 * t19 * t49 * t66 + t11 * t62) -
              t182 * l_3 * t49 * t58 * t70 * 2.0) *
             2.0) -
        t92 *
            ((t10 * t66 - t182 * l_2 * t49 * t58 * t62 * 2.0) +
             W_act_motor * t19 * t49 * t58 * t70) *
            2.0) +
       t125 *
           ((t182 * l_z * t49 * t66 * -2.0 + t11 * t70) +
            W_dv_6 * t19 * t49 * t58 * t62) *
           2.0) +
      W_dv_1 * (W_dv_2 * t19 * t49 * t58 * t62 + t13 * t19 * t49 * t58 * t70) *
          -2.0;
  t10 = W_act_motor * t20 * t49;
  t11 = t12 * t20 * t49;
  t12 = K_p_M * t20 * t49 * t59;
  t182 = K_p_T * x[3];
  gradient[3] =
      ((((((m * (t20 - K_Cd * 2.0) + gain_motor * (t20 - t91)) -
           t190 *
               ((t86 * t20 * t49 * t67 -
                 t182 * t3 * t5 * t49 * t59 * t63 * 2.0) +
                t84 * t20 * t49 * t59 * t71) *
               2.0) +
          t93 *
              ((t9 * t20 * t49 * t67 - t182 * t6 * t7 * t49 * t59 * t71 * 2.0) +
               t82 * t20 * t49 * t59 * t63) *
              2.0) -
         t92 *
             ((K_p_M * x[3] * t49 * t67 * -2.0 + t11 * t59 * t63) +
              t10 * t59 * t71) *
             2.0) -
        t125 * ((t10 * t67 + t12 * t71) - t182 * l_3 * t49 * t59 * t63 * 2.0) *
            2.0) +
       W_dv_5 * ((t11 * t67 + t12 * t63) + W_dv_6 * t20 * t49 * t59 * t71) *
           2.0) +
      W_dv_1 * (W_dv_2 * t20 * t49 * t59 * t63 + t13 * t20 * t49 * t59 * t71) *
          -2.0;
  t9 = K_p_T * gain_el;
  t10 = gain_el * t6;
  t11 = gain_el * t3;
  t12 = gain_el * l_4;
  t13 = gain_el * t60;
  m = gain_el * t68;
  gain_motor = gamma_quadratic_du2 * t37;
  t190 = gamma_quadratic_du * t33;
  t86 = t11 * t5;
  t84 = gain_el * t5 * t6;
  t82 = t9 * t7;
  W_dv_6 = -gain_el * t3 * t7;
  W_dv_2 = t10 * t7;
  t182 = t9 * t5;
  V = t9 * l_1;
  W_act_motor = t9 * l_z;
  t8 = K_p_M * gain_el;
  gradient[4] =
      ((((((gain_motor * (t40 - CL_aileron * 2.0) + t190 * (t40 + t96)) -
           t179 *
               ((t86 * t60 * t107 - t84 * t68 * t107) + t82 * t21 * t49 * t56) *
               I_yy * 2.0) +
          t93 *
              ((W_dv_6 * t60 * t107 + W_dv_2 * t68 * t107) +
               t182 * t21 * t49 * t56) *
              2.0) +
         W_dv_1 * (t10 * t60 * t107 + t11 * t68 * t107) * 2.0) +
        W_dv_5 * ((t13 * t103 + t12 * t68 * t107) + V * t21 * t49 * t56) *
            2.0) -
       t125 * ((m * t103 - t12 * t60 * t107) + W_act_motor * t21 * t49 * t56) *
           2.0) +
      t92 * ((t13 * t113 + m * t117) - t8 * t21 * t49 * t56) * 2.0;
  t13 = gain_el * t69;
  gradient[5] =
      ((((((gain_motor * (t41 - I_xx * 2.0) + t190 * (t41 + t96)) -
           t179 *
               ((t86 * t61 * t108 - t84 * t69 * t108) + t82 * t22 * t49 * t57) *
               I_yy * 2.0) +
          t93 *
              ((W_dv_6 * t61 * t108 + W_dv_2 * t69 * t108) +
               t182 * t22 * t49 * t57) *
              2.0) +
         W_dv_1 * (t10 * t61 * t108 + t11 * t69 * t108) * 2.0) -
        W_dv_5 *
            ((gain_el * t61 * t104 - t12 * t69 * t108) + V * t22 * t49 * t57) *
            2.0) +
       t125 *
           ((t13 * t104 + t12 * t61 * t108) - W_act_motor * t22 * t49 * t57) *
           2.0) +
      t92 * ((-gain_el * t61 * t114 + t13 * t118) + t8 * t22 * t49 * t57) * 2.0;
  t12 = gain_el * l_3;
  t13 = gain_el * t70;
  t9 *= l_2;
  gradient[6] =
      ((((((gain_motor * (t42 - Beta * 2.0) + t190 * (t42 + t96)) -
           t179 *
               ((t86 * t62 * t109 - t84 * t70 * t109) + t82 * t23 * t49 * t58) *
               I_yy * 2.0) +
          t93 *
              ((W_dv_6 * t62 * t109 + W_dv_2 * t70 * t109) +
               t182 * t23 * t49 * t58) *
              2.0) +
         W_dv_1 * (t10 * t62 * t109 + t11 * t70 * t109) * 2.0) -
        W_dv_5 *
            ((-gain_el * t62 * t105 + t12 * t70 * t109) +
             t9 * t23 * t49 * t58) *
            2.0) -
       t125 *
           ((t13 * t105 + t12 * t62 * t109) + W_act_motor * t23 * t49 * t58) *
           2.0) -
      t92 * ((gain_el * t62 * t115 - t13 * t119) + t8 * t23 * t49 * t58) * 2.0;
  t13 = gain_el * t63;
  m = gain_el * t71;
  gradient[7] =
      ((((((gain_motor * (t43 - Cl_alpha * 2.0) + t190 * (t43 + t96)) -
           t179 *
               ((t86 * t63 * t110 - t84 * t71 * t110) + t82 * t24 * t49 * t59) *
               I_yy * 2.0) +
          t93 *
              ((W_dv_6 * t63 * t110 + W_dv_2 * t71 * t110) +
               t182 * t24 * t49 * t59) *
              2.0) +
         W_dv_1 * (t10 * t63 * t110 + t11 * t71 * t110) * 2.0) -
        W_dv_5 * ((t13 * t106 + t12 * t71 * t110) - t9 * t24 * t49 * t59) *
            2.0) -
       t125 * ((m * -t106 + t12 * t63 * t110) + W_act_motor * t24 * t49 * t59) *
           2.0) +
      t92 * ((t13 * t116 + m * t120) + t8 * t24 * t49 * t59) * 2.0;
  t9 = gain_az * t3;
  t10 = gain_az * t6;
  t11 = gamma_quadratic_du2 * t38;
  t12 = gamma_quadratic_du * t34;
  t13 = gain_az * t5 * t6;
  m = t9 * t5;
  gain_motor = t10 * t7;
  t190 = t9 * t7;
  t86 = gain_az * l_1;
  t84 = gain_az * l_z;
  gradient[8] =
      ((((((t11 * (t45 - desired_motor_value * 2.0) + t12 * (t45 + t98)) -
           t179 * (t13 * t129 + m * t137) * I_yy * 2.0) -
          t93 * (gain_motor * t129 + t190 * t137) * 2.0) +
         t92 * (t86 * t137 - t84 * t129) * 2.0) -
        W_dv_1 * (t9 * t129 - t10 * t137) * 2.0) +
       W_dv_5 * (gain_az * t134 - gain_az * t143) * 2.0) +
      t125 * (gain_az * t126 + gain_az * t150) * 2.0;
  gradient[9] =
      ((((((t11 * (t46 - desired_az_value * 2.0) + t12 * (t46 + t98)) -
           t179 * (t13 * t131 + m * t139) * I_yy * 2.0) -
          t93 * (gain_motor * t131 + t190 * t139) * 2.0) -
         t92 * (t86 * t139 + t84 * t131) * 2.0) -
        W_dv_1 * (t9 * t131 - t10 * t139) * 2.0) -
       W_dv_5 * (gain_az * t135 + gain_az * t145) * 2.0) -
      t125 * (gain_az * t127 - gain_az * t151) * 2.0;
  t86 = gain_az * l_2;
  gradient[10] = ((((((t11 * (t47 - t15 * 2.0) + t12 * (t47 + t98)) -
                      t179 * (t13 * t132 + m * t140) * I_yy * 2.0) -
                     t93 * (gain_motor * t132 + t190 * t140) * 2.0) -
                    t92 * (t86 * t140 + t84 * t132) * 2.0) -
                   W_dv_1 * (t9 * t132 - t10 * t140) * 2.0) +
                  W_dv_5 * (gain_az * t136 + gain_az * t147) * 2.0) +
                 t125 * (gain_az * t128 + gain_az * t166) * 2.0;
  gradient[11] = ((((((t11 * (t48 - t14 * 2.0) + t12 * (t48 + t98)) -
                      t179 * (t13 * t133 + m * t141) * I_yy * 2.0) -
                     t93 * (gain_motor * t133 + t190 * t141) * 2.0) +
                    t92 * (t86 * t141 - t84 * t133) * 2.0) -
                   W_dv_1 * (t9 * t133 - t10 * t141) * 2.0) -
                  t125 * (gain_az * t130 + gain_az * t153) * 2.0) -
                 W_dv_5 * (gain_az * t138 + gain_az * t165) * 2.0;
  gradient[12] = (gamma_quadratic_du * t35 * (t44 - Cm_zero * 2.0) +
                  gamma_quadratic_du2 * t39 * (t44 - Cd_zero * 2.0)) -
                 t208_tmp * gain_ailerons * rho * t25 * t30 * t72 * t208;
  memcpy(&grad_workspace[0], &gradient[0], 13U * sizeof(double));
  *status = 1;
  allFinite = rtIsNaN(fval);
  if (rtIsInf(fval) || allFinite) {
    if (allFinite) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    int idx_current;
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 13)) {
      allFinite = ((!rtIsInf(grad_workspace[idx_current])) &&
                   (!rtIsNaN(grad_workspace[idx_current])));
      idx_current++;
    }
    if (!allFinite) {
      idx_current--;
      if (rtIsNaN(grad_workspace[idx_current])) {
        *status = -3;
      } else if (grad_workspace[idx_current] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }
  if (*status == 1) {
    *status = 1;
  }
  return fval;
}

/*
 * Arguments    : const d_struct_T *c_obj_next_next_next_next_next_
 *                const double x[15]
 *                double grad_workspace[16]
 *                int *status
 * Return Type  : double
 */
double evalObjAndConstrAndDerivatives(
    const d_struct_T *c_obj_next_next_next_next_next_, const double x[15],
    double grad_workspace[16], int *status)
{
  double gradient[15];
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double fval;
  bool allFinite;
  /* The thought was here that feeding it an all zero reference */
  /* acceleration vectro would calm down the oscillatory behaviour. As */
  /* this was not successful, the vector remains as desired linear */
  /* accelerations and zero angulars. Instead the gains responsible for */
  /* generating the desired roll commands were adjusted. */
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  fval = c_compute_cost_and_gradient_fir(
      c_obj_next_next_next_next_next_->Beta->contents,
      c_obj_next_next_next_next_next_->CL_aileron->contents,
      c_obj_next_next_next_next_next_->Cd_zero->contents,
      c_obj_next_next_next_next_next_->Cl_alpha->contents,
      c_obj_next_next_next_next_next_->Cm_zero->contents,
      c_obj_next_next_next_next_next_->Cm_alpha->contents,
      c_obj_next_next_next_next_next_->I_xx->contents,
      c_obj_next_next_next_next_next_->I_yy->contents,
      c_obj_next_next_next_next_next_->I_zz->contents,
      c_obj_next_next_next_next_next_->K_Cd->contents,
      c_obj_next_next_next_next_next_->K_p_M->contents,
      c_obj_next_next_next_next_next_->K_p_T->contents, x[0], x[1], x[2], x[3],
      x[13], c_obj_next_next_next_next_next_->S->contents, x[12],
      c_obj_next_next_next_next_next_->V->contents,
      c_obj_next_next_next_next_next_->W_act_phi->contents,
      c_obj_next_next_next_next_next_->W_act_theta->contents,
      c_obj_next_next_next_next_next_->W_act_motor->contents,
      c_obj_next_next_next_next_next_->W_act_phi_du->contents,
      c_obj_next_next_next_next_next_->W_dv_1->contents,
      c_obj_next_next_next_next_next_->W_dv_2->contents,
      c_obj_next_next_next_next_next_->W_dv_3->contents,
      c_obj_next_next_next_next_next_->W_dv_4->contents,
      c_obj_next_next_next_next_next_->W_dv_5->contents,
      c_obj_next_next_next_next_next_->W_dv_6->contents,
      c_obj_next_next_next_next_next_->W_act_tilt_el->contents,
      c_obj_next_next_next_next_next_->W_act_tilt_az->contents,
      c_obj_next_next_next_next_next_->W_act_theta_du->contents,
      c_obj_next_next_next_next_next_->W_act_ailerons->contents,
      c_obj_next_next_next_next_next_->W_act_motor_du->contents,
      c_obj_next_next_next_next_next_->W_act_tilt_el_du->contents,
      c_obj_next_next_next_next_next_->W_act_tilt_az_du->contents,
      c_obj_next_next_next_next_next_->W_act_ailerons_du->contents, x[4], x[5],
      x[6], x[7], x[14],
      c_obj_next_next_next_next_next_->desired_el_value->contents,
      c_obj_next_next_next_next_next_->desired_az_value->contents,
      c_obj_next_next_next_next_next_->desired_phi_value->contents,
      c_obj_next_next_next_next_next_->desired_theta_value->contents,
      c_obj_next_next_next_next_next_->desired_motor_value->contents,
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents,
      dv_global_1, dv_global_2, dv_global_3, dv_global_4, dv_global_5,
      dv_global_6, c_obj_next_next_next_next_next_->flight_path_angle->contents,
      x[8], x[9], x[10], x[11],
      c_obj_next_next_next_next_next_->gain_el->contents,
      c_obj_next_next_next_next_next_->gain_az->contents,
      c_obj_next_next_next_next_next_->gain_phi->contents,
      c_obj_next_next_next_next_next_->gain_theta->contents,
      c_obj_next_next_next_next_next_->gain_motor->contents,
      c_obj_next_next_next_next_next_->gain_ailerons->contents,
      c_obj_next_next_next_next_next_->gamma_quadratic_du->contents,
      c_obj_next_next_next_next_next_->gamma_quadratic_du2->contents,
      c_obj_next_next_next_next_next_->l_1->contents,
      c_obj_next_next_next_next_next_->l_2->contents,
      c_obj_next_next_next_next_next_->l_3->contents,
      c_obj_next_next_next_next_next_->l_4->contents,
      c_obj_next_next_next_next_next_->l_z->contents,
      c_obj_next_next_next_next_next_->m->contents,
      c_obj_next_next_next_next_next_->p->contents,
      c_obj_next_next_next_next_next_->actual_u->contents[4],
      c_obj_next_next_next_next_next_->actual_u->contents[5],
      c_obj_next_next_next_next_next_->actual_u->contents[6],
      c_obj_next_next_next_next_next_->actual_u->contents[7],
      c_obj_next_next_next_next_next_->actual_u->contents[8],
      c_obj_next_next_next_next_next_->actual_u->contents[9],
      c_obj_next_next_next_next_next_->actual_u->contents[10],
      c_obj_next_next_next_next_next_->actual_u->contents[11],
      c_obj_next_next_next_next_next_->actual_u->contents[13],
      c_obj_next_next_next_next_next_->actual_u->contents[12],
      c_obj_next_next_next_next_next_->actual_u->contents[0],
      c_obj_next_next_next_next_next_->actual_u->contents[1],
      c_obj_next_next_next_next_next_->actual_u->contents[2],
      c_obj_next_next_next_next_next_->actual_u->contents[3],
      c_obj_next_next_next_next_next_->actual_u->contents[14],
      c_obj_next_next_next_next_next_->q->contents,
      c_obj_next_next_next_next_next_->r->contents,
      c_obj_next_next_next_next_next_->rho->contents,
      c_obj_next_next_next_next_next_->wing_chord->contents, gradient);
  memcpy(&grad_workspace[0], &gradient[0], 15U * sizeof(double));
  *status = 1;
  allFinite = rtIsNaN(fval);
  if (rtIsInf(fval) || allFinite) {
    if (allFinite) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    int idx_current;
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 15)) {
      allFinite = ((!rtIsInf(grad_workspace[idx_current])) &&
                   (!rtIsNaN(grad_workspace[idx_current])));
      idx_current++;
    }
    if (!allFinite) {
      idx_current--;
      if (rtIsNaN(grad_workspace[idx_current])) {
        *status = -3;
      } else if (grad_workspace[idx_current] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }
  if (*status == 1) {
    *status = 1;
  }
  return fval;
}

/*
 * File trailer for evalObjAndConstrAndDerivatives.c
 *
 * [EOF]
 */
