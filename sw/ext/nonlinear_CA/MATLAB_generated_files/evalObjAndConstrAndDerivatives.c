/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstrAndDerivatives.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
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
double b_evalObjAndConstrAndDerivative(const e_struct_T
  *c_obj_next_next_next_next_next_, const double x[13], double grad_workspace[14],
  int *status)
{
  double previous_controls[15];
  double gradient[13];
  double Beta;
  double CL_aileron;
  double Cd_zero;
  double Cl_alpha;
  double Cm_alpha;
  double Cm_zero;
  double I_xx;
  double I_yy;
  double I_zz;
  double K_Cd;
  double K_p_M;
  double K_p_T;
  double Phi;
  double S;
  double Theta;
  double V;
  double W_act_ailerons;
  double W_act_ailerons_du;
  double W_act_motor1;
  double W_act_motor2;
  double W_act_motor3;
  double W_act_motor4;
  double W_act_motor_du;
  double W_act_tilt_az1;
  double W_act_tilt_az2;
  double W_act_tilt_az3;
  double W_act_tilt_az4;
  double W_act_tilt_az_du;
  double W_act_tilt_el1;
  double W_act_tilt_el2;
  double W_act_tilt_el3;
  double W_act_tilt_el4;
  double W_act_tilt_el_du;
  double W_dv_1;
  double W_dv_2;
  double W_dv_3;
  double W_dv_4;
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
  int idx_current;
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
  Cm_alpha = c_obj_next_next_next_next_next_->Cm_alpha->contents;
  I_xx = c_obj_next_next_next_next_next_->I_xx->contents;
  I_yy = c_obj_next_next_next_next_next_->I_yy->contents;
  I_zz = c_obj_next_next_next_next_next_->I_zz->contents;
  K_Cd = c_obj_next_next_next_next_next_->K_Cd->contents;
  K_p_M = c_obj_next_next_next_next_next_->K_p_M->contents;
  K_p_T = c_obj_next_next_next_next_next_->K_p_T->contents;
  Phi = c_obj_next_next_next_next_next_->Phi->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  Theta = c_obj_next_next_next_next_next_->Theta->contents;
  V = c_obj_next_next_next_next_next_->V->contents;
  W_act_motor_du = c_obj_next_next_next_next_next_->W_act_motor_du->contents;
  W_act_tilt_el_du = c_obj_next_next_next_next_next_->W_act_tilt_el_du->contents;
  W_act_tilt_az_du = c_obj_next_next_next_next_next_->W_act_tilt_az_du->contents;
  W_act_ailerons_du = c_obj_next_next_next_next_next_->
    W_act_ailerons_du->contents;
  W_act_motor1 = c_obj_next_next_next_next_next_->W_act_motor1->contents;
  W_act_motor2 = c_obj_next_next_next_next_next_->W_act_motor2->contents;
  W_act_motor3 = c_obj_next_next_next_next_next_->W_act_motor3->contents;
  W_act_motor4 = c_obj_next_next_next_next_next_->W_act_motor4->contents;
  W_dv_1 = c_obj_next_next_next_next_next_->W_dv_1->contents;
  W_dv_2 = c_obj_next_next_next_next_next_->W_dv_2->contents;
  W_dv_3 = c_obj_next_next_next_next_next_->W_dv_3->contents;
  W_dv_4 = c_obj_next_next_next_next_next_->W_dv_4->contents;
  W_dv_5 = c_obj_next_next_next_next_next_->W_dv_5->contents;
  W_dv_6 = c_obj_next_next_next_next_next_->W_dv_6->contents;
  W_act_tilt_el1 = c_obj_next_next_next_next_next_->W_act_tilt_el1->contents;
  W_act_tilt_el2 = c_obj_next_next_next_next_next_->W_act_tilt_el2->contents;
  W_act_tilt_el3 = c_obj_next_next_next_next_next_->W_act_tilt_el3->contents;
  W_act_tilt_el4 = c_obj_next_next_next_next_next_->W_act_tilt_el4->contents;
  W_act_tilt_az1 = c_obj_next_next_next_next_next_->W_act_tilt_az1->contents;
  W_act_tilt_az2 = c_obj_next_next_next_next_next_->W_act_tilt_az2->contents;
  W_act_tilt_az3 = c_obj_next_next_next_next_next_->W_act_tilt_az3->contents;
  W_act_tilt_az4 = c_obj_next_next_next_next_next_->W_act_tilt_az4->contents;
  W_act_ailerons = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  desired_el_value = c_obj_next_next_next_next_next_->desired_el_value->contents;
  desired_az_value = c_obj_next_next_next_next_next_->desired_az_value->contents;
  desired_motor_value = c_obj_next_next_next_next_next_->
    desired_motor_value->contents;
  desired_ailerons_value =
    c_obj_next_next_next_next_next_->desired_ailerons_value->contents;
  flight_path_angle = c_obj_next_next_next_next_next_->
    flight_path_angle->contents;
  gain_el = c_obj_next_next_next_next_next_->gain_el->contents;
  gain_az = c_obj_next_next_next_next_next_->gain_az->contents;
  gain_motor = c_obj_next_next_next_next_next_->gain_motor->contents;
  gain_ailerons = c_obj_next_next_next_next_next_->gain_ailerons->contents;
  gamma_quadratic_du = c_obj_next_next_next_next_next_->
    gamma_quadratic_du->contents;
  gamma_quadratic_du2 = c_obj_next_next_next_next_next_->
    gamma_quadratic_du2->contents;
  for (idx_current = 0; idx_current < 15; idx_current++) {
    previous_controls[idx_current] =
      c_obj_next_next_next_next_next_->previous_controls->contents[idx_current];
  }

  double a;
  double a_tmp;
  double a_tmp_tmp;
  double ab_a;
  double ab_a_tmp;
  double ab_a_tmp_tmp;
  double b_a;
  double b_a_tmp;
  double b_a_tmp_tmp;
  double b_fval_tmp;
  double bb_a;
  double bb_a_tmp;
  double bb_a_tmp_tmp;
  double c_a;
  double c_a_tmp;
  double c_a_tmp_tmp;
  double c_fval_tmp;
  double cb_a;
  double cb_a_tmp;
  double cb_a_tmp_tmp;
  double d_a;
  double d_a_tmp;
  double d_a_tmp_tmp;
  double d_fval_tmp;
  double db_a;
  double db_a_tmp;
  double db_a_tmp_tmp;
  double e_a;
  double e_a_tmp;
  double e_a_tmp_tmp;
  double e_fval_tmp;
  double eb_a_tmp;
  double f_a;
  double f_a_tmp;
  double f_a_tmp_tmp;
  double f_fval_tmp;
  double fb_a_tmp;
  double fval_tmp;
  double g_a;
  double g_a_tmp;
  double g_a_tmp_tmp;
  double g_fval_tmp;
  double gb_a_tmp;
  double h_a;
  double h_a_tmp;
  double h_a_tmp_tmp;
  double h_fval_tmp;
  double hb_a_tmp;
  double i_a;
  double i_a_tmp;
  double i_a_tmp_tmp;
  double i_fval_tmp;
  double ib_a_tmp;
  double j_a;
  double j_a_tmp;
  double j_a_tmp_tmp;
  double j_fval_tmp;
  double jb_a_tmp;
  double k_a;
  double k_a_tmp;
  double k_a_tmp_tmp;
  double kb_a_tmp;
  double l_1;
  double l_2;
  double l_3;
  double l_4;
  double l_a;
  double l_a_tmp;
  double l_a_tmp_tmp;
  double l_z;
  double lb_a_tmp;
  double m;
  double m_a;
  double m_a_tmp;
  double m_a_tmp_tmp;
  double mb_a_tmp;
  double n_a;
  double n_a_tmp;
  double n_a_tmp_tmp;
  double nb_a_tmp;
  double o_a;
  double o_a_tmp;
  double o_a_tmp_tmp;
  double ob_a_tmp;
  double p;
  double p_a;
  double p_a_tmp;
  double p_a_tmp_tmp;
  double pb_a_tmp;
  double q;
  double q_a;
  double q_a_tmp;
  double q_a_tmp_tmp;
  double qb_a_tmp;
  double r;
  double r_a;
  double r_a_tmp;
  double r_a_tmp_tmp;
  double rb_a_tmp;
  double rho;
  double s_a;
  double s_a_tmp;
  double s_a_tmp_tmp;
  double sb_a_tmp;
  double t_a;
  double t_a_tmp;
  double t_a_tmp_tmp;
  double tb_a_tmp;
  double u_a;
  double u_a_tmp;
  double u_a_tmp_tmp;
  double ub_a_tmp;
  double v_a;
  double v_a_tmp;
  double v_a_tmp_tmp;
  double w_a;
  double w_a_tmp;
  double w_a_tmp_tmp;
  double wing_chord;
  double x_a;
  double x_a_tmp;
  double x_a_tmp_tmp;
  double y_a;
  double y_a_tmp;
  double y_a_tmp_tmp;
  l_1 = c_obj_next_next_next_next_next_->l_1->contents;
  l_2 = c_obj_next_next_next_next_next_->l_2->contents;
  l_3 = c_obj_next_next_next_next_next_->l_3->contents;
  l_4 = c_obj_next_next_next_next_next_->l_4->contents;
  l_z = c_obj_next_next_next_next_next_->l_z->contents;
  m = c_obj_next_next_next_next_next_->m->contents;
  p = c_obj_next_next_next_next_next_->p->contents;
  q = c_obj_next_next_next_next_next_->q->contents;
  r = c_obj_next_next_next_next_next_->r->contents;
  rho = c_obj_next_next_next_next_next_->rho->contents;
  wing_chord = c_obj_next_next_next_next_next_->wing_chord->contents;

  /* COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2 */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi,S,Theta,V,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD) */
  /*  function [cost,gradient] = compute_cost_and_gradient_second_iteration_v2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi,S,Theta,V,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,b_1_scaled,b_2_scaled,b_3_scaled,b_4_scaled,delta_ailerons_scaled,desired_el_value,desired_az_value,desired_motor_value,desired_ailerons_value,dv_global_1,dv_global_2,dv_global_3,dv_global_4,dv_global_5,dv_global_6,flight_path_angle,g_1_scaled,g_2_scaled,g_3_scaled,g_4_scaled,gain_el,gain_az,gain_motor,gain_ailerons,gamma_quadratic_du,l_1,l_2,l_3,l_4,l_z,m,p,q,r,rho,wing_chord) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     30-Nov-2023 18:08:51 */
  /*   */
  /*  t2 = cos(Beta); */
  /*  t3 = cos(Phi); */
  /*  t4 = sin(Beta); */
  /*  t5 = cos(Theta); */
  /*  t6 = sin(Phi); */
  /*  t7 = sin(Theta); */
  /*  t8 = b_1_scaled.*gain_el; */
  /*  t9 = b_2_scaled.*gain_el; */
  /*  t10 = b_3_scaled.*gain_el; */
  /*  t11 = b_4_scaled.*gain_el; */
  /*  t12 = g_1_scaled.*gain_az; */
  /*  t13 = g_2_scaled.*gain_az; */
  /*  t14 = g_3_scaled.*gain_az; */
  /*  t15 = g_4_scaled.*gain_az; */
  /*  t16 = Cl_alpha.^2; */
  /*  t17 = Omega_1_scaled.^2; */
  /*  t18 = Omega_2_scaled.^2; */
  /*  t19 = Omega_3_scaled.^2; */
  /*  t20 = Omega_4_scaled.^2; */
  /*  t21 = V.^2; */
  /*  t22 = W_act_motor.^2; */
  /*  t23 = W_dv_1.^2; */
  /*  t24 = W_dv_2.^2; */
  /*  t25 = W_dv_3.^2; */
  /*  t26 = W_dv_4.^2; */
  /*  t27 = W_dv_5.^2; */
  /*  t28 = W_dv_6.^2; */
  /*  t29 = W_act_tilt_el.^2; */
  /*  t30 = W_act_tilt_az.^2; */
  /*  t31 = W_act_ailerons.^2; */
  /*  t32 = gain_motor.^2; */
  /*  t33 = I_xx.*p.*q; */
  /*  t34 = I_yy.*p.*q; */
  /*  t35 = I_xx.*p.*r; */
  /*  t36 = I_zz.*p.*r; */
  /*  t37 = I_yy.*q.*r; */
  /*  t38 = I_zz.*q.*r; */
  /*  t55 = 1.0./I_xx; */
  /*  t56 = 1.0./I_yy; */
  /*  t57 = 1.0./I_zz; */
  /*  t59 = -flight_path_angle; */
  /*  t60 = 1.0./gain_el; */
  /*  t61 = 1.0./gain_az; */
  /*  t62 = 1.0./gain_motor; */
  /*  t63 = 1.0./gain_ailerons; */
  /*  t64 = 1.0./m; */
  /*  t39 = cos(t8); */
  /*  t40 = cos(t9); */
  /*  t41 = cos(t10); */
  /*  t42 = cos(t11); */
  /*  t43 = cos(t12); */
  /*  t44 = cos(t13); */
  /*  t45 = cos(t14); */
  /*  t46 = cos(t15); */
  /*  t47 = sin(t8); */
  /*  t48 = sin(t9); */
  /*  t49 = sin(t10); */
  /*  t50 = sin(t11); */
  /*  t51 = sin(t12); */
  /*  t52 = sin(t13); */
  /*  t53 = sin(t14); */
  /*  t54 = sin(t15); */
  /*  t65 = Theta+t59; */
  /*  t66 = -t34; */
  /*  t67 = desired_el_value.*t60; */
  /*  t68 = desired_az_value.*t61; */
  /*  t69 = desired_motor_value.*t62; */
  /*  t70 = -t35; */
  /*  t71 = -t38; */
  /*  t94 = (CL_aileron.*S.*delta_ailerons_scaled.*gain_ailerons.*rho.*t21)./2.0; */
  /*  t72 = t67.*2.0; */
  /*  t73 = t68.*2.0; */
  /*  t74 = t69.*2.0; */
  /*  t75 = cos(t65); */
  /*  t76 = sin(t65); */
  /*  t77 = Cm_alpha.*t65; */
  /*  t78 = -t67; */
  /*  t80 = -t68; */
  /*  t82 = -t69; */
  /*  t84 = t65.^2; */
  /*  t86 = K_p_M.*t17.*t32.*t47; */
  /*  t87 = K_p_M.*t18.*t32.*t48; */
  /*  t88 = K_p_M.*t19.*t32.*t49; */
  /*  t89 = K_p_M.*t20.*t32.*t50; */
  /*  t90 = K_p_T.*t17.*t32.*t47; */
  /*  t91 = K_p_T.*t18.*t32.*t48; */
  /*  t92 = K_p_T.*t19.*t32.*t49; */
  /*  t93 = K_p_T.*t20.*t32.*t50; */
  /*  t109 = K_p_M.*t17.*t32.*t39.*t43; */
  /*  t110 = K_p_M.*t18.*t32.*t40.*t44; */
  /*  t111 = K_p_M.*t19.*t32.*t41.*t45; */
  /*  t112 = K_p_T.*t17.*t32.*t39.*t43; */
  /*  t113 = K_p_M.*t20.*t32.*t42.*t46; */
  /*  t114 = K_p_T.*t18.*t32.*t40.*t44; */
  /*  t115 = K_p_T.*t19.*t32.*t41.*t45; */
  /*  t116 = K_p_T.*t20.*t32.*t42.*t46; */
  /*  t117 = K_p_M.*t17.*t32.*t39.*t51; */
  /*  t118 = K_p_M.*t18.*t32.*t40.*t52; */
  /*  t119 = K_p_M.*t19.*t32.*t41.*t53; */
  /*  t120 = K_p_T.*t17.*t32.*t39.*t51; */
  /*  t121 = K_p_M.*t20.*t32.*t42.*t54; */
  /*  t122 = K_p_T.*t18.*t32.*t40.*t52; */
  /*  t123 = K_p_T.*t19.*t32.*t41.*t53; */
  /*  t124 = K_p_T.*t20.*t32.*t42.*t54; */
  /*  t79 = -t72; */
  /*  t81 = -t73; */
  /*  t83 = -t74; */
  /*  t85 = Cm_zero+t77; */
  /*  t95 = K_Cd.*t16.*t84; */
  /*  t96 = l_1.*t90; */
  /*  t97 = l_1.*t91; */
  /*  t98 = l_2.*t92; */
  /*  t99 = l_2.*t93; */
  /*  t100 = l_z.*t90; */
  /*  t101 = l_z.*t91; */
  /*  t102 = l_z.*t92; */
  /*  t103 = l_z.*t93; */
  /*  t104 = -t87; */
  /*  t105 = -t89; */
  /*  t125 = l_1.*t112; */
  /*  t126 = l_4.*t112; */
  /*  t127 = l_1.*t114; */
  /*  t128 = l_4.*t114; */
  /*  t129 = l_2.*t115; */
  /*  t130 = l_3.*t115; */
  /*  t131 = l_2.*t116; */
  /*  t132 = l_3.*t116; */
  /*  t133 = l_4.*t120; */
  /*  t134 = l_4.*t122; */
  /*  t135 = l_3.*t123; */
  /*  t136 = l_3.*t124; */
  /*  t137 = l_z.*t120; */
  /*  t138 = l_z.*t122; */
  /*  t139 = l_z.*t123; */
  /*  t140 = l_z.*t124; */
  /*  t141 = -t110; */
  /*  t142 = -t113; */
  /*  t143 = -t117; */
  /*  t144 = -t119; */
  /*  t152 = (Cl_alpha.*S.*rho.*t21.*t65.*t75)./2.0; */
  /*  t153 = (Cl_alpha.*S.*rho.*t21.*t65.*t76)./2.0; */
  /*  t162 = t90+t91+t92+t93; */
  /*  t173 = t112+t114+t115+t116; */
  /*  t174 = t120+t122+t123+t124; */
  /*  t106 = -t96; */
  /*  t107 = -t99; */
  /*  t108 = Cd_zero+t95; */
  /*  t145 = -t127; */
  /*  t146 = -t129; */
  /*  t147 = -t130; */
  /*  t148 = -t132; */
  /*  t149 = -t135; */
  /*  t150 = -t136; */
  /*  t151 = (S.*rho.*t21.*t85.*wing_chord)./2.0; */
  /*  t163 = t5.*t162; */
  /*  t164 = t7.*t162; */
  /*  t175 = t6.*t173; */
  /*  t176 = t3.*t174; */
  /*  t177 = t3.*t5.*t173; */
  /*  t178 = t3.*t7.*t173; */
  /*  t179 = t5.*t6.*t174; */
  /*  t180 = t6.*t7.*t174; */
  /*  t155 = (S.*rho.*t3.*t4.*t21.*t108)./2.0; */
  /*  t157 = (S.*rho.*t4.*t5.*t6.*t21.*t108)./2.0; */
  /*  t158 = (S.*rho.*t4.*t6.*t7.*t21.*t108)./2.0; */
  /*  t160 = (S.*rho.*t2.*t21.*t75.*t108)./2.0; */
  /*  t161 = (S.*rho.*t2.*t21.*t76.*t108)./2.0; */
  /*  t182 = -t180; */
  /*  t183 = t33+t66+t97+t98+t106+t107+t109+t111+t133+t134+t141+t142+t149+t150; */
  /*  t186 = t37+t71+t86+t88+t94+t104+t105+t125+t131+t137+t138+t139+t140+t145+t146; */
  /*  t190 = t36+t70+t100+t101+t102+t103+t118+t121+t126+t128+t143+t144+t147+t148+t151; */
  /*  t156 = -t155; */
  /*  t165 = t152+t161; */
  /*  t168 = -t5.*(t153-t160); */
  /*  t184 = t57.*t183; */
  /*  t188 = t55.*t186; */
  /*  t193 = t56.*t190; */
  /*  t167 = t6.*t165; */
  /*  t170 = t3.*t5.*t165; */
  /*  t171 = t3.*t7.*t165; */
  /*  t185 = -t184; */
  /*  t189 = -t188; */
  /*  t194 = -t193; */
  /*  t187 = dv_global_6+t185; */
  /*  t191 = dv_global_4+t189; */
  /*  t192 = t156+t167+t175+t176; */
  /*  t196 = dv_global_5+t194; */
  /*  t199 = t158+t163+t168+t171+t178+t182; */
  /*  t195 = t64.*t192; */
  /*  t201 = t64.*t199; */
  /*  t197 = -t195; */
  /*  t202 = dv_global_1+t201; */
  /*  t198 = dv_global_2+t197; */
  /*  cost = gamma_quadratic_du.*(t31.*(delta_ailerons_scaled-desired_ailerons_value.*t63).^2+t22.*(Omega_1_scaled+t82).^2+t22.*(Omega_2_scaled+t82).^2+t22.*(Omega_3_scaled+t82).^2+t22.*(Omega_4_scaled+t82).^2+t29.*(b_1_scaled+t78).^2+t29.*(b_2_scaled+t78).^2+t29.*(b_3_scaled+t78).^2+t29.*(b_4_scaled+t78).^2+t30.*(g_1_scaled+t80).^2+t30.*(g_2_scaled+t80).^2+t30.*(g_3_scaled+t80).^2+t30.*(g_4_scaled+t80).^2)+t25.*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).^2+t28.*t187.^2+t26.*t191.^2+t24.*t198.^2+t27.*t196.^2+t23.*t202.^2; */
  /*  if nargout > 1 */
  /*      et1 = gamma_quadratic_du.*t22.*(Omega_1_scaled.*2.0+t83)-t28.*t57.*t187.*(K_p_T.*Omega_1_scaled.*l_1.*t32.*t47.*-2.0+K_p_M.*Omega_1_scaled.*t32.*t39.*t43.*2.0+K_p_T.*Omega_1_scaled.*l_4.*t32.*t39.*t51.*2.0).*2.0-t26.*t55.*t191.*(K_p_M.*Omega_1_scaled.*t32.*t47.*2.0+K_p_T.*Omega_1_scaled.*l_1.*t32.*t39.*t43.*2.0+K_p_T.*Omega_1_scaled.*l_z.*t32.*t39.*t51.*2.0).*2.0-t27.*t56.*t196.*(K_p_T.*Omega_1_scaled.*l_z.*t32.*t47.*2.0-K_p_M.*Omega_1_scaled.*t32.*t39.*t51.*2.0+K_p_T.*Omega_1_scaled.*l_4.*t32.*t39.*t43.*2.0).*2.0-t24.*t64.*t198.*(K_p_T.*Omega_1_scaled.*t6.*t32.*t39.*t43.*2.0+K_p_T.*Omega_1_scaled.*t3.*t32.*t39.*t51.*2.0).*2.0-t25.*t64.*(K_p_T.*Omega_1_scaled.*t7.*t32.*t47.*2.0-K_p_T.*Omega_1_scaled.*t3.*t5.*t32.*t39.*t43.*2.0+K_p_T.*Omega_1_scaled.*t5.*t6.*t32.*t39.*t51.*2.0).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0; */
  /*      et2 = t23.*t64.*t202.*(K_p_T.*Omega_1_scaled.*t5.*t32.*t47.*2.0+K_p_T.*Omega_1_scaled.*t3.*t7.*t32.*t39.*t43.*2.0-K_p_T.*Omega_1_scaled.*t6.*t7.*t32.*t39.*t51.*2.0).*2.0; */
  /*      et3 = gamma_quadratic_du.*t22.*(Omega_2_scaled.*2.0+t83)-t28.*t57.*t187.*(K_p_T.*Omega_2_scaled.*l_1.*t32.*t48.*2.0-K_p_M.*Omega_2_scaled.*t32.*t40.*t44.*2.0+K_p_T.*Omega_2_scaled.*l_4.*t32.*t40.*t52.*2.0).*2.0+t26.*t55.*t191.*(K_p_M.*Omega_2_scaled.*t32.*t48.*2.0+K_p_T.*Omega_2_scaled.*l_1.*t32.*t40.*t44.*2.0-K_p_T.*Omega_2_scaled.*l_z.*t32.*t40.*t52.*2.0).*2.0-t27.*t56.*t196.*(K_p_T.*Omega_2_scaled.*l_z.*t32.*t48.*2.0+K_p_M.*Omega_2_scaled.*t32.*t40.*t52.*2.0+K_p_T.*Omega_2_scaled.*l_4.*t32.*t40.*t44.*2.0).*2.0-t24.*t64.*t198.*(K_p_T.*Omega_2_scaled.*t6.*t32.*t40.*t44.*2.0+K_p_T.*Omega_2_scaled.*t3.*t32.*t40.*t52.*2.0).*2.0-t25.*t64.*(K_p_T.*Omega_2_scaled.*t7.*t32.*t48.*2.0-K_p_T.*Omega_2_scaled.*t3.*t5.*t32.*t40.*t44.*2.0+K_p_T.*Omega_2_scaled.*t5.*t6.*t32.*t40.*t52.*2.0).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0; */
  /*      et4 = t23.*t64.*t202.*(K_p_T.*Omega_2_scaled.*t5.*t32.*t48.*2.0+K_p_T.*Omega_2_scaled.*t3.*t7.*t32.*t40.*t44.*2.0-K_p_T.*Omega_2_scaled.*t6.*t7.*t32.*t40.*t52.*2.0).*2.0; */
  /*      et5 = gamma_quadratic_du.*t22.*(Omega_3_scaled.*2.0+t83)-t28.*t57.*t187.*(K_p_T.*Omega_3_scaled.*l_2.*t32.*t49.*2.0+K_p_M.*Omega_3_scaled.*t32.*t41.*t45.*2.0-K_p_T.*Omega_3_scaled.*l_3.*t32.*t41.*t53.*2.0).*2.0-t26.*t55.*t191.*(K_p_M.*Omega_3_scaled.*t32.*t49.*2.0-K_p_T.*Omega_3_scaled.*l_2.*t32.*t41.*t45.*2.0+K_p_T.*Omega_3_scaled.*l_z.*t32.*t41.*t53.*2.0).*2.0+t27.*t56.*t196.*(K_p_T.*Omega_3_scaled.*l_z.*t32.*t49.*-2.0+K_p_M.*Omega_3_scaled.*t32.*t41.*t53.*2.0+K_p_T.*Omega_3_scaled.*l_3.*t32.*t41.*t45.*2.0).*2.0-t24.*t64.*t198.*(K_p_T.*Omega_3_scaled.*t6.*t32.*t41.*t45.*2.0+K_p_T.*Omega_3_scaled.*t3.*t32.*t41.*t53.*2.0).*2.0-t25.*t64.*(K_p_T.*Omega_3_scaled.*t7.*t32.*t49.*2.0-K_p_T.*Omega_3_scaled.*t3.*t5.*t32.*t41.*t45.*2.0+K_p_T.*Omega_3_scaled.*t5.*t6.*t32.*t41.*t53.*2.0).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0; */
  /*      et6 = t23.*t64.*t202.*(K_p_T.*Omega_3_scaled.*t5.*t32.*t49.*2.0+K_p_T.*Omega_3_scaled.*t3.*t7.*t32.*t41.*t45.*2.0-K_p_T.*Omega_3_scaled.*t6.*t7.*t32.*t41.*t53.*2.0).*2.0; */
  /*      et7 = gamma_quadratic_du.*t22.*(Omega_4_scaled.*2.0+t83)+t28.*t57.*t187.*(K_p_T.*Omega_4_scaled.*l_2.*t32.*t50.*2.0+K_p_M.*Omega_4_scaled.*t32.*t42.*t46.*2.0+K_p_T.*Omega_4_scaled.*l_3.*t32.*t42.*t54.*2.0).*2.0-t26.*t55.*t191.*(K_p_M.*Omega_4_scaled.*t32.*t50.*-2.0+K_p_T.*Omega_4_scaled.*l_2.*t32.*t42.*t46.*2.0+K_p_T.*Omega_4_scaled.*l_z.*t32.*t42.*t54.*2.0).*2.0-t27.*t56.*t196.*(K_p_T.*Omega_4_scaled.*l_z.*t32.*t50.*2.0+K_p_M.*Omega_4_scaled.*t32.*t42.*t54.*2.0-K_p_T.*Omega_4_scaled.*l_3.*t32.*t42.*t46.*2.0).*2.0-t24.*t64.*t198.*(K_p_T.*Omega_4_scaled.*t6.*t32.*t42.*t46.*2.0+K_p_T.*Omega_4_scaled.*t3.*t32.*t42.*t54.*2.0).*2.0-t25.*t64.*(K_p_T.*Omega_4_scaled.*t7.*t32.*t50.*2.0-K_p_T.*Omega_4_scaled.*t3.*t5.*t32.*t42.*t46.*2.0+K_p_T.*Omega_4_scaled.*t5.*t6.*t32.*t42.*t54.*2.0).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0; */
  /*      et8 = t23.*t64.*t202.*(K_p_T.*Omega_4_scaled.*t5.*t32.*t50.*2.0+K_p_T.*Omega_4_scaled.*t3.*t7.*t32.*t42.*t46.*2.0-K_p_T.*Omega_4_scaled.*t6.*t7.*t32.*t42.*t54.*2.0).*2.0; */
  /*      mt1 = [et1+et2;et3+et4;et5+et6;et7+et8;gamma_quadratic_du.*t29.*(b_1_scaled.*2.0+t79)-t25.*t64.*(gain_el.*t3.*t5.*t43.*t90-gain_el.*t5.*t6.*t51.*t90+K_p_T.*gain_el.*t7.*t17.*t32.*t39).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0+t23.*t64.*t202.*(-gain_el.*t3.*t7.*t43.*t90+gain_el.*t6.*t7.*t51.*t90+K_p_T.*gain_el.*t5.*t17.*t32.*t39).*2.0+t24.*t64.*t198.*(gain_el.*t6.*t43.*t90+gain_el.*t3.*t51.*t90).*2.0+t28.*t57.*t187.*(gain_el.*t43.*t86+gain_el.*l_4.*t51.*t90+K_p_T.*gain_el.*l_1.*t17.*t32.*t39).*2.0-t27.*t56.*t196.*(gain_el.*t51.*t86-gain_el.*l_4.*t43.*t90+K_p_T.*gain_el.*l_z.*t17.*t32.*t39).*2.0+t26.*t55.*t191.*(gain_el.*t43.*t96+gain_el.*t51.*t100-K_p_M.*gain_el.*t17.*t32.*t39).*2.0]; */
  /*      mt2 = [gamma_quadratic_du.*t29.*(b_2_scaled.*2.0+t79)-t25.*t64.*(gain_el.*t3.*t5.*t44.*t91-gain_el.*t5.*t6.*t52.*t91+K_p_T.*gain_el.*t7.*t18.*t32.*t40).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0+t23.*t64.*t202.*(-gain_el.*t3.*t7.*t44.*t91+gain_el.*t6.*t7.*t52.*t91+K_p_T.*gain_el.*t5.*t18.*t32.*t40).*2.0+t24.*t64.*t198.*(gain_el.*t6.*t44.*t91+gain_el.*t3.*t52.*t91).*2.0-t28.*t57.*t187.*(gain_el.*t44.*t87-gain_el.*l_4.*t52.*t91+K_p_T.*gain_el.*l_1.*t18.*t32.*t40).*2.0+t27.*t56.*t196.*(gain_el.*t52.*t87+gain_el.*l_4.*t44.*t91-K_p_T.*gain_el.*l_z.*t18.*t32.*t40).*2.0+t26.*t55.*t191.*(-gain_el.*t44.*t97+gain_el.*t52.*t101+K_p_M.*gain_el.*t18.*t32.*t40).*2.0]; */
  /*      mt3 = [gamma_quadratic_du.*t29.*(b_3_scaled.*2.0+t79)-t25.*t64.*(gain_el.*t3.*t5.*t45.*t92-gain_el.*t5.*t6.*t53.*t92+K_p_T.*gain_el.*t7.*t19.*t32.*t41).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0+t23.*t64.*t202.*(-gain_el.*t3.*t7.*t45.*t92+gain_el.*t6.*t7.*t53.*t92+K_p_T.*gain_el.*t5.*t19.*t32.*t41).*2.0+t24.*t64.*t198.*(gain_el.*t6.*t45.*t92+gain_el.*t3.*t53.*t92).*2.0-t28.*t57.*t187.*(-gain_el.*t45.*t88+gain_el.*l_3.*t53.*t92+K_p_T.*gain_el.*l_2.*t19.*t32.*t41).*2.0-t27.*t56.*t196.*(gain_el.*t53.*t88+gain_el.*l_3.*t45.*t92+K_p_T.*gain_el.*l_z.*t19.*t32.*t41).*2.0-t26.*t55.*t191.*(gain_el.*t45.*t98-gain_el.*t53.*t102+K_p_M.*gain_el.*t19.*t32.*t41).*2.0]; */
  /*      mt4 = [gamma_quadratic_du.*t29.*(b_4_scaled.*2.0+t79)-t25.*t64.*(gain_el.*t3.*t5.*t46.*t93-gain_el.*t5.*t6.*t54.*t93+K_p_T.*gain_el.*t7.*t20.*t32.*t42).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0+t23.*t64.*t202.*(-gain_el.*t3.*t7.*t46.*t93+gain_el.*t6.*t7.*t54.*t93+K_p_T.*gain_el.*t5.*t20.*t32.*t42).*2.0+t24.*t64.*t198.*(gain_el.*t6.*t46.*t93+gain_el.*t3.*t54.*t93).*2.0-t28.*t57.*t187.*(gain_el.*t46.*t89+gain_el.*l_3.*t54.*t93-K_p_T.*gain_el.*l_2.*t20.*t32.*t42).*2.0-t27.*t56.*t196.*(gain_el.*t54.*t105+gain_el.*l_3.*t46.*t93+K_p_T.*gain_el.*l_z.*t20.*t32.*t42).*2.0+t26.*t55.*t191.*(gain_el.*t46.*t99+gain_el.*t54.*t103+K_p_M.*gain_el.*t20.*t32.*t42).*2.0]; */
  /*      mt5 = [gamma_quadratic_du.*t30.*(g_1_scaled.*2.0+t81)-t25.*t64.*(gain_az.*t5.*t6.*t112+gain_az.*t3.*t5.*t120).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0-t23.*t64.*t202.*(gain_az.*t6.*t7.*t112+gain_az.*t3.*t7.*t120).*2.0+t26.*t55.*t191.*(gain_az.*l_1.*t120-gain_az.*l_z.*t112).*2.0-t24.*t64.*t198.*(gain_az.*t3.*t112-gain_az.*t6.*t120).*2.0+t28.*t57.*t187.*(gain_az.*t117-gain_az.*t126).*2.0+t27.*t56.*t196.*(gain_az.*t109+gain_az.*t133).*2.0]; */
  /*      mt6 = [gamma_quadratic_du.*t30.*(g_2_scaled.*2.0+t81)-t25.*t64.*(gain_az.*t5.*t6.*t114+gain_az.*t3.*t5.*t122).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0-t23.*t64.*t202.*(gain_az.*t6.*t7.*t114+gain_az.*t3.*t7.*t122).*2.0-t26.*t55.*t191.*(gain_az.*l_1.*t122+gain_az.*l_z.*t114).*2.0-t24.*t64.*t198.*(gain_az.*t3.*t114-gain_az.*t6.*t122).*2.0-t28.*t57.*t187.*(gain_az.*t118+gain_az.*t128).*2.0-t27.*t56.*t196.*(gain_az.*t110-gain_az.*t134).*2.0]; */
  /*      mt7 = [gamma_quadratic_du.*t30.*(g_3_scaled.*2.0+t81)-t25.*t64.*(gain_az.*t5.*t6.*t115+gain_az.*t3.*t5.*t123).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0-t23.*t64.*t202.*(gain_az.*t6.*t7.*t115+gain_az.*t3.*t7.*t123).*2.0-t26.*t55.*t191.*(gain_az.*l_2.*t123+gain_az.*l_z.*t115).*2.0-t24.*t64.*t198.*(gain_az.*t3.*t115-gain_az.*t6.*t123).*2.0+t28.*t57.*t187.*(gain_az.*t119+gain_az.*t130).*2.0+t27.*t56.*t196.*(gain_az.*t111+gain_az.*t149).*2.0]; */
  /*      mt8 = [gamma_quadratic_du.*t30.*(g_4_scaled.*2.0+t81)-t25.*t64.*(gain_az.*t5.*t6.*t116+gain_az.*t3.*t5.*t124).*(dv_global_3+t64.*(t157-t164+t170+t177-t179+t7.*(t153-t160))-9.81e+2./1.0e+2).*2.0-t23.*t64.*t202.*(gain_az.*t6.*t7.*t116+gain_az.*t3.*t7.*t124).*2.0+t26.*t55.*t191.*(gain_az.*l_2.*t124-gain_az.*l_z.*t116).*2.0-t24.*t64.*t198.*(gain_az.*t3.*t116-gain_az.*t6.*t124).*2.0-t27.*t56.*t196.*(gain_az.*t113+gain_az.*t136).*2.0-t28.*t57.*t187.*(gain_az.*t121+gain_az.*t148).*2.0;gamma_quadratic_du.*t31.*(delta_ailerons_scaled.*2.0-desired_ailerons_value.*t63.*2.0)-CL_aileron.*S.*gain_ailerons.*rho.*t21.*t26.*t55.*t191]; */
  /*      gradient = [mt1;mt2;mt3;mt4;mt5;mt6;mt7;mt8]; */
  /*  previous_theta_value  = previous_controls(13); */
  /*  previous_phi_value    = previous_controls(14); */
  /*  gamma_quadratic_du */
  /*  cost = W_dv_1^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)^2 + (W_dv_3^2*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981)^2)/10000 + gamma_quadratic_du2*(W_act_motor1^2*(Omega_1_scaled - previous_motor_value1/gain_motor)^2 + W_act_motor2^2*(Omega_2_scaled - previous_motor_value2/gain_motor)^2 + W_act_motor3^2*(Omega_3_scaled - previous_motor_value3/gain_motor)^2 + W_act_motor4^2*(Omega_4_scaled - previous_motor_value4/gain_motor)^2 + W_act_tilt_el1^2*(b_1_scaled - previous_el_value1/gain_el)^2 + W_act_tilt_el2^2*(b_2_scaled - previous_el_value2/gain_el)^2 + W_act_tilt_el3^2*(b_3_scaled - previous_el_value3/gain_el)^2 + W_act_tilt_el4^2*(b_4_scaled - previous_el_value4/gain_el)^2 + W_act_ailerons^2*(delta_ailerons_scaled - previous_ailerons_value/gain_ailerons)^2 + W_act_tilt_az1^2*(g_1_scaled - previous_az_value1/gain_az)^2 + W_act_tilt_az2^2*(g_2_scaled - previous_az_value2/gain_az)^2 + W_act_tilt_az3^2*(g_3_scaled - previous_az_value3/gain_az)^2 + W_act_tilt_az4^2*(g_4_scaled - previous_az_value4/gain_az)^2) + W_dv_2^2*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)^2 + gamma_quadratic_du*(W_act_motor1^2*(Omega_1_scaled - desired_motor_value/gain_motor)^2 + W_act_motor2^2*(Omega_2_scaled - desired_motor_value/gain_motor)^2 + W_act_motor3^2*(Omega_3_scaled - desired_motor_value/gain_motor)^2 + W_act_motor4^2*(Omega_4_scaled - desired_motor_value/gain_motor)^2 + W_act_tilt_el1^2*(b_1_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el2^2*(b_2_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el3^2*(b_3_scaled - desired_el_value/gain_el)^2 + W_act_tilt_el4^2*(b_4_scaled - desired_el_value/gain_el)^2 + W_act_ailerons^2*(delta_ailerons_scaled - desired_ailerons_value/gain_ailerons)^2 + W_act_tilt_az1^2*(g_1_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az2^2*(g_2_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az3^2*(g_3_scaled - desired_az_value/gain_az)^2 + W_act_tilt_az4^2*(g_4_scaled - desired_az_value/gain_az)^2) + (W_dv_4^2*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho)^2)/(4*I_xx^2) + (W_dv_6^2*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))^2)/I_zz^2 + (W_dv_5^2*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord)^2)/(4*I_yy^2); */
  /*   */
  /*  if nargout > 1 */
  /*   */
  /*  gradient =                  [ (2*Omega_1_scaled*W_dv_5^2*gain_motor^2*(K_p_T*l_z*sin(b_1_scaled*gain_el) - K_p_M*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + K_p_T*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_motor1^2*gamma_quadratic_du2*(previous_motor_value1 - Omega_1_scaled*gain_motor))/gain_motor - (4*Omega_1_scaled*W_dv_6^2*gain_motor^2*(K_p_M*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - K_p_T*l_1*sin(b_1_scaled*gain_el) + K_p_T*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*W_act_motor1^2*gamma_quadratic_du*(desired_motor_value - Omega_1_scaled*gain_motor))/gain_motor + (2*Omega_1_scaled*W_dv_4^2*gain_motor^2*(K_p_M*sin(b_1_scaled*gain_el) + K_p_T*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_T*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 + (4*K_p_T*Omega_1_scaled*W_dv_1^2*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(sin(b_1_scaled*gain_el)*cos(Theta) - cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*sin(Phi)*sin(Theta) + cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*cos(Phi)*sin(Theta)))/m + (K_p_T*Omega_1_scaled*W_dv_3^2*gain_motor^2*(sin(b_1_scaled*gain_el)*sin(Theta) + cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*cos(Theta)*sin(Phi) - cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(25*m) - (4*K_p_T*Omega_1_scaled*W_dv_2^2*gain_motor^2*sin(Phi + g_1_scaled*gain_az)*cos(b_1_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*                                    (2*Omega_2_scaled*W_dv_5^2*gain_motor^2*(K_p_T*l_z*sin(b_2_scaled*gain_el) + K_p_M*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_motor2^2*gamma_quadratic_du2*(previous_motor_value2 - Omega_2_scaled*gain_motor))/gain_motor - (4*Omega_2_scaled*W_dv_6^2*gain_motor^2*(K_p_T*l_1*sin(b_2_scaled*gain_el) - K_p_M*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) + K_p_T*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*W_act_motor2^2*gamma_quadratic_du*(desired_motor_value - Omega_2_scaled*gain_motor))/gain_motor - (2*Omega_2_scaled*W_dv_4^2*gain_motor^2*(K_p_M*sin(b_2_scaled*gain_el) + K_p_T*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_T*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 + (4*K_p_T*Omega_2_scaled*W_dv_1^2*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(sin(b_2_scaled*gain_el)*cos(Theta) - cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*sin(Phi)*sin(Theta) + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*cos(Phi)*sin(Theta)))/m + (K_p_T*Omega_2_scaled*W_dv_3^2*gain_motor^2*(sin(b_2_scaled*gain_el)*sin(Theta) + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*cos(Theta)*sin(Phi) - cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(25*m) - (4*K_p_T*Omega_2_scaled*W_dv_2^2*gain_motor^2*sin(Phi + g_2_scaled*gain_az)*cos(b_2_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*                                    (2*Omega_3_scaled*W_dv_4^2*gain_motor^2*(K_p_M*sin(b_3_scaled*gain_el) - K_p_T*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_T*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*W_act_motor3^2*gamma_quadratic_du2*(previous_motor_value3 - Omega_3_scaled*gain_motor))/gain_motor - (4*Omega_3_scaled*W_dv_6^2*gain_motor^2*(K_p_T*l_2*sin(b_3_scaled*gain_el) + K_p_M*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - K_p_T*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*Omega_3_scaled*W_dv_5^2*gain_motor^2*(K_p_M*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) - K_p_T*l_z*sin(b_3_scaled*gain_el) + K_p_T*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_motor3^2*gamma_quadratic_du*(desired_motor_value - Omega_3_scaled*gain_motor))/gain_motor + (4*K_p_T*Omega_3_scaled*W_dv_1^2*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(sin(b_3_scaled*gain_el)*cos(Theta) - cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*sin(Phi)*sin(Theta) + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*cos(Phi)*sin(Theta)))/m + (K_p_T*Omega_3_scaled*W_dv_3^2*gain_motor^2*(sin(b_3_scaled*gain_el)*sin(Theta) + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*cos(Theta)*sin(Phi) - cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(25*m) - (4*K_p_T*Omega_3_scaled*W_dv_2^2*gain_motor^2*sin(Phi + g_3_scaled*gain_az)*cos(b_3_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*                                    (4*Omega_4_scaled*W_dv_6^2*gain_motor^2*(K_p_T*l_2*sin(b_4_scaled*gain_el) + K_p_M*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + K_p_T*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*W_act_motor4^2*gamma_quadratic_du2*(previous_motor_value4 - Omega_4_scaled*gain_motor))/gain_motor - (2*W_act_motor4^2*gamma_quadratic_du*(desired_motor_value - Omega_4_scaled*gain_motor))/gain_motor + (2*Omega_4_scaled*W_dv_5^2*gain_motor^2*(K_p_T*l_z*sin(b_4_scaled*gain_el) + K_p_M*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) - K_p_T*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 + (2*Omega_4_scaled*W_dv_4^2*gain_motor^2*(K_p_T*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_M*sin(b_4_scaled*gain_el) + K_p_T*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 + (4*K_p_T*Omega_4_scaled*W_dv_1^2*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(sin(b_4_scaled*gain_el)*cos(Theta) - cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*sin(Phi)*sin(Theta) + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*cos(Phi)*sin(Theta)))/m + (K_p_T*Omega_4_scaled*W_dv_3^2*gain_motor^2*(sin(b_4_scaled*gain_el)*sin(Theta) + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*cos(Theta)*sin(Phi) - cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(25*m) - (4*K_p_T*Omega_4_scaled*W_dv_2^2*gain_motor^2*sin(Phi + g_4_scaled*gain_az)*cos(b_4_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*  (Omega_1_scaled^2*W_dv_5^2*gain_el*gain_motor^2*(K_p_T*l_z*cos(b_1_scaled*gain_el) + K_p_M*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*l_4*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_tilt_el1^2*gamma_quadratic_du2*(previous_el_value1 - b_1_scaled*gain_el))/gain_el - (2*W_act_tilt_el1^2*gamma_quadratic_du*(desired_el_value - b_1_scaled*gain_el))/gain_el - (Omega_1_scaled^2*W_dv_4^2*gain_el*gain_motor^2*(K_p_T*l_1*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) - K_p_M*cos(b_1_scaled*gain_el) + K_p_T*l_z*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 + (2*Omega_1_scaled^2*W_dv_6^2*gain_el*gain_motor^2*(K_p_T*l_1*cos(b_1_scaled*gain_el) + K_p_M*cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el) + K_p_T*l_4*sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 + (K_p_T*Omega_1_scaled^2*W_dv_3^2*gain_el*gain_motor^2*(cos(b_1_scaled*gain_el)*sin(Theta) - sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*cos(Theta)*sin(Phi) + cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m) + (2*K_p_T*Omega_1_scaled^2*W_dv_1^2*gain_el*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(cos(b_1_scaled*gain_el)*cos(Theta) - cos(g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*cos(Phi)*sin(Theta) + sin(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*sin(Phi)*sin(Theta)))/m + (2*K_p_T*Omega_1_scaled^2*W_dv_2^2*gain_el*gain_motor^2*sin(Phi + g_1_scaled*gain_az)*sin(b_1_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*  (K_p_T*Omega_2_scaled^2*W_dv_3^2*gain_el*gain_motor^2*(cos(b_2_scaled*gain_el)*sin(Theta) - sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*cos(Theta)*sin(Phi) + cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m) - (2*W_act_tilt_el2^2*gamma_quadratic_du2*(previous_el_value2 - b_2_scaled*gain_el))/gain_el - (Omega_2_scaled^2*W_dv_5^2*gain_el*gain_motor^2*(K_p_M*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - K_p_T*l_z*cos(b_2_scaled*gain_el) + K_p_T*l_4*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (Omega_2_scaled^2*W_dv_4^2*gain_el*gain_motor^2*(K_p_M*cos(b_2_scaled*gain_el) - K_p_T*l_1*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) + K_p_T*l_z*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*Omega_2_scaled^2*W_dv_6^2*gain_el*gain_motor^2*(K_p_T*l_1*cos(b_2_scaled*gain_el) + K_p_M*cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el) - K_p_T*l_4*sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*W_act_tilt_el2^2*gamma_quadratic_du*(desired_el_value - b_2_scaled*gain_el))/gain_el + (2*K_p_T*Omega_2_scaled^2*W_dv_1^2*gain_el*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(cos(b_2_scaled*gain_el)*cos(Theta) - cos(g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*cos(Phi)*sin(Theta) + sin(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*sin(Phi)*sin(Theta)))/m + (2*K_p_T*Omega_2_scaled^2*W_dv_2^2*gain_el*gain_motor^2*sin(Phi + g_2_scaled*gain_az)*sin(b_2_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*  (Omega_3_scaled^2*W_dv_5^2*gain_el*gain_motor^2*(K_p_T*l_z*cos(b_3_scaled*gain_el) + K_p_M*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*l_3*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_tilt_el3^2*gamma_quadratic_du2*(previous_el_value3 - b_3_scaled*gain_el))/gain_el - (2*W_act_tilt_el3^2*gamma_quadratic_du*(desired_el_value - b_3_scaled*gain_el))/gain_el + (Omega_3_scaled^2*W_dv_4^2*gain_el*gain_motor^2*(K_p_M*cos(b_3_scaled*gain_el) + K_p_T*l_2*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) - K_p_T*l_z*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*Omega_3_scaled^2*W_dv_6^2*gain_el*gain_motor^2*(K_p_T*l_2*cos(b_3_scaled*gain_el) - K_p_M*cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el) + K_p_T*l_3*sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 + (K_p_T*Omega_3_scaled^2*W_dv_3^2*gain_el*gain_motor^2*(cos(b_3_scaled*gain_el)*sin(Theta) - sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*cos(Theta)*sin(Phi) + cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m) + (2*K_p_T*Omega_3_scaled^2*W_dv_1^2*gain_el*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(cos(b_3_scaled*gain_el)*cos(Theta) - cos(g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*cos(Phi)*sin(Theta) + sin(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*sin(Phi)*sin(Theta)))/m + (2*K_p_T*Omega_3_scaled^2*W_dv_2^2*gain_el*gain_motor^2*sin(Phi + g_3_scaled*gain_az)*sin(b_3_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*  (Omega_4_scaled^2*W_dv_5^2*gain_el*gain_motor^2*(K_p_T*l_z*cos(b_4_scaled*gain_el) - K_p_M*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + K_p_T*l_3*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_tilt_el4^2*gamma_quadratic_du2*(previous_el_value4 - b_4_scaled*gain_el))/gain_el - (2*W_act_tilt_el4^2*gamma_quadratic_du*(desired_el_value - b_4_scaled*gain_el))/gain_el - (Omega_4_scaled^2*W_dv_4^2*gain_el*gain_motor^2*(K_p_M*cos(b_4_scaled*gain_el) + K_p_T*l_2*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) + K_p_T*l_z*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*Omega_4_scaled^2*W_dv_6^2*gain_el*gain_motor^2*(K_p_M*cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el) - K_p_T*l_2*cos(b_4_scaled*gain_el) + K_p_T*l_3*sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 + (K_p_T*Omega_4_scaled^2*W_dv_3^2*gain_el*gain_motor^2*(cos(b_4_scaled*gain_el)*sin(Theta) - sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*cos(Theta)*sin(Phi) + cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*cos(Phi)*cos(Theta))*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m) + (2*K_p_T*Omega_4_scaled^2*W_dv_1^2*gain_el*gain_motor^2*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m)*(cos(b_4_scaled*gain_el)*cos(Theta) - cos(g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*cos(Phi)*sin(Theta) + sin(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*sin(Phi)*sin(Theta)))/m + (2*K_p_T*Omega_4_scaled^2*W_dv_2^2*gain_el*gain_motor^2*sin(Phi + g_4_scaled*gain_az)*sin(b_4_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m; */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                        (2*Omega_1_scaled^2*W_dv_6^2*gain_az*gain_motor^2*cos(b_1_scaled*gain_el)*(K_p_M*sin(g_1_scaled*gain_az) - K_p_T*l_4*cos(g_1_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*W_act_tilt_az1^2*gamma_quadratic_du2*(previous_az_value1 - g_1_scaled*gain_az))/gain_az - (Omega_1_scaled^2*W_dv_5^2*gain_az*gain_motor^2*cos(b_1_scaled*gain_el)*(K_p_M*cos(g_1_scaled*gain_az) + K_p_T*l_4*sin(g_1_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_tilt_az1^2*gamma_quadratic_du*(desired_az_value - g_1_scaled*gain_az))/gain_az - (2*K_p_T*Omega_1_scaled^2*W_dv_2^2*gain_az*gain_motor^2*cos(Phi + g_1_scaled*gain_az)*cos(b_1_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_1_scaled^2*W_dv_4^2*gain_az*gain_motor^2*cos(b_1_scaled*gain_el)*(l_z*cos(g_1_scaled*gain_az) - l_1*sin(g_1_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*K_p_T*Omega_1_scaled^2*W_dv_1^2*gain_az*gain_motor^2*sin(Phi + g_1_scaled*gain_az)*cos(b_1_scaled*gain_el)*sin(Theta)*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_1_scaled^2*W_dv_3^2*gain_az*gain_motor^2*sin(Phi + g_1_scaled*gain_az)*cos(b_1_scaled*gain_el)*cos(Theta)*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                        (Omega_2_scaled^2*W_dv_5^2*gain_az*gain_motor^2*cos(b_2_scaled*gain_el)*(K_p_M*cos(g_2_scaled*gain_az) - K_p_T*l_4*sin(g_2_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_tilt_az2^2*gamma_quadratic_du2*(previous_az_value2 - g_2_scaled*gain_az))/gain_az - (2*W_act_tilt_az2^2*gamma_quadratic_du*(desired_az_value - g_2_scaled*gain_az))/gain_az - (2*Omega_2_scaled^2*W_dv_6^2*gain_az*gain_motor^2*cos(b_2_scaled*gain_el)*(K_p_M*sin(g_2_scaled*gain_az) + K_p_T*l_4*cos(g_2_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*K_p_T*Omega_2_scaled^2*W_dv_2^2*gain_az*gain_motor^2*cos(Phi + g_2_scaled*gain_az)*cos(b_2_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_2_scaled^2*W_dv_4^2*gain_az*gain_motor^2*cos(b_2_scaled*gain_el)*(l_z*cos(g_2_scaled*gain_az) + l_1*sin(g_2_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*K_p_T*Omega_2_scaled^2*W_dv_1^2*gain_az*gain_motor^2*sin(Phi + g_2_scaled*gain_az)*cos(b_2_scaled*gain_el)*sin(Theta)*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_2_scaled^2*W_dv_3^2*gain_az*gain_motor^2*sin(Phi + g_2_scaled*gain_az)*cos(b_2_scaled*gain_el)*cos(Theta)*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                        (2*Omega_3_scaled^2*W_dv_6^2*gain_az*gain_motor^2*cos(b_3_scaled*gain_el)*(K_p_M*sin(g_3_scaled*gain_az) + K_p_T*l_3*cos(g_3_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*W_act_tilt_az3^2*gamma_quadratic_du2*(previous_az_value3 - g_3_scaled*gain_az))/gain_az - (Omega_3_scaled^2*W_dv_5^2*gain_az*gain_motor^2*cos(b_3_scaled*gain_el)*(K_p_M*cos(g_3_scaled*gain_az) - K_p_T*l_3*sin(g_3_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_tilt_az3^2*gamma_quadratic_du*(desired_az_value - g_3_scaled*gain_az))/gain_az - (2*K_p_T*Omega_3_scaled^2*W_dv_2^2*gain_az*gain_motor^2*cos(Phi + g_3_scaled*gain_az)*cos(b_3_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_3_scaled^2*W_dv_4^2*gain_az*gain_motor^2*cos(b_3_scaled*gain_el)*(l_z*cos(g_3_scaled*gain_az) + l_2*sin(g_3_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*K_p_T*Omega_3_scaled^2*W_dv_1^2*gain_az*gain_motor^2*sin(Phi + g_3_scaled*gain_az)*cos(b_3_scaled*gain_el)*sin(Theta)*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_3_scaled^2*W_dv_3^2*gain_az*gain_motor^2*sin(Phi + g_3_scaled*gain_az)*cos(b_3_scaled*gain_el)*cos(Theta)*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                        (Omega_4_scaled^2*W_dv_5^2*gain_az*gain_motor^2*cos(b_4_scaled*gain_el)*(K_p_M*cos(g_4_scaled*gain_az) + K_p_T*l_3*sin(g_4_scaled*gain_az))*(2*I_zz*p*r - 2*I_xx*p*r - 2*I_yy*dv_global_5 + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*sin(b_1_scaled*gain_el) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*sin(b_2_scaled*gain_el) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*sin(b_3_scaled*gain_el) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*sin(b_4_scaled*gain_el) - 2*K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) - 2*K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + Cm_zero*S*V^2*rho*wing_chord + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) - 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + Cm_alpha*S*Theta*V^2*rho*wing_chord - Cm_alpha*S*V^2*flight_path_angle*rho*wing_chord))/I_yy^2 - (2*W_act_tilt_az4^2*gamma_quadratic_du2*(previous_az_value4 - g_4_scaled*gain_az))/gain_az - (2*W_act_tilt_az4^2*gamma_quadratic_du*(desired_az_value - g_4_scaled*gain_az))/gain_az - (2*Omega_4_scaled^2*W_dv_6^2*gain_az*gain_motor^2*cos(b_4_scaled*gain_el)*(K_p_M*sin(g_4_scaled*gain_az) - K_p_T*l_3*cos(g_4_scaled*gain_az))*(I_zz*dv_global_6 - I_xx*p*q + I_yy*p*q + K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*sin(b_1_scaled*gain_el) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*sin(b_2_scaled*gain_el) - K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*sin(b_3_scaled*gain_el) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*sin(b_4_scaled*gain_el) - K_p_M*Omega_1_scaled^2*gain_motor^2*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) + K_p_M*Omega_2_scaled^2*gain_motor^2*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - K_p_M*Omega_3_scaled^2*gain_motor^2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + K_p_M*Omega_4_scaled^2*gain_motor^2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) - K_p_T*Omega_1_scaled^2*gain_motor^2*l_4*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) - K_p_T*Omega_2_scaled^2*gain_motor^2*l_4*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + K_p_T*Omega_3_scaled^2*gain_motor^2*l_3*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + K_p_T*Omega_4_scaled^2*gain_motor^2*l_3*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)))/I_zz^2 - (2*K_p_T*Omega_4_scaled^2*W_dv_2^2*gain_az*gain_motor^2*cos(Phi + g_4_scaled*gain_az)*cos(b_4_scaled*gain_el)*(dv_global_2 - (sin(Phi)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Phi)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*cos(Phi)*sin(Beta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_4_scaled^2*W_dv_4^2*gain_az*gain_motor^2*cos(b_4_scaled*gain_el)*(l_z*cos(g_4_scaled*gain_az) - l_2*sin(g_4_scaled*gain_az))*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/I_xx^2 - (2*K_p_T*Omega_4_scaled^2*W_dv_1^2*gain_az*gain_motor^2*sin(Phi + g_4_scaled*gain_az)*cos(b_4_scaled*gain_el)*sin(Theta)*(dv_global_1 + (cos(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + cos(Phi)*sin(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*cos(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) - K_p_T*gain_motor^2*sin(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Phi)*sin(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) + (S*V^2*rho*sin(Beta)*sin(Phi)*sin(Theta)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2)/m))/m + (K_p_T*Omega_4_scaled^2*W_dv_3^2*gain_az*gain_motor^2*sin(Phi + g_4_scaled*gain_az)*cos(b_4_scaled*gain_el)*cos(Theta)*((100*(sin(Theta)*((S*V^2*rho*cos(Beta)*cos(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 - (Cl_alpha*S*V^2*rho*sin(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) - cos(Phi)*cos(Theta)*((S*V^2*rho*cos(Beta)*sin(Theta - flight_path_angle)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2 + (Cl_alpha*S*V^2*rho*cos(Theta - flight_path_angle)*(Theta - flight_path_angle))/2) + K_p_T*gain_motor^2*sin(Theta)*(sin(b_1_scaled*gain_el)*Omega_1_scaled^2 + sin(b_2_scaled*gain_el)*Omega_2_scaled^2 + sin(b_3_scaled*gain_el)*Omega_3_scaled^2 + sin(b_4_scaled*gain_el)*Omega_4_scaled^2) + K_p_T*gain_motor^2*cos(Theta)*sin(Phi)*(cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az)*Omega_4_scaled^2) - K_p_T*gain_motor^2*cos(Phi)*cos(Theta)*(cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az)*Omega_1_scaled^2 + cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az)*Omega_2_scaled^2 + cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az)*Omega_3_scaled^2 + cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az)*Omega_4_scaled^2) - (S*V^2*rho*sin(Beta)*cos(Theta)*sin(Phi)*(K_Cd*Cl_alpha^2*Theta^2 - 2*K_Cd*Cl_alpha^2*Theta*flight_path_angle + K_Cd*Cl_alpha^2*flight_path_angle^2 + Cd_zero))/2))/m - 100*dv_global_3 + 981))/(50*m); */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              W_act_ailerons^2*gamma_quadratic_du*(2*delta_ailerons_scaled - (2*desired_ailerons_value)/gain_ailerons) + W_act_ailerons^2*gamma_quadratic_du2*(2*delta_ailerons_scaled - (2*previous_ailerons_value)/gain_ailerons) + (CL_aileron*S*V^2*W_dv_4^2*gain_ailerons*rho*(2*I_yy*q*r - 2*I_xx*dv_global_4 - 2*I_zz*q*r + 2*K_p_M*Omega_1_scaled^2*gain_motor^2*sin(b_1_scaled*gain_el) - 2*K_p_M*Omega_2_scaled^2*gain_motor^2*sin(b_2_scaled*gain_el) + 2*K_p_M*Omega_3_scaled^2*gain_motor^2*sin(b_3_scaled*gain_el) - 2*K_p_M*Omega_4_scaled^2*gain_motor^2*sin(b_4_scaled*gain_el) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_1*cos(b_1_scaled*gain_el)*cos(g_1_scaled*gain_az) - 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_1*cos(b_2_scaled*gain_el)*cos(g_2_scaled*gain_az) - 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_2*cos(b_3_scaled*gain_el)*cos(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_2*cos(b_4_scaled*gain_el)*cos(g_4_scaled*gain_az) + 2*K_p_T*Omega_1_scaled^2*gain_motor^2*l_z*cos(b_1_scaled*gain_el)*sin(g_1_scaled*gain_az) + 2*K_p_T*Omega_2_scaled^2*gain_motor^2*l_z*cos(b_2_scaled*gain_el)*sin(g_2_scaled*gain_az) + 2*K_p_T*Omega_3_scaled^2*gain_motor^2*l_z*cos(b_3_scaled*gain_el)*sin(g_3_scaled*gain_az) + 2*K_p_T*Omega_4_scaled^2*gain_motor^2*l_z*cos(b_4_scaled*gain_el)*sin(g_4_scaled*gain_az) + CL_aileron*S*V^2*delta_ailerons_scaled*gain_ailerons*rho))/(2*I_xx^2)]; */
  a_tmp = Cl_alpha * Cl_alpha;
  b_a_tmp = K_Cd * a_tmp;
  c_a_tmp = V * V;
  d_a_tmp = Theta - flight_path_angle;
  a_tmp_tmp = S * c_a_tmp * rho;
  e_a_tmp = a_tmp_tmp * cos(Beta);
  f_a_tmp = sin(d_a_tmp);
  a_tmp = ((b_a_tmp * (Theta * Theta) - 2.0 * K_Cd * a_tmp * Theta *
            flight_path_angle) + b_a_tmp * (flight_path_angle *
            flight_path_angle)) + Cd_zero;
  b_a_tmp = Cl_alpha * S * c_a_tmp * rho;
  g_a_tmp = cos(d_a_tmp);
  h_a_tmp = cos(Theta);
  b_a_tmp_tmp = gain_motor * gain_motor;
  i_a_tmp = K_p_T * b_a_tmp_tmp;
  j_a_tmp = sin(Theta);
  k_a_tmp = cos(Phi);
  l_a_tmp = sin(Phi);
  m_a_tmp = e_a_tmp * g_a_tmp * a_tmp / 2.0 - b_a_tmp * f_a_tmp * d_a_tmp / 2.0;
  b_a_tmp = e_a_tmp * f_a_tmp * a_tmp / 2.0 + b_a_tmp * g_a_tmp * d_a_tmp / 2.0;
  d_a_tmp = i_a_tmp * h_a_tmp;
  e_a_tmp = i_a_tmp * k_a_tmp;
  c_a_tmp_tmp = sin(Beta);
  f_a_tmp = a_tmp_tmp * c_a_tmp_tmp;
  g_a_tmp = i_a_tmp * l_a_tmp;
  n_a_tmp = h_a_tmp * m_a_tmp + k_a_tmp * j_a_tmp * b_a_tmp;
  o_a_tmp = g_a_tmp * j_a_tmp;
  p_a_tmp = e_a_tmp * j_a_tmp;
  q_a_tmp = f_a_tmp * l_a_tmp * j_a_tmp * a_tmp / 2.0;
  r_a_tmp = x[4] * gain_el;
  s_a_tmp = x[0] * x[0];
  t_a_tmp = x[5] * gain_el;
  u_a_tmp = x[1] * x[1];
  v_a_tmp = x[6] * gain_el;
  w_a_tmp = x[2] * x[2];
  x_a_tmp = x[7] * gain_el;
  y_a_tmp = x[3] * x[3];
  ab_a_tmp = x[8] * gain_az;
  bb_a_tmp = cos(r_a_tmp);
  cb_a_tmp = x[9] * gain_az;
  db_a_tmp = cos(t_a_tmp);
  eb_a_tmp = x[10] * gain_az;
  fb_a_tmp = cos(v_a_tmp);
  gb_a_tmp = x[11] * gain_az;
  hb_a_tmp = cos(x_a_tmp);
  d_a_tmp_tmp = sin(r_a_tmp);
  e_a_tmp_tmp = sin(t_a_tmp);
  f_a_tmp_tmp = sin(v_a_tmp);
  g_a_tmp_tmp = sin(x_a_tmp);
  ib_a_tmp = ((d_a_tmp_tmp * s_a_tmp + e_a_tmp_tmp * u_a_tmp) + f_a_tmp_tmp *
              w_a_tmp) + g_a_tmp_tmp * y_a_tmp;
  h_a_tmp_tmp = sin(ab_a_tmp);
  i_a_tmp_tmp = sin(cb_a_tmp);
  j_a_tmp_tmp = sin(eb_a_tmp);
  k_a_tmp_tmp = sin(gb_a_tmp);
  l_a_tmp_tmp = bb_a_tmp * h_a_tmp_tmp;
  m_a_tmp_tmp = db_a_tmp * i_a_tmp_tmp;
  n_a_tmp_tmp = fb_a_tmp * j_a_tmp_tmp;
  o_a_tmp_tmp = hb_a_tmp * k_a_tmp_tmp;
  jb_a_tmp = ((l_a_tmp_tmp * s_a_tmp + m_a_tmp_tmp * u_a_tmp) + n_a_tmp_tmp *
              w_a_tmp) + o_a_tmp_tmp * y_a_tmp;
  p_a_tmp_tmp = cos(ab_a_tmp);
  q_a_tmp_tmp = cos(cb_a_tmp);
  r_a_tmp_tmp = cos(eb_a_tmp);
  s_a_tmp_tmp = cos(gb_a_tmp);
  t_a_tmp_tmp = bb_a_tmp * p_a_tmp_tmp;
  u_a_tmp_tmp = db_a_tmp * q_a_tmp_tmp;
  v_a_tmp_tmp = fb_a_tmp * r_a_tmp_tmp;
  w_a_tmp_tmp = hb_a_tmp * s_a_tmp_tmp;
  kb_a_tmp = ((t_a_tmp_tmp * s_a_tmp + u_a_tmp_tmp * u_a_tmp) + v_a_tmp_tmp *
              w_a_tmp) + w_a_tmp_tmp * y_a_tmp;
  a = dv_global_1 + ((((n_a_tmp + d_a_tmp * ib_a_tmp) - o_a_tmp * jb_a_tmp) +
                      p_a_tmp * kb_a_tmp) + q_a_tmp) / m;
  m_a_tmp = j_a_tmp * m_a_tmp - k_a_tmp * h_a_tmp * b_a_tmp;
  i_a_tmp *= j_a_tmp;
  lb_a_tmp = d_a_tmp * l_a_tmp;
  mb_a_tmp = e_a_tmp * h_a_tmp;
  f_a_tmp = f_a_tmp * h_a_tmp * l_a_tmp * a_tmp / 2.0;
  b_a = (100.0 * ((((m_a_tmp + i_a_tmp * ib_a_tmp) + lb_a_tmp * jb_a_tmp) -
                   mb_a_tmp * kb_a_tmp) - f_a_tmp) / m - 100.0 * dv_global_3) +
    981.0;
  c_a = x[0] - previous_controls[0] / gain_motor;
  d_a = x[1] - previous_controls[1] / gain_motor;
  e_a = x[2] - previous_controls[2] / gain_motor;
  f_a = x[3] - previous_controls[3] / gain_motor;
  g_a = x[4] - previous_controls[4] / gain_el;
  h_a = x[5] - previous_controls[5] / gain_el;
  i_a = x[6] - previous_controls[6] / gain_el;
  j_a = x[7] - previous_controls[7] / gain_el;
  k_a = x[12] - previous_controls[14] / gain_ailerons;
  l_a = x[8] - previous_controls[8] / gain_az;
  m_a = x[9] - previous_controls[9] / gain_az;
  n_a = x[10] - previous_controls[10] / gain_az;
  o_a = x[11] - previous_controls[11] / gain_az;
  a_tmp = dv_global_2 - (((l_a_tmp * b_a_tmp + g_a_tmp * kb_a_tmp) + e_a_tmp *
    jb_a_tmp) - a_tmp_tmp * k_a_tmp * c_a_tmp_tmp * a_tmp / 2.0) / m;
  b_a_tmp = desired_motor_value / gain_motor;
  p_a = x[0] - b_a_tmp;
  q_a = x[1] - b_a_tmp;
  r_a = x[2] - b_a_tmp;
  s_a = x[3] - b_a_tmp;
  b_a_tmp = desired_el_value / gain_el;
  t_a = x[4] - b_a_tmp;
  u_a = x[5] - b_a_tmp;
  v_a = x[6] - b_a_tmp;
  w_a = x[7] - b_a_tmp;
  x_a = x[12] - desired_ailerons_value / gain_ailerons;
  b_a_tmp = desired_az_value / gain_az;
  y_a = x[8] - b_a_tmp;
  ab_a = x[9] - b_a_tmp;
  bb_a = x[10] - b_a_tmp;
  cb_a = x[11] - b_a_tmp;
  b_a_tmp = (2.0 * I_yy * q * r - 2.0 * I_xx * dv_global_4) - 2.0 * I_zz * q * r;
  e_a_tmp = CL_aileron * S * c_a_tmp;
  a_tmp_tmp = 2.0 * K_p_T * s_a_tmp;
  g_a_tmp = a_tmp_tmp * b_a_tmp_tmp;
  c_a_tmp_tmp = 2.0 * K_p_T * u_a_tmp;
  ib_a_tmp = c_a_tmp_tmp * b_a_tmp_tmp;
  x_a_tmp_tmp = 2.0 * K_p_T * w_a_tmp;
  jb_a_tmp = x_a_tmp_tmp * b_a_tmp_tmp;
  y_a_tmp_tmp = 2.0 * K_p_T * y_a_tmp;
  kb_a_tmp = y_a_tmp_tmp * b_a_tmp_tmp;
  Cd_zero = g_a_tmp * l_z;
  Cl_alpha = ib_a_tmp * l_z;
  K_Cd = jb_a_tmp * l_z;
  Beta = kb_a_tmp * l_z;
  nb_a_tmp = 2.0 * K_p_M * s_a_tmp * b_a_tmp_tmp;
  ob_a_tmp = 2.0 * K_p_M * u_a_tmp * b_a_tmp_tmp;
  pb_a_tmp = 2.0 * K_p_M * w_a_tmp * b_a_tmp_tmp;
  qb_a_tmp = 2.0 * K_p_M * y_a_tmp * b_a_tmp_tmp;
  db_a = ((((((((((((b_a_tmp + nb_a_tmp * d_a_tmp_tmp) - ob_a_tmp * e_a_tmp_tmp)
                   + pb_a_tmp * f_a_tmp_tmp) - qb_a_tmp * g_a_tmp_tmp) + g_a_tmp
                 * l_1 * bb_a_tmp * p_a_tmp_tmp) - ib_a_tmp * l_1 * db_a_tmp *
                q_a_tmp_tmp) - jb_a_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) +
              kb_a_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + Cd_zero * bb_a_tmp *
             h_a_tmp_tmp) + Cl_alpha * db_a_tmp * i_a_tmp_tmp) + K_Cd * fb_a_tmp
           * j_a_tmp_tmp) + Beta * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
    gain_ailerons * rho;
  rb_a_tmp = (I_zz * dv_global_6 - I_xx * p * q) + I_yy * p * q;
  ab_a_tmp_tmp = K_p_T * s_a_tmp;
  V = ab_a_tmp_tmp * b_a_tmp_tmp;
  bb_a_tmp_tmp = K_p_T * u_a_tmp;
  sb_a_tmp = bb_a_tmp_tmp * b_a_tmp_tmp;
  cb_a_tmp_tmp = K_p_T * w_a_tmp;
  tb_a_tmp = cb_a_tmp_tmp * b_a_tmp_tmp;
  db_a_tmp_tmp = K_p_T * y_a_tmp;
  ub_a_tmp = db_a_tmp_tmp * b_a_tmp_tmp;
  q = (((((((((((rb_a_tmp + V * l_1 * d_a_tmp_tmp) - sb_a_tmp * l_1 *
                e_a_tmp_tmp) - tb_a_tmp * l_2 * f_a_tmp_tmp) + ub_a_tmp * l_2 *
              g_a_tmp_tmp) - K_p_M * s_a_tmp * b_a_tmp_tmp * bb_a_tmp *
             p_a_tmp_tmp) + K_p_M * u_a_tmp * b_a_tmp_tmp * db_a_tmp *
            q_a_tmp_tmp) - K_p_M * w_a_tmp * b_a_tmp_tmp * fb_a_tmp *
           r_a_tmp_tmp) + K_p_M * y_a_tmp * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp)
         - V * l_4 * bb_a_tmp * h_a_tmp_tmp) - sb_a_tmp * l_4 * db_a_tmp *
        i_a_tmp_tmp) + tb_a_tmp * l_3 * fb_a_tmp * j_a_tmp_tmp) + ub_a_tmp * l_3
    * hb_a_tmp * k_a_tmp_tmp;
  V = Cm_alpha * S;
  sb_a_tmp = (2.0 * I_zz * p * r - 2.0 * I_xx * p * r) - 2.0 * I_yy *
    dv_global_5;
  tb_a_tmp = Cm_zero * S * c_a_tmp * rho * wing_chord;
  ub_a_tmp = V * Theta * c_a_tmp * rho * wing_chord;
  c_a_tmp = V * c_a_tmp * flight_path_angle * rho * wing_chord;
  V = ((((((((((((((sb_a_tmp + Cd_zero * d_a_tmp_tmp) + Cl_alpha * e_a_tmp_tmp)
                  + K_Cd * f_a_tmp_tmp) + Beta * g_a_tmp_tmp) - nb_a_tmp *
                bb_a_tmp * h_a_tmp_tmp) + ob_a_tmp * db_a_tmp * i_a_tmp_tmp) -
              pb_a_tmp * fb_a_tmp * j_a_tmp_tmp) + qb_a_tmp * hb_a_tmp *
             k_a_tmp_tmp) + tb_a_tmp) + g_a_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp)
          + ib_a_tmp * l_4 * db_a_tmp * q_a_tmp_tmp) - jb_a_tmp * l_3 * fb_a_tmp
         * r_a_tmp_tmp) - kb_a_tmp * l_3 * hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) -
    c_a_tmp;
  CL_aileron = W_act_motor_du * W_act_motor_du;
  dv_global_6 = W_act_tilt_el_du * W_act_tilt_el_du;
  fval_tmp = W_act_tilt_az_du * W_act_tilt_az_du;
  b_fval_tmp = W_dv_5 * W_dv_5;
  c_fval_tmp = I_yy * I_yy;
  d_fval_tmp = W_dv_6 * W_dv_6;
  e_fval_tmp = I_zz * I_zz;
  dv_global_2 = W_act_motor1 * W_act_motor1;
  f_fval_tmp = W_dv_4 * W_dv_4;
  g_fval_tmp = I_xx * I_xx;
  h_fval_tmp = W_dv_1 * W_dv_1;
  i_fval_tmp = W_dv_3 * W_dv_3;
  j_fval_tmp = W_dv_2 * W_dv_2;
  Cd_zero = W_act_motor2 * W_act_motor2;
  Cl_alpha = W_act_motor3 * W_act_motor3;
  K_Cd = W_act_motor4 * W_act_motor4;
  Beta = W_act_tilt_el1 * W_act_tilt_el1;
  dv_global_4 = W_act_tilt_el2 * W_act_tilt_el2;
  W_dv_1 = W_act_tilt_el3 * W_act_tilt_el3;
  W_act_motor3 = W_act_tilt_el4 * W_act_tilt_el4;
  W_act_motor2 = W_act_tilt_az1 * W_act_tilt_az1;
  W_dv_2 = W_act_tilt_az2 * W_act_tilt_az2;
  W_dv_3 = W_act_tilt_az3 * W_act_tilt_az3;
  I_xx = W_act_tilt_az4 * W_act_tilt_az4;
  W_dv_4 = W_act_ailerons * W_act_ailerons;
  W_act_motor1 = W_act_ailerons_du * W_act_ailerons_du;
  fval = ((((((h_fval_tmp * (a * a) + i_fval_tmp * (b_a * b_a) / 10000.0) +
              gamma_quadratic_du2 * ((((((((((((CL_aileron * (c_a * c_a) +
    CL_aileron * (d_a * d_a)) + CL_aileron * (e_a * e_a)) + CL_aileron * (f_a *
    f_a)) + dv_global_6 * (g_a * g_a)) + dv_global_6 * (h_a * h_a)) +
    dv_global_6 * (i_a * i_a)) + dv_global_6 * (j_a * j_a)) + W_act_motor1 *
    (k_a * k_a)) + fval_tmp * (l_a * l_a)) + fval_tmp * (m_a * m_a)) + fval_tmp *
    (n_a * n_a)) + fval_tmp * (o_a * o_a))) + j_fval_tmp * (a_tmp * a_tmp)) +
            gamma_quadratic_du * ((((((((((((dv_global_2 * (p_a * p_a) + Cd_zero
    * (q_a * q_a)) + Cl_alpha * (r_a * r_a)) + K_Cd * (s_a * s_a)) + Beta * (t_a
    * t_a)) + dv_global_4 * (u_a * u_a)) + W_dv_1 * (v_a * v_a)) + W_act_motor3 *
    (w_a * w_a)) + W_dv_4 * (x_a * x_a)) + W_act_motor2 * (y_a * y_a)) + W_dv_2 *
    (ab_a * ab_a)) + W_dv_3 * (bb_a * bb_a)) + I_xx * (cb_a * cb_a))) +
           f_fval_tmp * (db_a * db_a) / (4.0 * g_fval_tmp)) + d_fval_tmp * (q *
           q) / e_fval_tmp) + b_fval_tmp * (V * V) / (4.0 * c_fval_tmp);
  W_act_tilt_el_du = Phi + ab_a_tmp;
  W_act_tilt_az_du = sin(W_act_tilt_el_du);
  W_dv_5 = Phi + cb_a_tmp;
  I_yy = sin(W_dv_5);
  W_dv_6 = Phi + eb_a_tmp;
  I_zz = sin(W_dv_6);
  flight_path_angle = Phi + gb_a_tmp;
  p = sin(flight_path_angle);
  r = K_p_T * l_4;
  wing_chord = K_p_T * l_1;
  W_act_motor_du = K_p_T * l_z;
  Cm_zero = 2.0 * CL_aileron * gamma_quadratic_du2;
  Cm_alpha = x[0] * gain_motor;
  S = 4.0 * K_p_T * x[0];
  Theta = K_p_M * cos(x[4] * gain_el);
  dv_global_5 = t_a_tmp_tmp * k_a_tmp;
  gradient[0] = ((((((2.0 * x[0] * b_fval_tmp * b_a_tmp_tmp * ((W_act_motor_du *
    d_a_tmp_tmp - K_p_M * bb_a_tmp * h_a_tmp_tmp) + r * bb_a_tmp * p_a_tmp_tmp) *
                      (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] * x[0]) *
    b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
    * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z *
    f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * g_a_tmp_tmp)
    - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * h_a_tmp_tmp) + 2.0 *
    K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp * i_a_tmp_tmp) - 2.0 * K_p_M *
    (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] *
    x[3]) * b_a_tmp_tmp * hb_a_tmp * k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T
    * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T *
    (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T
    * (x[2] * x[2]) * b_a_tmp_tmp * l_3 * fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T *
    (x[3] * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) -
                       c_a_tmp) / c_fval_tmp - Cm_zero * (previous_controls[0] -
    Cm_alpha) / gain_motor) - 4.0 * x[0] * d_fval_tmp * b_a_tmp_tmp * ((Theta *
    p_a_tmp_tmp - wing_chord * d_a_tmp_tmp) + r * cos(x[4] * gain_el) *
    h_a_tmp_tmp) * ((((((((((((rb_a_tmp + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp *
    l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * e_a_tmp_tmp)
    - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * f_a_tmp_tmp) + K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) - K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp)
                       - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp *
                       h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
                      db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) *
                     b_a_tmp_tmp * l_3 * fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3]
    * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp) - 2.0 *
                    dv_global_2 * gamma_quadratic_du * (desired_motor_value -
    Cm_alpha) / gain_motor) + 2.0 * x[0] * f_fval_tmp * b_a_tmp_tmp * ((K_p_M *
    d_a_tmp_tmp + wing_chord * bb_a_tmp * p_a_tmp_tmp) + W_act_motor_du *
    bb_a_tmp * h_a_tmp_tmp) * (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0])
    * b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
    (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp * i_a_tmp_tmp) + 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x
    [12] * gain_ailerons * rho) / g_fval_tmp) + S * h_fval_tmp * b_a_tmp_tmp *
                  (dv_global_1 + ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] *
    x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) +
    g_a_tmp_tmp * (x[3] * x[3]))) - o_a_tmp * (((bb_a_tmp * h_a_tmp_tmp * (x[0] *
    x[0]) + db_a_tmp * i_a_tmp_tmp * (x[1] * x[1])) + fb_a_tmp * j_a_tmp_tmp *
    (x[2] * x[2])) + hb_a_tmp * k_a_tmp_tmp * (x[3] * x[3]))) + p_a_tmp *
    (((bb_a_tmp * p_a_tmp_tmp * (x[0] * x[0]) + db_a_tmp * q_a_tmp_tmp * (x[1] *
    x[1])) + fb_a_tmp * r_a_tmp_tmp * (x[2] * x[2])) + hb_a_tmp * s_a_tmp_tmp *
     (x[3] * x[3]))) + q_a_tmp) / m) * ((d_a_tmp_tmp * h_a_tmp - l_a_tmp_tmp *
    l_a_tmp * j_a_tmp) + dv_global_5 * j_a_tmp) / m) + K_p_T * x[0] * i_fval_tmp
                 * b_a_tmp_tmp * ((d_a_tmp_tmp * j_a_tmp + l_a_tmp_tmp * h_a_tmp
    * l_a_tmp) - dv_global_5 * h_a_tmp) * ((100.0 * ((((m_a_tmp + i_a_tmp *
    (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp *
      (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] *
    gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x
    [3] * x[3]))) - mb_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 *
    dv_global_3) + 981.0) / (25.0 * m)) - S * j_fval_tmp * b_a_tmp_tmp *
    W_act_tilt_az_du * bb_a_tmp * a_tmp / m;
  Cm_alpha = x[1] * gain_motor;
  S = 4.0 * K_p_T * x[1];
  dv_global_5 = K_p_M * cos(x[5] * gain_el);
  g_a_tmp = u_a_tmp_tmp * k_a_tmp;
  gradient[1] = ((((((2.0 * x[1] * b_fval_tmp * b_a_tmp_tmp * ((W_act_motor_du *
    e_a_tmp_tmp + K_p_M * db_a_tmp * i_a_tmp_tmp) + r * db_a_tmp * q_a_tmp_tmp) *
                      (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] * x[0]) *
    b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
    * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z *
    f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * g_a_tmp_tmp)
    - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * h_a_tmp_tmp) + 2.0 *
    K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp * i_a_tmp_tmp) - 2.0 * K_p_M *
    (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] *
    x[3]) * b_a_tmp_tmp * hb_a_tmp * k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T
    * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T *
    (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T
    * (x[2] * x[2]) * b_a_tmp_tmp * l_3 * fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T *
    (x[3] * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) -
                       c_a_tmp) / c_fval_tmp - Cm_zero * (previous_controls[1] -
    Cm_alpha) / gain_motor) - 4.0 * x[1] * d_fval_tmp * b_a_tmp_tmp *
                     ((wing_chord * e_a_tmp_tmp - dv_global_5 * q_a_tmp_tmp) + r
                      * cos(x[5] * gain_el) * i_a_tmp_tmp) *
                     ((((((((((((rb_a_tmp + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp *
    l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * e_a_tmp_tmp)
    - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * f_a_tmp_tmp) + K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) - K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp)
    - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * h_a_tmp_tmp) -
                        K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp *
                        i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3
                       * fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) *
                      b_a_tmp_tmp * l_3 * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp)
                    - 2.0 * Cd_zero * gamma_quadratic_du * (desired_motor_value
    - Cm_alpha) / gain_motor) - 2.0 * x[1] * f_fval_tmp * b_a_tmp_tmp * ((K_p_M *
    e_a_tmp_tmp + wing_chord * db_a_tmp * q_a_tmp_tmp) - W_act_motor_du *
    db_a_tmp * i_a_tmp_tmp) * (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0])
    * b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
    (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp * i_a_tmp_tmp) + 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x
    [12] * gain_ailerons * rho) / g_fval_tmp) + S * h_fval_tmp * b_a_tmp_tmp *
                  (dv_global_1 + ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] *
    x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) +
    g_a_tmp_tmp * (x[3] * x[3]))) - o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] *
    gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1]
    * x[1])) + cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos
    (x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) + p_a_tmp * (((cos
    (x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el)
    * cos(x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x
    [3] * x[3]))) + q_a_tmp) / m) * ((e_a_tmp_tmp * h_a_tmp - m_a_tmp_tmp *
    l_a_tmp * j_a_tmp) + g_a_tmp * j_a_tmp) / m) + K_p_T * x[1] * i_fval_tmp *
                 b_a_tmp_tmp * ((e_a_tmp_tmp * j_a_tmp + m_a_tmp_tmp * h_a_tmp *
    l_a_tmp) - g_a_tmp * h_a_tmp) * ((100.0 * ((((m_a_tmp + i_a_tmp *
    (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp *
      (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] *
    gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x
    [3] * x[3]))) - mb_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 *
    dv_global_3) + 981.0) / (25.0 * m)) - S * j_fval_tmp * b_a_tmp_tmp * I_yy *
    db_a_tmp * a_tmp / m;
  Cm_alpha = K_p_T * l_2;
  S = K_p_T * l_3;
  g_a_tmp = x[2] * gain_motor;
  jb_a_tmp = 4.0 * K_p_T * x[2];
  kb_a_tmp = K_p_M * cos(x[6] * gain_el);
  ib_a_tmp = v_a_tmp_tmp * k_a_tmp;
  gradient[2] = ((((((2.0 * x[2] * f_fval_tmp * b_a_tmp_tmp * ((K_p_M *
    f_a_tmp_tmp - Cm_alpha * fb_a_tmp * r_a_tmp_tmp) + W_act_motor_du * fb_a_tmp
    * j_a_tmp_tmp) * (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
    (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp * i_a_tmp_tmp) + 2.0 * K_p_T *
                        (x[2] * x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp *
                        j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp
                       * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
                      gain_ailerons * rho) / g_fval_tmp - Cm_zero *
                      (previous_controls[2] - g_a_tmp) / gain_motor) - 4.0 * x[2]
                     * d_fval_tmp * b_a_tmp_tmp * ((Cm_alpha * f_a_tmp_tmp +
    K_p_M * fb_a_tmp * r_a_tmp_tmp) - S * fb_a_tmp * j_a_tmp_tmp) *
                     ((((((((((((rb_a_tmp + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp *
    l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * e_a_tmp_tmp)
    - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * f_a_tmp_tmp) + K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) - K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp)
    - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * h_a_tmp_tmp) -
                        K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp *
                        i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3
                       * fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) *
                      b_a_tmp_tmp * l_3 * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp)
                    - 2.0 * x[2] * b_fval_tmp * b_a_tmp_tmp * ((kb_a_tmp *
    j_a_tmp_tmp - W_act_motor_du * f_a_tmp_tmp) + S * cos(x[6] * gain_el) *
    r_a_tmp_tmp) * (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] * x[0]) *
    b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
    * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z *
    f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * g_a_tmp_tmp)
    - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * h_a_tmp_tmp) + 2.0 *
    K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp * i_a_tmp_tmp) - 2.0 * K_p_M *
    (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] *
    x[3]) * b_a_tmp_tmp * hb_a_tmp * k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T
    * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T *
                        (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp *
                        q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
                       * l_3 * fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) /
                    c_fval_tmp) - 2.0 * Cl_alpha * gamma_quadratic_du *
                   (desired_motor_value - g_a_tmp) / gain_motor) + jb_a_tmp *
                  h_fval_tmp * b_a_tmp_tmp * (dv_global_1 + ((((n_a_tmp +
    d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) +
                f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) -
    o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos
                 (x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) + cos
                (x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos
               (x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) +
    p_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos
                 (x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) + cos
                (x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos
               (x[7] * gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) +
    q_a_tmp) / m) * ((f_a_tmp_tmp * h_a_tmp - n_a_tmp_tmp * l_a_tmp * j_a_tmp) +
                     ib_a_tmp * j_a_tmp) / m) + K_p_T * x[2] * i_fval_tmp *
                 b_a_tmp_tmp * ((f_a_tmp_tmp * j_a_tmp + n_a_tmp_tmp * h_a_tmp *
    l_a_tmp) - ib_a_tmp * h_a_tmp) * ((100.0 * ((((m_a_tmp + i_a_tmp *
    (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp *
      (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] *
    gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x
    [3] * x[3]))) - mb_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 *
    dv_global_3) + 981.0) / (25.0 * m)) - jb_a_tmp * j_fval_tmp * b_a_tmp_tmp *
    I_zz * fb_a_tmp * a_tmp / m;
  g_a_tmp = x[3] * gain_motor;
  jb_a_tmp = 4.0 * K_p_T * x[3];
  ib_a_tmp = K_p_M * cos(x[7] * gain_el);
  qb_a_tmp = w_a_tmp_tmp * k_a_tmp;
  gradient[3] = ((((((4.0 * x[3] * d_fval_tmp * b_a_tmp_tmp * ((Cm_alpha *
    g_a_tmp_tmp + K_p_M * hb_a_tmp * s_a_tmp_tmp) + S * hb_a_tmp * k_a_tmp_tmp) *
                      ((((((((((((rb_a_tmp + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp
    * l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 *
    e_a_tmp_tmp) - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * f_a_tmp_tmp) +
    K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) - K_p_M * (x[0] *
    x[0]) * b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) *
    b_a_tmp_tmp * db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp *
    fb_a_tmp * r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp *
    s_a_tmp_tmp) - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp *
    h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp *
    i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 * fb_a_tmp *
                        j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3
                       * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp - Cm_zero *
                      (previous_controls[3] - g_a_tmp) / gain_motor) - 2.0 *
                     K_Cd * gamma_quadratic_du * (desired_motor_value - g_a_tmp)
                     / gain_motor) + 2.0 * x[3] * b_fval_tmp * b_a_tmp_tmp *
                    ((W_act_motor_du * g_a_tmp_tmp + ib_a_tmp * k_a_tmp_tmp) - S
                     * cos(x[7] * gain_el) * s_a_tmp_tmp) *
                    (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] * x[0]) *
    b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
    * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z *
    f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * g_a_tmp_tmp)
    - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * h_a_tmp_tmp) + 2.0 *
    K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp * i_a_tmp_tmp) - 2.0 * K_p_M *
    (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] *
    x[3]) * b_a_tmp_tmp * hb_a_tmp * k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T
    * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T *
    (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T
                        * (x[2] * x[2]) * b_a_tmp_tmp * l_3 * fb_a_tmp *
                        r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp
                       * l_3 * hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) /
                    c_fval_tmp) + 2.0 * x[3] * f_fval_tmp * b_a_tmp_tmp *
                   ((Cm_alpha * hb_a_tmp * s_a_tmp_tmp - K_p_M * g_a_tmp_tmp) +
                    W_act_motor_du * hb_a_tmp * k_a_tmp_tmp) *
                   (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
                        (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp *
                        h_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
                       * l_z * db_a_tmp * i_a_tmp_tmp) + 2.0 * K_p_T * (x[2] *
    x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
                    gain_ailerons * rho) / g_fval_tmp) + jb_a_tmp * h_fval_tmp *
                  b_a_tmp_tmp * (dv_global_1 + ((((n_a_tmp + d_a_tmp *
    (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp *
      (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) - o_a_tmp * (((cos(x[4] *
    gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x
    [3] * x[3]))) + p_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x
    [0] * x[0]) + cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) + q_a_tmp) / m) *
                  ((g_a_tmp_tmp * h_a_tmp - o_a_tmp_tmp * l_a_tmp * j_a_tmp) +
                   qb_a_tmp * j_a_tmp) / m) + K_p_T * x[3] * i_fval_tmp *
                 b_a_tmp_tmp * ((g_a_tmp_tmp * j_a_tmp + o_a_tmp_tmp * h_a_tmp *
    l_a_tmp) - qb_a_tmp * h_a_tmp) * ((100.0 * ((((m_a_tmp + i_a_tmp *
    (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp *
      (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] *
    gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x
    [3] * x[3]))) - mb_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 *
    dv_global_3) + 981.0) / (25.0 * m)) - jb_a_tmp * j_fval_tmp * b_a_tmp_tmp *
    p * hb_a_tmp * a_tmp / m;
  Cm_zero = 2.0 * dv_global_6 * gamma_quadratic_du2;
  g_a_tmp = 2.0 * s_a_tmp * d_fval_tmp;
  jb_a_tmp = s_a_tmp * b_fval_tmp;
  qb_a_tmp = a_tmp_tmp * j_fval_tmp;
  ob_a_tmp = a_tmp_tmp * h_fval_tmp;
  pb_a_tmp = ab_a_tmp_tmp * i_fval_tmp;
  gradient[4] = ((((((jb_a_tmp * gain_el * b_a_tmp_tmp * ((W_act_motor_du * cos
    (x[4] * gain_el) + K_p_M * sin(x[4] * gain_el) * h_a_tmp_tmp) - r *
    p_a_tmp_tmp * d_a_tmp_tmp) * (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] *
    x[0]) * b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
    * l_z * f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z *
    g_a_tmp_tmp) - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp *
    h_a_tmp_tmp) + 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp *
    i_a_tmp_tmp) - 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp *
    k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) / c_fval_tmp - Cm_zero *
                      (previous_controls[4] - r_a_tmp) / gain_el) - 2.0 * Beta *
                     gamma_quadratic_du * (desired_el_value - r_a_tmp) / gain_el)
                    - s_a_tmp * f_fval_tmp * gain_el * b_a_tmp_tmp *
                    ((wing_chord * p_a_tmp_tmp * d_a_tmp_tmp - Theta) +
                     W_act_motor_du * sin(x[4] * gain_el) * h_a_tmp_tmp) *
                    (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
    (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_T
                        * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp *
                        i_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
                       * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
                     gain_ailerons * rho) / g_fval_tmp) + g_a_tmp * gain_el *
                   b_a_tmp_tmp * ((wing_chord * cos(x[4] * gain_el) + K_p_M *
    p_a_tmp_tmp * d_a_tmp_tmp) + r * d_a_tmp_tmp * h_a_tmp_tmp) *
                   ((((((((((((rb_a_tmp + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp *
    l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * e_a_tmp_tmp)
    - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * f_a_tmp_tmp) + K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) - K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp)
                       - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp *
                       h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
                      db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) *
                     b_a_tmp_tmp * l_3 * fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3]
    * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp) +
                  pb_a_tmp * gain_el * b_a_tmp_tmp * ((bb_a_tmp * j_a_tmp -
    d_a_tmp_tmp * h_a_tmp_tmp * h_a_tmp * l_a_tmp) + p_a_tmp_tmp * d_a_tmp_tmp *
    k_a_tmp * h_a_tmp) * ((100.0 * ((((m_a_tmp + i_a_tmp * (((d_a_tmp_tmp * (x[0]
    * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) +
    g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] * gain_el) * sin(x[8]
    * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x
    [1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) +
    cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) - mb_a_tmp *
    (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] *
    gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos
      (x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] *
    gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 * dv_global_3) + 981.0) /
                  (50.0 * m)) + ob_a_tmp * gain_el * b_a_tmp_tmp * (dv_global_1
    + ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x
    [1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3])))
         - o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0])
                        + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x
    [1])) + cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos(x
    [7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) + p_a_tmp * (((cos
    (x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el)
    * cos(x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x
    [3] * x[3]))) + q_a_tmp) / m) * ((bb_a_tmp * h_a_tmp - cos(x[8] * gain_az) *
    sin(x[4] * gain_el) * k_a_tmp * j_a_tmp) + sin(x[4] * gain_el) * sin(x[8] *
    gain_az) * l_a_tmp * j_a_tmp) / m) + qb_a_tmp * gain_el * b_a_tmp_tmp *
    W_act_tilt_az_du * d_a_tmp_tmp * a_tmp / m;
  Theta = u_a_tmp * b_fval_tmp;
  q = 2.0 * u_a_tmp * d_fval_tmp;
  nb_a_tmp = c_a_tmp_tmp * j_fval_tmp;
  CL_aileron = c_a_tmp_tmp * h_fval_tmp;
  dv_global_6 = bb_a_tmp_tmp * i_fval_tmp;
  gradient[5] = ((((((dv_global_6 * gain_el * b_a_tmp_tmp * ((db_a_tmp * j_a_tmp
    - e_a_tmp_tmp * i_a_tmp_tmp * h_a_tmp * l_a_tmp) + q_a_tmp_tmp * e_a_tmp_tmp
    * k_a_tmp * h_a_tmp) * ((100.0 * ((((m_a_tmp + i_a_tmp * (((d_a_tmp_tmp *
    (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2]))
    + g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] * gain_el) * sin(x
    [8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) *
    (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2]))
    + cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) - mb_a_tmp *
    (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] *
    gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos
      (x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] *
    gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 * dv_global_3) + 981.0) /
                      (50.0 * m) - Cm_zero * (previous_controls[5] - t_a_tmp) /
                      gain_el) - Theta * gain_el * b_a_tmp_tmp * ((K_p_M * sin
    (x[5] * gain_el) * i_a_tmp_tmp - W_act_motor_du * cos(x[5] * gain_el)) + r *
    q_a_tmp_tmp * e_a_tmp_tmp) * (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] *
    x[0]) * b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
    * l_z * f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z *
    g_a_tmp_tmp) - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp *
    h_a_tmp_tmp) + 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp *
    i_a_tmp_tmp) - 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp *
    k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) / c_fval_tmp) - u_a_tmp *
                    f_fval_tmp * gain_el * b_a_tmp_tmp * ((dv_global_5 -
    wing_chord * q_a_tmp_tmp * e_a_tmp_tmp) + W_act_motor_du * sin(x[5] *
    gain_el) * i_a_tmp_tmp) * (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0])
    * b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
    (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp * i_a_tmp_tmp) + 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x
    [12] * gain_ailerons * rho) / g_fval_tmp) - q * gain_el * b_a_tmp_tmp *
                   ((wing_chord * cos(x[5] * gain_el) + K_p_M * q_a_tmp_tmp *
                     e_a_tmp_tmp) - r * e_a_tmp_tmp * i_a_tmp_tmp) *
                   ((((((((((((rb_a_tmp + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp *
    l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * e_a_tmp_tmp)
    - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * f_a_tmp_tmp) + K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) - K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp)
                       - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp *
                       h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
                      db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) *
                     b_a_tmp_tmp * l_3 * fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3]
    * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp) - 2.0 *
                  dv_global_4 * gamma_quadratic_du * (desired_el_value - t_a_tmp)
                  / gain_el) + CL_aileron * gain_el * b_a_tmp_tmp * (dv_global_1
    + ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x
    [1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3])))
         - o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0])
                        + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x
    [1])) + cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos(x
    [7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) + p_a_tmp * (((cos
    (x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el)
    * cos(x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x
    [3] * x[3]))) + q_a_tmp) / m) * ((db_a_tmp * h_a_tmp - cos(x[9] * gain_az) *
    sin(x[5] * gain_el) * k_a_tmp * j_a_tmp) + sin(x[5] * gain_el) * sin(x[9] *
    gain_az) * l_a_tmp * j_a_tmp) / m) + nb_a_tmp * gain_el * b_a_tmp_tmp * I_yy
    * e_a_tmp_tmp * a_tmp / m;
  wing_chord = 2.0 * w_a_tmp * d_fval_tmp;
  dv_global_5 = w_a_tmp * b_fval_tmp;
  dv_global_4 = x_a_tmp_tmp * j_fval_tmp;
  Beta = x_a_tmp_tmp * h_fval_tmp;
  dv_global_2 = cb_a_tmp_tmp * i_fval_tmp;
  gradient[6] = ((((((dv_global_5 * gain_el * b_a_tmp_tmp * ((W_act_motor_du *
    cos(x[6] * gain_el) + K_p_M * sin(x[6] * gain_el) * j_a_tmp_tmp) + S *
    r_a_tmp_tmp * f_a_tmp_tmp) * (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] *
    x[0]) * b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
    * l_z * f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z *
    g_a_tmp_tmp) - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp *
    h_a_tmp_tmp) + 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp *
    i_a_tmp_tmp) - 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp *
    k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) / c_fval_tmp - Cm_zero *
                      (previous_controls[6] - v_a_tmp) / gain_el) - 2.0 * W_dv_1
                     * gamma_quadratic_du * (desired_el_value - v_a_tmp) /
                     gain_el) + w_a_tmp * f_fval_tmp * gain_el * b_a_tmp_tmp *
                    ((kb_a_tmp + Cm_alpha * r_a_tmp_tmp * f_a_tmp_tmp) -
                     W_act_motor_du * sin(x[6] * gain_el) * j_a_tmp_tmp) *
                    (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
    (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_T
                        * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp *
                        i_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
                       * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
                     gain_ailerons * rho) / g_fval_tmp) - wing_chord * gain_el *
                   b_a_tmp_tmp * ((Cm_alpha * cos(x[6] * gain_el) - K_p_M *
    r_a_tmp_tmp * f_a_tmp_tmp) + S * f_a_tmp_tmp * j_a_tmp_tmp) *
                   ((((((((((((rb_a_tmp + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp *
    l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * e_a_tmp_tmp)
    - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * f_a_tmp_tmp) + K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) - K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp)
                       - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp *
                       h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
                      db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) *
                     b_a_tmp_tmp * l_3 * fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3]
    * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp) +
                  dv_global_2 * gain_el * b_a_tmp_tmp * ((fb_a_tmp * j_a_tmp -
    f_a_tmp_tmp * j_a_tmp_tmp * h_a_tmp * l_a_tmp) + r_a_tmp_tmp * f_a_tmp_tmp *
    k_a_tmp * h_a_tmp) * ((100.0 * ((((m_a_tmp + i_a_tmp * (((d_a_tmp_tmp * (x[0]
    * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) +
    g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] * gain_el) * sin(x[8]
    * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x
    [1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) +
    cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) - mb_a_tmp *
    (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] *
    gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos
      (x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] *
    gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 * dv_global_3) + 981.0) /
                  (50.0 * m)) + Beta * gain_el * b_a_tmp_tmp * (dv_global_1 +
    ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1]
    * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) -
       o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) +
                    cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) +
                   cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) +
                  cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) +
      p_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) +
                   cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
                  cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) +
                 cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) +
     q_a_tmp) / m) * ((fb_a_tmp * h_a_tmp - cos(x[10] * gain_az) * sin(x[6] *
    gain_el) * k_a_tmp * j_a_tmp) + sin(x[6] * gain_el) * sin(x[10] * gain_az) *
                      l_a_tmp * j_a_tmp) / m) + dv_global_4 * gain_el *
    b_a_tmp_tmp * I_zz * f_a_tmp_tmp * a_tmp / m;
  kb_a_tmp = y_a_tmp * b_fval_tmp;
  K_Cd = 2.0 * y_a_tmp * d_fval_tmp;
  Cl_alpha = y_a_tmp_tmp * j_fval_tmp;
  Cd_zero = y_a_tmp_tmp * h_fval_tmp;
  V = db_a_tmp_tmp * i_fval_tmp;
  gradient[7] = ((((((kb_a_tmp * gain_el * b_a_tmp_tmp * ((W_act_motor_du * cos
    (x[7] * gain_el) - K_p_M * sin(x[7] * gain_el) * k_a_tmp_tmp) + S *
    s_a_tmp_tmp * g_a_tmp_tmp) * (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] *
    x[0]) * b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
    * l_z * f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z *
    g_a_tmp_tmp) - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp *
    h_a_tmp_tmp) + 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp *
    i_a_tmp_tmp) - 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp *
    k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) / c_fval_tmp - Cm_zero *
                      (previous_controls[7] - x_a_tmp) / gain_el) - 2.0 *
                     W_act_motor3 * gamma_quadratic_du * (desired_el_value -
    x_a_tmp) / gain_el) - y_a_tmp * f_fval_tmp * gain_el * b_a_tmp_tmp *
                    ((ib_a_tmp + Cm_alpha * s_a_tmp_tmp * g_a_tmp_tmp) +
                     W_act_motor_du * sin(x[7] * gain_el) * k_a_tmp_tmp) *
                    (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
    (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_T
                        * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp *
                        i_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp
                       * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
                     gain_ailerons * rho) / g_fval_tmp) - K_Cd * gain_el *
                   b_a_tmp_tmp * ((K_p_M * s_a_tmp_tmp * g_a_tmp_tmp - Cm_alpha *
    cos(x[7] * gain_el)) + S * g_a_tmp_tmp * k_a_tmp_tmp) * ((((((((((((rb_a_tmp
    + K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_1 * d_a_tmp_tmp) - K_p_T * (x[1] *
    x[1]) * b_a_tmp_tmp * l_1 * e_a_tmp_tmp) - K_p_T * (x[2] * x[2]) *
    b_a_tmp_tmp * l_2 * f_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2
    * g_a_tmp_tmp) - K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp *
    p_a_tmp_tmp) + K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp * q_a_tmp_tmp)
    - K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp * r_a_tmp_tmp) + K_p_M *
    (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp * s_a_tmp_tmp) - K_p_T * (x[0] * x[0])
    * b_a_tmp_tmp * l_4 * bb_a_tmp * h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_4 * db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) *
    b_a_tmp_tmp * l_3 * fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) *
    b_a_tmp_tmp * l_3 * hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp) + V * gain_el *
                  b_a_tmp_tmp * ((hb_a_tmp * j_a_tmp - g_a_tmp_tmp * k_a_tmp_tmp
    * h_a_tmp * l_a_tmp) + s_a_tmp_tmp * g_a_tmp_tmp * k_a_tmp * h_a_tmp) *
                  ((100.0 * ((((m_a_tmp + i_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0])
    + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp *
    (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) - mb_a_tmp * (((cos(x[4] *
    gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * cos
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x
    [3] * x[3]))) - f_a_tmp) / m - 100.0 * dv_global_3) + 981.0) / (50.0 * m)) +
                 Cd_zero * gain_el * b_a_tmp_tmp * (dv_global_1 + ((((n_a_tmp +
    d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) +
                f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) -
    o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos
                 (x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) + cos
                (x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos
               (x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) +
    p_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos
                 (x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) + cos
                (x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos
               (x[7] * gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) +
    q_a_tmp) / m) * ((hb_a_tmp * h_a_tmp - cos(x[11] * gain_az) * sin(x[7] *
    gain_el) * k_a_tmp * j_a_tmp) + sin(x[7] * gain_el) * sin(x[11] * gain_az) *
                     l_a_tmp * j_a_tmp) / m) + Cl_alpha * gain_el * b_a_tmp_tmp *
    p * g_a_tmp_tmp * a_tmp / m;
  W_act_motor_du = 2.0 * fval_tmp * gamma_quadratic_du2;
  gradient[8] = ((((((g_a_tmp * gain_az * b_a_tmp_tmp * bb_a_tmp * (K_p_M *
    h_a_tmp_tmp - r * cos(x[8] * gain_az)) * ((((((((((((rb_a_tmp + K_p_T * (x[0]
    * x[0]) * b_a_tmp_tmp * l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_1 * e_a_tmp_tmp) - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2
    * f_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) -
    K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1]
    * x[1]) * b_a_tmp_tmp * db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) *
    b_a_tmp_tmp * fb_a_tmp * r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp *
    hb_a_tmp * s_a_tmp_tmp) - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp - W_act_motor_du * (previous_controls[8]
    - ab_a_tmp) / gain_az) - jb_a_tmp * gain_az * b_a_tmp_tmp * bb_a_tmp *
                     (K_p_M * cos(x[8] * gain_az) + r * h_a_tmp_tmp) *
                     (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] * x[0]) *
    b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
    * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z *
    f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * g_a_tmp_tmp)
    - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * h_a_tmp_tmp) + 2.0 *
    K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp * i_a_tmp_tmp) - 2.0 * K_p_M *
    (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] *
    x[3]) * b_a_tmp_tmp * hb_a_tmp * k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T
    * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T *
    (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T
    * (x[2] * x[2]) * b_a_tmp_tmp * l_3 * fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T *
                        (x[3] * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp *
                        s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) / c_fval_tmp) - 2.0 *
                    W_act_motor2 * gamma_quadratic_du * (desired_az_value -
    ab_a_tmp) / gain_az) - qb_a_tmp * gain_az * b_a_tmp_tmp * cos
                   (W_act_tilt_el_du) * bb_a_tmp * a_tmp / m) + ab_a_tmp_tmp *
                  f_fval_tmp * gain_az * b_a_tmp_tmp * bb_a_tmp * (l_z *
    p_a_tmp_tmp - l_1 * h_a_tmp_tmp) * (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0]
    * x[0]) * b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) *
    b_a_tmp_tmp * e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp *
    f_a_tmp_tmp) - 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) +
    2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) -
    2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) -
    2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) +
    2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) +
    2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) +
    2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp * i_a_tmp_tmp) +
    2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) +
    2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) +
    e_a_tmp * x[12] * gain_ailerons * rho) / g_fval_tmp) - ob_a_tmp * gain_az *
                 b_a_tmp_tmp * W_act_tilt_az_du * bb_a_tmp * j_a_tmp *
                 (dv_global_1 + ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] *
    x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) +
    g_a_tmp_tmp * (x[3] * x[3]))) - o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] *
    gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1]
    * x[1])) + cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos
    (x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) + p_a_tmp * (((cos
    (x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el)
    * cos(x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x
    [3] * x[3]))) + q_a_tmp) / m) / m) + pb_a_tmp * gain_az * b_a_tmp_tmp *
    W_act_tilt_az_du * bb_a_tmp * h_a_tmp * ((100.0 * ((((m_a_tmp + i_a_tmp *
    (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp *
      (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] *
    gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x
    [3] * x[3]))) - mb_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 *
    dv_global_3) + 981.0) / (50.0 * m);
  gradient[9] = ((((((Theta * gain_az * b_a_tmp_tmp * db_a_tmp * (K_p_M * cos(x
    [9] * gain_az) - r * i_a_tmp_tmp) * (((((((((((((((sb_a_tmp + 2.0 * K_p_T
    * (x[0] * x[0]) * b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] *
    x[1]) * b_a_tmp_tmp * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) *
    b_a_tmp_tmp * l_z * f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp
    * l_z * g_a_tmp_tmp) - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp *
    h_a_tmp_tmp) + 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp *
    i_a_tmp_tmp) - 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp *
    j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * hb_a_tmp *
    k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) / c_fval_tmp - W_act_motor_du
                      * (previous_controls[9] - cb_a_tmp) / gain_az) - 2.0 *
                     W_dv_2 * gamma_quadratic_du * (desired_az_value - cb_a_tmp)
                     / gain_az) - q * gain_az * b_a_tmp_tmp * db_a_tmp * (K_p_M *
    i_a_tmp_tmp + r * cos(x[9] * gain_az)) * ((((((((((((rb_a_tmp + K_p_T * (x[0]
    * x[0]) * b_a_tmp_tmp * l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_1 * e_a_tmp_tmp) - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2
    * f_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) -
    K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1]
    * x[1]) * b_a_tmp_tmp * db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) *
    b_a_tmp_tmp * fb_a_tmp * r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp *
    hb_a_tmp * s_a_tmp_tmp) - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp) - nb_a_tmp * gain_az * b_a_tmp_tmp *
                   cos(W_dv_5) * db_a_tmp * a_tmp / m) + bb_a_tmp_tmp *
                  f_fval_tmp * gain_az * b_a_tmp_tmp * db_a_tmp * (l_z *
    q_a_tmp_tmp + l_1 * i_a_tmp_tmp) * (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0]
    * x[0]) * b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) *
    b_a_tmp_tmp * e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp *
    f_a_tmp_tmp) - 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) +
    2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) -
    2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) -
    2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) +
    2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) +
    2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp * h_a_tmp_tmp) +
    2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp * i_a_tmp_tmp) +
    2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) +
    2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) +
    e_a_tmp * x[12] * gain_ailerons * rho) / g_fval_tmp) - CL_aileron * gain_az *
                 b_a_tmp_tmp * I_yy * db_a_tmp * j_a_tmp * (dv_global_1 +
    ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1]
    * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) -
       o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) +
                    cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) +
                   cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) +
                  cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) +
      p_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) +
                   cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
                  cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) +
                 cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) +
     q_a_tmp) / m) / m) + dv_global_6 * gain_az * b_a_tmp_tmp * I_yy * db_a_tmp *
    h_a_tmp * ((100.0 * ((((m_a_tmp + i_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) +
    e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp *
    (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) - mb_a_tmp * (((cos(x[4] *
    gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * cos
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x
    [3] * x[3]))) - f_a_tmp) / m - 100.0 * dv_global_3) + 981.0) / (50.0 * m);
  gradient[10] = ((((((wing_chord * gain_az * b_a_tmp_tmp * fb_a_tmp * (K_p_M *
    j_a_tmp_tmp + S * cos(x[10] * gain_az)) * ((((((((((((rb_a_tmp + K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_1 * e_a_tmp_tmp) - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2
    * f_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) -
    K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1]
    * x[1]) * b_a_tmp_tmp * db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) *
    b_a_tmp_tmp * fb_a_tmp * r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp *
    hb_a_tmp * s_a_tmp_tmp) - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp - W_act_motor_du * (previous_controls
    [10] - eb_a_tmp) / gain_az) - dv_global_5 * gain_az * b_a_tmp_tmp * fb_a_tmp
                      * (K_p_M * cos(x[10] * gain_az) - S * j_a_tmp_tmp) *
                      (((((((((((((((sb_a_tmp + 2.0 * K_p_T * (x[0] * x[0]) *
    b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
    * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z *
    f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * g_a_tmp_tmp)
    - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * h_a_tmp_tmp) + 2.0 *
    K_p_M * (x[1] * x[1]) * b_a_tmp_tmp * db_a_tmp * i_a_tmp_tmp) - 2.0 * K_p_M *
    (x[2] * x[2]) * b_a_tmp_tmp * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] *
    x[3]) * b_a_tmp_tmp * hb_a_tmp * k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T
    * (x[0] * x[0]) * b_a_tmp_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T *
    (x[1] * x[1]) * b_a_tmp_tmp * l_4 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T
    * (x[2] * x[2]) * b_a_tmp_tmp * l_3 * fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T *
    (x[3] * x[3]) * b_a_tmp_tmp * l_3 * hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) -
                       c_a_tmp) / c_fval_tmp) - 2.0 * W_dv_3 *
                     gamma_quadratic_du * (desired_az_value - eb_a_tmp) /
                     gain_az) - dv_global_4 * gain_az * b_a_tmp_tmp * cos(W_dv_6)
                    * fb_a_tmp * a_tmp / m) + cb_a_tmp_tmp * f_fval_tmp *
                   gain_az * b_a_tmp_tmp * fb_a_tmp * (l_z * r_a_tmp_tmp + l_2 *
    j_a_tmp_tmp) * (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
                        (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp *
                        h_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
                       * l_z * db_a_tmp * i_a_tmp_tmp) + 2.0 * K_p_T * (x[2] *
    x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
                    gain_ailerons * rho) / g_fval_tmp) - Beta * gain_az *
                  b_a_tmp_tmp * I_zz * fb_a_tmp * j_a_tmp * (dv_global_1 +
    ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1]
    * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) -
       o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) +
                    cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) +
                   cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) +
                  cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) +
      p_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) +
                   cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
                  cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) +
                 cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) +
     q_a_tmp) / m) / m) + dv_global_2 * gain_az * b_a_tmp_tmp * I_zz * fb_a_tmp *
    h_a_tmp * ((100.0 * ((((m_a_tmp + i_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) +
    e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp *
    (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) *
    (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) +
    cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) + cos(x[7] *
    gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) - mb_a_tmp * (((cos(x[4] *
    gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * cos
    (x[9] * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
    gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x
    [3] * x[3]))) - f_a_tmp) / m - 100.0 * dv_global_3) + 981.0) / (50.0 * m);
  gradient[11] = ((((((kb_a_tmp * gain_az * b_a_tmp_tmp * hb_a_tmp * (K_p_M *
    cos(x[11] * gain_az) + S * k_a_tmp_tmp) * (((((((((((((((sb_a_tmp + 2.0 *
    K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_z * d_a_tmp_tmp) + 2.0 * K_p_T * (x
    [1] * x[1]) * b_a_tmp_tmp * l_z * e_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2])
    * b_a_tmp_tmp * l_z * f_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) *
    b_a_tmp_tmp * l_z * g_a_tmp_tmp) - 2.0 * K_p_M * (x[0] * x[0]) * b_a_tmp_tmp
    * bb_a_tmp * h_a_tmp_tmp) + 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    db_a_tmp * i_a_tmp_tmp) - 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp *
    fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp *
    hb_a_tmp * k_a_tmp_tmp) + tb_a_tmp) + 2.0 * K_p_T * (x[0] * x[0]) *
    b_a_tmp_tmp * l_4 * bb_a_tmp * p_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_4 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) *
    b_a_tmp_tmp * l_3 * fb_a_tmp * r_a_tmp_tmp) - 2.0 * K_p_T * (x[3] * x[3]) *
    b_a_tmp_tmp * l_3 * hb_a_tmp * s_a_tmp_tmp) + ub_a_tmp) - c_a_tmp) /
                       c_fval_tmp - W_act_motor_du * (previous_controls[11] -
    gb_a_tmp) / gain_az) - 2.0 * I_xx * gamma_quadratic_du * (desired_az_value -
    gb_a_tmp) / gain_az) - K_Cd * gain_az * b_a_tmp_tmp * hb_a_tmp * (K_p_M *
    k_a_tmp_tmp - S * cos(x[11] * gain_az)) * ((((((((((((rb_a_tmp + K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * d_a_tmp_tmp) - K_p_T * (x[1] * x[1]) *
    b_a_tmp_tmp * l_1 * e_a_tmp_tmp) - K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2
    * f_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * g_a_tmp_tmp) -
    K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * bb_a_tmp * p_a_tmp_tmp) + K_p_M * (x[1]
    * x[1]) * b_a_tmp_tmp * db_a_tmp * q_a_tmp_tmp) - K_p_M * (x[2] * x[2]) *
    b_a_tmp_tmp * fb_a_tmp * r_a_tmp_tmp) + K_p_M * (x[3] * x[3]) * b_a_tmp_tmp *
    hb_a_tmp * s_a_tmp_tmp) - K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_4 *
    bb_a_tmp * h_a_tmp_tmp) - K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_4 *
    db_a_tmp * i_a_tmp_tmp) + K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_3 *
    fb_a_tmp * j_a_tmp_tmp) + K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_3 *
    hb_a_tmp * k_a_tmp_tmp) / e_fval_tmp) - Cl_alpha * gain_az * b_a_tmp_tmp *
                    cos(flight_path_angle) * hb_a_tmp * a_tmp / m) +
                   db_a_tmp_tmp * f_fval_tmp * gain_az * b_a_tmp_tmp * hb_a_tmp *
                   (l_z * s_a_tmp_tmp - l_2 * k_a_tmp_tmp) *
                   (((((((((((((b_a_tmp + 2.0 * K_p_M * (x[0] * x[0]) *
    b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] * x[1]) * b_a_tmp_tmp *
    e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) * b_a_tmp_tmp * f_a_tmp_tmp) -
    2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp * g_a_tmp_tmp) + 2.0 * K_p_T * (x
    [0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp * p_a_tmp_tmp) - 2.0 * K_p_T
    * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp * q_a_tmp_tmp) - 2.0 * K_p_T *
    (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp * r_a_tmp_tmp) + 2.0 * K_p_T
    * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp * s_a_tmp_tmp) + 2.0 * K_p_T *
                        (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp *
                        h_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp
                       * l_z * db_a_tmp * i_a_tmp_tmp) + 2.0 * K_p_T * (x[2] *
    x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp * j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] *
    x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp * k_a_tmp_tmp) + e_a_tmp * x[12] *
                    gain_ailerons * rho) / g_fval_tmp) - Cd_zero * gain_az *
                  b_a_tmp_tmp * p * hb_a_tmp * j_a_tmp * (dv_global_1 +
    ((((n_a_tmp + d_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) + e_a_tmp_tmp * (x[1]
    * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) + g_a_tmp_tmp * (x[3] * x[3]))) -
       o_a_tmp * (((cos(x[4] * gain_el) * sin(x[8] * gain_az) * (x[0] * x[0]) +
                    cos(x[5] * gain_el) * sin(x[9] * gain_az) * (x[1] * x[1])) +
                   cos(x[6] * gain_el) * sin(x[10] * gain_az) * (x[2] * x[2])) +
                  cos(x[7] * gain_el) * sin(x[11] * gain_az) * (x[3] * x[3]))) +
      p_a_tmp * (((cos(x[4] * gain_el) * cos(x[8] * gain_az) * (x[0] * x[0]) +
                   cos(x[5] * gain_el) * cos(x[9] * gain_az) * (x[1] * x[1])) +
                  cos(x[6] * gain_el) * cos(x[10] * gain_az) * (x[2] * x[2])) +
                 cos(x[7] * gain_el) * cos(x[11] * gain_az) * (x[3] * x[3]))) +
     q_a_tmp) / m) / m) + V * gain_az * b_a_tmp_tmp * p * hb_a_tmp * h_a_tmp *
    ((100.0 * ((((m_a_tmp + i_a_tmp * (((d_a_tmp_tmp * (x[0] * x[0]) +
             e_a_tmp_tmp * (x[1] * x[1])) + f_a_tmp_tmp * (x[2] * x[2])) +
           g_a_tmp_tmp * (x[3] * x[3]))) + lb_a_tmp * (((cos(x[4] * gain_el) *
            sin(x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * sin(x[9]
             * gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * sin(x[10] *
            gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * sin(x[11] *
           gain_az) * (x[3] * x[3]))) - mb_a_tmp * (((cos(x[4] * gain_el) * cos
           (x[8] * gain_az) * (x[0] * x[0]) + cos(x[5] * gain_el) * cos(x[9] *
            gain_az) * (x[1] * x[1])) + cos(x[6] * gain_el) * cos(x[10] *
           gain_az) * (x[2] * x[2])) + cos(x[7] * gain_el) * cos(x[11] * gain_az)
         * (x[3] * x[3]))) - f_a_tmp) / m - 100.0 * dv_global_3) + 981.0) /
    (50.0 * m);
  gradient[12] = (W_dv_4 * gamma_quadratic_du * (2.0 * x[12] - 2.0 *
    desired_ailerons_value / gain_ailerons) + W_act_motor1 * gamma_quadratic_du2
                  * (2.0 * x[12] - 2.0 * previous_controls[14] / gain_ailerons))
    + e_a_tmp * f_fval_tmp * gain_ailerons * rho * (((((((((((((b_a_tmp + 2.0 *
    K_p_M * (x[0] * x[0]) * b_a_tmp_tmp * d_a_tmp_tmp) - 2.0 * K_p_M * (x[1] *
    x[1]) * b_a_tmp_tmp * e_a_tmp_tmp) + 2.0 * K_p_M * (x[2] * x[2]) *
    b_a_tmp_tmp * f_a_tmp_tmp) - 2.0 * K_p_M * (x[3] * x[3]) * b_a_tmp_tmp *
    g_a_tmp_tmp) + 2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_1 * bb_a_tmp *
    p_a_tmp_tmp) - 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_1 * db_a_tmp *
    q_a_tmp_tmp) - 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_2 * fb_a_tmp *
    r_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_2 * hb_a_tmp *
    s_a_tmp_tmp) + 2.0 * K_p_T * (x[0] * x[0]) * b_a_tmp_tmp * l_z * bb_a_tmp *
    h_a_tmp_tmp) + 2.0 * K_p_T * (x[1] * x[1]) * b_a_tmp_tmp * l_z * db_a_tmp *
    i_a_tmp_tmp) + 2.0 * K_p_T * (x[2] * x[2]) * b_a_tmp_tmp * l_z * fb_a_tmp *
    j_a_tmp_tmp) + 2.0 * K_p_T * (x[3] * x[3]) * b_a_tmp_tmp * l_z * hb_a_tmp *
    k_a_tmp_tmp) + e_a_tmp * x[12] * gain_ailerons * rho) / (2.0 * g_fval_tmp);
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
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 13)) {
      allFinite = ((!rtIsInf(grad_workspace[idx_current])) && (!rtIsNaN
        (grad_workspace[idx_current])));
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
double evalObjAndConstrAndDerivatives(const d_struct_T
  *c_obj_next_next_next_next_next_, const double x[15], double grad_workspace[16],
  int *status)
{
  double d_obj_next_next_next_next_next_[15];
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

  /*  if nargout > 1 */
  memcpy(&d_obj_next_next_next_next_next_[0],
         &c_obj_next_next_next_next_next_->previous_controls->contents[0], 15U *
         sizeof(double));
  fval = c_compute_cost_and_gradient_fir(c_obj_next_next_next_next_next_->
    Beta->contents, c_obj_next_next_next_next_next_->CL_aileron->contents,
    c_obj_next_next_next_next_next_->Cd_zero->contents,
    c_obj_next_next_next_next_next_->Cl_alpha->contents,
    c_obj_next_next_next_next_next_->Cm_zero->contents,
    c_obj_next_next_next_next_next_->Cm_alpha->contents,
    c_obj_next_next_next_next_next_->I_xx->contents,
    c_obj_next_next_next_next_next_->I_yy->contents,
    c_obj_next_next_next_next_next_->I_zz->contents,
    c_obj_next_next_next_next_next_->K_Cd->contents,
    c_obj_next_next_next_next_next_->K_p_M->contents,
    c_obj_next_next_next_next_next_->K_p_T->contents, x[0], x[1], x[2], x[3], x
    [13], c_obj_next_next_next_next_next_->S->contents, x[12],
    c_obj_next_next_next_next_next_->V->contents,
    c_obj_next_next_next_next_next_->W_act_motor_du->contents,
    c_obj_next_next_next_next_next_->W_act_theta_du->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_el_du->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_az_du->contents,
    c_obj_next_next_next_next_next_->W_act_phi_du->contents,
    c_obj_next_next_next_next_next_->W_act_ailerons_du->contents,
    c_obj_next_next_next_next_next_->W_act_phi->contents,
    c_obj_next_next_next_next_next_->W_act_theta->contents,
    c_obj_next_next_next_next_next_->W_act_motor->contents,
    c_obj_next_next_next_next_next_->W_dv_1->contents,
    c_obj_next_next_next_next_next_->W_dv_2->contents,
    c_obj_next_next_next_next_next_->W_dv_3->contents,
    c_obj_next_next_next_next_next_->W_dv_4->contents,
    c_obj_next_next_next_next_next_->W_dv_5->contents,
    c_obj_next_next_next_next_next_->W_dv_6->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_el->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_az->contents,
    c_obj_next_next_next_next_next_->W_act_ailerons->contents, x[4], x[5], x[6],
    x[7], x[14], c_obj_next_next_next_next_next_->desired_el_value->contents,
    c_obj_next_next_next_next_next_->desired_az_value->contents,
    c_obj_next_next_next_next_next_->desired_phi_value->contents,
    c_obj_next_next_next_next_next_->desired_theta_value->contents,
    c_obj_next_next_next_next_next_->desired_motor_value->contents,
    c_obj_next_next_next_next_next_->desired_ailerons_value->contents,
    dv_global_1, dv_global_2, dv_global_3, dv_global_4, dv_global_5, dv_global_6,
    c_obj_next_next_next_next_next_->flight_path_angle->contents, x[8], x[9], x
    [10], x[11], c_obj_next_next_next_next_next_->gain_el->contents,
    c_obj_next_next_next_next_next_->gain_az->contents,
    c_obj_next_next_next_next_next_->gain_phi->contents,
    c_obj_next_next_next_next_next_->gain_theta->contents,
    c_obj_next_next_next_next_next_->gain_motor->contents,
    c_obj_next_next_next_next_next_->gain_ailerons->contents,
    c_obj_next_next_next_next_next_->gamma_quadratic_du->contents,
    c_obj_next_next_next_next_next_->gamma_quadratic_du2->contents,
    d_obj_next_next_next_next_next_, c_obj_next_next_next_next_next_->
    l_1->contents, c_obj_next_next_next_next_next_->l_2->contents,
    c_obj_next_next_next_next_next_->l_3->contents,
    c_obj_next_next_next_next_next_->l_4->contents,
    c_obj_next_next_next_next_next_->l_z->contents,
    c_obj_next_next_next_next_next_->m->contents,
    c_obj_next_next_next_next_next_->p->contents,
    c_obj_next_next_next_next_next_->q->contents,
    c_obj_next_next_next_next_next_->r->contents,
    c_obj_next_next_next_next_next_->rho->contents,
    c_obj_next_next_next_next_next_->wing_chord->contents, gradient);

  /*  else */
  /*  [cost] = compute_cost_and_gradient_first_iteration_v2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_motor_du,W_act_theta_du,W_act_phi_du,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,b_1_scaled,b_2_scaled,b_3_scaled,b_4_scaled,delta_ailerons_scaled,desired_el_value,desired_az_value,desired_phi_value,desired_theta_value,desired_motor_value,desired_ailerons_value,dv_global_1,dv_global_2,dv_global_3,dv_global_4,dv_global_5,dv_global_6,flight_path_angle,g_1_scaled,g_2_scaled,g_3_scaled,g_4_scaled,gain_el,gain_az,gain_phi,gain_theta,gain_motor,gain_ailerons,gamma_quadratic_du,gamma_quadratic_du2,previous_controls,l_1,l_2,l_3,l_4,l_z,m,p,q,r,rho,wing_chord); */
  /*  end */
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
      allFinite = ((!rtIsInf(grad_workspace[idx_current])) && (!rtIsNaN
        (grad_workspace[idx_current])));
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
