/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 07-May-2024 15:18:42
 */

/* Include Files */
#include "evalObjAndConstr.h"
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "Cascaded_nonlinear_controller_w_ail_new_aero_rtwutil.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const e_struct_T *c_obj_next_next_next_next_next_
 *                const double x[13]
 *                int *status
 * Return Type  : double
 */
double b_evalObjAndConstr(const e_struct_T *c_obj_next_next_next_next_next_,
                          const double x[13], int *status)
{
  double CL_aileron;
  double Cd_zero;
  double Cl_alpha;
  double Cm_alpha;
  double Cm_zero;
  double I_xx;
  double I_yy;
  double I_zz;
  double Omega_1_scaled;
  double Omega_2_scaled;
  double Omega_3_scaled;
  double Omega_4_scaled;
  double S;
  double V;
  double W_act_ailerons;
  double W_act_motor;
  double W_act_tilt_az;
  double W_act_tilt_el;
  double W_dv_1;
  double W_dv_2;
  double W_dv_3;
  double W_dv_4;
  double W_dv_5;
  double W_dv_6;
  double b_1_scaled;
  double b_2_scaled;
  double b_3_scaled;
  double b_4_scaled;
  double ct_idx_88;
  double d;
  double delta_ailerons_scaled;
  double desired_ailerons_value;
  double desired_motor_value;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double fval;
  double g_1_scaled;
  double g_2_scaled;
  double g_3_scaled;
  double g_4_scaled;
  double gain_ailerons;
  double gain_motor;
  double gamma_quadratic_du;
  double l_1;
  double l_2;
  double l_3;
  double l_4;
  double l_z;
  double m;
  double p;
  double power_Cd_0;
  double power_Cd_a;
  double prop_Cl_0;
  double prop_Cl_a;
  double prop_R;
  double prop_delta;
  double prop_sigma;
  double prop_theta;
  double q;
  double r;
  double rho;
  double t10;
  double t103;
  double t103_tmp;
  double t104;
  double t106;
  double t107;
  double t11;
  double t114;
  double t115;
  double t118;
  double t119;
  double t12;
  double t120;
  double t129;
  double t13;
  double t132;
  double t134;
  double t138;
  double t139;
  double t14;
  double t140;
  double t141;
  double t144;
  double t145;
  double t146;
  double t147;
  double t148;
  double t149;
  double t15;
  double t150;
  double t155;
  double t156;
  double t157;
  double t158;
  double t16;
  double t162_tmp;
  double t175_tmp;
  double t175_tmp_tmp;
  double t19;
  double t21;
  double t23;
  double t25;
  double t27;
  double t28;
  double t3;
  double t307;
  double t356;
  double t356_tmp;
  double t357;
  double t357_tmp;
  double t358;
  double t358_tmp;
  double t359;
  double t359_tmp;
  double t364;
  double t365;
  double t366;
  double t367;
  double t39;
  double t4;
  double t40;
  double t42;
  double t43;
  double t47;
  double t48;
  double t5;
  double t55;
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
  double t719;
  double t721;
  double t74;
  double t75;
  double t76;
  double t78;
  double t79;
  double t8;
  double t802;
  double t81;
  double t82;
  double t84;
  double t89;
  double t9;
  double t90;
  double t95;
  double t97;
  double t98;
  double t99;
  double wing_chord;
  double wing_span;
  bool b;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  t719 = c_obj_next_next_next_next_next_->Beta->contents;
  CL_aileron = c_obj_next_next_next_next_next_->CL_aileron->contents;
  Cd_zero = c_obj_next_next_next_next_next_->Cd_zero->contents;
  Cl_alpha = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  Cm_zero = c_obj_next_next_next_next_next_->Cm_zero->contents;
  Cm_alpha = c_obj_next_next_next_next_next_->Cm_alpha->contents;
  I_xx = c_obj_next_next_next_next_next_->I_xx->contents;
  I_yy = c_obj_next_next_next_next_next_->I_yy->contents;
  I_zz = c_obj_next_next_next_next_next_->I_zz->contents;
  t721 = c_obj_next_next_next_next_next_->K_Cd->contents;
  Omega_1_scaled = x[0];
  Omega_2_scaled = x[1];
  Omega_3_scaled = x[2];
  Omega_4_scaled = x[3];
  t16 = c_obj_next_next_next_next_next_->Phi->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  t144 = c_obj_next_next_next_next_next_->Theta->contents;
  V = c_obj_next_next_next_next_next_->V->contents;
  t138 = c_obj_next_next_next_next_next_->V_scaled->contents;
  W_act_motor = c_obj_next_next_next_next_next_->W_act_motor->contents;
  W_dv_1 = c_obj_next_next_next_next_next_->W_dv_1->contents;
  W_dv_2 = c_obj_next_next_next_next_next_->W_dv_2->contents;
  W_dv_3 = c_obj_next_next_next_next_next_->W_dv_3->contents;
  W_dv_4 = c_obj_next_next_next_next_next_->W_dv_4->contents;
  W_dv_5 = c_obj_next_next_next_next_next_->W_dv_5->contents;
  W_dv_6 = c_obj_next_next_next_next_next_->W_dv_6->contents;
  W_act_tilt_el = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  W_act_tilt_az = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  W_act_ailerons = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  b_1_scaled = x[4];
  b_2_scaled = x[5];
  b_3_scaled = x[6];
  b_4_scaled = x[7];
  delta_ailerons_scaled = x[12];
  t134 = c_obj_next_next_next_next_next_->desired_el_value->contents;
  t141 = c_obj_next_next_next_next_next_->desired_az_value->contents;
  desired_motor_value =
      c_obj_next_next_next_next_next_->desired_motor_value->contents;
  desired_ailerons_value =
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents;
  t145 = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  g_1_scaled = x[8];
  g_2_scaled = x[9];
  g_3_scaled = x[10];
  g_4_scaled = x[11];
  t140 = c_obj_next_next_next_next_next_->gain_el->contents;
  t802 = c_obj_next_next_next_next_next_->gain_az->contents;
  gain_motor = c_obj_next_next_next_next_next_->gain_motor->contents;
  gain_ailerons = c_obj_next_next_next_next_next_->gain_ailerons->contents;
  gamma_quadratic_du =
      c_obj_next_next_next_next_next_->gamma_quadratic_du->contents;
  l_1 = c_obj_next_next_next_next_next_->l_1->contents;
  l_2 = c_obj_next_next_next_next_next_->l_2->contents;
  l_3 = c_obj_next_next_next_next_next_->l_3->contents;
  l_4 = c_obj_next_next_next_next_next_->l_4->contents;
  l_z = c_obj_next_next_next_next_next_->l_z->contents;
  m = c_obj_next_next_next_next_next_->m->contents;
  p = c_obj_next_next_next_next_next_->p->contents;
  power_Cd_0 = c_obj_next_next_next_next_next_->power_Cd_0->contents;
  power_Cd_a = c_obj_next_next_next_next_next_->power_Cd_a->contents;
  prop_R = c_obj_next_next_next_next_next_->prop_R->contents;
  t139 = c_obj_next_next_next_next_next_->prop_Cd_0->contents;
  prop_Cl_0 = c_obj_next_next_next_next_next_->prop_Cl_0->contents;
  t132 = c_obj_next_next_next_next_next_->prop_Cd_a->contents;
  prop_Cl_a = c_obj_next_next_next_next_next_->prop_Cl_a->contents;
  prop_delta = c_obj_next_next_next_next_next_->prop_delta->contents;
  prop_sigma = c_obj_next_next_next_next_next_->prop_sigma->contents;
  prop_theta = c_obj_next_next_next_next_next_->prop_theta->contents;
  q = c_obj_next_next_next_next_next_->q->contents;
  r = c_obj_next_next_next_next_next_->r->contents;
  rho = c_obj_next_next_next_next_next_->rho->contents;
  wing_span = c_obj_next_next_next_next_next_->wing_span->contents;
  wing_chord = c_obj_next_next_next_next_next_->wing_chord->contents;
  /* compute_cost_and_gradient_w_Mx_noah_inner */
  /*     [COST,GRADIENT] =
   * compute_cost_and_gradient_w_Mx_noah_inner(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi,S,Theta,V,V_scaled,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,power_Cd_0,power_Cd_a,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_SPAN,WING_CHORD)
   */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     02-May-2024 14:31:18 */
  t3 = cos(t16);
  t4 = sin(t719);
  t5 = cos(t144);
  t6 = sin(t16);
  t7 = sin(t144);
  t8 = log(prop_delta);
  t9 = b_1_scaled * t140;
  t10 = b_2_scaled * t140;
  t11 = b_3_scaled * t140;
  t12 = b_4_scaled * t140;
  t13 = g_1_scaled * t802;
  t14 = g_2_scaled * t802;
  t15 = g_3_scaled * t802;
  t16 = g_4_scaled * t802;
  t19 = Omega_1_scaled * Omega_1_scaled;
  t21 = Omega_2_scaled * Omega_2_scaled;
  t23 = Omega_3_scaled * Omega_3_scaled;
  t25 = Omega_4_scaled * Omega_4_scaled;
  t27 = V * V;
  t28 = t138 * t138;
  t39 = gain_motor * gain_motor;
  t40 = t132 * 2.0;
  t42 = rt_powd_snf(prop_R, 3.0);
  t43 = rt_powd_snf(prop_R, 4.0);
  t47 = t139 * prop_delta * 2.0;
  t74 = 1.0 / Omega_1_scaled;
  t76 = 1.0 / Omega_2_scaled;
  t79 = 1.0 / Omega_3_scaled;
  t82 = 1.0 / Omega_4_scaled;
  t89 = 1.0 / gain_motor;
  t95 = prop_delta * 16.0;
  t97 = 1.0 / prop_R;
  t99 = 1.0 / prop_delta;
  t48 = prop_theta * t40;
  t55 = cos(t9);
  t56 = cos(t10);
  t57 = cos(t11);
  t58 = cos(t12);
  t59 = cos(t13);
  t60 = cos(t14);
  t61 = cos(t15);
  t62 = cos(t16);
  t63 = sin(t9);
  t64 = sin(t10);
  t65 = sin(t11);
  t66 = sin(t12);
  t67 = sin(t13);
  t68 = sin(t14);
  t69 = sin(t15);
  t70 = sin(t16);
  t75 = 1.0 / t19;
  t78 = 1.0 / t21;
  t81 = 1.0 / t23;
  t84 = 1.0 / t25;
  t90 = 1.0 / t39;
  t98 = t97 * t97;
  t103_tmp = prop_Cl_0 * prop_delta;
  t103 = t103_tmp * (prop_delta + 1.0);
  t104 = t144 - t145;
  t106 = t134 * (1.0 / t140);
  t107 = t141 * (1.0 / t802);
  t132 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t134 = prop_Cl_a * prop_sigma * (prop_delta - 1.0) / 8.0;
  t114 = t57 * t57;
  t115 = t58 * t58;
  t118 = cos(t104);
  t119 = sin(t104);
  t120 = t103 * 8.0;
  t129 = power_Cd_0 * prop_delta + power_Cd_a * (prop_theta * prop_theta);
  t138 = (t9 + 1.5707963267948966) + t104;
  t139 = (t10 + 1.5707963267948966) + t104;
  t140 = (t11 + 1.5707963267948966) + t104;
  t141 = (t12 + 1.5707963267948966) + t104;
  t15 = cos(t138);
  t16 = cos(t139);
  t144 = cos(t140);
  t145 = cos(t141);
  t146 = sin(t138);
  t147 = sin(t139);
  t148 = sin(t140);
  t149 = sin(t141);
  t162_tmp = Cl_alpha * S * rho * t27 * t104;
  t150 = Cd_zero + t721 * (Cl_alpha * Cl_alpha) * (t104 * t104);
  t155 = t146 * t146;
  t156 = t147 * t147;
  t157 = t148 * t148;
  t158 = t149 * t149;
  t175_tmp_tmp = S * rho;
  t175_tmp = t175_tmp_tmp * cos(t719) * t27;
  t307 = t162_tmp * t118 / 2.0 + t175_tmp * t119 * t150 / 2.0;
  t356_tmp = t27 * t75 * t90 * t98;
  t802 = V * prop_Cl_a * prop_sigma;
  t13 = prop_Cl_0 * prop_sigma * t8 * t27;
  t14 = prop_sigma * (prop_delta - 1.0) * t99;
  t356 = (t134 + V * t74 * t89 * t97 * t15 / 2.0) +
         sqrt(((t356_tmp * (t15 * t15) * 16.0 +
                t802 * t74 * t89 * (prop_delta - 1.0) * t97 * t15 * 8.0) +
               t13 * t75 * t90 * t98 * t155 * -8.0) -
              t14 * (t120 +
                     prop_Cl_a *
                         (t132 + prop_theta * (t95 + t356_tmp * t155 * 8.0)))) /
             8.0;
  t357_tmp = t27 * t78 * t90 * t98;
  t357 = (t134 + V * t76 * t89 * t97 * t16 / 2.0) +
         sqrt(((t357_tmp * (t16 * t16) * 16.0 +
                t802 * t76 * t89 * (prop_delta - 1.0) * t97 * t16 * 8.0) +
               t13 * t78 * t90 * t98 * t156 * -8.0) -
              t14 * (t120 +
                     prop_Cl_a *
                         (t132 + prop_theta * (t95 + t357_tmp * t156 * 8.0)))) /
             8.0;
  t358_tmp = t27 * t81 * t90 * t98;
  t358 = (t134 + V * t79 * t89 * t97 * t144 / 2.0) +
         sqrt(((t358_tmp * (t144 * t144) * 16.0 +
                t802 * t79 * t89 * (prop_delta - 1.0) * t97 * t144 * 8.0) +
               t13 * t81 * t90 * t98 * t157 * -8.0) -
              t14 * (t120 +
                     prop_Cl_a *
                         (t132 + prop_theta * (t95 + t358_tmp * t157 * 8.0)))) /
             8.0;
  t359_tmp = t27 * t84 * t90 * t98;
  t359 = (t134 + V * t82 * t89 * t97 * t145 / 2.0) +
         sqrt(((t359_tmp * (t145 * t145) * 16.0 +
                t802 * t82 * t89 * (prop_delta - 1.0) * t97 * t145 * 8.0) +
               t13 * t84 * t90 * t98 * t158 * -8.0) -
              t14 * (t120 +
                     prop_Cl_a *
                         (t132 + prop_theta * (t95 + t359_tmp * t158 * 8.0)))) /
             8.0;
  t364 = -prop_theta + t356;
  t365 = -prop_theta + t357;
  t366 = -prop_theta + t358;
  t367 = -prop_theta + t359;
  d = rt_powd_snf(prop_R, 5.0);
  desired_motor_value *= t89;
  t89 = power_Cd_0 * ((prop_delta + 1.0) + prop_delta * prop_delta) * 2.0;
  prop_R = prop_delta / 6.0 - 0.16666666666666666;
  ct_idx_88 = W_act_motor * W_act_motor;
  W_act_tilt_el *= W_act_tilt_el;
  W_act_motor = W_act_tilt_az * W_act_tilt_az;
  power_Cd_0 = 1.0 / m;
  t14 = t103_tmp * t8;
  t802 = t14 * t27;
  Cd_zero = prop_Cl_a * prop_theta * t27;
  t13 = prop_Cl_a * prop_delta;
  t76 = t802 * t75 * t90 * t98 * t155 +
        (prop_delta - 1.0) *
            ((t103 + Cd_zero * t75 * t90 * t98 * t155) - t13 * t364 * 2.0);
  t79 = t802 * t78 * t90 * t98 * t156 +
        (prop_delta - 1.0) *
            ((t103 + Cd_zero * t78 * t90 * t98 * t156) - t13 * t365 * 2.0);
  t82 = t802 * t81 * t90 * t98 * t157 +
        (prop_delta - 1.0) *
            ((t103 + Cd_zero * t81 * t90 * t98 * t157) - t13 * t366 * 2.0);
  t12 = t802 * t84 * t90 * t98 * t158 +
        (prop_delta - 1.0) *
            ((t103 + Cd_zero * t84 * t90 * t98 * t158) - t13 * t367 * 2.0);
  t802 = prop_Cl_a - t40;
  t9 = t14 * t356 +
       (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t356 * t802));
  t10 = t14 * t357 +
        (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t357 * t802));
  t11 = t14 * t358 +
        (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t358 * t802));
  t141 = t14 * t359 +
         (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t359 * t802));
  Cd_zero = prop_sigma * rho;
  t139 = Cd_zero * t19 * t39 * t43;
  t16 = t139 * t55;
  t144 = Omega_1_scaled * V * gain_motor * prop_sigma * rho * t42;
  t97 = t16 * t59 * t99 * t76 * 3.1415926535897931 / 4.0 +
        t144 * t59 * t63 * t99 * t146 * t9 * 3.1415926535897931 * -0.25;
  t140 = Cd_zero * t21 * t39 * t43;
  t145 = t140 * t56;
  t132 = Omega_2_scaled * V * gain_motor * prop_sigma * rho * t42;
  t95 = t145 * t60 * t99 * t79 * 3.1415926535897931 / 4.0 +
        t132 * t60 * t64 * t99 * t147 * t10 * 3.1415926535897931 * -0.25;
  Cl_alpha = Cd_zero * t23 * t39 * t43;
  t14 = Cl_alpha * t57;
  t15 = Omega_3_scaled * V * gain_motor * prop_sigma * rho * t42;
  t74 = t14 * t61 * t99 * t82 * 3.1415926535897931 / 4.0 +
        t15 * t61 * t65 * t99 * t148 * t11 * 3.1415926535897931 * -0.25;
  t134 = Cd_zero * t25 * t39 * t43;
  t802 = t134 * t58;
  t13 = Omega_4_scaled * V * gain_motor * prop_sigma * rho * t42;
  t719 = t802 * t62 * t99 * t12 * 3.1415926535897931 / 4.0 +
         t13 * t62 * t66 * t99 * t149 * t141 * 3.1415926535897931 * -0.25;
  t120 = t16 * t67 * t99 * t76 * 3.1415926535897931 / 4.0 +
         t144 * t63 * t67 * t99 * t146 * t9 * 3.1415926535897931 * -0.25;
  t721 = t145 * t68 * t99 * t79 * 3.1415926535897931 / 4.0 +
         t132 * t64 * t68 * t99 * t147 * t10 * 3.1415926535897931 * -0.25;
  t16 = t14 * t69 * t99 * t82 * 3.1415926535897931 / 4.0 +
        t15 * t65 * t69 * t99 * t148 * t11 * 3.1415926535897931 * -0.25;
  t15 = t802 * t70 * t99 * t12 * 3.1415926535897931 / 4.0 +
        t13 * t66 * t70 * t99 * t149 * t141 * 3.1415926535897931 * -0.25;
  t138 = V * gain_motor * prop_sigma * rho * t42;
  t14 = t139 * t63 * t99 * t76 * 3.1415926535897931 / 4.0 +
        Omega_1_scaled *
            (t138 * t55 * t99 * t146 * t9 * 3.1415926535897931 / 4.0);
  t13 = t140 * t64 * t99 * t79 * 3.1415926535897931 / 4.0 +
        Omega_2_scaled *
            (t138 * t56 * t99 * t147 * t10 * 3.1415926535897931 / 4.0);
  t132 = Cl_alpha * t65 * t99 * t82 * 3.1415926535897931 / 4.0 +
         Omega_3_scaled *
             (t138 * t57 * t99 * t148 * t11 * 3.1415926535897931 / 4.0);
  t10 = t134 * t66 * t99 * t12 * 3.1415926535897931 / 4.0 +
        Omega_4_scaled *
            (t138 * t58 * t99 * t149 * t141 * 3.1415926535897931 / 4.0);
  t138 = prop_Cl_0 * (prop_delta + 1.0);
  Cl_alpha =
      desired_motor_value +
      Cd_zero * rt_powd_snf(Omega_1_scaled, 3.0) * t39 * d *
          1.5707963267948966 * prop_R *
          (((t89 + t356_tmp * t99 * t129 * t155 * 3.0) + t138 * t356 * 3.0) -
           t364 * (prop_Cl_a * t356 * 6.0 - power_Cd_a * t364 * 6.0));
  t12 = desired_motor_value +
        Cd_zero * rt_powd_snf(Omega_2_scaled, 3.0) * t39 * d *
            1.5707963267948966 * prop_R *
            (((t89 + t357_tmp * t99 * t129 * t156 * 3.0) + t138 * t357 * 3.0) -
             t365 * (prop_Cl_a * t357 * 6.0 - power_Cd_a * t365 * 6.0));
  t11 = desired_motor_value +
        Cd_zero * rt_powd_snf(Omega_3_scaled, 3.0) * t39 * d *
            1.5707963267948966 * prop_R *
            (((t89 + t358_tmp * t99 * t129 * t157 * 3.0) + t138 * t358 * 3.0) -
             t366 * (prop_Cl_a * t358 * 6.0 - power_Cd_a * t366 * 6.0));
  Cd_zero =
      desired_motor_value +
      Cd_zero * rt_powd_snf(Omega_4_scaled, 3.0) * t39 * d *
          1.5707963267948966 * prop_R *
          (((t89 + t359_tmp * t99 * t129 * t158 * 3.0) + t138 * t359 * 3.0) -
           t367 * (prop_Cl_a * t359 * 6.0 - power_Cd_a * t367 * 6.0));
  t140 = ((t120 + t721) + t16) + t15;
  t141 = ((t97 + t95) + t74) + t719;
  t802 = ((t14 + t13) + t132) + t10;
  t138 = I_xx * p;
  t9 = dv_global_6 +
       -(1.0 / I_zz) *
           (((((((((t138 * q - I_yy * p * q) + l_1 * t14) - l_1 * t13) -
                 l_2 * t132) +
                l_2 * t10) -
               l_4 * t120) -
              l_4 * t721) +
             l_3 * t16) +
            l_3 * t15);
  t134 = t175_tmp_tmp * t27;
  t145 = dv_global_5 +
         1.0 / I_yy *
             ((((((((((-(I_zz * p * r) + t138 * r) -
                      t134 * (Cm_zero + Cm_alpha * t104) * wing_chord / 2.0) +
                     l_z * t14) +
                    l_z * t13) +
                   l_z * t132) +
                  l_z * t10) +
                 l_4 * t97) +
                l_4 * t95) -
               l_3 * t74) -
              l_3 * t719);
  t144 =
      dv_global_2 +
      power_Cd_0 * (((t175_tmp_tmp * t3 * t4 * t27 * t150 / 2.0 - t6 * t307) +
                     t3 * t140) +
                    t6 * t141);
  t138 = Omega_2_scaled * t65;
  t10 = Omega_1_scaled * t66;
  t15 =
      dv_global_4 +
      1.0 / I_xx *
          (((((((((((-(I_yy * q * r) + I_zz * q * r) -
                    CL_aileron * S * delta_ailerons_scaled * gain_ailerons *
                        rho * t27 / 2.0) +
                   t134 *
                       (((((((((((Omega_1_scaled * t55 * 0.01228634392023026 -
                                  Omega_2_scaled * t56 * 0.01228634392023026) +
                                 t19 * t55 * 0.0075456152077779167) +
                                Omega_2_scaled * t28 * t56 *
                                    0.020516396677824081) -
                               t21 * t56 * 0.0075456152077779167) -
                              Omega_1_scaled * t28 * t55 *
                                  0.020516396677824081) +
                             t138 * t114 * 0.0064381447596962606) +
                            t138 * (t56 * t56) * 0.0039349871274520724) +
                           Omega_1_scaled * t63 * t115 *
                               0.0046429750925043979) -
                          t10 * t115 * 0.0064381447596962606) -
                         t10 * (t55 * t55) * 0.0039349871274520724) -
                        Omega_2_scaled * t64 * t114 * 0.0046429750925043979) *
                       wing_span / 2.0) +
                  l_1 * t97) +
                 l_2 * t719) +
                l_z * t120) +
               l_z * t721) +
              l_z * t16) +
             l_z * t15) -
            l_1 * t95) -
           l_2 * t74);
  t138 = t3 * t5;
  t134 = t175_tmp_tmp * t4;
  t10 = t162_tmp * t119 / 2.0 - t175_tmp * t118 * t150 / 2.0;
  t14 = (dv_global_3 +
         power_Cd_0 *
             (((((t134 * t5 * t6 * t27 * t150 / 2.0 + t138 * t307) + t7 * t10) +
                t7 * t802) +
               t5 * t6 * t140) -
              t138 * t141)) -
        9.81;
  t138 = t3 * t7;
  t13 = dv_global_1 +
        power_Cd_0 *
            (((((t134 * t6 * t7 * t27 * t150 / 2.0 + -t5 * t10) + t138 * t307) -
               t5 * t802) +
              t6 * t7 * t140) -
             t138 * t141);
  t802 = delta_ailerons_scaled - desired_ailerons_value * (1.0 / gain_ailerons);
  t141 = b_1_scaled - t106;
  t140 = b_2_scaled - t106;
  t134 = b_3_scaled - t106;
  t139 = b_4_scaled - t106;
  t132 = g_1_scaled - t107;
  t138 = g_2_scaled - t107;
  t10 = g_3_scaled - t107;
  t16 = g_4_scaled - t107;
  fval = (((((gamma_quadratic_du *
                  ((((((((((((W_act_ailerons * W_act_ailerons * (t802 * t802) +
                              W_act_tilt_el * (t141 * t141)) +
                             W_act_tilt_el * (t140 * t140)) +
                            W_act_tilt_el * (t134 * t134)) +
                           W_act_tilt_el * (t139 * t139)) +
                          W_act_motor * (t132 * t132)) +
                         W_act_motor * (t138 * t138)) +
                        W_act_motor * (t10 * t10)) +
                       W_act_motor * (t16 * t16)) +
                      ct_idx_88 * (Cl_alpha * Cl_alpha)) +
                     ct_idx_88 * (t12 * t12)) +
                    ct_idx_88 * (t11 * t11)) +
                   ct_idx_88 * (Cd_zero * Cd_zero)) +
              W_dv_6 * W_dv_6 * (t9 * t9)) +
             W_dv_2 * W_dv_2 * (t144 * t144)) +
            W_dv_5 * W_dv_5 * (t145 * t145)) +
           W_dv_4 * W_dv_4 * (t15 * t15)) +
          W_dv_1 * W_dv_1 * (t13 * t13)) +
         W_dv_3 * W_dv_3 * (t14 * t14);
  *status = 1;
  b = rtIsNaN(fval);
  if (rtIsInf(fval) || b) {
    if (b) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
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
 *                int *status
 * Return Type  : double
 */
double evalObjAndConstr(const d_struct_T *c_obj_next_next_next_next_next_,
                        const double x[15], int *status)
{
  double Beta;
  double CL_aileron;
  double Cl_alpha;
  double Cm_alpha;
  double Cm_zero;
  double I_xx;
  double I_yy;
  double I_zz;
  double S;
  double V;
  double W_act_ailerons;
  double W_act_motor;
  double W_act_phi;
  double W_act_theta;
  double W_act_tilt_az;
  double W_act_tilt_el;
  double W_dv_1;
  double W_dv_2;
  double W_dv_3;
  double W_dv_4;
  double W_dv_5;
  double W_dv_6;
  double ct_idx_114;
  double ct_idx_52;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double desired_ailerons_value;
  double desired_motor_value;
  double desired_phi_value;
  double desired_theta_value;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double fval;
  double gain_ailerons;
  double gain_motor;
  double gain_phi;
  double gain_theta;
  double gamma_quadratic_du;
  double l_1;
  double l_2;
  double l_3;
  double l_4;
  double l_z;
  double m;
  double p;
  double power_Cd_0;
  double power_Cd_a;
  double prop_Cd_a;
  double prop_Cl_0;
  double prop_Cl_a;
  double prop_delta;
  double prop_sigma;
  double prop_theta;
  double q;
  double r;
  double rho;
  double t10;
  double t101;
  double t103;
  double t104;
  double t105;
  double t11;
  double t110;
  double t110_tmp;
  double t112;
  double t113;
  double t12;
  double t120;
  double t121;
  double t124;
  double t126;
  double t127;
  double t13;
  double t134;
  double t139;
  double t14;
  double t140;
  double t146;
  double t147;
  double t148;
  double t149;
  double t150;
  double t151;
  double t152;
  double t155;
  double t156;
  double t157;
  double t158;
  double t165;
  double t166;
  double t167;
  double t168;
  double t17;
  double t19;
  double t21;
  double t23;
  double t25;
  double t26;
  double t350;
  double t350_tmp_tmp;
  double t351;
  double t39;
  double t4;
  double t408;
  double t408_tmp;
  double t409;
  double t409_tmp;
  double t410;
  double t410_tmp;
  double t411;
  double t411_tmp;
  double t416;
  double t417;
  double t418;
  double t419;
  double t43;
  double t47;
  double t49;
  double t5;
  double t50;
  double t53;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t62;
  double t641;
  double t642;
  double t643;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t78;
  double t79;
  double t8;
  double t80;
  double t82;
  double t826;
  double t83;
  double t85;
  double t86;
  double t88;
  double t9;
  double t95;
  double t953;
  double t96;
  double wing_chord;
  double wing_span;
  bool b;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  Beta = c_obj_next_next_next_next_next_->Beta->contents;
  CL_aileron = c_obj_next_next_next_next_next_->CL_aileron->contents;
  t643 = c_obj_next_next_next_next_next_->Cd_zero->contents;
  Cl_alpha = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  Cm_zero = c_obj_next_next_next_next_next_->Cm_zero->contents;
  Cm_alpha = c_obj_next_next_next_next_next_->Cm_alpha->contents;
  I_xx = c_obj_next_next_next_next_next_->I_xx->contents;
  I_yy = c_obj_next_next_next_next_next_->I_yy->contents;
  I_zz = c_obj_next_next_next_next_next_->I_zz->contents;
  t642 = c_obj_next_next_next_next_next_->K_Cd->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  V = c_obj_next_next_next_next_next_->V->contents;
  t78 = c_obj_next_next_next_next_next_->V_scaled->contents;
  W_act_phi = c_obj_next_next_next_next_next_->W_act_phi->contents;
  W_act_theta = c_obj_next_next_next_next_next_->W_act_theta->contents;
  W_act_motor = c_obj_next_next_next_next_next_->W_act_motor->contents;
  W_dv_1 = c_obj_next_next_next_next_next_->W_dv_1->contents;
  W_dv_2 = c_obj_next_next_next_next_next_->W_dv_2->contents;
  W_dv_3 = c_obj_next_next_next_next_next_->W_dv_3->contents;
  W_dv_4 = c_obj_next_next_next_next_next_->W_dv_4->contents;
  W_dv_5 = c_obj_next_next_next_next_next_->W_dv_5->contents;
  W_dv_6 = c_obj_next_next_next_next_next_->W_dv_6->contents;
  W_act_tilt_el = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  W_act_tilt_az = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  W_act_ailerons = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  t641 = c_obj_next_next_next_next_next_->desired_el_value->contents;
  t152 = c_obj_next_next_next_next_next_->desired_az_value->contents;
  desired_phi_value =
      c_obj_next_next_next_next_next_->desired_phi_value->contents;
  desired_theta_value =
      c_obj_next_next_next_next_next_->desired_theta_value->contents;
  desired_motor_value =
      c_obj_next_next_next_next_next_->desired_motor_value->contents;
  desired_ailerons_value =
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents;
  t953 = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  t151 = c_obj_next_next_next_next_next_->gain_el->contents;
  t139 = c_obj_next_next_next_next_next_->gain_az->contents;
  gain_phi = c_obj_next_next_next_next_next_->gain_phi->contents;
  gain_theta = c_obj_next_next_next_next_next_->gain_theta->contents;
  gain_motor = c_obj_next_next_next_next_next_->gain_motor->contents;
  gain_ailerons = c_obj_next_next_next_next_next_->gain_ailerons->contents;
  gamma_quadratic_du =
      c_obj_next_next_next_next_next_->gamma_quadratic_du->contents;
  l_1 = c_obj_next_next_next_next_next_->l_1->contents;
  l_2 = c_obj_next_next_next_next_next_->l_2->contents;
  l_3 = c_obj_next_next_next_next_next_->l_3->contents;
  l_4 = c_obj_next_next_next_next_next_->l_4->contents;
  l_z = c_obj_next_next_next_next_next_->l_z->contents;
  m = c_obj_next_next_next_next_next_->m->contents;
  p = c_obj_next_next_next_next_next_->p->contents;
  power_Cd_0 = c_obj_next_next_next_next_next_->power_Cd_0->contents;
  power_Cd_a = c_obj_next_next_next_next_next_->power_Cd_a->contents;
  t826 = c_obj_next_next_next_next_next_->prop_R->contents;
  t146 = c_obj_next_next_next_next_next_->prop_Cd_0->contents;
  prop_Cl_0 = c_obj_next_next_next_next_next_->prop_Cl_0->contents;
  prop_Cd_a = c_obj_next_next_next_next_next_->prop_Cd_a->contents;
  prop_Cl_a = c_obj_next_next_next_next_next_->prop_Cl_a->contents;
  prop_delta = c_obj_next_next_next_next_next_->prop_delta->contents;
  prop_sigma = c_obj_next_next_next_next_next_->prop_sigma->contents;
  prop_theta = c_obj_next_next_next_next_next_->prop_theta->contents;
  q = c_obj_next_next_next_next_next_->q->contents;
  r = c_obj_next_next_next_next_next_->r->contents;
  rho = c_obj_next_next_next_next_next_->rho->contents;
  wing_span = c_obj_next_next_next_next_next_->wing_span->contents;
  wing_chord = c_obj_next_next_next_next_next_->wing_chord->contents;
  /* compute_cost_and_gradient_w_Mx_noah_outer */
  /*     [COST,GRADIENT] =
   * compute_cost_and_gradient_w_Mx_noah_outer(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,V_scaled,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,power_Cd_0,power_Cd_a,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_SPAN,WING_CHORD)
   */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     02-May-2024 14:33:01 */
  t4 = log(prop_delta);
  t5 = x[13] * gain_phi;
  t6 = x[12] * gain_theta;
  t7 = x[4] * t151;
  t8 = x[5] * t151;
  t9 = x[6] * t151;
  t10 = x[7] * t151;
  t11 = x[8] * t139;
  t12 = x[9] * t139;
  t13 = x[10] * t139;
  t14 = x[11] * t139;
  t17 = x[0] * x[0];
  t19 = x[1] * x[1];
  t21 = x[2] * x[2];
  t23 = x[3] * x[3];
  t25 = V * V;
  t26 = t78 * t78;
  t39 = gain_motor * gain_motor;
  t43 = rt_powd_snf(t826, 4.0);
  t47 = t146 * prop_delta * 2.0;
  t78 = 1.0 / x[0];
  t80 = 1.0 / x[1];
  t83 = 1.0 / x[2];
  t86 = 1.0 / x[3];
  t95 = 1.0 / gain_motor;
  t101 = prop_delta * 16.0;
  t103 = 1.0 / t826;
  t105 = 1.0 / prop_delta;
  t49 = cos(t5);
  t50 = cos(t6);
  t53 = sin(t5);
  t58 = cos(t7);
  t59 = cos(t8);
  t60 = cos(t9);
  t61 = cos(t10);
  t62 = sin(t6);
  t67 = sin(t7);
  t68 = sin(t8);
  t69 = sin(t9);
  t70 = sin(t10);
  t79 = 1.0 / t17;
  t82 = 1.0 / t19;
  t85 = 1.0 / t21;
  t88 = 1.0 / t23;
  t96 = 1.0 / t39;
  t104 = t103 * t103;
  t110_tmp = prop_Cl_0 * prop_delta;
  t110 = t110_tmp * (prop_delta + 1.0);
  t112 = t641 * (1.0 / t151);
  t113 = t152 * (1.0 / t139);
  t139 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t140 = prop_Cl_a * prop_sigma * (prop_delta - 1.0) / 8.0;
  t146 = ((t6 + t7) - t953) + 1.5707963267948966;
  t147 = ((t6 + t8) - t953) + 1.5707963267948966;
  t148 = ((t6 + t9) - t953) + 1.5707963267948966;
  t149 = ((t6 + t10) - t953) + 1.5707963267948966;
  t120 = t60 * t60;
  t121 = t61 * t61;
  t124 = t953 - t6;
  t8 = t110 * 8.0;
  t134 = power_Cd_0 * prop_delta + power_Cd_a * (prop_theta * prop_theta);
  t151 = cos(t146);
  t152 = cos(t147);
  t7 = cos(t148);
  t9 = cos(t149);
  t155 = sin(t146);
  t156 = sin(t147);
  t157 = sin(t148);
  t158 = sin(t149);
  t126 = cos(t124);
  t127 = sin(t124);
  t165 = t155 * t155;
  t166 = t156 * t156;
  t167 = t157 * t157;
  t168 = t158 * t158;
  t150 = t643 + t642 * (Cl_alpha * Cl_alpha) * (t124 * t124);
  t953 = Cl_alpha * S * rho * t25 * t124;
  t350_tmp_tmp = S * rho;
  t5 = t350_tmp_tmp * cos(Beta) * t25;
  t350 = t953 * t126 / 2.0 + t5 * t127 * t150 / 2.0;
  t351 = t953 * t127 / 2.0 - t5 * t126 * t150 / 2.0;
  t408_tmp = t25 * t79 * t96 * t104;
  t953 = V * prop_Cl_a * prop_sigma;
  t5 = prop_Cl_0 * prop_sigma * t4 * t25;
  t641 = prop_sigma * (prop_delta - 1.0) * t105;
  t408 = (t140 + V * t78 * t95 * t103 * t151 / 2.0) +
         sqrt(((t408_tmp * (t151 * t151) * 16.0 +
                t953 * t78 * t95 * (prop_delta - 1.0) * t103 * t151 * 8.0) +
               t5 * t79 * t96 * t104 * t165 * -8.0) -
              t641 * (t8 + prop_Cl_a *
                               (t139 +
                                prop_theta * (t101 + t408_tmp * t165 * 8.0)))) /
             8.0;
  t409_tmp = t25 * t82 * t96 * t104;
  t409 = (t140 + V * t80 * t95 * t103 * t152 / 2.0) +
         sqrt(((t409_tmp * (t152 * t152) * 16.0 +
                t953 * t80 * t95 * (prop_delta - 1.0) * t103 * t152 * 8.0) +
               t5 * t82 * t96 * t104 * t166 * -8.0) -
              t641 * (t8 + prop_Cl_a *
                               (t139 +
                                prop_theta * (t101 + t409_tmp * t166 * 8.0)))) /
             8.0;
  t410_tmp = t25 * t85 * t96 * t104;
  t410 = (t140 + V * t83 * t95 * t103 * t7 / 2.0) +
         sqrt(((t410_tmp * (t7 * t7) * 16.0 +
                t953 * t83 * t95 * (prop_delta - 1.0) * t103 * t7 * 8.0) +
               t5 * t85 * t96 * t104 * t167 * -8.0) -
              t641 * (t8 + prop_Cl_a *
                               (t139 +
                                prop_theta * (t101 + t410_tmp * t167 * 8.0)))) /
             8.0;
  t411_tmp = t25 * t88 * t96 * t104;
  t411 = (t140 + V * t86 * t95 * t103 * t9 / 2.0) +
         sqrt(((t411_tmp * (t9 * t9) * 16.0 +
                t953 * t86 * t95 * (prop_delta - 1.0) * t103 * t9 * 8.0) +
               t5 * t88 * t96 * t104 * t168 * -8.0) -
              t641 * (t8 + prop_Cl_a *
                               (t139 +
                                prop_theta * (t101 + t411_tmp * t168 * 8.0)))) /
             8.0;
  t416 = -prop_theta + t408;
  t417 = -prop_theta + t409;
  t418 = -prop_theta + t410;
  t419 = -prop_theta + t411;
  d = rt_powd_snf(t826, 3.0);
  d1 = rt_powd_snf(t826, 5.0);
  Cl_alpha = cos(t11);
  t140 = cos(t12);
  t80 = cos(t13);
  d2 = cos(t14);
  d3 = sin(t11);
  d4 = sin(t12);
  t13 = sin(t13);
  t14 = sin(t14);
  t9 = x[0];
  t152 = x[1];
  t139 = x[2];
  t12 = x[3];
  ct_idx_52 = desired_motor_value * t95;
  t95 = power_Cd_0 * ((prop_delta + 1.0) + prop_delta * prop_delta) * 2.0;
  desired_motor_value = prop_delta / 6.0 - 0.16666666666666666;
  ct_idx_114 = W_act_motor * W_act_motor;
  W_act_tilt_el *= W_act_tilt_el;
  W_act_motor = W_act_tilt_az * W_act_tilt_az;
  power_Cd_0 = 1.0 / m;
  t151 = t110_tmp * t4;
  t5 = prop_Cl_a - prop_Cd_a * 2.0;
  t953 = prop_Cd_a * prop_theta * 2.0;
  t127 = t151 * t408 +
         (prop_delta - 1.0) * (t47 + prop_theta * (t408 * t5 + t953));
  t641 = t151 * t409 +
         (prop_delta - 1.0) * (t47 + prop_theta * (t409 * t5 + t953));
  t642 = t151 * t410 +
         (prop_delta - 1.0) * (t47 + prop_theta * (t410 * t5 + t953));
  t643 = t151 * t411 +
         (prop_delta - 1.0) * (t47 + prop_theta * (t411 * t5 + t953));
  t151 *= t25;
  t953 = prop_Cl_a * prop_theta * t25;
  t5 = prop_Cl_a * prop_delta;
  t149 = t151 * t79 * t96 * t104 * t165 +
         (prop_delta - 1.0) *
             ((t110 + t953 * t79 * t96 * t104 * t165) - t5 * t416 * 2.0);
  t6 = t151 * t82 * t96 * t104 * t166 +
       (prop_delta - 1.0) *
           ((t110 + t953 * t82 * t96 * t104 * t166) - t5 * t417 * 2.0);
  t126 = t151 * t85 * t96 * t104 * t167 +
         (prop_delta - 1.0) *
             ((t110 + t953 * t85 * t96 * t104 * t167) - t5 * t418 * 2.0);
  t147 = t151 * t88 * t96 * t104 * t168 +
         (prop_delta - 1.0) *
             ((t110 + t953 * t88 * t96 * t104 * t168) - t5 * t419 * 2.0);
  t148 = prop_sigma * rho;
  t953 = V * gain_motor * prop_sigma * rho * d;
  t5 = t148 * t17 * t39 * t43;
  t826 = t5 * t67 * t105 * t149 * 3.1415926535897931 / 4.0 +
         t9 * (t953 * t58 * t105 * t155 * t127 * 3.1415926535897931 / 4.0);
  t7 = t148 * t19 * t39 * t43;
  t11 = t7 * t68 * t105 * t6 * 3.1415926535897931 / 4.0 +
        t152 * (t953 * t59 * t105 * t156 * t641 * 3.1415926535897931 / 4.0);
  t8 = t148 * t21 * t39 * t43;
  t103 = t8 * t69 * t105 * t126 * 3.1415926535897931 / 4.0 +
         t139 * (t953 * t60 * t105 * t157 * t642 * 3.1415926535897931 / 4.0);
  t10 = t148 * t23 * t39 * t43;
  t86 = t10 * t70 * t105 * t147 * 3.1415926535897931 / 4.0 +
        t12 * (t953 * t61 * t105 * t158 * t643 * 3.1415926535897931 / 4.0);
  t146 = t5 * t58;
  t78 = t9 * V * gain_motor * prop_sigma * rho * d;
  t101 = t146 * Cl_alpha * t105 * t149 * 3.1415926535897931 / 4.0 +
         t78 * Cl_alpha * t67 * t105 * t155 * t127 * 3.1415926535897931 * -0.25;
  t9 = t7 * t59;
  t7 = t152 * V * gain_motor * prop_sigma * rho * d;
  t83 = t9 * t140 * t105 * t6 * 3.1415926535897931 / 4.0 +
        t7 * t140 * t68 * t105 * t156 * t641 * 3.1415926535897931 * -0.25;
  t152 = t8 * t60;
  t5 = t139 * V * gain_motor * prop_sigma * rho * d;
  t80 = t152 * t80 * t105 * t126 * 3.1415926535897931 / 4.0 +
        t5 * t80 * t69 * t105 * t157 * t642 * 3.1415926535897931 * -0.25;
  t151 = t10 * t61;
  t953 = t12 * V * gain_motor * prop_sigma * rho * d;
  t140 = t151 * d2 * t105 * t147 * 3.1415926535897931 / 4.0 +
         t953 * d2 * t70 * t105 * t158 * t643 * 3.1415926535897931 * -0.25;
  t139 = t146 * d3 * t105 * t149 * 3.1415926535897931 / 4.0 +
         t78 * t67 * d3 * t105 * t155 * t127 * 3.1415926535897931 * -0.25;
  t641 = t9 * d4 * t105 * t6 * 3.1415926535897931 / 4.0 +
         t7 * t68 * d4 * t105 * t156 * t641 * 3.1415926535897931 * -0.25;
  t5 = t152 * t13 * t105 * t126 * 3.1415926535897931 / 4.0 +
       t5 * t69 * t13 * t105 * t157 * t642 * 3.1415926535897931 * -0.25;
  t78 = t151 * t14 * t105 * t147 * 3.1415926535897931 / 4.0 +
        t953 * t70 * t14 * t105 * t158 * t643 * 3.1415926535897931 * -0.25;
  t146 = prop_Cl_0 * (prop_delta + 1.0);
  Cl_alpha =
      ct_idx_52 +
      t148 * rt_powd_snf(x[0], 3.0) * t39 * d1 * 1.5707963267948966 *
          desired_motor_value *
          (((t95 + t408_tmp * t105 * t134 * t165 * 3.0) + t146 * t408 * 3.0) -
           t416 * (prop_Cl_a * t408 * 6.0 - power_Cd_a * t416 * 6.0));
  t642 =
      ct_idx_52 +
      t148 * rt_powd_snf(x[1], 3.0) * t39 * d1 * 1.5707963267948966 *
          desired_motor_value *
          (((t95 + t409_tmp * t105 * t134 * t166 * 3.0) + t146 * t409 * 3.0) -
           t417 * (prop_Cl_a * t409 * 6.0 - power_Cd_a * t417 * 6.0));
  t643 =
      ct_idx_52 +
      t148 * rt_powd_snf(x[2], 3.0) * t39 * d1 * 1.5707963267948966 *
          desired_motor_value *
          (((t95 + t410_tmp * t105 * t134 * t167 * 3.0) + t146 * t410 * 3.0) -
           t418 * (prop_Cl_a * t410 * 6.0 - power_Cd_a * t418 * 6.0));
  t127 =
      ct_idx_52 +
      t148 * rt_powd_snf(x[3], 3.0) * t39 * d1 * 1.5707963267948966 *
          desired_motor_value *
          (((t95 + t411_tmp * t105 * t134 * t168 * 3.0) + t146 * t411 * 3.0) -
           t419 * (prop_Cl_a * t411 * 6.0 - power_Cd_a * t419 * 6.0));
  t149 = ((t826 + t11) + t103) + t86;
  t953 = ((t139 + t641) + t5) + t78;
  t6 = ((t101 + t83) + t80) + t140;
  t146 = I_xx * p;
  t126 = dv_global_6 +
         -(1.0 / I_zz) *
             (((((((((t146 * q - I_yy * p * q) + l_1 * t826) - l_1 * t11) -
                   l_2 * t103) +
                  l_2 * t86) -
                 l_4 * t139) -
                l_4 * t641) +
               l_3 * t5) +
              l_3 * t78);
  t147 = t350_tmp_tmp * t25;
  t148 = dv_global_5 +
         1.0 / I_yy *
             ((((((((((-(I_zz * p * r) + t146 * r) -
                      t147 * (Cm_zero - Cm_alpha * t124) * wing_chord / 2.0) +
                     l_z * t826) +
                    l_z * t11) +
                   l_z * t103) +
                  l_z * t86) +
                 l_4 * t101) +
                l_4 * t83) -
               l_3 * t80) -
              l_3 * t140);
  t146 = t350_tmp_tmp * sin(Beta) * t25;
  t10 = dv_global_2 +
        power_Cd_0 *
            (((t146 * t49 * t150 / 2.0 + t53 * t350) + t53 * t6) + t49 * t953);
  t9 = x[1] * t69;
  t8 = x[0] * t70;
  t7 = dv_global_4 +
       1.0 / I_xx *
           (((((((((((-(I_yy * q * r) + I_zz * q * r) -
                     CL_aileron * S * x[14] * gain_ailerons * rho * t25 / 2.0) +
                    t147 *
                        (((((((((((x[0] * t58 * 0.01228634392023026 -
                                   x[1] * t59 * 0.01228634392023026) +
                                  t17 * t58 * 0.0075456152077779167) +
                                 x[1] * t26 * t59 * 0.020516396677824081) -
                                t19 * t59 * 0.0075456152077779167) -
                               x[0] * t26 * t58 * 0.020516396677824081) +
                              t9 * t120 * 0.0064381447596962606) +
                             t9 * (t59 * t59) * 0.0039349871274520724) +
                            x[0] * t67 * t121 * 0.0046429750925043979) -
                           t8 * t121 * 0.0064381447596962606) -
                          t8 * (t58 * t58) * 0.0039349871274520724) -
                         x[1] * t68 * t120 * 0.0046429750925043979) *
                        wing_span / 2.0) +
                   l_z * t139) +
                  l_z * t641) +
                 l_z * t5) +
                l_z * t78) +
               l_1 * t101) +
              l_2 * t140) -
             l_1 * t83) -
            l_2 * t80);
  t147 = t49 * t50;
  t139 = (dv_global_3 +
          power_Cd_0 *
              (((((t146 * t50 * t53 * t150 / 2.0 + t62 * t351) - t147 * t350) +
                 t62 * t149) +
                t50 * t53 * t953) -
               t147 * t6)) -
         9.81;
  t147 = t49 * t62;
  t152 =
      dv_global_1 -
      power_Cd_0 *
          (((((-(t146 * t53 * t62 * t150 / 2.0) + t50 * t351) + t147 * t350) +
             t50 * t149) +
            t147 * t6) -
           t53 * t62 * t953);
  t151 = x[13] - desired_phi_value * (1.0 / gain_phi);
  t641 = x[12] - desired_theta_value * (1.0 / gain_theta);
  t149 = x[14] - desired_ailerons_value * (1.0 / gain_ailerons);
  t953 = x[4] - t112;
  t6 = x[5] - t112;
  t5 = x[6] - t112;
  t9 = x[7] - t112;
  t78 = x[8] - t113;
  t146 = x[9] - t113;
  t147 = x[10] - t113;
  t8 = x[11] - t113;
  fval =
      (((((gamma_quadratic_du *
               ((((((((((((((W_act_phi * W_act_phi * (t151 * t151) +
                             W_act_theta * W_act_theta * (t641 * t641)) +
                            W_act_ailerons * W_act_ailerons * (t149 * t149)) +
                           W_act_tilt_el * (t953 * t953)) +
                          W_act_tilt_el * (t6 * t6)) +
                         W_act_tilt_el * (t5 * t5)) +
                        W_act_tilt_el * (t9 * t9)) +
                       W_act_motor * (t78 * t78)) +
                      W_act_motor * (t146 * t146)) +
                     W_act_motor * (t147 * t147)) +
                    W_act_motor * (t8 * t8)) +
                   ct_idx_114 * (Cl_alpha * Cl_alpha)) +
                  ct_idx_114 * (t642 * t642)) +
                 ct_idx_114 * (t643 * t643)) +
                ct_idx_114 * (t127 * t127)) +
           W_dv_6 * W_dv_6 * (t126 * t126)) +
          W_dv_2 * W_dv_2 * (t10 * t10)) +
         W_dv_5 * W_dv_5 * (t148 * t148)) +
        W_dv_4 * W_dv_4 * (t7 * t7)) +
       W_dv_1 * W_dv_1 * (t152 * t152)) +
      W_dv_3 * W_dv_3 * (t139 * t139);
  *status = 1;
  b = rtIsNaN(fval);
  if (rtIsInf(fval) || b) {
    if (b) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }
  if (*status == 1) {
    *status = 1;
  }
  return fval;
}

/*
 * File trailer for evalObjAndConstr.c
 *
 * [EOF]
 */
