/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeObjectiveAndUserGradient_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

/* Include Files */
#include "computeObjectiveAndUserGradient_.h"
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "Cascaded_nonlinear_controller_w_ail_new_aero_rtwutil.h"
#include "compute_cost_and_gradient_w_Mx_noah_inner.h"
#include "compute_cost_and_gradient_w_Mx_noah_outer.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const d_struct_T *c_obj_next_next_next_next_next_
 *                const double x[15]
 *                double grad_workspace[16]
 *                int *status
 * Return Type  : double
 */
double c_computeObjectiveAndUserGradie(
    const d_struct_T *c_obj_next_next_next_next_next_, const double x[15],
    double grad_workspace[16], int *status)
{
  double b_CL_aileron[263];
  double gradient_data[15];
  double CL_aileron;
  double Cl_alpha;
  double Cm_alpha;
  double Cm_zero;
  double I_xx;
  double I_yy;
  double I_zz;
  double K_Cd;
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
  double gain_az;
  double gain_el;
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
  double prop_R;
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
  double t118;
  double t119;
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
  double t149;
  double t150;
  double t151;
  double t152;
  double t153;
  double t154;
  double t155;
  double t156;
  double t157;
  double t158;
  double t159;
  double t159_tmp;
  double t16;
  double t164;
  double t165;
  double t166;
  double t167;
  double t168;
  double t17;
  double t18;
  double t188;
  double t188_tmp_tmp;
  double t19;
  double t199;
  double t2;
  double t20;
  double t21;
  double t22;
  double t220;
  double t220_tmp;
  double t23;
  double t24;
  double t242;
  double t242_tmp;
  double t242_tmp_tmp;
  double t25;
  double t26;
  double t3;
  double t350;
  double t351;
  double t388;
  double t388_tmp;
  double t389_tmp;
  double t39;
  double t390_tmp;
  double t391_tmp;
  double t392;
  double t393;
  double t394;
  double t395;
  double t4;
  double t40;
  double t408;
  double t409;
  double t410;
  double t411;
  double t416;
  double t417;
  double t418;
  double t419;
  double t43;
  double t47;
  double t49;
  double t5;
  double t50;
  double t528;
  double t529;
  double t53;
  double t530;
  double t531;
  double t562;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t62;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t78;
  double t79;
  double t8;
  double t80;
  double t81;
  double t82;
  double t83;
  double t84;
  double t85;
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t95;
  double t96;
  double wing_chord;
  double wing_span;
  int idx_current;
  bool allFinite;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  t5 = c_obj_next_next_next_next_next_->Beta->contents;
  CL_aileron = c_obj_next_next_next_next_next_->CL_aileron->contents;
  t531 = c_obj_next_next_next_next_next_->Cd_zero->contents;
  Cl_alpha = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  Cm_zero = c_obj_next_next_next_next_next_->Cm_zero->contents;
  Cm_alpha = c_obj_next_next_next_next_next_->Cm_alpha->contents;
  I_xx = c_obj_next_next_next_next_next_->I_xx->contents;
  I_yy = c_obj_next_next_next_next_next_->I_yy->contents;
  I_zz = c_obj_next_next_next_next_next_->I_zz->contents;
  K_Cd = c_obj_next_next_next_next_next_->K_Cd->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  V = c_obj_next_next_next_next_next_->V->contents;
  t146 = c_obj_next_next_next_next_next_->V_scaled->contents;
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
  t149 = c_obj_next_next_next_next_next_->desired_el_value->contents;
  t388 = c_obj_next_next_next_next_next_->desired_az_value->contents;
  desired_phi_value =
      c_obj_next_next_next_next_next_->desired_phi_value->contents;
  desired_theta_value =
      c_obj_next_next_next_next_next_->desired_theta_value->contents;
  desired_motor_value =
      c_obj_next_next_next_next_next_->desired_motor_value->contents;
  desired_ailerons_value =
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents;
  t164 = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  gain_el = c_obj_next_next_next_next_next_->gain_el->contents;
  gain_az = c_obj_next_next_next_next_next_->gain_az->contents;
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
  prop_R = c_obj_next_next_next_next_next_->prop_R->contents;
  t147 = c_obj_next_next_next_next_next_->prop_Cd_0->contents;
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
  t2 = cos(t5);
  t3 = sin(t5);
  t4 = log(prop_delta);
  t5 = x[13] * gain_phi;
  t6 = x[12] * gain_theta;
  t7 = x[4] * gain_el;
  t8 = x[5] * gain_el;
  t9 = x[6] * gain_el;
  t10 = x[7] * gain_el;
  t11 = x[8] * gain_az;
  t12 = x[9] * gain_az;
  t13 = x[10] * gain_az;
  t14 = x[11] * gain_az;
  t16 = Cl_alpha * Cl_alpha;
  t17 = x[0] * x[0];
  t18 = rt_powd_snf(x[0], 3.0);
  t19 = x[1] * x[1];
  t20 = rt_powd_snf(x[1], 3.0);
  t21 = x[2] * x[2];
  t22 = rt_powd_snf(x[2], 3.0);
  t23 = x[3] * x[3];
  t24 = rt_powd_snf(x[3], 3.0);
  t25 = V * V;
  t26 = t146 * t146;
  t39 = gain_motor * gain_motor;
  t40 = prop_Cd_a * 2.0;
  t43 = rt_powd_snf(prop_R, 4.0);
  t47 = t147 * prop_delta * 2.0;
  t78 = 1.0 / x[0];
  t80 = 1.0 / x[1];
  t83 = 1.0 / x[2];
  t86 = 1.0 / x[3];
  t95 = 1.0 / gain_motor;
  t101 = prop_delta * 16.0;
  t103 = 1.0 / prop_R;
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
  t81 = 1.0 / t18;
  t82 = 1.0 / t19;
  t84 = 1.0 / t20;
  t85 = 1.0 / t21;
  t87 = 1.0 / t22;
  t88 = 1.0 / t23;
  t89 = 1.0 / t24;
  t96 = 1.0 / t39;
  t104 = t103 * t103;
  t110_tmp = prop_Cl_0 * prop_delta;
  t110 = t110_tmp * (prop_delta + 1.0);
  t112 = t149 * (1.0 / gain_el);
  t113 = t388 * (1.0 / gain_az);
  t139 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t562 = prop_Cl_a * prop_sigma;
  t140 = t562 * (prop_delta - 1.0) / 8.0;
  t146 = ((t6 + t7) - t164) + 1.5707963267948966;
  t147 = ((t6 + t8) - t164) + 1.5707963267948966;
  t5 = ((t6 + t9) - t164) + 1.5707963267948966;
  t149 = ((t6 + t10) - t164) + 1.5707963267948966;
  t118 = t58 * t58;
  t119 = t59 * t59;
  t120 = t60 * t60;
  t121 = t61 * t61;
  t124 = t164 - t6;
  t9 = t110 * 8.0;
  t134 = power_Cd_0 * prop_delta + power_Cd_a * (prop_theta * prop_theta);
  t151 = cos(t146);
  t152 = cos(t147);
  t153 = cos(t5);
  t154 = cos(t149);
  t155 = sin(t146);
  t156 = sin(t147);
  t157 = sin(t5);
  t158 = sin(t149);
  t126 = cos(t124);
  t127 = sin(t124);
  t146 = t151 * t151;
  t147 = t152 * t152;
  t149 = t153 * t153;
  t164 = t154 * t154;
  t165 = t155 * t155;
  t166 = t156 * t156;
  t167 = t157 * t157;
  t168 = t158 * t158;
  t388 = Cl_alpha * S;
  t159_tmp = t388 * gain_theta * rho * t25;
  t159 = t159_tmp * t126 / 2.0;
  t220_tmp = prop_Cl_a * prop_theta * t25;
  t220 = t220_tmp * t81 * t96 * t104 * t165 * 2.0;
  t242_tmp_tmp = t110_tmp * t4;
  t242_tmp = t242_tmp_tmp * t25;
  t242 = t242_tmp * t81 * t96 * t104 * t165 * 2.0;
  t150 = t531 + K_Cd * t16 * (t124 * t124);
  t188_tmp_tmp = S * rho;
  t5 = t188_tmp_tmp * t2 * t25;
  t188 = t5 * t127 * t150 / 2.0;
  t199 = t5 * t126 * t150 / 2.0;
  t5 = t388 * rho * t25 * t124;
  t350 = t5 * t126 / 2.0 + t188;
  t351 = t5 * t127 / 2.0 - t199;
  t388_tmp = t25 * t79 * t96 * t104;
  t10 = V * prop_Cl_a * prop_sigma;
  t6 = prop_Cl_0 * prop_sigma * t4 * t25;
  t5 = prop_sigma * (prop_delta - 1.0) * t105;
  t388 =
      sqrt(((t388_tmp * t146 * 16.0 +
             t10 * t78 * t95 * (prop_delta - 1.0) * t103 * t151 * 8.0) +
            t6 * t79 * t96 * t104 * t165 * -8.0) -
           t5 * (t9 + prop_Cl_a * (t139 + prop_theta *
                                              (t101 + t388_tmp * t165 * 8.0))));
  t389_tmp = t25 * t82 * t96 * t104;
  t7 =
      sqrt(((t389_tmp * t147 * 16.0 +
             t10 * t80 * t95 * (prop_delta - 1.0) * t103 * t152 * 8.0) +
            t6 * t82 * t96 * t104 * t166 * -8.0) -
           t5 * (t9 + prop_Cl_a * (t139 + prop_theta *
                                              (t101 + t389_tmp * t166 * 8.0))));
  t390_tmp = t25 * t85 * t96 * t104;
  t8 =
      sqrt(((t390_tmp * t149 * 16.0 +
             t10 * t83 * t95 * (prop_delta - 1.0) * t103 * t153 * 8.0) +
            t6 * t85 * t96 * t104 * t167 * -8.0) -
           t5 * (t9 + prop_Cl_a * (t139 + prop_theta *
                                              (t101 + t390_tmp * t167 * 8.0))));
  t391_tmp = t25 * t88 * t96 * t104;
  t5 =
      sqrt(((t391_tmp * t164 * 16.0 +
             t10 * t86 * t95 * (prop_delta - 1.0) * t103 * t154 * 8.0) +
            t6 * t88 * t96 * t104 * t168 * -8.0) -
           t5 * (t9 + prop_Cl_a * (t139 + prop_theta *
                                              (t101 + t391_tmp * t168 * 8.0))));
  t392 = 1.0 / t388;
  t393 = 1.0 / t7;
  t394 = 1.0 / t8;
  t395 = 1.0 / t5;
  t408 = (t140 + V * t78 * t95 * t103 * t151 / 2.0) + t388 / 8.0;
  t409 = (t140 + V * t80 * t95 * t103 * t152 / 2.0) + t7 / 8.0;
  t410 = (t140 + V * t83 * t95 * t103 * t153 / 2.0) + t8 / 8.0;
  t411 = (t140 + V * t86 * t95 * t103 * t154 / 2.0) + t5 / 8.0;
  t416 = -prop_theta + t408;
  t417 = -prop_theta + t409;
  t418 = -prop_theta + t410;
  t419 = -prop_theta + t411;
  t5 = t562 * prop_theta * t25;
  t528 = V * t79 * t95 * t103 * t151 / 2.0 +
         (((t25 * t81 * t96 * t104 * t146 * 32.0 +
            t10 * t79 * t95 * (prop_delta - 1.0) * t103 * t151 * 8.0) -
           t6 * t81 * t96 * t104 * t165 * 16.0) -
          t5 * t81 * t96 * (prop_delta - 1.0) * t104 * t105 * t165 * 16.0) *
             t392 / 16.0;
  t529 = V * t82 * t95 * t103 * t152 / 2.0 +
         (((t25 * t84 * t96 * t104 * t147 * 32.0 +
            t10 * t82 * t95 * (prop_delta - 1.0) * t103 * t152 * 8.0) -
           t6 * t84 * t96 * t104 * t166 * 16.0) -
          t5 * t84 * t96 * (prop_delta - 1.0) * t104 * t105 * t166 * 16.0) *
             t393 / 16.0;
  t530 = V * t85 * t95 * t103 * t153 / 2.0 +
         (((t25 * t87 * t96 * t104 * t149 * 32.0 +
            t10 * t85 * t95 * (prop_delta - 1.0) * t103 * t153 * 8.0) -
           t6 * t87 * t96 * t104 * t167 * 16.0) -
          t5 * t87 * t96 * (prop_delta - 1.0) * t104 * t105 * t167 * 16.0) *
             t394 / 16.0;
  t531 = V * t88 * t95 * t103 * t154 / 2.0 +
         (((t25 * t89 * t96 * t104 * t164 * 32.0 +
            t10 * t88 * t95 * (prop_delta - 1.0) * t103 * t154 * 8.0) -
           t6 * t89 * t96 * t104 * t168 * 16.0) -
          t5 * t89 * t96 * (prop_delta - 1.0) * t104 * t105 * t168 * 16.0) *
             t395 / 16.0;
  t5 = V * gain_el;
  t7 = t5 * prop_Cl_a * prop_sigma;
  t8 = gain_el * t25;
  t110_tmp = gain_el * prop_Cl_0;
  t9 = t110_tmp * prop_sigma * t4 * t25;
  t139 = gain_el * prop_Cl_a;
  t10 = t139 * prop_sigma * prop_theta * t25;
  t101 = t5 * t78 * t95 * t103 * t155 / 2.0 +
         t392 *
             (((t7 * t78 * t95 * (prop_delta - 1.0) * t103 * t155 * 8.0 +
                t8 * t79 * t96 * t104 * t151 * t155 * 32.0) +
               t9 * t79 * t96 * t104 * t151 * t155 * 16.0) +
              t10 * t79 * t96 * (prop_delta - 1.0) * t104 * t105 * t151 * t155 *
                  16.0) /
             16.0;
  t140 = t5 * t80 * t95 * t103 * t156 / 2.0 +
         t393 *
             (((t7 * t80 * t95 * (prop_delta - 1.0) * t103 * t156 * 8.0 +
                t8 * t82 * t96 * t104 * t152 * t156 * 32.0) +
               t9 * t82 * t96 * t104 * t152 * t156 * 16.0) +
              t10 * t82 * t96 * (prop_delta - 1.0) * t104 * t105 * t152 * t156 *
                  16.0) /
             16.0;
  t562 = t5 * t83 * t95 * t103 * t157 / 2.0 +
         t394 *
             (((t7 * t83 * t95 * (prop_delta - 1.0) * t103 * t157 * 8.0 +
                t8 * t85 * t96 * t104 * t153 * t157 * 32.0) +
               t9 * t85 * t96 * t104 * t153 * t157 * 16.0) +
              t10 * t85 * t96 * (prop_delta - 1.0) * t104 * t105 * t153 * t157 *
                  16.0) /
             16.0;
  t149 = t5 * t86 * t95 * t103 * t158 / 2.0 +
         t395 *
             (((t7 * t86 * t95 * (prop_delta - 1.0) * t103 * t158 * 8.0 +
                t8 * t88 * t96 * t104 * t154 * t158 * 32.0) +
               t9 * t88 * t96 * t104 * t154 * t158 * 16.0) +
              t10 * t88 * t96 * (prop_delta - 1.0) * t104 * t105 * t154 * t158 *
                  16.0) /
             16.0;
  t5 = V * gain_theta;
  t7 = t5 * prop_Cl_a * prop_sigma;
  t8 = gain_theta * t25;
  t147 = gain_theta * prop_Cl_0;
  t9 = t147 * prop_sigma * t4 * t25;
  Cl_alpha = gain_theta * prop_Cl_a;
  t10 = Cl_alpha * prop_sigma * prop_theta * t25;
  t146 = t5 * t78 * t95 * t103 * t155 / 2.0 +
         t392 *
             (((t7 * t78 * t95 * (prop_delta - 1.0) * t103 * t155 * 8.0 +
                t8 * t79 * t96 * t104 * t151 * t155 * 32.0) +
               t9 * t79 * t96 * t104 * t151 * t155 * 16.0) +
              t10 * t79 * t96 * (prop_delta - 1.0) * t104 * t105 * t151 * t155 *
                  16.0) /
             16.0;
  t164 = t5 * t80 * t95 * t103 * t156 / 2.0 +
         t393 *
             (((t7 * t80 * t95 * (prop_delta - 1.0) * t103 * t156 * 8.0 +
                t8 * t82 * t96 * t104 * t152 * t156 * 32.0) +
               t9 * t82 * t96 * t104 * t152 * t156 * 16.0) +
              t10 * t82 * t96 * (prop_delta - 1.0) * t104 * t105 * t152 * t156 *
                  16.0) /
             16.0;
  t6 = t5 * t83 * t95 * t103 * t157 / 2.0 +
       t394 *
           (((t7 * t83 * t95 * (prop_delta - 1.0) * t103 * t157 * 8.0 +
              t8 * t85 * t96 * t104 * t153 * t157 * 32.0) +
             t9 * t85 * t96 * t104 * t153 * t157 * 16.0) +
            t10 * t85 * t96 * (prop_delta - 1.0) * t104 * t105 * t153 * t157 *
                16.0) /
           16.0;
  t8 = t5 * t86 * t95 * t103 * t158 / 2.0 +
       t395 *
           (((t7 * t86 * t95 * (prop_delta - 1.0) * t103 * t158 * 8.0 +
              t8 * t88 * t96 * t104 * t154 * t158 * 32.0) +
             t9 * t88 * t96 * t104 * t154 * t158 * 16.0) +
            t10 * t88 * t96 * (prop_delta - 1.0) * t104 * t105 * t154 * t158 *
                16.0) /
           16.0;
  t9 = prop_Cl_a * prop_delta;
  t10 = t9 * t528 * 2.0;
  b_CL_aileron[0] = CL_aileron;
  b_CL_aileron[1] = Cm_alpha;
  b_CL_aileron[2] = K_Cd;
  b_CL_aileron[3] = x[0];
  b_CL_aileron[4] = x[1];
  b_CL_aileron[5] = x[2];
  b_CL_aileron[6] = x[3];
  b_CL_aileron[7] = x[13];
  b_CL_aileron[8] = S;
  b_CL_aileron[9] = x[12];
  b_CL_aileron[10] = V;
  b_CL_aileron[11] = x[4];
  b_CL_aileron[12] = x[5];
  b_CL_aileron[13] = x[6];
  b_CL_aileron[14] = x[7];
  b_CL_aileron[15] = x[14];
  b_CL_aileron[16] = desired_ailerons_value;
  b_CL_aileron[17] = desired_phi_value;
  b_CL_aileron[18] = desired_theta_value;
  b_CL_aileron[19] = dv_global_1;
  b_CL_aileron[20] = dv_global_2;
  b_CL_aileron[21] = dv_global_3;
  b_CL_aileron[22] = dv_global_4;
  b_CL_aileron[23] = dv_global_5;
  b_CL_aileron[24] = dv_global_6;
  b_CL_aileron[25] = x[8];
  b_CL_aileron[26] = x[9];
  b_CL_aileron[27] = x[10];
  b_CL_aileron[28] = x[11];
  b_CL_aileron[29] = gain_ailerons;
  b_CL_aileron[30] = gain_az;
  b_CL_aileron[31] = gain_el;
  b_CL_aileron[32] = gain_motor;
  b_CL_aileron[33] = gain_phi;
  b_CL_aileron[34] = gain_theta;
  b_CL_aileron[35] = gamma_quadratic_du;
  b_CL_aileron[36] = l_1;
  b_CL_aileron[37] = l_2;
  b_CL_aileron[38] = l_3;
  b_CL_aileron[39] = l_4;
  b_CL_aileron[40] = l_z;
  b_CL_aileron[41] = power_Cd_a;
  b_CL_aileron[42] = prop_Cl_0;
  b_CL_aileron[43] = prop_Cl_a;
  b_CL_aileron[44] = prop_sigma;
  b_CL_aileron[45] = prop_theta;
  b_CL_aileron[46] = rho;
  b_CL_aileron[47] = prop_delta - 1.0;
  b_CL_aileron[48] = t104;
  b_CL_aileron[49] = t105;
  b_CL_aileron[50] = 1.5707963267948966;
  b_CL_aileron[51] = t110;
  b_CL_aileron[52] = desired_motor_value * t95;
  b_CL_aileron[53] = -(I_zz * p * r);
  b_CL_aileron[54] = -(I_yy * q * r);
  b_CL_aileron[55] = t118;
  b_CL_aileron[56] = t119;
  b_CL_aileron[57] = t120;
  b_CL_aileron[58] = t121;
  b_CL_aileron[59] = t124;
  b_CL_aileron[60] = -t112;
  b_CL_aileron[61] = -(t112 * 2.0);
  b_CL_aileron[62] = -t113;
  b_CL_aileron[63] = -(t113 * 2.0);
  b_CL_aileron[64] = t134;
  b_CL_aileron[65] =
      power_Cd_0 * ((prop_delta + 1.0) + prop_delta * prop_delta) * 2.0;
  b_CL_aileron[66] = prop_delta / 6.0 - 0.16666666666666666;
  b_CL_aileron[67] =
      -(CL_aileron * S * x[14] * gain_ailerons * rho * t25 / 2.0);
  b_CL_aileron[68] = t150;
  b_CL_aileron[69] = t151;
  b_CL_aileron[70] = t152;
  b_CL_aileron[71] = t153;
  b_CL_aileron[72] = t154;
  b_CL_aileron[73] = t155;
  b_CL_aileron[74] = t156;
  b_CL_aileron[75] = t157;
  b_CL_aileron[76] = t158;
  b_CL_aileron[77] = t16;
  b_CL_aileron[78] = t165;
  b_CL_aileron[79] = t166;
  b_CL_aileron[80] = t167;
  b_CL_aileron[81] = t168;
  b_CL_aileron[82] = t17;
  t7 = t188_tmp_tmp * t25;
  b_CL_aileron[83] = -(t7 * (Cm_zero - Cm_alpha * t124) * wing_chord / 2.0);
  t388 = t188_tmp_tmp * t3 * t25;
  b_CL_aileron[84] = t388 * t49 * t150 / 2.0;
  b_CL_aileron[85] = t18;
  b_CL_aileron[86] = t388 * t50 * t53 * t150 / 2.0;
  b_CL_aileron[87] = t19;
  b_CL_aileron[88] = t20;
  b_CL_aileron[89] = -(t388 * t53 * t62 * t150 / 2.0);
  b_CL_aileron[90] = t220_tmp * t79 * t96 * t104 * t165;
  b_CL_aileron[91] = t220_tmp * t82 * t96 * t104 * t166;
  b_CL_aileron[92] = t220_tmp * t85 * t96 * t104 * t167;
  b_CL_aileron[93] = t220_tmp * t88 * t96 * t104 * t168;
  b_CL_aileron[94] = t21;
  b_CL_aileron[95] = t22;
  b_CL_aileron[96] = t220;
  b_CL_aileron[97] = t220_tmp * t84 * t96 * t104 * t166 * 2.0;
  b_CL_aileron[98] = t220_tmp * t87 * t96 * t104 * t167 * 2.0;
  b_CL_aileron[99] = t220_tmp * t89 * t96 * t104 * t168 * 2.0;
  b_CL_aileron[100] = t242_tmp * t79 * t96 * t104 * t165;
  b_CL_aileron[101] = t242_tmp * t82 * t96 * t104 * t166;
  b_CL_aileron[102] = t242_tmp * t85 * t96 * t104 * t167;
  b_CL_aileron[103] = t242_tmp * t88 * t96 * t104 * t168;
  b_CL_aileron[104] = t23;
  b_CL_aileron[105] = t24;
  b_CL_aileron[106] = t242;
  b_CL_aileron[107] = t242_tmp * t84 * t96 * t104 * t166 * 2.0;
  b_CL_aileron[108] = t242_tmp * t87 * t96 * t104 * t167 * 2.0;
  b_CL_aileron[109] = t242_tmp * t89 * t96 * t104 * t168 * 2.0;
  b_CL_aileron[110] = t25;
  b_CL_aileron[111] = t26;
  b_CL_aileron[112] = W_act_phi * W_act_phi;
  b_CL_aileron[113] = W_act_theta * W_act_theta;
  b_CL_aileron[114] = W_act_motor * W_act_motor;
  b_CL_aileron[115] = t3;
  b_CL_aileron[116] = W_dv_1 * W_dv_1;
  b_CL_aileron[117] = t388_tmp * t105 * t134 * t165 * 3.0;
  b_CL_aileron[118] = t389_tmp * t105 * t134 * t166 * 3.0;
  b_CL_aileron[119] = t390_tmp * t105 * t134 * t167 * 3.0;
  b_CL_aileron[120] = t391_tmp * t105 * t134 * t168 * 3.0;
  b_CL_aileron[121] = W_dv_2 * W_dv_2;
  b_CL_aileron[122] = W_dv_3 * W_dv_3;
  b_CL_aileron[123] = W_dv_4 * W_dv_4;
  t388 = t110_tmp * prop_delta * t4 * t25;
  b_CL_aileron[124] = t388 * t79 * t96 * t104 * t151 * t155 * 2.0;
  b_CL_aileron[125] = t388 * t82 * t96 * t104 * t152 * t156 * 2.0;
  b_CL_aileron[126] = t388 * t85 * t96 * t104 * t153 * t157 * 2.0;
  b_CL_aileron[127] = t388 * t88 * t96 * t104 * t154 * t158 * 2.0;
  t388 = t147 * prop_delta * t4 * t25;
  b_CL_aileron[128] = t388 * t79 * t96 * t104 * t151 * t155 * 2.0;
  b_CL_aileron[129] = t388 * t82 * t96 * t104 * t152 * t156 * 2.0;
  b_CL_aileron[130] = t388 * t85 * t96 * t104 * t153 * t157 * 2.0;
  b_CL_aileron[131] = t388 * t88 * t96 * t104 * t154 * t158 * 2.0;
  b_CL_aileron[132] = W_dv_5 * W_dv_5;
  b_CL_aileron[133] = W_dv_6 * W_dv_6;
  b_CL_aileron[134] = t350;
  b_CL_aileron[135] = W_act_tilt_el * W_act_tilt_el;
  b_CL_aileron[136] = t53 * t350;
  b_CL_aileron[137] = t50 * t351;
  b_CL_aileron[138] = t62 * t351;
  b_CL_aileron[139] = t49 * t62 * t350;
  b_CL_aileron[140] = -(t49 * t50 * t350);
  b_CL_aileron[141] = W_act_tilt_az * W_act_tilt_az;
  t388 = K_Cd * S * gain_theta * rho * t2 * t16 * t25 * t124;
  b_CL_aileron[142] = ((t159 + t159_tmp * t124 * t127 * -0.5) + t388 * t127) +
                      gain_theta * t199;
  b_CL_aileron[143] =
      ((t159_tmp * t127 / 2.0 + t124 * t159) - t388 * t126) + gain_theta * t188;
  b_CL_aileron[144] = W_act_ailerons * W_act_ailerons;
  b_CL_aileron[145] = t39;
  b_CL_aileron[146] = t40;
  b_CL_aileron[147] = prop_delta + 1.0;
  b_CL_aileron[148] = prop_Cl_a * t408 * 6.0;
  b_CL_aileron[149] = prop_Cl_a * t409 * 6.0;
  b_CL_aileron[150] = prop_Cl_a * t410 * 6.0;
  b_CL_aileron[151] = prop_Cl_a * t411 * 6.0;
  b_CL_aileron[152] = t416;
  b_CL_aileron[153] = t417;
  b_CL_aileron[154] = t418;
  b_CL_aileron[155] = t419;
  b_CL_aileron[156] = rt_powd_snf(prop_R, 3.0);
  b_CL_aileron[157] = t242_tmp_tmp * t408;
  b_CL_aileron[158] = t242_tmp_tmp * t409;
  b_CL_aileron[159] = t242_tmp_tmp * t410;
  b_CL_aileron[160] = t242_tmp_tmp * t411;
  t388 = prop_Cl_0 * (prop_delta + 1.0);
  b_CL_aileron[161] = t388 * t408 * 3.0;
  b_CL_aileron[162] = t388 * t409 * 3.0;
  b_CL_aileron[163] = t388 * t410 * 3.0;
  b_CL_aileron[164] = t388 * t411 * 3.0;
  b_CL_aileron[165] = t43;
  b_CL_aileron[166] = -(power_Cd_a * t416 * 6.0);
  b_CL_aileron[167] = -(power_Cd_a * t417 * 6.0);
  b_CL_aileron[168] = -(power_Cd_a * t418 * 6.0);
  b_CL_aileron[169] = -(power_Cd_a * t419 * 6.0);
  b_CL_aileron[170] = rt_powd_snf(prop_R, 5.0);
  b_CL_aileron[171] = -(t9 * t416 * 2.0);
  b_CL_aileron[172] = -(t9 * t417 * 2.0);
  b_CL_aileron[173] = -(t9 * t418 * 2.0);
  b_CL_aileron[174] = -(t9 * t419 * 2.0);
  t388 = prop_Cl_a - t40;
  t5 = prop_Cd_a * prop_theta * 2.0;
  b_CL_aileron[175] = t47 + prop_theta * (t408 * t388 + t5);
  b_CL_aileron[176] = t47 + prop_theta * (t409 * t388 + t5);
  b_CL_aileron[177] = t47 + prop_theta * (t410 * t388 + t5);
  b_CL_aileron[178] = t47 + prop_theta * (t411 * t388 + t5);
  b_CL_aileron[179] = t49;
  b_CL_aileron[180] = t50;
  t5 = I_xx * p;
  b_CL_aileron[181] = t5 * q;
  b_CL_aileron[182] = I_yy * p * q;
  b_CL_aileron[183] = t528;
  b_CL_aileron[184] = t529;
  b_CL_aileron[185] = t53;
  b_CL_aileron[186] = t530;
  b_CL_aileron[187] = t531;
  b_CL_aileron[188] = t10;
  b_CL_aileron[189] = t9 * t529 * 2.0;
  b_CL_aileron[190] = t9 * t530 * 2.0;
  b_CL_aileron[191] = t9 * t531 * 2.0;
  b_CL_aileron[192] = t242_tmp_tmp * t528;
  b_CL_aileron[193] = t242_tmp_tmp * t529;
  b_CL_aileron[194] = t242_tmp_tmp * t530;
  b_CL_aileron[195] = t242_tmp_tmp * t531;
  b_CL_aileron[196] = t5 * r;
  t5 = prop_theta * (prop_delta - 1.0);
  b_CL_aileron[197] = t5 * t528 * t388;
  b_CL_aileron[198] = t5 * t529 * t388;
  b_CL_aileron[199] = t5 * t530 * t388;
  b_CL_aileron[200] = t5 * t531 * t388;
  b_CL_aileron[201] = t101;
  b_CL_aileron[202] = t140;
  b_CL_aileron[203] = t562;
  b_CL_aileron[204] = t149;
  b_CL_aileron[205] = t146;
  b_CL_aileron[206] = t164;
  b_CL_aileron[207] = t6;
  b_CL_aileron[208] = t8;
  b_CL_aileron[209] = I_zz * q * r;
  b_CL_aileron[210] = t58;
  b_CL_aileron[211] = t242_tmp_tmp * t101;
  b_CL_aileron[212] = t242_tmp_tmp * t140;
  b_CL_aileron[213] = t242_tmp_tmp * t562;
  b_CL_aileron[214] = t242_tmp_tmp * t149;
  b_CL_aileron[215] = t242_tmp_tmp * t146;
  b_CL_aileron[216] = t242_tmp_tmp * t164;
  b_CL_aileron[217] = t242_tmp_tmp * t6;
  b_CL_aileron[218] = t242_tmp_tmp * t8;
  b_CL_aileron[219] = t59;
  b_CL_aileron[220] = t60;
  b_CL_aileron[221] = t61;
  t388 = t139 * prop_theta * t25;
  b_CL_aileron[222] =
      t388 * t79 * t96 * t104 * t151 * t155 * 2.0 + t9 * t101 * 2.0;
  b_CL_aileron[223] =
      t388 * t82 * t96 * t104 * t152 * t156 * 2.0 + t9 * t140 * 2.0;
  b_CL_aileron[224] =
      t388 * t85 * t96 * t104 * t153 * t157 * 2.0 + t9 * t562 * 2.0;
  b_CL_aileron[225] =
      t388 * t88 * t96 * t104 * t154 * t158 * 2.0 + t9 * t149 * 2.0;
  t388 = Cl_alpha * prop_theta * t25;
  b_CL_aileron[226] =
      t388 * t79 * t96 * t104 * t151 * t155 * 2.0 + t9 * t146 * 2.0;
  b_CL_aileron[227] =
      t388 * t82 * t96 * t104 * t152 * t156 * 2.0 + t9 * t164 * 2.0;
  b_CL_aileron[228] =
      t388 * t85 * t96 * t104 * t153 * t157 * 2.0 + t9 * t6 * 2.0;
  b_CL_aileron[229] =
      t388 * t88 * t96 * t104 * t154 * t158 * 2.0 + t9 * t8 * 2.0;
  b_CL_aileron[230] = t62;
  b_CL_aileron[231] = prop_sigma * rho * t17 * t39 * t43 * t67 * t105 *
                      3.1415926535897931 *
                      (t242 + (prop_delta - 1.0) * (t220 - t10)) * -0.25;
  b_CL_aileron[232] = cos(t11);
  b_CL_aileron[233] = cos(t12);
  t388 = x[1] * t69;
  t5 = x[0] * t70;
  b_CL_aileron[234] = t7 *
                      (((((((((((x[0] * t58 * 0.01228634392023026 -
                                 x[1] * t59 * 0.01228634392023026) +
                                t17 * t58 * 0.0075456152077779167) +
                               x[1] * t26 * t59 * 0.020516396677824081) -
                              t19 * t59 * 0.0075456152077779167) -
                             x[0] * t26 * t58 * 0.020516396677824081) +
                            t388 * t120 * 0.0064381447596962606) +
                           t388 * t119 * 0.0039349871274520724) +
                          x[0] * t67 * t121 * 0.0046429750925043979) -
                         t5 * t121 * 0.0064381447596962606) -
                        t5 * t118 * 0.0039349871274520724) -
                       x[1] * t68 * t120 * 0.0046429750925043979) *
                      wing_span / 2.0;
  b_CL_aileron[235] = cos(t13);
  b_CL_aileron[236] = cos(t14);
  b_CL_aileron[237] = t67;
  b_CL_aileron[238] = t68;
  b_CL_aileron[239] = t69;
  b_CL_aileron[240] = t70;
  b_CL_aileron[241] = sin(t11);
  b_CL_aileron[242] = sin(t12);
  b_CL_aileron[243] = sin(t13);
  b_CL_aileron[244] = sin(t14);
  b_CL_aileron[245] = 1.0 / I_xx;
  b_CL_aileron[246] = 1.0 / I_yy;
  b_CL_aileron[247] = 1.0 / I_zz;
  b_CL_aileron[248] = t79;
  b_CL_aileron[249] = t81;
  b_CL_aileron[250] = t82;
  b_CL_aileron[251] = t84;
  b_CL_aileron[252] = t85;
  b_CL_aileron[253] = t87;
  b_CL_aileron[254] = t88;
  b_CL_aileron[255] = t89;
  b_CL_aileron[256] = 1.0 / gain_phi;
  b_CL_aileron[257] = 1.0 / gain_theta;
  b_CL_aileron[258] = t96;
  b_CL_aileron[259] = 1.0 / gain_ailerons;
  b_CL_aileron[260] = 1.0 / m;
  b_CL_aileron[261] = wing_chord;
  b_CL_aileron[262] = wing_span;
  fval = ft_1(b_CL_aileron, gradient_data, &idx_current);
  memcpy(&grad_workspace[0], &gradient_data[0], 15U * sizeof(double));
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
  return fval;
}

/*
 * Arguments    : const e_struct_T *c_obj_next_next_next_next_next_
 *                const double x[13]
 *                double grad_workspace[14]
 *                int *status
 * Return Type  : double
 */
double d_computeObjectiveAndUserGradie(
    const e_struct_T *c_obj_next_next_next_next_next_, const double x[13],
    double grad_workspace[14], int *status)
{
  double b_CL_aileron[226];
  double gradient_data[13];
  double CL_aileron;
  double Cm_alpha;
  double Cm_zero;
  double I_xx;
  double I_yy;
  double I_zz;
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
  double b_t504_tmp_tmp;
  double desired_ailerons_value;
  double desired_motor_value;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double fval;
  double gain_ailerons;
  double gain_az;
  double gain_el;
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
  double t104;
  double t106;
  double t107;
  double t11;
  double t112;
  double t113;
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
  double t155;
  double t156;
  double t157;
  double t158;
  double t16;
  double t162_tmp;
  double t175_tmp;
  double t175_tmp_tmp;
  double t19;
  double t197_tmp;
  double t20;
  double t21;
  double t217_tmp;
  double t217_tmp_tmp;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t3;
  double t307;
  double t340_tmp;
  double t341_tmp;
  double t342_tmp;
  double t343_tmp;
  double t347;
  double t356;
  double t357;
  double t358;
  double t359;
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
  double t477;
  double t478;
  double t479;
  double t48;
  double t5;
  double t504;
  double t504_tmp_tmp;
  double t506;
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
  double t679;
  double t68;
  double t680;
  double t681;
  double t69;
  double t7;
  double t70;
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
  double t83;
  double t84;
  double t85;
  double t89;
  double t9;
  double t90;
  double t95;
  double t97;
  double t98;
  double t99;
  double wing_chord;
  double wing_span;
  int idx_current;
  bool allFinite;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  t347 = c_obj_next_next_next_next_next_->Beta->contents;
  CL_aileron = c_obj_next_next_next_next_next_->CL_aileron->contents;
  t679 = c_obj_next_next_next_next_next_->Cd_zero->contents;
  t680 = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  Cm_zero = c_obj_next_next_next_next_next_->Cm_zero->contents;
  Cm_alpha = c_obj_next_next_next_next_next_->Cm_alpha->contents;
  I_xx = c_obj_next_next_next_next_next_->I_xx->contents;
  I_yy = c_obj_next_next_next_next_next_->I_yy->contents;
  I_zz = c_obj_next_next_next_next_next_->I_zz->contents;
  t681 = c_obj_next_next_next_next_next_->K_Cd->contents;
  t13 = c_obj_next_next_next_next_next_->Phi->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  t132 = c_obj_next_next_next_next_next_->Theta->contents;
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
  t151 = c_obj_next_next_next_next_next_->desired_el_value->contents;
  t152 = c_obj_next_next_next_next_next_->desired_az_value->contents;
  desired_motor_value =
      c_obj_next_next_next_next_next_->desired_motor_value->contents;
  desired_ailerons_value =
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents;
  t120 = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  gain_el = c_obj_next_next_next_next_next_->gain_el->contents;
  gain_az = c_obj_next_next_next_next_next_->gain_az->contents;
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
  t140 = c_obj_next_next_next_next_next_->prop_Cd_0->contents;
  prop_Cl_0 = c_obj_next_next_next_next_next_->prop_Cl_0->contents;
  t139 = c_obj_next_next_next_next_next_->prop_Cd_a->contents;
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
  t3 = cos(t13);
  t4 = sin(t347);
  t5 = cos(t132);
  t6 = sin(t13);
  t7 = sin(t132);
  t8 = log(prop_delta);
  t9 = x[4] * gain_el;
  t10 = x[5] * gain_el;
  t11 = x[6] * gain_el;
  t12 = x[7] * gain_el;
  t13 = x[8] * gain_az;
  t14 = x[9] * gain_az;
  t15 = x[10] * gain_az;
  t16 = x[11] * gain_az;
  t19 = x[0] * x[0];
  t20 = rt_powd_snf(x[0], 3.0);
  t21 = x[1] * x[1];
  t22 = rt_powd_snf(x[1], 3.0);
  t23 = x[2] * x[2];
  t24 = rt_powd_snf(x[2], 3.0);
  t25 = x[3] * x[3];
  t26 = rt_powd_snf(x[3], 3.0);
  t27 = V * V;
  t28 = t138 * t138;
  t39 = gain_motor * gain_motor;
  t40 = t139 * 2.0;
  t42 = rt_powd_snf(prop_R, 3.0);
  t43 = rt_powd_snf(prop_R, 4.0);
  t47 = t140 * prop_delta * 2.0;
  t74 = 1.0 / x[0];
  t76 = 1.0 / x[1];
  t79 = 1.0 / x[2];
  t82 = 1.0 / x[3];
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
  t77 = 1.0 / t20;
  t78 = 1.0 / t21;
  t80 = 1.0 / t22;
  t81 = 1.0 / t23;
  t83 = 1.0 / t24;
  t84 = 1.0 / t25;
  t85 = 1.0 / t26;
  t90 = 1.0 / t39;
  t98 = t97 * t97;
  t14 = prop_Cl_0 * prop_delta;
  t103 = t14 * (prop_delta + 1.0);
  t104 = t132 - t120;
  t106 = t151 * (1.0 / gain_el);
  t107 = t152 * (1.0 / gain_az);
  t132 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t506 = prop_Cl_a * prop_sigma;
  t134 = t506 * (prop_delta - 1.0) / 8.0;
  t112 = t55 * t55;
  t113 = t56 * t56;
  t114 = t57 * t57;
  t115 = t58 * t58;
  t118 = cos(t104);
  t119 = sin(t104);
  t120 = t103 * 8.0;
  t129 = power_Cd_0 * prop_delta + power_Cd_a * (prop_theta * prop_theta);
  t138 = (t9 + 1.5707963267948966) + t104;
  t139 = (t10 + 1.5707963267948966) + t104;
  t140 = (t11 + 1.5707963267948966) + t104;
  t13 = (t12 + 1.5707963267948966) + t104;
  t142 = cos(t138);
  t143 = cos(t139);
  t144 = cos(t140);
  t145 = cos(t13);
  t146 = sin(t138);
  t147 = sin(t139);
  t148 = sin(t140);
  t149 = sin(t13);
  t162_tmp = t680 * S * rho * t27 * t104;
  t150 = t679 + t681 * (t680 * t680) * (t104 * t104);
  t151 = t142 * t142;
  t152 = t143 * t143;
  t140 = t144 * t144;
  t9 = t145 * t145;
  t155 = t146 * t146;
  t156 = t147 * t147;
  t157 = t148 * t148;
  t158 = t149 * t149;
  t175_tmp_tmp = S * rho;
  t175_tmp = t175_tmp_tmp * cos(t347) * t27;
  t197_tmp = prop_Cl_a * prop_theta * t27;
  t217_tmp_tmp = t14 * t8;
  t217_tmp = t217_tmp_tmp * t27;
  t307 = t162_tmp * t118 / 2.0 + t175_tmp * t119 * t150 / 2.0;
  t340_tmp = t27 * t75 * t90 * t98;
  t138 = V * prop_Cl_a * prop_sigma;
  t139 = prop_Cl_0 * prop_sigma * t8 * t27;
  t13 = prop_sigma * (prop_delta - 1.0) * t99;
  t14 = sqrt(
      ((t340_tmp * t151 * 16.0 +
        t138 * t74 * t89 * (prop_delta - 1.0) * t97 * t142 * 8.0) +
       t139 * t75 * t90 * t98 * t155 * -8.0) -
      t13 * (t120 +
             prop_Cl_a * (t132 + prop_theta * (t95 + t340_tmp * t155 * 8.0))));
  t341_tmp = t27 * t78 * t90 * t98;
  t15 = sqrt(
      ((t341_tmp * t152 * 16.0 +
        t138 * t76 * t89 * (prop_delta - 1.0) * t97 * t143 * 8.0) +
       t139 * t78 * t90 * t98 * t156 * -8.0) -
      t13 * (t120 +
             prop_Cl_a * (t132 + prop_theta * (t95 + t341_tmp * t156 * 8.0))));
  t342_tmp = t27 * t81 * t90 * t98;
  t16 = sqrt(
      ((t342_tmp * t140 * 16.0 +
        t138 * t79 * t89 * (prop_delta - 1.0) * t97 * t144 * 8.0) +
       t139 * t81 * t90 * t98 * t157 * -8.0) -
      t13 * (t120 +
             prop_Cl_a * (t132 + prop_theta * (t95 + t342_tmp * t157 * 8.0))));
  t343_tmp = t27 * t84 * t90 * t98;
  t13 = sqrt(
      ((t343_tmp * t9 * 16.0 +
        t138 * t82 * t89 * (prop_delta - 1.0) * t97 * t145 * 8.0) +
       t139 * t84 * t90 * t98 * t158 * -8.0) -
      t13 * (t120 +
             prop_Cl_a * (t132 + prop_theta * (t95 + t343_tmp * t158 * 8.0))));
  t10 = 1.0 / t14;
  t11 = 1.0 / t15;
  t12 = 1.0 / t16;
  t347 = 1.0 / t13;
  t356 = (t134 + V * t74 * t89 * t97 * t142 / 2.0) + t14 / 8.0;
  t357 = (t134 + V * t76 * t89 * t97 * t143 / 2.0) + t15 / 8.0;
  t358 = (t134 + V * t79 * t89 * t97 * t144 / 2.0) + t16 / 8.0;
  t359 = (t134 + V * t82 * t89 * t97 * t145 / 2.0) + t13 / 8.0;
  t364 = -prop_theta + t356;
  t365 = -prop_theta + t357;
  t366 = -prop_theta + t358;
  t367 = -prop_theta + t359;
  t13 = t506 * prop_theta * t27;
  t120 = V * t75 * t89 * t97 * t142 / 2.0 +
         (((t27 * t77 * t90 * t98 * t151 * 32.0 +
            t138 * t75 * t89 * (prop_delta - 1.0) * t97 * t142 * 8.0) -
           t139 * t77 * t90 * t98 * t155 * 16.0) -
          t13 * t77 * t90 * (prop_delta - 1.0) * t98 * t99 * t155 * 16.0) *
             t10 / 16.0;
  t477 = V * t78 * t89 * t97 * t143 / 2.0 +
         (((t27 * t80 * t90 * t98 * t152 * 32.0 +
            t138 * t78 * t89 * (prop_delta - 1.0) * t97 * t143 * 8.0) -
           t139 * t80 * t90 * t98 * t156 * 16.0) -
          t13 * t80 * t90 * (prop_delta - 1.0) * t98 * t99 * t156 * 16.0) *
             t11 / 16.0;
  t478 = V * t81 * t89 * t97 * t144 / 2.0 +
         (((t27 * t83 * t90 * t98 * t140 * 32.0 +
            t138 * t81 * t89 * (prop_delta - 1.0) * t97 * t144 * 8.0) -
           t139 * t83 * t90 * t98 * t157 * 16.0) -
          t13 * t83 * t90 * (prop_delta - 1.0) * t98 * t99 * t157 * 16.0) *
             t12 / 16.0;
  t479 = V * t84 * t89 * t97 * t145 / 2.0 +
         (((t27 * t85 * t90 * t98 * t9 * 32.0 +
            t138 * t84 * t89 * (prop_delta - 1.0) * t97 * t145 * 8.0) -
           t139 * t85 * t90 * t98 * t158 * 16.0) -
          t13 * t85 * t90 * (prop_delta - 1.0) * t98 * t99 * t158 * 16.0) *
             t347 / 16.0;
  t13 = V * gain_el;
  t14 = t13 * prop_Cl_a * prop_sigma;
  t15 = gain_el * t27;
  t504_tmp_tmp = gain_el * prop_Cl_0;
  t16 = t504_tmp_tmp * prop_sigma * t8 * t27;
  b_t504_tmp_tmp = gain_el * prop_Cl_a;
  t132 = b_t504_tmp_tmp * prop_sigma * prop_theta * t27;
  t504 = t13 * t74 * t89 * t97 * t146 / 2.0 +
         t10 *
             (((t14 * t74 * t89 * (prop_delta - 1.0) * t97 * t146 * 8.0 +
                t15 * t75 * t90 * t98 * t142 * t146 * 32.0) +
               t16 * t75 * t90 * t98 * t142 * t146 * 16.0) +
              t132 * t75 * t90 * (prop_delta - 1.0) * t98 * t99 * t142 * t146 *
                  16.0) /
             16.0;
  t74 = t13 * t76 * t89 * t97 * t147 / 2.0 +
        t11 *
            (((t14 * t76 * t89 * (prop_delta - 1.0) * t97 * t147 * 8.0 +
               t15 * t78 * t90 * t98 * t143 * t147 * 32.0) +
              t16 * t78 * t90 * t98 * t143 * t147 * 16.0) +
             t132 * t78 * t90 * (prop_delta - 1.0) * t98 * t99 * t143 * t147 *
                 16.0) /
            16.0;
  t506 = t13 * t79 * t89 * t97 * t148 / 2.0 +
         t12 *
             (((t14 * t79 * t89 * (prop_delta - 1.0) * t97 * t148 * 8.0 +
                t15 * t81 * t90 * t98 * t144 * t148 * 32.0) +
               t16 * t81 * t90 * t98 * t144 * t148 * 16.0) +
              t132 * t81 * t90 * (prop_delta - 1.0) * t98 * t99 * t144 * t148 *
                  16.0) /
             16.0;
  t95 = t13 * t82 * t89 * t97 * t149 / 2.0 +
        t347 *
            (((t14 * t82 * t89 * (prop_delta - 1.0) * t97 * t149 * 8.0 +
               t15 * t84 * t90 * t98 * t145 * t149 * 32.0) +
              t16 * t84 * t90 * t98 * t145 * t149 * 16.0) +
             t132 * t84 * t90 * (prop_delta - 1.0) * t98 * t99 * t145 * t149 *
                 16.0) /
            16.0;
  t11 = prop_Cl_a * prop_delta;
  t134 = prop_theta * (prop_delta - 1.0);
  t347 = prop_Cl_a - t40;
  t12 = t217_tmp_tmp * t120 + t134 * t120 * t347;
  t679 = t217_tmp_tmp * t477 + t134 * t477 * t347;
  t680 = t217_tmp_tmp * t478 + t134 * t478 * t347;
  t681 = t217_tmp_tmp * t479 + t134 * t479 * t347;
  t13 = prop_Cl_a * t356 * 6.0 - power_Cd_a * t364 * 6.0;
  t138 = prop_Cl_a * t357 * 6.0 - power_Cd_a * t365 * 6.0;
  t139 = prop_Cl_a * t358 * 6.0 - power_Cd_a * t366 * 6.0;
  t140 = prop_Cl_a * t359 * 6.0 - power_Cd_a * t367 * 6.0;
  b_CL_aileron[0] = CL_aileron;
  b_CL_aileron[1] = x[0];
  b_CL_aileron[2] = x[1];
  b_CL_aileron[3] = x[2];
  b_CL_aileron[4] = x[3];
  b_CL_aileron[5] = S;
  b_CL_aileron[6] = V;
  b_CL_aileron[7] = x[4];
  b_CL_aileron[8] = x[5];
  b_CL_aileron[9] = x[6];
  b_CL_aileron[10] = x[7];
  b_CL_aileron[11] = x[12];
  b_CL_aileron[12] = desired_ailerons_value;
  b_CL_aileron[13] = dv_global_1;
  b_CL_aileron[14] = dv_global_2;
  b_CL_aileron[15] = dv_global_3;
  b_CL_aileron[16] = dv_global_4;
  b_CL_aileron[17] = dv_global_5;
  b_CL_aileron[18] = dv_global_6;
  b_CL_aileron[19] = x[8];
  b_CL_aileron[20] = x[9];
  b_CL_aileron[21] = x[10];
  b_CL_aileron[22] = x[11];
  b_CL_aileron[23] = gain_ailerons;
  b_CL_aileron[24] = gain_az;
  b_CL_aileron[25] = gain_el;
  b_CL_aileron[26] = gain_motor;
  b_CL_aileron[27] = gamma_quadratic_du;
  b_CL_aileron[28] = l_1;
  b_CL_aileron[29] = l_2;
  b_CL_aileron[30] = l_3;
  b_CL_aileron[31] = l_4;
  b_CL_aileron[32] = l_z;
  b_CL_aileron[33] = power_Cd_a;
  b_CL_aileron[34] = prop_Cl_0;
  b_CL_aileron[35] = prop_Cl_a;
  b_CL_aileron[36] = prop_sigma;
  b_CL_aileron[37] = rho;
  b_CL_aileron[38] = 1.5707963267948966;
  b_CL_aileron[39] = desired_motor_value * t89;
  b_CL_aileron[40] = -(I_zz * p * r);
  b_CL_aileron[41] = -(I_yy * q * r);
  b_CL_aileron[42] = t112;
  b_CL_aileron[43] = t113;
  b_CL_aileron[44] = t114;
  b_CL_aileron[45] = t115;
  b_CL_aileron[46] = -t106;
  b_CL_aileron[47] = -(t106 * 2.0);
  b_CL_aileron[48] = -t107;
  b_CL_aileron[49] = -(t107 * 2.0);
  b_CL_aileron[50] = t129;
  b_CL_aileron[51] =
      power_Cd_0 * ((prop_delta + 1.0) + prop_delta * prop_delta) * 2.0;
  b_CL_aileron[52] = prop_delta / 6.0 - 0.16666666666666666;
  b_CL_aileron[53] =
      -(CL_aileron * S * x[12] * gain_ailerons * rho * t27 / 2.0);
  b_CL_aileron[54] = t142;
  b_CL_aileron[55] = t143;
  b_CL_aileron[56] = t144;
  b_CL_aileron[57] = t145;
  b_CL_aileron[58] = t146;
  b_CL_aileron[59] = t147;
  b_CL_aileron[60] = t148;
  b_CL_aileron[61] = t149;
  b_CL_aileron[62] = t155;
  b_CL_aileron[63] = t156;
  b_CL_aileron[64] = t157;
  b_CL_aileron[65] = t158;
  t10 = t175_tmp_tmp * t27;
  b_CL_aileron[66] = -(t10 * (Cm_zero + Cm_alpha * t104) * wing_chord / 2.0);
  b_CL_aileron[67] = t175_tmp_tmp * t3 * t4 * t27 * t150 / 2.0;
  t9 = t175_tmp_tmp * t4;
  b_CL_aileron[68] = t9 * t5 * t6 * t27 * t150 / 2.0;
  b_CL_aileron[69] = t9 * t6 * t7 * t27 * t150 / 2.0;
  b_CL_aileron[70] = t19;
  b_CL_aileron[71] = t20;
  b_CL_aileron[72] = t217_tmp * t75 * t90 * t98 * t155;
  b_CL_aileron[73] = t217_tmp * t78 * t90 * t98 * t156;
  b_CL_aileron[74] = t217_tmp * t81 * t90 * t98 * t157;
  b_CL_aileron[75] = t217_tmp * t84 * t90 * t98 * t158;
  b_CL_aileron[76] = t21;
  b_CL_aileron[77] = t22;
  b_CL_aileron[78] = t23;
  b_CL_aileron[79] = t24;
  b_CL_aileron[80] = t25;
  b_CL_aileron[81] = t26;
  b_CL_aileron[82] = t27;
  b_CL_aileron[83] = t340_tmp * t99 * t129 * t155 * 3.0;
  b_CL_aileron[84] = t341_tmp * t99 * t129 * t156 * 3.0;
  b_CL_aileron[85] = t342_tmp * t99 * t129 * t157 * 3.0;
  b_CL_aileron[86] = t343_tmp * t99 * t129 * t158 * 3.0;
  b_CL_aileron[87] = t28;
  b_CL_aileron[88] = W_act_motor * W_act_motor;
  b_CL_aileron[89] = t3;
  b_CL_aileron[90] = W_dv_1 * W_dv_1;
  b_CL_aileron[91] = W_dv_2 * W_dv_2;
  t9 = t162_tmp * t119 / 2.0 - t175_tmp * t118 * t150 / 2.0;
  b_CL_aileron[92] = -t5 * t9;
  b_CL_aileron[93] = -(t6 * t307);
  b_CL_aileron[94] = t3 * t5 * t307;
  b_CL_aileron[95] = t3 * t7 * t307;
  b_CL_aileron[96] = t7 * t9;
  b_CL_aileron[97] = W_dv_3 * W_dv_3;
  b_CL_aileron[98] = W_dv_4 * W_dv_4;
  b_CL_aileron[99] = W_dv_5 * W_dv_5;
  b_CL_aileron[100] = W_dv_6 * W_dv_6;
  b_CL_aileron[101] = W_act_tilt_el * W_act_tilt_el;
  b_CL_aileron[102] = t364;
  b_CL_aileron[103] = t365;
  b_CL_aileron[104] = t366;
  b_CL_aileron[105] = t367;
  b_CL_aileron[106] = t217_tmp_tmp * t356;
  b_CL_aileron[107] = t217_tmp_tmp * t357;
  b_CL_aileron[108] = W_act_tilt_az * W_act_tilt_az;
  b_CL_aileron[109] = t217_tmp_tmp * t358;
  b_CL_aileron[110] = t217_tmp_tmp * t359;
  t9 = prop_Cl_0 * (prop_delta + 1.0);
  b_CL_aileron[111] = t9 * t356 * 3.0;
  b_CL_aileron[112] = t9 * t357 * 3.0;
  b_CL_aileron[113] = t9 * t358 * 3.0;
  b_CL_aileron[114] = t9 * t359 * 3.0;
  b_CL_aileron[115] = W_act_ailerons * W_act_ailerons;
  b_CL_aileron[116] = t39;
  b_CL_aileron[117] = prop_delta + 1.0;
  b_CL_aileron[118] =
      (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t356 * t347));
  b_CL_aileron[119] =
      (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t357 * t347));
  b_CL_aileron[120] =
      (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t358 * t347));
  b_CL_aileron[121] =
      (prop_delta - 1.0) * (t47 + prop_theta * (t48 + t359 * t347));
  b_CL_aileron[122] = t42;
  b_CL_aileron[123] =
      (prop_delta - 1.0) *
      ((t103 + t197_tmp * t75 * t90 * t98 * t155) - t11 * t364 * 2.0);
  b_CL_aileron[124] =
      (prop_delta - 1.0) *
      ((t103 + t197_tmp * t78 * t90 * t98 * t156) - t11 * t365 * 2.0);
  b_CL_aileron[125] =
      (prop_delta - 1.0) *
      ((t103 + t197_tmp * t81 * t90 * t98 * t157) - t11 * t366 * 2.0);
  b_CL_aileron[126] =
      (prop_delta - 1.0) *
      ((t103 + t197_tmp * t84 * t90 * t98 * t158) - t11 * t367 * 2.0);
  b_CL_aileron[127] = t43;
  b_CL_aileron[128] = rt_powd_snf(prop_R, 5.0);
  b_CL_aileron[129] = t120;
  b_CL_aileron[130] = t477;
  b_CL_aileron[131] = t478;
  b_CL_aileron[132] = t479;
  t9 = I_xx * p;
  b_CL_aileron[133] = t9 * q;
  b_CL_aileron[134] = t5;
  b_CL_aileron[135] = I_yy * p * q;
  b_CL_aileron[136] = t504;
  b_CL_aileron[137] = t74;
  b_CL_aileron[138] = t506;
  b_CL_aileron[139] = t95;
  b_CL_aileron[140] = t9 * r;
  b_CL_aileron[141] = I_zz * q * r;
  b_CL_aileron[142] = t13;
  b_CL_aileron[143] = t138;
  b_CL_aileron[144] = t139;
  b_CL_aileron[145] = t140;
  t9 = prop_sigma * rho;
  t152 = t9 * t19 * t39 * t43;
  t120 = t217_tmp * t77 * t90 * t98 * t155 * 2.0 +
         (prop_delta - 1.0) *
             (t197_tmp * t77 * t90 * t98 * t155 * 2.0 - t11 * t120 * 2.0);
  b_CL_aileron[146] = t152 * t63 * t99 * 3.1415926535897931 * t120 * -0.25;
  t151 = t9 * t21 * t39 * t43;
  t132 = t217_tmp * t80 * t90 * t98 * t156 * 2.0 +
         (prop_delta - 1.0) *
             (t197_tmp * t80 * t90 * t98 * t156 * 2.0 - t11 * t477 * 2.0);
  b_CL_aileron[147] = t151 * t64 * t99 * 3.1415926535897931 * t132 * -0.25;
  t16 = t9 * t23 * t39 * t43;
  t15 = t217_tmp * t83 * t90 * t98 * t157 * 2.0 +
        (prop_delta - 1.0) *
            (t197_tmp * t83 * t90 * t98 * t157 * 2.0 - t11 * t478 * 2.0);
  b_CL_aileron[148] = t16 * t65 * t99 * 3.1415926535897931 * t15 * -0.25;
  t9 = t9 * t25 * t39 * t43;
  t14 = t217_tmp * t85 * t90 * t98 * t158 * 2.0 +
        (prop_delta - 1.0) *
            (t197_tmp * t85 * t90 * t98 * t158 * 2.0 - t11 * t479 * 2.0);
  b_CL_aileron[149] = t9 * t66 * t99 * 3.1415926535897931 * t14 * -0.25;
  t152 *= t55;
  b_CL_aileron[150] = t152 * t59 * t99 * 3.1415926535897931 * t120 * -0.25;
  t151 *= t56;
  b_CL_aileron[151] = t151 * t60 * t99 * 3.1415926535897931 * t132 * -0.25;
  b_CL_aileron[152] = t55;
  t16 *= t57;
  b_CL_aileron[153] = t16 * t61 * t99 * 3.1415926535897931 * t15 * -0.25;
  t9 *= t58;
  b_CL_aileron[154] = t9 * t62 * t99 * 3.1415926535897931 * t14 * -0.25;
  b_CL_aileron[155] = t152 * t67 * t99 * 3.1415926535897931 * t120 * -0.25;
  b_CL_aileron[156] = t151 * t68 * t99 * 3.1415926535897931 * t132 * -0.25;
  b_CL_aileron[157] = t16 * t69 * t99 * 3.1415926535897931 * t15 * -0.25;
  b_CL_aileron[158] = t9 * t70 * t99 * 3.1415926535897931 * t14 * -0.25;
  b_CL_aileron[159] = t56;
  t9 = t504_tmp_tmp * prop_delta * t8 * t27;
  t152 = b_t504_tmp_tmp * prop_theta * t27;
  b_CL_aileron[160] =
      t9 * t75 * t90 * t98 * t142 * t146 * 2.0 +
      (prop_delta - 1.0) *
          (t152 * t75 * t90 * t98 * t142 * t146 * 2.0 + t11 * t504 * 2.0);
  b_CL_aileron[161] =
      t9 * t78 * t90 * t98 * t143 * t147 * 2.0 +
      (prop_delta - 1.0) *
          (t152 * t78 * t90 * t98 * t143 * t147 * 2.0 + t11 * t74 * 2.0);
  b_CL_aileron[162] =
      t9 * t81 * t90 * t98 * t144 * t148 * 2.0 +
      (prop_delta - 1.0) *
          (t152 * t81 * t90 * t98 * t144 * t148 * 2.0 + t11 * t506 * 2.0);
  b_CL_aileron[163] =
      t9 * t84 * t90 * t98 * t145 * t149 * 2.0 +
      (prop_delta - 1.0) *
          (t152 * t84 * t90 * t98 * t145 * t149 * 2.0 + t11 * t95 * 2.0);
  b_CL_aileron[164] = t57;
  b_CL_aileron[165] = t58;
  t9 = x[1] * t65;
  t152 = x[0] * t66;
  b_CL_aileron[166] = t10 *
                      (((((((((((x[0] * t55 * 0.01228634392023026 -
                                 x[1] * t56 * 0.01228634392023026) +
                                t19 * t55 * 0.0075456152077779167) +
                               x[1] * t28 * t56 * 0.020516396677824081) -
                              t21 * t56 * 0.0075456152077779167) -
                             x[0] * t28 * t55 * 0.020516396677824081) +
                            t9 * t114 * 0.0064381447596962606) +
                           t9 * t113 * 0.0039349871274520724) +
                          x[0] * t63 * t115 * 0.0046429750925043979) -
                         t152 * t115 * 0.0064381447596962606) -
                        t152 * t112 * 0.0039349871274520724) -
                       x[1] * t64 * t114 * 0.0046429750925043979) *
                      wing_span / 2.0;
  b_CL_aileron[167] = t59;
  b_CL_aileron[168] = t6;
  b_CL_aileron[169] = t60;
  b_CL_aileron[170] = t61;
  b_CL_aileron[171] = t62;
  b_CL_aileron[172] = t63;
  b_CL_aileron[173] = t64;
  b_CL_aileron[174] = t65;
  b_CL_aileron[175] = t66;
  b_CL_aileron[176] = t67;
  b_CL_aileron[177] = t364 * t13;
  b_CL_aileron[178] = t365 * t138;
  b_CL_aileron[179] = t366 * t139;
  b_CL_aileron[180] = t367 * t140;
  b_CL_aileron[181] = t68;
  t10 = x[2] * V * gain_motor * prop_sigma * rho * t42;
  t9 = t10 * t57 * t99 * t148;
  b_CL_aileron[182] = t9 * t680 * 3.1415926535897931 / 4.0;
  t152 = x[3] * V * gain_motor * prop_sigma * rho * t42;
  t120 = t152 * t58 * t99 * t149;
  b_CL_aileron[183] = t120 * t681 * 3.1415926535897931 / 4.0;
  t151 = x[0] * V * gain_motor * prop_sigma * rho * t42;
  t132 = t151 * t55 * t99 * t146;
  b_CL_aileron[184] = -(t132 * t12 * 3.1415926535897931 / 4.0);
  t16 = x[1] * V * gain_motor * prop_sigma * rho * t42;
  t15 = t16 * t56 * t99 * t147;
  b_CL_aileron[185] = -(t15 * t679 * 3.1415926535897931 / 4.0);
  b_CL_aileron[186] = t69;
  t14 = t151 * t59 * t63 * t99 * t146;
  b_CL_aileron[187] = t14 * t12 * 3.1415926535897931 / 4.0;
  t140 = t16 * t60 * t64 * t99 * t147;
  b_CL_aileron[188] = t140 * t679 * 3.1415926535897931 / 4.0;
  t139 = t10 * t61 * t65 * t99 * t148;
  b_CL_aileron[189] = t139 * t680 * 3.1415926535897931 / 4.0;
  t138 = t152 * t62 * t66 * t99 * t149;
  b_CL_aileron[190] = t138 * t681 * 3.1415926535897931 / 4.0;
  t151 = t151 * t63 * t67 * t99 * t146;
  b_CL_aileron[191] = t151 * t12 * 3.1415926535897931 / 4.0;
  t16 = t16 * t64 * t68 * t99 * t147;
  b_CL_aileron[192] = t16 * t679 * 3.1415926535897931 / 4.0;
  t10 = t10 * t65 * t69 * t99 * t148;
  b_CL_aileron[193] = t10 * t680 * 3.1415926535897931 / 4.0;
  t152 = t152 * t66 * t70 * t99 * t149;
  b_CL_aileron[194] = t152 * t681 * 3.1415926535897931 / 4.0;
  b_CL_aileron[195] = t7;
  b_CL_aileron[196] = t70;
  b_CL_aileron[197] = 1.0 / I_xx;
  b_CL_aileron[198] = 1.0 / I_yy;
  b_CL_aileron[199] = 1.0 / I_zz;
  t13 = t217_tmp_tmp * t504 + t134 * t504 * t347;
  b_CL_aileron[200] = t132 * 3.1415926535897931 * t13 * -0.25;
  t132 = t217_tmp_tmp * t74 + t134 * t74 * t347;
  b_CL_aileron[201] = t15 * 3.1415926535897931 * t132 * -0.25;
  b_CL_aileron[202] = t75;
  t15 = t217_tmp_tmp * t506 + t134 * t506 * t347;
  b_CL_aileron[203] = t9 * 3.1415926535897931 * t15 * -0.25;
  t9 = t217_tmp_tmp * t95 + t134 * t95 * t347;
  b_CL_aileron[204] = t120 * 3.1415926535897931 * t9 * -0.25;
  b_CL_aileron[205] = t14 * 3.1415926535897931 * t13 * -0.25;
  b_CL_aileron[206] = t140 * 3.1415926535897931 * t132 * -0.25;
  b_CL_aileron[207] = t139 * 3.1415926535897931 * t15 * -0.25;
  b_CL_aileron[208] = t138 * 3.1415926535897931 * t9 * -0.25;
  b_CL_aileron[209] = t151 * 3.1415926535897931 * t13 * -0.25;
  b_CL_aileron[210] = t16 * 3.1415926535897931 * t132 * -0.25;
  b_CL_aileron[211] = t10 * 3.1415926535897931 * t15 * -0.25;
  b_CL_aileron[212] = t152 * 3.1415926535897931 * t9 * -0.25;
  b_CL_aileron[213] = t77;
  b_CL_aileron[214] = t78;
  b_CL_aileron[215] = t80;
  b_CL_aileron[216] = t81;
  b_CL_aileron[217] = t83;
  b_CL_aileron[218] = t84;
  b_CL_aileron[219] = t85;
  b_CL_aileron[220] = t90;
  b_CL_aileron[221] = 1.0 / gain_ailerons;
  b_CL_aileron[222] = 1.0 / m;
  b_CL_aileron[223] = t98;
  b_CL_aileron[224] = t99;
  b_CL_aileron[225] = wing_span;
  fval = b_ft_1(b_CL_aileron, gradient_data, &idx_current);
  memcpy(&grad_workspace[0], &gradient_data[0], 13U * sizeof(double));
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
  return fval;
}

/*
 * File trailer for computeObjectiveAndUserGradient_.c
 *
 * [EOF]
 */
