/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:29:34
 */

/* Include Files */
#include "evalObjAndConstr.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
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
  double K_Cd;
  double K_p_M;
  double K_p_T;
  double S;
  double W_act_ailerons;
  double W_act_ailerons_du;
  double W_act_tilt_az;
  double W_act_tilt_el;
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
  double gamma_quadratic_du;
  double gamma_quadratic_du2;
  double l_1;
  double l_2;
  double l_3;
  double l_4;
  double l_z;
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
  double q;
  double r;
  double rho;
  double t10;
  double t11;
  double t12;
  double t125;
  double t13;
  double t132_tmp;
  double t133;
  double t133_tmp;
  double t137;
  double t139;
  double t14;
  double t140;
  double t141;
  double t15;
  double t179;
  double t182;
  double t190;
  double t191;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t3;
  double t33;
  double t34;
  double t36;
  double t37;
  double t38;
  double t4;
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
  double t9;
  double t93;
  double wing_chord;
  bool b;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  t182 = c_obj_next_next_next_next_next_->Beta->contents;
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
  t9 = c_obj_next_next_next_next_next_->Phi->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  t82 = c_obj_next_next_next_next_next_->Theta->contents;
  t49 = c_obj_next_next_next_next_next_->V->contents;
  t57 = c_obj_next_next_next_next_next_->W_act_motor->contents;
  W_dv_1 = c_obj_next_next_next_next_next_->W_dv_1->contents;
  W_dv_2 = c_obj_next_next_next_next_next_->W_dv_2->contents;
  W_dv_3 = c_obj_next_next_next_next_next_->W_dv_3->contents;
  W_dv_4 = c_obj_next_next_next_next_next_->W_dv_4->contents;
  W_dv_5 = c_obj_next_next_next_next_next_->W_dv_5->contents;
  W_dv_6 = c_obj_next_next_next_next_next_->W_dv_6->contents;
  W_act_tilt_el = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  W_act_tilt_az = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  W_act_ailerons = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  t141 = c_obj_next_next_next_next_next_->W_act_motor_du->contents;
  W_act_tilt_el_du =
      c_obj_next_next_next_next_next_->W_act_tilt_el_du->contents;
  t140 = c_obj_next_next_next_next_next_->W_act_tilt_az_du->contents;
  W_act_ailerons_du =
      c_obj_next_next_next_next_next_->W_act_ailerons_du->contents;
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
  t139 = c_obj_next_next_next_next_next_->gain_el->contents;
  gain_az = c_obj_next_next_next_next_next_->gain_az->contents;
  t137 = c_obj_next_next_next_next_next_->gain_motor->contents;
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
  t133 = c_obj_next_next_next_next_next_->m->contents;
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
  q = c_obj_next_next_next_next_next_->q->contents;
  r = c_obj_next_next_next_next_next_->r->contents;
  rho = c_obj_next_next_next_next_next_->rho->contents;
  wing_chord = c_obj_next_next_next_next_next_->wing_chord->contents;
  /* COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2 */
  /*     [COST,GRADIENT] =
   * COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi,S,Theta,V,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,W_act_motor_du,W_act_tilt_el_du,W_act_tilt_az_du,W_act_ailerons_du,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,GAMMA_QUADRATIC_DU2,L_1,L_2,L_3,L_4,L_Z,M,P,PREVIOUS_EL_VALUE1,PREVIOUS_EL_VALUE2,PREVIOUS_EL_VALUE3,PREVIOUS_EL_VALUE4,PREVIOUS_AZ_VALUE1,PREVIOUS_AZ_VALUE2,PREVIOUS_AZ_VALUE3,PREVIOUS_AZ_VALUE4,PREVIOUS_MOTOR_VALUE1,PREVIOUS_MOTOR_VALUE2,PREVIOUS_MOTOR_VALUE3,PREVIOUS_MOTOR_VALUE4,PREVIOUS_AILERONS_VALUE,Q,R,RHO,WING_CHORD)
   */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     20-Feb-2024 20:24:21 */
  t3 = cos(t9);
  t4 = sin(t182);
  t5 = cos(t82);
  t6 = sin(t9);
  t7 = sin(t82);
  t8 = x[4] * t139;
  t9 = x[5] * t139;
  t10 = x[6] * t139;
  t11 = x[7] * t139;
  t12 = x[8] * gain_az;
  t13 = x[9] * gain_az;
  t14 = x[10] * gain_az;
  t15 = x[11] * gain_az;
  t21 = x[0] * x[0];
  t22 = x[1] * x[1];
  t23 = x[2] * x[2];
  t24 = x[3] * x[3];
  t25 = t49 * t49;
  t26 = t57 * t57;
  t33 = W_act_tilt_el * W_act_tilt_el;
  t34 = W_act_tilt_az * W_act_tilt_az;
  t36 = t141 * t141;
  t37 = W_act_tilt_el_du * W_act_tilt_el_du;
  t38 = t140 * t140;
  t49 = t137 * t137;
  t77 = 1.0 / t139;
  t78 = 1.0 / gain_az;
  t79 = 1.0 / t137;
  t80 = 1.0 / gain_ailerons;
  t81 = 1.0 / t133;
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
  t82 -= flight_path_angle;
  t84 = desired_el_value * t77;
  t85 = desired_az_value * t78;
  t86 = desired_motor_value * t79;
  t15 = cos(t82);
  t93 = sin(t82);
  flight_path_angle = K_p_T * t21 * t49;
  t12 = flight_path_angle * t64;
  t9 = K_p_T * t22 * t49;
  t13 = t9 * t65;
  desired_el_value = K_p_T * t23 * t49;
  t14 = desired_el_value * t66;
  desired_az_value = K_p_T * t24 * t49;
  t11 = desired_az_value * t67;
  desired_motor_value = flight_path_angle * t56;
  t10 = desired_motor_value * t60;
  K_p_T = t9 * t57;
  t9 = K_p_T * t61;
  t132_tmp = desired_el_value * t58;
  t8 = t132_tmp * t62;
  t133_tmp = desired_az_value * t59;
  t133 = t133_tmp * t63;
  t137 = desired_motor_value * t68;
  t139 = K_p_T * t69;
  t140 = t132_tmp * t70;
  t141 = t133_tmp * t71;
  gain_az = Cl_alpha * S * rho * t25 * t82;
  t179 = ((t12 + t13) + t14) + t11;
  t190 = ((t10 + t9) + t8) + t133;
  t191 = ((t137 + t139) + t140) + t141;
  t125 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t82 * t82);
  W_act_tilt_el_du = S * rho;
  W_act_tilt_az = W_act_tilt_el_du * cos(t182) * t25;
  t182 = gain_az * t15 / 2.0 + W_act_tilt_az * t93 * t125 / 2.0;
  K_Cd = K_p_M * t21 * t49;
  Cl_alpha = K_p_M * t23 * t49;
  flight_path_angle = K_p_M * t22 * t49;
  t49 *= K_p_M * t24;
  W_act_tilt_el = I_xx * p;
  t57 *= flight_path_angle;
  Cd_zero = t49 * t59;
  t56 *= K_Cd;
  t59 = Cl_alpha * t58;
  t58 = dv_global_6 -
        1.0 / I_zz *
            (((((((((((((W_act_tilt_el * q - I_yy * p * q) + l_1 * t13) +
                       l_2 * t14) -
                      l_1 * t12) -
                     l_2 * t11) +
                    t56 * t60) +
                   t59 * t62) +
                  l_4 * t137) +
                 l_4 * t139) -
                t57 * t61) -
               Cd_zero * t63) -
              l_3 * t140) -
             l_3 * t141);
  t67 =
      dv_global_4 -
      1.0 / I_xx *
          ((((((((((((((I_yy * q * r - I_zz * q * r) + K_Cd * t64) +
                      Cl_alpha * t66) +
                     CL_aileron * S * x[12] * gain_ailerons * rho * t25 / 2.0) -
                    flight_path_angle * t65) -
                   t49 * t67) +
                  l_1 * t10) +
                 l_2 * t133) +
                l_z * t137) +
               l_z * t139) +
              l_z * t140) +
             l_z * t141) -
            l_1 * t9) -
           l_2 * t8);
  t82 = dv_global_5 -
        1.0 / I_yy *
            ((((((((((((((I_zz * p * r - W_act_tilt_el * r) + l_z * t12) +
                        l_z * t13) +
                       l_z * t14) +
                      l_z * t11) +
                     t57 * t69) +
                    Cd_zero * t71) +
                   l_4 * t10) +
                  l_4 * t9) -
                 t56 * t68) -
                t59 * t70) -
               l_3 * t8) -
              l_3 * t133) +
             W_act_tilt_el_du * t25 * (Cm_zero + Cm_alpha * t82) * wing_chord /
                 2.0);
  K_Cd = t3 * t7;
  Cl_alpha = W_act_tilt_el_du * t4;
  flight_path_angle = gain_az * t93 / 2.0 - W_act_tilt_az * t15 * t125 / 2.0;
  Cd_zero = dv_global_1 +
            t81 * (((((Cl_alpha * t6 * t7 * t25 * t125 / 2.0 + t5 * t179) +
                      -t5 * flight_path_angle) +
                     K_Cd * t182) +
                    K_Cd * t190) -
                   t6 * t7 * t191);
  t56 = dv_global_2 -
        t81 * (((-(W_act_tilt_el_du * t3 * t4 * t25 * t125 / 2.0) + t6 * t182) +
                t6 * t190) +
               t3 * t191);
  t13 = x[12] - desired_ailerons_value * t80;
  t14 = x[0] - t86;
  t59 = x[1] - t86;
  t15 = x[2] - t86;
  desired_motor_value = x[3] - t86;
  K_p_T = x[4] - t84;
  t132_tmp = x[5] - t84;
  t133_tmp = x[6] - t84;
  desired_el_value = x[7] - t84;
  t11 = x[8] - t85;
  desired_az_value = x[9] - t85;
  t12 = x[10] - t85;
  t10 = x[11] - t85;
  K_Cd = t3 * t5;
  t8 = (dv_global_3 +
        t81 * (((((Cl_alpha * t5 * t6 * t25 * t125 / 2.0 - t7 * t179) +
                  K_Cd * t182) +
                 K_Cd * t190) -
                t5 * t6 * t191) +
               t7 * flight_path_angle)) -
       9.81;
  flight_path_angle = x[0] - previous_motor_value1 * t79;
  t133 = x[1] - previous_motor_value2 * t79;
  t137 = x[2] - previous_motor_value3 * t79;
  gain_az = x[3] - previous_motor_value4 * t79;
  t139 = x[4] - previous_el_value1 * t77;
  t140 = x[5] - previous_el_value2 * t77;
  W_act_tilt_el_du = x[6] - previous_el_value3 * t77;
  t141 = x[7] - previous_el_value4 * t77;
  W_act_tilt_az = x[12] - previous_ailerons_value * t80;
  W_act_tilt_el = x[8] - previous_az_value1 * t78;
  t57 = x[9] - previous_az_value2 * t78;
  t49 = x[10] - previous_az_value3 * t78;
  t9 = x[11] - previous_az_value4 * t78;
  fval = ((((((gamma_quadratic_du *
                   ((((((((((((W_act_ailerons * W_act_ailerons * (t13 * t13) +
                               t26 * (t14 * t14)) +
                              t26 * (t59 * t59)) +
                             t26 * (t15 * t15)) +
                            t26 * (desired_motor_value * desired_motor_value)) +
                           t33 * (K_p_T * K_p_T)) +
                          t33 * (t132_tmp * t132_tmp)) +
                         t33 * (t133_tmp * t133_tmp)) +
                        t33 * (desired_el_value * desired_el_value)) +
                       t34 * (t11 * t11)) +
                      t34 * (desired_az_value * desired_az_value)) +
                     t34 * (t12 * t12)) +
                    t34 * (t10 * t10)) +
               W_dv_3 * W_dv_3 * (t8 * t8)) +
              W_dv_6 * W_dv_6 * (t58 * t58)) +
             W_dv_4 * W_dv_4 * (t67 * t67)) +
            W_dv_2 * W_dv_2 * (t56 * t56)) +
           W_dv_5 * W_dv_5 * (t82 * t82)) +
          W_dv_1 * W_dv_1 * (Cd_zero * Cd_zero)) +
         gamma_quadratic_du2 *
             ((((((((((((t36 * (flight_path_angle * flight_path_angle) +
                         t36 * (t133 * t133)) +
                        t36 * (t137 * t137)) +
                       t36 * (gain_az * gain_az)) +
                      t37 * (t139 * t139)) +
                     t37 * (t140 * t140)) +
                    t37 * (W_act_tilt_el_du * W_act_tilt_el_du)) +
                   t37 * (t141 * t141)) +
                  W_act_ailerons_du * W_act_ailerons_du *
                      (W_act_tilt_az * W_act_tilt_az)) +
                 t38 * (W_act_tilt_el * W_act_tilt_el)) +
                t38 * (t57 * t57)) +
               t38 * (t49 * t49)) +
              t38 * (t9 * t9));
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
  double Cd_zero;
  double Cl_alpha;
  double K_Cd;
  double K_p_T;
  double S;
  double V;
  double W_act_ailerons;
  double W_act_ailerons_du;
  double W_act_motor;
  double W_act_phi;
  double W_act_phi_du;
  double W_act_theta;
  double W_act_theta_du;
  double W_dv_1;
  double a_tmp;
  double b_a_tmp;
  double c_a_tmp;
  double d_a_tmp;
  double desired_ailerons_value;
  double desired_az_value;
  double desired_el_value;
  double desired_motor_value;
  double desired_phi_value;
  double desired_theta_value;
  double dv_global_1;
  double e_a_tmp;
  double fval;
  double gain_ailerons;
  double gain_motor;
  double gain_phi;
  double gain_theta;
  double gamma_quadratic_du;
  double gamma_quadratic_du2;
  double m;
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
  double previous_phi_value;
  double previous_theta_value;
  double rho;
  double t10;
  double t11;
  double t12;
  double t13;
  double t25;
  double t28;
  double t31;
  double t32;
  double t35;
  double t36;
  double t37;
  double t4;
  double t48;
  double t5;
  double t50;
  double t51;
  double t56;
  double t6;
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
  double t83;
  double t84;
  double t85;
  double t9;
  double t98;
  bool b;
  /* The thought was here that feeding it an all zero reference */
  /* acceleration vectro would calm down the oscillatory behaviour. As */
  /* this was not successful, the vector remains as desired linear */
  /* accelerations and zero angulars. Instead the gains responsible for */
  /* generating the desired roll commands were adjusted. */
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  Beta = c_obj_next_next_next_next_next_->Beta->contents;
  Cd_zero = c_obj_next_next_next_next_next_->Cd_zero->contents;
  Cl_alpha = c_obj_next_next_next_next_next_->Cl_alpha->contents;
  K_Cd = c_obj_next_next_next_next_next_->K_Cd->contents;
  K_p_T = c_obj_next_next_next_next_next_->K_p_T->contents;
  S = c_obj_next_next_next_next_next_->S->contents;
  V = c_obj_next_next_next_next_next_->V->contents;
  W_act_phi = c_obj_next_next_next_next_next_->W_act_phi->contents;
  W_act_theta = c_obj_next_next_next_next_next_->W_act_theta->contents;
  W_act_motor = c_obj_next_next_next_next_next_->W_act_motor->contents;
  W_act_phi_du = c_obj_next_next_next_next_next_->W_act_phi_du->contents;
  W_dv_1 = c_obj_next_next_next_next_next_->W_dv_1->contents;
  t98 = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  t84 = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  W_act_theta_du = c_obj_next_next_next_next_next_->W_act_theta_du->contents;
  W_act_ailerons = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  t85 = c_obj_next_next_next_next_next_->W_act_motor_du->contents;
  t50 = c_obj_next_next_next_next_next_->W_act_tilt_el_du->contents;
  t51 = c_obj_next_next_next_next_next_->W_act_tilt_az_du->contents;
  W_act_ailerons_du =
      c_obj_next_next_next_next_next_->W_act_ailerons_du->contents;
  desired_el_value =
      c_obj_next_next_next_next_next_->desired_el_value->contents;
  desired_az_value =
      c_obj_next_next_next_next_next_->desired_az_value->contents;
  desired_phi_value =
      c_obj_next_next_next_next_next_->desired_phi_value->contents;
  desired_theta_value =
      c_obj_next_next_next_next_next_->desired_theta_value->contents;
  desired_motor_value =
      c_obj_next_next_next_next_next_->desired_motor_value->contents;
  desired_ailerons_value =
      c_obj_next_next_next_next_next_->desired_ailerons_value->contents;
  t83 = c_obj_next_next_next_next_next_->flight_path_angle->contents;
  t56 = c_obj_next_next_next_next_next_->gain_el->contents;
  t73 = c_obj_next_next_next_next_next_->gain_az->contents;
  gain_phi = c_obj_next_next_next_next_next_->gain_phi->contents;
  gain_theta = c_obj_next_next_next_next_next_->gain_theta->contents;
  gain_motor = c_obj_next_next_next_next_next_->gain_motor->contents;
  gain_ailerons = c_obj_next_next_next_next_next_->gain_ailerons->contents;
  gamma_quadratic_du =
      c_obj_next_next_next_next_next_->gamma_quadratic_du->contents;
  gamma_quadratic_du2 =
      c_obj_next_next_next_next_next_->gamma_quadratic_du2->contents;
  m = c_obj_next_next_next_next_next_->m->contents;
  previous_el_value1 = c_obj_next_next_next_next_next_->actual_u->contents[4];
  previous_el_value2 = c_obj_next_next_next_next_next_->actual_u->contents[5];
  previous_el_value3 = c_obj_next_next_next_next_next_->actual_u->contents[6];
  previous_el_value4 = c_obj_next_next_next_next_next_->actual_u->contents[7];
  previous_az_value1 = c_obj_next_next_next_next_next_->actual_u->contents[8];
  previous_az_value2 = c_obj_next_next_next_next_next_->actual_u->contents[9];
  previous_az_value3 = c_obj_next_next_next_next_next_->actual_u->contents[10];
  previous_az_value4 = c_obj_next_next_next_next_next_->actual_u->contents[11];
  previous_phi_value = c_obj_next_next_next_next_next_->actual_u->contents[13];
  previous_theta_value =
      c_obj_next_next_next_next_next_->actual_u->contents[12];
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
  rho = c_obj_next_next_next_next_next_->rho->contents;
  /* COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2 */
  /*     [COST,GRADIENT] =
   * COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2(Beta,Cd_zero,Cl_alpha,K_Cd,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_act_phi_du,W_dv_1,W_act_tilt_el,W_act_tilt_az,W_act_theta_du,W_act_ailerons,W_act_motor_du,W_act_tilt_el_du,W_act_tilt_az_du,W_act_ailerons_du,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,GAMMA_QUADRATIC_DU2,M,PREVIOUS_EL_VALUE1,PREVIOUS_EL_VALUE2,PREVIOUS_EL_VALUE3,PREVIOUS_EL_VALUE4,PREVIOUS_AZ_VALUE1,PREVIOUS_AZ_VALUE2,PREVIOUS_AZ_VALUE3,PREVIOUS_AZ_VALUE4,PREVIOUS_PHI_VALUE,PREVIOUS_THETA_VALUE,PREVIOUS_MOTOR_VALUE1,PREVIOUS_MOTOR_VALUE2,PREVIOUS_MOTOR_VALUE3,PREVIOUS_MOTOR_VALUE4,PREVIOUS_AILERONS_VALUE,RHO)
   */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     20-Feb-2024 20:24:17 */
  t4 = x[13] * gain_phi;
  t5 = x[12] * gain_theta;
  t6 = x[4] * t56;
  t7 = x[5] * t56;
  t8 = x[6] * t56;
  t9 = x[7] * t56;
  t10 = x[8] * t73;
  t11 = x[9] * t73;
  t12 = x[10] * t73;
  t13 = x[11] * t73;
  t25 = V * V;
  t28 = W_act_motor * W_act_motor;
  t31 = t98 * t98;
  t32 = t84 * t84;
  t35 = t85 * t85;
  t36 = t50 * t50;
  t37 = t51 * t51;
  t48 = gain_motor * gain_motor;
  t69 = 1.0 / t56;
  t70 = 1.0 / t73;
  t71 = 1.0 / gain_phi;
  t72 = 1.0 / gain_theta;
  t73 = 1.0 / gain_motor;
  t74 = 1.0 / gain_ailerons;
  t50 = cos(t5);
  t51 = sin(t4);
  t56 = sin(t5);
  t77 = desired_el_value * t69;
  t78 = desired_az_value * t70;
  t79 = desired_motor_value * t73;
  t83 -= t5;
  t84 = cos(t83);
  t85 = sin(t83);
  t98 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t83 * t83);
  a_tmp = S * rho;
  V = Cl_alpha * S * rho * t25 * t83;
  W_act_motor = a_tmp * cos(Beta) * t25;
  b_a_tmp = cos(t4) * t56;
  c_a_tmp = K_p_T * (x[0] * x[0]) * t48;
  d_a_tmp = K_p_T * (x[1] * x[1]) * t48;
  e_a_tmp = K_p_T * (x[2] * x[2]) * t48;
  K_Cd = K_p_T * (x[3] * x[3]) * t48;
  t48 = c_a_tmp * cos(t6);
  Cl_alpha = d_a_tmp * cos(t7);
  S = e_a_tmp * cos(t8);
  rho = K_Cd * cos(t9);
  K_p_T =
      dv_global_1 +
      1.0 / m *
          (((((a_tmp * sin(Beta) * t25 * t51 * t56 * t98 / 2.0 +
               t50 * (((c_a_tmp * sin(t6) + d_a_tmp * sin(t7)) +
                       e_a_tmp * sin(t8)) +
                      K_Cd * sin(t9))) -
              t50 * (V * t85 / 2.0 - W_act_motor * t84 * t98 / 2.0)) -
             b_a_tmp * (V * t84 / 2.0 + W_act_motor * t85 * t98 / 2.0)) +
            b_a_tmp * (((t48 * cos(t10) + Cl_alpha * cos(t11)) + S * cos(t12)) +
                       rho * cos(t13))) -
           t51 * t56 *
               (((t48 * sin(t10) + Cl_alpha * sin(t11)) + S * sin(t12)) +
                rho * sin(t13)));
  t4 = x[0] - previous_motor_value1 * t73;
  Cd_zero = x[1] - previous_motor_value2 * t73;
  t5 = x[2] - previous_motor_value3 * t73;
  desired_motor_value = x[3] - previous_motor_value4 * t73;
  desired_az_value = x[13] - previous_phi_value * t71;
  desired_el_value = x[12] - previous_theta_value * t72;
  a_tmp = x[4] - previous_el_value1 * t69;
  gain_ailerons = x[5] - previous_el_value2 * t69;
  gain_motor = x[6] - previous_el_value3 * t69;
  e_a_tmp = x[7] - previous_el_value4 * t69;
  d_a_tmp = x[14] - previous_ailerons_value * t74;
  c_a_tmp = x[8] - previous_az_value1 * t70;
  b_a_tmp = x[9] - previous_az_value2 * t70;
  gain_theta = x[10] - previous_az_value3 * t70;
  gain_phi = x[11] - previous_az_value4 * t70;
  rho = x[13] - desired_phi_value * t71;
  S = x[12] - desired_theta_value * t72;
  t73 = x[14] - desired_ailerons_value * t74;
  K_Cd = x[0] - t79;
  t48 = x[1] - t79;
  Cl_alpha = x[2] - t79;
  t50 = x[3] - t79;
  t51 = x[4] - t77;
  t83 = x[5] - t77;
  t56 = x[6] - t77;
  W_act_motor = x[7] - t77;
  t98 = x[8] - t78;
  t84 = x[9] - t78;
  t85 = x[10] - t78;
  V = x[11] - t78;
  fval = gamma_quadratic_du2 *
             ((((((((((((((t35 * (t4 * t4) + t35 * (Cd_zero * Cd_zero)) +
                          t35 * (t5 * t5)) +
                         t35 * (desired_motor_value * desired_motor_value)) +
                        W_act_phi_du * W_act_phi_du *
                            (desired_az_value * desired_az_value)) +
                       W_act_theta_du * W_act_theta_du *
                           (desired_el_value * desired_el_value)) +
                      t36 * (a_tmp * a_tmp)) +
                     t36 * (gain_ailerons * gain_ailerons)) +
                    t36 * (gain_motor * gain_motor)) +
                   t36 * (e_a_tmp * e_a_tmp)) +
                  W_act_ailerons_du * W_act_ailerons_du * (d_a_tmp * d_a_tmp)) +
                 t37 * (c_a_tmp * c_a_tmp)) +
                t37 * (b_a_tmp * b_a_tmp)) +
               t37 * (gain_theta * gain_theta)) +
              t37 * (gain_phi * gain_phi)) +
         (gamma_quadratic_du *
              ((((((((((((((W_act_phi * W_act_phi * (rho * rho) +
                            W_act_theta * W_act_theta * (S * S)) +
                           W_act_ailerons * W_act_ailerons * (t73 * t73)) +
                          t28 * (K_Cd * K_Cd)) +
                         t28 * (t48 * t48)) +
                        t28 * (Cl_alpha * Cl_alpha)) +
                       t28 * (t50 * t50)) +
                      t31 * (t51 * t51)) +
                     t31 * (t83 * t83)) +
                    t31 * (t56 * t56)) +
                   t31 * (W_act_motor * W_act_motor)) +
                  t32 * (t98 * t98)) +
                 t32 * (t84 * t84)) +
                t32 * (t85 * t85)) +
               t32 * (V * V)) +
          W_dv_1 * W_dv_1 * (K_p_T * K_p_T));
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
