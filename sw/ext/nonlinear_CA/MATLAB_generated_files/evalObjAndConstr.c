/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: evalObjAndConstr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:26:49
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
  /*     This function was generated by the Symbolic Math Toolbox version 9.3.
   */
  /*     20-Feb-2024 16:03:48 */
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
  double W_act_motor_du;
  double W_act_phi;
  double W_act_phi_du;
  double W_act_theta;
  double W_act_theta_du;
  double W_act_tilt_az;
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
  double desired_phi_value;
  double desired_theta_value;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double flight_path_angle;
  double fval;
  double gain_ailerons;
  double gain_el;
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
  double previous_phi_value;
  double previous_theta_value;
  double q;
  double r;
  double rho;
  double t10;
  double t101;
  double t102;
  double t103;
  double t11;
  double t117;
  double t118;
  double t119;
  double t12;
  double t13;
  double t136;
  double t150;
  double t151;
  double t152;
  double t19;
  double t20;
  double t200;
  double t203;
  double t204;
  double t21;
  double t22;
  double t25;
  double t28;
  double t36;
  double t37;
  double t4;
  double t40;
  double t41;
  double t42;
  double t5;
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
  double wing_chord;
  bool b;
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
  S = c_obj_next_next_next_next_next_->S->contents;
  t102 = c_obj_next_next_next_next_next_->V->contents;
  W_act_phi = c_obj_next_next_next_next_next_->W_act_phi->contents;
  W_act_theta = c_obj_next_next_next_next_next_->W_act_theta->contents;
  t103 = c_obj_next_next_next_next_next_->W_act_motor->contents;
  W_act_phi_du = c_obj_next_next_next_next_next_->W_act_phi_du->contents;
  W_dv_1 = c_obj_next_next_next_next_next_->W_dv_1->contents;
  W_dv_2 = c_obj_next_next_next_next_next_->W_dv_2->contents;
  W_dv_3 = c_obj_next_next_next_next_next_->W_dv_3->contents;
  W_dv_4 = c_obj_next_next_next_next_next_->W_dv_4->contents;
  W_dv_5 = c_obj_next_next_next_next_next_->W_dv_5->contents;
  W_dv_6 = c_obj_next_next_next_next_next_->W_dv_6->contents;
  t53 = c_obj_next_next_next_next_next_->W_act_tilt_el->contents;
  W_act_tilt_az = c_obj_next_next_next_next_next_->W_act_tilt_az->contents;
  W_act_theta_du = c_obj_next_next_next_next_next_->W_act_theta_du->contents;
  W_act_ailerons = c_obj_next_next_next_next_next_->W_act_ailerons->contents;
  W_act_motor_du = c_obj_next_next_next_next_next_->W_act_motor_du->contents;
  W_act_tilt_el_du =
      c_obj_next_next_next_next_next_->W_act_tilt_el_du->contents;
  t204 = c_obj_next_next_next_next_next_->W_act_tilt_az_du->contents;
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
  flight_path_angle =
      c_obj_next_next_next_next_next_->flight_path_angle->contents;
  gain_el = c_obj_next_next_next_next_next_->gain_el->contents;
  t152 = c_obj_next_next_next_next_next_->gain_az->contents;
  t151 = c_obj_next_next_next_next_next_->gain_phi->contents;
  t203 = c_obj_next_next_next_next_next_->gain_theta->contents;
  t150 = c_obj_next_next_next_next_next_->gain_motor->contents;
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
  t136 = c_obj_next_next_next_next_next_->m->contents;
  p = c_obj_next_next_next_next_next_->p->contents;
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
  q = c_obj_next_next_next_next_next_->q->contents;
  r = c_obj_next_next_next_next_next_->r->contents;
  rho = c_obj_next_next_next_next_next_->rho->contents;
  wing_chord = c_obj_next_next_next_next_next_->wing_chord->contents;
  /* COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2 */
  /*     [COST,GRADIENT] =
   * COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_act_phi_du,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_theta_du,W_act_ailerons,W_act_motor_du,W_act_tilt_el_du,W_act_tilt_az_du,W_act_ailerons_du,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,GAMMA_QUADRATIC_DU2,L_1,L_2,L_3,L_4,L_Z,M,P,PREVIOUS_EL_VALUE1,PREVIOUS_EL_VALUE2,PREVIOUS_EL_VALUE3,PREVIOUS_EL_VALUE4,PREVIOUS_AZ_VALUE1,PREVIOUS_AZ_VALUE2,PREVIOUS_AZ_VALUE3,PREVIOUS_AZ_VALUE4,PREVIOUS_PHI_VALUE,PREVIOUS_THETA_VALUE,PREVIOUS_MOTOR_VALUE1,PREVIOUS_MOTOR_VALUE2,PREVIOUS_MOTOR_VALUE3,PREVIOUS_MOTOR_VALUE4,PREVIOUS_AILERONS_VALUE,Q,R,RHO,WING_CHORD)
   */
  /*     This function was generated by the Symbolic Math Toolbox version 9.3.
   */
  /*     20-Feb-2024 16:03:43 */
  t4 = x[13] * t151;
  t5 = x[12] * t203;
  t6 = x[4] * gain_el;
  t7 = x[5] * gain_el;
  t8 = x[6] * gain_el;
  t9 = x[7] * gain_el;
  t10 = x[8] * t152;
  t11 = x[9] * t152;
  t12 = x[10] * t152;
  t13 = x[11] * t152;
  t19 = x[0] * x[0];
  t20 = x[1] * x[1];
  t21 = x[2] * x[2];
  t22 = x[3] * x[3];
  t25 = t102 * t102;
  t28 = t103 * t103;
  t36 = t53 * t53;
  t37 = W_act_tilt_az * W_act_tilt_az;
  t40 = W_act_motor_du * W_act_motor_du;
  t41 = W_act_tilt_el_du * W_act_tilt_el_du;
  t42 = t204 * t204;
  t53 = t150 * t150;
  t84 = 1.0 / gain_el;
  t85 = 1.0 / t152;
  t86 = 1.0 / t151;
  t87 = 1.0 / t203;
  t88 = 1.0 / t150;
  t89 = 1.0 / gain_ailerons;
  t90 = 1.0 / t136;
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
  t13 = K_p_T * t19 * t53;
  t117 = t13 * t72;
  desired_el_value = K_p_T * t20 * t53;
  t118 = desired_el_value * t73;
  desired_az_value = K_p_T * t21 * t53;
  t119 = desired_az_value * t74;
  desired_motor_value = K_p_T * t22 * t53;
  t12 = desired_motor_value * t75;
  flight_path_angle = t13 * t63;
  t11 = flight_path_angle * t68;
  t5 = desired_el_value * t64;
  t10 = t5 * t69;
  K_p_T = desired_az_value * t65;
  t9 = K_p_T * t70;
  t200 = desired_motor_value * t66;
  t8 = t200 * t71;
  t4 = flight_path_angle * t76;
  t150 = t5 * t77;
  t151 = K_p_T * t78;
  t152 = t200 * t79;
  t102 = cos(t101);
  t103 = sin(t101);
  t200 = ((t117 + t118) + t119) + t12;
  t6 = ((t11 + t10) + t9) + t8;
  t7 = ((t4 + t150) + t151) + t152;
  t136 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t101 * t101);
  Cd_zero = Cl_alpha * S * rho * t25 * t101;
  gain_el = S * rho;
  K_Cd = gain_el * cos(Beta) * t25;
  t203 = Cd_zero * t102 / 2.0 + K_Cd * t103 * t136 / 2.0;
  t204 = Cd_zero * t103 / 2.0 - K_Cd * t102 * t136 / 2.0;
  W_act_tilt_el_du = K_p_M * t19 * t53;
  W_act_motor_du = K_p_M * t21 * t53;
  W_act_tilt_az = K_p_M * t20 * t53;
  t102 = K_p_M * t22 * t53;
  t53 = I_xx * p;
  t103 = W_act_tilt_az * t64;
  Cl_alpha = t102 * t66;
  t66 = W_act_tilt_el_du * t63;
  t64 = W_act_motor_du * t65;
  t63 = dv_global_6 -
        1.0 / I_zz *
            (((((((((((((t53 * q - I_yy * p * q) + l_1 * t118) + l_2 * t119) -
                      l_1 * t117) -
                     l_2 * t12) +
                    t66 * t68) +
                   t64 * t70) +
                  l_4 * t4) +
                 l_4 * t150) -
                t103 * t69) -
               Cl_alpha * t71) -
              l_3 * t151) -
             l_3 * t152);
  t75 =
      dv_global_4 -
      1.0 / I_xx *
          ((((((((((((((I_yy * q * r - I_zz * q * r) + W_act_tilt_el_du * t72) +
                      W_act_motor_du * t74) +
                     CL_aileron * S * x[14] * gain_ailerons * rho * t25 / 2.0) -
                    W_act_tilt_az * t73) -
                   t102 * t75) +
                  l_1 * t11) +
                 l_2 * t8) +
                l_z * t4) +
               l_z * t150) +
              l_z * t151) +
             l_z * t152) -
            l_1 * t10) -
           l_2 * t9);
  t12 = dv_global_5 -
        1.0 / I_yy *
            ((((((((((((((I_zz * p * r - t53 * r) + l_z * t117) + l_z * t118) +
                       l_z * t119) +
                      l_z * t12) +
                     t103 * t77) +
                    Cl_alpha * t79) +
                   l_4 * t11) +
                  l_4 * t10) -
                 t66 * t76) -
                t64 * t78) -
               l_3 * t9) -
              l_3 * t8) +
             gain_el * t25 * (Cm_zero - Cm_alpha * t101) * wing_chord / 2.0);
  W_act_tilt_el_du = gain_el * sin(Beta) * t25;
  t11 = dv_global_2 +
        t90 * (((W_act_tilt_el_du * t54 * t136 / 2.0 + t58 * t203) - t58 * t6) -
               t54 * t7);
  W_act_motor_du = t54 * t55;
  t10 = (-dv_global_3 +
         -t90 * (((((W_act_tilt_el_du * t55 * t58 * t136 / 2.0 - t67 * t200) +
                    t67 * t204) -
                   W_act_motor_du * t203) -
                  t55 * t58 * t7) +
                 W_act_motor_du * t6)) +
        9.81;
  W_act_motor_du = t54 * t67;
  Cl_alpha =
      dv_global_1 +
      t90 * (((((W_act_tilt_el_du * t58 * t67 * t136 / 2.0 + t55 * t200) -
                t55 * t204) -
               W_act_motor_du * t203) +
              W_act_motor_du * t6) -
             t58 * t67 * t7);
  K_Cd = x[0] - previous_motor_value1 * t88;
  Cd_zero = x[1] - previous_motor_value2 * t88;
  t9 = x[2] - previous_motor_value3 * t88;
  t66 = x[3] - previous_motor_value4 * t88;
  t8 = x[13] - previous_phi_value * t86;
  t64 = x[12] - previous_theta_value * t87;
  t7 = x[4] - previous_el_value1 * t84;
  t6 = x[5] - previous_el_value2 * t84;
  t200 = x[6] - previous_el_value3 * t84;
  K_p_T = x[7] - previous_el_value4 * t84;
  t5 = x[14] - previous_ailerons_value * t89;
  flight_path_angle = x[8] - previous_az_value1 * t85;
  desired_motor_value = x[9] - previous_az_value2 * t85;
  desired_az_value = x[10] - previous_az_value3 * t85;
  desired_el_value = x[11] - previous_az_value4 * t85;
  t13 = x[13] - desired_phi_value * t86;
  t4 = x[12] - desired_theta_value * t87;
  t151 = x[14] - desired_ailerons_value * t89;
  t203 = x[0] - t95;
  t150 = x[1] - t95;
  t136 = x[2] - t95;
  W_act_tilt_el_du = x[3] - t95;
  t204 = x[4] - t93;
  gain_el = x[5] - t93;
  t152 = x[6] - t93;
  t103 = x[7] - t93;
  t53 = x[8] - t94;
  W_act_tilt_az = x[9] - t94;
  W_act_motor_du = x[10] - t94;
  t102 = x[11] - t94;
  fval =
      gamma_quadratic_du2 *
          ((((((((((((((t40 * (K_Cd * K_Cd) + t40 * (Cd_zero * Cd_zero)) +
                       t40 * (t9 * t9)) +
                      t40 * (t66 * t66)) +
                     W_act_phi_du * W_act_phi_du * (t8 * t8)) +
                    W_act_theta_du * W_act_theta_du * (t64 * t64)) +
                   t41 * (t7 * t7)) +
                  t41 * (t6 * t6)) +
                 t41 * (t200 * t200)) +
                t41 * (K_p_T * K_p_T)) +
               W_act_ailerons_du * W_act_ailerons_du * (t5 * t5)) +
              t42 * (flight_path_angle * flight_path_angle)) +
             t42 * (desired_motor_value * desired_motor_value)) +
            t42 * (desired_az_value * desired_az_value)) +
           t42 * (desired_el_value * desired_el_value)) +
      ((((((gamma_quadratic_du *
                ((((((((((((((W_act_phi * W_act_phi * (t13 * t13) +
                              W_act_theta * W_act_theta * (t4 * t4)) +
                             W_act_ailerons * W_act_ailerons * (t151 * t151)) +
                            t28 * (t203 * t203)) +
                           t28 * (t150 * t150)) +
                          t28 * (t136 * t136)) +
                         t28 * (W_act_tilt_el_du * W_act_tilt_el_du)) +
                        t36 * (t204 * t204)) +
                       t36 * (gain_el * gain_el)) +
                      t36 * (t152 * t152)) +
                     t36 * (t103 * t103)) +
                    t37 * (t53 * t53)) +
                   t37 * (W_act_tilt_az * W_act_tilt_az)) +
                  t37 * (W_act_motor_du * W_act_motor_du)) +
                 t37 * (t102 * t102)) +
            W_dv_6 * W_dv_6 * (t63 * t63)) +
           W_dv_4 * W_dv_4 * (t75 * t75)) +
          W_dv_5 * W_dv_5 * (t12 * t12)) +
         W_dv_2 * W_dv_2 * (t11 * t11)) +
        W_dv_1 * W_dv_1 * (Cl_alpha * Cl_alpha)) +
       W_dv_3 * W_dv_3 * (t10 * t10));
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
