/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-Oct-2024 12:53:41
 */

/* Include Files */
#include "compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_V2_NEW_AERO
 *     ACCELERATIONS_ARRAY =
 * COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_V2_NEW_AERO(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1,Omega_2,Omega_3,Omega_4,Omega_1_scaled,Omega_2_scaled,Phi,S,Theta,V,V_scaled,B_1,B_2,B_3,B_4,DELTA_AILERONS,FLIGHT_PATH_ANGLE,G_1,G_2,G_3,G_4,L_1,L_2,L_3,L_4,L_Z,M,P,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_SPAN,WING_CHORD)
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
 *                double Omega_1
 *                double Omega_2
 *                double Omega_3
 *                double Omega_4
 *                double Omega_1_scaled
 *                double Omega_2_scaled
 *                double Phi
 *                double S
 *                double Theta
 *                double V
 *                double V_scaled
 *                double b_1
 *                double b_2
 *                double b_3
 *                double b_4
 *                double delta_ailerons
 *                double flight_path_angle
 *                double g_1
 *                double g_2
 *                double g_3
 *                double g_4
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double m
 *                double p
 *                double prop_R
 *                double prop_Cd_0
 *                double prop_Cl_0
 *                double prop_Cd_a
 *                double prop_Cl_a
 *                double prop_delta
 *                double prop_sigma
 *                double prop_theta
 *                double q
 *                double r
 *                double rho
 *                double wing_span
 *                double wing_chord
 *                double accelerations_array[6]
 * Return Type  : void
 */
void compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero(
    double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
    double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz,
    double K_Cd, double Omega_1, double Omega_2, double Omega_3, double Omega_4,
    double Omega_1_scaled, double Omega_2_scaled, double Phi, double S,
    double Theta, double V, double V_scaled, double b_1, double b_2, double b_3,
    double b_4, double delta_ailerons, double flight_path_angle, double g_1,
    double g_2, double g_3, double g_4, double l_1, double l_2, double l_3,
    double l_4, double l_z, double m, double p, double prop_R, double prop_Cd_0,
    double prop_Cl_0, double prop_Cd_a, double prop_Cl_a, double prop_delta,
    double prop_sigma, double prop_theta, double q, double r, double rho,
    double wing_span, double wing_chord, double accelerations_array[6])
{
  double t10;
  double t11;
  double t117_tmp;
  double t117_tmp_tmp;
  double t12;
  double t13;
  double t14;
  double t143;
  double t15;
  double t16;
  double t169;
  double t17;
  double t170;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t222;
  double t23;
  double t237;
  double t238;
  double t239;
  double t24;
  double t26;
  double t27;
  double t274;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t34;
  double t35;
  double t36;
  double t37;
  double t38;
  double t4;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t49;
  double t5;
  double t52;
  double t54;
  double t55;
  double t56;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t62;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t71;
  double t73;
  double t74;
  double t75;
  double t76;
  double t77;
  double t78;
  double t79;
  double t8;
  double t80;
  double t85;
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t91_tmp;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     11-Oct-2024 01:04:12 */
  t3 = cos(Phi);
  t4 = sin(Beta);
  t5 = cos(Theta);
  t6 = sin(Phi);
  t7 = cos(b_1);
  t8 = cos(b_2);
  t9 = cos(b_3);
  t10 = cos(b_4);
  t11 = sin(Theta);
  t12 = cos(g_1);
  t13 = cos(g_2);
  t14 = cos(g_3);
  t15 = cos(g_4);
  t16 = sin(b_1);
  t17 = sin(b_2);
  t18 = sin(b_3);
  t19 = sin(b_4);
  t20 = sin(g_1);
  t21 = sin(g_2);
  t22 = sin(g_3);
  t23 = sin(g_4);
  t24 = log(prop_delta);
  t26 = Omega_1 * Omega_1;
  t27 = Omega_2 * Omega_2;
  t28 = Omega_3 * Omega_3;
  t29 = Omega_4 * Omega_4;
  t30 = V * V;
  t31 = V_scaled * V_scaled;
  t34 = rt_powd_snf(prop_R, 3.0);
  t35 = rt_powd_snf(prop_R, 4.0);
  t38 = prop_Cd_0 * prop_delta * 2.0;
  t40 = 1.0 / Omega_1;
  t42 = 1.0 / Omega_2;
  t44 = 1.0 / Omega_3;
  t46 = 1.0 / Omega_4;
  t49 = 1.0 / m;
  t52 = prop_delta * 16.0;
  t54 = 1.0 / prop_R;
  t56 = 1.0 / prop_delta;
  t36 = t9 * t9;
  t37 = t10 * t10;
  t41 = 1.0 / t26;
  t43 = 1.0 / t27;
  t45 = 1.0 / t28;
  t47 = 1.0 / t29;
  t55 = t54 * t54;
  t222 = prop_Cl_0 * prop_delta;
  t58 = t222 * (prop_delta + 1.0);
  t59 = Theta - flight_path_angle;
  t66 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t67 = prop_Cl_a * prop_sigma * (prop_delta - 1.0) / 8.0;
  t60 = cos(t59);
  t61 = sin(t59);
  t62 = t58 * 8.0;
  t68 = (b_1 + 1.5707963267948966) + t59;
  t69 = (b_2 + 1.5707963267948966) + t59;
  t70 = (b_3 + 1.5707963267948966) + t59;
  t71 = (b_4 + 1.5707963267948966) + t59;
  t73 = cos(t68);
  t74 = cos(t69);
  t75 = cos(t70);
  t76 = cos(t71);
  t77 = sin(t68);
  t78 = sin(t69);
  t79 = sin(t70);
  t80 = sin(t71);
  t91_tmp = Cl_alpha * S * rho * t30 * t59;
  t85 = t77 * t77;
  t86 = t78 * t78;
  t87 = t79 * t79;
  t88 = t80 * t80;
  t89 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t59 * t59);
  t117_tmp_tmp = S * rho;
  t117_tmp = t117_tmp_tmp * cos(Beta) * t30;
  t143 = t91_tmp * t60 / 2.0 + t117_tmp * t61 * t89 / 2.0;
  t274 = t30 * t41 * t55;
  t68 = V * prop_Cl_a * prop_sigma;
  t69 = prop_Cl_0 * prop_sigma * t24 * t30;
  t70 = prop_sigma * (prop_delta - 1.0) * t56;
  t169 = (t67 + V * t40 * t54 * t73 / 2.0) +
         sqrt(((t274 * (t73 * t73) * 16.0 +
                t68 * t40 * (prop_delta - 1.0) * t54 * t73 * 8.0) +
               t69 * t41 * t55 * t85 * -8.0) -
              t70 * (t62 + prop_Cl_a *
                               (t66 + prop_theta * (t52 + t274 * t85 * 8.0)))) /
             8.0;
  t274 = t30 * t43 * t55;
  t170 = (t67 + V * t42 * t54 * t74 / 2.0) +
         sqrt(((t274 * (t74 * t74) * 16.0 +
                t68 * t42 * (prop_delta - 1.0) * t54 * t74 * 8.0) +
               t69 * t43 * t55 * t86 * -8.0) -
              t70 * (t62 + prop_Cl_a *
                               (t66 + prop_theta * (t52 + t274 * t86 * 8.0)))) /
             8.0;
  t274 = t30 * t45 * t55;
  t71 = (t67 + V * t44 * t54 * t75 / 2.0) +
        sqrt(((t274 * (t75 * t75) * 16.0 +
               t68 * t44 * (prop_delta - 1.0) * t54 * t75 * 8.0) +
              t69 * t45 * t55 * t87 * -8.0) -
             t70 * (t62 + prop_Cl_a *
                              (t66 + prop_theta * (t52 + t274 * t87 * 8.0)))) /
            8.0;
  t274 = t30 * t47 * t55;
  t75 = (t67 + V * t46 * t54 * t76 / 2.0) +
        sqrt(((t274 * (t76 * t76) * 16.0 +
               t68 * t46 * (prop_delta - 1.0) * t54 * t76 * 8.0) +
              t69 * t47 * t55 * t88 * -8.0) -
             t70 * (t62 + prop_Cl_a *
                              (t66 + prop_theta * (t52 + t274 * t88 * 8.0)))) /
            8.0;
  t74 = t222 * t24;
  t274 = prop_Cl_a - prop_Cd_a * 2.0;
  t40 = prop_Cd_a * prop_theta * 2.0;
  t237 = t74 * t169 +
         (prop_delta - 1.0) * (t38 + prop_theta * (t169 * t274 + t40));
  t238 = t74 * t170 +
         (prop_delta - 1.0) * (t38 + prop_theta * (t170 * t274 + t40));
  t239 =
      t74 * t71 + (prop_delta - 1.0) * (t38 + prop_theta * (t71 * t274 + t40));
  t38 =
      t74 * t75 + (prop_delta - 1.0) * (t38 + prop_theta * (t75 * t274 + t40));
  t274 = t74 * t30;
  t40 = prop_Cl_a * prop_theta * t30;
  t73 = prop_Cl_a * prop_delta;
  t42 = t274 * t41 * t55 * t85 +
        (prop_delta - 1.0) *
            ((t58 + t40 * t41 * t55 * t85) - t73 * (-prop_theta + t169) * 2.0);
  t222 = t274 * t43 * t55 * t86 +
         (prop_delta - 1.0) *
             ((t58 + t40 * t43 * t55 * t86) - t73 * (-prop_theta + t170) * 2.0);
  t66 = t274 * t45 * t55 * t87 +
        (prop_delta - 1.0) *
            ((t58 + t40 * t45 * t55 * t87) - t73 * (-prop_theta + t71) * 2.0);
  t71 = t274 * t47 * t55 * t88 +
        (prop_delta - 1.0) *
            ((t58 + t40 * t47 * t55 * t88) - t73 * (-prop_theta + t75) * 2.0);
  t274 = prop_sigma * rho;
  t73 = Omega_1 * V * prop_sigma * rho;
  t67 = t274 * t16 * t26 * t35 * t56 * t42 * 3.1415926535897931 / 4.0 +
        t73 * t7 * t34 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t69 = Omega_2 * V * prop_sigma * rho;
  t62 = t274 * t17 * t27 * t35 * t56 * t222 * 3.1415926535897931 / 4.0 +
        t69 * t8 * t34 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t70 = Omega_3 * V * prop_sigma * rho;
  t76 = t274 * t18 * t28 * t35 * t56 * t66 * 3.1415926535897931 / 4.0 +
        t70 * t9 * t34 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t74 = Omega_4 * V * prop_sigma * rho;
  t24 = t274 * t19 * t29 * t35 * t56 * t71 * 3.1415926535897931 / 4.0 +
        t74 * t10 * t34 * t56 * t80 * t38 * 3.1415926535897931 / 4.0;
  t75 = t274 * t7;
  t54 = t75 * t12 * t26 * t35 * t56 * t42 * 3.1415926535897931 / 4.0 -
        t73 * t12 * t16 * t34 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t68 = t274 * t8;
  t52 = t68 * t13 * t27 * t35 * t56 * t222 * 3.1415926535897931 / 4.0 -
        t69 * t13 * t17 * t34 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t40 = t274 * t9;
  t46 = t40 * t14 * t28 * t35 * t56 * t66 * 3.1415926535897931 / 4.0 -
        t70 * t14 * t18 * t34 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t274 *= t10;
  t44 = t274 * t15 * t29 * t35 * t56 * t71 * 3.1415926535897931 / 4.0 -
        t74 * t15 * t19 * t34 * t56 * t80 * t38 * 3.1415926535897931 / 4.0;
  t42 = t75 * t20 * t26 * t35 * t56 * t42 * 3.1415926535897931 / 4.0 -
        t73 * t16 * t20 * t34 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t75 = t68 * t21 * t27 * t35 * t56 * t222 * 3.1415926535897931 / 4.0 -
        t69 * t17 * t21 * t34 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t73 = t40 * t22 * t28 * t35 * t56 * t66 * 3.1415926535897931 / 4.0 -
        t70 * t18 * t22 * t34 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t70 = t274 * t23 * t29 * t35 * t56 * t71 * 3.1415926535897931 / 4.0 -
        t74 * t19 * t23 * t34 * t56 * t80 * t38 * 3.1415926535897931 / 4.0;
  t71 = ((t67 + t62) + t76) + t24;
  t274 = ((t42 + t75) + t73) + t70;
  t40 = ((t54 + t52) + t46) + t44;
  t74 = t3 * t11;
  t68 = t91_tmp * t61 / 2.0 - t117_tmp * t60 * t89 / 2.0;
  t69 = t117_tmp_tmp * t4;
  accelerations_array[0] =
      t49 *
      (((((t5 * t71 + t5 * t68) - t74 * t143) + t74 * t40) - t6 * t11 * t274) -
       t69 * t6 * t11 * t30 * t89 / 2.0);
  accelerations_array[1] = -t49 * (((-t6 * t143 + t3 * t274) + t6 * t40) +
                                   t117_tmp_tmp * t3 * t4 * t30 * t89 / 2.0);
  t74 = t3 * t5;
  accelerations_array[2] =
      -t49 * (((((t11 * t71 + t11 * t68) + t74 * t143) - t74 * t40) +
               t5 * t6 * t274) +
              t69 * t5 * t6 * t30 * t89 / 2.0) +
      9.81;
  t74 = Omega_1_scaled * t7;
  t68 = Omega_2_scaled * t8;
  t69 = t117_tmp_tmp * t30;
  accelerations_array[3] =
      -(((((((((((l_1 * t54 - l_1 * t52) - l_2 * t46) + l_2 * t44) +
               l_z * t42) +
              l_z * t75) +
             l_z * t73) +
            l_z * t70) -
           I_yy * q * r) +
          I_zz * q * r) -
         CL_aileron * S * delta_ailerons * rho * t30 / 2.0) +
        t69 * wing_span *
            (((((t74 * 0.01228861457458129 - t68 * 0.01228861457458129) +
                Omega_1_scaled * Omega_1_scaled * t7 * 0.0075450647873164947) -
               Omega_2_scaled * Omega_2_scaled * t8 * 0.0075450647873164947) +
              ((((t74 * t31 * -0.02051939481844044 +
                  t68 * t31 * 0.02051939481844044) +
                 Omega_1_scaled * t16 * t37 * 0.0046438440894603552) -
                Omega_2_scaled * t17 * t36 * 0.0046438440894603552) +
               Omega_2_scaled * t18 * t36 * 0.0064371741699132748)) +
             ((Omega_1_scaled * t19 * t37 * -0.0064371741699132748 -
               Omega_1_scaled * (t7 * t7) * t19 * 0.0039345373862287912) +
              Omega_2_scaled * (t8 * t8) * t18 * 0.0039345373862287912)) /
            2.0) /
      I_xx;
  t74 = I_xx * p;
  accelerations_array[4] =
      -((((((((((l_4 * t54 - l_3 * t46) + l_4 * t52) - l_3 * t44) + l_z * t67) +
             l_z * t62) +
            l_z * t76) +
           l_z * t24) +
          t74 * r) -
         I_zz * p * r) -
        t69 * wing_chord * (Cm_zero + Cm_alpha * t59) / 2.0) /
      I_yy;
  accelerations_array[5] =
      (((((((((l_1 * t67 - l_1 * t62) - l_2 * t76) + l_2 * t24) - l_4 * t42) +
           l_3 * t73) -
          l_4 * t75) +
         l_3 * t70) +
        t74 * q) -
       I_yy * p * q) /
      I_zz;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero_terminate(void)
{
}

/*
 * File trailer for compute_acc_nonlinear_control_rf_w_ailerons_v2_new_aero.c
 *
 * [EOF]
 */
