/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_acc_nonlinear_control_rf_w_ailerons.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Mar-2024 00:35:29
 */

/* Include Files */
#include "compute_acc_nonlinear_control_rf_w_ailerons.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double u_in[15]
 *                double p
 *                double q
 *                double r
 *                double K_p_T
 *                double K_p_M
 *                double m
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double Cm_zero
 *                double CL_aileron
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double Beta
 *                double computed_acc[6]
 * Return Type  : void
 */
void c_compute_acc_nonlinear_control(
    const double u_in[15], double p, double q, double r, double K_p_T,
    double K_p_M, double m, double I_xx, double I_yy, double I_zz, double l_1,
    double l_2, double l_3, double l_4, double l_z, double Cl_alpha,
    double Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero,
    double CL_aileron, double rho, double V, double S, double wing_chord,
    double flight_path_angle, double Beta, double computed_acc[6])
{
  double ab_computed_acc_tmp;
  double b_computed_acc_tmp;
  double b_computed_acc_tmp_tmp;
  double bb_computed_acc_tmp;
  double c_computed_acc_tmp;
  double cb_computed_acc_tmp;
  double computed_acc_tmp;
  double computed_acc_tmp_tmp;
  double d_computed_acc_tmp;
  double db_computed_acc_tmp;
  double e_computed_acc_tmp;
  double eb_computed_acc_tmp;
  double f_computed_acc_tmp;
  double fb_computed_acc_tmp;
  double g_computed_acc_tmp;
  double gb_computed_acc_tmp;
  double h_computed_acc_tmp;
  double hb_computed_acc_tmp;
  double i_computed_acc_tmp;
  double ib_computed_acc_tmp;
  double j_computed_acc_tmp;
  double jb_computed_acc_tmp;
  double k_computed_acc_tmp;
  double kb_computed_acc_tmp;
  double l_computed_acc_tmp;
  double lb_computed_acc_tmp;
  double m_computed_acc_tmp;
  double mb_computed_acc_tmp;
  double n_computed_acc_tmp;
  double o_computed_acc_tmp;
  double p_computed_acc_tmp;
  double q_computed_acc_tmp;
  double r_computed_acc_tmp;
  double s_computed_acc_tmp;
  double t_computed_acc_tmp;
  double u_computed_acc_tmp;
  double v_computed_acc_tmp;
  double w_computed_acc_tmp;
  double x_computed_acc_tmp;
  double y_computed_acc_tmp;
  computed_acc_tmp = cos(u_in[12]);
  b_computed_acc_tmp = sin(u_in[12]);
  computed_acc_tmp_tmp = u_in[12] - flight_path_angle;
  c_computed_acc_tmp = sin(computed_acc_tmp_tmp);
  d_computed_acc_tmp = cos(computed_acc_tmp_tmp);
  e_computed_acc_tmp = sin(u_in[13]);
  f_computed_acc_tmp = cos(u_in[13]);
  g_computed_acc_tmp = sin(Beta);
  h_computed_acc_tmp = sin(u_in[4]);
  i_computed_acc_tmp = sin(u_in[5]);
  j_computed_acc_tmp = sin(u_in[6]);
  k_computed_acc_tmp = sin(u_in[7]);
  l_computed_acc_tmp = cos(u_in[4]);
  m_computed_acc_tmp = cos(u_in[8]);
  n_computed_acc_tmp = cos(u_in[5]);
  o_computed_acc_tmp = cos(u_in[9]);
  p_computed_acc_tmp = cos(u_in[6]);
  q_computed_acc_tmp = cos(u_in[10]);
  r_computed_acc_tmp = cos(u_in[7]);
  s_computed_acc_tmp = cos(u_in[11]);
  t_computed_acc_tmp = sin(u_in[8]);
  u_computed_acc_tmp = sin(u_in[9]);
  v_computed_acc_tmp = sin(u_in[10]);
  w_computed_acc_tmp = sin(u_in[11]);
  x_computed_acc_tmp = V * V;
  y_computed_acc_tmp = u_in[0] * u_in[0];
  ab_computed_acc_tmp = u_in[1] * u_in[1];
  bb_computed_acc_tmp = u_in[2] * u_in[2];
  cb_computed_acc_tmp = u_in[3] * u_in[3];
  db_computed_acc_tmp = K_p_T * l_computed_acc_tmp;
  eb_computed_acc_tmp = K_p_T * n_computed_acc_tmp;
  fb_computed_acc_tmp = K_p_T * p_computed_acc_tmp;
  gb_computed_acc_tmp = K_p_T * r_computed_acc_tmp;
  hb_computed_acc_tmp = f_computed_acc_tmp * b_computed_acc_tmp;
  b_computed_acc_tmp_tmp = S * x_computed_acc_tmp * rho;
  ib_computed_acc_tmp = b_computed_acc_tmp_tmp * cos(Beta);
  jb_computed_acc_tmp =
      Cd_zero + Cl_alpha * Cl_alpha * K_Cd *
                    (computed_acc_tmp_tmp * computed_acc_tmp_tmp);
  kb_computed_acc_tmp = Cl_alpha * S * x_computed_acc_tmp * rho;
  lb_computed_acc_tmp =
      ((db_computed_acc_tmp * m_computed_acc_tmp * y_computed_acc_tmp +
        eb_computed_acc_tmp * o_computed_acc_tmp * ab_computed_acc_tmp) +
       fb_computed_acc_tmp * q_computed_acc_tmp * bb_computed_acc_tmp) +
      gb_computed_acc_tmp * s_computed_acc_tmp * cb_computed_acc_tmp;
  mb_computed_acc_tmp =
      ib_computed_acc_tmp * c_computed_acc_tmp * jb_computed_acc_tmp / 2.0 +
      kb_computed_acc_tmp * d_computed_acc_tmp * computed_acc_tmp_tmp / 2.0;
  db_computed_acc_tmp =
      ((db_computed_acc_tmp * t_computed_acc_tmp * y_computed_acc_tmp +
        eb_computed_acc_tmp * u_computed_acc_tmp * ab_computed_acc_tmp) +
       fb_computed_acc_tmp * v_computed_acc_tmp * bb_computed_acc_tmp) +
      gb_computed_acc_tmp * w_computed_acc_tmp * cb_computed_acc_tmp;
  c_computed_acc_tmp =
      ib_computed_acc_tmp * d_computed_acc_tmp * jb_computed_acc_tmp / 2.0 -
      kb_computed_acc_tmp * c_computed_acc_tmp * computed_acc_tmp_tmp / 2.0;
  d_computed_acc_tmp = ((K_p_T * h_computed_acc_tmp * y_computed_acc_tmp +
                         K_p_T * i_computed_acc_tmp * ab_computed_acc_tmp) +
                        K_p_T * j_computed_acc_tmp * bb_computed_acc_tmp) +
                       K_p_T * k_computed_acc_tmp * cb_computed_acc_tmp;
  eb_computed_acc_tmp = b_computed_acc_tmp_tmp * g_computed_acc_tmp;
  computed_acc[0] =
      -(((((computed_acc_tmp * c_computed_acc_tmp +
            computed_acc_tmp * d_computed_acc_tmp) -
           e_computed_acc_tmp * b_computed_acc_tmp * db_computed_acc_tmp) +
          hb_computed_acc_tmp * lb_computed_acc_tmp) +
         hb_computed_acc_tmp * mb_computed_acc_tmp) +
        eb_computed_acc_tmp * e_computed_acc_tmp * b_computed_acc_tmp *
            jb_computed_acc_tmp / 2.0) /
      m;
  computed_acc[1] = (((e_computed_acc_tmp * lb_computed_acc_tmp +
                       e_computed_acc_tmp * mb_computed_acc_tmp) +
                      f_computed_acc_tmp * db_computed_acc_tmp) -
                     b_computed_acc_tmp_tmp * f_computed_acc_tmp *
                         g_computed_acc_tmp * jb_computed_acc_tmp / 2.0) /
                    m;
  f_computed_acc_tmp *= computed_acc_tmp;
  computed_acc[2] =
      (((((b_computed_acc_tmp * c_computed_acc_tmp +
           b_computed_acc_tmp * d_computed_acc_tmp) +
          computed_acc_tmp * e_computed_acc_tmp * db_computed_acc_tmp) -
         f_computed_acc_tmp * lb_computed_acc_tmp) -
        f_computed_acc_tmp * mb_computed_acc_tmp) -
       eb_computed_acc_tmp * computed_acc_tmp * e_computed_acc_tmp *
           jb_computed_acc_tmp / 2.0) /
          m +
      9.81;
  computed_acc_tmp = K_p_T * y_computed_acc_tmp;
  b_computed_acc_tmp = K_p_T * ab_computed_acc_tmp;
  c_computed_acc_tmp = K_p_T * bb_computed_acc_tmp;
  d_computed_acc_tmp = K_p_T * cb_computed_acc_tmp;
  e_computed_acc_tmp = computed_acc_tmp * l_z;
  f_computed_acc_tmp = b_computed_acc_tmp * l_z;
  g_computed_acc_tmp = c_computed_acc_tmp * l_z;
  db_computed_acc_tmp = d_computed_acc_tmp * l_z;
  y_computed_acc_tmp *= K_p_M;
  ab_computed_acc_tmp *= K_p_M;
  bb_computed_acc_tmp *= K_p_M;
  cb_computed_acc_tmp *= K_p_M;
  eb_computed_acc_tmp = computed_acc_tmp * l_1;
  fb_computed_acc_tmp = b_computed_acc_tmp * l_1;
  gb_computed_acc_tmp = c_computed_acc_tmp * l_2;
  hb_computed_acc_tmp = d_computed_acc_tmp * l_2;
  computed_acc[3] =
      ((((((((((((((y_computed_acc_tmp * h_computed_acc_tmp -
                    ab_computed_acc_tmp * i_computed_acc_tmp) +
                   bb_computed_acc_tmp * j_computed_acc_tmp) -
                  cb_computed_acc_tmp * k_computed_acc_tmp) +
                 I_yy * q * r) -
                I_zz * q * r) +
               eb_computed_acc_tmp * l_computed_acc_tmp * m_computed_acc_tmp) -
              fb_computed_acc_tmp * n_computed_acc_tmp * o_computed_acc_tmp) -
             gb_computed_acc_tmp * p_computed_acc_tmp * q_computed_acc_tmp) +
            hb_computed_acc_tmp * r_computed_acc_tmp * s_computed_acc_tmp) +
           e_computed_acc_tmp * l_computed_acc_tmp * t_computed_acc_tmp) +
          f_computed_acc_tmp * n_computed_acc_tmp * u_computed_acc_tmp) +
         g_computed_acc_tmp * p_computed_acc_tmp * v_computed_acc_tmp) +
        db_computed_acc_tmp * r_computed_acc_tmp * w_computed_acc_tmp) +
       CL_aileron * S * x_computed_acc_tmp * u_in[14] * rho / 2.0) /
      I_xx;
  x_computed_acc_tmp = I_xx * p;
  y_computed_acc_tmp *= l_computed_acc_tmp;
  ab_computed_acc_tmp *= n_computed_acc_tmp;
  bb_computed_acc_tmp *= p_computed_acc_tmp;
  cb_computed_acc_tmp *= r_computed_acc_tmp;
  computed_acc_tmp = computed_acc_tmp * l_4 * l_computed_acc_tmp;
  b_computed_acc_tmp = b_computed_acc_tmp * l_4 * n_computed_acc_tmp;
  c_computed_acc_tmp = c_computed_acc_tmp * l_3 * p_computed_acc_tmp;
  d_computed_acc_tmp = d_computed_acc_tmp * l_3 * r_computed_acc_tmp;
  computed_acc[4] =
      ((((((((((((((I_zz * p * r - x_computed_acc_tmp * r) +
                   e_computed_acc_tmp * h_computed_acc_tmp) +
                  f_computed_acc_tmp * i_computed_acc_tmp) +
                 g_computed_acc_tmp * j_computed_acc_tmp) +
                db_computed_acc_tmp * k_computed_acc_tmp) -
               y_computed_acc_tmp * t_computed_acc_tmp) +
              ab_computed_acc_tmp * u_computed_acc_tmp) -
             bb_computed_acc_tmp * v_computed_acc_tmp) +
            cb_computed_acc_tmp * w_computed_acc_tmp) +
           b_computed_acc_tmp_tmp * wing_chord *
               (Cm_zero + Cm_alpha * computed_acc_tmp_tmp) / 2.0) +
          computed_acc_tmp * m_computed_acc_tmp) +
         b_computed_acc_tmp * o_computed_acc_tmp) -
        c_computed_acc_tmp * q_computed_acc_tmp) -
       d_computed_acc_tmp * s_computed_acc_tmp) /
      I_yy;
  computed_acc[5] = (((((((((((((x_computed_acc_tmp * q - I_yy * p * q) -
                                eb_computed_acc_tmp * h_computed_acc_tmp) +
                               fb_computed_acc_tmp * i_computed_acc_tmp) +
                              gb_computed_acc_tmp * j_computed_acc_tmp) -
                             hb_computed_acc_tmp * k_computed_acc_tmp) +
                            y_computed_acc_tmp * m_computed_acc_tmp) -
                           ab_computed_acc_tmp * o_computed_acc_tmp) +
                          bb_computed_acc_tmp * q_computed_acc_tmp) -
                         cb_computed_acc_tmp * s_computed_acc_tmp) +
                        computed_acc_tmp * t_computed_acc_tmp) +
                       b_computed_acc_tmp * u_computed_acc_tmp) -
                      c_computed_acc_tmp * v_computed_acc_tmp) -
                     d_computed_acc_tmp * w_computed_acc_tmp) /
                    I_zz;
}

/*
 * File trailer for compute_acc_nonlinear_control_rf_w_ailerons.c
 *
 * [EOF]
 */
