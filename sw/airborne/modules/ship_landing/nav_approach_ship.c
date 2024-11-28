/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nav_approach_ship.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Nov-2024 21:06:40
 */

/* Include Files */
#include "nav_approach_ship.h"
#include <math.h>

/* Function Declarations */
static double b_norm(const double x[3]);

static void c_optimal_linear_coeff_calculat(
    double Ax_0_UAV, double Ay_0_UAV, double Az_0_UAV, double Psi_ship_rad,
    double Vx_0_UAV, double Vy_0_UAV, double Vz_0_UAV, double X_0_UAV,
    double X_0_SHIP, double Y_0_UAV, double Y_0_SHIP, double Z_0_UAV,
    double Z_0_SHIP, double coeff_1_ship_prediction,
    double coeff_2_ship_prediction, double coeff_3_ship_prediction,
    double coeff_4_ship_prediction, double coeff_5_ship_prediction,
    double coeff_6_ship_prediction, double coeff_7_ship_prediction,
    double coeff_8_ship_prediction, double coeff_9_ship_prediction,
    double coeff_10_ship_prediction, double coeff_11_ship_prediction,
    double coeff_12_ship_prediction, double coeff_13_ship_prediction,
    double coeff_14_ship_prediction, double coeff_15_ship_prediction,
    double coeff_16_ship_prediction, double coeff_17_ship_prediction,
    double coeff_18_ship_prediction, double coeff_19_ship_prediction,
    double coeff_20_ship_prediction, double coeff_21_ship_prediction,
    double coeff_22_ship_prediction, double coeff_23_ship_prediction,
    double coeff_24_ship_prediction, double t_delay_ship_prediction,
    double t_landing, double coeffs_optimized[18]);

static double c_optimal_linear_coeff_fcn_calc(
    const double coeffs_ship_prediction[24], const double P0_NED[3],
    const double V0_NED[3], const double A0_NED[3], double Psi_ship_rad,
    const double P0_SHIP_NED[3], const double v_max_control_rf[3],
    const double v_min_control_rf[3], const double a_max_control_rf[3],
    const double a_min_control_rf[3], double max_time_of_landing_seconds,
    double t_delay_ship_prediction, double optimal_coeffs[18], double *exitflag,
    double V_OOB[6], double A_OOB[6]);

static void
evaluate_UAV_path_from_coeffs(const double coeffs[18], double landing_time,
                              double UAV_path[60], double UAV_path_speed[60],
                              double UAV_path_acc[60], double t_array[20]);

/* Function Definitions */
/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
static double b_norm(const double x[3])
{
  double absxk;
  double scale;
  double t;
  double y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  return scale * sqrt(y);
}

/*
 * OPTIMAL_LINEAR_COEFF_CALCULATION_W_DELAY_FROM_SPEED
 *     COEFFS_OPTIMIZED =
 * OPTIMAL_LINEAR_COEFF_CALCULATION_W_DELAY_FROM_SPEED(Ax_0_UAV,Ay_0_UAV,Az_0_UAV,Psi_ship_rad,Vx_0_UAV,Vy_0_UAV,Vz_0_UAV,X_0_UAV,X_0_SHIP,Y_0_UAV,Y_0_SHIP,Z_0_UAV,Z_0_SHIP,COEFF_1_SHIP_PREDICTION,COEFF_2_SHIP_PREDICTION,COEFF_3_SHIP_PREDICTION,COEFF_4_SHIP_PREDICTION,COEFF_5_SHIP_PREDICTION,COEFF_6_SHIP_PREDICTION,COEFF_7_SHIP_PREDICTION,COEFF_8_SHIP_PREDICTION,COEFF_9_SHIP_PREDICTION,COEFF_10_SHIP_PREDICTION,COEFF_11_SHIP_PREDICTION,COEFF_12_SHIP_PREDICTION,COEFF_13_SHIP_PREDICTION,COEFF_14_SHIP_PREDICTION,COEFF_15_SHIP_PREDICTION,COEFF_16_SHIP_PREDICTION,COEFF_17_SHIP_PREDICTION,COEFF_18_SHIP_PREDICTION,COEFF_19_SHIP_PREDICTION,COEFF_20_SHIP_PREDICTION,COEFF_21_SHIP_PREDICTION,COEFF_22_SHIP_PREDICTION,COEFF_23_SHIP_PREDICTION,COEFF_24_SHIP_PREDICTION,T_DELAY_SHIP_PREDICTION,T_LANDING)
 *
 * Arguments    : double Ax_0_UAV
 *                double Ay_0_UAV
 *                double Az_0_UAV
 *                double Psi_ship_rad
 *                double Vx_0_UAV
 *                double Vy_0_UAV
 *                double Vz_0_UAV
 *                double X_0_UAV
 *                double X_0_SHIP
 *                double Y_0_UAV
 *                double Y_0_SHIP
 *                double Z_0_UAV
 *                double Z_0_SHIP
 *                double coeff_1_ship_prediction
 *                double coeff_2_ship_prediction
 *                double coeff_3_ship_prediction
 *                double coeff_4_ship_prediction
 *                double coeff_5_ship_prediction
 *                double coeff_6_ship_prediction
 *                double coeff_7_ship_prediction
 *                double coeff_8_ship_prediction
 *                double coeff_9_ship_prediction
 *                double coeff_10_ship_prediction
 *                double coeff_11_ship_prediction
 *                double coeff_12_ship_prediction
 *                double coeff_13_ship_prediction
 *                double coeff_14_ship_prediction
 *                double coeff_15_ship_prediction
 *                double coeff_16_ship_prediction
 *                double coeff_17_ship_prediction
 *                double coeff_18_ship_prediction
 *                double coeff_19_ship_prediction
 *                double coeff_20_ship_prediction
 *                double coeff_21_ship_prediction
 *                double coeff_22_ship_prediction
 *                double coeff_23_ship_prediction
 *                double coeff_24_ship_prediction
 *                double t_delay_ship_prediction
 *                double t_landing
 *                double coeffs_optimized[18]
 * Return Type  : void
 */
static void c_optimal_linear_coeff_calculat(
    double Ax_0_UAV, double Ay_0_UAV, double Az_0_UAV, double Psi_ship_rad,
    double Vx_0_UAV, double Vy_0_UAV, double Vz_0_UAV, double X_0_UAV,
    double X_0_SHIP, double Y_0_UAV, double Y_0_SHIP, double Z_0_UAV,
    double Z_0_SHIP, double coeff_1_ship_prediction,
    double coeff_2_ship_prediction, double coeff_3_ship_prediction,
    double coeff_4_ship_prediction, double coeff_5_ship_prediction,
    double coeff_6_ship_prediction, double coeff_7_ship_prediction,
    double coeff_8_ship_prediction, double coeff_9_ship_prediction,
    double coeff_10_ship_prediction, double coeff_11_ship_prediction,
    double coeff_12_ship_prediction, double coeff_13_ship_prediction,
    double coeff_14_ship_prediction, double coeff_15_ship_prediction,
    double coeff_16_ship_prediction, double coeff_17_ship_prediction,
    double coeff_18_ship_prediction, double coeff_19_ship_prediction,
    double coeff_20_ship_prediction, double coeff_21_ship_prediction,
    double coeff_22_ship_prediction, double coeff_23_ship_prediction,
    double coeff_24_ship_prediction, double t_delay_ship_prediction,
    double t_landing, double coeffs_optimized[18])
{
  double ab_coeffs_optimized_tmp;
  double ac_coeffs_optimized_tmp;
  double b_coeffs_optimized_tmp;
  double bb_coeffs_optimized_tmp;
  double bc_coeffs_optimized_tmp;
  double c_coeffs_optimized_tmp;
  double cb_coeffs_optimized_tmp;
  double cc_coeffs_optimized_tmp;
  double coeffs_optimized_tmp;
  double d_coeffs_optimized_tmp;
  double db_coeffs_optimized_tmp;
  double dc_coeffs_optimized_tmp;
  double e_coeffs_optimized_tmp;
  double eb_coeffs_optimized_tmp;
  double ec_coeffs_optimized_tmp;
  double f_coeffs_optimized_tmp;
  double fb_coeffs_optimized_tmp;
  double fc_coeffs_optimized_tmp;
  double g_coeffs_optimized_tmp;
  double gb_coeffs_optimized_tmp;
  double gc_coeffs_optimized_tmp;
  double h_coeffs_optimized_tmp;
  double hb_coeffs_optimized_tmp;
  double i_coeffs_optimized_tmp;
  double ib_coeffs_optimized_tmp;
  double j_coeffs_optimized_tmp;
  double jb_coeffs_optimized_tmp;
  double k_coeffs_optimized_tmp;
  double kb_coeffs_optimized_tmp;
  double l_coeffs_optimized_tmp;
  double lb_coeffs_optimized_tmp;
  double m_coeffs_optimized_tmp;
  double mb_coeffs_optimized_tmp;
  double n_coeffs_optimized_tmp;
  double nb_coeffs_optimized_tmp;
  double o_coeffs_optimized_tmp;
  double ob_coeffs_optimized_tmp;
  double p_coeffs_optimized_tmp;
  double pb_coeffs_optimized_tmp;
  double q_coeffs_optimized_tmp;
  double qb_coeffs_optimized_tmp;
  double r_coeffs_optimized_tmp;
  double rb_coeffs_optimized_tmp;
  double s_coeffs_optimized_tmp;
  double sb_coeffs_optimized_tmp;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  double t_coeffs_optimized_tmp;
  double tb_coeffs_optimized_tmp;
  double u_coeffs_optimized_tmp;
  double ub_coeffs_optimized_tmp;
  double v_coeffs_optimized_tmp;
  double vb_coeffs_optimized_tmp;
  double w_coeffs_optimized_tmp;
  double wb_coeffs_optimized_tmp;
  double x_coeffs_optimized_tmp;
  double xb_coeffs_optimized_tmp;
  double y_coeffs_optimized_tmp;
  double yb_coeffs_optimized_tmp;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     09-Sep-2024 13:07:23 */
  t2 = cos(Psi_ship_rad);
  t3 = sin(Psi_ship_rad);
  t4 = t_delay_ship_prediction * t_delay_ship_prediction;
  t5 = pow(t_delay_ship_prediction, 3.0);
  t7 = pow(t_delay_ship_prediction, 5.0);
  t9 = pow(t_delay_ship_prediction, 7.0);
  t10 = t_landing * t_landing;
  t11 = pow(t_landing, 3.0);
  t13 = pow(t_landing, 5.0);
  t15 = pow(t_landing, 7.0);
  t6 = t4 * t4;
  t8 = pow(t4, 3.0);
  t12 = t10 * t10;
  t14 = pow(t10, 3.0);
  t16 = 1.0 / t11;
  t18 = 1.0 / t13;
  t17 = 1.0 / t12;
  t19 = 1.0 / (t17 * t17);
  coeffs_optimized_tmp = coeff_1_ship_prediction * t2;
  b_coeffs_optimized_tmp = coeff_2_ship_prediction * t2;
  c_coeffs_optimized_tmp = coeff_3_ship_prediction * t2;
  d_coeffs_optimized_tmp = coeff_4_ship_prediction * t2;
  e_coeffs_optimized_tmp = coeff_9_ship_prediction * t3;
  f_coeffs_optimized_tmp = coeff_10_ship_prediction * t3;
  g_coeffs_optimized_tmp = coeff_11_ship_prediction * t3;
  h_coeffs_optimized_tmp = coeff_12_ship_prediction * t3;
  i_coeffs_optimized_tmp = coeff_5_ship_prediction * t2;
  j_coeffs_optimized_tmp = coeff_6_ship_prediction * t2;
  k_coeffs_optimized_tmp = coeff_13_ship_prediction * t3;
  l_coeffs_optimized_tmp = coeff_14_ship_prediction * t3;
  m_coeffs_optimized_tmp = coeff_7_ship_prediction * t2;
  n_coeffs_optimized_tmp = coeff_15_ship_prediction * t3;
  o_coeffs_optimized_tmp = X_0_UAV * 840.0 - X_0_SHIP * 840.0;
  p_coeffs_optimized_tmp = Ax_0_UAV * t10;
  q_coeffs_optimized_tmp = Vx_0_UAV * t_landing;
  r_coeffs_optimized_tmp = b_coeffs_optimized_tmp * t15;
  s_coeffs_optimized_tmp = c_coeffs_optimized_tmp * t14;
  t_coeffs_optimized_tmp = m_coeffs_optimized_tmp * t10;
  u_coeffs_optimized_tmp = coeffs_optimized_tmp * t19;
  v_coeffs_optimized_tmp = f_coeffs_optimized_tmp * t15;
  w_coeffs_optimized_tmp = g_coeffs_optimized_tmp * t14;
  x_coeffs_optimized_tmp = n_coeffs_optimized_tmp * t10;
  y_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t19;
  ab_coeffs_optimized_tmp = coeff_8_ship_prediction * t2 * t_landing;
  bb_coeffs_optimized_tmp = coeff_16_ship_prediction * t3 * t_landing;
  cb_coeffs_optimized_tmp = coeffs_optimized_tmp * t4 * t14;
  db_coeffs_optimized_tmp = coeffs_optimized_tmp * t8 * t10;
  eb_coeffs_optimized_tmp = b_coeffs_optimized_tmp * t7 * t10;
  fb_coeffs_optimized_tmp = c_coeffs_optimized_tmp * t6 * t10;
  gb_coeffs_optimized_tmp = d_coeffs_optimized_tmp * t5 * t10;
  hb_coeffs_optimized_tmp = i_coeffs_optimized_tmp * t4 * t10;
  ib_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t4 * t14;
  jb_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t8 * t10;
  kb_coeffs_optimized_tmp = f_coeffs_optimized_tmp * t7 * t10;
  lb_coeffs_optimized_tmp = g_coeffs_optimized_tmp * t6 * t10;
  mb_coeffs_optimized_tmp = h_coeffs_optimized_tmp * t5 * t10;
  nb_coeffs_optimized_tmp = k_coeffs_optimized_tmp * t4 * t10;
  ob_coeffs_optimized_tmp =
      coeffs_optimized_tmp * t15 * t_delay_ship_prediction;
  pb_coeffs_optimized_tmp =
      b_coeffs_optimized_tmp * t14 * t_delay_ship_prediction;
  qb_coeffs_optimized_tmp =
      j_coeffs_optimized_tmp * t10 * t_delay_ship_prediction;
  rb_coeffs_optimized_tmp =
      e_coeffs_optimized_tmp * t15 * t_delay_ship_prediction;
  sb_coeffs_optimized_tmp =
      f_coeffs_optimized_tmp * t14 * t_delay_ship_prediction;
  tb_coeffs_optimized_tmp =
      l_coeffs_optimized_tmp * t10 * t_delay_ship_prediction;
  ub_coeffs_optimized_tmp = coeffs_optimized_tmp * t9 * t_landing;
  vb_coeffs_optimized_tmp = b_coeffs_optimized_tmp * t8 * t_landing;
  wb_coeffs_optimized_tmp = c_coeffs_optimized_tmp * t7 * t_landing;
  xb_coeffs_optimized_tmp = d_coeffs_optimized_tmp * t6 * t_landing;
  yb_coeffs_optimized_tmp = i_coeffs_optimized_tmp * t5 * t_landing;
  ac_coeffs_optimized_tmp = j_coeffs_optimized_tmp * t4 * t_landing;
  bc_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t9 * t_landing;
  cc_coeffs_optimized_tmp = f_coeffs_optimized_tmp * t8 * t_landing;
  dc_coeffs_optimized_tmp = g_coeffs_optimized_tmp * t7 * t_landing;
  ec_coeffs_optimized_tmp = h_coeffs_optimized_tmp * t6 * t_landing;
  fc_coeffs_optimized_tmp = k_coeffs_optimized_tmp * t5 * t_landing;
  gc_coeffs_optimized_tmp = l_coeffs_optimized_tmp * t4 * t_landing;
  m_coeffs_optimized_tmp =
      m_coeffs_optimized_tmp * t_delay_ship_prediction * t_landing;
  n_coeffs_optimized_tmp =
      n_coeffs_optimized_tmp * t_delay_ship_prediction * t_landing;
  coeffs_optimized[0] =
      t18 *
      ((((((((((((((((((((((((((o_coeffs_optimized_tmp +
                                p_coeffs_optimized_tmp * 70.0) +
                               q_coeffs_optimized_tmp * 420.0) -
                              r_coeffs_optimized_tmp * 120.0) -
                             s_coeffs_optimized_tmp * 70.0) -
                            d_coeffs_optimized_tmp * t13 * 28.0) -
                           t_coeffs_optimized_tmp * 70.0) -
                          u_coeffs_optimized_tmp * 175.0) +
                         v_coeffs_optimized_tmp * 120.0) +
                        w_coeffs_optimized_tmp * 70.0) +
                       h_coeffs_optimized_tmp * t13 * 28.0) +
                      x_coeffs_optimized_tmp * 70.0) +
                     y_coeffs_optimized_tmp * 175.0) -
                    ab_coeffs_optimized_tmp * 420.0) +
                   bb_coeffs_optimized_tmp * 420.0) -
                  cb_coeffs_optimized_tmp * 1470.0) -
                 coeffs_optimized_tmp * t5 * t13 * 980.0) -
                db_coeffs_optimized_tmp * 490.0) -
               b_coeffs_optimized_tmp * t4 * t13 * 420.0) -
              eb_coeffs_optimized_tmp * 420.0) -
             fb_coeffs_optimized_tmp * 350.0) -
            gb_coeffs_optimized_tmp * 280.0) -
           hb_coeffs_optimized_tmp * 210.0) +
          ib_coeffs_optimized_tmp * 1470.0) +
         e_coeffs_optimized_tmp * t5 * t13 * 980.0) +
        ((((((((((((((((((((((jb_coeffs_optimized_tmp * 490.0 +
                              f_coeffs_optimized_tmp * t4 * t13 * 420.0) +
                             kb_coeffs_optimized_tmp * 420.0) +
                            lb_coeffs_optimized_tmp * 350.0) +
                           mb_coeffs_optimized_tmp * 280.0) +
                          nb_coeffs_optimized_tmp * 210.0) -
                         ob_coeffs_optimized_tmp * 840.0) -
                        pb_coeffs_optimized_tmp * 420.0) -
                       c_coeffs_optimized_tmp * t13 * t_delay_ship_prediction *
                           140.0) -
                      qb_coeffs_optimized_tmp * 140.0) +
                     rb_coeffs_optimized_tmp * 840.0) +
                    sb_coeffs_optimized_tmp * 420.0) +
                   g_coeffs_optimized_tmp * t13 * t_delay_ship_prediction *
                       140.0) +
                  tb_coeffs_optimized_tmp * 140.0) -
                 ub_coeffs_optimized_tmp * 420.0) -
                vb_coeffs_optimized_tmp * 420.0) -
               wb_coeffs_optimized_tmp * 420.0) -
              xb_coeffs_optimized_tmp * 420.0) -
             yb_coeffs_optimized_tmp * 420.0) -
            ac_coeffs_optimized_tmp * 420.0) +
           bc_coeffs_optimized_tmp * 420.0) +
          cc_coeffs_optimized_tmp * 420.0) +
         dc_coeffs_optimized_tmp * 420.0)) +
       ((((ec_coeffs_optimized_tmp * 420.0 + fc_coeffs_optimized_tmp * 420.0) +
          gc_coeffs_optimized_tmp * 420.0) -
         m_coeffs_optimized_tmp * 420.0) +
        n_coeffs_optimized_tmp * 420.0)) *
      -0.0071428571428571426;
  coeffs_optimized[1] =
      t17 *
      (((((((((((((((((((((((((((o_coeffs_optimized_tmp +
                                 p_coeffs_optimized_tmp * 84.0) +
                                q_coeffs_optimized_tmp * 448.0) -
                               r_coeffs_optimized_tmp * 64.0) -
                              s_coeffs_optimized_tmp * 28.0) +
                             i_coeffs_optimized_tmp * t12 * 14.0) -
                            t_coeffs_optimized_tmp * 84.0) -
                           u_coeffs_optimized_tmp * 105.0) +
                          v_coeffs_optimized_tmp * 64.0) +
                         w_coeffs_optimized_tmp * 28.0) -
                        k_coeffs_optimized_tmp * t12 * 14.0) +
                       x_coeffs_optimized_tmp * 84.0) +
                      y_coeffs_optimized_tmp * 105.0) -
                     ab_coeffs_optimized_tmp * 448.0) +
                    bb_coeffs_optimized_tmp * 448.0) -
                   cb_coeffs_optimized_tmp * 588.0) +
                  coeffs_optimized_tmp * t6 * t12 * 490.0) -
                 db_coeffs_optimized_tmp * 588.0) +
                b_coeffs_optimized_tmp * t5 * t12 * 280.0) -
               eb_coeffs_optimized_tmp * 504.0) +
              c_coeffs_optimized_tmp * t4 * t12 * 140.0) -
             fb_coeffs_optimized_tmp * 420.0) -
            gb_coeffs_optimized_tmp * 336.0) -
           hb_coeffs_optimized_tmp * 252.0) +
          ib_coeffs_optimized_tmp * 588.0) -
         e_coeffs_optimized_tmp * t6 * t12 * 490.0) +
        ((((((((((((((((((((((jb_coeffs_optimized_tmp * 588.0 -
                              f_coeffs_optimized_tmp * t5 * t12 * 280.0) +
                             kb_coeffs_optimized_tmp * 504.0) -
                            g_coeffs_optimized_tmp * t4 * t12 * 140.0) +
                           lb_coeffs_optimized_tmp * 420.0) +
                          mb_coeffs_optimized_tmp * 336.0) +
                         nb_coeffs_optimized_tmp * 252.0) -
                        ob_coeffs_optimized_tmp * 448.0) -
                       pb_coeffs_optimized_tmp * 168.0) +
                      d_coeffs_optimized_tmp * t12 * t_delay_ship_prediction *
                          56.0) -
                     qb_coeffs_optimized_tmp * 168.0) +
                    rb_coeffs_optimized_tmp * 448.0) +
                   sb_coeffs_optimized_tmp * 168.0) -
                  h_coeffs_optimized_tmp * t12 * t_delay_ship_prediction *
                      56.0) +
                 tb_coeffs_optimized_tmp * 168.0) -
                ub_coeffs_optimized_tmp * 448.0) -
               vb_coeffs_optimized_tmp * 448.0) -
              wb_coeffs_optimized_tmp * 448.0) -
             xb_coeffs_optimized_tmp * 448.0) -
            yb_coeffs_optimized_tmp * 448.0) -
           ac_coeffs_optimized_tmp * 448.0) +
          bc_coeffs_optimized_tmp * 448.0) +
         cc_coeffs_optimized_tmp * 448.0)) +
       (((((dc_coeffs_optimized_tmp * 448.0 + ec_coeffs_optimized_tmp * 448.0) +
           fc_coeffs_optimized_tmp * 448.0) +
          gc_coeffs_optimized_tmp * 448.0) -
         m_coeffs_optimized_tmp * 448.0) +
        n_coeffs_optimized_tmp * 448.0)) /
      56.0;
  coeffs_optimized[2] =
      t16 *
      (((((((((((((((((((((((((((o_coeffs_optimized_tmp +
                                 p_coeffs_optimized_tmp * 126.0) +
                                q_coeffs_optimized_tmp * 504.0) -
                               r_coeffs_optimized_tmp * 36.0) -
                              s_coeffs_optimized_tmp * 14.0) -
                             j_coeffs_optimized_tmp * t11 * 28.0) -
                            t_coeffs_optimized_tmp * 126.0) -
                           u_coeffs_optimized_tmp * 63.0) +
                          v_coeffs_optimized_tmp * 36.0) +
                         w_coeffs_optimized_tmp * 14.0) +
                        l_coeffs_optimized_tmp * t11 * 28.0) +
                       x_coeffs_optimized_tmp * 126.0) +
                      y_coeffs_optimized_tmp * 63.0) -
                     ab_coeffs_optimized_tmp * 504.0) +
                    bb_coeffs_optimized_tmp * 504.0) -
                   cb_coeffs_optimized_tmp * 294.0) -
                  coeffs_optimized_tmp * t7 * t11 * 588.0) -
                 db_coeffs_optimized_tmp * 882.0) -
                b_coeffs_optimized_tmp * t6 * t11 * 420.0) -
               eb_coeffs_optimized_tmp * 756.0) -
              c_coeffs_optimized_tmp * t5 * t11 * 280.0) -
             fb_coeffs_optimized_tmp * 630.0) -
            d_coeffs_optimized_tmp * t4 * t11 * 168.0) -
           gb_coeffs_optimized_tmp * 504.0) -
          hb_coeffs_optimized_tmp * 378.0) +
         ib_coeffs_optimized_tmp * 294.0) +
        ((((((((((((((((((((((e_coeffs_optimized_tmp * t7 * t11 * 588.0 +
                              jb_coeffs_optimized_tmp * 882.0) +
                             f_coeffs_optimized_tmp * t6 * t11 * 420.0) +
                            kb_coeffs_optimized_tmp * 756.0) +
                           g_coeffs_optimized_tmp * t5 * t11 * 280.0) +
                          lb_coeffs_optimized_tmp * 630.0) +
                         h_coeffs_optimized_tmp * t4 * t11 * 168.0) +
                        mb_coeffs_optimized_tmp * 504.0) +
                       nb_coeffs_optimized_tmp * 378.0) -
                      ob_coeffs_optimized_tmp * 252.0) -
                     pb_coeffs_optimized_tmp * 84.0) -
                    i_coeffs_optimized_tmp * t11 * t_delay_ship_prediction *
                        84.0) -
                   qb_coeffs_optimized_tmp * 252.0) +
                  rb_coeffs_optimized_tmp * 252.0) +
                 sb_coeffs_optimized_tmp * 84.0) +
                k_coeffs_optimized_tmp * t11 * t_delay_ship_prediction * 84.0) +
               tb_coeffs_optimized_tmp * 252.0) -
              ub_coeffs_optimized_tmp * 504.0) -
             vb_coeffs_optimized_tmp * 504.0) -
            wb_coeffs_optimized_tmp * 504.0) -
           xb_coeffs_optimized_tmp * 504.0) -
          yb_coeffs_optimized_tmp * 504.0) -
         ac_coeffs_optimized_tmp * 504.0)) +
       (((((((bc_coeffs_optimized_tmp * 504.0 +
              cc_coeffs_optimized_tmp * 504.0) +
             dc_coeffs_optimized_tmp * 504.0) +
            ec_coeffs_optimized_tmp * 504.0) +
           fc_coeffs_optimized_tmp * 504.0) +
          gc_coeffs_optimized_tmp * 504.0) -
         m_coeffs_optimized_tmp * 504.0) +
        n_coeffs_optimized_tmp * 504.0)) *
      -0.011904761904761904;
  coeffs_optimized[3] = Ax_0_UAV / 2.0;
  coeffs_optimized[4] = Vx_0_UAV;
  coeffs_optimized[5] = X_0_UAV;
  coeffs_optimized_tmp = coeff_1_ship_prediction * t3;
  b_coeffs_optimized_tmp = coeff_2_ship_prediction * t3;
  c_coeffs_optimized_tmp = coeff_3_ship_prediction * t3;
  d_coeffs_optimized_tmp = coeff_4_ship_prediction * t3;
  e_coeffs_optimized_tmp = coeff_9_ship_prediction * t2;
  f_coeffs_optimized_tmp = coeff_10_ship_prediction * t2;
  g_coeffs_optimized_tmp = coeff_11_ship_prediction * t2;
  h_coeffs_optimized_tmp = coeff_12_ship_prediction * t2;
  i_coeffs_optimized_tmp = coeff_5_ship_prediction * t3;
  j_coeffs_optimized_tmp = coeff_6_ship_prediction * t3;
  k_coeffs_optimized_tmp = coeff_13_ship_prediction * t2;
  l_coeffs_optimized_tmp = coeff_14_ship_prediction * t2;
  m_coeffs_optimized_tmp = coeff_7_ship_prediction * t3;
  n_coeffs_optimized_tmp = coeff_15_ship_prediction * t2;
  o_coeffs_optimized_tmp = Y_0_SHIP * 840.0 - Y_0_UAV * 840.0;
  p_coeffs_optimized_tmp = Ay_0_UAV * t10;
  q_coeffs_optimized_tmp = Vy_0_UAV * t_landing;
  r_coeffs_optimized_tmp = b_coeffs_optimized_tmp * t15;
  s_coeffs_optimized_tmp = c_coeffs_optimized_tmp * t14;
  t_coeffs_optimized_tmp = m_coeffs_optimized_tmp * t10;
  u_coeffs_optimized_tmp = coeffs_optimized_tmp * t19;
  v_coeffs_optimized_tmp = f_coeffs_optimized_tmp * t15;
  w_coeffs_optimized_tmp = g_coeffs_optimized_tmp * t14;
  x_coeffs_optimized_tmp = n_coeffs_optimized_tmp * t10;
  y_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t19;
  ab_coeffs_optimized_tmp = coeff_8_ship_prediction * t3 * t_landing;
  bb_coeffs_optimized_tmp = coeff_16_ship_prediction * t2 * t_landing;
  cb_coeffs_optimized_tmp = coeffs_optimized_tmp * t4 * t14;
  db_coeffs_optimized_tmp = coeffs_optimized_tmp * t8 * t10;
  eb_coeffs_optimized_tmp = b_coeffs_optimized_tmp * t7 * t10;
  fb_coeffs_optimized_tmp = c_coeffs_optimized_tmp * t6 * t10;
  gb_coeffs_optimized_tmp = d_coeffs_optimized_tmp * t5 * t10;
  hb_coeffs_optimized_tmp = i_coeffs_optimized_tmp * t4 * t10;
  ib_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t4 * t14;
  jb_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t8 * t10;
  kb_coeffs_optimized_tmp = f_coeffs_optimized_tmp * t7 * t10;
  lb_coeffs_optimized_tmp = g_coeffs_optimized_tmp * t6 * t10;
  mb_coeffs_optimized_tmp = h_coeffs_optimized_tmp * t5 * t10;
  nb_coeffs_optimized_tmp = k_coeffs_optimized_tmp * t4 * t10;
  ob_coeffs_optimized_tmp =
      coeffs_optimized_tmp * t15 * t_delay_ship_prediction;
  pb_coeffs_optimized_tmp =
      b_coeffs_optimized_tmp * t14 * t_delay_ship_prediction;
  qb_coeffs_optimized_tmp =
      j_coeffs_optimized_tmp * t10 * t_delay_ship_prediction;
  rb_coeffs_optimized_tmp =
      e_coeffs_optimized_tmp * t15 * t_delay_ship_prediction;
  sb_coeffs_optimized_tmp =
      f_coeffs_optimized_tmp * t14 * t_delay_ship_prediction;
  tb_coeffs_optimized_tmp =
      l_coeffs_optimized_tmp * t10 * t_delay_ship_prediction;
  ub_coeffs_optimized_tmp = coeffs_optimized_tmp * t9 * t_landing;
  vb_coeffs_optimized_tmp = b_coeffs_optimized_tmp * t8 * t_landing;
  wb_coeffs_optimized_tmp = c_coeffs_optimized_tmp * t7 * t_landing;
  xb_coeffs_optimized_tmp = d_coeffs_optimized_tmp * t6 * t_landing;
  yb_coeffs_optimized_tmp = i_coeffs_optimized_tmp * t5 * t_landing;
  ac_coeffs_optimized_tmp = j_coeffs_optimized_tmp * t4 * t_landing;
  bc_coeffs_optimized_tmp = e_coeffs_optimized_tmp * t9 * t_landing;
  cc_coeffs_optimized_tmp = f_coeffs_optimized_tmp * t8 * t_landing;
  dc_coeffs_optimized_tmp = g_coeffs_optimized_tmp * t7 * t_landing;
  ec_coeffs_optimized_tmp = h_coeffs_optimized_tmp * t6 * t_landing;
  fc_coeffs_optimized_tmp = k_coeffs_optimized_tmp * t5 * t_landing;
  gc_coeffs_optimized_tmp = l_coeffs_optimized_tmp * t4 * t_landing;
  m_coeffs_optimized_tmp =
      m_coeffs_optimized_tmp * t_delay_ship_prediction * t_landing;
  n_coeffs_optimized_tmp =
      n_coeffs_optimized_tmp * t_delay_ship_prediction * t_landing;
  coeffs_optimized[6] =
      t18 *
      ((((((((((((((((((((((((((((o_coeffs_optimized_tmp -
                                  p_coeffs_optimized_tmp * 70.0) -
                                 q_coeffs_optimized_tmp * 420.0) +
                                r_coeffs_optimized_tmp * 120.0) +
                               s_coeffs_optimized_tmp * 70.0) +
                              d_coeffs_optimized_tmp * t13 * 28.0) +
                             t_coeffs_optimized_tmp * 70.0) +
                            u_coeffs_optimized_tmp * 175.0) +
                           v_coeffs_optimized_tmp * 120.0) +
                          w_coeffs_optimized_tmp * 70.0) +
                         h_coeffs_optimized_tmp * t13 * 28.0) +
                        x_coeffs_optimized_tmp * 70.0) +
                       y_coeffs_optimized_tmp * 175.0) +
                      ab_coeffs_optimized_tmp * 420.0) +
                     bb_coeffs_optimized_tmp * 420.0) +
                    cb_coeffs_optimized_tmp * 1470.0) +
                   coeffs_optimized_tmp * t5 * t13 * 980.0) +
                  db_coeffs_optimized_tmp * 490.0) +
                 b_coeffs_optimized_tmp * t4 * t13 * 420.0) +
                eb_coeffs_optimized_tmp * 420.0) +
               fb_coeffs_optimized_tmp * 350.0) +
              gb_coeffs_optimized_tmp * 280.0) +
             hb_coeffs_optimized_tmp * 210.0) +
            ib_coeffs_optimized_tmp * 1470.0) +
           e_coeffs_optimized_tmp * t5 * t13 * 980.0) +
          jb_coeffs_optimized_tmp * 490.0) +
         f_coeffs_optimized_tmp * t4 * t13 * 420.0) +
        (((((((((((((((((((((((kb_coeffs_optimized_tmp * 420.0 +
                               lb_coeffs_optimized_tmp * 350.0) +
                              mb_coeffs_optimized_tmp * 280.0) +
                             nb_coeffs_optimized_tmp * 210.0) +
                            ob_coeffs_optimized_tmp * 840.0) +
                           pb_coeffs_optimized_tmp * 420.0) +
                          c_coeffs_optimized_tmp * t13 *
                              t_delay_ship_prediction * 140.0) +
                         qb_coeffs_optimized_tmp * 140.0) +
                        rb_coeffs_optimized_tmp * 840.0) +
                       sb_coeffs_optimized_tmp * 420.0) +
                      g_coeffs_optimized_tmp * t13 * t_delay_ship_prediction *
                          140.0) +
                     tb_coeffs_optimized_tmp * 140.0) +
                    ub_coeffs_optimized_tmp * 420.0) +
                   vb_coeffs_optimized_tmp * 420.0) +
                  wb_coeffs_optimized_tmp * 420.0) +
                 xb_coeffs_optimized_tmp * 420.0) +
                yb_coeffs_optimized_tmp * 420.0) +
               ac_coeffs_optimized_tmp * 420.0) +
              bc_coeffs_optimized_tmp * 420.0) +
             cc_coeffs_optimized_tmp * 420.0) +
            dc_coeffs_optimized_tmp * 420.0) +
           ec_coeffs_optimized_tmp * 420.0) +
          fc_coeffs_optimized_tmp * 420.0) +
         gc_coeffs_optimized_tmp * 420.0)) +
       (m_coeffs_optimized_tmp * 420.0 + n_coeffs_optimized_tmp * 420.0)) /
      140.0;
  coeffs_optimized[7] =
      t17 *
      (((((((((((((((((((((((((((o_coeffs_optimized_tmp -
                                 p_coeffs_optimized_tmp * 84.0) -
                                q_coeffs_optimized_tmp * 448.0) +
                               r_coeffs_optimized_tmp * 64.0) +
                              s_coeffs_optimized_tmp * 28.0) -
                             i_coeffs_optimized_tmp * t12 * 14.0) +
                            t_coeffs_optimized_tmp * 84.0) +
                           u_coeffs_optimized_tmp * 105.0) +
                          v_coeffs_optimized_tmp * 64.0) +
                         w_coeffs_optimized_tmp * 28.0) -
                        k_coeffs_optimized_tmp * t12 * 14.0) +
                       x_coeffs_optimized_tmp * 84.0) +
                      y_coeffs_optimized_tmp * 105.0) +
                     ab_coeffs_optimized_tmp * 448.0) +
                    bb_coeffs_optimized_tmp * 448.0) +
                   cb_coeffs_optimized_tmp * 588.0) -
                  coeffs_optimized_tmp * t6 * t12 * 490.0) +
                 db_coeffs_optimized_tmp * 588.0) -
                b_coeffs_optimized_tmp * t5 * t12 * 280.0) +
               eb_coeffs_optimized_tmp * 504.0) -
              c_coeffs_optimized_tmp * t4 * t12 * 140.0) +
             fb_coeffs_optimized_tmp * 420.0) +
            gb_coeffs_optimized_tmp * 336.0) +
           hb_coeffs_optimized_tmp * 252.0) +
          ib_coeffs_optimized_tmp * 588.0) -
         e_coeffs_optimized_tmp * t6 * t12 * 490.0) +
        (((((((((((((((((((((((jb_coeffs_optimized_tmp * 588.0 -
                               f_coeffs_optimized_tmp * t5 * t12 * 280.0) +
                              kb_coeffs_optimized_tmp * 504.0) -
                             g_coeffs_optimized_tmp * t4 * t12 * 140.0) +
                            lb_coeffs_optimized_tmp * 420.0) +
                           mb_coeffs_optimized_tmp * 336.0) +
                          nb_coeffs_optimized_tmp * 252.0) +
                         ob_coeffs_optimized_tmp * 448.0) +
                        pb_coeffs_optimized_tmp * 168.0) -
                       d_coeffs_optimized_tmp * t12 * t_delay_ship_prediction *
                           56.0) +
                      qb_coeffs_optimized_tmp * 168.0) +
                     rb_coeffs_optimized_tmp * 448.0) +
                    sb_coeffs_optimized_tmp * 168.0) -
                   h_coeffs_optimized_tmp * t12 * t_delay_ship_prediction *
                       56.0) +
                  tb_coeffs_optimized_tmp * 168.0) +
                 ub_coeffs_optimized_tmp * 448.0) +
                vb_coeffs_optimized_tmp * 448.0) +
               wb_coeffs_optimized_tmp * 448.0) +
              xb_coeffs_optimized_tmp * 448.0) +
             yb_coeffs_optimized_tmp * 448.0) +
            ac_coeffs_optimized_tmp * 448.0) +
           bc_coeffs_optimized_tmp * 448.0) +
          cc_coeffs_optimized_tmp * 448.0) +
         dc_coeffs_optimized_tmp * 448.0)) +
       ((((ec_coeffs_optimized_tmp * 448.0 + fc_coeffs_optimized_tmp * 448.0) +
          gc_coeffs_optimized_tmp * 448.0) +
         m_coeffs_optimized_tmp * 448.0) +
        n_coeffs_optimized_tmp * 448.0)) *
      -0.017857142857142856;
  coeffs_optimized[8] =
      t16 *
      ((((((((((((((((((((((((((((o_coeffs_optimized_tmp -
                                  p_coeffs_optimized_tmp * 126.0) -
                                 q_coeffs_optimized_tmp * 504.0) +
                                r_coeffs_optimized_tmp * 36.0) +
                               s_coeffs_optimized_tmp * 14.0) +
                              j_coeffs_optimized_tmp * t11 * 28.0) +
                             t_coeffs_optimized_tmp * 126.0) +
                            u_coeffs_optimized_tmp * 63.0) +
                           v_coeffs_optimized_tmp * 36.0) +
                          w_coeffs_optimized_tmp * 14.0) +
                         l_coeffs_optimized_tmp * t11 * 28.0) +
                        x_coeffs_optimized_tmp * 126.0) +
                       y_coeffs_optimized_tmp * 63.0) +
                      ab_coeffs_optimized_tmp * 504.0) +
                     bb_coeffs_optimized_tmp * 504.0) +
                    cb_coeffs_optimized_tmp * 294.0) +
                   coeffs_optimized_tmp * t7 * t11 * 588.0) +
                  db_coeffs_optimized_tmp * 882.0) +
                 b_coeffs_optimized_tmp * t6 * t11 * 420.0) +
                eb_coeffs_optimized_tmp * 756.0) +
               c_coeffs_optimized_tmp * t5 * t11 * 280.0) +
              fb_coeffs_optimized_tmp * 630.0) +
             d_coeffs_optimized_tmp * t4 * t11 * 168.0) +
            gb_coeffs_optimized_tmp * 504.0) +
           hb_coeffs_optimized_tmp * 378.0) +
          ib_coeffs_optimized_tmp * 294.0) +
         e_coeffs_optimized_tmp * t7 * t11 * 588.0) +
        (((((((((((((((((((((((jb_coeffs_optimized_tmp * 882.0 +
                               f_coeffs_optimized_tmp * t6 * t11 * 420.0) +
                              kb_coeffs_optimized_tmp * 756.0) +
                             g_coeffs_optimized_tmp * t5 * t11 * 280.0) +
                            lb_coeffs_optimized_tmp * 630.0) +
                           h_coeffs_optimized_tmp * t4 * t11 * 168.0) +
                          mb_coeffs_optimized_tmp * 504.0) +
                         nb_coeffs_optimized_tmp * 378.0) +
                        ob_coeffs_optimized_tmp * 252.0) +
                       pb_coeffs_optimized_tmp * 84.0) +
                      i_coeffs_optimized_tmp * t11 * t_delay_ship_prediction *
                          84.0) +
                     qb_coeffs_optimized_tmp * 252.0) +
                    rb_coeffs_optimized_tmp * 252.0) +
                   sb_coeffs_optimized_tmp * 84.0) +
                  k_coeffs_optimized_tmp * t11 * t_delay_ship_prediction *
                      84.0) +
                 tb_coeffs_optimized_tmp * 252.0) +
                ub_coeffs_optimized_tmp * 504.0) +
               vb_coeffs_optimized_tmp * 504.0) +
              wb_coeffs_optimized_tmp * 504.0) +
             xb_coeffs_optimized_tmp * 504.0) +
            yb_coeffs_optimized_tmp * 504.0) +
           ac_coeffs_optimized_tmp * 504.0) +
          bc_coeffs_optimized_tmp * 504.0) +
         cc_coeffs_optimized_tmp * 504.0)) +
       (((((dc_coeffs_optimized_tmp * 504.0 + ec_coeffs_optimized_tmp * 504.0) +
           fc_coeffs_optimized_tmp * 504.0) +
          gc_coeffs_optimized_tmp * 504.0) +
         m_coeffs_optimized_tmp * 504.0) +
        n_coeffs_optimized_tmp * 504.0)) /
      84.0;
  coeffs_optimized[9] = Ay_0_UAV / 2.0;
  coeffs_optimized[10] = Vy_0_UAV;
  coeffs_optimized[11] = Y_0_UAV;
  coeffs_optimized_tmp = Z_0_SHIP * 840.0 - Z_0_UAV * 840.0;
  b_coeffs_optimized_tmp = Az_0_UAV * t10;
  c_coeffs_optimized_tmp = Vz_0_UAV * t_landing;
  d_coeffs_optimized_tmp = coeff_18_ship_prediction * t15;
  e_coeffs_optimized_tmp = coeff_19_ship_prediction * t14;
  f_coeffs_optimized_tmp = coeff_23_ship_prediction * t10;
  g_coeffs_optimized_tmp = coeff_17_ship_prediction * t19;
  h_coeffs_optimized_tmp = coeff_24_ship_prediction * t_landing;
  i_coeffs_optimized_tmp = coeff_17_ship_prediction * t4 * t14;
  j_coeffs_optimized_tmp = coeff_17_ship_prediction * t8 * t10;
  k_coeffs_optimized_tmp = coeff_18_ship_prediction * t7 * t10;
  l_coeffs_optimized_tmp = coeff_19_ship_prediction * t6 * t10;
  m_coeffs_optimized_tmp = coeff_20_ship_prediction * t5 * t10;
  n_coeffs_optimized_tmp = coeff_21_ship_prediction * t4 * t10;
  o_coeffs_optimized_tmp =
      coeff_17_ship_prediction * t15 * t_delay_ship_prediction;
  p_coeffs_optimized_tmp =
      coeff_18_ship_prediction * t14 * t_delay_ship_prediction;
  q_coeffs_optimized_tmp =
      coeff_22_ship_prediction * t10 * t_delay_ship_prediction;
  r_coeffs_optimized_tmp = coeff_17_ship_prediction * t9 * t_landing;
  s_coeffs_optimized_tmp = coeff_18_ship_prediction * t8 * t_landing;
  t_coeffs_optimized_tmp = coeff_19_ship_prediction * t7 * t_landing;
  u_coeffs_optimized_tmp = coeff_20_ship_prediction * t6 * t_landing;
  v_coeffs_optimized_tmp = coeff_21_ship_prediction * t5 * t_landing;
  w_coeffs_optimized_tmp = coeff_22_ship_prediction * t4 * t_landing;
  x_coeffs_optimized_tmp =
      coeff_23_ship_prediction * t_delay_ship_prediction * t_landing;
  coeffs_optimized[12] =
      t18 *
      (((((((((((((((((((((((((((coeffs_optimized_tmp -
                                 b_coeffs_optimized_tmp * 70.0) -
                                c_coeffs_optimized_tmp * 420.0) +
                               d_coeffs_optimized_tmp * 120.0) +
                              e_coeffs_optimized_tmp * 70.0) +
                             coeff_20_ship_prediction * t13 * 28.0) +
                            f_coeffs_optimized_tmp * 70.0) +
                           g_coeffs_optimized_tmp * 175.0) +
                          h_coeffs_optimized_tmp * 420.0) +
                         i_coeffs_optimized_tmp * 1470.0) +
                        coeff_17_ship_prediction * t5 * t13 * 980.0) +
                       j_coeffs_optimized_tmp * 490.0) +
                      coeff_18_ship_prediction * t4 * t13 * 420.0) +
                     k_coeffs_optimized_tmp * 420.0) +
                    l_coeffs_optimized_tmp * 350.0) +
                   m_coeffs_optimized_tmp * 280.0) +
                  n_coeffs_optimized_tmp * 210.0) +
                 o_coeffs_optimized_tmp * 840.0) +
                p_coeffs_optimized_tmp * 420.0) +
               coeff_19_ship_prediction * t13 * t_delay_ship_prediction *
                   140.0) +
              q_coeffs_optimized_tmp * 140.0) +
             r_coeffs_optimized_tmp * 420.0) +
            s_coeffs_optimized_tmp * 420.0) +
           t_coeffs_optimized_tmp * 420.0) +
          u_coeffs_optimized_tmp * 420.0) +
         v_coeffs_optimized_tmp * 420.0) +
        w_coeffs_optimized_tmp * 420.0) +
       x_coeffs_optimized_tmp * 420.0) /
      140.0;
  coeffs_optimized[13] =
      t17 *
      ((((((((((((((((((((((((((((coeffs_optimized_tmp -
                                  b_coeffs_optimized_tmp * 84.0) -
                                 c_coeffs_optimized_tmp * 448.0) +
                                d_coeffs_optimized_tmp * 64.0) +
                               e_coeffs_optimized_tmp * 28.0) -
                              coeff_21_ship_prediction * t12 * 14.0) +
                             f_coeffs_optimized_tmp * 84.0) +
                            g_coeffs_optimized_tmp * 105.0) +
                           h_coeffs_optimized_tmp * 448.0) +
                          i_coeffs_optimized_tmp * 588.0) -
                         coeff_17_ship_prediction * t6 * t12 * 490.0) +
                        j_coeffs_optimized_tmp * 588.0) -
                       coeff_18_ship_prediction * t5 * t12 * 280.0) +
                      k_coeffs_optimized_tmp * 504.0) -
                     coeff_19_ship_prediction * t4 * t12 * 140.0) +
                    l_coeffs_optimized_tmp * 420.0) +
                   m_coeffs_optimized_tmp * 336.0) +
                  n_coeffs_optimized_tmp * 252.0) +
                 o_coeffs_optimized_tmp * 448.0) +
                p_coeffs_optimized_tmp * 168.0) -
               coeff_20_ship_prediction * t12 * t_delay_ship_prediction *
                   56.0) +
              q_coeffs_optimized_tmp * 168.0) +
             r_coeffs_optimized_tmp * 448.0) +
            s_coeffs_optimized_tmp * 448.0) +
           t_coeffs_optimized_tmp * 448.0) +
          u_coeffs_optimized_tmp * 448.0) +
         v_coeffs_optimized_tmp * 448.0) +
        w_coeffs_optimized_tmp * 448.0) +
       x_coeffs_optimized_tmp * 448.0) *
      -0.017857142857142856;
  coeffs_optimized[14] =
      t16 *
      (((((((((((((((((((((((((((((coeffs_optimized_tmp -
                                   b_coeffs_optimized_tmp * 126.0) -
                                  c_coeffs_optimized_tmp * 504.0) +
                                 d_coeffs_optimized_tmp * 36.0) +
                                e_coeffs_optimized_tmp * 14.0) +
                               coeff_22_ship_prediction * t11 * 28.0) +
                              f_coeffs_optimized_tmp * 126.0) +
                             g_coeffs_optimized_tmp * 63.0) +
                            h_coeffs_optimized_tmp * 504.0) +
                           i_coeffs_optimized_tmp * 294.0) +
                          coeff_17_ship_prediction * t7 * t11 * 588.0) +
                         j_coeffs_optimized_tmp * 882.0) +
                        coeff_18_ship_prediction * t6 * t11 * 420.0) +
                       k_coeffs_optimized_tmp * 756.0) +
                      coeff_19_ship_prediction * t5 * t11 * 280.0) +
                     l_coeffs_optimized_tmp * 630.0) +
                    coeff_20_ship_prediction * t4 * t11 * 168.0) +
                   m_coeffs_optimized_tmp * 504.0) +
                  n_coeffs_optimized_tmp * 378.0) +
                 o_coeffs_optimized_tmp * 252.0) +
                p_coeffs_optimized_tmp * 84.0) +
               coeff_21_ship_prediction * t11 * t_delay_ship_prediction *
                   84.0) +
              q_coeffs_optimized_tmp * 252.0) +
             r_coeffs_optimized_tmp * 504.0) +
            s_coeffs_optimized_tmp * 504.0) +
           t_coeffs_optimized_tmp * 504.0) +
          u_coeffs_optimized_tmp * 504.0) +
         v_coeffs_optimized_tmp * 504.0) +
        w_coeffs_optimized_tmp * 504.0) +
       x_coeffs_optimized_tmp * 504.0) *
      0.011904761904761904;
  coeffs_optimized[15] = Az_0_UAV / 2.0;
  coeffs_optimized[16] = Vz_0_UAV;
  coeffs_optimized[17] = Z_0_UAV;
}

/*
 * Prepare variables for funcion:
 *
 * Arguments    : const double coeffs_ship_prediction[24]
 *                const double P0_NED[3]
 *                const double V0_NED[3]
 *                const double A0_NED[3]
 *                double Psi_ship_rad
 *                const double P0_SHIP_NED[3]
 *                const double v_max_control_rf[3]
 *                const double v_min_control_rf[3]
 *                const double a_max_control_rf[3]
 *                const double a_min_control_rf[3]
 *                double max_time_of_landing_seconds
 *                double t_delay_ship_prediction
 *                double optimal_coeffs[18]
 *                double *exitflag
 *                double V_OOB[6]
 *                double A_OOB[6]
 * Return Type  : double
 */
static double c_optimal_linear_coeff_fcn_calc(
    const double coeffs_ship_prediction[24], const double P0_NED[3],
    const double V0_NED[3], const double A0_NED[3], double Psi_ship_rad,
    const double P0_SHIP_NED[3], const double v_max_control_rf[3],
    const double v_min_control_rf[3], const double a_max_control_rf[3],
    const double a_min_control_rf[3], double max_time_of_landing_seconds,
    double t_delay_ship_prediction, double optimal_coeffs[18], double *exitflag,
    double V_OOB[6], double A_OOB[6])
{
  double landing_time;
  int b_i;
  int i;
  int i1;
  /* Init guess coefficients:  */
  optimal_coeffs[3] = A0_NED[0];
  optimal_coeffs[4] = V0_NED[0];
  optimal_coeffs[5] = P0_NED[0];
  optimal_coeffs[9] = A0_NED[1];
  optimal_coeffs[10] = V0_NED[1];
  optimal_coeffs[11] = P0_NED[1];
  optimal_coeffs[0] = 0.0;
  optimal_coeffs[6] = 0.0;
  optimal_coeffs[12] = 0.0;
  optimal_coeffs[1] = 0.0;
  optimal_coeffs[7] = 0.0;
  optimal_coeffs[13] = 0.0;
  optimal_coeffs[2] = 0.0;
  optimal_coeffs[8] = 0.0;
  optimal_coeffs[14] = 0.0;
  optimal_coeffs[15] = A0_NED[2];
  optimal_coeffs[16] = V0_NED[2];
  optimal_coeffs[17] = P0_NED[2];
  landing_time = 0.3;
  *exitflag = -1.0;
  for (i = 0; i < 6; i++) {
    V_OOB[i] = 0.0;
    A_OOB[i] = 0.0;
  }
  while ((*exitflag < 0.0) && (landing_time < max_time_of_landing_seconds)) {
    double R_EC[180];
    double UAV_path_acc_control_rf[60];
    double UAV_path_speed_control_rf[60];
    double optimal_UAV_path_acc[60];
    double optimal_UAV_path_speed[60];
    double UAV_path_heading[20];
    int UAV_path_acc_control_rf_tmp;
    signed char b_x[12];
    bool x[20];
    bool Vx_max_OOB;
    bool Vx_min_OOB;
    bool Vy_max_OOB;
    bool Vy_min_OOB;
    bool Vz_max_OOB;
    bool Vz_min_OOB;
    bool exitg1;
    landing_time += 0.1;
    c_optimal_linear_coeff_calculat(
        A0_NED[0], A0_NED[1], A0_NED[2], Psi_ship_rad, V0_NED[0], V0_NED[1],
        V0_NED[2], P0_NED[0], P0_SHIP_NED[0], P0_NED[1], P0_SHIP_NED[1],
        P0_NED[2], P0_SHIP_NED[2], coeffs_ship_prediction[0],
        coeffs_ship_prediction[1], coeffs_ship_prediction[2],
        coeffs_ship_prediction[3], coeffs_ship_prediction[4],
        coeffs_ship_prediction[5], coeffs_ship_prediction[6],
        coeffs_ship_prediction[7], coeffs_ship_prediction[8],
        coeffs_ship_prediction[9], coeffs_ship_prediction[10],
        coeffs_ship_prediction[11], coeffs_ship_prediction[12],
        coeffs_ship_prediction[13], coeffs_ship_prediction[14],
        coeffs_ship_prediction[15], coeffs_ship_prediction[16],
        coeffs_ship_prediction[17], coeffs_ship_prediction[18],
        coeffs_ship_prediction[19], coeffs_ship_prediction[20],
        coeffs_ship_prediction[21], coeffs_ship_prediction[22],
        coeffs_ship_prediction[23], t_delay_ship_prediction, landing_time,
        optimal_coeffs);
    evaluate_UAV_path_from_coeffs(
        optimal_coeffs, landing_time, UAV_path_speed_control_rf,
        optimal_UAV_path_speed, optimal_UAV_path_acc, UAV_path_heading);
    /* Check that the speed and accelerations are always within the bounds: */
    /* Report speeds in the control reference frame:  */
    /* Check if the speed and acceleration are within the bounds: */
    /* Speeds Out Of Bounds: */
    for (b_i = 0; b_i < 20; b_i++) {
      double R_EC_tmp;
      double b_R_EC_tmp;
      double d;
      double d1;
      double d2;
      d = optimal_UAV_path_speed[3 * b_i];
      i = 3 * b_i + 1;
      d1 = optimal_UAV_path_speed[i];
      d2 = atan2(d1, d);
      R_EC_tmp = sin(d2);
      b_R_EC_tmp = cos(d2);
      R_EC[9 * b_i] = b_R_EC_tmp;
      R_EC[9 * b_i + 3] = R_EC_tmp;
      R_EC[9 * b_i + 6] = 0.0;
      R_EC[9 * b_i + 1] = -R_EC_tmp;
      R_EC[9 * b_i + 4] = b_R_EC_tmp;
      R_EC[9 * b_i + 7] = 0.0;
      R_EC[9 * b_i + 2] = 0.0;
      R_EC[9 * b_i + 5] = 0.0;
      R_EC[9 * b_i + 8] = 1.0;
      d2 = optimal_UAV_path_acc[3 * b_i];
      b_R_EC_tmp = optimal_UAV_path_acc[i];
      i = 3 * b_i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        double d3;
        double d4;
        UAV_path_acc_control_rf_tmp = i1 + 9 * b_i;
        R_EC_tmp = R_EC[UAV_path_acc_control_rf_tmp];
        d3 = R_EC_tmp * d;
        d4 = R_EC_tmp * d2;
        R_EC_tmp = R_EC[UAV_path_acc_control_rf_tmp + 3];
        d3 += R_EC_tmp * d1;
        d4 += R_EC_tmp * b_R_EC_tmp;
        R_EC_tmp = R_EC[UAV_path_acc_control_rf_tmp + 6];
        d3 += R_EC_tmp * optimal_UAV_path_speed[i];
        d4 += R_EC_tmp * optimal_UAV_path_acc[i];
        UAV_path_acc_control_rf_tmp = i1 + 3 * b_i;
        UAV_path_acc_control_rf[UAV_path_acc_control_rf_tmp] = d4;
        UAV_path_speed_control_rf[UAV_path_acc_control_rf_tmp] = d3;
      }
      x[b_i] = (UAV_path_speed_control_rf[3 * b_i] > v_max_control_rf[0]);
    }
    Vx_max_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vx_max_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_speed_control_rf[3 * i] < v_min_control_rf[0]);
    }
    Vx_min_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vx_min_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_speed_control_rf[3 * i + 1] > v_max_control_rf[1]);
    }
    Vy_max_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vy_max_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_speed_control_rf[3 * i + 1] < v_min_control_rf[1]);
    }
    Vy_min_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vy_min_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_speed_control_rf[3 * i + 2] > v_max_control_rf[2]);
    }
    Vz_max_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vz_max_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_speed_control_rf[3 * i + 2] < v_min_control_rf[2]);
    }
    Vz_min_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vz_min_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    V_OOB[0] = Vx_max_OOB;
    V_OOB[1] = Vy_max_OOB;
    V_OOB[2] = Vz_max_OOB;
    V_OOB[3] = Vx_min_OOB;
    V_OOB[4] = Vy_min_OOB;
    V_OOB[5] = Vz_min_OOB;
    /* Accellerations Out Of Bounds: */
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_acc_control_rf[3 * i] > a_max_control_rf[0]);
    }
    Vx_max_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vx_max_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_acc_control_rf[3 * i] < a_min_control_rf[0]);
    }
    Vx_min_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vx_min_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_acc_control_rf[3 * i + 1] > a_max_control_rf[1]);
    }
    Vy_max_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vy_max_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_acc_control_rf[3 * i + 1] < a_min_control_rf[1]);
    }
    Vy_min_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vy_min_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_acc_control_rf[3 * i + 2] > a_max_control_rf[2]);
    }
    Vz_max_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vz_max_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    for (i = 0; i < 20; i++) {
      x[i] = (UAV_path_acc_control_rf[3 * i + 2] < a_min_control_rf[2]);
    }
    Vz_min_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
      if (x[UAV_path_acc_control_rf_tmp]) {
        Vz_min_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    A_OOB[0] = Vx_max_OOB;
    A_OOB[1] = Vy_max_OOB;
    A_OOB[2] = Vz_max_OOB;
    A_OOB[3] = Vx_min_OOB;
    A_OOB[4] = Vy_min_OOB;
    A_OOB[5] = Vz_min_OOB;
    for (i = 0; i < 6; i++) {
      b_x[i] = (signed char)V_OOB[i];
      b_x[i + 6] = (signed char)A_OOB[i];
    }
    Vx_max_OOB = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 12)) {
      if (b_x[UAV_path_acc_control_rf_tmp] != 0) {
        Vx_max_OOB = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    if (!Vx_max_OOB) {
      *exitflag = 1.0;
    }
  }
  /*  Display the total travel time and computational time.  */
  return landing_time;
}

/*
 * Arguments    : const double coeffs[18]
 *                double landing_time
 *                double UAV_path[60]
 *                double UAV_path_speed[60]
 *                double UAV_path_acc[60]
 *                double t_array[20]
 * Return Type  : void
 */
static void
evaluate_UAV_path_from_coeffs(const double coeffs[18], double landing_time,
                              double UAV_path[60], double UAV_path_speed[60],
                              double UAV_path_acc[60], double t_array[20])
{
  double a;
  double a_tmp;
  double b_a;
  double b_a_tmp;
  double c_a;
  double c_a_tmp;
  double d;
  double d1;
  double d10;
  double d11;
  double d12;
  double d13;
  double d14;
  double d15;
  double d16;
  double d17;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  double d_a;
  double delta1;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double m_a;
  double n_a;
  double o_a;
  double p_a;
  double q_a;
  int k;
  /*  Generate UAV path from 5rd-order polynomial coefficients: */
  t_array[19] = landing_time;
  t_array[0] = 0.0;
  delta1 = landing_time / 19.0;
  for (k = 0; k < 18; k++) {
    t_array[k + 1] = ((double)k + 1.0) * delta1;
  }
  delta1 = 5.0 * coeffs[0];
  a = 4.0 * coeffs[1];
  b_a = 3.0 * coeffs[2];
  a_tmp = 2.0 * coeffs[3];
  c_a = 5.0 * coeffs[6];
  d_a = 4.0 * coeffs[7];
  e_a = 3.0 * coeffs[8];
  b_a_tmp = 2.0 * coeffs[9];
  f_a = 5.0 * coeffs[12];
  g_a = 4.0 * coeffs[13];
  h_a = 3.0 * coeffs[14];
  c_a_tmp = 2.0 * coeffs[15];
  i_a = 20.0 * coeffs[0];
  j_a = 12.0 * coeffs[1];
  k_a = 6.0 * coeffs[2];
  l_a = 20.0 * coeffs[6];
  m_a = 12.0 * coeffs[7];
  n_a = 6.0 * coeffs[8];
  o_a = 20.0 * coeffs[12];
  p_a = 12.0 * coeffs[13];
  q_a = 6.0 * coeffs[14];
  d = coeffs[0];
  d1 = coeffs[1];
  d2 = coeffs[2];
  d3 = coeffs[3];
  d4 = coeffs[4];
  d5 = coeffs[5];
  d6 = coeffs[6];
  d7 = coeffs[7];
  d8 = coeffs[8];
  d9 = coeffs[9];
  d10 = coeffs[10];
  d11 = coeffs[11];
  d12 = coeffs[12];
  d13 = coeffs[13];
  d14 = coeffs[14];
  d15 = coeffs[15];
  d16 = coeffs[16];
  d17 = coeffs[17];
  for (k = 0; k < 20; k++) {
    double d18;
    double d19;
    double d20;
    double d21;
    double d22;
    int UAV_path_tmp;
    int b_UAV_path_tmp;
    d18 = t_array[k];
    d19 = pow(d18, 5.0);
    d20 = pow(d18, 4.0);
    d21 = pow(d18, 3.0);
    d22 = d18 * d18;
    UAV_path[3 * k] =
        ((((d * d19 + d1 * d20) + d2 * d21) + d3 * d22) + d4 * d18) + d5;
    UAV_path_tmp = 3 * k + 1;
    UAV_path[UAV_path_tmp] =
        ((((d6 * d19 + d7 * d20) + d8 * d21) + d9 * d22) + d10 * d18) + d11;
    b_UAV_path_tmp = 3 * k + 2;
    UAV_path[b_UAV_path_tmp] =
        ((((d12 * d19 + d13 * d20) + d14 * d21) + d15 * d22) + d16 * d18) + d17;
    UAV_path_speed[3 * k] =
        (((delta1 * d20 + a * d21) + b_a * d22) + a_tmp * d18) + d4;
    UAV_path_speed[UAV_path_tmp] =
        (((c_a * d20 + d_a * d21) + e_a * d22) + b_a_tmp * d18) + d10;
    UAV_path_speed[b_UAV_path_tmp] =
        (((f_a * d20 + g_a * d21) + h_a * d22) + c_a_tmp * d18) + d16;
    UAV_path_acc[3 * k] = ((i_a * d21 + j_a * d22) + k_a * d18) + a_tmp;
    UAV_path_acc[UAV_path_tmp] =
        ((l_a * d21 + m_a * d22) + n_a * d18) + b_a_tmp;
    UAV_path_acc[b_UAV_path_tmp] =
        ((o_a * d21 + p_a * d22) + q_a * d18) + c_a_tmp;
  }
}

/*
 * Based on the ship coefficents, generate the landing path coefficients in the
 * NED reference frame:
 *
 * Arguments    : const double P0_UAV_NED[3]
 *                const double V0_UAV_NED[3]
 *                const double V0_SHIP_NED[3]
 *                const double A0_UAV_NED[3]
 *                const double A0_UAV_NED_FILT[3]
 *                const double PhiThetaPsi_SHIP_rad[3]
 *                const double P0_SHIP_NED[3]
 *                double psi_rad_UAV
 *                const double v_max_control_rf[3]
 *                const double v_min_control_rf[3]
 *                const double a_max_control_rf[3]
 *                const double a_min_control_rf[3]
 *                const double coeffs_ship_prediction_speed_7[24]
 *                double t_delay_ship_prediction
 *                double max_time_of_landing_seconds
 *                double pos_gain_landing
 *                double speed_gain_landing
 *                double assume_zero_UAV_acc
 *                double flare_low_distance_m
 *                double v_speed_docking_m_s
 *                const double average_speed_NED_ship[3]
 *                double diag_approach_speed_m_s
 *                double approach_ship_mode_old
 *                const double c_NED_offset_end_point_diag_lan[3]
 *                double pos_tracking_distance_m
 *                double approach_heading_ship_rad
 *                double dist_line_gain
 *                double max_line_gain
 *                double A_err_control_rf[3]
 *                double *expected_landing_time_relative
 *                double optimal_coeffs[18]
 *                double A_target_control[3]
 *                double V_target_control[3]
 *                double *exitflag_approach_path
 *                double *approach_ship_mode
 *                double V_OOB[6]
 *                double A_OOB[6]
 *                double UAV_to_SHIP_dist_NED[3]
 *                double *Desired_phi_rad
 *                double *Desired_theta_rad
 *                double *UAV_to_SHIP_azimuth_angle_rad
 *                double *UAV_to_SHIP_elevation_angle_rad
 *                double *delta_psi
 *                double *psi_UAV_to_ship
 * Return Type  : void
 */
void nav_approach_ship(
    const double P0_UAV_NED[3], const double V0_UAV_NED[3],
    const double V0_SHIP_NED[3], const double A0_UAV_NED[3],
    const double A0_UAV_NED_FILT[3], const double PhiThetaPsi_SHIP_rad[3],
    const double P0_SHIP_NED[3], double psi_rad_UAV,
    const double v_max_control_rf[3], const double v_min_control_rf[3],
    const double a_max_control_rf[3], const double a_min_control_rf[3],
    const double coeffs_ship_prediction_speed_7[24],
    double t_delay_ship_prediction, double max_time_of_landing_seconds,
    double pos_gain_landing, double speed_gain_landing,
    double assume_zero_UAV_acc, double flare_low_distance_m,
    double v_speed_docking_m_s, const double average_speed_NED_ship[3],
    double diag_approach_speed_m_s, double approach_ship_mode_old,
    const double c_NED_offset_end_point_diag_lan[3],
    double pos_tracking_distance_m, double approach_heading_ship_rad,
    double dist_line_gain, double max_line_gain, double A_err_control_rf[3],
    double *expected_landing_time_relative, double optimal_coeffs[18],
    double A_target_control[3], double V_target_control[3],
    double *exitflag_approach_path, double *approach_ship_mode, double V_OOB[6],
    double A_OOB[6], double UAV_to_SHIP_dist_NED[3], double *Desired_phi_rad,
    double *Desired_theta_rad, double *UAV_to_SHIP_azimuth_angle_rad,
    double *UAV_to_SHIP_elevation_angle_rad, double *delta_psi,
    double *psi_UAV_to_ship)
{
  double UAV_path_acc_NED[60];
  double UAV_path_acc_control_rf[60];
  double UAV_path_speed_control_rf[60];
  double UAV_path_heading[20];
  double R_EC[9];
  double P0_SHIP_NED_w_offset[3];
  double UAV_to_SHIP_dist_NED_w_offset[3];
  double Relative_psi_UAV_to_SHIP;
  double UAV_to_SHIP_dist_norm;
  double V_err_NED_idx_0;
  double V_err_NED_idx_0_tmp;
  double V_err_NED_idx_1;
  double V_err_NED_idx_2;
  double chosen_approach_heading_rad;
  double d;
  double landing_time;
  double psi_UAV_to_ship_tmp;
  double ship_to_UAV_ned_x;
  double ship_to_UAV_ned_x_tmp;
  int b_i;
  int i;
  int i1;
  if (assume_zero_UAV_acc != 0.0) {
    int b_exitflag_approach_path;
    /* Prepare variables for funcion:  */
    /* Init guess coefficients:  */
    optimal_coeffs[3] = 0.0;
    optimal_coeffs[4] = V0_UAV_NED[0];
    optimal_coeffs[5] = P0_UAV_NED[0];
    optimal_coeffs[9] = 0.0;
    optimal_coeffs[10] = V0_UAV_NED[1];
    optimal_coeffs[11] = P0_UAV_NED[1];
    optimal_coeffs[0] = 0.0;
    optimal_coeffs[6] = 0.0;
    optimal_coeffs[12] = 0.0;
    optimal_coeffs[1] = 0.0;
    optimal_coeffs[7] = 0.0;
    optimal_coeffs[13] = 0.0;
    optimal_coeffs[2] = 0.0;
    optimal_coeffs[8] = 0.0;
    optimal_coeffs[14] = 0.0;
    optimal_coeffs[15] = 0.0;
    optimal_coeffs[16] = V0_UAV_NED[2];
    optimal_coeffs[17] = P0_UAV_NED[2];
    landing_time = 0.3;
    b_exitflag_approach_path = -1;
    for (i = 0; i < 6; i++) {
      V_OOB[i] = 0.0;
      A_OOB[i] = 0.0;
    }
    while ((b_exitflag_approach_path < 0) &&
           (landing_time < max_time_of_landing_seconds)) {
      double b_R_EC[180];
      double optimal_UAV_path_acc[60];
      int UAV_path_acc_control_rf_tmp;
      signed char b_x[12];
      bool x[20];
      bool b_y;
      bool c_y;
      bool d_y;
      bool e_y;
      bool exitg1;
      bool f_y;
      bool y;
      landing_time += 0.1;
      c_optimal_linear_coeff_calculat(
          0.0, 0.0, 0.0, PhiThetaPsi_SHIP_rad[2], V0_UAV_NED[0], V0_UAV_NED[1],
          V0_UAV_NED[2], P0_UAV_NED[0], P0_SHIP_NED[0], P0_UAV_NED[1],
          P0_SHIP_NED[1], P0_UAV_NED[2], P0_SHIP_NED[2],
          coeffs_ship_prediction_speed_7[0], coeffs_ship_prediction_speed_7[1],
          coeffs_ship_prediction_speed_7[2], coeffs_ship_prediction_speed_7[3],
          coeffs_ship_prediction_speed_7[4], coeffs_ship_prediction_speed_7[5],
          coeffs_ship_prediction_speed_7[6], coeffs_ship_prediction_speed_7[7],
          coeffs_ship_prediction_speed_7[8], coeffs_ship_prediction_speed_7[9],
          coeffs_ship_prediction_speed_7[10],
          coeffs_ship_prediction_speed_7[11],
          coeffs_ship_prediction_speed_7[12],
          coeffs_ship_prediction_speed_7[13],
          coeffs_ship_prediction_speed_7[14],
          coeffs_ship_prediction_speed_7[15],
          coeffs_ship_prediction_speed_7[16],
          coeffs_ship_prediction_speed_7[17],
          coeffs_ship_prediction_speed_7[18],
          coeffs_ship_prediction_speed_7[19],
          coeffs_ship_prediction_speed_7[20],
          coeffs_ship_prediction_speed_7[21],
          coeffs_ship_prediction_speed_7[22],
          coeffs_ship_prediction_speed_7[23], t_delay_ship_prediction,
          landing_time, optimal_coeffs);
      evaluate_UAV_path_from_coeffs(optimal_coeffs, landing_time,
                                    UAV_path_speed_control_rf, UAV_path_acc_NED,
                                    optimal_UAV_path_acc, UAV_path_heading);
      /* Check that the speed and accelerations are always within the bounds: */
      /* Report speeds in the control reference frame:  */
      /* Check if the speed and acceleration are within the bounds: */
      /* Speeds Out Of Bounds: */
      for (b_i = 0; b_i < 20; b_i++) {
        d = UAV_path_acc_NED[3 * b_i];
        i = 3 * b_i + 1;
        V_err_NED_idx_0 = UAV_path_acc_NED[i];
        UAV_to_SHIP_dist_norm = atan2(V_err_NED_idx_0, d);
        ship_to_UAV_ned_x = sin(UAV_to_SHIP_dist_norm);
        Relative_psi_UAV_to_SHIP = cos(UAV_to_SHIP_dist_norm);
        b_R_EC[9 * b_i] = Relative_psi_UAV_to_SHIP;
        b_R_EC[9 * b_i + 3] = ship_to_UAV_ned_x;
        b_R_EC[9 * b_i + 6] = 0.0;
        b_R_EC[9 * b_i + 1] = -ship_to_UAV_ned_x;
        b_R_EC[9 * b_i + 4] = Relative_psi_UAV_to_SHIP;
        b_R_EC[9 * b_i + 7] = 0.0;
        b_R_EC[9 * b_i + 2] = 0.0;
        b_R_EC[9 * b_i + 5] = 0.0;
        b_R_EC[9 * b_i + 8] = 1.0;
        UAV_to_SHIP_dist_norm = optimal_UAV_path_acc[3 * b_i];
        ship_to_UAV_ned_x = optimal_UAV_path_acc[i];
        i = 3 * b_i + 2;
        for (i1 = 0; i1 < 3; i1++) {
          UAV_path_acc_control_rf_tmp = i1 + 9 * b_i;
          chosen_approach_heading_rad = b_R_EC[UAV_path_acc_control_rf_tmp];
          V_err_NED_idx_1 = chosen_approach_heading_rad * d;
          V_err_NED_idx_2 = chosen_approach_heading_rad * UAV_to_SHIP_dist_norm;
          chosen_approach_heading_rad = b_R_EC[UAV_path_acc_control_rf_tmp + 3];
          V_err_NED_idx_1 += chosen_approach_heading_rad * V_err_NED_idx_0;
          V_err_NED_idx_2 += chosen_approach_heading_rad * ship_to_UAV_ned_x;
          chosen_approach_heading_rad = b_R_EC[UAV_path_acc_control_rf_tmp + 6];
          V_err_NED_idx_1 += chosen_approach_heading_rad * UAV_path_acc_NED[i];
          V_err_NED_idx_2 +=
              chosen_approach_heading_rad * optimal_UAV_path_acc[i];
          UAV_path_acc_control_rf_tmp = i1 + 3 * b_i;
          UAV_path_acc_control_rf[UAV_path_acc_control_rf_tmp] =
              V_err_NED_idx_2;
          UAV_path_speed_control_rf[UAV_path_acc_control_rf_tmp] =
              V_err_NED_idx_1;
        }
        x[b_i] = (UAV_path_speed_control_rf[3 * b_i] > v_max_control_rf[0]);
      }
      y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_speed_control_rf[3 * i + 1] > v_max_control_rf[1]);
      }
      b_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          b_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_speed_control_rf[3 * i + 2] > v_max_control_rf[2]);
      }
      c_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          c_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_speed_control_rf[3 * i] < v_min_control_rf[0]);
      }
      d_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          d_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_speed_control_rf[3 * i + 1] < v_min_control_rf[1]);
      }
      e_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          e_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_speed_control_rf[3 * i + 2] < v_min_control_rf[2]);
      }
      f_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          f_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      V_OOB[0] = y;
      V_OOB[1] = b_y;
      V_OOB[2] = c_y;
      V_OOB[3] = d_y;
      V_OOB[4] = e_y;
      V_OOB[5] = f_y;
      /* Accellerations Out Of Bounds: */
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_acc_control_rf[3 * i] > a_max_control_rf[0]);
      }
      y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_acc_control_rf[3 * i + 1] > a_max_control_rf[1]);
      }
      b_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          b_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_acc_control_rf[3 * i + 2] > a_max_control_rf[2]);
      }
      c_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          c_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_acc_control_rf[3 * i] < a_min_control_rf[0]);
      }
      d_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          d_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_acc_control_rf[3 * i + 1] < a_min_control_rf[1]);
      }
      e_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          e_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      for (i = 0; i < 20; i++) {
        x[i] = (UAV_path_acc_control_rf[3 * i + 2] < a_min_control_rf[2]);
      }
      f_y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 20)) {
        if (x[UAV_path_acc_control_rf_tmp]) {
          f_y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      A_OOB[0] = y;
      A_OOB[1] = b_y;
      A_OOB[2] = c_y;
      A_OOB[3] = d_y;
      A_OOB[4] = e_y;
      A_OOB[5] = f_y;
      for (i = 0; i < 6; i++) {
        b_x[i] = (signed char)V_OOB[i];
        b_x[i + 6] = (signed char)A_OOB[i];
      }
      y = false;
      UAV_path_acc_control_rf_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 12)) {
        if (b_x[UAV_path_acc_control_rf_tmp] != 0) {
          y = true;
          exitg1 = true;
        } else {
          UAV_path_acc_control_rf_tmp++;
        }
      }
      if (!y) {
        b_exitflag_approach_path = 1;
      }
    }
    /*  Display the total travel time and computational time.  */
    *exitflag_approach_path = b_exitflag_approach_path;
  } else {
    landing_time = c_optimal_linear_coeff_fcn_calc(
        coeffs_ship_prediction_speed_7, P0_UAV_NED, V0_UAV_NED, A0_UAV_NED,
        PhiThetaPsi_SHIP_rad[2], P0_SHIP_NED, v_max_control_rf,
        v_min_control_rf, a_max_control_rf, a_min_control_rf,
        max_time_of_landing_seconds, t_delay_ship_prediction, optimal_coeffs,
        exitflag_approach_path, V_OOB, A_OOB);
  }
  evaluate_UAV_path_from_coeffs(
      optimal_coeffs, landing_time, UAV_path_speed_control_rf,
      UAV_path_acc_control_rf, UAV_path_acc_NED, UAV_path_heading);
  ship_to_UAV_ned_x = sin(psi_rad_UAV);
  Relative_psi_UAV_to_SHIP = cos(psi_rad_UAV);
  R_EC[0] = Relative_psi_UAV_to_SHIP;
  R_EC[3] = ship_to_UAV_ned_x;
  R_EC[6] = 0.0;
  R_EC[1] = -ship_to_UAV_ned_x;
  R_EC[4] = Relative_psi_UAV_to_SHIP;
  R_EC[7] = 0.0;
  /* Initialize the expected_landing_time_relative to be invalid:  */
  *expected_landing_time_relative = -1.0;
  /* If we are far away, initialize the diagonal landing starting from the UAV
   */
  /* altitude and initial UAV position: */
  *approach_ship_mode = 0.0;
  *Desired_phi_rad = 0.0;
  *Desired_theta_rad = 0.0;
  R_EC[2] = 0.0;
  UAV_to_SHIP_dist_NED[0] = P0_UAV_NED[0] - P0_SHIP_NED[0];
  d = P0_SHIP_NED[0] + c_NED_offset_end_point_diag_lan[0];
  P0_SHIP_NED_w_offset[0] = d;
  UAV_to_SHIP_dist_NED_w_offset[0] = P0_UAV_NED[0] - d;
  R_EC[5] = 0.0;
  UAV_to_SHIP_dist_NED[1] = P0_UAV_NED[1] - P0_SHIP_NED[1];
  d = P0_SHIP_NED[1] + c_NED_offset_end_point_diag_lan[1];
  P0_SHIP_NED_w_offset[1] = d;
  UAV_to_SHIP_dist_NED_w_offset[1] = P0_UAV_NED[1] - d;
  R_EC[8] = 1.0;
  UAV_to_SHIP_dist_NED[2] = P0_UAV_NED[2] - P0_SHIP_NED[2];
  d = P0_SHIP_NED[2] + c_NED_offset_end_point_diag_lan[2];
  UAV_to_SHIP_dist_NED_w_offset[2] = P0_UAV_NED[2] - d;
  UAV_to_SHIP_dist_norm = b_norm(UAV_to_SHIP_dist_NED);
  *UAV_to_SHIP_azimuth_angle_rad = 0.0;
  *UAV_to_SHIP_elevation_angle_rad = 0.0;
  V_err_NED_idx_0 = b_norm(UAV_to_SHIP_dist_NED_w_offset);
  if ((V_err_NED_idx_0 > 0.1) &&
      (fabs(UAV_to_SHIP_dist_NED_w_offset[0]) > 0.0)) {
    *UAV_to_SHIP_azimuth_angle_rad = atan2(UAV_to_SHIP_dist_NED_w_offset[0],
                                           UAV_to_SHIP_dist_NED_w_offset[1]);
    *UAV_to_SHIP_elevation_angle_rad =
        asin(UAV_to_SHIP_dist_NED_w_offset[2] / V_err_NED_idx_0);
  }
  /* Generate references for being on the diagonal line:  */
  /* Add the component to stay on the desired line:  */
  chosen_approach_heading_rad =
      PhiThetaPsi_SHIP_rad[2] + approach_heading_ship_rad;
  /*  Calculate wp_diff and pos_diff */
  ship_to_UAV_ned_x_tmp = P0_SHIP_NED_w_offset[0] - P0_UAV_NED[0];
  ship_to_UAV_ned_x = ship_to_UAV_ned_x_tmp;
  /* Calculate the heading of the UAV to ship line:  */
  if (fabs(ship_to_UAV_ned_x_tmp) < 0.1) {
    ship_to_UAV_ned_x = 0.1;
  }
  psi_UAV_to_ship_tmp = P0_SHIP_NED_w_offset[1] - P0_UAV_NED[1];
  *psi_UAV_to_ship = atan2(psi_UAV_to_ship_tmp, ship_to_UAV_ned_x);
  /* Calculate the error between the angles:  */
  *delta_psi = chosen_approach_heading_rad - *psi_UAV_to_ship;
  V_err_NED_idx_0_tmp =
      -diag_approach_speed_m_s * cos(*UAV_to_SHIP_elevation_angle_rad);
  Relative_psi_UAV_to_SHIP =
      fmin(max_line_gain, dist_line_gain / V_err_NED_idx_0) *
      UAV_to_SHIP_dist_norm * sin(*delta_psi);
  V_err_NED_idx_0 =
      ((V_err_NED_idx_0_tmp * sin(*UAV_to_SHIP_azimuth_angle_rad) +
        average_speed_NED_ship[0]) +
       Relative_psi_UAV_to_SHIP *
           cos(chosen_approach_heading_rad - 1.5707963267948966)) -
      V0_UAV_NED[0];
  V_err_NED_idx_1 =
      ((V_err_NED_idx_0_tmp * cos(*UAV_to_SHIP_azimuth_angle_rad) +
        average_speed_NED_ship[1]) +
       Relative_psi_UAV_to_SHIP *
           sin(chosen_approach_heading_rad - 1.5707963267948966)) -
      V0_UAV_NED[1];
  V_err_NED_idx_2 =
      (-diag_approach_speed_m_s * sin(*UAV_to_SHIP_elevation_angle_rad) +
       average_speed_NED_ship[2]) -
      V0_UAV_NED[2];
  UAV_to_SHIP_dist_NED_w_offset[0] =
      V_err_NED_idx_0 * speed_gain_landing - A0_UAV_NED_FILT[0];
  UAV_to_SHIP_dist_NED_w_offset[1] =
      V_err_NED_idx_1 * speed_gain_landing - A0_UAV_NED_FILT[1];
  UAV_to_SHIP_dist_NED_w_offset[2] =
      V_err_NED_idx_2 * speed_gain_landing - A0_UAV_NED_FILT[2];
  /* If we are close enough, switch from line follow to landing pad position */
  /* tracking:  */
  /* If we are close enough to the landing pad, then enter the flare low mode:
   */
  if (UAV_to_SHIP_dist_norm <= pos_tracking_distance_m) {
    *approach_ship_mode = 1.0;
    /* Track ship position and speed of the landing pad:  */
    V_err_NED_idx_2 =
        (ship_to_UAV_ned_x_tmp * pos_gain_landing + V0_SHIP_NED[0]) -
        V0_UAV_NED[0];
    V_err_NED_idx_0 = V_err_NED_idx_2;
    UAV_to_SHIP_dist_NED_w_offset[0] =
        V_err_NED_idx_2 * speed_gain_landing - A0_UAV_NED_FILT[0];
    V_err_NED_idx_2 =
        (psi_UAV_to_ship_tmp * pos_gain_landing + V0_SHIP_NED[1]) -
        V0_UAV_NED[1];
    V_err_NED_idx_1 = V_err_NED_idx_2;
    UAV_to_SHIP_dist_NED_w_offset[1] =
        V_err_NED_idx_2 * speed_gain_landing - A0_UAV_NED_FILT[1];
    V_err_NED_idx_2 =
        ((d - P0_UAV_NED[2]) * pos_gain_landing + V0_SHIP_NED[2]) -
        V0_UAV_NED[2];
    UAV_to_SHIP_dist_NED_w_offset[2] =
        V_err_NED_idx_2 * speed_gain_landing - A0_UAV_NED_FILT[2];
  }
  /* If a landing time is available, then feed the path provided by the */
  /* polynomio: */
  if (*exitflag_approach_path == 1.0) {
    *expected_landing_time_relative = landing_time;
    *approach_ship_mode = 2.0;
    /* Generate the errors accounting for the references and the linear EC:  */
    V_err_NED_idx_2 =
        (UAV_path_acc_control_rf[3] +
         (UAV_path_speed_control_rf[3] - P0_UAV_NED[0]) * pos_gain_landing) -
        V0_UAV_NED[0];
    V_err_NED_idx_0 = V_err_NED_idx_2;
    UAV_to_SHIP_dist_NED_w_offset[0] =
        (UAV_path_acc_NED[3] + V_err_NED_idx_2 * speed_gain_landing) -
        A0_UAV_NED_FILT[0];
    V_err_NED_idx_2 =
        (UAV_path_acc_control_rf[4] +
         (UAV_path_speed_control_rf[4] - P0_UAV_NED[1]) * pos_gain_landing) -
        V0_UAV_NED[1];
    V_err_NED_idx_1 = V_err_NED_idx_2;
    UAV_to_SHIP_dist_NED_w_offset[1] =
        (UAV_path_acc_NED[4] + V_err_NED_idx_2 * speed_gain_landing) -
        A0_UAV_NED_FILT[1];
    V_err_NED_idx_2 =
        (UAV_path_acc_control_rf[5] +
         (UAV_path_speed_control_rf[5] - P0_UAV_NED[2]) * pos_gain_landing) -
        V0_UAV_NED[2];
    UAV_to_SHIP_dist_NED_w_offset[2] =
        (UAV_path_acc_NED[5] + V_err_NED_idx_2 * speed_gain_landing) -
        A0_UAV_NED_FILT[2];
  }
  /* If we are close enough to the landing pad, then enter the flare low mode:
   */
  if ((UAV_to_SHIP_dist_norm <= flare_low_distance_m) ||
      (approach_ship_mode_old == 3.0)) {
    *approach_ship_mode = 3.0;
    /* Track ship position, speed and attitude, just apply a slight vertical
     * speed:  */
    V_err_NED_idx_0 =
        ((P0_SHIP_NED[0] - P0_UAV_NED[0]) * pos_gain_landing + V0_SHIP_NED[0]) -
        V0_UAV_NED[0];
    V_err_NED_idx_1 =
        ((P0_SHIP_NED[1] - P0_UAV_NED[1]) * pos_gain_landing + V0_SHIP_NED[1]) -
        V0_UAV_NED[1];
    V_err_NED_idx_2 = (V0_SHIP_NED[2] + v_speed_docking_m_s) - V0_UAV_NED[2];
    UAV_to_SHIP_dist_NED_w_offset[0] =
        V_err_NED_idx_0 * speed_gain_landing - A0_UAV_NED_FILT[0];
    UAV_to_SHIP_dist_NED_w_offset[1] =
        V_err_NED_idx_1 * speed_gain_landing - A0_UAV_NED_FILT[1];
    UAV_to_SHIP_dist_NED_w_offset[2] =
        V_err_NED_idx_2 * speed_gain_landing - A0_UAV_NED_FILT[2];
    Relative_psi_UAV_to_SHIP = psi_rad_UAV - PhiThetaPsi_SHIP_rad[2];
    V_err_NED_idx_0_tmp = sin(Relative_psi_UAV_to_SHIP);
    ship_to_UAV_ned_x = cos(Relative_psi_UAV_to_SHIP);
    *Desired_phi_rad = PhiThetaPsi_SHIP_rad[0] * ship_to_UAV_ned_x +
                       PhiThetaPsi_SHIP_rad[1] * V_err_NED_idx_0_tmp;
    *Desired_theta_rad = -PhiThetaPsi_SHIP_rad[0] * V_err_NED_idx_0_tmp +
                         PhiThetaPsi_SHIP_rad[1] * ship_to_UAV_ned_x;
  }
  V_err_NED_idx_0 += V0_UAV_NED[0];
  V_err_NED_idx_1 += V0_UAV_NED[1];
  V_err_NED_idx_2 += V0_UAV_NED[2];
  for (i = 0; i < 3; i++) {
    V_target_control[i] =
        (R_EC[i] * V_err_NED_idx_0 + R_EC[i + 3] * V_err_NED_idx_1) +
        R_EC[i + 6] * V_err_NED_idx_2;
    P0_SHIP_NED_w_offset[i] =
        UAV_to_SHIP_dist_NED_w_offset[i] + A0_UAV_NED_FILT[i];
  }
  /* Transpose errors in control rf: */
  d = P0_SHIP_NED_w_offset[0];
  V_err_NED_idx_0 = P0_SHIP_NED_w_offset[1];
  UAV_to_SHIP_dist_norm = P0_SHIP_NED_w_offset[2];
  ship_to_UAV_ned_x = UAV_to_SHIP_dist_NED_w_offset[0];
  chosen_approach_heading_rad = UAV_to_SHIP_dist_NED_w_offset[1];
  V_err_NED_idx_1 = UAV_to_SHIP_dist_NED_w_offset[2];
  for (i = 0; i < 3; i++) {
    V_err_NED_idx_2 = R_EC[i];
    Relative_psi_UAV_to_SHIP = V_err_NED_idx_2 * d;
    V_err_NED_idx_0_tmp = V_err_NED_idx_2 * ship_to_UAV_ned_x;
    V_err_NED_idx_2 = R_EC[i + 3];
    Relative_psi_UAV_to_SHIP += V_err_NED_idx_2 * V_err_NED_idx_0;
    V_err_NED_idx_0_tmp += V_err_NED_idx_2 * chosen_approach_heading_rad;
    V_err_NED_idx_2 = R_EC[i + 6];
    Relative_psi_UAV_to_SHIP += V_err_NED_idx_2 * UAV_to_SHIP_dist_norm;
    V_err_NED_idx_0_tmp += V_err_NED_idx_2 * V_err_NED_idx_1;
    A_err_control_rf[i] = V_err_NED_idx_0_tmp;
    A_target_control[i] = Relative_psi_UAV_to_SHIP;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void nav_approach_ship_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void nav_approach_ship_terminate(void)
{
}

/*
 * File trailer for nav_approach_ship.c
 *
 * [EOF]
 */
