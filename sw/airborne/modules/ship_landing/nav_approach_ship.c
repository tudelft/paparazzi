/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nav_approach_ship.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Dec-2024 14:21:21
 */

/* Include Files */
#include "nav_approach_ship.h"
#include <math.h>

/* Function Declarations */
static double c_optimal_linear_coeff_fcn_calc(
    const double coeffs_ship_prediction[24], const double P0_NED[3],
    const double V0_NED[3], double Psi_ship_rad, const double P0_SHIP_NED[3],
    const double v_max_control_rf[3], const double v_min_control_rf[3],
    const double a_max_control_rf[3], const double a_min_control_rf[3],
    double max_time_of_landing_seconds, double t_delay_ship_prediction,
    double optimal_coeffs[18], double *exitflag, double V_OOB[6],
    double A_OOB[6]);

static void
evaluate_UAV_path_from_coeffs(const double coeffs[18], double landing_time,
                              double UAV_path[60], double UAV_path_speed[60],
                              double UAV_path_acc[60], double t_array[20]);

/* Function Definitions */
/*
 * Prepare variables for funcion:
 *
 * Arguments    : const double coeffs_ship_prediction[24]
 *                const double P0_NED[3]
 *                const double V0_NED[3]
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
    const double V0_NED[3], double Psi_ship_rad, const double P0_SHIP_NED[3],
    const double v_max_control_rf[3], const double v_min_control_rf[3],
    const double a_max_control_rf[3], const double a_min_control_rf[3],
    double max_time_of_landing_seconds, double t_delay_ship_prediction,
    double optimal_coeffs[18], double *exitflag, double V_OOB[6],
    double A_OOB[6])
{
  double Vx_0_UAV;
  double Vy_0_UAV;
  double Vz_0_UAV;
  double X_0_UAV;
  double Y_0_UAV;
  double Z_0_UAV;
  double coeff_10_ship_prediction;
  double coeff_11_ship_prediction;
  double coeff_12_ship_prediction;
  double coeff_13_ship_prediction;
  double coeff_14_ship_prediction;
  double coeff_15_ship_prediction;
  double coeff_16_ship_prediction;
  double coeff_17_ship_prediction;
  double coeff_18_ship_prediction;
  double coeff_19_ship_prediction;
  double coeff_1_ship_prediction;
  double coeff_20_ship_prediction;
  double coeff_21_ship_prediction;
  double coeff_22_ship_prediction;
  double coeff_23_ship_prediction;
  double coeff_24_ship_prediction;
  double coeff_2_ship_prediction;
  double coeff_3_ship_prediction;
  double coeff_4_ship_prediction;
  double coeff_5_ship_prediction;
  double coeff_6_ship_prediction;
  double coeff_7_ship_prediction;
  double coeff_8_ship_prediction;
  double coeff_9_ship_prediction;
  double landing_time;
  int b_i;
  int i;
  int i1;
  X_0_UAV = P0_NED[0];
  Y_0_UAV = P0_NED[1];
  Z_0_UAV = P0_NED[2];
  Vx_0_UAV = V0_NED[0];
  Vy_0_UAV = V0_NED[1];
  Vz_0_UAV = V0_NED[2];
  coeff_1_ship_prediction = coeffs_ship_prediction[0];
  coeff_2_ship_prediction = coeffs_ship_prediction[1];
  coeff_3_ship_prediction = coeffs_ship_prediction[2];
  coeff_4_ship_prediction = coeffs_ship_prediction[3];
  coeff_5_ship_prediction = coeffs_ship_prediction[4];
  coeff_6_ship_prediction = coeffs_ship_prediction[5];
  coeff_7_ship_prediction = coeffs_ship_prediction[6];
  coeff_8_ship_prediction = coeffs_ship_prediction[7];
  coeff_9_ship_prediction = coeffs_ship_prediction[8];
  coeff_10_ship_prediction = coeffs_ship_prediction[9];
  coeff_11_ship_prediction = coeffs_ship_prediction[10];
  coeff_12_ship_prediction = coeffs_ship_prediction[11];
  coeff_13_ship_prediction = coeffs_ship_prediction[12];
  coeff_14_ship_prediction = coeffs_ship_prediction[13];
  coeff_15_ship_prediction = coeffs_ship_prediction[14];
  coeff_16_ship_prediction = coeffs_ship_prediction[15];
  coeff_17_ship_prediction = coeffs_ship_prediction[16];
  coeff_18_ship_prediction = coeffs_ship_prediction[17];
  coeff_19_ship_prediction = coeffs_ship_prediction[18];
  coeff_20_ship_prediction = coeffs_ship_prediction[19];
  coeff_21_ship_prediction = coeffs_ship_prediction[20];
  coeff_22_ship_prediction = coeffs_ship_prediction[21];
  coeff_23_ship_prediction = coeffs_ship_prediction[22];
  coeff_24_ship_prediction = coeffs_ship_prediction[23];
  /* Init guess coefficients:  */
  optimal_coeffs[3] = 0.0;
  optimal_coeffs[4] = V0_NED[0];
  optimal_coeffs[5] = P0_NED[0];
  optimal_coeffs[9] = 0.0;
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
  optimal_coeffs[15] = 0.0;
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
    double ab_optimal_coeffs_tmp;
    double ac_optimal_coeffs_tmp;
    double b_optimal_coeffs_tmp;
    double bb_optimal_coeffs_tmp;
    double bc_optimal_coeffs_tmp;
    double c_optimal_coeffs_tmp;
    double cb_optimal_coeffs_tmp;
    double cc_optimal_coeffs_tmp;
    double d_optimal_coeffs_tmp;
    double db_optimal_coeffs_tmp;
    double dc_optimal_coeffs_tmp;
    double e_optimal_coeffs_tmp;
    double eb_optimal_coeffs_tmp;
    double ec_optimal_coeffs_tmp;
    double f_optimal_coeffs_tmp;
    double fb_optimal_coeffs_tmp;
    double fc_optimal_coeffs_tmp;
    double g_optimal_coeffs_tmp;
    double gb_optimal_coeffs_tmp;
    double h_optimal_coeffs_tmp;
    double hb_optimal_coeffs_tmp;
    double i_optimal_coeffs_tmp;
    double ib_optimal_coeffs_tmp;
    double j_optimal_coeffs_tmp;
    double jb_optimal_coeffs_tmp;
    double k_optimal_coeffs_tmp;
    double kb_optimal_coeffs_tmp;
    double l_optimal_coeffs_tmp;
    double lb_optimal_coeffs_tmp;
    double m_optimal_coeffs_tmp;
    double mb_optimal_coeffs_tmp;
    double n_optimal_coeffs_tmp;
    double nb_optimal_coeffs_tmp;
    double o_optimal_coeffs_tmp;
    double ob_optimal_coeffs_tmp;
    double optimal_coeffs_tmp;
    double p_optimal_coeffs_tmp;
    double pb_optimal_coeffs_tmp;
    double q_optimal_coeffs_tmp;
    double qb_optimal_coeffs_tmp;
    double r_optimal_coeffs_tmp;
    double rb_optimal_coeffs_tmp;
    double s_optimal_coeffs_tmp;
    double sb_optimal_coeffs_tmp;
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
    double t_optimal_coeffs_tmp;
    double tb_optimal_coeffs_tmp;
    double u_optimal_coeffs_tmp;
    double ub_optimal_coeffs_tmp;
    double v_optimal_coeffs_tmp;
    double vb_optimal_coeffs_tmp;
    double w_optimal_coeffs_tmp;
    double wb_optimal_coeffs_tmp;
    double x_optimal_coeffs_tmp;
    double xb_optimal_coeffs_tmp;
    double y_optimal_coeffs_tmp;
    double yb_optimal_coeffs_tmp;
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
    /* OPTIMAL_LINEAR_COEFF_CALCULATION_W_DELAY_FROM_SPEED */
    /*     COEFFS_OPTIMIZED =
     * OPTIMAL_LINEAR_COEFF_CALCULATION_W_DELAY_FROM_SPEED(Ax_0_UAV,Ay_0_UAV,Az_0_UAV,Psi_ship_rad,Vx_0_UAV,Vy_0_UAV,Vz_0_UAV,X_0_UAV,X_0_SHIP,Y_0_UAV,Y_0_SHIP,Z_0_UAV,Z_0_SHIP,COEFF_1_SHIP_PREDICTION,COEFF_2_SHIP_PREDICTION,COEFF_3_SHIP_PREDICTION,COEFF_4_SHIP_PREDICTION,COEFF_5_SHIP_PREDICTION,COEFF_6_SHIP_PREDICTION,COEFF_7_SHIP_PREDICTION,COEFF_8_SHIP_PREDICTION,COEFF_9_SHIP_PREDICTION,COEFF_10_SHIP_PREDICTION,COEFF_11_SHIP_PREDICTION,COEFF_12_SHIP_PREDICTION,COEFF_13_SHIP_PREDICTION,COEFF_14_SHIP_PREDICTION,COEFF_15_SHIP_PREDICTION,COEFF_16_SHIP_PREDICTION,COEFF_17_SHIP_PREDICTION,COEFF_18_SHIP_PREDICTION,COEFF_19_SHIP_PREDICTION,COEFF_20_SHIP_PREDICTION,COEFF_21_SHIP_PREDICTION,COEFF_22_SHIP_PREDICTION,COEFF_23_SHIP_PREDICTION,COEFF_24_SHIP_PREDICTION,T_DELAY_SHIP_PREDICTION,T_LANDING)
     */
    /*     This function was generated by the Symbolic Math Toolbox
     * version 23.2. */
    /*     09-Sep-2024 13:07:23 */
    t2 = cos(Psi_ship_rad);
    t3 = sin(Psi_ship_rad);
    t4 = t_delay_ship_prediction * t_delay_ship_prediction;
    t5 = pow(t_delay_ship_prediction, 3.0);
    t7 = pow(t_delay_ship_prediction, 5.0);
    t9 = pow(t_delay_ship_prediction, 7.0);
    t10 = landing_time * landing_time;
    t11 = pow(landing_time, 3.0);
    t13 = pow(landing_time, 5.0);
    t15 = pow(landing_time, 7.0);
    t6 = t4 * t4;
    t8 = pow(t4, 3.0);
    t12 = t10 * t10;
    t14 = pow(t10, 3.0);
    t16 = 1.0 / t11;
    t18 = 1.0 / t13;
    t17 = 1.0 / t12;
    t19 = 1.0 / (t17 * t17);
    optimal_coeffs_tmp = coeff_1_ship_prediction * t2;
    b_optimal_coeffs_tmp = coeff_2_ship_prediction * t2;
    c_optimal_coeffs_tmp = coeff_3_ship_prediction * t2;
    d_optimal_coeffs_tmp = coeff_4_ship_prediction * t2;
    e_optimal_coeffs_tmp = coeff_9_ship_prediction * t3;
    f_optimal_coeffs_tmp = coeff_10_ship_prediction * t3;
    g_optimal_coeffs_tmp = coeff_11_ship_prediction * t3;
    h_optimal_coeffs_tmp = coeff_12_ship_prediction * t3;
    i_optimal_coeffs_tmp = coeff_5_ship_prediction * t2;
    j_optimal_coeffs_tmp = coeff_6_ship_prediction * t2;
    k_optimal_coeffs_tmp = coeff_13_ship_prediction * t3;
    l_optimal_coeffs_tmp = coeff_14_ship_prediction * t3;
    m_optimal_coeffs_tmp = coeff_7_ship_prediction * t2;
    n_optimal_coeffs_tmp = coeff_15_ship_prediction * t3;
    o_optimal_coeffs_tmp = X_0_UAV * 840.0 - P0_SHIP_NED[0] * 840.0;
    p_optimal_coeffs_tmp = Vx_0_UAV * landing_time;
    q_optimal_coeffs_tmp = b_optimal_coeffs_tmp * t15;
    r_optimal_coeffs_tmp = c_optimal_coeffs_tmp * t14;
    s_optimal_coeffs_tmp = m_optimal_coeffs_tmp * t10;
    t_optimal_coeffs_tmp = optimal_coeffs_tmp * t19;
    u_optimal_coeffs_tmp = f_optimal_coeffs_tmp * t15;
    v_optimal_coeffs_tmp = g_optimal_coeffs_tmp * t14;
    w_optimal_coeffs_tmp = n_optimal_coeffs_tmp * t10;
    x_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t19;
    y_optimal_coeffs_tmp = coeff_8_ship_prediction * t2 * landing_time;
    ab_optimal_coeffs_tmp = coeff_16_ship_prediction * t3 * landing_time;
    bb_optimal_coeffs_tmp = optimal_coeffs_tmp * t4 * t14;
    cb_optimal_coeffs_tmp = optimal_coeffs_tmp * t8 * t10;
    db_optimal_coeffs_tmp = b_optimal_coeffs_tmp * t7 * t10;
    eb_optimal_coeffs_tmp = c_optimal_coeffs_tmp * t6 * t10;
    fb_optimal_coeffs_tmp = d_optimal_coeffs_tmp * t5 * t10;
    gb_optimal_coeffs_tmp = i_optimal_coeffs_tmp * t4 * t10;
    hb_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t4 * t14;
    ib_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t8 * t10;
    jb_optimal_coeffs_tmp = f_optimal_coeffs_tmp * t7 * t10;
    kb_optimal_coeffs_tmp = g_optimal_coeffs_tmp * t6 * t10;
    lb_optimal_coeffs_tmp = h_optimal_coeffs_tmp * t5 * t10;
    mb_optimal_coeffs_tmp = k_optimal_coeffs_tmp * t4 * t10;
    nb_optimal_coeffs_tmp = optimal_coeffs_tmp * t15 * t_delay_ship_prediction;
    ob_optimal_coeffs_tmp =
        b_optimal_coeffs_tmp * t14 * t_delay_ship_prediction;
    pb_optimal_coeffs_tmp =
        j_optimal_coeffs_tmp * t10 * t_delay_ship_prediction;
    qb_optimal_coeffs_tmp =
        e_optimal_coeffs_tmp * t15 * t_delay_ship_prediction;
    rb_optimal_coeffs_tmp =
        f_optimal_coeffs_tmp * t14 * t_delay_ship_prediction;
    sb_optimal_coeffs_tmp =
        l_optimal_coeffs_tmp * t10 * t_delay_ship_prediction;
    tb_optimal_coeffs_tmp = optimal_coeffs_tmp * t9 * landing_time;
    ub_optimal_coeffs_tmp = b_optimal_coeffs_tmp * t8 * landing_time;
    vb_optimal_coeffs_tmp = c_optimal_coeffs_tmp * t7 * landing_time;
    wb_optimal_coeffs_tmp = d_optimal_coeffs_tmp * t6 * landing_time;
    xb_optimal_coeffs_tmp = i_optimal_coeffs_tmp * t5 * landing_time;
    yb_optimal_coeffs_tmp = j_optimal_coeffs_tmp * t4 * landing_time;
    ac_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t9 * landing_time;
    bc_optimal_coeffs_tmp = f_optimal_coeffs_tmp * t8 * landing_time;
    cc_optimal_coeffs_tmp = g_optimal_coeffs_tmp * t7 * landing_time;
    dc_optimal_coeffs_tmp = h_optimal_coeffs_tmp * t6 * landing_time;
    ec_optimal_coeffs_tmp = k_optimal_coeffs_tmp * t5 * landing_time;
    fc_optimal_coeffs_tmp = l_optimal_coeffs_tmp * t4 * landing_time;
    m_optimal_coeffs_tmp =
        m_optimal_coeffs_tmp * t_delay_ship_prediction * landing_time;
    n_optimal_coeffs_tmp =
        n_optimal_coeffs_tmp * t_delay_ship_prediction * landing_time;
    optimal_coeffs[0] =
        t18 *
        (((((((((((((((((((((((((o_optimal_coeffs_tmp +
                                 p_optimal_coeffs_tmp * 420.0) -
                                q_optimal_coeffs_tmp * 120.0) -
                               r_optimal_coeffs_tmp * 70.0) -
                              d_optimal_coeffs_tmp * t13 * 28.0) -
                             s_optimal_coeffs_tmp * 70.0) -
                            t_optimal_coeffs_tmp * 175.0) +
                           u_optimal_coeffs_tmp * 120.0) +
                          v_optimal_coeffs_tmp * 70.0) +
                         h_optimal_coeffs_tmp * t13 * 28.0) +
                        w_optimal_coeffs_tmp * 70.0) +
                       x_optimal_coeffs_tmp * 175.0) -
                      y_optimal_coeffs_tmp * 420.0) +
                     ab_optimal_coeffs_tmp * 420.0) -
                    bb_optimal_coeffs_tmp * 1470.0) -
                   optimal_coeffs_tmp * t5 * t13 * 980.0) -
                  cb_optimal_coeffs_tmp * 490.0) -
                 b_optimal_coeffs_tmp * t4 * t13 * 420.0) -
                db_optimal_coeffs_tmp * 420.0) -
               eb_optimal_coeffs_tmp * 350.0) -
              fb_optimal_coeffs_tmp * 280.0) -
             gb_optimal_coeffs_tmp * 210.0) +
            hb_optimal_coeffs_tmp * 1470.0) +
           e_optimal_coeffs_tmp * t5 * t13 * 980.0) +
          ((((((((((((((((((((((ib_optimal_coeffs_tmp * 490.0 +
                                f_optimal_coeffs_tmp * t4 * t13 * 420.0) +
                               jb_optimal_coeffs_tmp * 420.0) +
                              kb_optimal_coeffs_tmp * 350.0) +
                             lb_optimal_coeffs_tmp * 280.0) +
                            mb_optimal_coeffs_tmp * 210.0) -
                           nb_optimal_coeffs_tmp * 840.0) -
                          ob_optimal_coeffs_tmp * 420.0) -
                         c_optimal_coeffs_tmp * t13 * t_delay_ship_prediction *
                             140.0) -
                        pb_optimal_coeffs_tmp * 140.0) +
                       qb_optimal_coeffs_tmp * 840.0) +
                      rb_optimal_coeffs_tmp * 420.0) +
                     g_optimal_coeffs_tmp * t13 * t_delay_ship_prediction *
                         140.0) +
                    sb_optimal_coeffs_tmp * 140.0) -
                   tb_optimal_coeffs_tmp * 420.0) -
                  ub_optimal_coeffs_tmp * 420.0) -
                 vb_optimal_coeffs_tmp * 420.0) -
                wb_optimal_coeffs_tmp * 420.0) -
               xb_optimal_coeffs_tmp * 420.0) -
              yb_optimal_coeffs_tmp * 420.0) +
             ac_optimal_coeffs_tmp * 420.0) +
            bc_optimal_coeffs_tmp * 420.0) +
           cc_optimal_coeffs_tmp * 420.0)) +
         ((((dc_optimal_coeffs_tmp * 420.0 + ec_optimal_coeffs_tmp * 420.0) +
            fc_optimal_coeffs_tmp * 420.0) -
           m_optimal_coeffs_tmp * 420.0) +
          n_optimal_coeffs_tmp * 420.0)) *
        -0.0071428571428571426;
    optimal_coeffs[1] =
        t17 *
        ((((((((((((((((((((((((((o_optimal_coeffs_tmp +
                                  p_optimal_coeffs_tmp * 448.0) -
                                 q_optimal_coeffs_tmp * 64.0) -
                                r_optimal_coeffs_tmp * 28.0) +
                               i_optimal_coeffs_tmp * t12 * 14.0) -
                              s_optimal_coeffs_tmp * 84.0) -
                             t_optimal_coeffs_tmp * 105.0) +
                            u_optimal_coeffs_tmp * 64.0) +
                           v_optimal_coeffs_tmp * 28.0) -
                          k_optimal_coeffs_tmp * t12 * 14.0) +
                         w_optimal_coeffs_tmp * 84.0) +
                        x_optimal_coeffs_tmp * 105.0) -
                       y_optimal_coeffs_tmp * 448.0) +
                      ab_optimal_coeffs_tmp * 448.0) -
                     bb_optimal_coeffs_tmp * 588.0) +
                    optimal_coeffs_tmp * t6 * t12 * 490.0) -
                   cb_optimal_coeffs_tmp * 588.0) +
                  b_optimal_coeffs_tmp * t5 * t12 * 280.0) -
                 db_optimal_coeffs_tmp * 504.0) +
                c_optimal_coeffs_tmp * t4 * t12 * 140.0) -
               eb_optimal_coeffs_tmp * 420.0) -
              fb_optimal_coeffs_tmp * 336.0) -
             gb_optimal_coeffs_tmp * 252.0) +
            hb_optimal_coeffs_tmp * 588.0) -
           e_optimal_coeffs_tmp * t6 * t12 * 490.0) +
          ((((((((((((((((((((((ib_optimal_coeffs_tmp * 588.0 -
                                f_optimal_coeffs_tmp * t5 * t12 * 280.0) +
                               jb_optimal_coeffs_tmp * 504.0) -
                              g_optimal_coeffs_tmp * t4 * t12 * 140.0) +
                             kb_optimal_coeffs_tmp * 420.0) +
                            lb_optimal_coeffs_tmp * 336.0) +
                           mb_optimal_coeffs_tmp * 252.0) -
                          nb_optimal_coeffs_tmp * 448.0) -
                         ob_optimal_coeffs_tmp * 168.0) +
                        d_optimal_coeffs_tmp * t12 * t_delay_ship_prediction *
                            56.0) -
                       pb_optimal_coeffs_tmp * 168.0) +
                      qb_optimal_coeffs_tmp * 448.0) +
                     rb_optimal_coeffs_tmp * 168.0) -
                    h_optimal_coeffs_tmp * t12 * t_delay_ship_prediction *
                        56.0) +
                   sb_optimal_coeffs_tmp * 168.0) -
                  tb_optimal_coeffs_tmp * 448.0) -
                 ub_optimal_coeffs_tmp * 448.0) -
                vb_optimal_coeffs_tmp * 448.0) -
               wb_optimal_coeffs_tmp * 448.0) -
              xb_optimal_coeffs_tmp * 448.0) -
             yb_optimal_coeffs_tmp * 448.0) +
            ac_optimal_coeffs_tmp * 448.0) +
           bc_optimal_coeffs_tmp * 448.0)) +
         (((((cc_optimal_coeffs_tmp * 448.0 + dc_optimal_coeffs_tmp * 448.0) +
             ec_optimal_coeffs_tmp * 448.0) +
            fc_optimal_coeffs_tmp * 448.0) -
           m_optimal_coeffs_tmp * 448.0) +
          n_optimal_coeffs_tmp * 448.0)) /
        56.0;
    optimal_coeffs[2] =
        t16 *
        ((((((((((((((((((((((((((o_optimal_coeffs_tmp +
                                  p_optimal_coeffs_tmp * 504.0) -
                                 q_optimal_coeffs_tmp * 36.0) -
                                r_optimal_coeffs_tmp * 14.0) -
                               j_optimal_coeffs_tmp * t11 * 28.0) -
                              s_optimal_coeffs_tmp * 126.0) -
                             t_optimal_coeffs_tmp * 63.0) +
                            u_optimal_coeffs_tmp * 36.0) +
                           v_optimal_coeffs_tmp * 14.0) +
                          l_optimal_coeffs_tmp * t11 * 28.0) +
                         w_optimal_coeffs_tmp * 126.0) +
                        x_optimal_coeffs_tmp * 63.0) -
                       y_optimal_coeffs_tmp * 504.0) +
                      ab_optimal_coeffs_tmp * 504.0) -
                     bb_optimal_coeffs_tmp * 294.0) -
                    optimal_coeffs_tmp * t7 * t11 * 588.0) -
                   cb_optimal_coeffs_tmp * 882.0) -
                  b_optimal_coeffs_tmp * t6 * t11 * 420.0) -
                 db_optimal_coeffs_tmp * 756.0) -
                c_optimal_coeffs_tmp * t5 * t11 * 280.0) -
               eb_optimal_coeffs_tmp * 630.0) -
              d_optimal_coeffs_tmp * t4 * t11 * 168.0) -
             fb_optimal_coeffs_tmp * 504.0) -
            gb_optimal_coeffs_tmp * 378.0) +
           hb_optimal_coeffs_tmp * 294.0) +
          ((((((((((((((((((((((e_optimal_coeffs_tmp * t7 * t11 * 588.0 +
                                ib_optimal_coeffs_tmp * 882.0) +
                               f_optimal_coeffs_tmp * t6 * t11 * 420.0) +
                              jb_optimal_coeffs_tmp * 756.0) +
                             g_optimal_coeffs_tmp * t5 * t11 * 280.0) +
                            kb_optimal_coeffs_tmp * 630.0) +
                           h_optimal_coeffs_tmp * t4 * t11 * 168.0) +
                          lb_optimal_coeffs_tmp * 504.0) +
                         mb_optimal_coeffs_tmp * 378.0) -
                        nb_optimal_coeffs_tmp * 252.0) -
                       ob_optimal_coeffs_tmp * 84.0) -
                      i_optimal_coeffs_tmp * t11 * t_delay_ship_prediction *
                          84.0) -
                     pb_optimal_coeffs_tmp * 252.0) +
                    qb_optimal_coeffs_tmp * 252.0) +
                   rb_optimal_coeffs_tmp * 84.0) +
                  k_optimal_coeffs_tmp * t11 * t_delay_ship_prediction * 84.0) +
                 sb_optimal_coeffs_tmp * 252.0) -
                tb_optimal_coeffs_tmp * 504.0) -
               ub_optimal_coeffs_tmp * 504.0) -
              vb_optimal_coeffs_tmp * 504.0) -
             wb_optimal_coeffs_tmp * 504.0) -
            xb_optimal_coeffs_tmp * 504.0) -
           yb_optimal_coeffs_tmp * 504.0)) +
         (((((((ac_optimal_coeffs_tmp * 504.0 + bc_optimal_coeffs_tmp * 504.0) +
               cc_optimal_coeffs_tmp * 504.0) +
              dc_optimal_coeffs_tmp * 504.0) +
             ec_optimal_coeffs_tmp * 504.0) +
            fc_optimal_coeffs_tmp * 504.0) -
           m_optimal_coeffs_tmp * 504.0) +
          n_optimal_coeffs_tmp * 504.0)) *
        -0.011904761904761904;
    optimal_coeffs[3] = 0.0;
    optimal_coeffs[4] = Vx_0_UAV;
    optimal_coeffs[5] = X_0_UAV;
    optimal_coeffs_tmp = coeff_1_ship_prediction * t3;
    b_optimal_coeffs_tmp = coeff_2_ship_prediction * t3;
    c_optimal_coeffs_tmp = coeff_3_ship_prediction * t3;
    d_optimal_coeffs_tmp = coeff_4_ship_prediction * t3;
    e_optimal_coeffs_tmp = coeff_9_ship_prediction * t2;
    f_optimal_coeffs_tmp = coeff_10_ship_prediction * t2;
    g_optimal_coeffs_tmp = coeff_11_ship_prediction * t2;
    h_optimal_coeffs_tmp = coeff_12_ship_prediction * t2;
    i_optimal_coeffs_tmp = coeff_5_ship_prediction * t3;
    j_optimal_coeffs_tmp = coeff_6_ship_prediction * t3;
    k_optimal_coeffs_tmp = coeff_13_ship_prediction * t2;
    l_optimal_coeffs_tmp = coeff_14_ship_prediction * t2;
    m_optimal_coeffs_tmp = coeff_7_ship_prediction * t3;
    n_optimal_coeffs_tmp = coeff_15_ship_prediction * t2;
    o_optimal_coeffs_tmp = P0_SHIP_NED[1] * 840.0 - Y_0_UAV * 840.0;
    p_optimal_coeffs_tmp = Vy_0_UAV * landing_time;
    q_optimal_coeffs_tmp = b_optimal_coeffs_tmp * t15;
    r_optimal_coeffs_tmp = c_optimal_coeffs_tmp * t14;
    s_optimal_coeffs_tmp = m_optimal_coeffs_tmp * t10;
    t_optimal_coeffs_tmp = optimal_coeffs_tmp * t19;
    u_optimal_coeffs_tmp = f_optimal_coeffs_tmp * t15;
    v_optimal_coeffs_tmp = g_optimal_coeffs_tmp * t14;
    w_optimal_coeffs_tmp = n_optimal_coeffs_tmp * t10;
    x_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t19;
    y_optimal_coeffs_tmp = coeff_8_ship_prediction * t3 * landing_time;
    ab_optimal_coeffs_tmp = coeff_16_ship_prediction * t2 * landing_time;
    bb_optimal_coeffs_tmp = optimal_coeffs_tmp * t4 * t14;
    cb_optimal_coeffs_tmp = optimal_coeffs_tmp * t8 * t10;
    db_optimal_coeffs_tmp = b_optimal_coeffs_tmp * t7 * t10;
    eb_optimal_coeffs_tmp = c_optimal_coeffs_tmp * t6 * t10;
    fb_optimal_coeffs_tmp = d_optimal_coeffs_tmp * t5 * t10;
    gb_optimal_coeffs_tmp = i_optimal_coeffs_tmp * t4 * t10;
    hb_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t4 * t14;
    ib_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t8 * t10;
    jb_optimal_coeffs_tmp = f_optimal_coeffs_tmp * t7 * t10;
    kb_optimal_coeffs_tmp = g_optimal_coeffs_tmp * t6 * t10;
    lb_optimal_coeffs_tmp = h_optimal_coeffs_tmp * t5 * t10;
    mb_optimal_coeffs_tmp = k_optimal_coeffs_tmp * t4 * t10;
    nb_optimal_coeffs_tmp = optimal_coeffs_tmp * t15 * t_delay_ship_prediction;
    ob_optimal_coeffs_tmp =
        b_optimal_coeffs_tmp * t14 * t_delay_ship_prediction;
    pb_optimal_coeffs_tmp =
        j_optimal_coeffs_tmp * t10 * t_delay_ship_prediction;
    qb_optimal_coeffs_tmp =
        e_optimal_coeffs_tmp * t15 * t_delay_ship_prediction;
    rb_optimal_coeffs_tmp =
        f_optimal_coeffs_tmp * t14 * t_delay_ship_prediction;
    sb_optimal_coeffs_tmp =
        l_optimal_coeffs_tmp * t10 * t_delay_ship_prediction;
    tb_optimal_coeffs_tmp = optimal_coeffs_tmp * t9 * landing_time;
    ub_optimal_coeffs_tmp = b_optimal_coeffs_tmp * t8 * landing_time;
    vb_optimal_coeffs_tmp = c_optimal_coeffs_tmp * t7 * landing_time;
    wb_optimal_coeffs_tmp = d_optimal_coeffs_tmp * t6 * landing_time;
    xb_optimal_coeffs_tmp = i_optimal_coeffs_tmp * t5 * landing_time;
    yb_optimal_coeffs_tmp = j_optimal_coeffs_tmp * t4 * landing_time;
    ac_optimal_coeffs_tmp = e_optimal_coeffs_tmp * t9 * landing_time;
    bc_optimal_coeffs_tmp = f_optimal_coeffs_tmp * t8 * landing_time;
    cc_optimal_coeffs_tmp = g_optimal_coeffs_tmp * t7 * landing_time;
    dc_optimal_coeffs_tmp = h_optimal_coeffs_tmp * t6 * landing_time;
    ec_optimal_coeffs_tmp = k_optimal_coeffs_tmp * t5 * landing_time;
    fc_optimal_coeffs_tmp = l_optimal_coeffs_tmp * t4 * landing_time;
    m_optimal_coeffs_tmp =
        m_optimal_coeffs_tmp * t_delay_ship_prediction * landing_time;
    n_optimal_coeffs_tmp =
        n_optimal_coeffs_tmp * t_delay_ship_prediction * landing_time;
    optimal_coeffs[6] =
        t18 *
        (((((((((((((((((((((((((((o_optimal_coeffs_tmp -
                                   p_optimal_coeffs_tmp * 420.0) +
                                  q_optimal_coeffs_tmp * 120.0) +
                                 r_optimal_coeffs_tmp * 70.0) +
                                d_optimal_coeffs_tmp * t13 * 28.0) +
                               s_optimal_coeffs_tmp * 70.0) +
                              t_optimal_coeffs_tmp * 175.0) +
                             u_optimal_coeffs_tmp * 120.0) +
                            v_optimal_coeffs_tmp * 70.0) +
                           h_optimal_coeffs_tmp * t13 * 28.0) +
                          w_optimal_coeffs_tmp * 70.0) +
                         x_optimal_coeffs_tmp * 175.0) +
                        y_optimal_coeffs_tmp * 420.0) +
                       ab_optimal_coeffs_tmp * 420.0) +
                      bb_optimal_coeffs_tmp * 1470.0) +
                     optimal_coeffs_tmp * t5 * t13 * 980.0) +
                    cb_optimal_coeffs_tmp * 490.0) +
                   b_optimal_coeffs_tmp * t4 * t13 * 420.0) +
                  db_optimal_coeffs_tmp * 420.0) +
                 eb_optimal_coeffs_tmp * 350.0) +
                fb_optimal_coeffs_tmp * 280.0) +
               gb_optimal_coeffs_tmp * 210.0) +
              hb_optimal_coeffs_tmp * 1470.0) +
             e_optimal_coeffs_tmp * t5 * t13 * 980.0) +
            ib_optimal_coeffs_tmp * 490.0) +
           f_optimal_coeffs_tmp * t4 * t13 * 420.0) +
          (((((((((((((((((((((((jb_optimal_coeffs_tmp * 420.0 +
                                 kb_optimal_coeffs_tmp * 350.0) +
                                lb_optimal_coeffs_tmp * 280.0) +
                               mb_optimal_coeffs_tmp * 210.0) +
                              nb_optimal_coeffs_tmp * 840.0) +
                             ob_optimal_coeffs_tmp * 420.0) +
                            c_optimal_coeffs_tmp * t13 *
                                t_delay_ship_prediction * 140.0) +
                           pb_optimal_coeffs_tmp * 140.0) +
                          qb_optimal_coeffs_tmp * 840.0) +
                         rb_optimal_coeffs_tmp * 420.0) +
                        g_optimal_coeffs_tmp * t13 * t_delay_ship_prediction *
                            140.0) +
                       sb_optimal_coeffs_tmp * 140.0) +
                      tb_optimal_coeffs_tmp * 420.0) +
                     ub_optimal_coeffs_tmp * 420.0) +
                    vb_optimal_coeffs_tmp * 420.0) +
                   wb_optimal_coeffs_tmp * 420.0) +
                  xb_optimal_coeffs_tmp * 420.0) +
                 yb_optimal_coeffs_tmp * 420.0) +
                ac_optimal_coeffs_tmp * 420.0) +
               bc_optimal_coeffs_tmp * 420.0) +
              cc_optimal_coeffs_tmp * 420.0) +
             dc_optimal_coeffs_tmp * 420.0) +
            ec_optimal_coeffs_tmp * 420.0) +
           fc_optimal_coeffs_tmp * 420.0)) +
         (m_optimal_coeffs_tmp * 420.0 + n_optimal_coeffs_tmp * 420.0)) /
        140.0;
    optimal_coeffs[7] =
        t17 *
        ((((((((((((((((((((((((((o_optimal_coeffs_tmp -
                                  p_optimal_coeffs_tmp * 448.0) +
                                 q_optimal_coeffs_tmp * 64.0) +
                                r_optimal_coeffs_tmp * 28.0) -
                               i_optimal_coeffs_tmp * t12 * 14.0) +
                              s_optimal_coeffs_tmp * 84.0) +
                             t_optimal_coeffs_tmp * 105.0) +
                            u_optimal_coeffs_tmp * 64.0) +
                           v_optimal_coeffs_tmp * 28.0) -
                          k_optimal_coeffs_tmp * t12 * 14.0) +
                         w_optimal_coeffs_tmp * 84.0) +
                        x_optimal_coeffs_tmp * 105.0) +
                       y_optimal_coeffs_tmp * 448.0) +
                      ab_optimal_coeffs_tmp * 448.0) +
                     bb_optimal_coeffs_tmp * 588.0) -
                    optimal_coeffs_tmp * t6 * t12 * 490.0) +
                   cb_optimal_coeffs_tmp * 588.0) -
                  b_optimal_coeffs_tmp * t5 * t12 * 280.0) +
                 db_optimal_coeffs_tmp * 504.0) -
                c_optimal_coeffs_tmp * t4 * t12 * 140.0) +
               eb_optimal_coeffs_tmp * 420.0) +
              fb_optimal_coeffs_tmp * 336.0) +
             gb_optimal_coeffs_tmp * 252.0) +
            hb_optimal_coeffs_tmp * 588.0) -
           e_optimal_coeffs_tmp * t6 * t12 * 490.0) +
          (((((((((((((((((((((((ib_optimal_coeffs_tmp * 588.0 -
                                 f_optimal_coeffs_tmp * t5 * t12 * 280.0) +
                                jb_optimal_coeffs_tmp * 504.0) -
                               g_optimal_coeffs_tmp * t4 * t12 * 140.0) +
                              kb_optimal_coeffs_tmp * 420.0) +
                             lb_optimal_coeffs_tmp * 336.0) +
                            mb_optimal_coeffs_tmp * 252.0) +
                           nb_optimal_coeffs_tmp * 448.0) +
                          ob_optimal_coeffs_tmp * 168.0) -
                         d_optimal_coeffs_tmp * t12 * t_delay_ship_prediction *
                             56.0) +
                        pb_optimal_coeffs_tmp * 168.0) +
                       qb_optimal_coeffs_tmp * 448.0) +
                      rb_optimal_coeffs_tmp * 168.0) -
                     h_optimal_coeffs_tmp * t12 * t_delay_ship_prediction *
                         56.0) +
                    sb_optimal_coeffs_tmp * 168.0) +
                   tb_optimal_coeffs_tmp * 448.0) +
                  ub_optimal_coeffs_tmp * 448.0) +
                 vb_optimal_coeffs_tmp * 448.0) +
                wb_optimal_coeffs_tmp * 448.0) +
               xb_optimal_coeffs_tmp * 448.0) +
              yb_optimal_coeffs_tmp * 448.0) +
             ac_optimal_coeffs_tmp * 448.0) +
            bc_optimal_coeffs_tmp * 448.0) +
           cc_optimal_coeffs_tmp * 448.0)) +
         ((((dc_optimal_coeffs_tmp * 448.0 + ec_optimal_coeffs_tmp * 448.0) +
            fc_optimal_coeffs_tmp * 448.0) +
           m_optimal_coeffs_tmp * 448.0) +
          n_optimal_coeffs_tmp * 448.0)) *
        -0.017857142857142856;
    optimal_coeffs[8] =
        t16 *
        (((((((((((((((((((((((((((o_optimal_coeffs_tmp -
                                   p_optimal_coeffs_tmp * 504.0) +
                                  q_optimal_coeffs_tmp * 36.0) +
                                 r_optimal_coeffs_tmp * 14.0) +
                                j_optimal_coeffs_tmp * t11 * 28.0) +
                               s_optimal_coeffs_tmp * 126.0) +
                              t_optimal_coeffs_tmp * 63.0) +
                             u_optimal_coeffs_tmp * 36.0) +
                            v_optimal_coeffs_tmp * 14.0) +
                           l_optimal_coeffs_tmp * t11 * 28.0) +
                          w_optimal_coeffs_tmp * 126.0) +
                         x_optimal_coeffs_tmp * 63.0) +
                        y_optimal_coeffs_tmp * 504.0) +
                       ab_optimal_coeffs_tmp * 504.0) +
                      bb_optimal_coeffs_tmp * 294.0) +
                     optimal_coeffs_tmp * t7 * t11 * 588.0) +
                    cb_optimal_coeffs_tmp * 882.0) +
                   b_optimal_coeffs_tmp * t6 * t11 * 420.0) +
                  db_optimal_coeffs_tmp * 756.0) +
                 c_optimal_coeffs_tmp * t5 * t11 * 280.0) +
                eb_optimal_coeffs_tmp * 630.0) +
               d_optimal_coeffs_tmp * t4 * t11 * 168.0) +
              fb_optimal_coeffs_tmp * 504.0) +
             gb_optimal_coeffs_tmp * 378.0) +
            hb_optimal_coeffs_tmp * 294.0) +
           e_optimal_coeffs_tmp * t7 * t11 * 588.0) +
          (((((((((((((((((((((((ib_optimal_coeffs_tmp * 882.0 +
                                 f_optimal_coeffs_tmp * t6 * t11 * 420.0) +
                                jb_optimal_coeffs_tmp * 756.0) +
                               g_optimal_coeffs_tmp * t5 * t11 * 280.0) +
                              kb_optimal_coeffs_tmp * 630.0) +
                             h_optimal_coeffs_tmp * t4 * t11 * 168.0) +
                            lb_optimal_coeffs_tmp * 504.0) +
                           mb_optimal_coeffs_tmp * 378.0) +
                          nb_optimal_coeffs_tmp * 252.0) +
                         ob_optimal_coeffs_tmp * 84.0) +
                        i_optimal_coeffs_tmp * t11 * t_delay_ship_prediction *
                            84.0) +
                       pb_optimal_coeffs_tmp * 252.0) +
                      qb_optimal_coeffs_tmp * 252.0) +
                     rb_optimal_coeffs_tmp * 84.0) +
                    k_optimal_coeffs_tmp * t11 * t_delay_ship_prediction *
                        84.0) +
                   sb_optimal_coeffs_tmp * 252.0) +
                  tb_optimal_coeffs_tmp * 504.0) +
                 ub_optimal_coeffs_tmp * 504.0) +
                vb_optimal_coeffs_tmp * 504.0) +
               wb_optimal_coeffs_tmp * 504.0) +
              xb_optimal_coeffs_tmp * 504.0) +
             yb_optimal_coeffs_tmp * 504.0) +
            ac_optimal_coeffs_tmp * 504.0) +
           bc_optimal_coeffs_tmp * 504.0)) +
         (((((cc_optimal_coeffs_tmp * 504.0 + dc_optimal_coeffs_tmp * 504.0) +
             ec_optimal_coeffs_tmp * 504.0) +
            fc_optimal_coeffs_tmp * 504.0) +
           m_optimal_coeffs_tmp * 504.0) +
          n_optimal_coeffs_tmp * 504.0)) /
        84.0;
    optimal_coeffs[9] = 0.0;
    optimal_coeffs[10] = Vy_0_UAV;
    optimal_coeffs[11] = Y_0_UAV;
    optimal_coeffs_tmp = P0_SHIP_NED[2] * 840.0 - Z_0_UAV * 840.0;
    b_optimal_coeffs_tmp = Vz_0_UAV * landing_time;
    c_optimal_coeffs_tmp = coeff_18_ship_prediction * t15;
    d_optimal_coeffs_tmp = coeff_19_ship_prediction * t14;
    e_optimal_coeffs_tmp = coeff_23_ship_prediction * t10;
    f_optimal_coeffs_tmp = coeff_17_ship_prediction * t19;
    g_optimal_coeffs_tmp = coeff_24_ship_prediction * landing_time;
    h_optimal_coeffs_tmp = coeff_17_ship_prediction * t4 * t14;
    i_optimal_coeffs_tmp = coeff_17_ship_prediction * t8 * t10;
    j_optimal_coeffs_tmp = coeff_18_ship_prediction * t7 * t10;
    k_optimal_coeffs_tmp = coeff_19_ship_prediction * t6 * t10;
    l_optimal_coeffs_tmp = coeff_20_ship_prediction * t5 * t10;
    m_optimal_coeffs_tmp = coeff_21_ship_prediction * t4 * t10;
    n_optimal_coeffs_tmp =
        coeff_17_ship_prediction * t15 * t_delay_ship_prediction;
    o_optimal_coeffs_tmp =
        coeff_18_ship_prediction * t14 * t_delay_ship_prediction;
    p_optimal_coeffs_tmp =
        coeff_22_ship_prediction * t10 * t_delay_ship_prediction;
    q_optimal_coeffs_tmp = coeff_17_ship_prediction * t9 * landing_time;
    r_optimal_coeffs_tmp = coeff_18_ship_prediction * t8 * landing_time;
    s_optimal_coeffs_tmp = coeff_19_ship_prediction * t7 * landing_time;
    t_optimal_coeffs_tmp = coeff_20_ship_prediction * t6 * landing_time;
    u_optimal_coeffs_tmp = coeff_21_ship_prediction * t5 * landing_time;
    v_optimal_coeffs_tmp = coeff_22_ship_prediction * t4 * landing_time;
    w_optimal_coeffs_tmp =
        coeff_23_ship_prediction * t_delay_ship_prediction * landing_time;
    optimal_coeffs[12] =
        t18 *
        ((((((((((((((((((((((((((optimal_coeffs_tmp -
                                  b_optimal_coeffs_tmp * 420.0) +
                                 c_optimal_coeffs_tmp * 120.0) +
                                d_optimal_coeffs_tmp * 70.0) +
                               coeff_20_ship_prediction * t13 * 28.0) +
                              e_optimal_coeffs_tmp * 70.0) +
                             f_optimal_coeffs_tmp * 175.0) +
                            g_optimal_coeffs_tmp * 420.0) +
                           h_optimal_coeffs_tmp * 1470.0) +
                          coeff_17_ship_prediction * t5 * t13 * 980.0) +
                         i_optimal_coeffs_tmp * 490.0) +
                        coeff_18_ship_prediction * t4 * t13 * 420.0) +
                       j_optimal_coeffs_tmp * 420.0) +
                      k_optimal_coeffs_tmp * 350.0) +
                     l_optimal_coeffs_tmp * 280.0) +
                    m_optimal_coeffs_tmp * 210.0) +
                   n_optimal_coeffs_tmp * 840.0) +
                  o_optimal_coeffs_tmp * 420.0) +
                 coeff_19_ship_prediction * t13 * t_delay_ship_prediction *
                     140.0) +
                p_optimal_coeffs_tmp * 140.0) +
               q_optimal_coeffs_tmp * 420.0) +
              r_optimal_coeffs_tmp * 420.0) +
             s_optimal_coeffs_tmp * 420.0) +
            t_optimal_coeffs_tmp * 420.0) +
           u_optimal_coeffs_tmp * 420.0) +
          v_optimal_coeffs_tmp * 420.0) +
         w_optimal_coeffs_tmp * 420.0) /
        140.0;
    optimal_coeffs[13] =
        t17 *
        (((((((((((((((((((((((((((optimal_coeffs_tmp -
                                   b_optimal_coeffs_tmp * 448.0) +
                                  c_optimal_coeffs_tmp * 64.0) +
                                 d_optimal_coeffs_tmp * 28.0) -
                                coeff_21_ship_prediction * t12 * 14.0) +
                               e_optimal_coeffs_tmp * 84.0) +
                              f_optimal_coeffs_tmp * 105.0) +
                             g_optimal_coeffs_tmp * 448.0) +
                            h_optimal_coeffs_tmp * 588.0) -
                           coeff_17_ship_prediction * t6 * t12 * 490.0) +
                          i_optimal_coeffs_tmp * 588.0) -
                         coeff_18_ship_prediction * t5 * t12 * 280.0) +
                        j_optimal_coeffs_tmp * 504.0) -
                       coeff_19_ship_prediction * t4 * t12 * 140.0) +
                      k_optimal_coeffs_tmp * 420.0) +
                     l_optimal_coeffs_tmp * 336.0) +
                    m_optimal_coeffs_tmp * 252.0) +
                   n_optimal_coeffs_tmp * 448.0) +
                  o_optimal_coeffs_tmp * 168.0) -
                 coeff_20_ship_prediction * t12 * t_delay_ship_prediction *
                     56.0) +
                p_optimal_coeffs_tmp * 168.0) +
               q_optimal_coeffs_tmp * 448.0) +
              r_optimal_coeffs_tmp * 448.0) +
             s_optimal_coeffs_tmp * 448.0) +
            t_optimal_coeffs_tmp * 448.0) +
           u_optimal_coeffs_tmp * 448.0) +
          v_optimal_coeffs_tmp * 448.0) +
         w_optimal_coeffs_tmp * 448.0) *
        -0.017857142857142856;
    optimal_coeffs[14] =
        t16 *
        ((((((((((((((((((((((((((((optimal_coeffs_tmp -
                                    b_optimal_coeffs_tmp * 504.0) +
                                   c_optimal_coeffs_tmp * 36.0) +
                                  d_optimal_coeffs_tmp * 14.0) +
                                 coeff_22_ship_prediction * t11 * 28.0) +
                                e_optimal_coeffs_tmp * 126.0) +
                               f_optimal_coeffs_tmp * 63.0) +
                              g_optimal_coeffs_tmp * 504.0) +
                             h_optimal_coeffs_tmp * 294.0) +
                            coeff_17_ship_prediction * t7 * t11 * 588.0) +
                           i_optimal_coeffs_tmp * 882.0) +
                          coeff_18_ship_prediction * t6 * t11 * 420.0) +
                         j_optimal_coeffs_tmp * 756.0) +
                        coeff_19_ship_prediction * t5 * t11 * 280.0) +
                       k_optimal_coeffs_tmp * 630.0) +
                      coeff_20_ship_prediction * t4 * t11 * 168.0) +
                     l_optimal_coeffs_tmp * 504.0) +
                    m_optimal_coeffs_tmp * 378.0) +
                   n_optimal_coeffs_tmp * 252.0) +
                  o_optimal_coeffs_tmp * 84.0) +
                 coeff_21_ship_prediction * t11 * t_delay_ship_prediction *
                     84.0) +
                p_optimal_coeffs_tmp * 252.0) +
               q_optimal_coeffs_tmp * 504.0) +
              r_optimal_coeffs_tmp * 504.0) +
             s_optimal_coeffs_tmp * 504.0) +
            t_optimal_coeffs_tmp * 504.0) +
           u_optimal_coeffs_tmp * 504.0) +
          v_optimal_coeffs_tmp * 504.0) +
         w_optimal_coeffs_tmp * 504.0) *
        0.011904761904761904;
    optimal_coeffs[15] = 0.0;
    optimal_coeffs[16] = Vz_0_UAV;
    optimal_coeffs[17] = Z_0_UAV;
    evaluate_UAV_path_from_coeffs(
        optimal_coeffs, landing_time, UAV_path_speed_control_rf,
        optimal_UAV_path_speed, optimal_UAV_path_acc, UAV_path_heading);
    /* Check that the speed and accelerations are always within the bounds: */
    /* Report speeds in the control reference frame:  */
    /* Check if the speed and acceleration are within the bounds: */
    /* Speeds Out Of Bounds: */
    for (b_i = 0; b_i < 20; b_i++) {
      t9 = optimal_UAV_path_speed[3 * b_i];
      i = 3 * b_i + 1;
      t10 = optimal_UAV_path_speed[i];
      t7 = atan2(t10, t9);
      t3 = sin(t7);
      t2 = cos(t7);
      R_EC[9 * b_i] = t2;
      R_EC[9 * b_i + 3] = t3;
      R_EC[9 * b_i + 6] = 0.0;
      R_EC[9 * b_i + 1] = -t3;
      R_EC[9 * b_i + 4] = t2;
      R_EC[9 * b_i + 7] = 0.0;
      R_EC[9 * b_i + 2] = 0.0;
      R_EC[9 * b_i + 5] = 0.0;
      R_EC[9 * b_i + 8] = 1.0;
      t7 = optimal_UAV_path_acc[3 * b_i];
      t2 = optimal_UAV_path_acc[i];
      i = 3 * b_i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        UAV_path_acc_control_rf_tmp = i1 + 9 * b_i;
        t3 = R_EC[UAV_path_acc_control_rf_tmp];
        t4 = t3 * t9;
        t5 = t3 * t7;
        t3 = R_EC[UAV_path_acc_control_rf_tmp + 3];
        t4 += t3 * t10;
        t5 += t3 * t2;
        t3 = R_EC[UAV_path_acc_control_rf_tmp + 6];
        t4 += t3 * optimal_UAV_path_speed[i];
        t5 += t3 * optimal_UAV_path_acc[i];
        UAV_path_acc_control_rf_tmp = i1 + 3 * b_i;
        UAV_path_acc_control_rf[UAV_path_acc_control_rf_tmp] = t5;
        UAV_path_speed_control_rf[UAV_path_acc_control_rf_tmp] = t4;
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
 *                const double pos_gain_landing_array[3]
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
 *                double *expected_landing_time_relative
 *                double optimal_coeffs[18]
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
    const double V0_SHIP_NED[3], const double PhiThetaPsi_SHIP_rad[3],
    const double P0_SHIP_NED[3], double psi_rad_UAV,
    const double v_max_control_rf[3], const double v_min_control_rf[3],
    const double a_max_control_rf[3], const double a_min_control_rf[3],
    const double coeffs_ship_prediction_speed_7[24],
    double t_delay_ship_prediction, double max_time_of_landing_seconds,
    const double pos_gain_landing_array[3], double flare_low_distance_m,
    double v_speed_docking_m_s, const double average_speed_NED_ship[3],
    double diag_approach_speed_m_s, double approach_ship_mode_old,
    const double c_NED_offset_end_point_diag_lan[3],
    double pos_tracking_distance_m, double approach_heading_ship_rad,
    double dist_line_gain, double max_line_gain,
    double *expected_landing_time_relative, double optimal_coeffs[18],
    double V_target_control[3], double *exitflag_approach_path,
    double *approach_ship_mode, double V_OOB[6], double A_OOB[6],
    double UAV_to_SHIP_dist_NED[3], double *Desired_phi_rad,
    double *Desired_theta_rad, double *UAV_to_SHIP_azimuth_angle_rad,
    double *UAV_to_SHIP_elevation_angle_rad, double *delta_psi,
    double *psi_UAV_to_ship)
{
  double UAV_path_NED[60];
  double UAV_path_acc_NED[60];
  double UAV_path_speed_NED[60];
  double a__1[20];
  double c_R_EC_tmp[9];
  double P0_SHIP_NED_w_offset_idx_0;
  double P0_SHIP_NED_w_offset_idx_1;
  double R_EC_tmp;
  double UAV_to_SHIP_dist_norm;
  double absxk;
  double absxk_tmp;
  double b_R_EC_tmp;
  double b_scale;
  double c_UAV_to_SHIP_dist_NED_w_offset;
  double chosen_approach_heading_rad;
  double d;
  double d1;
  double d_UAV_to_SHIP_dist_NED_w_offset;
  double landing_time;
  double scale;
  double t;
  int i;
  /* Run the path planning module based on the prediction coefficients: */
  if (approach_ship_mode_old == 2.0) {
    double b_a_max_control_rf[3];
    double b_a_min_control_rf[3];
    double b_v_max_control_rf[3];
    double b_v_min_control_rf[3];
    b_v_max_control_rf[0] = v_max_control_rf[0] * 1.2;
    b_v_min_control_rf[0] = v_min_control_rf[0] * 1.2;
    b_a_max_control_rf[0] = a_max_control_rf[0] * 1.2;
    b_a_min_control_rf[0] = a_min_control_rf[0] * 1.2;
    b_v_max_control_rf[1] = v_max_control_rf[1] * 1.2;
    b_v_min_control_rf[1] = v_min_control_rf[1] * 1.2;
    b_a_max_control_rf[1] = a_max_control_rf[1] * 1.2;
    b_a_min_control_rf[1] = a_min_control_rf[1] * 1.2;
    b_v_max_control_rf[2] = v_max_control_rf[2] * 1.2;
    b_v_min_control_rf[2] = v_min_control_rf[2] * 1.2;
    b_a_max_control_rf[2] = a_max_control_rf[2] * 1.2;
    b_a_min_control_rf[2] = a_min_control_rf[2] * 1.2;
    landing_time = c_optimal_linear_coeff_fcn_calc(
        coeffs_ship_prediction_speed_7, P0_UAV_NED, V0_UAV_NED,
        PhiThetaPsi_SHIP_rad[2], P0_SHIP_NED, b_v_max_control_rf,
        b_v_min_control_rf, b_a_max_control_rf, b_a_min_control_rf,
        max_time_of_landing_seconds, t_delay_ship_prediction, optimal_coeffs,
        exitflag_approach_path, V_OOB, A_OOB);
  } else {
    landing_time = c_optimal_linear_coeff_fcn_calc(
        coeffs_ship_prediction_speed_7, P0_UAV_NED, V0_UAV_NED,
        PhiThetaPsi_SHIP_rad[2], P0_SHIP_NED, v_max_control_rf,
        v_min_control_rf, a_max_control_rf, a_min_control_rf,
        max_time_of_landing_seconds, t_delay_ship_prediction, optimal_coeffs,
        exitflag_approach_path, V_OOB, A_OOB);
  }
  evaluate_UAV_path_from_coeffs(optimal_coeffs, landing_time, UAV_path_NED,
                                UAV_path_speed_NED, UAV_path_acc_NED, a__1);
  R_EC_tmp = sin(psi_rad_UAV);
  b_R_EC_tmp = cos(psi_rad_UAV);
  /* Initialize the expected_landing_time_relative to be invalid:  */
  *expected_landing_time_relative = -1.0;
  /* If we are far away, initialize the diagonal landing starting from the UAV
   */
  /* altitude and initial UAV position: */
  *approach_ship_mode = 0.0;
  *Desired_phi_rad = 0.0;
  *Desired_theta_rad = 0.0;
  scale = 3.3121686421112381E-170;
  *UAV_to_SHIP_azimuth_angle_rad = 0.0;
  *UAV_to_SHIP_elevation_angle_rad = 0.0;
  b_scale = 3.3121686421112381E-170;
  d = P0_UAV_NED[0] - P0_SHIP_NED[0];
  UAV_to_SHIP_dist_NED[0] = d;
  absxk = fabs(d);
  if (absxk > 3.3121686421112381E-170) {
    UAV_to_SHIP_dist_norm = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    UAV_to_SHIP_dist_norm = t * t;
  }
  d = P0_SHIP_NED[0] + c_NED_offset_end_point_diag_lan[0];
  P0_SHIP_NED_w_offset_idx_0 = d;
  chosen_approach_heading_rad = P0_UAV_NED[0] - d;
  c_UAV_to_SHIP_dist_NED_w_offset = chosen_approach_heading_rad;
  absxk_tmp = fabs(chosen_approach_heading_rad);
  if (absxk_tmp > 3.3121686421112381E-170) {
    d1 = 1.0;
    b_scale = absxk_tmp;
  } else {
    t = absxk_tmp / 3.3121686421112381E-170;
    d1 = t * t;
  }
  d = P0_UAV_NED[1] - P0_SHIP_NED[1];
  UAV_to_SHIP_dist_NED[1] = d;
  absxk = fabs(d);
  if (absxk > scale) {
    t = scale / absxk;
    UAV_to_SHIP_dist_norm = UAV_to_SHIP_dist_norm * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    UAV_to_SHIP_dist_norm += t * t;
  }
  d = P0_SHIP_NED[1] + c_NED_offset_end_point_diag_lan[1];
  P0_SHIP_NED_w_offset_idx_1 = d;
  chosen_approach_heading_rad = P0_UAV_NED[1] - d;
  d_UAV_to_SHIP_dist_NED_w_offset = chosen_approach_heading_rad;
  absxk = fabs(chosen_approach_heading_rad);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    d1 = d1 * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    d1 += t * t;
  }
  d = P0_UAV_NED[2] - P0_SHIP_NED[2];
  UAV_to_SHIP_dist_NED[2] = d;
  absxk = fabs(d);
  if (absxk > scale) {
    t = scale / absxk;
    UAV_to_SHIP_dist_norm = UAV_to_SHIP_dist_norm * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    UAV_to_SHIP_dist_norm += t * t;
  }
  d = P0_SHIP_NED[2] + c_NED_offset_end_point_diag_lan[2];
  chosen_approach_heading_rad = P0_UAV_NED[2] - d;
  absxk = fabs(chosen_approach_heading_rad);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    d1 = d1 * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    d1 += t * t;
  }
  UAV_to_SHIP_dist_norm = scale * sqrt(UAV_to_SHIP_dist_norm);
  d1 = b_scale * sqrt(d1);
  if ((d1 > 0.1) && (absxk_tmp > 0.0)) {
    *UAV_to_SHIP_azimuth_angle_rad =
        atan2(c_UAV_to_SHIP_dist_NED_w_offset, d_UAV_to_SHIP_dist_NED_w_offset);
    *UAV_to_SHIP_elevation_angle_rad = asin(chosen_approach_heading_rad / d1);
  }
  /* Generate references for being on the diagonal line:  */
  absxk = sin(*UAV_to_SHIP_azimuth_angle_rad);
  t = cos(*UAV_to_SHIP_azimuth_angle_rad);
  /* Now add 90 degrees to the azimuth as it is in a different rf (conventional
   */
  /* atan2 rf, we want it to be in the back of the ship and positive as the
   * heading:  */
  *UAV_to_SHIP_azimuth_angle_rad =
      -(*UAV_to_SHIP_azimuth_angle_rad + 1.5707963267948966);
  /* Add the component to stay on the desired line:  */
  chosen_approach_heading_rad =
      PhiThetaPsi_SHIP_rad[2] + approach_heading_ship_rad;
  /*  Calculate wp_diff and pos_diff */
  absxk_tmp = P0_SHIP_NED_w_offset_idx_0 - P0_UAV_NED[0];
  b_scale = absxk_tmp;
  /* Calculate the heading of the UAV to ship line:  */
  if (fabs(absxk_tmp) < 0.1) {
    b_scale = 0.1;
  }
  c_UAV_to_SHIP_dist_NED_w_offset = P0_SHIP_NED_w_offset_idx_1 - P0_UAV_NED[1];
  *psi_UAV_to_ship = atan2(c_UAV_to_SHIP_dist_NED_w_offset, b_scale);
  /* Calculate the error between the angles:  */
  *delta_psi = chosen_approach_heading_rad - *psi_UAV_to_ship;
  scale = -diag_approach_speed_m_s * cos(*UAV_to_SHIP_elevation_angle_rad);
  b_scale = fmin(max_line_gain, dist_line_gain / d1) * UAV_to_SHIP_dist_norm *
            sin(*delta_psi);
  d_UAV_to_SHIP_dist_NED_w_offset =
      (scale * absxk + average_speed_NED_ship[0]) +
      b_scale * cos(chosen_approach_heading_rad - 1.5707963267948966);
  absxk = (scale * t + average_speed_NED_ship[1]) +
          b_scale * sin(chosen_approach_heading_rad - 1.5707963267948966);
  t = -diag_approach_speed_m_s * sin(*UAV_to_SHIP_elevation_angle_rad) +
      average_speed_NED_ship[2];
  /* If we are close enough, switch from line follow to landing pad position
   * tracking:  */
  if ((UAV_to_SHIP_dist_norm <= pos_tracking_distance_m) ||
      ((approach_ship_mode_old == 1.0) &&
       (UAV_to_SHIP_dist_norm <= 2.0 * pos_tracking_distance_m))) {
    *approach_ship_mode = 1.0;
    /* Track ship position and speed of the landing pad:  */
    d_UAV_to_SHIP_dist_NED_w_offset =
        absxk_tmp * pos_gain_landing_array[0] + V0_SHIP_NED[0];
    absxk = c_UAV_to_SHIP_dist_NED_w_offset * pos_gain_landing_array[1] +
            V0_SHIP_NED[1];
    t = (d - P0_UAV_NED[2]) * pos_gain_landing_array[2] + V0_SHIP_NED[2];
  }
  /* If a landing time is available, then feed the path provided by the
   * polynomio: */
  /* Only use vertical speed polynomio, keep feedback on horizontal! */
  if ((V_OOB[2] == 0.0) && (A_OOB[2] == 0.0) &&
      (t_delay_ship_prediction < 3.0) &&
      ((d1 < 0.4) || (approach_ship_mode_old == 2.0))) {
    *expected_landing_time_relative = landing_time;
    *approach_ship_mode = 2.0;
    /* Generate the errors accounting for the references and the linear EC, only
     * for the vertical component:  */
    d_UAV_to_SHIP_dist_NED_w_offset =
        (P0_SHIP_NED[0] - P0_UAV_NED[0]) * pos_gain_landing_array[0] +
        V0_SHIP_NED[0];
    absxk = (P0_SHIP_NED[1] - P0_UAV_NED[1]) * pos_gain_landing_array[1] +
            V0_SHIP_NED[1];
    t = UAV_path_speed_NED[5] +
        (UAV_path_NED[5] - P0_UAV_NED[2]) * pos_gain_landing_array[2];
  }
  /* If we are close enough to the landing pad, then enter the flare low mode:
   */
  if ((UAV_to_SHIP_dist_norm <= flare_low_distance_m) ||
      (approach_ship_mode_old == 3.0)) {
    *approach_ship_mode = 3.0;
    /* Track ship position, speed and attitude, just apply a slight vertical
     * speed:  */
    /*  V_err_NED = P_err_NED.*pos_gain_landing + V0_SHIP_NED + [0; 0;
     * v_speed_docking_m_s] - V0_UAV_NED ; */
    /*  A_err_NED = V_err_NED.*speed_gain_landing - A0_UAV_NED_FILT; */
    d_UAV_to_SHIP_dist_NED_w_offset =
        (P0_SHIP_NED[0] - P0_UAV_NED[0]) * pos_gain_landing_array[0] +
        V0_SHIP_NED[0];
    absxk = (P0_SHIP_NED[1] - P0_UAV_NED[1]) * pos_gain_landing_array[1] +
            V0_SHIP_NED[1];
    t = V0_SHIP_NED[2] + v_speed_docking_m_s;
    b_scale = psi_rad_UAV - PhiThetaPsi_SHIP_rad[2];
    scale = sin(b_scale);
    b_scale = cos(b_scale);
    *Desired_phi_rad =
        PhiThetaPsi_SHIP_rad[0] * b_scale + PhiThetaPsi_SHIP_rad[1] * scale;
    *Desired_theta_rad =
        -PhiThetaPsi_SHIP_rad[0] * scale + PhiThetaPsi_SHIP_rad[1] * b_scale;
  }
  /*  V_target_control = R_EC * (V_err_NED + V0_UAV_NED); */
  /*  A_target_control = R_EC * (A_err_NED + A0_UAV_NED_FILT); */
  /*   */
  /*  %Transpose errors in control rf: */
  /*  A_err_control_rf = R_EC * A_err_NED; */
  c_R_EC_tmp[0] = b_R_EC_tmp;
  c_R_EC_tmp[3] = R_EC_tmp;
  c_R_EC_tmp[6] = 0.0;
  c_R_EC_tmp[1] = -R_EC_tmp;
  c_R_EC_tmp[4] = b_R_EC_tmp;
  c_R_EC_tmp[7] = 0.0;
  c_R_EC_tmp[2] = 0.0;
  c_R_EC_tmp[5] = 0.0;
  c_R_EC_tmp[8] = 1.0;
  for (i = 0; i < 3; i++) {
    V_target_control[i] = (c_R_EC_tmp[i] * d_UAV_to_SHIP_dist_NED_w_offset +
                           c_R_EC_tmp[i + 3] * absxk) +
                          c_R_EC_tmp[i + 6] * t;
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
