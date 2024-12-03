/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nav_approach_ship.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Dec-2024 22:15:36
 */

/* Include Files */
#include "nav_approach_ship.h"
#include <math.h>

/* Function Declarations */
static bool any(const bool x[10]);

static float c_optimal_linear_coeff_fcn_calc(
    const float coeffs_ship_prediction[24], const float P0_NED[3],
    const float V0_NED[3], float Psi_ship_rad, const float P0_SHIP_NED[3],
    const float v_max_control_rf[3], const float v_min_control_rf[3],
    const float a_max_control_rf[3], const float a_min_control_rf[3],
    float max_time_of_landing_seconds, float t_delay_ship_prediction,
    float optimal_coeffs[18], float *exitflag, float V_OOB[6], float A_OOB[6]);

static void
evaluate_UAV_path_from_coeffs(const float coeffs[18], float landing_time,
                              float UAV_path[30], float UAV_path_speed[30],
                              float UAV_path_acc[30], float t_array[10]);

/* Function Definitions */
/*
 * Arguments    : const bool x[10]
 * Return Type  : bool
 */
static bool any(const bool x[10])
{
  int k;
  bool exitg1;
  bool y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 10)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * Prepare variables for funcion:
 *
 * Arguments    : const float coeffs_ship_prediction[24]
 *                const float P0_NED[3]
 *                const float V0_NED[3]
 *                float Psi_ship_rad
 *                const float P0_SHIP_NED[3]
 *                const float v_max_control_rf[3]
 *                const float v_min_control_rf[3]
 *                const float a_max_control_rf[3]
 *                const float a_min_control_rf[3]
 *                float max_time_of_landing_seconds
 *                float t_delay_ship_prediction
 *                float optimal_coeffs[18]
 *                float *exitflag
 *                float V_OOB[6]
 *                float A_OOB[6]
 * Return Type  : float
 */
static float c_optimal_linear_coeff_fcn_calc(
    const float coeffs_ship_prediction[24], const float P0_NED[3],
    const float V0_NED[3], float Psi_ship_rad, const float P0_SHIP_NED[3],
    const float v_max_control_rf[3], const float v_min_control_rf[3],
    const float a_max_control_rf[3], const float a_min_control_rf[3],
    float max_time_of_landing_seconds, float t_delay_ship_prediction,
    float optimal_coeffs[18], float *exitflag, float V_OOB[6], float A_OOB[6])
{
  float Vx_0_UAV;
  float Vy_0_UAV;
  float Vz_0_UAV;
  float X_0_UAV;
  float Y_0_UAV;
  float Z_0_UAV;
  float coeff_10_ship_prediction;
  float coeff_11_ship_prediction;
  float coeff_12_ship_prediction;
  float coeff_13_ship_prediction;
  float coeff_14_ship_prediction;
  float coeff_15_ship_prediction;
  float coeff_16_ship_prediction;
  float coeff_17_ship_prediction;
  float coeff_18_ship_prediction;
  float coeff_19_ship_prediction;
  float coeff_1_ship_prediction;
  float coeff_20_ship_prediction;
  float coeff_21_ship_prediction;
  float coeff_22_ship_prediction;
  float coeff_23_ship_prediction;
  float coeff_2_ship_prediction;
  float coeff_3_ship_prediction;
  float coeff_4_ship_prediction;
  float coeff_5_ship_prediction;
  float coeff_6_ship_prediction;
  float coeff_7_ship_prediction;
  float coeff_8_ship_prediction;
  float coeff_9_ship_prediction;
  float landing_time;
  int b_i;
  int i;
  int i2;
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
  /* Init guess coefficients:  */
  optimal_coeffs[3] = 0.0F;
  optimal_coeffs[4] = V0_NED[0];
  optimal_coeffs[5] = P0_NED[0];
  optimal_coeffs[9] = 0.0F;
  optimal_coeffs[10] = V0_NED[1];
  optimal_coeffs[11] = P0_NED[1];
  optimal_coeffs[0] = 0.0F;
  optimal_coeffs[6] = 0.0F;
  optimal_coeffs[12] = 0.0F;
  optimal_coeffs[1] = 0.0F;
  optimal_coeffs[7] = 0.0F;
  optimal_coeffs[13] = 0.0F;
  optimal_coeffs[2] = 0.0F;
  optimal_coeffs[8] = 0.0F;
  optimal_coeffs[14] = 0.0F;
  optimal_coeffs[15] = 0.0F;
  optimal_coeffs[16] = V0_NED[2];
  optimal_coeffs[17] = P0_NED[2];
  landing_time = 0.3F;
  *exitflag = -1.0F;
  for (i = 0; i < 6; i++) {
    V_OOB[i] = 0.0F;
    A_OOB[i] = 0.0F;
  }
  while ((*exitflag < 0.0F) && (landing_time < max_time_of_landing_seconds)) {
    float R_EC[90];
    float UAV_path_acc_control_rf[30];
    float UAV_path_speed_control_rf[30];
    float optimal_UAV_path_acc[30];
    float optimal_UAV_path_speed[30];
    float UAV_path_heading[10];
    float ab_et10_tmp;
    float ab_et1_tmp;
    float b_et10_tmp;
    float b_et11_tmp;
    float b_et12_tmp;
    float b_et19_tmp;
    float b_et1_tmp;
    float b_et2_tmp;
    float b_et3_tmp;
    float bb_et10_tmp;
    float bb_et1_tmp;
    float c_et10_tmp;
    float c_et11_tmp;
    float c_et19_tmp;
    float c_et1_tmp;
    float c_et2_tmp;
    float c_et3_tmp;
    float cb_et10_tmp;
    float d_et10_tmp;
    float d_et11_tmp;
    float d_et19_tmp;
    float d_et1_tmp;
    float d_et2_tmp;
    float d_et3_tmp;
    float e_et10_tmp;
    float e_et11_tmp;
    float e_et19_tmp;
    float e_et1_tmp;
    float e_et2_tmp;
    float et10_tmp;
    float et11_tmp;
    float et12_tmp;
    float et19_tmp;
    float et1_tmp;
    float et2_tmp;
    float et3_tmp;
    float f_et10_tmp;
    float f_et11_tmp;
    float f_et19_tmp;
    float f_et1_tmp;
    float f_et2_tmp;
    float g_et10_tmp;
    float g_et11_tmp;
    float g_et19_tmp;
    float g_et1_tmp;
    float g_et2_tmp;
    float h_et10_tmp;
    float h_et11_tmp;
    float h_et19_tmp;
    float h_et1_tmp;
    float h_et2_tmp;
    float i_et10_tmp;
    float i_et11_tmp;
    float i_et19_tmp;
    float i_et1_tmp;
    float i_et2_tmp;
    float j_et10_tmp;
    float j_et11_tmp;
    float j_et19_tmp;
    float j_et1_tmp;
    float j_et2_tmp;
    float k_et10_tmp;
    float k_et11_tmp;
    float k_et19_tmp;
    float k_et1_tmp;
    float k_et2_tmp;
    float l_et10_tmp;
    float l_et11_tmp;
    float l_et19_tmp;
    float l_et1_tmp;
    float l_et2_tmp;
    float m_et10_tmp;
    float m_et11_tmp;
    float m_et19_tmp;
    float m_et1_tmp;
    float m_et2_tmp;
    float n_et10_tmp;
    float n_et11_tmp;
    float n_et19_tmp;
    float n_et1_tmp;
    float n_et2_tmp;
    float o_et10_tmp;
    float o_et11_tmp;
    float o_et19_tmp;
    float o_et1_tmp;
    float o_et2_tmp;
    float optimal_coeffs_tmp;
    float p_et10_tmp;
    float p_et11_tmp;
    float p_et1_tmp;
    float p_et2_tmp;
    float q_et10_tmp;
    float q_et11_tmp;
    float q_et1_tmp;
    float q_et2_tmp;
    float r_et10_tmp;
    float r_et11_tmp;
    float r_et1_tmp;
    float r_et2_tmp;
    float s_et10_tmp;
    float s_et11_tmp;
    float s_et1_tmp;
    float s_et2_tmp;
    float t10;
    float t11;
    float t12;
    float t13;
    float t14;
    float t15;
    float t16;
    float t17;
    float t18;
    float t19;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
    float t7;
    float t8;
    float t9;
    float t_et10_tmp;
    float t_et11_tmp;
    float t_et1_tmp;
    float t_et2_tmp;
    float u_et10_tmp;
    float u_et11_tmp;
    float u_et1_tmp;
    float u_et2_tmp;
    float v_et10_tmp;
    float v_et11_tmp;
    float v_et1_tmp;
    float w_et10_tmp;
    float w_et1_tmp;
    float x_et10_tmp;
    float x_et1_tmp;
    float y_et10_tmp;
    float y_et1_tmp;
    int UAV_path_acc_control_rf_tmp;
    signed char x[12];
    bool b_UAV_path_speed_control_rf[10];
    bool c_UAV_path_speed_control_rf[10];
    bool d_UAV_path_speed_control_rf[10];
    bool e_UAV_path_speed_control_rf[10];
    bool f_UAV_path_speed_control_rf[10];
    bool g_UAV_path_speed_control_rf[10];
    bool exitg1;
    bool out;
    landing_time += 0.6F;
    /* OPTIMAL_LINEAR_COEFF_CALCULATION_W_DELAY_FROM_SPEED */
    /*     COEFFS_OPTIMIZED =
     * OPTIMAL_LINEAR_COEFF_CALCULATION_W_DELAY_FROM_SPEED(Ax_0_UAV,Ay_0_UAV,Az_0_UAV,Psi_ship_rad,Vx_0_UAV,Vy_0_UAV,Vz_0_UAV,X_0_UAV,X_0_SHIP,Y_0_UAV,Y_0_SHIP,Z_0_UAV,Z_0_SHIP,COEFF_1_SHIP_PREDICTION,COEFF_2_SHIP_PREDICTION,COEFF_3_SHIP_PREDICTION,COEFF_4_SHIP_PREDICTION,COEFF_5_SHIP_PREDICTION,COEFF_6_SHIP_PREDICTION,COEFF_7_SHIP_PREDICTION,COEFF_8_SHIP_PREDICTION,COEFF_9_SHIP_PREDICTION,COEFF_10_SHIP_PREDICTION,COEFF_11_SHIP_PREDICTION,COEFF_12_SHIP_PREDICTION,COEFF_13_SHIP_PREDICTION,COEFF_14_SHIP_PREDICTION,COEFF_15_SHIP_PREDICTION,COEFF_16_SHIP_PREDICTION,COEFF_17_SHIP_PREDICTION,COEFF_18_SHIP_PREDICTION,COEFF_19_SHIP_PREDICTION,COEFF_20_SHIP_PREDICTION,COEFF_21_SHIP_PREDICTION,COEFF_22_SHIP_PREDICTION,COEFF_23_SHIP_PREDICTION,COEFF_24_SHIP_PREDICTION,T_DELAY_SHIP_PREDICTION,T_LANDING)
     */
    /*     This function was generated by the Symbolic Math Toolbox
     * version 23.2. */
    /*     09-Sep-2024 13:07:23 */
    t2 = cosf(Psi_ship_rad);
    t3 = sinf(Psi_ship_rad);
    t4 = t_delay_ship_prediction * t_delay_ship_prediction;
    t5 = powf(t_delay_ship_prediction, 3.0F);
    t7 = powf(t_delay_ship_prediction, 5.0F);
    t9 = powf(t_delay_ship_prediction, 7.0F);
    t10 = landing_time * landing_time;
    t11 = powf(landing_time, 3.0F);
    t13 = powf(landing_time, 5.0F);
    t15 = powf(landing_time, 7.0F);
    t6 = t4 * t4;
    t8 = powf(t4, 3.0F);
    t12 = t10 * t10;
    t14 = powf(t10, 3.0F);
    t16 = 1.0F / t11;
    t18 = 1.0F / t13;
    t17 = 1.0F / t12;
    t19 = 1.0F / (t17 * t17);
    et1_tmp = coeff_1_ship_prediction * t2;
    b_et1_tmp = coeff_2_ship_prediction * t2;
    c_et1_tmp = coeff_3_ship_prediction * t2;
    d_et1_tmp = coeff_4_ship_prediction * t2;
    e_et1_tmp = coeff_9_ship_prediction * t3;
    f_et1_tmp = coeff_10_ship_prediction * t3;
    g_et1_tmp = coeff_11_ship_prediction * t3;
    h_et1_tmp = coeff_12_ship_prediction * t3;
    i_et1_tmp = coeff_5_ship_prediction * t2;
    j_et1_tmp = coeff_7_ship_prediction * t2;
    k_et1_tmp = coeff_15_ship_prediction * t3;
    l_et1_tmp = X_0_UAV * 840.0F - P0_SHIP_NED[0] * 840.0F;
    m_et1_tmp = Vx_0_UAV * landing_time;
    n_et1_tmp = b_et1_tmp * t15;
    o_et1_tmp = c_et1_tmp * t14;
    p_et1_tmp = j_et1_tmp * t10;
    q_et1_tmp = et1_tmp * t19;
    r_et1_tmp = f_et1_tmp * t15;
    s_et1_tmp = g_et1_tmp * t14;
    t_et1_tmp = k_et1_tmp * t10;
    u_et1_tmp = e_et1_tmp * t19;
    v_et1_tmp = coeff_8_ship_prediction * t2 * landing_time;
    w_et1_tmp = coeff_16_ship_prediction * t3 * landing_time;
    x_et1_tmp = et1_tmp * t8 * t10;
    y_et1_tmp = b_et1_tmp * t7 * t10;
    ab_et1_tmp = c_et1_tmp * t6 * t10;
    bb_et1_tmp = d_et1_tmp * t5 * t10;
    et2_tmp = coeff_6_ship_prediction * t2;
    b_et2_tmp = coeff_13_ship_prediction * t3;
    c_et2_tmp = coeff_14_ship_prediction * t3;
    d_et2_tmp = e_et1_tmp * t8 * t10;
    e_et2_tmp = f_et1_tmp * t7 * t10;
    f_et2_tmp = g_et1_tmp * t6 * t10;
    g_et2_tmp = h_et1_tmp * t5 * t10;
    h_et2_tmp = et1_tmp * t15 * t_delay_ship_prediction;
    i_et2_tmp = b_et1_tmp * t14 * t_delay_ship_prediction;
    j_et2_tmp = et2_tmp * t10 * t_delay_ship_prediction;
    k_et2_tmp = e_et1_tmp * t15 * t_delay_ship_prediction;
    l_et2_tmp = f_et1_tmp * t14 * t_delay_ship_prediction;
    m_et2_tmp = c_et2_tmp * t10 * t_delay_ship_prediction;
    n_et2_tmp = et1_tmp * t9 * landing_time;
    o_et2_tmp = b_et1_tmp * t8 * landing_time;
    p_et2_tmp = c_et1_tmp * t7 * landing_time;
    q_et2_tmp = d_et1_tmp * t6 * landing_time;
    r_et2_tmp = i_et1_tmp * t5 * landing_time;
    s_et2_tmp = e_et1_tmp * t9 * landing_time;
    t_et2_tmp = f_et1_tmp * t8 * landing_time;
    u_et2_tmp = g_et1_tmp * t7 * landing_time;
    et3_tmp = h_et1_tmp * t6 * landing_time;
    b_et3_tmp = b_et2_tmp * t5 * landing_time;
    c_et3_tmp = j_et1_tmp * t_delay_ship_prediction * landing_time;
    d_et3_tmp = k_et1_tmp * t_delay_ship_prediction * landing_time;
    et10_tmp = coeff_1_ship_prediction * t3;
    b_et10_tmp = coeff_2_ship_prediction * t3;
    c_et10_tmp = coeff_3_ship_prediction * t3;
    d_et10_tmp = coeff_4_ship_prediction * t3;
    e_et10_tmp = coeff_9_ship_prediction * t2;
    f_et10_tmp = coeff_10_ship_prediction * t2;
    g_et10_tmp = coeff_11_ship_prediction * t2;
    h_et10_tmp = coeff_12_ship_prediction * t2;
    i_et10_tmp = coeff_5_ship_prediction * t3;
    j_et10_tmp = coeff_7_ship_prediction * t3;
    k_et10_tmp = coeff_15_ship_prediction * t2;
    l_et10_tmp = P0_SHIP_NED[1] * 840.0F - Y_0_UAV * 840.0F;
    m_et10_tmp = Vy_0_UAV * landing_time;
    n_et10_tmp = b_et10_tmp * t15;
    o_et10_tmp = c_et10_tmp * t14;
    p_et10_tmp = j_et10_tmp * t10;
    q_et10_tmp = et10_tmp * t19;
    r_et10_tmp = f_et10_tmp * t15;
    s_et10_tmp = g_et10_tmp * t14;
    t_et10_tmp = k_et10_tmp * t10;
    u_et10_tmp = e_et10_tmp * t19;
    v_et10_tmp = coeff_8_ship_prediction * t3 * landing_time;
    w_et10_tmp = coeff_16_ship_prediction * t2 * landing_time;
    x_et10_tmp = et10_tmp * t8 * t10;
    y_et10_tmp = b_et10_tmp * t7 * t10;
    ab_et10_tmp = c_et10_tmp * t6 * t10;
    bb_et10_tmp = d_et10_tmp * t5 * t10;
    cb_et10_tmp = e_et10_tmp * t8 * t10;
    et11_tmp = coeff_6_ship_prediction * t3;
    b_et11_tmp = coeff_13_ship_prediction * t2;
    c_et11_tmp = coeff_14_ship_prediction * t2;
    d_et11_tmp = f_et10_tmp * t7 * t10;
    e_et11_tmp = g_et10_tmp * t6 * t10;
    f_et11_tmp = h_et10_tmp * t5 * t10;
    g_et11_tmp = et10_tmp * t15 * t_delay_ship_prediction;
    h_et11_tmp = b_et10_tmp * t14 * t_delay_ship_prediction;
    i_et11_tmp = et11_tmp * t10 * t_delay_ship_prediction;
    j_et11_tmp = e_et10_tmp * t15 * t_delay_ship_prediction;
    k_et11_tmp = f_et10_tmp * t14 * t_delay_ship_prediction;
    l_et11_tmp = c_et11_tmp * t10 * t_delay_ship_prediction;
    m_et11_tmp = et10_tmp * t9 * landing_time;
    n_et11_tmp = b_et10_tmp * t8 * landing_time;
    o_et11_tmp = c_et10_tmp * t7 * landing_time;
    p_et11_tmp = d_et10_tmp * t6 * landing_time;
    q_et11_tmp = i_et10_tmp * t5 * landing_time;
    r_et11_tmp = e_et10_tmp * t9 * landing_time;
    s_et11_tmp = f_et10_tmp * t8 * landing_time;
    t_et11_tmp = g_et10_tmp * t7 * landing_time;
    u_et11_tmp = h_et10_tmp * t6 * landing_time;
    v_et11_tmp = b_et11_tmp * t5 * landing_time;
    et12_tmp = j_et10_tmp * t_delay_ship_prediction * landing_time;
    b_et12_tmp = k_et10_tmp * t_delay_ship_prediction * landing_time;
    et19_tmp = P0_SHIP_NED[2] * 840.0F - Z_0_UAV * 840.0F;
    b_et19_tmp = Vz_0_UAV * landing_time;
    c_et19_tmp = coeff_18_ship_prediction * t15;
    d_et19_tmp = coeff_19_ship_prediction * t14;
    e_et19_tmp = coeff_23_ship_prediction * t10;
    f_et19_tmp = coeff_17_ship_prediction * t19;
    g_et19_tmp = coeffs_ship_prediction[23] * landing_time;
    h_et19_tmp = coeff_17_ship_prediction * t8 * t10;
    i_et19_tmp = coeff_18_ship_prediction * t7 * t10;
    j_et19_tmp = coeff_19_ship_prediction * t6 * t10;
    k_et19_tmp = coeff_20_ship_prediction * t5 * t10;
    l_et19_tmp = coeff_17_ship_prediction * t15 * t_delay_ship_prediction;
    m_et19_tmp = coeff_18_ship_prediction * t14 * t_delay_ship_prediction;
    n_et19_tmp = coeff_22_ship_prediction * t10 * t_delay_ship_prediction;
    o_et19_tmp = coeff_17_ship_prediction * t9 * landing_time;
    j_et1_tmp = coeff_18_ship_prediction * t8 * landing_time;
    k_et1_tmp = coeff_19_ship_prediction * t7 * landing_time;
    t3 = coeff_20_ship_prediction * t6 * landing_time;
    t2 = coeff_21_ship_prediction * t5 * landing_time;
    j_et10_tmp =
        coeff_23_ship_prediction * t_delay_ship_prediction * landing_time;
    k_et10_tmp = et1_tmp * t4 * t14;
    t19 = i_et1_tmp * t4 * t10;
    t15 = e_et1_tmp * t4 * t14;
    t9 = b_et2_tmp * t4 * t10;
    t8 = et2_tmp * t4 * landing_time;
    optimal_coeffs_tmp = c_et2_tmp * t4 * landing_time;
    optimal_coeffs[0] =
        t18 *
        (((((((((((((((((((((((((l_et1_tmp + m_et1_tmp * 420.0F) -
                                n_et1_tmp * 120.0F) -
                               o_et1_tmp * 70.0F) -
                              d_et1_tmp * t13 * 28.0F) -
                             p_et1_tmp * 70.0F) -
                            q_et1_tmp * 175.0F) +
                           r_et1_tmp * 120.0F) +
                          s_et1_tmp * 70.0F) +
                         h_et1_tmp * t13 * 28.0F) +
                        t_et1_tmp * 70.0F) +
                       u_et1_tmp * 175.0F) -
                      v_et1_tmp * 420.0F) +
                     w_et1_tmp * 420.0F) -
                    k_et10_tmp * 1470.0F) -
                   et1_tmp * t5 * t13 * 980.0F) -
                  x_et1_tmp * 490.0F) -
                 b_et1_tmp * t4 * t13 * 420.0F) -
                y_et1_tmp * 420.0F) -
               ab_et1_tmp * 350.0F) -
              bb_et1_tmp * 280.0F) -
             t19 * 210.0F) +
            t15 * 1470.0F) +
           e_et1_tmp * t5 * t13 * 980.0F) +
          ((((((((((((((((((((((d_et2_tmp * 490.0F +
                                f_et1_tmp * t4 * t13 * 420.0F) +
                               e_et2_tmp * 420.0F) +
                              f_et2_tmp * 350.0F) +
                             g_et2_tmp * 280.0F) +
                            t9 * 210.0F) -
                           h_et2_tmp * 840.0F) -
                          i_et2_tmp * 420.0F) -
                         c_et1_tmp * t13 * t_delay_ship_prediction * 140.0F) -
                        j_et2_tmp * 140.0F) +
                       k_et2_tmp * 840.0F) +
                      l_et2_tmp * 420.0F) +
                     g_et1_tmp * t13 * t_delay_ship_prediction * 140.0F) +
                    m_et2_tmp * 140.0F) -
                   n_et2_tmp * 420.0F) -
                  o_et2_tmp * 420.0F) -
                 p_et2_tmp * 420.0F) -
                q_et2_tmp * 420.0F) -
               r_et2_tmp * 420.0F) -
              t8 * 420.0F) +
             s_et2_tmp * 420.0F) +
            t_et2_tmp * 420.0F) +
           u_et2_tmp * 420.0F)) +
         ((((et3_tmp * 420.0F + b_et3_tmp * 420.0F) +
            optimal_coeffs_tmp * 420.0F) -
           c_et3_tmp * 420.0F) +
          d_et3_tmp * 420.0F)) *
        -0.00714285718F;
    optimal_coeffs[1] =
        t17 *
        ((((((((((((((((((((((((((l_et1_tmp + m_et1_tmp * 448.0F) -
                                 n_et1_tmp * 64.0F) -
                                o_et1_tmp * 28.0F) +
                               i_et1_tmp * t12 * 14.0F) -
                              p_et1_tmp * 84.0F) -
                             q_et1_tmp * 105.0F) +
                            r_et1_tmp * 64.0F) +
                           s_et1_tmp * 28.0F) -
                          b_et2_tmp * t12 * 14.0F) +
                         t_et1_tmp * 84.0F) +
                        u_et1_tmp * 105.0F) -
                       v_et1_tmp * 448.0F) +
                      w_et1_tmp * 448.0F) -
                     k_et10_tmp * 588.0F) +
                    et1_tmp * t6 * t12 * 490.0F) -
                   x_et1_tmp * 588.0F) +
                  b_et1_tmp * t5 * t12 * 280.0F) -
                 y_et1_tmp * 504.0F) +
                c_et1_tmp * t4 * t12 * 140.0F) -
               ab_et1_tmp * 420.0F) -
              bb_et1_tmp * 336.0F) -
             t19 * 252.0F) +
            t15 * 588.0F) -
           e_et1_tmp * t6 * t12 * 490.0F) +
          ((((((((((((((((((((((d_et2_tmp * 588.0F -
                                f_et1_tmp * t5 * t12 * 280.0F) +
                               e_et2_tmp * 504.0F) -
                              g_et1_tmp * t4 * t12 * 140.0F) +
                             f_et2_tmp * 420.0F) +
                            g_et2_tmp * 336.0F) +
                           t9 * 252.0F) -
                          h_et2_tmp * 448.0F) -
                         i_et2_tmp * 168.0F) +
                        d_et1_tmp * t12 * t_delay_ship_prediction * 56.0F) -
                       j_et2_tmp * 168.0F) +
                      k_et2_tmp * 448.0F) +
                     l_et2_tmp * 168.0F) -
                    h_et1_tmp * t12 * t_delay_ship_prediction * 56.0F) +
                   m_et2_tmp * 168.0F) -
                  n_et2_tmp * 448.0F) -
                 o_et2_tmp * 448.0F) -
                p_et2_tmp * 448.0F) -
               q_et2_tmp * 448.0F) -
              r_et2_tmp * 448.0F) -
             t8 * 448.0F) +
            s_et2_tmp * 448.0F) +
           t_et2_tmp * 448.0F)) +
         (((((u_et2_tmp * 448.0F + et3_tmp * 448.0F) + b_et3_tmp * 448.0F) +
            optimal_coeffs_tmp * 448.0F) -
           c_et3_tmp * 448.0F) +
          d_et3_tmp * 448.0F)) /
        56.0F;
    optimal_coeffs[2] =
        t16 *
        ((((((((((((((((((((((((((l_et1_tmp + m_et1_tmp * 504.0F) -
                                 n_et1_tmp * 36.0F) -
                                o_et1_tmp * 14.0F) -
                               et2_tmp * t11 * 28.0F) -
                              p_et1_tmp * 126.0F) -
                             q_et1_tmp * 63.0F) +
                            r_et1_tmp * 36.0F) +
                           s_et1_tmp * 14.0F) +
                          c_et2_tmp * t11 * 28.0F) +
                         t_et1_tmp * 126.0F) +
                        u_et1_tmp * 63.0F) -
                       v_et1_tmp * 504.0F) +
                      w_et1_tmp * 504.0F) -
                     k_et10_tmp * 294.0F) -
                    et1_tmp * t7 * t11 * 588.0F) -
                   x_et1_tmp * 882.0F) -
                  b_et1_tmp * t6 * t11 * 420.0F) -
                 y_et1_tmp * 756.0F) -
                c_et1_tmp * t5 * t11 * 280.0F) -
               ab_et1_tmp * 630.0F) -
              d_et1_tmp * t4 * t11 * 168.0F) -
             bb_et1_tmp * 504.0F) -
            t19 * 378.0F) +
           t15 * 294.0F) +
          ((((((((((((((((((((((e_et1_tmp * t7 * t11 * 588.0F +
                                d_et2_tmp * 882.0F) +
                               f_et1_tmp * t6 * t11 * 420.0F) +
                              e_et2_tmp * 756.0F) +
                             g_et1_tmp * t5 * t11 * 280.0F) +
                            f_et2_tmp * 630.0F) +
                           h_et1_tmp * t4 * t11 * 168.0F) +
                          g_et2_tmp * 504.0F) +
                         t9 * 378.0F) -
                        h_et2_tmp * 252.0F) -
                       i_et2_tmp * 84.0F) -
                      i_et1_tmp * t11 * t_delay_ship_prediction * 84.0F) -
                     j_et2_tmp * 252.0F) +
                    k_et2_tmp * 252.0F) +
                   l_et2_tmp * 84.0F) +
                  b_et2_tmp * t11 * t_delay_ship_prediction * 84.0F) +
                 m_et2_tmp * 252.0F) -
                n_et2_tmp * 504.0F) -
               o_et2_tmp * 504.0F) -
              p_et2_tmp * 504.0F) -
             q_et2_tmp * 504.0F) -
            r_et2_tmp * 504.0F) -
           t8 * 504.0F)) +
         (((((((s_et2_tmp * 504.0F + t_et2_tmp * 504.0F) + u_et2_tmp * 504.0F) +
              et3_tmp * 504.0F) +
             b_et3_tmp * 504.0F) +
            optimal_coeffs_tmp * 504.0F) -
           c_et3_tmp * 504.0F) +
          d_et3_tmp * 504.0F)) *
        -0.0119047621F;
    optimal_coeffs[3] = 0.0F;
    optimal_coeffs[4] = Vx_0_UAV;
    optimal_coeffs[5] = X_0_UAV;
    k_et10_tmp = et10_tmp * t4 * t14;
    t19 = i_et10_tmp * t4 * t10;
    t15 = e_et10_tmp * t4 * t14;
    t9 = b_et11_tmp * t4 * t10;
    t8 = et11_tmp * t4 * landing_time;
    optimal_coeffs_tmp = c_et11_tmp * t4 * landing_time;
    optimal_coeffs[6] =
        t18 *
        (((((((((((((((((((((((((((l_et10_tmp - m_et10_tmp * 420.0F) +
                                  n_et10_tmp * 120.0F) +
                                 o_et10_tmp * 70.0F) +
                                d_et10_tmp * t13 * 28.0F) +
                               p_et10_tmp * 70.0F) +
                              q_et10_tmp * 175.0F) +
                             r_et10_tmp * 120.0F) +
                            s_et10_tmp * 70.0F) +
                           h_et10_tmp * t13 * 28.0F) +
                          t_et10_tmp * 70.0F) +
                         u_et10_tmp * 175.0F) +
                        v_et10_tmp * 420.0F) +
                       w_et10_tmp * 420.0F) +
                      k_et10_tmp * 1470.0F) +
                     et10_tmp * t5 * t13 * 980.0F) +
                    x_et10_tmp * 490.0F) +
                   b_et10_tmp * t4 * t13 * 420.0F) +
                  y_et10_tmp * 420.0F) +
                 ab_et10_tmp * 350.0F) +
                bb_et10_tmp * 280.0F) +
               t19 * 210.0F) +
              t15 * 1470.0F) +
             e_et10_tmp * t5 * t13 * 980.0F) +
            cb_et10_tmp * 490.0F) +
           f_et10_tmp * t4 * t13 * 420.0F) +
          (((((((((((((((((((((((d_et11_tmp * 420.0F + e_et11_tmp * 350.0F) +
                                f_et11_tmp * 280.0F) +
                               t9 * 210.0F) +
                              g_et11_tmp * 840.0F) +
                             h_et11_tmp * 420.0F) +
                            c_et10_tmp * t13 * t_delay_ship_prediction *
                                140.0F) +
                           i_et11_tmp * 140.0F) +
                          j_et11_tmp * 840.0F) +
                         k_et11_tmp * 420.0F) +
                        g_et10_tmp * t13 * t_delay_ship_prediction * 140.0F) +
                       l_et11_tmp * 140.0F) +
                      m_et11_tmp * 420.0F) +
                     n_et11_tmp * 420.0F) +
                    o_et11_tmp * 420.0F) +
                   p_et11_tmp * 420.0F) +
                  q_et11_tmp * 420.0F) +
                 t8 * 420.0F) +
                r_et11_tmp * 420.0F) +
               s_et11_tmp * 420.0F) +
              t_et11_tmp * 420.0F) +
             u_et11_tmp * 420.0F) +
            v_et11_tmp * 420.0F) +
           optimal_coeffs_tmp * 420.0F)) +
         (et12_tmp * 420.0F + b_et12_tmp * 420.0F)) /
        140.0F;
    optimal_coeffs[7] =
        t17 *
        ((((((((((((((((((((((((((l_et10_tmp - m_et10_tmp * 448.0F) +
                                 n_et10_tmp * 64.0F) +
                                o_et10_tmp * 28.0F) -
                               i_et10_tmp * t12 * 14.0F) +
                              p_et10_tmp * 84.0F) +
                             q_et10_tmp * 105.0F) +
                            r_et10_tmp * 64.0F) +
                           s_et10_tmp * 28.0F) -
                          b_et11_tmp * t12 * 14.0F) +
                         t_et10_tmp * 84.0F) +
                        u_et10_tmp * 105.0F) +
                       v_et10_tmp * 448.0F) +
                      w_et10_tmp * 448.0F) +
                     k_et10_tmp * 588.0F) -
                    et10_tmp * t6 * t12 * 490.0F) +
                   x_et10_tmp * 588.0F) -
                  b_et10_tmp * t5 * t12 * 280.0F) +
                 y_et10_tmp * 504.0F) -
                c_et10_tmp * t4 * t12 * 140.0F) +
               ab_et10_tmp * 420.0F) +
              bb_et10_tmp * 336.0F) +
             t19 * 252.0F) +
            t15 * 588.0F) -
           e_et10_tmp * t6 * t12 * 490.0F) +
          (((((((((((((((((((((((cb_et10_tmp * 588.0F -
                                 f_et10_tmp * t5 * t12 * 280.0F) +
                                d_et11_tmp * 504.0F) -
                               g_et10_tmp * t4 * t12 * 140.0F) +
                              e_et11_tmp * 420.0F) +
                             f_et11_tmp * 336.0F) +
                            t9 * 252.0F) +
                           g_et11_tmp * 448.0F) +
                          h_et11_tmp * 168.0F) -
                         d_et10_tmp * t12 * t_delay_ship_prediction * 56.0F) +
                        i_et11_tmp * 168.0F) +
                       j_et11_tmp * 448.0F) +
                      k_et11_tmp * 168.0F) -
                     h_et10_tmp * t12 * t_delay_ship_prediction * 56.0F) +
                    l_et11_tmp * 168.0F) +
                   m_et11_tmp * 448.0F) +
                  n_et11_tmp * 448.0F) +
                 o_et11_tmp * 448.0F) +
                p_et11_tmp * 448.0F) +
               q_et11_tmp * 448.0F) +
              t8 * 448.0F) +
             r_et11_tmp * 448.0F) +
            s_et11_tmp * 448.0F) +
           t_et11_tmp * 448.0F)) +
         ((((u_et11_tmp * 448.0F + v_et11_tmp * 448.0F) +
            optimal_coeffs_tmp * 448.0F) +
           et12_tmp * 448.0F) +
          b_et12_tmp * 448.0F)) *
        -0.0178571437F;
    optimal_coeffs[8] =
        t16 *
        (((((((((((((((((((((((((((l_et10_tmp - m_et10_tmp * 504.0F) +
                                  n_et10_tmp * 36.0F) +
                                 o_et10_tmp * 14.0F) +
                                et11_tmp * t11 * 28.0F) +
                               p_et10_tmp * 126.0F) +
                              q_et10_tmp * 63.0F) +
                             r_et10_tmp * 36.0F) +
                            s_et10_tmp * 14.0F) +
                           c_et11_tmp * t11 * 28.0F) +
                          t_et10_tmp * 126.0F) +
                         u_et10_tmp * 63.0F) +
                        v_et10_tmp * 504.0F) +
                       w_et10_tmp * 504.0F) +
                      k_et10_tmp * 294.0F) +
                     et10_tmp * t7 * t11 * 588.0F) +
                    x_et10_tmp * 882.0F) +
                   b_et10_tmp * t6 * t11 * 420.0F) +
                  y_et10_tmp * 756.0F) +
                 c_et10_tmp * t5 * t11 * 280.0F) +
                ab_et10_tmp * 630.0F) +
               d_et10_tmp * t4 * t11 * 168.0F) +
              bb_et10_tmp * 504.0F) +
             t19 * 378.0F) +
            t15 * 294.0F) +
           e_et10_tmp * t7 * t11 * 588.0F) +
          (((((((((((((((((((((((cb_et10_tmp * 882.0F +
                                 f_et10_tmp * t6 * t11 * 420.0F) +
                                d_et11_tmp * 756.0F) +
                               g_et10_tmp * t5 * t11 * 280.0F) +
                              e_et11_tmp * 630.0F) +
                             h_et10_tmp * t4 * t11 * 168.0F) +
                            f_et11_tmp * 504.0F) +
                           t9 * 378.0F) +
                          g_et11_tmp * 252.0F) +
                         h_et11_tmp * 84.0F) +
                        i_et10_tmp * t11 * t_delay_ship_prediction * 84.0F) +
                       i_et11_tmp * 252.0F) +
                      j_et11_tmp * 252.0F) +
                     k_et11_tmp * 84.0F) +
                    b_et11_tmp * t11 * t_delay_ship_prediction * 84.0F) +
                   l_et11_tmp * 252.0F) +
                  m_et11_tmp * 504.0F) +
                 n_et11_tmp * 504.0F) +
                o_et11_tmp * 504.0F) +
               p_et11_tmp * 504.0F) +
              q_et11_tmp * 504.0F) +
             t8 * 504.0F) +
            r_et11_tmp * 504.0F) +
           s_et11_tmp * 504.0F)) +
         (((((t_et11_tmp * 504.0F + u_et11_tmp * 504.0F) +
             v_et11_tmp * 504.0F) +
            optimal_coeffs_tmp * 504.0F) +
           et12_tmp * 504.0F) +
          b_et12_tmp * 504.0F)) /
        84.0F;
    optimal_coeffs[9] = 0.0F;
    optimal_coeffs[10] = Vy_0_UAV;
    optimal_coeffs[11] = Y_0_UAV;
    k_et10_tmp = coeff_17_ship_prediction * t4 * t14;
    t19 = coeff_21_ship_prediction * t4 * t10;
    t15 = coeff_22_ship_prediction * t4 * landing_time;
    optimal_coeffs[12] =
        t18 *
        ((((((((((((((((((((((((((et19_tmp - b_et19_tmp * 420.0F) +
                                 c_et19_tmp * 120.0F) +
                                d_et19_tmp * 70.0F) +
                               coeff_20_ship_prediction * t13 * 28.0F) +
                              e_et19_tmp * 70.0F) +
                             f_et19_tmp * 175.0F) +
                            g_et19_tmp * 420.0F) +
                           k_et10_tmp * 1470.0F) +
                          coeff_17_ship_prediction * t5 * t13 * 980.0F) +
                         h_et19_tmp * 490.0F) +
                        coeff_18_ship_prediction * t4 * t13 * 420.0F) +
                       i_et19_tmp * 420.0F) +
                      j_et19_tmp * 350.0F) +
                     k_et19_tmp * 280.0F) +
                    t19 * 210.0F) +
                   l_et19_tmp * 840.0F) +
                  m_et19_tmp * 420.0F) +
                 coeff_19_ship_prediction * t13 * t_delay_ship_prediction *
                     140.0F) +
                n_et19_tmp * 140.0F) +
               o_et19_tmp * 420.0F) +
              j_et1_tmp * 420.0F) +
             k_et1_tmp * 420.0F) +
            t3 * 420.0F) +
           t2 * 420.0F) +
          t15 * 420.0F) +
         j_et10_tmp * 420.0F) /
        140.0F;
    optimal_coeffs[13] =
        t17 *
        (((((((((((((((((((((((((((et19_tmp - b_et19_tmp * 448.0F) +
                                  c_et19_tmp * 64.0F) +
                                 d_et19_tmp * 28.0F) -
                                coeff_21_ship_prediction * t12 * 14.0F) +
                               e_et19_tmp * 84.0F) +
                              f_et19_tmp * 105.0F) +
                             g_et19_tmp * 448.0F) +
                            k_et10_tmp * 588.0F) -
                           coeff_17_ship_prediction * t6 * t12 * 490.0F) +
                          h_et19_tmp * 588.0F) -
                         coeff_18_ship_prediction * t5 * t12 * 280.0F) +
                        i_et19_tmp * 504.0F) -
                       coeff_19_ship_prediction * t4 * t12 * 140.0F) +
                      j_et19_tmp * 420.0F) +
                     k_et19_tmp * 336.0F) +
                    t19 * 252.0F) +
                   l_et19_tmp * 448.0F) +
                  m_et19_tmp * 168.0F) -
                 coeff_20_ship_prediction * t12 * t_delay_ship_prediction *
                     56.0F) +
                n_et19_tmp * 168.0F) +
               o_et19_tmp * 448.0F) +
              j_et1_tmp * 448.0F) +
             k_et1_tmp * 448.0F) +
            t3 * 448.0F) +
           t2 * 448.0F) +
          t15 * 448.0F) +
         j_et10_tmp * 448.0F) *
        -0.0178571437F;
    optimal_coeffs[14] =
        t16 *
        ((((((((((((((((((((((((((((et19_tmp - b_et19_tmp * 504.0F) +
                                   c_et19_tmp * 36.0F) +
                                  d_et19_tmp * 14.0F) +
                                 coeff_22_ship_prediction * t11 * 28.0F) +
                                e_et19_tmp * 126.0F) +
                               f_et19_tmp * 63.0F) +
                              g_et19_tmp * 504.0F) +
                             k_et10_tmp * 294.0F) +
                            coeff_17_ship_prediction * t7 * t11 * 588.0F) +
                           h_et19_tmp * 882.0F) +
                          coeff_18_ship_prediction * t6 * t11 * 420.0F) +
                         i_et19_tmp * 756.0F) +
                        coeff_19_ship_prediction * t5 * t11 * 280.0F) +
                       j_et19_tmp * 630.0F) +
                      coeff_20_ship_prediction * t4 * t11 * 168.0F) +
                     k_et19_tmp * 504.0F) +
                    t19 * 378.0F) +
                   l_et19_tmp * 252.0F) +
                  m_et19_tmp * 84.0F) +
                 coeff_21_ship_prediction * t11 * t_delay_ship_prediction *
                     84.0F) +
                n_et19_tmp * 252.0F) +
               o_et19_tmp * 504.0F) +
              j_et1_tmp * 504.0F) +
             k_et1_tmp * 504.0F) +
            t3 * 504.0F) +
           t2 * 504.0F) +
          t15 * 504.0F) +
         j_et10_tmp * 504.0F) *
        0.0119047621F;
    optimal_coeffs[15] = 0.0F;
    optimal_coeffs[16] = Vz_0_UAV;
    optimal_coeffs[17] = Z_0_UAV;
    evaluate_UAV_path_from_coeffs(
        optimal_coeffs, landing_time, UAV_path_speed_control_rf,
        optimal_UAV_path_speed, optimal_UAV_path_acc, UAV_path_heading);
    /* Check that the speed and accelerations are always within the bounds: */
    /* Report speeds in the control reference frame:  */
    /* Check if the speed and acceleration are within the bounds: */
    /* Speeds Out Of Bounds: */
    for (b_i = 0; b_i < 10; b_i++) {
      int i1;
      t15 = optimal_UAV_path_speed[3 * b_i];
      i = 3 * b_i + 1;
      t9 = optimal_UAV_path_speed[i];
      t19 = atan2f(t9, t15);
      k_et1_tmp = sinf(t19);
      j_et1_tmp = cosf(t19);
      R_EC[9 * b_i] = j_et1_tmp;
      R_EC[9 * b_i + 3] = k_et1_tmp;
      R_EC[9 * b_i + 6] = 0.0F;
      R_EC[9 * b_i + 1] = -k_et1_tmp;
      R_EC[9 * b_i + 4] = j_et1_tmp;
      R_EC[9 * b_i + 7] = 0.0F;
      R_EC[9 * b_i + 2] = 0.0F;
      R_EC[9 * b_i + 5] = 0.0F;
      R_EC[9 * b_i + 8] = 1.0F;
      t19 = optimal_UAV_path_acc[3 * b_i];
      j_et1_tmp = optimal_UAV_path_acc[i];
      i1 = 3 * b_i + 2;
      for (i2 = 0; i2 < 3; i2++) {
        UAV_path_acc_control_rf_tmp = i2 + 9 * b_i;
        k_et1_tmp = R_EC[UAV_path_acc_control_rf_tmp];
        t3 = k_et1_tmp * t15;
        t2 = k_et1_tmp * t19;
        k_et1_tmp = R_EC[UAV_path_acc_control_rf_tmp + 3];
        t3 += k_et1_tmp * t9;
        t2 += k_et1_tmp * j_et1_tmp;
        k_et1_tmp = R_EC[UAV_path_acc_control_rf_tmp + 6];
        t3 += k_et1_tmp * optimal_UAV_path_speed[i1];
        t2 += k_et1_tmp * optimal_UAV_path_acc[i1];
        UAV_path_acc_control_rf_tmp = i2 + 3 * b_i;
        UAV_path_acc_control_rf[UAV_path_acc_control_rf_tmp] = t2;
        UAV_path_speed_control_rf[UAV_path_acc_control_rf_tmp] = t3;
      }
      t15 = UAV_path_speed_control_rf[3 * b_i];
      b_UAV_path_speed_control_rf[b_i] = (t15 > v_max_control_rf[0]);
      t9 = UAV_path_speed_control_rf[i];
      c_UAV_path_speed_control_rf[b_i] = (t9 > v_max_control_rf[1]);
      t19 = UAV_path_speed_control_rf[3 * b_i + 2];
      d_UAV_path_speed_control_rf[b_i] = (t19 > v_max_control_rf[2]);
      e_UAV_path_speed_control_rf[b_i] = (t15 < v_min_control_rf[0]);
      f_UAV_path_speed_control_rf[b_i] = (t9 < v_min_control_rf[1]);
      g_UAV_path_speed_control_rf[b_i] = (t19 < v_min_control_rf[2]);
    }
    V_OOB[0] = any(b_UAV_path_speed_control_rf);
    V_OOB[1] = any(c_UAV_path_speed_control_rf);
    V_OOB[2] = any(d_UAV_path_speed_control_rf);
    V_OOB[3] = any(e_UAV_path_speed_control_rf);
    V_OOB[4] = any(f_UAV_path_speed_control_rf);
    V_OOB[5] = any(g_UAV_path_speed_control_rf);
    /* Accellerations Out Of Bounds: */
    t15 = a_max_control_rf[0];
    t9 = a_max_control_rf[1];
    t19 = a_max_control_rf[2];
    j_et1_tmp = a_min_control_rf[0];
    k_et1_tmp = a_min_control_rf[1];
    t3 = a_min_control_rf[2];
    for (i = 0; i < 10; i++) {
      t2 = UAV_path_acc_control_rf[3 * i];
      b_UAV_path_speed_control_rf[i] = (t2 > t15);
      j_et10_tmp = UAV_path_acc_control_rf[3 * i + 1];
      c_UAV_path_speed_control_rf[i] = (j_et10_tmp > t9);
      k_et10_tmp = UAV_path_acc_control_rf[3 * i + 2];
      d_UAV_path_speed_control_rf[i] = (k_et10_tmp > t19);
      e_UAV_path_speed_control_rf[i] = (t2 < j_et1_tmp);
      f_UAV_path_speed_control_rf[i] = (j_et10_tmp < k_et1_tmp);
      g_UAV_path_speed_control_rf[i] = (k_et10_tmp < t3);
    }
    A_OOB[0] = any(b_UAV_path_speed_control_rf);
    A_OOB[1] = any(c_UAV_path_speed_control_rf);
    A_OOB[2] = any(d_UAV_path_speed_control_rf);
    A_OOB[3] = any(e_UAV_path_speed_control_rf);
    A_OOB[4] = any(f_UAV_path_speed_control_rf);
    A_OOB[5] = any(g_UAV_path_speed_control_rf);
    for (i = 0; i < 6; i++) {
      x[i] = (signed char)V_OOB[i];
      x[i + 6] = (signed char)A_OOB[i];
    }
    out = false;
    UAV_path_acc_control_rf_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (UAV_path_acc_control_rf_tmp < 12)) {
      if (x[UAV_path_acc_control_rf_tmp] != 0) {
        out = true;
        exitg1 = true;
      } else {
        UAV_path_acc_control_rf_tmp++;
      }
    }
    if (!out) {
      *exitflag = 1.0F;
    }
  }
  /*  Display the total travel time and computational time.  */
  return landing_time;
}

/*
 * Arguments    : const float coeffs[18]
 *                float landing_time
 *                float UAV_path[30]
 *                float UAV_path_speed[30]
 *                float UAV_path_acc[30]
 *                float t_array[10]
 * Return Type  : void
 */
static void
evaluate_UAV_path_from_coeffs(const float coeffs[18], float landing_time,
                              float UAV_path[30], float UAV_path_speed[30],
                              float UAV_path_acc[30], float t_array[10])
{
  float UAV_path_speed_tmp;
  float a;
  float b_UAV_path_speed_tmp;
  float b_a;
  float c_a;
  float d_a;
  float delta1;
  float e_a;
  float f;
  float f1;
  float f10;
  float f11;
  float f12;
  float f13;
  float f14;
  float f15;
  float f16;
  float f17;
  float f2;
  float f3;
  float f4;
  float f5;
  float f6;
  float f7;
  float f8;
  float f9;
  float f_a;
  float g_a;
  float h_a;
  float i_a;
  float j_a;
  float k_a;
  float l_a;
  float m_a;
  float n_a;
  float o_a;
  float p_a;
  float q_a;
  float r_a;
  int k;
  /*  Generate UAV path from 5rd-order polynomial coefficients: */
  t_array[9] = landing_time;
  t_array[0] = 0.0F;
  if (-landing_time == 0.0F) {
    delta1 = landing_time / 9.0F;
    for (k = 0; k < 8; k++) {
      t_array[k + 1] = (2.0F * ((float)k + 2.0F) - 11.0F) * delta1;
    }
  } else if ((landing_time < 0.0F) && (fabsf(landing_time) > 1.70141173E+38F)) {
    delta1 = landing_time / 9.0F;
    for (k = 0; k < 8; k++) {
      t_array[k + 1] = delta1 * ((float)k + 1.0F);
    }
  } else {
    delta1 = landing_time / 9.0F;
    for (k = 0; k < 8; k++) {
      t_array[k + 1] = ((float)k + 1.0F) * delta1;
    }
  }
  delta1 = 2.0F * coeffs[3];
  UAV_path_speed_tmp = 2.0F * coeffs[9];
  b_UAV_path_speed_tmp = 2.0F * coeffs[15];
  a = 5.0F * coeffs[0];
  b_a = 4.0F * coeffs[1];
  c_a = 3.0F * coeffs[2];
  d_a = 5.0F * coeffs[6];
  e_a = 4.0F * coeffs[7];
  f_a = 3.0F * coeffs[8];
  g_a = 5.0F * coeffs[12];
  h_a = 4.0F * coeffs[13];
  i_a = 3.0F * coeffs[14];
  j_a = 20.0F * coeffs[0];
  k_a = 12.0F * coeffs[1];
  l_a = 6.0F * coeffs[2];
  m_a = 20.0F * coeffs[6];
  n_a = 12.0F * coeffs[7];
  o_a = 6.0F * coeffs[8];
  p_a = 20.0F * coeffs[12];
  q_a = 12.0F * coeffs[13];
  r_a = 6.0F * coeffs[14];
  f = coeffs[0];
  f1 = coeffs[1];
  f2 = coeffs[2];
  f3 = coeffs[3];
  f4 = coeffs[4];
  f5 = coeffs[5];
  f6 = coeffs[6];
  f7 = coeffs[7];
  f8 = coeffs[8];
  f9 = coeffs[9];
  f10 = coeffs[10];
  f11 = coeffs[11];
  f12 = coeffs[12];
  f13 = coeffs[13];
  f14 = coeffs[14];
  f15 = coeffs[15];
  f16 = coeffs[16];
  f17 = coeffs[17];
  for (k = 0; k < 10; k++) {
    float f18;
    float f19;
    float f20;
    float f21;
    float f22;
    int UAV_path_tmp;
    int b_UAV_path_tmp;
    f18 = t_array[k];
    f19 = powf(f18, 5.0F);
    f20 = powf(f18, 4.0F);
    f21 = powf(f18, 3.0F);
    f22 = f18 * f18;
    UAV_path[3 * k] =
        ((((f * f19 + f1 * f20) + f2 * f21) + f3 * f22) + f4 * f18) + f5;
    UAV_path_tmp = 3 * k + 1;
    UAV_path[UAV_path_tmp] =
        ((((f6 * f19 + f7 * f20) + f8 * f21) + f9 * f22) + f10 * f18) + f11;
    b_UAV_path_tmp = 3 * k + 2;
    UAV_path[b_UAV_path_tmp] =
        ((((f12 * f19 + f13 * f20) + f14 * f21) + f15 * f22) + f16 * f18) + f17;
    UAV_path_speed[3 * k] =
        (((a * f20 + b_a * f21) + c_a * f22) + delta1 * f18) + f4;
    UAV_path_speed[UAV_path_tmp] =
        (((d_a * f20 + e_a * f21) + f_a * f22) + UAV_path_speed_tmp * f18) +
        f10;
    UAV_path_speed[b_UAV_path_tmp] =
        (((g_a * f20 + h_a * f21) + i_a * f22) + b_UAV_path_speed_tmp * f18) +
        f16;
    UAV_path_acc[3 * k] = ((j_a * f21 + k_a * f22) + l_a * f18) + delta1;
    UAV_path_acc[UAV_path_tmp] =
        ((m_a * f21 + n_a * f22) + o_a * f18) + UAV_path_speed_tmp;
    UAV_path_acc[b_UAV_path_tmp] =
        ((p_a * f21 + q_a * f22) + r_a * f18) + b_UAV_path_speed_tmp;
  }
}

/*
 * Based on the ship coefficents, generate the landing path coefficients in the
 * NED reference frame:
 *
 * Arguments    : const float P0_UAV_NED[3]
 *                const float V0_UAV_NED[3]
 *                const float V0_SHIP_NED[3]
 *                const float PhiThetaPsi_SHIP_rad[3]
 *                const float P0_SHIP_NED[3]
 *                float psi_rad_UAV
 *                const float v_max_control_rf[3]
 *                const float v_min_control_rf[3]
 *                const float a_max_control_rf[3]
 *                const float a_min_control_rf[3]
 *                const float coeffs_ship_prediction_speed_7[24]
 *                float t_delay_ship_prediction
 *                float max_time_of_landing_seconds
 *                const float pos_gain_landing_array[3]
 *                float flare_low_distance_m
 *                float v_speed_docking_m_s
 *                const float average_speed_NED_ship[3]
 *                float diag_approach_speed_m_s
 *                float approach_ship_mode_old
 *                const float c_NED_offset_end_point_diag_lan[3]
 *                float pos_tracking_distance_m
 *                float approach_heading_ship_rad
 *                float dist_line_gain
 *                float max_line_gain
 *                float *expected_landing_time_relative
 *                float optimal_coeffs[18]
 *                float V_target_control[3]
 *                float *exitflag_approach_path
 *                float *approach_ship_mode
 *                float V_OOB[6]
 *                float A_OOB[6]
 *                float UAV_to_SHIP_dist_NED[3]
 *                float *Desired_phi_rad
 *                float *Desired_theta_rad
 *                float *UAV_to_SHIP_azimuth_angle_rad
 *                float *UAV_to_SHIP_elevation_angle_rad
 *                float *delta_psi
 *                float *psi_UAV_to_ship
 * Return Type  : void
 */
void nav_approach_ship(
    const float P0_UAV_NED[3], const float V0_UAV_NED[3],
    const float V0_SHIP_NED[3], const float PhiThetaPsi_SHIP_rad[3],
    const float P0_SHIP_NED[3], float psi_rad_UAV,
    const float v_max_control_rf[3], const float v_min_control_rf[3],
    const float a_max_control_rf[3], const float a_min_control_rf[3],
    const float coeffs_ship_prediction_speed_7[24],
    float t_delay_ship_prediction, float max_time_of_landing_seconds,
    const float pos_gain_landing_array[3], float flare_low_distance_m,
    float v_speed_docking_m_s, const float average_speed_NED_ship[3],
    float diag_approach_speed_m_s, float approach_ship_mode_old,
    const float c_NED_offset_end_point_diag_lan[3],
    float pos_tracking_distance_m, float approach_heading_ship_rad,
    float dist_line_gain, float max_line_gain,
    float *expected_landing_time_relative, float optimal_coeffs[18],
    float V_target_control[3], float *exitflag_approach_path,
    float *approach_ship_mode, float V_OOB[6], float A_OOB[6],
    float UAV_to_SHIP_dist_NED[3], float *Desired_phi_rad,
    float *Desired_theta_rad, float *UAV_to_SHIP_azimuth_angle_rad,
    float *UAV_to_SHIP_elevation_angle_rad, float *delta_psi,
    float *psi_UAV_to_ship)
{
  float UAV_path_NED[30];
  float UAV_path_speed_NED[30];
  float a__1[30];
  float a__2[10];
  float c_R_EC_tmp[9];
  float P0_SHIP_NED_w_offset_idx_0;
  float P0_SHIP_NED_w_offset_idx_1;
  float R_EC_tmp;
  float UAV_to_SHIP_dist_norm;
  float absxk;
  float absxk_tmp;
  float b_R_EC_tmp;
  float b_scale;
  float c_UAV_to_SHIP_dist_NED_w_offset;
  float d_UAV_to_SHIP_dist_NED_w_offset;
  float f;
  float landing_time;
  float out;
  float scale;
  float t;
  float y;
  int i;
  /* Run the path planning module based on the prediction coefficients: */
  if (approach_ship_mode_old == 2.0F) {
    float b_a_max_control_rf[3];
    float b_a_min_control_rf[3];
    float b_v_max_control_rf[3];
    float b_v_min_control_rf[3];
    b_v_max_control_rf[0] = v_max_control_rf[0] * 1.2F;
    b_v_min_control_rf[0] = v_min_control_rf[0] * 1.2F;
    b_a_max_control_rf[0] = a_max_control_rf[0] * 1.2F;
    b_a_min_control_rf[0] = a_min_control_rf[0] * 1.2F;
    b_v_max_control_rf[1] = v_max_control_rf[1] * 1.2F;
    b_v_min_control_rf[1] = v_min_control_rf[1] * 1.2F;
    b_a_max_control_rf[1] = a_max_control_rf[1] * 1.2F;
    b_a_min_control_rf[1] = a_min_control_rf[1] * 1.2F;
    b_v_max_control_rf[2] = v_max_control_rf[2] * 1.2F;
    b_v_min_control_rf[2] = v_min_control_rf[2] * 1.2F;
    b_a_max_control_rf[2] = a_max_control_rf[2] * 1.2F;
    b_a_min_control_rf[2] = a_min_control_rf[2] * 1.2F;
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
                                UAV_path_speed_NED, a__1, a__2);
  R_EC_tmp = sinf(psi_rad_UAV);
  b_R_EC_tmp = cosf(psi_rad_UAV);
  /* Initialize the expected_landing_time_relative to be invalid:  */
  *expected_landing_time_relative = -1.0F;
  /* If we are far away, initialize the diagonal landing starting from the UAV
   */
  /* altitude and initial UAV position: */
  *approach_ship_mode = 0.0F;
  *Desired_phi_rad = 0.0F;
  *Desired_theta_rad = 0.0F;
  scale = 1.29246971E-26F;
  *UAV_to_SHIP_azimuth_angle_rad = 0.0F;
  *UAV_to_SHIP_elevation_angle_rad = 0.0F;
  b_scale = 1.29246971E-26F;
  f = P0_UAV_NED[0] - P0_SHIP_NED[0];
  UAV_to_SHIP_dist_NED[0] = f;
  absxk = fabsf(f);
  if (absxk > 1.29246971E-26F) {
    UAV_to_SHIP_dist_norm = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    UAV_to_SHIP_dist_norm = t * t;
  }
  f = P0_SHIP_NED[0] + c_NED_offset_end_point_diag_lan[0];
  P0_SHIP_NED_w_offset_idx_0 = f;
  out = P0_UAV_NED[0] - f;
  c_UAV_to_SHIP_dist_NED_w_offset = out;
  absxk_tmp = fabsf(out);
  if (absxk_tmp > 1.29246971E-26F) {
    y = 1.0F;
    b_scale = absxk_tmp;
  } else {
    t = absxk_tmp / 1.29246971E-26F;
    y = t * t;
  }
  f = P0_UAV_NED[1] - P0_SHIP_NED[1];
  UAV_to_SHIP_dist_NED[1] = f;
  absxk = fabsf(f);
  if (absxk > scale) {
    t = scale / absxk;
    UAV_to_SHIP_dist_norm = UAV_to_SHIP_dist_norm * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    UAV_to_SHIP_dist_norm += t * t;
  }
  f = P0_SHIP_NED[1] + c_NED_offset_end_point_diag_lan[1];
  P0_SHIP_NED_w_offset_idx_1 = f;
  out = P0_UAV_NED[1] - f;
  d_UAV_to_SHIP_dist_NED_w_offset = out;
  absxk = fabsf(out);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    y = y * t * t + 1.0F;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    y += t * t;
  }
  f = P0_UAV_NED[2] - P0_SHIP_NED[2];
  UAV_to_SHIP_dist_NED[2] = f;
  absxk = fabsf(f);
  if (absxk > scale) {
    t = scale / absxk;
    UAV_to_SHIP_dist_norm = UAV_to_SHIP_dist_norm * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    UAV_to_SHIP_dist_norm += t * t;
  }
  f = P0_SHIP_NED[2] + c_NED_offset_end_point_diag_lan[2];
  out = P0_UAV_NED[2] - f;
  absxk = fabsf(out);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    y = y * t * t + 1.0F;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    y += t * t;
  }
  UAV_to_SHIP_dist_norm = scale * sqrtf(UAV_to_SHIP_dist_norm);
  y = b_scale * sqrtf(y);
  if ((y > 0.1F) && (absxk_tmp > 0.0F)) {
    *UAV_to_SHIP_azimuth_angle_rad = atan2f(c_UAV_to_SHIP_dist_NED_w_offset,
                                            d_UAV_to_SHIP_dist_NED_w_offset);
    *UAV_to_SHIP_elevation_angle_rad = asinf(out / y);
  }
  /* Generate references for being on the diagonal line:  */
  t = -diag_approach_speed_m_s * cosf(*UAV_to_SHIP_elevation_angle_rad);
  absxk = sinf(*UAV_to_SHIP_azimuth_angle_rad);
  out = cosf(*UAV_to_SHIP_azimuth_angle_rad);
  /* Now add 90 degrees to the azimuth as it is in a different rf (conventional
   */
  /* atan2 rf, we want it to be in the back of the ship and positive as the
   * heading:  */
  *UAV_to_SHIP_azimuth_angle_rad =
      -(*UAV_to_SHIP_azimuth_angle_rad + 1.57079637F);
  /* Add the component to stay on the desired line:  */
  c_UAV_to_SHIP_dist_NED_w_offset =
      PhiThetaPsi_SHIP_rad[2] + approach_heading_ship_rad;
  /*  Calculate wp_diff and pos_diff */
  d_UAV_to_SHIP_dist_NED_w_offset = P0_SHIP_NED_w_offset_idx_0 - P0_UAV_NED[0];
  b_scale = d_UAV_to_SHIP_dist_NED_w_offset;
  /* Calculate the heading of the UAV to ship line:  */
  if (fabsf(d_UAV_to_SHIP_dist_NED_w_offset) < 0.1) {
    b_scale = 0.1F;
  }
  absxk_tmp = P0_SHIP_NED_w_offset_idx_1 - P0_UAV_NED[1];
  *psi_UAV_to_ship = atan2f(absxk_tmp, b_scale);
  /* Calculate the error between the angles:  */
  *delta_psi = c_UAV_to_SHIP_dist_NED_w_offset - *psi_UAV_to_ship;
  scale = fminf(max_line_gain, dist_line_gain / y) * UAV_to_SHIP_dist_norm *
          sinf(*delta_psi);
  P0_SHIP_NED_w_offset_idx_0 =
      (t * absxk + average_speed_NED_ship[0]) +
      scale * cosf(c_UAV_to_SHIP_dist_NED_w_offset - 1.57079637F);
  absxk = (t * out + average_speed_NED_ship[1]) +
          scale * sinf(c_UAV_to_SHIP_dist_NED_w_offset - 1.57079637F);
  t = -diag_approach_speed_m_s * sinf(*UAV_to_SHIP_elevation_angle_rad) +
      average_speed_NED_ship[2];
  /* If we are close enough, switch from line follow to landing pad position
   * tracking:  */
  if ((UAV_to_SHIP_dist_norm <= pos_tracking_distance_m) ||
      ((approach_ship_mode_old == 1.0F) &&
       (UAV_to_SHIP_dist_norm <= 2.0F * pos_tracking_distance_m))) {
    *approach_ship_mode = 1.0F;
    /* Track ship position and speed of the landing pad:  */
    P0_SHIP_NED_w_offset_idx_0 =
        d_UAV_to_SHIP_dist_NED_w_offset * pos_gain_landing_array[0] +
        V0_SHIP_NED[0];
    absxk = absxk_tmp * pos_gain_landing_array[1] + V0_SHIP_NED[1];
    t = (f - P0_UAV_NED[2]) * pos_gain_landing_array[2] + V0_SHIP_NED[2];
  }
  /* If a landing time is available, then feed the path provided by the
   * polynomio: */
  /* Only use vertical speed polynomio, keep feedback on horizontal! */
  if ((V_OOB[2] == 0.0F) && (A_OOB[2] == 0.0F) &&
      (t_delay_ship_prediction < 3.0F) &&
      ((y < 0.4F) || (approach_ship_mode_old == 2.0F))) {
    *expected_landing_time_relative = landing_time;
    *approach_ship_mode = 2.0F;
    /* Generate the errors accounting for the references and the linear EC, only
     * for the vertical component:  */
    P0_SHIP_NED_w_offset_idx_0 =
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
      (approach_ship_mode_old == 3.0F)) {
    *approach_ship_mode = 3.0F;
    /* Track ship position, speed and attitude, just apply a slight vertical
     * speed:  */
    /*  V_err_NED = P_err_NED.*pos_gain_landing + V0_SHIP_NED + [0; 0;
     * v_speed_docking_m_s] - V0_UAV_NED ; */
    /*  A_err_NED = V_err_NED.*speed_gain_landing - A0_UAV_NED_FILT; */
    P0_SHIP_NED_w_offset_idx_0 =
        (P0_SHIP_NED[0] - P0_UAV_NED[0]) * pos_gain_landing_array[0] +
        V0_SHIP_NED[0];
    absxk = (P0_SHIP_NED[1] - P0_UAV_NED[1]) * pos_gain_landing_array[1] +
            V0_SHIP_NED[1];
    t = V0_SHIP_NED[2] + v_speed_docking_m_s;
    b_scale = psi_rad_UAV - PhiThetaPsi_SHIP_rad[2];
    scale = sinf(b_scale);
    b_scale = cosf(b_scale);
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
  c_R_EC_tmp[6] = 0.0F;
  c_R_EC_tmp[1] = -R_EC_tmp;
  c_R_EC_tmp[4] = b_R_EC_tmp;
  c_R_EC_tmp[7] = 0.0F;
  c_R_EC_tmp[2] = 0.0F;
  c_R_EC_tmp[5] = 0.0F;
  c_R_EC_tmp[8] = 1.0F;
  for (i = 0; i < 3; i++) {
    V_target_control[i] = (c_R_EC_tmp[i] * P0_SHIP_NED_w_offset_idx_0 +
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
