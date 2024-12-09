/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nav_approach_ship_1D.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 04-Dec-2024 16:29:25
 */

/* Include Files */
#include "nav_approach_ship_1D.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void
c_evaluate_UAV_path_from_coeffs(const float coeffs[6], float landing_time,
                                float UAV_path[10], float UAV_path_speed[10],
                                float UAV_path_acc[10], float t_array[10]);

static float c_optimal_linear_coeff_fcn_calc(
    const float coeffs_ship_prediction[8], const float P0_NED[3],
    const float V0_NED[3], const float P0_SHIP_NED[3],
    const float v_max_control_rf[3], const float v_min_control_rf[3],
    const float a_max_control_rf[3], const float a_min_control_rf[3],
    float max_time_of_landing_seconds, float t_delay_ship_prediction,
    float optimal_coeffs[6], float *exitflag, float V_OOB[6], float A_OOB[6]);

static float rt_atan2f_snf(float u0, float u1);

static float rt_powf_snf(float u0, float u1);

/* Function Definitions */
/*
 * Arguments    : const float coeffs[6]
 *                float landing_time
 *                float UAV_path[10]
 *                float UAV_path_speed[10]
 *                float UAV_path_acc[10]
 *                float t_array[10]
 * Return Type  : void
 */
static void
c_evaluate_UAV_path_from_coeffs(const float coeffs[6], float landing_time,
                                float UAV_path[10], float UAV_path_speed[10],
                                float UAV_path_acc[10], float t_array[10])
{
  float a;
  float b_a;
  float c_a;
  float d_a;
  float delta1;
  float e_a;
  float f;
  float f1;
  float f2;
  float f3;
  float f4;
  float f5;
  float f_a;
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
  a = 5.0F * coeffs[0];
  b_a = 4.0F * coeffs[1];
  c_a = 3.0F * coeffs[2];
  d_a = 20.0F * coeffs[0];
  e_a = 12.0F * coeffs[1];
  f_a = 6.0F * coeffs[2];
  f = coeffs[0];
  f1 = coeffs[1];
  f2 = coeffs[2];
  f3 = coeffs[3];
  f4 = coeffs[4];
  f5 = coeffs[5];
  for (k = 0; k < 10; k++) {
    float f6;
    float f7;
    float f8;
    float f9;
    f6 = t_array[k];
    f7 = rt_powf_snf(f6, 4.0F);
    f8 = rt_powf_snf(f6, 3.0F);
    f9 = f6 * f6;
    UAV_path[k] =
        ((((f * rt_powf_snf(f6, 5.0F) + f1 * f7) + f2 * f8) + f3 * f9) +
         f4 * f6) +
        f5;
    f7 = (((a * f7 + b_a * f8) + c_a * f9) + delta1 * f6) + f4;
    UAV_path_speed[k] = f7;
    f8 = ((d_a * f8 + e_a * f9) + f_a * f6) + delta1;
    UAV_path_acc[k] = f8;
  }
}

/*
 * Prepare variables for funcion:
 *
 * Arguments    : const float coeffs_ship_prediction[8]
 *                const float P0_NED[3]
 *                const float V0_NED[3]
 *                const float P0_SHIP_NED[3]
 *                const float v_max_control_rf[3]
 *                const float v_min_control_rf[3]
 *                const float a_max_control_rf[3]
 *                const float a_min_control_rf[3]
 *                float max_time_of_landing_seconds
 *                float t_delay_ship_prediction
 *                float optimal_coeffs[6]
 *                float *exitflag
 *                float V_OOB[6]
 *                float A_OOB[6]
 * Return Type  : float
 */
static float c_optimal_linear_coeff_fcn_calc(
    const float coeffs_ship_prediction[8], const float P0_NED[3],
    const float V0_NED[3], const float P0_SHIP_NED[3],
    const float v_max_control_rf[3], const float v_min_control_rf[3],
    const float a_max_control_rf[3], const float a_min_control_rf[3],
    float max_time_of_landing_seconds, float t_delay_ship_prediction,
    float optimal_coeffs[6], float *exitflag, float V_OOB[6], float A_OOB[6])
{
  float UAV_path_acc_control_rf[30];
  float UAV_path_speed_control_rf[30];
  float Vz_0_UAV;
  float Z_0_UAV;
  float coeff_1_ship_prediction;
  float coeff_2_ship_prediction;
  float coeff_3_ship_prediction;
  float coeff_4_ship_prediction;
  float coeff_5_ship_prediction;
  float coeff_6_ship_prediction;
  float coeff_7_ship_prediction;
  float landing_time;
  int k;
  Z_0_UAV = P0_NED[2];
  Vz_0_UAV = V0_NED[2];
  coeff_1_ship_prediction = coeffs_ship_prediction[0];
  coeff_2_ship_prediction = coeffs_ship_prediction[1];
  coeff_3_ship_prediction = coeffs_ship_prediction[2];
  coeff_4_ship_prediction = coeffs_ship_prediction[3];
  coeff_5_ship_prediction = coeffs_ship_prediction[4];
  coeff_6_ship_prediction = coeffs_ship_prediction[5];
  coeff_7_ship_prediction = coeffs_ship_prediction[6];
  /* Init guess coefficients:  */
  optimal_coeffs[0] = 0.0F;
  optimal_coeffs[1] = 0.0F;
  optimal_coeffs[2] = 0.0F;
  optimal_coeffs[3] = 0.0F;
  optimal_coeffs[4] = V0_NED[2];
  optimal_coeffs[5] = P0_NED[2];
  landing_time = 0.3F;
  *exitflag = -1.0F;
  for (k = 0; k < 6; k++) {
    V_OOB[k] = 0.0F;
    A_OOB[k] = 0.0F;
  }
  while ((*exitflag < 0.0F) && (landing_time < max_time_of_landing_seconds)) {
    float a__1[10];
    float a__2[10];
    float optimal_UAV_path_acc[10];
    float optimal_UAV_path_speed[10];
    float b_et2_tmp;
    float c_et2_tmp;
    float d_et2_tmp;
    float e_et2_tmp;
    float et2_tmp;
    float f_et2_tmp;
    float g_et2_tmp;
    float h_et2_tmp;
    float i_et2_tmp;
    float j_et2_tmp;
    float k_et2_tmp;
    float l_et2_tmp;
    float m_et2_tmp;
    float n_et2_tmp;
    float o_et2_tmp;
    float p_et2_tmp;
    float q_et2_tmp;
    float r_et2_tmp;
    float s_et2_tmp;
    float t10;
    float t11;
    float t12;
    float t13;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
    float t8;
    float t9;
    float t_et2_tmp;
    float u_et2_tmp;
    signed char b_x[12];
    bool x[10];
    bool Vz_max_OOB;
    bool Vz_min_OOB;
    bool exitg1;
    landing_time += 0.1F;
    /* optimal_linear_coeff_calculation_w_delay_from_speed_1D */
    /*     COEFFS_OPTIMIZED =
     * optimal_linear_coeff_calculation_w_delay_from_speed_1D(Az_0_UAV,Vz_0_UAV,Z_0_UAV,Z_0_SHIP,COEFF_1_SHIP_PREDICTION,COEFF_2_SHIP_PREDICTION,COEFF_3_SHIP_PREDICTION,COEFF_4_SHIP_PREDICTION,COEFF_5_SHIP_PREDICTION,COEFF_6_SHIP_PREDICTION,COEFF_7_SHIP_PREDICTION,COEFF_8_SHIP_PREDICTION,T_DELAY_SHIP_PREDICTION,T_LANDING)
     */
    /*     This function was generated by the Symbolic Math Toolbox
     * version 23.2. */
    /*     03-Dec-2024 08:50:47 */
    t2 = t_delay_ship_prediction * t_delay_ship_prediction;
    t3 = rt_powf_snf(t_delay_ship_prediction, 3.0F);
    t5 = rt_powf_snf(t_delay_ship_prediction, 5.0F);
    t8 = landing_time * landing_time;
    t9 = rt_powf_snf(landing_time, 3.0F);
    t11 = rt_powf_snf(landing_time, 5.0F);
    t13 = rt_powf_snf(landing_time, 7.0F);
    t4 = t2 * t2;
    t6 = rt_powf_snf(t2, 3.0F);
    t10 = t8 * t8;
    t12 = rt_powf_snf(t8, 3.0F);
    et2_tmp = 0.0F * t8;
    b_et2_tmp = P0_SHIP_NED[2] * 840.0F - Z_0_UAV * 840.0F;
    c_et2_tmp = Vz_0_UAV * landing_time;
    d_et2_tmp = coeff_1_ship_prediction * (t10 * t10);
    e_et2_tmp = coeff_2_ship_prediction * t13;
    f_et2_tmp = coeff_3_ship_prediction * t12;
    g_et2_tmp = coeff_7_ship_prediction * t8;
    h_et2_tmp = coeffs_ship_prediction[7] * landing_time;
    i_et2_tmp = coeff_1_ship_prediction * t6 * t8;
    j_et2_tmp = coeff_2_ship_prediction * t5 * t8;
    k_et2_tmp = coeff_3_ship_prediction * t4 * t8;
    l_et2_tmp = coeff_4_ship_prediction * t3 * t8;
    m_et2_tmp = coeff_1_ship_prediction * t13 * t_delay_ship_prediction;
    n_et2_tmp = coeff_2_ship_prediction * t12 * t_delay_ship_prediction;
    o_et2_tmp = coeff_6_ship_prediction * t8 * t_delay_ship_prediction;
    p_et2_tmp = coeff_1_ship_prediction *
                rt_powf_snf(t_delay_ship_prediction, 7.0F) * landing_time;
    q_et2_tmp = coeff_2_ship_prediction * t6 * landing_time;
    r_et2_tmp = coeff_3_ship_prediction * t5 * landing_time;
    s_et2_tmp = coeff_4_ship_prediction * t4 * landing_time;
    t_et2_tmp = coeff_5_ship_prediction * t3 * landing_time;
    u_et2_tmp =
        coeff_7_ship_prediction * t_delay_ship_prediction * landing_time;
    t12 *= coeff_1_ship_prediction * t2;
    t13 = coeff_5_ship_prediction * t2 * t8;
    t6 = coeff_6_ship_prediction * t2 * landing_time;
    optimal_coeffs[0] =
        (((((((((((((((((((((((((((b_et2_tmp - et2_tmp * 70.0F) -
                                  c_et2_tmp * 420.0F) +
                                 d_et2_tmp * 175.0F) +
                                e_et2_tmp * 120.0F) +
                               f_et2_tmp * 70.0F) +
                              coeff_4_ship_prediction * t11 * 28.0F) +
                             g_et2_tmp * 70.0F) +
                            h_et2_tmp * 420.0F) +
                           t12 * 1470.0F) +
                          coeff_1_ship_prediction * t3 * t11 * 980.0F) +
                         i_et2_tmp * 490.0F) +
                        coeff_2_ship_prediction * t2 * t11 * 420.0F) +
                       j_et2_tmp * 420.0F) +
                      k_et2_tmp * 350.0F) +
                     l_et2_tmp * 280.0F) +
                    t13 * 210.0F) +
                   m_et2_tmp * 840.0F) +
                  n_et2_tmp * 420.0F) +
                 coeff_3_ship_prediction * t11 * t_delay_ship_prediction *
                     140.0F) +
                o_et2_tmp * 140.0F) +
               p_et2_tmp * 420.0F) +
              q_et2_tmp * 420.0F) +
             r_et2_tmp * 420.0F) +
            s_et2_tmp * 420.0F) +
           t_et2_tmp * 420.0F) +
          t6 * 420.0F) +
         u_et2_tmp * 420.0F) /
        (t11 * 140.0F);
    optimal_coeffs[1] =
        1.0F / t10 *
        ((((((((((((((((((((((((((((b_et2_tmp - et2_tmp * 84.0F) -
                                   c_et2_tmp * 448.0F) +
                                  d_et2_tmp * 105.0F) +
                                 e_et2_tmp * 64.0F) +
                                f_et2_tmp * 28.0F) -
                               coeff_5_ship_prediction * t10 * 14.0F) +
                              g_et2_tmp * 84.0F) +
                             h_et2_tmp * 448.0F) +
                            t12 * 588.0F) -
                           coeff_1_ship_prediction * t4 * t10 * 490.0F) +
                          i_et2_tmp * 588.0F) -
                         coeff_2_ship_prediction * t3 * t10 * 280.0F) +
                        j_et2_tmp * 504.0F) -
                       coeff_3_ship_prediction * t2 * t10 * 140.0F) +
                      k_et2_tmp * 420.0F) +
                     l_et2_tmp * 336.0F) +
                    t13 * 252.0F) +
                   m_et2_tmp * 448.0F) +
                  n_et2_tmp * 168.0F) -
                 coeff_4_ship_prediction * t10 * t_delay_ship_prediction *
                     56.0F) +
                o_et2_tmp * 168.0F) +
               p_et2_tmp * 448.0F) +
              q_et2_tmp * 448.0F) +
             r_et2_tmp * 448.0F) +
            s_et2_tmp * 448.0F) +
           t_et2_tmp * 448.0F) +
          t6 * 448.0F) +
         u_et2_tmp * 448.0F) *
        -0.0178571437F;
    optimal_coeffs[2] =
        1.0F / t9 *
        (((((((((((((((((((((((((((((b_et2_tmp - et2_tmp * 126.0F) -
                                    c_et2_tmp * 504.0F) +
                                   d_et2_tmp * 63.0F) +
                                  e_et2_tmp * 36.0F) +
                                 f_et2_tmp * 14.0F) +
                                coeff_6_ship_prediction * t9 * 28.0F) +
                               g_et2_tmp * 126.0F) +
                              h_et2_tmp * 504.0F) +
                             t12 * 294.0F) +
                            coeff_1_ship_prediction * t5 * t9 * 588.0F) +
                           i_et2_tmp * 882.0F) +
                          coeff_2_ship_prediction * t4 * t9 * 420.0F) +
                         j_et2_tmp * 756.0F) +
                        coeff_3_ship_prediction * t3 * t9 * 280.0F) +
                       k_et2_tmp * 630.0F) +
                      coeff_4_ship_prediction * t2 * t9 * 168.0F) +
                     l_et2_tmp * 504.0F) +
                    t13 * 378.0F) +
                   m_et2_tmp * 252.0F) +
                  n_et2_tmp * 84.0F) +
                 coeff_5_ship_prediction * t9 * t_delay_ship_prediction *
                     84.0F) +
                o_et2_tmp * 252.0F) +
               p_et2_tmp * 504.0F) +
              q_et2_tmp * 504.0F) +
             r_et2_tmp * 504.0F) +
            s_et2_tmp * 504.0F) +
           t_et2_tmp * 504.0F) +
          t6 * 504.0F) +
         u_et2_tmp * 504.0F) *
        0.0119047621F;
    optimal_coeffs[3] = 0.0F;
    optimal_coeffs[4] = Vz_0_UAV;
    optimal_coeffs[5] = Z_0_UAV;
    c_evaluate_UAV_path_from_coeffs(optimal_coeffs, landing_time, a__1,
                                    optimal_UAV_path_speed,
                                    optimal_UAV_path_acc, a__2);
    memset(&UAV_path_speed_control_rf[0], 0, 30U * sizeof(float));
    memset(&UAV_path_acc_control_rf[0], 0, 30U * sizeof(float));
    /* Check if the speed and acceleration are within the bounds: */
    /* Speeds Out Of Bounds:   */
    for (k = 0; k < 10; k++) {
      int UAV_path_acc_control_rf_tmp;
      UAV_path_acc_control_rf_tmp = 3 * k + 2;
      UAV_path_acc_control_rf[UAV_path_acc_control_rf_tmp] =
          optimal_UAV_path_acc[k];
      t13 = optimal_UAV_path_speed[k];
      UAV_path_speed_control_rf[UAV_path_acc_control_rf_tmp] = t13;
      x[k] = (t13 > v_max_control_rf[2]);
    }
    Vz_max_OOB = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 10)) {
      if (x[k]) {
        Vz_max_OOB = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    for (k = 0; k < 10; k++) {
      x[k] = (UAV_path_speed_control_rf[3 * k + 2] < v_min_control_rf[2]);
    }
    Vz_min_OOB = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 10)) {
      if (x[k]) {
        Vz_min_OOB = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    V_OOB[0] = 0.0F;
    V_OOB[1] = 0.0F;
    V_OOB[2] = Vz_max_OOB;
    V_OOB[3] = 0.0F;
    V_OOB[4] = 0.0F;
    V_OOB[5] = Vz_min_OOB;
    /* Accellerations Out Of Bounds: */
    for (k = 0; k < 10; k++) {
      x[k] = (UAV_path_acc_control_rf[3 * k + 2] > a_max_control_rf[2]);
    }
    Vz_max_OOB = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 10)) {
      if (x[k]) {
        Vz_max_OOB = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    for (k = 0; k < 10; k++) {
      x[k] = (UAV_path_acc_control_rf[3 * k + 2] < a_min_control_rf[2]);
    }
    Vz_min_OOB = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 10)) {
      if (x[k]) {
        Vz_min_OOB = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    A_OOB[0] = 0.0F;
    A_OOB[1] = 0.0F;
    A_OOB[2] = Vz_max_OOB;
    A_OOB[3] = 0.0F;
    A_OOB[4] = 0.0F;
    A_OOB[5] = Vz_min_OOB;
    for (k = 0; k < 6; k++) {
      b_x[k] = (signed char)V_OOB[k];
      b_x[k + 6] = (signed char)A_OOB[k];
    }
    Vz_max_OOB = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 12)) {
      if (b_x[k] != 0) {
        Vz_max_OOB = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (!Vz_max_OOB) {
      *exitflag = 1.0F;
    }
  }
  /*  Display the total travel time and computational time.  */
  return landing_time;
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    int i;
    int i1;
    if (u0 > 0.0F) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0F) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2f((float)i, (float)i1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }
  return y;
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_powf_snf(float u0, float u1)
{
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else {
    float f;
    float f1;
    f = fabsf(u0);
    f1 = fabsf(u1);
    if (rtIsInfF(u1)) {
      if (f == 1.0F) {
        y = 1.0F;
      } else if (f > 1.0F) {
        if (u1 > 0.0F) {
          y = rtInfF;
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = rtInfF;
      }
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
      y = rtNaNF;
    } else {
      y = powf(u0, u1);
    }
  }
  return y;
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
 *                float optimal_coeffs[6]
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
void nav_approach_ship_1D(
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
    float *expected_landing_time_relative, float optimal_coeffs[6],
    float V_target_control[3], float *exitflag_approach_path,
    float *approach_ship_mode, float V_OOB[6], float A_OOB[6],
    float UAV_to_SHIP_dist_NED[3], float *Desired_phi_rad,
    float *Desired_theta_rad, float *UAV_to_SHIP_azimuth_angle_rad,
    float *UAV_to_SHIP_elevation_angle_rad, float *delta_psi,
    float *psi_UAV_to_ship)
{
  float UAV_path_NED[10];
  float UAV_path_speed_NED[10];
  float a__1[10];
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
  /* Run the path planning module based on the prediction coefficients on the
   * z-axis: */
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
        &coeffs_ship_prediction_speed_7[16], P0_UAV_NED, V0_UAV_NED,
        P0_SHIP_NED, b_v_max_control_rf, b_v_min_control_rf, b_a_max_control_rf,
        b_a_min_control_rf, max_time_of_landing_seconds,
        t_delay_ship_prediction, optimal_coeffs, exitflag_approach_path, V_OOB,
        A_OOB);
  } else {
    landing_time = c_optimal_linear_coeff_fcn_calc(
        &coeffs_ship_prediction_speed_7[16], P0_UAV_NED, V0_UAV_NED,
        P0_SHIP_NED, v_max_control_rf, v_min_control_rf, a_max_control_rf,
        a_min_control_rf, max_time_of_landing_seconds, t_delay_ship_prediction,
        optimal_coeffs, exitflag_approach_path, V_OOB, A_OOB);
  }
  c_evaluate_UAV_path_from_coeffs(optimal_coeffs, landing_time, UAV_path_NED,
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
    *UAV_to_SHIP_azimuth_angle_rad = rt_atan2f_snf(
        c_UAV_to_SHIP_dist_NED_w_offset, d_UAV_to_SHIP_dist_NED_w_offset);
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
  *psi_UAV_to_ship = rt_atan2f_snf(absxk_tmp, b_scale);
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
    t = UAV_path_speed_NED[1] +
        (UAV_path_NED[1] - P0_UAV_NED[2]) * pos_gain_landing_array[2];
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
    t = (0.0F * pos_gain_landing_array[2] + V0_SHIP_NED[2]) +
        v_speed_docking_m_s;
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
void nav_approach_ship_1D_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void nav_approach_ship_1D_terminate(void)
{
}

/*
 * File trailer for nav_approach_ship_1D.c
 *
 * [EOF]
 */
