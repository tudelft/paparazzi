/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_constraints_and_constraints_gradient_w_ship_coeff.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef COMPUTE_CONSTRAINTS_AND_CONSTRAINTS_GRADIENT_W_SHIP_COEFF_H
#define COMPUTE_CONSTRAINTS_AND_CONSTRAINTS_GRADIENT_W_SHIP_COEFF_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_compute_constraints_and_const(
    double Ax_0_UAV, double Ax_min_control_UAV, double Ax_max_control_UAV,
    double Ay_0_UAV, double Ay_min_control_UAV, double Ay_max_control_UAV,
    double Az_0_UAV, double Az_min_control_UAV, double Az_max_control_UAV,
    double Vx_0_UAV, double Vx_min_control_UAV, double Vx_max_control_UAV,
    double Vy_0_UAV, double Vy_min_control_UAV, double Vy_max_control_UAV,
    double Vz_0_UAV, double Vz_min_control_UAV, double Vz_max_control_UAV,
    double X_0_UAV, double Y_0_UAV, double Z_0_UAV, double acc_gain,
    double coeff_1_UAV, double coeff_2_UAV, double coeff_3_UAV,
    double coeff_4_UAV, double coeff_5_UAV, double coeff_6_UAV,
    double coeff_7_UAV, double coeff_8_UAV, double coeff_9_UAV,
    double coeff_10_UAV, double coeff_11_UAV, double coeff_12_UAV,
    double coeff_13_UAV, double coeff_14_UAV, double coeff_15_UAV,
    double coeff_16_UAV, double coeff_17_UAV, double coeff_18_UAV,
    double coeff_1_ship_prediction, double coeff_2_ship_prediction,
    double coeff_3_ship_prediction, double coeff_4_ship_prediction,
    double coeff_5_ship_prediction, double coeff_6_ship_prediction,
    double coeff_7_ship_prediction, double coeff_8_ship_prediction,
    double coeff_9_ship_prediction, double coeff_10_ship_prediction,
    double coeff_11_ship_prediction, double coeff_12_ship_prediction,
    double coeff_13_ship_prediction, double coeff_14_ship_prediction,
    double coeff_15_ship_prediction, double coeff_16_ship_prediction,
    double coeff_17_ship_prediction, double coeff_18_ship_prediction,
    double pos_gain, double speed_gain, double t_2_array, double t_3_array,
    double t_4_array, double t_5_array, double t_6_array, double t_7_array,
    double t_8_array, double t_9_array, double t_landing, double c_data[],
    int c_size[2], double ceq_data[], int ceq_size[2],
    creal_T c_gradient_data[], int c_gradient_size[2],
    double ceq_gradient_data[], int ceq_gradient_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_constraints_and_constraints_gradient_w_ship_coeff.h
 *
 * [EOF]
 */
