/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_acc_control_rf_basic.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 16-May-2024 19:15:38
 */

#ifndef COMPUTE_ACC_CONTROL_RF_BASIC_H
#define COMPUTE_ACC_CONTROL_RF_BASIC_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void compute_acc_control_rf_basic(
    double Beta, double CL_aileron, double Cd_zero, double Cl_alpha,
    double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz,
    double K_Cd, double K_p_M, double K_p_T, double Omega_1, double Omega_2,
    double Omega_3, double Omega_4, double Phi, double S, double Theta,
    double V, double b_1, double b_2, double b_3, double b_4,
    double delta_ailerons, double flight_path_angle, double g_1, double g_2,
    double g_3, double g_4, double l_1, double l_2, double l_3, double l_4,
    double l_z, double m, double p, double q, double r, double rho,
    double wing_chord, double accelerations_array[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_acc_control_rf_basic.h
 *
 * [EOF]
 */
