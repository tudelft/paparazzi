/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_acc_nonlinear_control_rf_w_ailerons.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Mar-2024 16:10:36
 */

#ifndef COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_H
#define COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_compute_acc_nonlinear_control(
    const double u_in[15], double p, double q, double r, double K_p_T,
    double K_p_M, double m, double I_xx, double I_yy, double I_zz, double l_1,
    double l_2, double l_3, double l_4, double l_z, double Cl_alpha,
    double Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero,
    double CL_aileron, double rho, double V, double S, double wing_chord,
    double flight_path_angle, double Beta, double computed_acc[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_acc_nonlinear_control_rf_w_ailerons.h
 *
 * [EOF]
 */
