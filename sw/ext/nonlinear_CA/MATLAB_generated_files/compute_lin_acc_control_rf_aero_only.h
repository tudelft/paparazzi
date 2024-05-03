/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: compute_lin_acc_control_rf_aero_only.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef COMPUTE_LIN_ACC_CONTROL_RF_AERO_ONLY_H
#define COMPUTE_LIN_ACC_CONTROL_RF_AERO_ONLY_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_compute_lin_acc_control_rf_ae(double Beta, double Cd_zero,
                                     double Cl_alpha, double K_Cd, double Phi,
                                     double S, double Theta, double V,
                                     double flight_path_angle, double m,
                                     double rho, double accelerations_array[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for compute_lin_acc_control_rf_aero_only.h
 *
 * [EOF]
 */
