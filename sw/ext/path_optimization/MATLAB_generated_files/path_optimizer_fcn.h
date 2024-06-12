/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: path_optimizer_fcn.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef PATH_OPTIMIZER_FCN_H
#define PATH_OPTIMIZER_FCN_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void path_optimizer_fcn(
    const double coeffs_ship_prediction[18], double landing_time_init,
    double time_sample_while, const double P0[3], const double V0[3],
    const double A0[3], const double v_max[3], const double v_min[3],
    const double a_max[3], const double a_min[3], double num_points,
    double num_points_plot, double pos_gain, double speed_gain, double acc_gain,
    double max_time_of_landing_seconds, double verbose,
    double optimal_coeffs[18], double *landing_time);

void path_optimizer_fcn_anonFcn2(
    const double P0[3], const double V0[3], const double A0[3],
    const double v_max[3], const double v_min[3], const double a_max[3],
    const double a_min[3], double landing_time, double num_points,
    const double coeffs_ship_prediction[18], double pos_gain, double speed_gain,
    double acc_gain, const double coeffs[18], double varargout_1_data[],
    int varargout_1_size[2], double varargout_2_data[], int varargout_2_size[2],
    double varargout_3_data[], int varargout_3_size[2],
    double varargout_4_data[], int varargout_4_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for path_optimizer_fcn.h
 *
 * [EOF]
 */
