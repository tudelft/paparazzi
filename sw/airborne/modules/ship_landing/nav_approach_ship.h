/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nav_approach_ship.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Dec-2024 14:21:21
 */

#ifndef NAV_APPROACH_SHIP_H
#define NAV_APPROACH_SHIP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void nav_approach_ship(
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
    double *psi_UAV_to_ship);

extern void nav_approach_ship_initialize(void);

extern void nav_approach_ship_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for nav_approach_ship.h
 *
 * [EOF]
 */
