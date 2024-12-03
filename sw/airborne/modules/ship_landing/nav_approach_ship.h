/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nav_approach_ship.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 02-Dec-2024 22:15:36
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
    float *psi_UAV_to_ship);

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
