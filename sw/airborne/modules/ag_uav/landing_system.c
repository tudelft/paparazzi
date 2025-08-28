/*
 * Copyright (C) 2025 AG <info@aerogriduav.com>

 */

/** @file "modules/ag_uav/landing_system.c"
 * @author AG <info@aerogriduav.com>
 * AG landing system interface
 */

#include "modules/ag_uav/landing_system.h"
#include "landing_system.h"
#include "state.h"
#include <math.h>
#include "modules/datalink/extra_pprz_dl.h"
#include "modules/datalink/telemetry.h"
#include "modules/datalink/downlink.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/core/abi.h"


/* Initialize the landing algorithm outputs struct*/
struct landing_algorithm_outputs_t landing_algorithm_outputs = {0};
struct ship_state_t ship_state = {0};
float uav_to_ship_time_offset = NAN; // ship time = UAV time + offset
float ship_coefficients[24] = {0};
float last_ship_coefficients_ship_time = 0; // TODO

// Clock offset new = old clock offset + (new clock offset - old clock offset) * CLOCK_OFFSET_UPDATE_FACTOR
#define CLOCK_OFFSET_UPDATE_FACTOR 0.1

#if PERIODIC_TELEMETRY
static void send_landing_algorithm_outputs_periodic(struct transport_tx *trans, struct link_device *dev) {  
  pprz_msg_send_LANDING_ALGORITHM_OUTPUT(trans, dev, AC_ID,
            &landing_algorithm_outputs.timestamp_output,
            landing_algorithm_outputs.UAV_acc_target_NED,
            landing_algorithm_outputs.UAV_desired_phi_theta_rad,
            &landing_algorithm_outputs.landing_algorithm_mode,
            &landing_algorithm_outputs.exitflag_path_planner,
            &landing_algorithm_outputs.expected_landing_time,
            landing_algorithm_outputs.V_out_of_bounds_array,
            landing_algorithm_outputs.A_out_of_bounds_array);
}

#endif

void receive_landing_algorithm_outputs(uint8_t *buf) 
{
  uint32_t timestamp_output = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_timestamp_output(buf);
  float *UAV_acc_target_NED = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_UAV_acc_target_NED(buf);
  float *UAV_desired_phi_theta_rad = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_UAV_desired_phi_theta_rad(buf);
  int8_t landing_algorithm_mode = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_landing_algorithm_mode(buf);
  int8_t exitflag_path_planner = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_exitflag_path_planner(buf);
  float expected_landing_time = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_expected_landing_time(buf);
  uint8_t *V_out_of_bounds_array = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_V_out_of_bounds_array(buf);
  uint8_t *A_out_of_bounds_array = pprzlink_get_DL_IMCU_LANDING_ALGORITHM_OUTPUT_A_out_of_bounds_array(buf);

  //Fill up the landing algorithm outputs structure
  landing_algorithm_outputs.timestamp_output = timestamp_output;
  landing_algorithm_outputs.UAV_acc_target_NED[0] = UAV_acc_target_NED[0];
  landing_algorithm_outputs.UAV_acc_target_NED[1] = UAV_acc_target_NED[1];
  landing_algorithm_outputs.UAV_acc_target_NED[2] = UAV_acc_target_NED[2];
  landing_algorithm_outputs.UAV_desired_phi_theta_rad[0] = UAV_desired_phi_theta_rad[0];
  landing_algorithm_outputs.UAV_desired_phi_theta_rad[1] = UAV_desired_phi_theta_rad[1];
  landing_algorithm_outputs.landing_algorithm_mode = landing_algorithm_mode;
  landing_algorithm_outputs.exitflag_path_planner = exitflag_path_planner;
  landing_algorithm_outputs.expected_landing_time = expected_landing_time;
  for (int i = 0; i < 6; i++) {
    landing_algorithm_outputs.V_out_of_bounds_array[i] = V_out_of_bounds_array[i];
    landing_algorithm_outputs.A_out_of_bounds_array[i] = A_out_of_bounds_array[i];
  }

  landing_algorithm_outputs.last_received_stamp = get_sys_time_usec();

  struct NedCoor_f accel_sp = {landing_algorithm_outputs.UAV_acc_target_NED[0], landing_algorithm_outputs.UAV_acc_target_NED[1], landing_algorithm_outputs.UAV_acc_target_NED[2]};
    struct EnuCoor_f accel_sp_enu;
    VECT3_ENU_OF_NED(accel_sp_enu, accel_sp);
    AbiSendMsgMOVING_BASE(LANDING_ALGORITHM_ID, stateGetPositionEnu_f(), stateGetSpeedEnu_f(), &accel_sp_enu);

  // Send the current state of the landing system module
  #if LANDING_SYSTEM_LOG_ON_ARRIVAL && !USE_NPS
  pprz_msg_send_LANDING_ALGORITHM_OUTPUT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
            &timestamp_output,
            UAV_acc_target_NED,
            UAV_desired_phi_theta_rad,
            &landing_algorithm_mode,
            &exitflag_path_planner,
            &expected_landing_time,
            V_out_of_bounds_array,
            A_out_of_bounds_array);
  #endif
}

void landing_system_init(void)
{
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LANDING_ALGORITHM_OUTPUT, send_landing_algorithm_outputs_periodic);
  #endif
}

void send_landing_algorithm_params(void){
  // Send the landing algorithm parameters
  float PH_offset_ship_ctr[3] = {-0.0f, -0.0f, -5.0f}; // Offset of the PH waypoint in the ship control reference frame
  float line_approach_speed = 2.0f; // Line approach speed in m/s
  float approach_line_angle_rad = 0.0f; // Angle with respect to the ship stern where the approach will be started
  float dist_line_gain = 2.0f; // Distance line gain
  float max_line_gain = 1.0f; // Maximum line gain
  float vel_gain_approach[3] = {1.0f, 1.0f, 1.0f}; // Velocity gain of the approach landing phase
  float pos_gain_hovering[3] = {1.0f, 1.0f, 1.0f}; // Position gain of the hovering landing phase
  float vel_gain_hovering[3] = {1.0f, 1.0f, 1.0f}; // Velocity gain of the hovering landing phase
  float hovering_engage_dist = 3.0f; // Euclidean distance for engagement of the PH hovering mode
  float min_time_landing_trajectory = 0.1f; // Minimum time for landing
  float max_time_landing_trajectory = 10.0f; // Maximum time for landing
  float landing_time_resolution_trajectory = 0.1f; // Resolution of the landing trajectopry generation 
  float V_bound_max_control[3] = {8.0f, 3.0f, 5.0f}; // Upper speed bounds for the trajectory generation in the control RF
  float V_bound_min_control[3] = {-5.0f, -3.0f, -5.0f}; // Lower speed bounds for the trajectory generation in the control RF
  float A_bound_max_control[3] = {3.0f, 1.0f, 1.0f}; // Upper acc bounds for the trajectory generation in the control RF
  float A_bound_min_control[3] = {-1.0f, -1.0f, -1.0f}; // Lower acc bounds for the trajectory generation in the control RF
  uint16_t num_points = 13; // Number of points where the speed and acc boundaries are checked
  float pos_gain_trajectory[3] = {1.0f, 1.0f, 1.0f}; // Position gain of the trajectory landing phase
  float vel_gain_trajectory[3] = {1.0f, 1.0f, 1.0f}; // Velocity gain of the trajectory landing phase
  float flare_engage_height = 1.0f; // Engaging height for the flare manoeuvre
  float flare_vertical_speed = 1.0f; // Vertical speed of the flare manoeuvre
  float pos_gain_flare[3] = {0.5f, 0.5f, 0.5f}; // Position gain of the flare landing phase
  float vel_gain_flare[3] = {4.0f, 4.0f, 8.0f}; // Velocity gain of the flare landing phase

  // Send the landing algorithm parameters
#ifdef EXTRA_DOWNLINK_DEVICE
  pprz_msg_send_IMCU_LANDING_ALGORITHM_PARAMS(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, 
    PH_offset_ship_ctr, &line_approach_speed, &approach_line_angle_rad, &dist_line_gain, &max_line_gain,
    vel_gain_approach, pos_gain_hovering, vel_gain_hovering, &hovering_engage_dist,
    &min_time_landing_trajectory, &max_time_landing_trajectory, &landing_time_resolution_trajectory,
    V_bound_max_control, V_bound_min_control, A_bound_max_control, A_bound_min_control,
    &num_points, pos_gain_trajectory, vel_gain_trajectory,
    &flare_engage_height, &flare_vertical_speed, pos_gain_flare, vel_gain_flare);
#endif // EXTRA_DOWNLINK_DEVICE
}

void request_landing_algorithm_outputs(void){

  // To request landing commands we just need to send the landing algorithm states message
  // uint32_t timestamp_states = get_sys_time_tow();
  // TODO: 
  float system_time_float = get_sys_time_float();
  uint32_t timestamp_states = (uint32_t)(system_time_float * 1000); // In ms for backwards compatibility, TODO

  // Current UAV position and speed in the NED frame
  float P0_UAV_NED[3] = {stateGetPositionNed_f()->x, stateGetPositionNed_f()->y, stateGetPositionNed_f()->z}; // Current UAV position in the NED frame
  float V0_UAV_NED[3] = {stateGetSpeedNed_f()->x, stateGetSpeedNed_f()->y, stateGetSpeedNed_f()->z}; // Current UAV speed in the NED frame

  //Ship states obtained from the kalman filter:
  float P0_ship_NED[3] = {ship_state.P_ship[0], ship_state.P_ship[1], ship_state.P_ship[2]}; // Current ship position in the NED frame
  float V0_ship_NED[3] = {ship_state.V_ship[0], ship_state.V_ship[1], ship_state.V_ship[2]}; // Current ship speed in the NED frame
  float V0_ship_NED_filt[3] = {ship_state.V_ship[0], ship_state.V_ship[1], ship_state.V_ship[2]}; // Filtered ship speed in the NED frame (TODO)
  float SHIP_att_rad[3] = {ship_state.ship_att_rad[0], ship_state.ship_att_rad[1], ship_state.ship_att_rad[2]}; // Ship attitude in radians
  float UAV_psi_rad = stateGetNedToBodyEulers_f()->psi; // UAV psi angle in radians; TODO



  //Coefficients for the ship speed prediction obtained from the ship speed prediction algorithm; TODO
  float prediction_uav_time = last_ship_coefficients_ship_time - uav_to_ship_time_offset; // Convert ship time to UAV time
  float t_delay_ship_prediction_seconds = system_time_float - prediction_uav_time; // Time delay of ship predictions in seconds
  // printf("last_coefficients_ship_time: %f, uav_to_ship_time_offset: %f, t_delay_ship_prediction_seconds: %f\n", 
  //        last_ship_coefficients_ship_time, uav_to_ship_time_offset, t_delay_ship_prediction_seconds);
  // printf("t_delay_ship_prediction_seconds: %f, system_time_float: %f\n", t_delay_ship_prediction_seconds, system_time_float);

  // Send the landing algorithm parameters
#ifdef EXTRA_DOWNLINK_DEVICE
  pprz_msg_send_IMCU_LANDING_ALGORITHM_STATES(&extra_pprz_tp.trans_tx, &EXTRA_DOWNLINK_DEVICE.device, AC_ID, 
    &timestamp_states, P0_UAV_NED, V0_UAV_NED,
    P0_ship_NED, V0_ship_NED, V0_ship_NED_filt,
    SHIP_att_rad, &UAV_psi_rad,
    ship_coefficients, &t_delay_ship_prediction_seconds);
#endif // EXTRA_DOWNLINK_DEVICE
}

void landing_system_periodic(void) {
  RunOnceEvery(10*LANDING_SYSTEM_PERIODIC_FREQ, {send_landing_algorithm_params();});
  // TODO: is this /2 valid / acceptable practice?
  RunOnceEvery(LANDING_SYSTEM_PERIODIC_FREQ / 100, {request_landing_algorithm_outputs();});
}

void landing_system_parse_ship_info_msg(uint8_t *buf) {
  // The clock might be different for the ship/drone. For the first vesion, we (partially) neglect propagation delay and simply measure the offset directly + average
  float received_timestamp = get_sys_time_float();
  float ship_timestamp = pprzlink_get_DL_SHIP_INFO_MSG_packet_timestamp(buf);
  float clock_offset_measurement = ship_timestamp - received_timestamp;

  if (isnan(uav_to_ship_time_offset)) {
    // First ship measuerment / time offset -> initialize from this measurement
    uav_to_ship_time_offset = clock_offset_measurement;
  } else {
    // Update the time offset
    uav_to_ship_time_offset = uav_to_ship_time_offset + (clock_offset_measurement - uav_to_ship_time_offset) * CLOCK_OFFSET_UPDATE_FACTOR;
    // printf("UAV -> ship time offset: %f, measurement: %f\n", uav_to_ship_time_offset, clock_offset_measurement);
  }

  // Store the ship timestamp on the ship state
  ship_state.ship_timestamp = ship_timestamp;

  // Convert ship lat/lon/alt to our local frame
  struct LlaCoor_i target_pos_lla = {pprzlink_get_DL_SHIP_INFO_MSG_lat(buf), pprzlink_get_DL_SHIP_INFO_MSG_lon(buf), pprzlink_get_DL_SHIP_INFO_MSG_alt(buf)};
  struct NedCoor_i target_pos_cm;
  ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target_pos_lla);

  // Convert to m, store it + speed globally
  ship_state.V_ship[0] = pprzlink_get_DL_SHIP_INFO_MSG_x_dot(buf);
  ship_state.V_ship[1] = pprzlink_get_DL_SHIP_INFO_MSG_y_dot(buf);
  ship_state.V_ship[2] = pprzlink_get_DL_SHIP_INFO_MSG_z_dot(buf);
  ship_state.P_ship[0] = target_pos_cm.x / 100.f;
  ship_state.P_ship[1] = target_pos_cm.y / 100.f;
  ship_state.P_ship[2] = target_pos_cm.z / 100.f;

  // Now for attitude
  ship_state.ship_att_rad[0] = pprzlink_get_DL_SHIP_INFO_MSG_psi(buf) * M_PI / 180.0f; // Convert from degrees to radians
  ship_state.ship_att_rad[1] = pprzlink_get_DL_SHIP_INFO_MSG_theta(buf) * M_PI / 180.0f; // Convert from degrees to radians
  ship_state.ship_att_rad[2] = pprzlink_get_DL_SHIP_INFO_MSG_phi(buf) * M_PI / 180.0f; // Convert from degrees to radians
  // ship_state.psi_ship_rad = ship_state.ship_att_rad[0]; // Ship yaw angle in radians
}

void landing_system_parse_ship_prediction_msg(uint8_t *buf)
{
  float *prediction_x = pprzlink_get_DL_SHIP_PREDICTION_MSG_speed_x_control_coeffs(buf);
  float *prediction_y = pprzlink_get_DL_SHIP_PREDICTION_MSG_speed_y_control_coeffs(buf);
  float *prediction_z = pprzlink_get_DL_SHIP_PREDICTION_MSG_speed_z_control_coeffs(buf);
  
  // Each prediction is a 10-element list consisting of 1/2/3 to indicate axis, then the timestamp and finally 8 coefficients for the 7th order polynomial
  last_ship_coefficients_ship_time = prediction_x[1];
  memcpy(ship_coefficients, prediction_x + 2, 8 * sizeof(float)); // Copy the coefficients for the x-axis
  memcpy(ship_coefficients + 8, prediction_y + 2, 8 * sizeof(float)); // Copy the coefficients for the y-axis
  memcpy(ship_coefficients + 16, prediction_z + 2, 8 * sizeof(float)); // Copy the coefficients for the z-axis
}

// bool nav_landing_system_run(void) {
//   if ((get_sys_time_usec() - landing_algorithm_outputs.last_received_stamp) < USEC_OF_SEC(0.5)) {
//     nav.horizontal_mode = NAV_HORIZONTAL_MODE_WAYPOINT;
//     nav.vertical_mode = NAV_VERTICAL_MODE_ALL;//NAV_VERTICAL_MODE_GUIDED;
//     nav.setpoint_mode = NAV_SETPOINT_MODE_ALL;
//     nav.climb = -1.f;
//     // NOTE: setting mode to all takes care of vertical as well (NOPE)
//     // nav.vertical_mode = NAV_VERTICAL_MODE_GUIDED;
    
//     // struct NedCoor_f accel_sp = {0.f, 0.f, 2.f};
//     struct NedCoor_f accel_sp = {landing_algorithm_outputs.UAV_acc_target_NED[0], landing_algorithm_outputs.UAV_acc_target_NED[1], landing_algorithm_outputs.UAV_acc_target_NED[2]};
    
//     // printf("[landing system] Setting acceleration setpoint to: [%f, %f, %f]\n", accel_sp.x, accel_sp.y, accel_sp.z);
//     VECT3_ENU_OF_NED(nav.accel, accel_sp); // Convert from NED to ENU frame (why is it ENU?!FJDKLSJFDSJ:)

//     // We want to track an acceleration setpoint, so only the FF terms. Set the position/velocity setpoints to the current state to have 0 error contributions from those terms.
//     struct EnuCoor_f pos_sp = *stateGetPositionEnu_f();
//     // pos_sp.z = pos_sp.z - 1.f;
//     struct EnuCoor_f vel_sp = *stateGetSpeedEnu_f();
//     // vel_sp.z -= 1.f;
//     VECT3_COPY(nav.speed, vel_sp);
//     VECT3_COPY(nav.target, pos_sp);
//     nav.heading = M_PI/2; // 90 degree heading -> towards east; TODO: don't hard-code!

//     // Heading
//     // NOTE: heading does not seem to be controlled by setpoint for hybrids (see "take_heading_control")
//     // nav.heading = landing_algorithm_outputs.UAV_desired_phi_theta_rad[0]; // Use the desired yaw angle as heading; TODO: abusing psi, probably shouldn't
//     // printf()

//     return true; // Not complete yet
//   } else {
//     printf("[landing system] No landing algorithm outputs received in the last 0.5 seconds, disengaging.\n");
//     return false; // Disengage
//   }

  
// }