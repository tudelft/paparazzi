/*
 * Copyright (C) 2025 AG <info@aerogriduav.com>

 */

/** @file "modules/ag_uav/landing_system.h"
 * @author AG <info@aerogriduav.com>
 * AG landing system interface
 */

#ifndef LANDING_SYSTEM_H
#define LANDING_SYSTEM_H
#include "std.h"

/**
 * Landing system outputs provided by the companion computer to the UAV.
 */
struct landing_algorithm_outputs_t {
  uint32_t last_received_stamp; // Timestamp of the last received message (in pprz/FMU system time)
  uint32_t timestamp_output; // Timestamp of the output
  float UAV_acc_target_NED[3]; // UAV commanded acceleration in NED frame
  float UAV_desired_phi_theta_rad[2]; // UAV commanded roll and pitch in radians
  int8_t landing_algorithm_mode; // Landing algorithm mode
  int8_t exitflag_path_planner; // Exit flag of the path planner
  float expected_landing_time; // Expected landing time in seconds
  uint8_t V_out_of_bounds_array[6]; // Array of out of bounds flags for velocity
  uint8_t A_out_of_bounds_array[6]; // Array of out of bounds flags for acceleration
};

struct ship_state_t {
  float ship_timestamp; // Timestamp of ship state measurement, in ship time
  float P_ship[3]; // Initial position of the ship in NED frame
  float V_ship[3]; // Initial velocity of the ship in NED frame
  float ship_att_rad[3];
  float psi_ship_rad; // Ship yaw angle in radians
};

extern void landing_system_init(void); 
extern void landing_system_periodic(void);
extern void receive_landing_algorithm_outputs(uint8_t *buf);

extern void landing_system_parse_ship_info_msg(uint8_t *buf);

extern void landing_system_parse_ship_prediction_msg(uint8_t *buf);

extern bool nav_landing_system_run(void);

void send_landing_algorithm_params(void);
void request_landing_algorithm_outputs(void);

#endif  // LANDING_SYSTEM_H
