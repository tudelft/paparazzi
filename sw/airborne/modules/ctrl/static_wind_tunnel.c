/*
 * Copyright (C) 2024 Ziqing Ma
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/static_wind_tunnel.c
 * @brief Windtunnel automatic step controller
 *
 */

#include "modules/ctrl/static_wind_tunnel.h"

#include "generated/modules.h"
#include "std.h"

// 20000 PPRZ units/s
#define CMD_STEP_LIM 20000/STATIC_WIND_TUNNEL_FREQUENCY
#define ATI_45_RESOLUTION 752
#define WIND_TUNNEL_FILTER_CUTOFF_HZ 5.0

#define STATIC_WIND_TUNNEL_NUM_CMD 6

#define DISTANCE_Z_TO_CG 0.095

// vars: motor tilt, thrust
#define NUM_VARIABLES 2
#define MAX_NUM_TEST_CASES 5

// Current indices for each variable
int current_indices[NUM_VARIABLES] = {0};
int max_indices[NUM_VARIABLES] = {5, 4};

//    {0, 1, 2}, // vehicle config
const int16_t test_cases[NUM_VARIABLES][MAX_NUM_TEST_CASES] = {
    {-1, 0, 2880, 5760, 9600}, // 0%, 30%, 60%, 100% tilt, use -1 to indicate balancing with tilt
    {-9600, 2880, 4600, 6720}, // 0%, 30%, 50%, 70% thrust
};

//----------------------------------------------
// STATIC WIND TUNNEL EXPERIMENT

// vars: motor tilt, thrust, elevon
#define STATIC_NUM_VARIABLES 3
#define STATIC_MAX_NUM_TEST_CASES 5

// Current indices for each variable
int static_current_indices[STATIC_NUM_VARIABLES] = {0};
int static_max_indices[STATIC_NUM_VARIABLES] = {5, 4, 5};

//    {0, 1, 2}, // vehicle config
const int16_t static_test_cases[STATIC_NUM_VARIABLES][STATIC_MAX_NUM_TEST_CASES] = {
    {0, 2400, 4800, 7200, 9600}, // 0%, 25%, 50%, 70%, 100% tilt
    {-9600, 2880, 4600, 6720}, // 0%, 30%, 50%, 70% thrust
    {0, 2400, 4800, 7200, 9600}, // 0%, 25%, 50%, 70%,100%elevon
};
//----------------------------------------------

int16_t old_thrust = -MAX_PPRZ;
int16_t old_tilt = 0;

struct ForceSensorData {
  float Fx;
  float Fy;
  float Fz;
  float Tx;
  float Ty;
  float Tz;
} force_sensor_data;

struct WT_data wt_data = {
  .max_stage_time = 15.0,
  .des_measurement_time = 8.0,
  .commands = {0},
  .run = false,
  .counter = 0,
  .measurement_counter = 0,
  .wait_for_controller_counter = 0,
  .dynamic_test = true,
  .vehicle_type = 0,
  .ki = 0.1,
  .integrator = 0.0,
  .moment_control_tolerance = 0.1,
};

Butterworth2LowPass pitch_moment_filter;

static void set_commands(int16_t tilt, int16_t thrust, int16_t elevon);
static bool set_next_test_case(void);
int16_t smooth_cmd(int16_t new_cmd, int16_t old_cmd);

void wt_parse_force_sensor_dl(uint8_t *buf)
{
  uint8_t ac_id = DL_FORCE_SENSOR_ac_id(buf);
  if (ac_id != AC_ID) { return; }
  force_sensor_data.Fx = DL_FORCE_SENSOR_Fx(buf);
  force_sensor_data.Fy = DL_FORCE_SENSOR_Fy(buf);
  force_sensor_data.Fz = DL_FORCE_SENSOR_Fz(buf);
  force_sensor_data.Tx = DL_FORCE_SENSOR_Mx(buf);
  force_sensor_data.Ty = DL_FORCE_SENSOR_My(buf);
  force_sensor_data.Tz = DL_FORCE_SENSOR_Mz(buf);
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_wt(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_WIND_TUNNEL(trans, dev, AC_ID,
                            &force_sensor_data.Fx,
                            &force_sensor_data.Fy,
                            &force_sensor_data.Fz,
                            &force_sensor_data.Tx,
                            &force_sensor_data.Ty,
                            &force_sensor_data.Tz,
                            6, wt_data.commands);
}
#endif

void wt_init(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * WIND_TUNNEL_FILTER_CUTOFF_HZ);
  float sample_time = 1.0 / STATIC_WIND_TUNNEL_FREQUENCY;
  // Filtering of the moment measurement
  init_butterworth_2_low_pass(&pitch_moment_filter, tau, sample_time, 0.0);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_TUNNEL, send_wt);
#endif
}

/**
 * Periodic function
 *
 * wt_data.commands are used in the airframe file to set the actuators
 */
void wt_run(void)
{
  float moment_y = force_sensor_data.Ty + force_sensor_data.Fx*DISTANCE_Z_TO_CG;
  update_butterworth_2_low_pass(&pitch_moment_filter, moment_y);

  if (wt_data.run) {
     float time_past = ((float) wt_data.counter / STATIC_WIND_TUNNEL_FREQUENCY);
     float measurement_time = (float) wt_data.measurement_counter / STATIC_WIND_TUNNEL_FREQUENCY;
     wt_data.counter += 1;
    if (wt_data.dynamic_test) {

      if (measurement_time > wt_data.des_measurement_time || time_past > wt_data.max_stage_time) {
        wt_data.counter = 0;
        wt_data.wait_for_controller_counter = 0;
        wt_data.measurement_counter = 0;

        // Check if we need to move to the next test case and update current_indices
        bool more_tests = set_next_test_case();
        if(!more_tests) {
          wt_data.run = false;
          return;
        }
      }

      // Set the actuators to the current test case
      // enum VehicleType vehicle_type = test_cases[0][current_indices[0]];
      // int16_t tilt = test_cases[1][current_indices[1]];
      // int16_t thrust = test_cases[2][current_indices[2]];

      int16_t tilt =   test_cases[0][current_indices[0]];
      int16_t thrust = test_cases[1][current_indices[1]];

      if (wt_data.wait_for_controller_counter < 100) {
        // use the elevon to control the moment
        float error = pitch_moment_filter.o[0];
        wt_data.integrator += ATI_45_RESOLUTION * error;
        // prevent integrator windup
        int32_t integrator_limit = MAX_PPRZ / wt_data.ki;
        BoundAbs(wt_data.integrator, integrator_limit);

        if (fabsf(pitch_moment_filter.o[0]) < wt_data.moment_control_tolerance) {
          wt_data.wait_for_controller_counter +=1;
        } else {
          wt_data.wait_for_controller_counter = 0;
        }
      } else{
        // keep elevon constant
        wt_data.measurement_counter += 1;
      }

      int16_t elevon = wt_data.integrator*wt_data.ki;

      // Handle special case when we want to balance the moment with the tilt
      if (tilt == -1) {
        elevon = 0;
        // balance the moment with the tilt
        tilt = wt_data.integrator*wt_data.ki;
      }

      set_commands(tilt, thrust, elevon);

    } else {
     // Iterate through all combinations of settings
    for (int i = 0; i < STATIC_MAX_NUM_TEST_CASES; i++) {
        for (int j = 0; j < STATIC_MAX_NUM_TEST_CASES - 1;j++) {//only 4 settings for thrust
            for (int k = 0; k < STATIC_MAX_NUM_TEST_CASES; k++) {
                // Get the actuator settings for the current combination
                int16_t tilt = static_test_cases[0][i];
                int16_t thrust = static_test_cases[1][j];
                int16_t elevon = static_test_cases[2][k];
                do
                {
                  set_commands(tilt, thrust, elevon);
                  wt_data.measurement_counter += 1;
                }while (measurement_time < wt_data.des_measurement_time);

            }
        }
    }
    }

  } else {
    // Set everything to 0 by default
    set_commands(0, -MAX_PPRZ, 0);

    wt_data.integrator = 0.0;
    wt_data.counter = 0;
    wt_data.measurement_counter = 0;
    wt_data.wait_for_controller_counter = 0;
  }

}

int16_t smooth_cmd(int16_t new_cmd, int16_t old_cmd) {
  if (new_cmd - old_cmd > CMD_STEP_LIM) {
    return old_cmd + CMD_STEP_LIM;
  } else if (new_cmd - old_cmd < -CMD_STEP_LIM) {
    return old_cmd - CMD_STEP_LIM;
  } else {
    return new_cmd;
  }
}

// Function to get the next test case combination
// Returns false when all combinations are exhausted
bool set_next_test_case(void) {

    // Update the indices to the next combination
    for (int i = 0; i < NUM_VARIABLES; i++) {
        current_indices[i]++;
        if (current_indices[i] < max_indices[i]) {
            // If the current index is valid, stop updating further
            return true;
        } else {
            // Reset the current index and carry over to the next variable
            current_indices[i] = 0;
        }
    }

    // If we reach here, all combinations have been tested
    return false;
}

void set_commands(int16_t tilt, int16_t thrust, int16_t elevon) {

  int16_t new_thrust = smooth_cmd(thrust, old_thrust);
  old_thrust = new_thrust;

  int16_t new_tilt = smooth_cmd(tilt, old_tilt);
  old_tilt = new_tilt;

  wt_data.commands[0] = new_tilt;
  wt_data.commands[1] = new_tilt;
  wt_data.commands[2] = new_thrust;
  wt_data.commands[3] = new_thrust;
  wt_data.commands[4] = elevon;
  wt_data.commands[5] = elevon;
}