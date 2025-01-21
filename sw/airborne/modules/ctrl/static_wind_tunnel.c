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

// 20000 PPRZ units/s
#define THRUST_STEP_LIM 20000/STATIC_WIND_TUNNEL_FREQUENCY
#define ATI_45_RESOLUTION 752

#define STATIC_WIND_TUNNEL_NUM_CMD 6

// vars: motor tilt, thrust
#define NUM_VARIABLES 3
#define MAX_NUM_TEST_CASES 4

const int16_t test_cases[NUM_VARIABLES][MAX_NUM_TEST_CASES] = {
    {0, 1, 2}, // vehicle config
    {0, 2880, 5760, 9600}, // 0%, 30%, 60%, 100% tilt
    {-9600, 2880, 4600, 6720}, // 0%, 30%, 50%, 70% thrust
};

// Current indices for each variable
int current_indices[NUM_VARIABLES] = {0};
int max_indices[NUM_VARIABLES] = {4,4};

int16_t old_thrust = -MAX_PPRZ;

struct ForceSensorData {
  float Fx;
  float Fy;
  float Fz;
  float Tx;
  float Ty;
  float Tz;
} force_sensor_data;

struct WT_data wt_data = {
  .measurement_time = 10.0,
  .commands = {0},
  .run = false,
  .counter = 0,
  .stage = 0,
  .dynamic_test = false,
  .vehicle_type = 0,
  .ki = 0.8,
  .integrator = 0.0,
};

static void set_commands(int16_t tilt, int16_t thrust, int16_t elevon);
static bool set_next_test_case(void);

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
  if (wt_data.run) {
    if (wt_data.dynamic_test) {

      wt_data.counter = wt_data.counter + 1;

      float time_past = (wt_data.counter / STATIC_WIND_TUNNEL_FREQUENCY);

      if (time_past > wt_data.measurement_time) {
        wt_data.counter = 0;

        // Check if we need to move to the next test case and update current_indices
        bool more_tests = set_next_test_case();
        if(!more_tests) {
          wt_data.run = false;
          return;
        }
      }

      // Set the actuators to the current test case
      enum VehicleType vehicle_type = test_cases[0][current_indices[0]];
      int16_t tilt = test_cases[1][current_indices[1]];
      int16_t thrust = test_cases[2][current_indices[2]];

      // use the elevon to control the moment
      float error = -force_sensor_data.Ty;
      wt_data.integrator += ATI_45_RESOLUTION * error;
      int16_t elevon = wt_data.integrator*wt_data.ki;

      set_commands(tilt, thrust, elevon);

    } else {
      // TODO: not implemented for now
      // Set everything to 0 by default
      set_commands(0, -MAX_PPRZ, 0);

      wt_data.integrator = 0.0;
      wt_data.counter = 0;
    }

  } else {
    // Set everything to 0 by default
    set_commands(0, -MAX_PPRZ, 0);

    wt_data.integrator = 0.0;
    wt_data.counter = 0;
  }

}

int16_t smooth_thrust(int16_t new_thrust) {
  if (new_thrust - old_thrust > THRUST_STEP_LIM) {
    return old_thrust + THRUST_STEP_LIM;
  } else if (new_thrust - old_thrust < -THRUST_STEP_LIM) {
    return old_thrust - THRUST_STEP_LIM;
  } else {
    return new_thrust;
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

  int16_t new_thrust = smooth_thrust(thrust);
  old_thrust = new_thrust;

  wt_data.commands[0] = tilt;
  wt_data.commands[1] = tilt;
  wt_data.commands[2] = new_thrust;
  wt_data.commands[3] = new_thrust;
  wt_data.commands[4] = elevon;
  wt_data.commands[5] = elevon;
}