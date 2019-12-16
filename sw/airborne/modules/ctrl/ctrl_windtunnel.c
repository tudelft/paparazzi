/*
 * Copyright (C) 2015
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
 * @file modules/ctrl/ctrl_module_innerloop_demo.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/ctrl_windtunnel.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/electrical.h"

struct ctrl_windtunnel_struct {
  int rc_throttle;
  int rc_roll;
  int rc_pitch;
  int rc_yaw;
} ctrl_windtunnel;

float ctrl_windtunnel_steptime = 3.0;
struct min_max_ctrl_t ctrl_windtunnel_throttle;
struct min_max_ctrl_t ctrl_windtunnel_flaps;
static float last_time = 0;

void ctrl_module_init(void);
void ctrl_module_run(bool in_flight);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_windtunnel_meas(struct transport_tx *trans, struct link_device *dev)
{
  float aoa = 0.0f;
  float power = electrical.vsupply * electrical.current;
  pprz_msg_send_WINDTUNNEL_MEAS(trans, dev, AC_ID, &aoa, &air_data.airspeed, &electrical.vsupply, &electrical.current, &power, COMMANDS_NB, stabilization_cmd);
}
#endif

void ctrl_module_init(void)
{
  ctrl_windtunnel.rc_throttle = 0;
  ctrl_windtunnel.rc_roll = 0;
  ctrl_windtunnel.rc_pitch = 0;
  ctrl_windtunnel.rc_yaw = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WINDTUNNEL_MEAS, send_windtunnel_meas);
#endif
}

void ctrl_module_run(bool in_flight)
{
  if (!in_flight) {
    stabilization_cmd[COMMAND_ROLL] = 0;
    stabilization_cmd[COMMAND_PITCH] = 0;
    stabilization_cmd[COMMAND_YAW] = 0;
    stabilization_cmd[COMMAND_THRUST] = 0;
    stabilization_cmd[COMMAND_FLAPS] = 0;

    ctrl_windtunnel_throttle.current = ctrl_windtunnel_throttle.min;
    ctrl_windtunnel_flaps.current = ctrl_windtunnel_flaps.min;
    last_time = get_sys_time_float();
  } else {
    bool done = false;
    // Increase step in steptime
    if(ctrl_windtunnel.rc_throttle > (MAX_PPRZ/2) && (get_sys_time_float() - last_time) > ctrl_windtunnel_steptime) { 
      // Increase throttle step if flaps at the end
      if((ctrl_windtunnel_flaps.current + ctrl_windtunnel_flaps.step) > ctrl_windtunnel_flaps.max) {
        // Only increase step if throttle is not at the end
        if((ctrl_windtunnel_throttle.current + ctrl_windtunnel_throttle.step) <= ctrl_windtunnel_throttle.max) {
          ctrl_windtunnel_throttle.current += ctrl_windtunnel_throttle.step;
          ctrl_windtunnel_flaps.current = ctrl_windtunnel_flaps.min;
          last_time = get_sys_time_float();
        }
        else {
          // Finished
          done = true;
        }
      }
      else {
        // By default increase flaps
        ctrl_windtunnel_flaps.current += ctrl_windtunnel_flaps.step;
        last_time = get_sys_time_float();
      }
    } else if (ctrl_windtunnel.rc_throttle < (MAX_PPRZ/2)) {
      // RESET
      ctrl_windtunnel_throttle.current = ctrl_windtunnel_throttle.min;
      ctrl_windtunnel_flaps.current = ctrl_windtunnel_flaps.min;
      last_time = get_sys_time_float();
    }

    stabilization_cmd[COMMAND_ROLL] = 0;
    stabilization_cmd[COMMAND_PITCH] = 0;
    stabilization_cmd[COMMAND_YAW] = 0;
    stabilization_cmd[COMMAND_THRUST] = (done)? 0 : ctrl_windtunnel_throttle.current;
    stabilization_cmd[COMMAND_FLAPS] = (done)? 0 : ctrl_windtunnel_flaps.current;
  }
}


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  ctrl_module_init();
}

void guidance_h_module_enter(void)
{
  ctrl_module_init();
}

void guidance_h_module_read_rc(void)
{
  // -MAX_PPRZ to MAX_PPRZ
  ctrl_windtunnel.rc_throttle = radio_control.values[RADIO_THROTTLE];
  ctrl_windtunnel.rc_roll = radio_control.values[RADIO_ROLL];
  ctrl_windtunnel.rc_pitch = radio_control.values[RADIO_PITCH];
  ctrl_windtunnel.rc_yaw = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  ctrl_module_run(in_flight);
}

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  // your vertical controller goes here
}
