/*
 * Copyright (C) Joost Meulenbeld
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/helicopter/sys_id_doublet.c"
 * @author Joost Meulenbeld
 * System identification doublet. See header for doc
 */

#include "modules/system_identification/sys_id_doublet.h"
#include "subsystems/datalink/telemetry.h"

bool doublet_active = false;
float doublet_amplitude_deg = 0;
float doublet_length_s = 2;
uint8_t doublet_axis = 0;

static float doublet_current_value = 0;
static float doublet_start_time = 0;
static float doublet_time_since_start = 0;

// Set doublet to inactive; clear values in doublet array
static void stop_doublet(void) {
  doublet_current_value = 0;
  doublet_start_time = 0;
  doublet_active = false;
  doublet_time_since_start = 0;
}

// Set doublet to active; save the current time to the start time
static void start_doublet(void) {
  doublet_current_value = 0;
  doublet_start_time = get_sys_time_float();
  doublet_active = true;
  doublet_time_since_start = 0;
}

// Implementation of doublet function
static float calc_doublet_value(float t_since_start, float length) {
  if (t_since_start < length / 2 && t_since_start >= 0)
    return 1;
  else if (t_since_start < length && t_since_start >= length / 2)
    return -1;
  return 0;
}

// If doublet time has passed, stop doublet. If doublet active, store doublet output in doublet_current_value
static void update_doublet_value(void) {
  doublet_time_since_start = get_sys_time_float() - doublet_start_time;
  if (doublet_time_since_start >= doublet_length_s) 
    stop_doublet();

  if (doublet_active)
    doublet_current_value = RadOfDeg(doublet_amplitude_deg) * calc_doublet_value(doublet_time_since_start, doublet_length_s);
  else
    doublet_current_value = 0;
}

static void send_doublet(struct transport_tx *trans, struct link_device *dev) {
    pprz_msg_send_DOUBLET(trans, dev, AC_ID, 
      ((uint8_t*) &doublet_active),
      &doublet_start_time,
      &doublet_time_since_start,
      &doublet_current_value);
}


void sys_id_doublet_init(void) {
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DOUBLET, send_doublet);
}

void sys_id_doublet_doublet_activate_handler(uint8_t activate) {
  if (activate)
    start_doublet();
  else
    stop_doublet();
}

void sys_id_doublet_add_rate(struct FloatRates* rates_sp) {
  update_doublet_value();

  switch (doublet_axis) {
    case 0:
      rates_sp->p += doublet_current_value;
      break;
    case 1:
      rates_sp->q += doublet_current_value;
      break;
    case 2:
      rates_sp->r += doublet_current_value;
      break;
    default: // to shut the compiler warnings up
      break;
  }

}

void sys_id_doublet_add_attitude(struct Int32Quat* att_sp_quat) {
  update_doublet_value();

  // Convert quaternion setpoint to eulers
  struct Int32Eulers att_sp_eulers;
  int32_t doublet_value_i = ANGLE_BFP_OF_REAL(doublet_current_value);
  int32_eulers_of_quat(&att_sp_eulers, att_sp_quat);

  // Add euler value to appropriate axis
  switch (doublet_axis) {
    case 0:
      att_sp_eulers.phi += doublet_value_i;
      break;
    case 1:
      att_sp_eulers.theta += doublet_value_i;
      break;
    case 2:
      att_sp_eulers.psi += doublet_value_i;
      break;
    default: // to shut the compiler warnings up
      break;
  }

  // Convert resulting euler back to quaternion
  int32_quat_of_eulers(att_sp_quat, &att_sp_eulers);
}
