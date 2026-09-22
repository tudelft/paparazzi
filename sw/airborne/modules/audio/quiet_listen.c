/*
 * Copyright (C) 2026 TU Delft
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/audio/quiet_listen.c
 *  Low-throttle listening for EARcam, see quiet_listen.h.
 */

#include "modules/audio/quiet_listen.h"
#include "modules/core/abi.h"
#include "modules/nav/common_flight_plan.h"
#include "modules/digital_cam/earcam_ctrl.h"
#include "firmwares/fixedwing/guidance/energy_ctrl.h"
#include "mcu_periph/sys_time.h"

#define QUIET_LISTEN_NGAINS 8

static float quiet_listen_saved[QUIET_LISTEN_NGAINS];
static bool quiet_listen_active = false;
static float quiet_listen_until = 0.0f;
static uint8_t quiet_listen_block = 0;

static bool session_on = false;
static uint8_t session_block = 0;

static abi_event gps_ev;

static void quiet_listen_gps_cb(uint8_t sender_id __attribute__((unused)),
                                uint32_t stamp __attribute__((unused)),
                                struct GpsState *gps_s __attribute__((unused)))
{
  quiet_listen_check();
  quiet_listen_session_check();
}

void quiet_listen_init(void)
{
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, quiet_listen_gps_cb);
}

void quiet_listen_off(void)
{
  if (!quiet_listen_active) { return; }
  v_ctl_auto_throttle_nominal_cruise_throttle = quiet_listen_saved[0];
  v_ctl_auto_throttle_climb_throttle_increment = quiet_listen_saved[1];
  v_ctl_auto_throttle_of_airspeed_pgain = quiet_listen_saved[2];
  v_ctl_auto_throttle_of_airspeed_igain = quiet_listen_saved[3];
  v_ctl_energy_total_pgain = quiet_listen_saved[4];
  v_ctl_energy_total_igain = quiet_listen_saved[5];
  v_ctl_energy_bank_throttle_gain = quiet_listen_saved[6];
  v_ctl_energy_bank_washout_gain = quiet_listen_saved[7];
  earcam_listen = false;
  quiet_listen_active = false;
}

void quiet_listen_on(float listen_thr)
{
  if (!quiet_listen_active) {
    quiet_listen_saved[0] = v_ctl_auto_throttle_nominal_cruise_throttle;
    quiet_listen_saved[1] = v_ctl_auto_throttle_climb_throttle_increment;
    quiet_listen_saved[2] = v_ctl_auto_throttle_of_airspeed_pgain;
    quiet_listen_saved[3] = v_ctl_auto_throttle_of_airspeed_igain;
    quiet_listen_saved[4] = v_ctl_energy_total_pgain;
    quiet_listen_saved[5] = v_ctl_energy_total_igain;
    quiet_listen_saved[6] = v_ctl_energy_bank_throttle_gain;
    quiet_listen_saved[7] = v_ctl_energy_bank_washout_gain;
    v_ctl_auto_throttle_nominal_cruise_throttle = listen_thr;
    v_ctl_auto_throttle_climb_throttle_increment = 0.f;
    v_ctl_auto_throttle_of_airspeed_pgain = 0.f;
    v_ctl_auto_throttle_of_airspeed_igain = 0.f;
    v_ctl_energy_total_pgain = 0.f;
    v_ctl_energy_total_igain = 0.f;
    v_ctl_energy_bank_throttle_gain = 0.f;
    v_ctl_energy_bank_washout_gain = 0.f;
    quiet_listen_block = nav_block;
    earcam_listen = true;
    quiet_listen_active = true;
  }
  quiet_listen_until = get_sys_time_float() + QUIET_LISTEN_HOLD_S;
}

void quiet_listen_check(void)
{
  if (!quiet_listen_active) { return; }
  if (get_sys_time_float() >= quiet_listen_until || nav_block != quiet_listen_block) {
    quiet_listen_off();
  }
}

void quiet_listen_session_hold(void)
{
  session_on = true;
  session_block = nav_block;
}

void quiet_listen_session_stop(void)
{
  if (!session_on) { return; }
  earcam_stop();
  earcam_listen = true;
  session_on = false;
}

void quiet_listen_session_check(void)
{
  if (session_on && nav_block != session_block) {
    quiet_listen_session_stop();
  }
}

bool quiet_listen_guard(void)
{
  quiet_listen_check();
  quiet_listen_session_check();
  return false;
}
