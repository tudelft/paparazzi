/*
 * Copyright (C) 2008-2012 The Paparazzi Team
 * Copyright (C) 2016-2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rotorcraft/autopilot.c
 *
 * Autopilot.
 *
 */

#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "generated/modules.h"

#include <stdint.h>
//#include "mcu_periph/sys_time.h"
#include "modules/energy/electrical.h"
#include "modules/datalink/telemetry.h"
#include "modules/radio_control/radio_control.h"

#if USE_GPS
#include "modules/gps/gps.h"
#else
#if NO_GPS_NEEDED_FOR_NAV
#define GpsIsLost() FALSE
#else
#define GpsIsLost() TRUE
#endif
#endif

uint8_t  autopilot_mode_auto2;
static uint32_t autopilot_in_flight_counter;

/* Geofence exceptions */
#include "modules/nav/nav_geofence.h"

/** time steps for in_flight detection (at 20Hz, so 20=1second) */
#ifndef AUTOPILOT_IN_FLIGHT_TIME
#define AUTOPILOT_IN_FLIGHT_TIME    20
#endif

/** minimum vertical speed for in_flight condition in m/s */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_SPEED
#define AUTOPILOT_IN_FLIGHT_MIN_SPEED 0.2
#endif

/** minimum vertical acceleration for in_flight condition in m/s^2 */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_ACCEL
#define AUTOPILOT_IN_FLIGHT_MIN_ACCEL 2.0
#endif

/** minimum thrust for in_flight condition in pprz_t units (max = 9600) */
#ifndef AUTOPILOT_IN_FLIGHT_MIN_THRUST
#define AUTOPILOT_IN_FLIGHT_MIN_THRUST 500
#endif

/** Z-acceleration threshold to detect ground in m/s^2 */
#ifndef THRESHOLD_GROUND_DETECT
#define THRESHOLD_GROUND_DETECT 25.0
#endif

/** Default ground-detection estimation based on accelerometer shock */
bool WEAK autopilot_ground_detection(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
  if (accel->z < -THRESHOLD_GROUND_DETECT ||
      accel->z > THRESHOLD_GROUND_DETECT) {
    return true;
  }
  return false;
}


/** Default end-of-in-flight detection estimation based on thrust and speed */
bool WEAK autopilot_in_flight_end_detection(bool motors_on UNUSED) {
  if (autopilot_in_flight_counter > 0) {
    /* probably in_flight if thrust, speed and accel above IN_FLIGHT_MIN thresholds */
    if ((stabilization.cmd[COMMAND_THRUST] <= AUTOPILOT_IN_FLIGHT_MIN_THRUST) &&
        (fabsf(stateGetSpeedNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_SPEED) &&
        (fabsf(stateGetAccelNed_f()->z) < AUTOPILOT_IN_FLIGHT_MIN_ACCEL)) {
      autopilot_in_flight_counter--;
      if (autopilot_in_flight_counter == 0) {
        return true;
      }
    } else { /* thrust, speed or accel not above min threshold, reset counter */
      autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
    }
  }
  return false;
}


#if USE_MOTOR_MIXING
#include "modules/actuators/motor_mixing.h"
#endif

static void send_energy(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t throttle = 100 * autopilot.throttle / MAX_PPRZ;
  float power = electrical.vsupply * electrical.current;
  float avg_power = 0;
  if(electrical.avg_cnt != 0) {
    avg_power = (float)electrical.avg_power / electrical.avg_cnt;
  }

  pprz_msg_send_ENERGY(trans, dev, AC_ID,
                       &throttle, &electrical.vsupply, &electrical.current, &power, &avg_power, &electrical.charge, &electrical.energy);
}

static void send_body_rates_accel(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_BODY_RATES_ACCEL(trans, dev, AC_ID,
                                  &(stateGetBodyRates_f()->p),
                                  &(stateGetBodyRates_f()->q),
                                  &(stateGetBodyRates_f()->r),
                                  &(stateGetAccelBody_i()->x),
                                  &(stateGetAccelBody_i()->y),
                                  &(stateGetAccelBody_i()->z));
}

static void send_fp_min(struct transport_tx *trans, struct link_device *dev)
{
#if USE_GPS
  uint16_t gspeed = gps.gspeed;
#else
  // ground speed in cm/s
  uint16_t gspeed = stateGetHorizontalSpeedNorm_f() / 100;
#endif
  pprz_msg_send_ROTORCRAFT_FP_MIN(trans, dev, AC_ID,
                                  &(stateGetPositionEnu_i()->x),
                                  &(stateGetPositionEnu_i()->y),
                                  &(stateGetPositionEnu_i()->z),
                                  &gspeed);
}

#ifdef RADIO_CONTROL
static void send_rotorcraft_rc(struct transport_tx *trans, struct link_device *dev)
{
#ifdef RADIO_KILL_SWITCH
  int16_t _kill_switch = radio_control.values[RADIO_KILL_SWITCH];
#else
  int16_t _kill_switch = 42;
#endif
  pprz_msg_send_ROTORCRAFT_RADIO_CONTROL(trans, dev, AC_ID,
                                         &radio_control.values[RADIO_ROLL],
                                         &radio_control.values[RADIO_PITCH],
                                         &radio_control.values[RADIO_YAW],
                                         &radio_control.values[RADIO_THROTTLE],
                                         &radio_control.values[RADIO_MODE],
                                         &_kill_switch,
                                         &radio_control.status);
}
#endif

#if defined(COMMAND_ROLL) && defined(COMMAND_PITCH) && defined(COMMAND_YAW)
static void send_rotorcraft_cmd(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROTORCRAFT_CMD(trans, dev, AC_ID,
                               &stabilization.cmd[COMMAND_ROLL],
                               &stabilization.cmd[COMMAND_PITCH],
                               &stabilization.cmd[COMMAND_YAW],
                               &stabilization.cmd[COMMAND_THRUST]);
}
#else
static void send_rotorcraft_cmd(struct transport_tx *trans UNUSED, struct link_device *dev UNUSED) {}
#endif

/** autopilot event function
 *
 * used for automatic ground detection
 */
void autopilot_event(void)
{
  if (autopilot.detect_ground_once
#ifdef AP_MODE_FAILSAFE
      || autopilot.mode == AP_MODE_FAILSAFE
#endif
     ) {
    if (autopilot_ground_detection()) {
      autopilot.ground_detected = true;
      autopilot.detect_ground_once = false;
    }
  }
}

/** reset in_flight counter
 */
void autopilot_reset_in_flight_counter(void)
{
  autopilot_in_flight_counter = 0;
}

/** in flight check utility function
 */
void autopilot_check_in_flight(bool motors_on)
{
  if (autopilot.in_flight) {
    if (autopilot_in_flight_end_detection(motors_on)) {
      autopilot.in_flight = false;
      autopilot_in_flight_counter = 0;
    }
  } else { /* currently not in flight */
    if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME &&
        motors_on) {
      /* if thrust above min threshold, assume in_flight.
       * Don't check for velocity and acceleration above threshold here...
       */
      if (stabilization.cmd[COMMAND_THRUST] > AUTOPILOT_IN_FLIGHT_MIN_THRUST) {
        autopilot_in_flight_counter++;
        if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME) {
          autopilot.in_flight = true;
        }
      } else { /* currently not in_flight and thrust below threshold, reset counter */
        autopilot_in_flight_counter = 0;
      }
    }
  }
}

