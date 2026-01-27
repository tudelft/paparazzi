/*
 * Copyright (C) 2008-2012 The Paparazzi Team
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file firmwares/rotorcraft/autopilot_static.c
 *
 * Static autopilot implementation.
 *
 */

#include "autopilot.h"
#include "autopilot_arming.h"

#include "modules/radio_control/radio_control.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "modules/energy/electrical.h"
#include "modules/core/settings.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#if USE_STABILIZATION_RATE
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#endif

#include "firmwares/rotorcraft/autopilot_guided.h"

#include "generated/settings.h"

#if USE_GPS
#include "modules/gps/gps.h"
#else
#if NO_GPS_NEEDED_FOR_NAV
#define GpsIsLost() FALSE
#else
#define GpsIsLost() TRUE
#endif
#endif

/* Geofence exceptions */
#include "modules/nav/nav_geofence.h"

/** Mode that is set when the plane is really too far from home */
#ifndef FAILSAFE_MODE_TOO_FAR_FROM_HOME
#define FAILSAFE_MODE_TOO_FAR_FROM_HOME AP_MODE_FAILSAFE
#endif


#ifndef MODE_STARTUP
#define MODE_STARTUP AP_MODE_KILL
PRINT_CONFIG_MSG("Using default AP_MODE_KILL as MODE_STARTUP")
#endif

#ifndef UNLOCKED_HOME_MODE
#if MODE_AUTO1 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO1 is AP_MODE_HOME")
#elif MODE_AUTO2 == AP_MODE_HOME
#define UNLOCKED_HOME_MODE TRUE
PRINT_CONFIG_MSG("Enabled UNLOCKED_HOME_MODE since MODE_AUTO2 is AP_MODE_HOME")
#else
#define UNLOCKED_HOME_MODE FALSE
#endif
#endif

#if MODE_MANUAL == AP_MODE_NAV
#error "MODE_MANUAL mustn't be AP_MODE_NAV"
#endif

static void send_minimal_com(struct transport_tx *trans, struct link_device *dev)
{
  float lat = DegOfRad(stateGetPositionLla_f()->lat);
  float lon = DegOfRad(stateGetPositionLla_f()->lon);
  float hmsl = stateGetPositionUtm_f()->alt;
  float gspeed = stateGetHorizontalSpeedNorm_f();
  float course = stateGetHorizontalSpeedDir_f();
  float climb = stateGetSpeedEnu_f()->z;
  uint8_t throttle = (uint8_t)(100 * autopilot.throttle / MAX_PPRZ);
#if USE_GPS
  uint8_t gps_fix = gps.fix;
#else
  uint8_t gps_fix = 0;
#endif
  pprz_msg_send_MINIMAL_COM(trans, dev, AC_ID,
                            &lat, &lon, &hmsl, &gspeed, &course, &climb,
                            &electrical.vsupply, &throttle, &autopilot.mode,
                            &nav_block, &gps_fix, &autopilot.flight_time);
}

static void send_status(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t imu_nb_err = 0;
#if USE_MOTOR_MIXING
  uint8_t _motor_nb_err = motor_mixing.nb_saturation + motor_mixing.nb_failure * 10;
#else
  uint8_t _motor_nb_err = 0;
#endif
#if USE_GPS
  uint8_t fix = gps.fix;
#else
  uint8_t fix = 0;
#endif
  uint8_t in_flight = autopilot.in_flight;
  uint8_t motors_on = autopilot.motors_on;
  uint16_t time_sec = sys_time.nb_sec;
  pprz_msg_send_ROTORCRAFT_STATUS(trans, dev, AC_ID,
                                  &imu_nb_err, &_motor_nb_err,
                                  &radio_control.status, &radio_control.frame_rate,
                                  &fix, &autopilot.mode, &in_flight, &motors_on,
                                  &autopilot.arming_status, &guidance_h.mode, &guidance_v.mode,
                                  &time_sec, &electrical.vsupply, &electrical.vboard);
}

static void send_fp(struct transport_tx *trans, struct link_device *dev)
{
  int32_t carrot_up = -guidance_v.z_sp;
  int32_t carrot_heading = ANGLE_BFP_OF_REAL(guidance_h.sp.heading);
  int32_t thrust = (int32_t)autopilot.throttle;
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
#if GUIDANCE_INDI_HYBRID
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
  struct Int32Eulers att;
  EULERS_BFP_OF_REAL(att, eulers_zxy);
#else
  struct Int32Eulers att = *stateGetNedToBodyEulers_i();
#endif
  pprz_msg_send_ROTORCRAFT_FP(trans, dev, AC_ID,
                              &pos->x,
                              &pos->y,
                              &pos->z,
                              &(stateGetSpeedEnu_i()->x),
                              &(stateGetSpeedEnu_i()->y),
                              &(stateGetSpeedEnu_i()->z),
                              &att.phi,
                              &att.theta,
                              &att.psi,
                              &guidance_h.sp.pos.y,
                              &guidance_h.sp.pos.x,
                              &carrot_up,
                              &carrot_heading,
                              &thrust,
                              &autopilot.flight_time);
}

void autopilot_firmware_init(void)
{
  autopilot_in_flight_counter = 0;
#ifdef MODE_AUTO2
  autopilot_mode_auto2 = MODE_AUTO2;
#endif

  // register messages
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_STATUS, send_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ENERGY, send_energy);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP, send_fp);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_FP_MIN, send_fp_min);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_CMD, send_rotorcraft_cmd);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BODY_RATES_ACCEL, send_body_rates_accel);
#ifdef RADIO_CONTROL
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_RADIO_CONTROL, send_rotorcraft_rc);
#endif
}

void autopilot_send_mode(void)
{
#if DOWNLINK
  send_status(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
}

void autopilot_static_init(void)
{
  /* Mode is finally set by autopilot_static_set_mode if MODE_STARTUP is not KILL.
   * For autopilot_static_set_mode to do anything, the requested mode needs to differ
   * from previous mode, so we set it to a safe KILL first.
   */
  autopilot.mode = AP_MODE_KILL;

  /* set startup mode, propagates through to guidance h/v */
  autopilot_static_set_mode(MODE_STARTUP);

  /* init arming */
  autopilot_arming_init();
}


#define NAV_PRESCALER (PERIODIC_FREQUENCY / NAVIGATION_FREQUENCY)
void autopilot_static_periodic(void)
{

  RunOnceEvery(NAV_PRESCALER, compute_dist2_to_home());

  if (autopilot_in_flight() && autopilot.mode == AP_MODE_NAV) {
    if (nav.too_far_from_home || datalink_lost() || higher_than_max_altitude()) {
      if (nav.dist2_to_home > nav.failsafe_mode_dist2) {
        autopilot_static_set_mode(FAILSAFE_MODE_TOO_FAR_FROM_HOME);
      } else {
        autopilot_static_set_mode(AP_MODE_HOME);
      }
    }
  }

  if (autopilot.mode == AP_MODE_HOME) {
    RunOnceEvery(NAV_PRESCALER, nav_home());
  } else {
    // otherwise always call nav_periodic_task so that carrot is always updated in GCS for other modes
    RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
  }


  /* If in FAILSAFE mode and either already not in_flight anymore
   * or just "detected" ground, go to KILL mode.
   */
  if (autopilot.mode == AP_MODE_FAILSAFE) {
    if (!autopilot_in_flight()) {
      autopilot_static_set_mode(AP_MODE_KILL);
    }

#if FAILSAFE_GROUND_DETECT
    INFO("Using FAILSAFE_GROUND_DETECT: KILL")
    if (autopilot.ground_detected) {
      autopilot_static_set_mode(AP_MODE_KILL);
    }
#endif
  }

  /* Reset ground detection _after_ running flight plan
   */
  if (!autopilot_in_flight()) {
    autopilot.ground_detected = false;
    autopilot.detect_ground_once = false;
  }

  /* Set fixed "failsafe" commands from airframe file if in KILL mode.
   * If in FAILSAFE mode, run normal loops with failsafe attitude and
   * downwards velocity setpoints.
   */
  struct StabilizationSetpoint stab_sp;
  struct ThrustSetpoint thrust_sp;
  switch (autopilot.mode) {
    case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
#ifdef FAILSAFE_THROTTLE
      thrust_sp = th_sp_from_thrust_i(FAILSAFE_THROTTLE, THRUST_AXIS_Z);
#else
      thrust_sp = guidance_v_run(autopilot_in_flight());
#endif
      stab_sp = stabilization_get_failsafe_sp();
      stabilization_run(autopilot_in_flight(), &stab_sp, &thrust_sp, stabilization.cmd);
      SetRotorcraftCommands(stabilization.cmd, autopilot.in_flight, autopilot.motors_on);
      break;
#endif
    case AP_MODE_KILL:
      SetCommands(commands_failsafe);
      break;
    default:
      thrust_sp = guidance_v_run(autopilot_in_flight());
      if (guidance_h.mode != GUIDANCE_H_MODE_NONE) {
        stab_sp = guidance_h_run(autopilot_in_flight());
      } else {
        stab_sp = stabilization.rc_sp;
      }
      stabilization_run(autopilot_in_flight(), &stab_sp, &thrust_sp, stabilization.cmd);
      // TODO maybe add RC limiter here as an option ?
      SetRotorcraftCommands(stabilization.cmd, autopilot.in_flight, autopilot.motors_on);
      break;
  }
#ifdef COMMAND_THRUST
  autopilot.throttle = commands[COMMAND_THRUST];
#endif

}

/** AP mode setting handler
 *
 * Checks RC status before calling autopilot_static_set_mode function
 */
void autopilot_static_SetModeHandler(float mode)
{
  if (mode == AP_MODE_KILL || mode == AP_MODE_FAILSAFE || mode == AP_MODE_HOME) {
    // safety modes are always accessible via settings
    autopilot_static_set_mode(mode);
  } else {
    if (radio_control.status != RC_OK &&
        (mode == AP_MODE_NAV || mode == AP_MODE_GUIDED)) {
      // without RC, only nav-like modes are accessible
      autopilot_static_set_mode(mode);
    }
  }
  // with RC, other modes can only be changed from the RC
}


void autopilot_static_set_mode(uint8_t new_autopilot_mode)
{
  if (new_autopilot_mode != autopilot.mode) {
    /* horizontal mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
        stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE,
            STABILIZATION_ATT_SUBMODE_HEADING);
        break;
#endif
      case AP_MODE_KILL:
        autopilot_set_in_flight(false);
        guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
        stabilization_mode_changed(STABILIZATION_MODE_NONE, 0);
        break;
      case AP_MODE_RC_DIRECT:
        guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
        stabilization_mode_changed(STABILIZATION_MODE_DIRECT, 0);
        break;
      case AP_MODE_RATE_RC_CLIMB:
      case AP_MODE_RATE_DIRECT:
      case AP_MODE_RATE_Z_HOLD:
#if USE_STABILIZATION_RATE
        guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
        stabilization_mode_changed(STABILIZATION_MODE_RATE, 0);
#else
        return;
#endif
        break;
      case AP_MODE_ATTITUDE_RC_CLIMB:
      case AP_MODE_ATTITUDE_DIRECT:
      case AP_MODE_ATTITUDE_CLIMB:
      case AP_MODE_ATTITUDE_Z_HOLD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
        stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE,
            STABILIZATION_ATT_SUBMODE_HEADING);
        break;
      case AP_MODE_FORWARD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
        stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE,
            STABILIZATION_ATT_SUBMODE_FORWARD);
        break;
      case AP_MODE_CARE_FREE_DIRECT:
        guidance_h_mode_changed(GUIDANCE_H_MODE_NONE);
        stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE,
            STABILIZATION_ATT_SUBMODE_CARE_FREE);
        break;
      case AP_MODE_HOVER_DIRECT:
      case AP_MODE_HOVER_CLIMB:
      case AP_MODE_HOVER_Z_HOLD:
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE,
            STABILIZATION_ATT_SUBMODE_HEADING);
        break;
      case AP_MODE_HOME:
      case AP_MODE_NAV:
        guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
        stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE,
            STABILIZATION_ATT_SUBMODE_HEADING); // TODO check
        break;
      case AP_MODE_GUIDED:
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE,
            STABILIZATION_ATT_SUBMODE_HEADING);
        break;
      default:
        break;
    }
    /* vertical mode */
    switch (new_autopilot_mode) {
      case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
        guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
        guidance_v_set_vz(FAILSAFE_DESCENT_SPEED);
        break;
#endif
      case AP_MODE_KILL:
        autopilot_set_motors_on(false);
#ifdef COMMAND_THRUST
        stabilization.cmd[COMMAND_THRUST] = 0; // FIXME maybe not needed ?
#endif
        guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
        break;
      case AP_MODE_RC_DIRECT:
      case AP_MODE_RATE_DIRECT:
      case AP_MODE_ATTITUDE_DIRECT:
      case AP_MODE_HOVER_DIRECT:
      case AP_MODE_CARE_FREE_DIRECT:
      case AP_MODE_FORWARD:
        guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
        break;
      case AP_MODE_RATE_RC_CLIMB:
      case AP_MODE_ATTITUDE_RC_CLIMB:
        guidance_v_mode_changed(GUIDANCE_V_MODE_RC_CLIMB);
        break;
      case AP_MODE_ATTITUDE_CLIMB:
      case AP_MODE_HOVER_CLIMB:
        guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
        break;
      case AP_MODE_RATE_Z_HOLD:
      case AP_MODE_ATTITUDE_Z_HOLD:
      case AP_MODE_HOVER_Z_HOLD:
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
        break;
      case AP_MODE_HOME:
      case AP_MODE_NAV:
        guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
        break;
      case AP_MODE_GUIDED:
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        break;
      default:
        break;
    }
    //if switching to rate mode but rate mode is not defined, the function returned
    autopilot.mode = new_autopilot_mode;
  }
}


void autopilot_static_set_motors_on(bool motors_on)
{
  if (autopilot.mode != AP_MODE_KILL && motors_on) {
    autopilot.motors_on = true;
  } else {
    autopilot.motors_on = false;
  }
  autopilot_arming_set(autopilot.motors_on);
}

void autopilot_static_on_rc_frame(void)
{

  if (kill_switch_is_on()) {
    autopilot_static_set_mode(AP_MODE_KILL);
  } else {
#ifdef RADIO_AUTO_MODE
    INFO("Using RADIO_AUTO_MODE to switch between AUTO1 and AUTO2.")
    uint8_t new_autopilot_mode = ap_mode_of_two_switches();
#else
#ifdef RADIO_MODE_2x3
    uint8_t new_autopilot_mode = ap_mode_of_3x2way_switch();
#else
    uint8_t new_autopilot_mode = ap_mode_of_3way_switch();
#endif
#endif

    /* don't enter NAV mode if GPS is lost (this also prevents mode oscillations) */
    if (!(new_autopilot_mode == AP_MODE_NAV && GpsIsLost())) {
      /* always allow to switch to manual */
      if (new_autopilot_mode == MODE_MANUAL) {
        autopilot_static_set_mode(new_autopilot_mode);
      }
      /* if in HOME or FAILSAFE mode, don't allow switching to non-manual modes */
      else if (((autopilot.mode != AP_MODE_HOME) && (autopilot.mode != AP_MODE_FAILSAFE))

#if UNLOCKED_HOME_MODE
               /* Allowed to leave home mode when UNLOCKED_HOME_MODE */
               || !nav.too_far_from_home
#endif
              ) {
        autopilot_static_set_mode(new_autopilot_mode);
      }
    }
  }

  /* an arming sequence is used to start/stop motors.
   */
  autopilot_arming_check_motors_on();
  autopilot.kill_throttle = ! autopilot.motors_on;

  /* if not in FAILSAFE or HOME mode, read RC and set commands accordingly */
  if (autopilot.mode != AP_MODE_FAILSAFE && autopilot.mode != AP_MODE_HOME) {

    /* if there are some commands that should always be set from RC, do it */
#ifdef SetAutoCommandsFromRC
    SetAutoCommandsFromRC(commands, radio_control.values);
#endif

    /* if not in NAV_MODE set commands from the rc */
#ifdef SetCommandsFromRC
    if (autopilot.mode != AP_MODE_NAV) {
      SetCommandsFromRC(commands, radio_control.values);
    }
#endif
  }

}

/** mode to enter when RC is lost while using a mode with RC input (not AP_MODE_NAV) */
#ifndef RC_LOST_MODE
#define RC_LOST_MODE AP_MODE_FAILSAFE
#endif

void autopilot_failsafe_checks(void)
{
  if (radio_control.status == RC_REALLY_LOST &&
      autopilot_get_mode() != AP_MODE_KILL &&
      autopilot_get_mode() != AP_MODE_HOME &&
      autopilot_get_mode() != AP_MODE_FAILSAFE &&
      autopilot_get_mode() != AP_MODE_NAV &&
      autopilot_get_mode() != AP_MODE_GUIDED) {
    autopilot_set_mode(RC_LOST_MODE);
  }

#if FAILSAFE_ON_BAT_CRITICAL
  if (autopilot_get_mode() != AP_MODE_KILL &&
      electrical.bat_critical) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

#if USE_GPS
  if (autopilot_get_mode() == AP_MODE_NAV &&
      autopilot_get_motors_on() &&
#if NO_GPS_LOST_WITH_RC_VALID
      radio_control.status != RC_OK &&
#endif
#ifdef NO_GPS_LOST_WITH_DATALINK_TIME
      datalink_time > NO_GPS_LOST_WITH_DATALINK_TIME &&
#endif
      GpsIsLost()) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }

  if (autopilot_get_mode() == AP_MODE_HOME &&
      autopilot_get_motors_on() && GpsIsLost()) {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif

}

