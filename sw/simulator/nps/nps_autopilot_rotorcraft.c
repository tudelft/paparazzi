/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include "nps_autopilot.h"

#include "main_ap.h"
#include "nps_sensors.h"
#include "nps_radio_control.h"
#include "nps_electrical.h"
#include "nps_fdm.h"

#include "modules/radio_control/radio_control.h"
#include "modules/imu/imu.h"
#include "mcu_periph/sys_time.h"
#include "state.h"
#include "modules/ahrs/ahrs.h"
#include "modules/ins/ins.h"
#include "math/pprz_algebra.h"

#ifndef NPS_NO_MOTOR_MIXING
#include "modules/actuators/motor_mixing.h"

#if NPS_COMMANDS_NB != MOTOR_MIXING_NB_MOTOR
#warning "NPS_COMMANDS_NB does not match MOTOR_MIXING_NB_MOTOR!"
#endif
#endif

#include "modules/core/abi.h"

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

// for datalink_time hack
#include "modules/datalink/datalink.h"
#include "modules/actuators/actuators.h"

// TODO: arrange this the other way around - fix the INS in the sim
// for experiments in the sim:
#include "subsystems/ins/ins_flow.h"

struct NpsAutopilot nps_autopilot;
bool nps_bypass_ahrs;
bool nps_bypass_ins;

#ifndef NPS_BYPASS_AHRS
#define NPS_BYPASS_AHRS FALSE
#endif

#ifndef NPS_BYPASS_INS
#define NPS_BYPASS_INS FALSE
#endif

#if INDI_RPM_FEEDBACK
#error "INDI_RPM_FEEDBACK can not be used in simulation!"
#endif

void nps_autopilot_init(enum NpsRadioControlType type_rc, int num_rc_script, char *rc_dev)
{
  nps_autopilot.launch = TRUE;

  nps_radio_control_init(type_rc, num_rc_script, rc_dev);
  nps_electrical_init();

  nps_bypass_ahrs = NPS_BYPASS_AHRS;
  nps_bypass_ins = NPS_BYPASS_INS;

  modules_mcu_init();
  main_ap_init();
}

void nps_autopilot_run_systime_step(void)
{
  sys_tick_handler();
}

#include <stdio.h>
#include "modules/gps/gps.h"

void nps_autopilot_run_step(double time)
{

  nps_electrical_run_step(time);

#if RADIO_CONTROL && !RADIO_CONTROL_TYPE_DATALINK
  if (nps_radio_control_available(time)) {
    radio_control_feed();
    main_ap_event();
  }
#endif

  if (nps_sensors_gyro_available()) {
    imu_feed_gyro_accel();
    main_ap_event();
  }

  if (nps_sensors_mag_available()) {
    imu_feed_mag();
    main_ap_event();
  }

  if (nps_sensors_baro_available()) {
    uint32_t now_ts = get_sys_time_usec();
    float pressure = (float) sensors.baro.value;
    AbiSendMsgBARO_ABS(BARO_SIM_SENDER_ID, now_ts, pressure);
    main_ap_event();
  }

  if (nps_sensors_temperature_available()) {
    AbiSendMsgTEMPERATURE(BARO_SIM_SENDER_ID, (float)sensors.temp.value);
    main_ap_event();
  }

#if USE_AIRSPEED
  if (nps_sensors_airspeed_available()) {
    stateSetAirspeed_f((float)sensors.airspeed.value);
    AbiSendMsgAIRSPEED(AIRSPEED_NPS_ID, (float)sensors.airspeed.value);
    main_ap_event();
  }
#endif

#if USE_SONAR
  if (nps_sensors_sonar_available()) {
    uint32_t now_ts = get_sys_time_usec();
    float dist = (float) sensors.sonar.value;
    if (dist >= 0.0) {
      AbiSendMsgAGL(AGL_SONAR_NPS_ID, now_ts, dist);
    }

#ifdef SENSOR_SYNC_SEND_SONAR
    uint16_t foo = 0;
    DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &foo, &dist);
#endif

    main_ap_event();
  }
#endif

#if USE_GPS
  if (nps_sensors_gps_available()) {
    gps_feed_value();
    main_ap_event();
  }
#endif

  if (nps_bypass_ahrs) {
  //    printf("bypass ahrs!\n");
      sim_overwrite_ahrs();
  }

  if (nps_bypass_ins) {
  //    printf("bypass ins!\n");
      sim_overwrite_ins();
  }

  main_ap_periodic();

  /* scale final motor commands to 0-1 for feeding the fdm */
  for (uint8_t i = 0; i < NPS_COMMANDS_NB; i++) {
#if NPS_NO_MOTOR_MIXING
    actuators_pprz[i] = autopilot_get_motors_on() ? actuators_pprz[i] : 0;
    nps_autopilot.commands[i] = (double)actuators_pprz[i] / MAX_PPRZ;
#else
    nps_autopilot.commands[i] = (double)motor_mixing.commands[i] / MAX_PPRZ;
#endif
  }
}


void sim_overwrite_ahrs(void)
{

  struct FloatQuat quat_f;
  QUAT_COPY(quat_f, fdm.ltp_to_body_quat);
  //stateSetNedToBodyQuat_f(&quat_f);

  //printf("SIM 0: qi = %f, qx = %f, qy = %f, qz = %f.\n", quat_f.qi, quat_f.qx, quat_f.qy, quat_f.qz);
  struct OrientationReps orient;
  // SetBit(orient.status, ORREP_QUAT_F);
  orient.status = 1 << ORREP_QUAT_F;
  orient.quat_f = quat_f;
  struct FloatEulers* eulers = orientationGetEulers_f(&orient);
  //printf("SIM 1: phi = %f, theta = %f, psi = %f.\n", (180.0f/M_PI)*eulers->phi, (180.0f/M_PI)*eulers->theta, (180.0f/M_PI)*eulers->psi);

  GT_phi = eulers->phi;
  if(use_filter) {
    // struct FloatEulers* eulers = stateGetNedToBodyEulers_f();
    // set part of the state with the filter:
    eulers->phi = OF_X[OF_ANGLE_IND];
    // printf("Set Euler roll angle to %f\n", eulers->phi);
  }
  //printf("SIM 2: phi = %f, theta = %f, psi = %f.\n", (180.0f/M_PI)*eulers->phi, (180.0f/M_PI)*eulers->theta, (180.0f/M_PI)*eulers->psi);

  // stateSetNedToBodyEulers_f(eulers);

  struct OrientationReps orient_euler;
  orient_euler.status = 1 << ORREP_EULER_F;
  orient_euler.eulers_f = (*eulers);
  struct FloatQuat* quat_f_adapted = orientationGetQuat_f(&orient_euler);

  //printf("SIM 4: qi = %f, qx = %f, qy = %f, qz = %f.\n", quat_f_adapted->qi, quat_f_adapted->qx, quat_f_adapted->qy, quat_f_adapted->qz);
  stateSetNedToBodyQuat_f(quat_f_adapted);


  struct FloatRates rates_f;
  RATES_COPY(rates_f, fdm.body_ecef_rotvel);
  stateSetBodyRates_f(&rates_f);

}

void sim_overwrite_ins(void)
{

  struct NedCoor_f ltp_pos;
  VECT3_COPY(ltp_pos, fdm.ltpprz_pos);
  if(use_filter >= USE_HEIGHT) {
      printf("Z true: %f, ", ltp_pos.z);
      // replace the z-coordinate:
      ltp_pos.z = -OF_X[OF_Z_IND];
      printf("Z filter: %f.\n", ltp_pos.z);
  }
  stateSetPositionNed_f(&ltp_pos);

  struct NedCoor_f ltp_speed;
  VECT3_COPY(ltp_speed, fdm.ltpprz_ecef_vel);

  if(use_filter >= USE_VELOCITY) {

    // get NED to body rotation matrix:
    struct FloatRMat* NTB = stateGetNedToBodyRMat_f();
    // get transpose (inverse):
    struct FloatRMat BTN;
    float_rmat_inv(&BTN, NTB);

    // the velocities from the filter are rotated from the body to the inertial frame:
    struct FloatVect3 NED_velocities, body_velocities;
    body_velocities.x = 0.0f; // filter does not determine this yet
    body_velocities.y = OF_X[OF_V_IND];
    if(CONSTANT_ALT_FILTER) {
	body_velocities.z = 0.0f;
    }
    else {
	body_velocities.z = -OF_X[OF_Z_DOT_IND];
    }
    float_rmat_vmult(&NED_velocities, &BTN, &body_velocities);
    // TODO: also estimate vx, so that we can just use the rotated vector:
    // For now, we need to keep the x, and y body axes aligned with the global ones.
    //printf("Original speed y = %f, ", ltp_speed.y);
    ltp_speed.y = NED_velocities.y;
    if(!CONSTANT_ALT_FILTER) ltp_speed.z =  NED_velocities.z;
    //printf("Changed speed y = %f\n", ltp_speed.y);

  }
  stateSetSpeedNed_f(&ltp_speed);

  struct NedCoor_f ltp_accel;
  VECT3_COPY(ltp_accel, fdm.ltpprz_ecef_accel);
  stateSetAccelNed_f(&ltp_accel);

}
