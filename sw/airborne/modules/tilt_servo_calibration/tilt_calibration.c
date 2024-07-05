/*
 * Copyright (C) 2024 Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
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
 * @file "modules/tilt_servo_calibration/tilt_calibration.c"
 * @author Alessandro Mancinelli <alessandro.mancinelli@outlook.com>
 * Calibration of the dual-axis tilting rotor quad-plane servos
 */

#include "tilt_calibration.h"
#include "modules/datalink/telemetry.h"
#include "modules/core/abi.h"
#include "modules/sensors/serial_act_t4.h"
#include <stdio.h>

static abi_event SERIAL_ACT_T4_IN;
static struct serial_act_t4_in myserial_act_t4_in_local;

/** Maximum combined message size for storing the errors */
#ifndef TILT_CALIBRATION_MAX_MSGBUF
#define TILT_CALIBRATION_MAX_MSGBUF 1024
#endif

bool rotor_1_calibrate = false;
bool rotor_2_calibrate = false;
bool rotor_3_calibrate = false;
bool rotor_4_calibrate = false;

/**
 * @brief ABI routine called by the serial_act_t4 ABI event
 */
static void serial_act_t4_abi_in(uint8_t sender_id __attribute__((unused)), struct serial_act_t4_in * myserial_act_t4_in_ptr, float * serial_act_t4_extra_data_in_ptr){
    memcpy(&myserial_act_t4_in_local,myserial_act_t4_in_ptr,sizeof(struct serial_act_t4_in));
    serial_act_t4_extra_data_in_ptr = serial_act_t4_extra_data_in_ptr;
}


/**
 * @brief Initialize the tilt calibration module by binding the ABI event to the serial actuator module
 */
void tilt_calibration_init(void) {
    //Init abi bind msg to Teensy 4.0:
    AbiBindMsgSERIAL_ACT_T4_IN(ABI_BROADCAST, &SERIAL_ACT_T4_IN, serial_act_t4_abi_in);
}

/**
 * @brief Runs the calibration routine waiting for user input to calibrate rotors
 */
void tilt_calibration_run(void){

  if(rotor_1_calibrate){
    char tilt_calib_msg[TILT_CALIBRATION_MAX_MSGBUF];
    float az_zero_value_rotor_1 = -(myserial_act_t4_in_local.servo_1_angle_int/100.0);
    float el_zero_value_rotor_1 = -(myserial_act_t4_in_local.servo_2_angle_int/100.0);
    int rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_EL_1_ZERO_VALUE = %f deg", el_zero_value_rotor_1); 
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_AZ_1_ZERO_VALUE = %f deg", az_zero_value_rotor_1);
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rotor_1_calibrate = false;
  }
  if(rotor_2_calibrate){
    char tilt_calib_msg[TILT_CALIBRATION_MAX_MSGBUF];
    float az_zero_value_rotor_2 = -(myserial_act_t4_in_local.servo_5_angle_int/100.0);
    float el_zero_value_rotor_2 = -(myserial_act_t4_in_local.servo_6_angle_int/100.0);
    int rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_EL_2_ZERO_VALUE = %f deg", el_zero_value_rotor_2); 
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_AZ_2_ZERO_VALUE = %f deg", az_zero_value_rotor_2);
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rotor_2_calibrate = false;
  }

  if(rotor_3_calibrate){
    char tilt_calib_msg[TILT_CALIBRATION_MAX_MSGBUF];
    float az_zero_value_rotor_3 = (myserial_act_t4_in_local.servo_7_angle_int/100.0);
    float el_zero_value_rotor_3 = -(myserial_act_t4_in_local.servo_8_angle_int/100.0);
    int rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_EL_3_ZERO_VALUE = %f deg", el_zero_value_rotor_3); 
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_AZ_3_ZERO_VALUE = %f deg", az_zero_value_rotor_3);
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rotor_3_calibrate = false;
  }

  if(rotor_4_calibrate){
    char tilt_calib_msg[TILT_CALIBRATION_MAX_MSGBUF];
    float az_zero_value_rotor_4 = (myserial_act_t4_in_local.servo_3_angle_int/100.0);
    float el_zero_value_rotor_4 = -(myserial_act_t4_in_local.servo_4_angle_int/100.0);
    int rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_EL_4_ZERO_VALUE = %f deg", el_zero_value_rotor_4); 
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rc = snprintf(tilt_calib_msg, TILT_CALIBRATION_MAX_MSGBUF, "SERVO_AZ_4_ZERO_VALUE = %f deg", az_zero_value_rotor_4);
    if (rc > 0) {
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, rc, tilt_calib_msg);
    }
    rotor_4_calibrate = false;
  }

}

