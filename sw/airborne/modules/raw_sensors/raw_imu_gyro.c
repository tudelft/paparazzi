/*
 * Copyright (C) Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/raw_sensors/raw_imu_gyro.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module reads the imu gyro angular acceleration and converts it to the body reference frame. To be used for direct feedback by INDI.
 */

#include "modules/raw_sensors/raw_imu_gyro.h"

#include "subsystems/abi.h"

#include "std.h"

/* All registered ABI events */
static abi_event gyro_ev;
static abi_event body_to_imu_ev;

/* All ABI callbacks */
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);

struct imu_gyro_body_t imu_gyro_body;

void init_raw_imu_gyro(void)
{
  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &gyro_ev, gyro_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);

  /* 
   * Initialize imu_gyro_body_struct 
   */
  imu_gyro_body.body_rate.p = 0;
  imu_gyro_body.body_rate.q = 0;
  imu_gyro_body.body_rate.r = 0;
}

static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t __attribute__((unused)) stamp, struct Int32Rates *gyro)
{
  struct FloatRates gyro_rate;
  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&imu_gyro_body.body_to_imu);

  // Convert Gyro information to float
  RATES_FLOAT_OF_BFP(gyro_rate, *gyro);

  // Rotate with respect to Body To IMU
  float_rmat_transp_ratemult(&imu_gyro_body.body_rate, body_to_imu_rmat, &gyro_rate);
}

/* Save the Body to IMU information */
static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  orientationSetQuat_f(&imu_gyro_body.body_to_imu, q_b2i_f);
}
