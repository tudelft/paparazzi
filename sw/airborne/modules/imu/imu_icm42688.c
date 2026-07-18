/*
 * Copyright (C) 2022 Freek van tieen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu_icm42688.c
 * Driver module for the Invensense v3 IMU ICM42688.
 */

#include "modules/imu/imu_icm42688.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#include "peripherals/invensense3.h"


#ifndef INVENSENSE3_GYRO_AAF
#define INVENSENSE3_GYRO_AAF 977
#endif

#ifndef INVENSENSE3_ACCEL_AAF
#define INVENSENSE3_ACCEL_AAF 213
#endif

#ifndef IMU_ICM42688_CHAN_X
#define IMU_ICM42688_CHAN_X 0
#endif
#ifndef IMU_ICM42688_CHAN_Y
#define IMU_ICM42688_CHAN_Y 1
#endif
#ifndef IMU_ICM42688_CHAN_Z
#define IMU_ICM42688_CHAN_Z 2
#endif
#ifndef IMU_ICM42688_X_SIGN
#define IMU_ICM42688_X_SIGN 1
#endif
#ifndef IMU_ICM42688_Y_SIGN
#define IMU_ICM42688_Y_SIGN 1
#endif
#ifndef IMU_ICM42688_Z_SIGN
#define IMU_ICM42688_Z_SIGN 1
#endif

/* Accept +, -, +1 and -1 as axis signs and normalize them to +/-1. */
#define IMU_ICM42688_SIGN_VALUE(sign) (((sign + 1) > 0) ? 1 : -1)

#if IMU_ICM42688_CHAN_X < 0 || IMU_ICM42688_CHAN_X > 2 || \
  IMU_ICM42688_CHAN_Y < 0 || IMU_ICM42688_CHAN_Y > 2 || \
  IMU_ICM42688_CHAN_Z < 0 || IMU_ICM42688_CHAN_Z > 2
#error "IMU_ICM42688_CHAN_X/Y/Z must be 0, 1 or 2"
#endif

#if IMU_ICM42688_CHAN_X == IMU_ICM42688_CHAN_Y || \
  IMU_ICM42688_CHAN_X == IMU_ICM42688_CHAN_Z || \
  IMU_ICM42688_CHAN_Y == IMU_ICM42688_CHAN_Z
#error "IMU_ICM42688_CHAN_X/Y/Z must form a permutation of 0, 1 and 2"
#endif

static const struct Int32RMat imu_icm42688_to_sensor = {
  .m = {
    [IMU_ICM42688_CHAN_X * 3] = IMU_ICM42688_SIGN_VALUE(IMU_ICM42688_X_SIGN) * (1 << INT32_TRIG_FRAC),
    [IMU_ICM42688_CHAN_Y * 3 + 1] = IMU_ICM42688_SIGN_VALUE(IMU_ICM42688_Y_SIGN) * (1 << INT32_TRIG_FRAC),
    [IMU_ICM42688_CHAN_Z * 3 + 2] = IMU_ICM42688_SIGN_VALUE(IMU_ICM42688_Z_SIGN) * (1 << INT32_TRIG_FRAC)
  }
};

static struct invensense3_t imu_icm42688;

void imu_icm42688_init(void)
{
  imu_icm42688.abi_id = IMU_ICM42688_ID;
  //imu_icm42688.parser = INVENSENSE3_PARSER_REGISTERS;
  imu_icm42688.parser = INVENSENSE3_PARSER_FIFO;

  imu_icm42688.bus = INVENSENSE3_SPI;
  imu_icm42688.spi.p = &IMU_SPI_DEV;
  imu_icm42688.spi.slave_idx = IMU_SPI_SLAVE_IDX;

  imu_icm42688.gyro_odr = INVENSENSE3_GYRO_ODR;
  imu_icm42688.gyro_range = INVENSENSE3_GYRO_RANGE;
  imu_icm42688.accel_odr = INVENSENSE3_ACCEL_ODR;
  imu_icm42688.accel_range = INVENSENSE3_ACCEL_RANGE;
    
  // The AAF freq needs to be high enough to avoid group delay and low enough to minimise noise and clipping
  imu_icm42688.gyro_aaf = INVENSENSE3_GYRO_AAF; // ~ODR/4
  imu_icm42688.accel_aaf = INVENSENSE3_ACCEL_AAF; // Fixed
  
  invensense3_init(&imu_icm42688);
  imu_set_defaults_gyro(IMU_ICM42688_ID, &imu_icm42688_to_sensor, NULL, NULL);
  imu_set_defaults_accel(IMU_ICM42688_ID, &imu_icm42688_to_sensor, NULL, NULL);
}

void imu_icm42688_periodic(void)
{
  invensense3_periodic(&imu_icm42688);
}

void imu_icm42688_event(void)
{
  invensense3_event(&imu_icm42688);
}



