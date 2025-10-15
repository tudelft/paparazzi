/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/imu/imu_parrot_minidrone.c
 * Driver for the Parrot minidrone accelerometer and gyroscope
 */

#include "modules/imu/imu_parrot_minidrone.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/i2c.h"


/* defaults suitable for Parrot minidrone */
#ifndef PARROT_MINIDRONE_MPU_I2C_DEV
#define PARROT_MINIDRONE_MPU_I2C_DEV i2c0
#endif
PRINT_CONFIG_VAR(PARROT_MINIDRONE_MPU_I2C_DEV)

#if !defined PARROT_MINIDRONE_LOWPASS_FILTER && !defined  PARROT_MINIDRONE_SMPLRT_DIV
#if (PERIODIC_FREQUENCY >= 60) && (PERIODIC_FREQUENCY <= 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define PARROT_MINIDRONE_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define PARROT_MINIDRONE_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif (PERIODIC_FREQUENCY == 512) || (PERIODIC_FREQUENCY == 500)
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define PARROT_MINIDRONE_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define PARROT_MINIDRONE_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(PARROT_MINIDRONE_SMPLRT_DIV)
PRINT_CONFIG_VAR(PARROT_MINIDRONE_LOWPASS_FILTER)

PRINT_CONFIG_VAR(PARROT_MINIDRONE_GYRO_RANGE)
PRINT_CONFIG_VAR(PARROT_MINIDRONE_ACCEL_RANGE)

/** Basic IMU data */
struct ImuParrotMinidrone imu_parrot_minidrone;

/**
 * IMU initializtion of the MPU
 */
void imu_parrot_minidrone_init(void)
{
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_parrot_minidrone.mpu, &(PARROT_MINIDRONE_MPU_I2C_DEV), MPU60X0_ADDR);
  imu_parrot_minidrone.mpu.config.smplrt_div = PARROT_MINIDRONE_SMPLRT_DIV;
  imu_parrot_minidrone.mpu.config.dlpf_cfg = PARROT_MINIDRONE_LOWPASS_FILTER;
  imu_parrot_minidrone.mpu.config.gyro_range = PARROT_MINIDRONE_GYRO_RANGE;
  imu_parrot_minidrone.mpu.config.accel_range = PARROT_MINIDRONE_ACCEL_RANGE;

    // Set the default scaling
  imu_set_defaults_gyro(IMU_BOARD_ID, NULL, NULL, &MPU60X0_GYRO_SENS_F[PARROT_MINIDRONE_GYRO_RANGE]);
  imu_set_defaults_accel(IMU_BOARD_ID, NULL, NULL, &MPU60X0_ACCEL_SENS_F[PARROT_MINIDRONE_ACCEL_RANGE]);

}

/**
 * Handle all the periodic tasks of the IMU components.
 * Read the MPU60x0 every periodic call
 */
void imu_parrot_minidrone_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_parrot_minidrone.mpu);
}

/**
 * Handle all the events of the IMU components.
 * When there is data available convert it to the correct axis and save it in the imu structure.
 */
void imu_parrot_minidrone_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* MPU-60x0 event tasks */
  mpu60x0_i2c_event(&imu_parrot_minidrone.mpu);

  if (imu_parrot_minidrone.mpu.data_available) {
    /* set correct orientation here */
    struct Int32Rates gyro;
    struct Int32Vect3 accel;

#if defined(IS_SWING)
    RATES_ASSIGN(gyro,
        -imu_parrot_minidrone.mpu.data_rates.rates.r,
        -imu_parrot_minidrone.mpu.data_rates.rates.q,
        -imu_parrot_minidrone.mpu.data_rates.rates.p);
    VECT3_ASSIGN(accel,
        -imu_parrot_minidrone.mpu.data_accel.vect.z,
        -imu_parrot_minidrone.mpu.data_accel.vect.y,
        -imu_parrot_minidrone.mpu.data_accel.vect.x);
#else
    RATES_ASSIGN(gyro,
        imu_parrot_minidrone.mpu.data_rates.rates.p,
        -imu_parrot_minidrone.mpu.data_rates.rates.q,
        -imu_parrot_minidrone.mpu.data_rates.rates.r);
    VECT3_ASSIGN(accel,
        imu_parrot_minidrone.mpu.data_accel.vect.x,
        -imu_parrot_minidrone.mpu.data_accel.vect.y,
        -imu_parrot_minidrone.mpu.data_accel.vect.z);
#endif

    imu_parrot_minidrone.mpu.data_available = false;
    AbiSendMsgIMU_GYRO_RAW(IMU_BOARD_ID, now_ts, &gyro, 1, IMU_SWING_PERIODIC_FREQ, imu_parrot_minidrone.mpu.temp);
    AbiSendMsgIMU_ACCEL_RAW(IMU_BOARD_ID, now_ts, &accel, 1, IMU_SWING_PERIODIC_FREQ, imu_parrot_minidrone.mpu.temp);
  }
}
