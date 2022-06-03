/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file peripherals/invensense1.h
 *
 * Driver for the Invensense V1 IMUs
 * MPU6000, MPU6500, MPU9250, MPU9255, ICM20608, ICM20602, ICM20601, ICM20789 and ICM20689
 */

#ifndef INVENSENSE1_H
#define INVENSENSE1_H

#include "std.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/i2c.h"

// Hold 22 measurements and 3 for the register address and length
#define INVENSENSE1_SAMPLE_CNT    22
#define INVENSENSE1_SAMPLE_SIZE   14
#define INVENSENSE1_BUFFER_SIZE   ((INVENSENSE1_SAMPLE_SIZE*INVENSENSE1_SAMPLE_CNT) + 3)

/* Invensense v1 SPI peripheral */
struct invensense1_spi_t {
  struct spi_periph *p;                             ///< Peripheral device for communication
  uint8_t slave_idx;                                ///< Slave index used for Slave Select
  struct spi_transaction trans;                     ///< Transaction used during configuration and measurements

  volatile uint8_t tx_buf[2];                       ///< Transmit buffer
  volatile uint8_t rx_buf[INVENSENSE1_BUFFER_SIZE]; ///< Receive buffer
};

/* Invensense v1 I2C peripheral */
struct invensense1_i2c_t {
  struct i2c_periph *p;           ///< Peripheral device for communication
  uint8_t slave_addr;             ///< The I2C slave address on the bus
  struct i2c_transaction trans;   ///< TRansaction used during configuration and measurements
};

/* Possible communication busses for the invense V1 driver */
enum invensense1_bus_t {
  INVENSENSE1_SPI,
  INVENSENSE1_I2C
};

/* Different states the invensense driver can be in */
enum invensense1_status_t {
  INVENSENSE1_IDLE,
  INVENSENSE1_CONFIG,
  INVENSENSE1_RUNNING
};

/* Different device types compatible with the invensense V1 driver */
enum invensense1_device_t {
  INVENSENSE1_UNKOWN,
  INVENSENSE1_MPU6000,
  INVENSENSE1_MPU6500,
  INVENSENSE1_MPU9250,
  INVENSENSE1_MPU9255,
  INVENSENSE1_ICM20608,
  INVENSENSE1_ICM20602,
  INVENSENSE1_ICM20601,
  INVENSENSE1_ICM20789,
  INVENSENSE1_ICM20689
};

/* The gyro digital low pass filter bandwidth configuration */
enum invensense21_gyro_dlpf_t {
  INVENSENSE1_GYRO_DLPF_256HZ,
  INVENSENSE1_GYRO_DLPF_188HZ,
  INVENSENSE1_GYRO_DLPF_98HZ,
  INVENSENSE1_GYRO_DLPF_42HZ,
  INVENSENSE1_GYRO_DLPF_20HZ,
  INVENSENSE1_GYRO_DLPF_10HZ,
  INVENSENSE1_GYRO_DLPF_5HZ,
  INVENSENSE1_GYRO_DLPF_OFF
};

/* The gyro range in degrees per second(dps) */
enum invensense1_gyro_range_t {
  INVENSENSE1_GYRO_RANGE_250DPS,
  INVENSENSE1_GYRO_RANGE_500DPS,
  INVENSENSE1_GYRO_RANGE_1000DPS,
  INVENSENSE1_GYRO_RANGE_2000DPS
};

/* The accelerometer digital low pass filter bandwidth configuration (only for ICM devices) */
enum invensense2_accel_dlpf_t {
  INVENSENSE1_ACCEL_DLPF_1046HZ,
  INVENSENSE1_ACCEL_DLPF_218HZ,
  INVENSENSE1_ACCEL_DLPF_99HZ,
  INVENSENSE1_ACCEL_DLPF_44HZ,
  INVENSENSE1_ACCEL_DLPF_21HZ,
  INVENSENSE1_ACCEL_DLPF_10HZ,
  INVENSENSE1_ACCEL_DLPF_5HZ,
  INVENSENSE1_ACCEL_DLPF_420HZ
};

  MPU60X0_DLPF_ACC_1046HZ = 0x0, // internal sampling rate 4kHz
  MPU60X0_DLPF_ACC_218HZ  = 0x1, // internal sampling rate 1kHz
  MPU60X0_DLPF_ACC_99HZ   = 0x2,
  MPU60X0_DLPF_ACC_44HZ   = 0x3,
  MPU60X0_DLPF_ACC_21HZ   = 0x4,
  MPU60X0_DLPF_ACC_10HZ   = 0x5,
  MPU60X0_DLPF_ACC_05HZ   = 0x6,
  MPU60X0_DLPF_ACC_420HZ  = 0x7

/* The accelerometer range in G */
enum invensense1_accel_range_t {
  INVENSENSE1_ACCEL_RANGE_2G,
  INVENSENSE1_ACCEL_RANGE_4G,
  INVENSENSE1_ACCEL_RANGE_8G,
  INVENSENSE1_ACCEL_RANGE_16G
};

/* Main invensense V1 device structure */
struct invensense1_t {
  uint8_t abi_id;                     ///< The ABI id used to broadcast the device measurements
  enum invensense1_status_t status;   ///< Status of the invensense V1 device
  enum invensense1_device_t device;   ///< The device type detected

  enum invensense1_bus_t bus;         ///< The communication bus used to connect the device SPI/I2C
  union {
    struct invensense1_spi_t spi;     ///< SPI specific configuration
    struct invensense1_i2c_t i2c;     ///< I2C specific configuration
  };

  uint8_t config_idx;                 ///< The current configuration index
  uint32_t timer;                     ///< Used to time operations during configuration (samples left during measuring)

  enum invensense1_gyro_dlpf_t gyro_dlpf;       ///< Gyro DLPF configuration
  enum invensense1_gyro_range_t gyro_range;     ///< Gyro range configuration
  enum invensense1_accel_dlpf_t accel_dlpf;     ///< Accelerometer DLPF configuration
  enum invensense1_accel_range_t accel_range;   ///< Accelerometer range configuration
};

/* External functions */
void invensense1_init(struct invensense1_t *inv);
void invensense1_periodic(struct invensense1_t *inv);
void invensense1_event(struct invensense1_t *inv);

#endif // INVENSENSE1_H
