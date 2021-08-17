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

/** @file "modules/raw_sensors/raw_imu_gyro.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module reads the imu gyro angular acceleration and converts it to the body reference frame. To be used for direct feedback by INDI.
 */

#ifndef RAW_IMU_GYRO_H
#define RAW_IMU_GYRO_H

#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

#ifndef RAW_IMU_GYRO_BODY_ENABLED
#define RAW_IMU_GYRO_BODY_ENABLED TRUE
#endif


struct imu_gyro_body_t
{
  struct FloatRates body_rate;
  struct OrientationReps body_to_imu;
};

extern struct imu_gyro_body_t imu_gyro_body;

extern void init_raw_imu_gyro(void);

static inline struct FloatRates *raw_imu_gyro_GetBodyRates_f(void)
{
  return &imu_gyro_body.body_rate;
}

#endif  // RAW_IMU_GYRO_H
