/*
 * $Id$
 *
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#include "subsystems/imu.h"

struct Imu imu;

void imu_init(void) {

  /* initialises neutrals */
  RATES_ASSIGN(imu.gyro_neutral,  IMU_GYRO_P_NEUTRAL,  IMU_GYRO_Q_NEUTRAL,  IMU_GYRO_R_NEUTRAL);
  VECT3_ASSIGN(imu.accel_neutral, IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL);
  //FIXME should not assume that every imu has a mag and this id defined?
  VECT3_ASSIGN(imu.mag_neutral,   IMU_MAG_X_NEUTRAL,   IMU_MAG_Y_NEUTRAL,   IMU_MAG_Z_NEUTRAL);

  /*
    Compute quaternion and rotation matrix
    for conversions between body and imu frame
  */
#if defined IMU_BODY_TO_IMU_PHI && defined IMU_BODY_TO_IMU_THETA & defined  IMU_BODY_TO_IMU_PSI
  struct Int32Eulers body_to_imu_eulers =
    { ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_PHI),
      ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_THETA),
      ANGLE_BFP_OF_REAL(IMU_BODY_TO_IMU_PSI) };
  INT32_QUAT_OF_EULERS(imu.body_to_imu_quat, body_to_imu_eulers);
  INT32_QUAT_NORMALISE(imu.body_to_imu_quat);
  INT32_RMAT_OF_EULERS(imu.body_to_imu_rmat, body_to_imu_eulers);
#else
  INT32_QUAT_ZERO(imu.body_to_imu_quat);
  INT32_RMAT_ZERO(imu.body_to_imu_rmat);
#endif

  imu_impl_init();
}


void imu_float_init(struct ImuFloat* imuf) {

  /*
    Compute quaternion and rotation matrix
    for conversions between body and imu frame
  */
#if defined IMU_BODY_TO_IMU_PHI && defined IMU_BODY_TO_IMU_THETA & defined  IMU_BODY_TO_IMU_PSI
  EULERS_ASSIGN(imuf->body_to_imu_eulers,
		IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI);
  FLOAT_QUAT_OF_EULERS(imuf->body_to_imu_quat, imuf->body_to_imu_eulers);
  FLOAT_QUAT_NORMALISE(imuf->body_to_imu_quat);
  FLOAT_RMAT_OF_EULERS(imuf->body_to_imu_rmat, imuf->body_to_imu_eulers);
#else
  EULERS_ASSIGN(imuf->body_to_imu_eulers, 0., 0., 0.);
  FLOAT_QUAT_ZERO(imuf->body_to_imu_quat);
  FLOAT_RMAT_ZERO(imuf->body_to_imu_rmat);
#endif

}
