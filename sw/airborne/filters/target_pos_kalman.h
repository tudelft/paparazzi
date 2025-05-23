/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "filters/target_pos_kalman.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * @author Antoine Leclerc, Nathan Puch, Pauline Molitor
 *
 * Adapted simple kinematic kalman to work with target pos module.
 */

#ifndef TARGET_POS_KALMAN_H
#define TARGET_POS_KALMAN_H

#include "std.h"
#include "math/pprz_algebra_float.h"

#ifndef TARGET_POS_KALMAN_DIM
#define TARGET_POS_KALMAN_DIM 6
#endif

struct KalmanSensor {
    float noise[TARGET_POS_KALMAN_DIM];            ///< The noise of the measurement, fill in order of Hmat, don't follow state vect.
    float meas[TARGET_POS_KALMAN_DIM];             ///< The measurement
    uint8_t Hmat[TARGET_POS_KALMAN_DIM];             ///< The measurement matrix, fill non-zero rows first as it is trimmed.
    uint8_t n_meas;                                ///< The number of measurements, used to trim Hmat
  };

/** Kalman structure
 *
 * state vector: X = [ x xd y yd z zd ]'
 * command vector: U = 0 (constant velocity model)
 * dynamic model: basic kinematic model x_k+1 = x_k + xd_k * dt
 * measures: distance between (fixed and known) anchors and UAV
 *
 * */
struct TargetPosKalman {
  float state[TARGET_POS_KALMAN_DIM];                         ///< state vector
  float P[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];      ///< covariance matrix
  float Q[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];      ///< process noise matrix
  float F[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];      ///< dynamic matrix
  float dt;                                                   ///< prediction step (in seconds)
};

/** Init TargetPosKalman internal struct
 *
 * @param[in] kalman TargetPosKalman structure
 * @param[in] P0 initial covariance on position
 * @param[in] Q0 process noise
 * @param[in] dt prediction time step in seconds
 */
extern void target_pos_kalman_init(struct TargetPosKalman *kalman, float *P0, float *Q_sigma2, float dt);

/** Set initial state vector
 *
 * This function should be called after initialization of the kalman struct and before
 * running the filter for better results and faster convergence
 *
 * @param[in] kalman TargetPosKalman structure
 * @param[in] pos initial position
 * @param[in] speed initial speed
 */
extern void target_pos_kalman_set_state(struct TargetPosKalman *kalman, struct FloatVect3 pos,
    struct FloatVect3 speed);

/** Get current state
 *
 * @param[in] kalman TargetPosKalman structure
 * @param[out] pos current position
 * @param[out] speed current speed
 */
extern void target_pos_kalman_get_state(struct TargetPosKalman *kalman, struct FloatVect3 *pos,
    struct FloatVect3 *speed);

/** Get current pos
 *
 * @param[in] kalman TargetPosKalman structure
 * @return current position
 */
extern struct FloatVect3 target_pos_kalman_get_pos(struct TargetPosKalman *kalman);

/** Get current speed
 *
 * @param[in] kalman TargetPosKalman structure
 * @return current speed
 */
extern struct FloatVect3 target_pos_kalman_get_speed(struct TargetPosKalman *kalman);

/** Prediction step
 *
 * @param[in] kalman TargetPosKalman structure
 */
extern void target_pos_kalman_predict(struct TargetPosKalman *kalman);

/** Update step based on new sensor data
 *
 * @param[in] kalman TargetPosKalman structure
 * @param[in] sensor KalmanSensor structure
 */
extern void target_pos_kalman_update(struct TargetPosKalman *kalman, struct KalmanSensor *sensor);

/** Update a Kalman sensor struct with new measurements
 *
 * @param[in] sensor KalmanSensor structure
 * @param[in] meas new measurements
 */
void target_pos_kalman_set_measurement(struct KalmanSensor *sensor, float *meas);

/** Update a Kalman sensor struct with a new measurement noise estimate
 *
 * @param[in] sensor KalmanSensor structure
 * @param[in] noise new measurement noise estimate
 */
void target_pos_kalman_set_noise(struct KalmanSensor *sensor, float *noise);

#endif
