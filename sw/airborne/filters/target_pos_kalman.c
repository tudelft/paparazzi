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

/** @file "filters/target_pos_kalman.c"
 * @author Noah Wechtler
 *
 * Adapted basic kinematic kalman filter for target pos module
 */

#include "filters/target_pos_kalman.h"
#include "filters/target_pos_kalman.h"
#include <math.h>

#ifndef TARGET_POS_KALMAN_DEBUG
#define TARGET_POS_KALMAN_DEBUG FALSE
#endif

#if TARGET_POS_KALMAN_DEBUG
#include "modules/datalink/telemetry.h"
#include "modules/datalink/downlink.h"
#include "state.h"
#include <stdio.h>
#endif

PRINT_CONFIG_VAR(TARGET_POS_KALMAN_DEBUG);

void target_pos_kalman_init(struct TargetPosKalman *kalman, float *P0, float *Q_sigma2, float dt)
{
  int i, j;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt / 2.f;
  const float dt4 = dt2 * dt2 / 4.f;
  for (i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    kalman->state[i] = 0.f; // don't forget to call set_state before running the filter for better results
    for (j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      kalman->P[i][j] = 0.f;
      kalman->Q[i][j] = 0.f;
    }
  }
  for (i = 0; i < TARGET_POS_KALMAN_DIM; i += 2) {
    kalman->P[i][i] = P0[i];
    kalman->P[i + 1][i + 1] = P0[i + 1];
    kalman->Q[i][i] = Q_sigma2[i] * dt4;
    kalman->Q[i + 1][i] = Q_sigma2[i] * dt3;
    kalman->Q[i][i + 1] = Q_sigma2[i] * dt3;
    kalman->Q[i + 1][i + 1] = Q_sigma2[i + 1] * dt2;
  }
  kalman->dt = dt;

  // kalman->F = {{1, dt, 0, 0 , 0, 0},
  //              {0, 1 , 0, 0,  0, 0},
  //              {0, 0,  1, dt 0, 0},
  //              {0, 0,  0, 1  0, 0},
  //              {0, 0,  0, 0,  1 ,dt}
  //              {0, 0,  0, 0,  0, 1 }};

  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      if (i == j) {
        kalman->F[i][i] = 1;
      } else if (i % 2 == 0 && j == i + 1) {
        kalman->F[i][j] = dt;
      } else {
        kalman->F[i][j] = 0;
      }
    }
  }
}

void target_pos_kalman_set_state(struct TargetPosKalman *kalman, struct FloatVect3 pos,
                                       struct FloatVect3 speed)
{
  kalman->state[0] = pos.x;
  kalman->state[1] = speed.x;
  kalman->state[2] = pos.y;
  kalman->state[3] = speed.y;
  kalman->state[4] = pos.z;
  kalman->state[5] = speed.z;
}

void target_pos_kalman_get_state(struct TargetPosKalman *kalman, struct FloatVect3 *pos,
                                       struct FloatVect3 *speed)
{
  pos->x = kalman->state[0];
  pos->y = kalman->state[2];
  pos->z = kalman->state[4];
  speed->x = kalman->state[1];
  speed->y = kalman->state[3];
  speed->z = kalman->state[5];
}

struct FloatVect3 target_pos_kalman_get_pos(struct TargetPosKalman *kalman)
{
  struct FloatVect3 pos;
  pos.x = kalman->state[0];
  pos.y = kalman->state[2];
  pos.z = kalman->state[4];
  return pos;
}

struct FloatVect3 target_pos_kalman_get_speed(struct TargetPosKalman *kalman)
{
  struct FloatVect3 speed;
  speed.x = kalman->state[1];
  speed.y = kalman->state[3];
  speed.z = kalman->state[5];
  return speed;
}

/** propagate dynamic model
 *
 * F = [ 1 dt 0 0  0 0
 *       0 1  0 0  0 0
 *       0 0  1 dt 0 0
 *       0 0  0 1  0 0
 *       0 0  0 0  1 dt
 *       0 0  0 0  0 1  ]
 */
void target_pos_kalman_predict(struct TargetPosKalman *kalman)
{
  int i;
#if TARGET_POS_KALMAN_DEBUG
  float state_in[TARGET_POS_KALMAN_DIM];
  for (i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    state_in[i] = kalman->state[i];
  }
#endif

  for (i = 0; i < TARGET_POS_KALMAN_DIM; i += 2) {
    // kinematic equation of the dynamic model X = F*X
    kalman->state[i] += kalman->state[i + 1] * kalman->dt;

    // propagate covariance P = F*P*Ft + Q
    // since F is diagonal by block, P can be updated by block here as well
    // let's unroll the matrix operations as it is simple

    const float d_dt = kalman->P[i + 1][i + 1] * kalman->dt;
    kalman->P[i][i] += kalman->P[i + 1][i] * kalman->dt + kalman->dt * (kalman->P[i][i + 1] + d_dt) + kalman->Q[i][i];
    kalman->P[i][i + 1] += d_dt + kalman->Q[i][i + 1];
    kalman->P[i + 1][i] += d_dt + kalman->Q[i + 1][i];
    kalman->P[i + 1][i + 1] += kalman->Q[i + 1][i + 1];
  }

  // Incase we ever expand the dynamic model...
  // // propagate covariance P = F*P*Ft + Q
  // float Ft[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];
  // float FP[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];
  // float FPFt[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];

  // MAKE_MATRIX_PTR(_F, kalman->F, TARGET_POS_KALMAN_DIM);
  // MAKE_MATRIX_PTR(_Ft, Ft, TARGET_POS_KALMAN_DIM);
  // MAKE_MATRIX_PTR(_P, kalman->P, TARGET_POS_KALMAN_DIM);
  // MAKE_MATRIX_PTR(_Q, kalman->Q, TARGET_POS_KALMAN_DIM);
  // MAKE_MATRIX_PTR(_FP, FP, TARGET_POS_KALMAN_DIM);
  // MAKE_MATRIX_PTR(_FPFt, FPFt, TARGET_POS_KALMAN_DIM);

  // float_mat_transpose(_Ft, _F, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM); // Ft mxn
  // float_mat_mul(_FP, _F, _P, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM); // F * P nxn
  // float_mat_mul(_FPFt, _FP, _Ft, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM); // F*P*Ft nxn
  // float_mat_sum(_P, _FPFt, _Q, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM); // P = F*P*Ft + Q nxn)

#if TARGET_POS_KALMAN_DEBUG
  float zeros[TARGET_POS_KALMAN_DIM] = {0};
  float P[TARGET_POS_KALMAN_DIM * TARGET_POS_KALMAN_DIM];
  float state_out[TARGET_POS_KALMAN_DIM];

  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    state_out[i] = kalman->state[i];
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      P[i * TARGET_POS_KALMAN_DIM + j] = kalman->P[i][j];
    }
  }

  char step[20];
  int rc = snprintf(step, sizeof(step), "p");
  // send debug message
  #if !USE_NPS
  pprz_msg_send_TARGET_POS_KALMAN_DEBUG(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 
                                  rc, step, state_in, state_out, P, zeros, zeros);
  #endif
  DOWNLINK_SEND_TARGET_POS_KALMAN_DEBUG(DefaultChannel, DefaultDevice,
                                  rc, step, state_in, state_out, P, zeros, zeros);
#endif
}

/** generic correction step
 *
 * K = PHt(HPHt+R)^-1 = PHtS^-1
 * X = X + K(Z-HX)
 * P = (I-KH)P
 *
 * @param kalman pointer to kalman structure
 * @param[in] H pointer to observation matrix
 * @param[in] Z pointer to measurement vector
 */
void target_pos_kalman_update(struct TargetPosKalman *kalman, struct KalmanSensor *sensor)
{

#if TARGET_POS_KALMAN_DEBUG
  float state_in[TARGET_POS_KALMAN_DIM];
  float meas[TARGET_POS_KALMAN_DIM] = {0};
  float Hmat[TARGET_POS_KALMAN_DIM] = {0};
  float noise[TARGET_POS_KALMAN_DIM] = {0};

  for (int i = 0; i < sensor->n_meas; i++) {
    meas[i] = sensor->meas[i];
    Hmat[i] = sensor->Hmat[i];
    noise[i] = sensor->noise[i];
  }

  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    state_in[i] = kalman->state[i];
  }
#endif

  // prepare variables and pointers
  float H[sensor->n_meas][TARGET_POS_KALMAN_DIM];
  float Ht[TARGET_POS_KALMAN_DIM][sensor->n_meas];
  float HP[sensor->n_meas][TARGET_POS_KALMAN_DIM];
  float S[sensor->n_meas][sensor->n_meas];
  float invS[sensor->n_meas][sensor->n_meas];
  float HtinvS_tmp[TARGET_POS_KALMAN_DIM][sensor->n_meas];
  float K[TARGET_POS_KALMAN_DIM][sensor->n_meas];
  float HX_tmp[TARGET_POS_KALMAN_DIM];
  float Z_HX[TARGET_POS_KALMAN_DIM];
  float K_ZHX_tmp[TARGET_POS_KALMAN_DIM];
  float KH_tmp[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];
  float P_tmp[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];

  // Generate the H matrix
  for (int i = 0; i < sensor->n_meas; i++) {
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      H[i][j] = 0;
    }
  }
  
  for (int i = 0; i < sensor->n_meas; i++) {
    H[i][sensor->Hmat[i]] = 1.0;
  }

  MAKE_MATRIX_PTR(_H, H, sensor->n_meas);
  MAKE_MATRIX_PTR(_Ht, Ht, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_HP, HP, sensor->n_meas);
  MAKE_MATRIX_PTR(_S, S, sensor->n_meas);
  MAKE_MATRIX_PTR(_invS, invS, sensor->n_meas);
  MAKE_MATRIX_PTR(_HtinvS_tmp, HtinvS_tmp, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_K, K, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_KH_tmp, KH_tmp, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_P, kalman->P, TARGET_POS_KALMAN_DIM);
  
  // Make S matrix
  float_mat_transpose(_Ht, _H, sensor->n_meas, TARGET_POS_KALMAN_DIM);                                        // Ht nxm
  float_mat_mul(_HP, _H, _P, sensor->n_meas, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);                   // S = H * P mxn
  float_mat_mul(_S, _HP, _Ht, sensor->n_meas, TARGET_POS_KALMAN_DIM, sensor->n_meas);                         // S = H * P * Ht mxm
  for (int i = 0; i < sensor->n_meas; i++) {
    _S[i][i] += sensor->noise[i];                                                                             // S = H * P * Ht + R
  }

  float abs_sum_S_diag = 0;
  for (int i = 0; i < sensor->n_meas; i++) {
    abs_sum_S_diag += fabsf(S[i][i]);
  }

  if (abs_sum_S_diag < 1e-5) {
    return; // don't invert S if it is too small
  }

  // finally compute gain and correct state
  float_mat_invert(_invS, _S, sensor->n_meas);                                                                // S^-1 mxm
  float_mat_mul(_HtinvS_tmp, _Ht, _invS, TARGET_POS_KALMAN_DIM, sensor->n_meas, sensor->n_meas);              // Ht * S^-1 nxm
  float_mat_mul(_K, _P, _HtinvS_tmp, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, sensor->n_meas);           // P * Ht * S^-1 nxm
  float_mat_vect_mul(HX_tmp, _H, kalman->state, sensor->n_meas, TARGET_POS_KALMAN_DIM);                       // H * x  mx1
  float_vect_diff(Z_HX, sensor->meas, HX_tmp, sensor->n_meas);                                                // Z - H * X mx1
  float_mat_vect_mul(K_ZHX_tmp, _K, Z_HX, TARGET_POS_KALMAN_DIM, sensor->n_meas);                             // K * (Z - H * X) nx1
  float_vect_add(kalman->state, K_ZHX_tmp, TARGET_POS_KALMAN_DIM);                                            // X + K * (Z - H * X) nx1

  // precompute K*H and store current P
  float_mat_mul(_KH_tmp, _K, _H, TARGET_POS_KALMAN_DIM, sensor->n_meas, TARGET_POS_KALMAN_DIM);
  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      P_tmp[i][j] = kalman->P[i][j];
    }
  }
  
  // correct covariance P = (I-K*H)*P = P - K*H*P
  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      for (int k = 0; k < TARGET_POS_KALMAN_DIM; k++) {
        kalman->P[i][j] -= KH_tmp[i][k] * P_tmp[k][j];
      }
    }
  }

  #if TARGET_POS_KALMAN_DEBUG
  float P[TARGET_POS_KALMAN_DIM * TARGET_POS_KALMAN_DIM];
  float state_out[TARGET_POS_KALMAN_DIM];

  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    state_out[i] = kalman->state[i];
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      P[i * TARGET_POS_KALMAN_DIM + j] = kalman->P[i][j];
    }
  }

  char step[20];
  int rc = snprintf(step, sizeof(step), "u");
  // send debug message
  #if !USE_NPS
  pprz_msg_send_TARGET_POS_KALMAN_DEBUG(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 
                                  rc, step, state_in, state_out, P, Hmat, meas);
  #endif
  DOWNLINK_SEND_TARGET_POS_KALMAN_DEBUG(DefaultChannel, DefaultDevice,
                                  rc, step, state_in, state_out, P, Hmat, meas);
#endif
}


void target_pos_kalman_set_measurement(struct KalmanSensor *sensor, float *meas) {
  for (int i = 0; i < sensor->n_meas; i++) {
    sensor->meas[i] = meas[i];
  }
}

void target_pos_kalman_set_noise(struct KalmanSensor *sensor, float *noise) {
  for (int i = 0; i < sensor->n_meas; i++) {
    sensor->noise[i] = noise[i];
  }
}