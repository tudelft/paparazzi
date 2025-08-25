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
#include "math/pprz_matrix_decomp_float.h"
#include <math.h>

#ifndef TARGET_POS_KALMAN_DEBUG
#define TARGET_POS_KALMAN_DEBUG FALSE
#endif

#ifndef TARGET_POS_KALMAN_USE_ACCEL
#define TARGET_POS_KALMAN_USE_ACCEL FALSE
#endif

#ifndef TARGET_POS_KALMAN_MIN_RCOND
#define TARGET_POS_KALMAN_MIN_RCOND 1e-6
#endif

#if TARGET_POS_KALMAN_DEBUG || TARGET_POS_KALMAN_USE_ACCEL
#include "state.h"
#endif

#if TARGET_POS_KALMAN_DEBUG
#include "modules/datalink/telemetry.h"
#include "modules/datalink/downlink.h"
#include <stdio.h>
#endif

PRINT_CONFIG_VAR(TARGET_POS_KALMAN_DEBUG);
PRINT_CONFIG_VAR(TARGET_POS_KALMAN_USE_ACCEL);

static void print_matrix(float **o, int rows, int cols, char *name)
{
  printf("%s:\n", name);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      printf("%f ", o[i][j]);
    }
    printf("\n");
  }
}

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
        kalman->F[i][i] = 1.0f;
      } else if (i % 2 == 0 && j == i + 1) {
        kalman->F[i][j] = dt;
      } else {
        kalman->F[i][j] = 0.0f;
      }
    }
  }

#if TARGET_POS_KALMAN_USE_ACCEL
  kalman->B[0][0] = dt2 / 2.f;
  kalman->B[1][0] = dt;
  kalman->B[2][1] = dt2 / 2.f;
  kalman->B[3][1] = dt;
  kalman->B[4][2] = dt2 / 2.f;
  kalman->B[5][2] = dt;
#endif
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

#if TARGET_POS_KALMAN_USE_ACCEL
  struct NedCoor_f accel_struct = *stateGetAccelNed_f();
  float accel[3] = {-accel_struct.x, -accel_struct.y, -accel_struct.z};
#endif

  for (i = 0; i < TARGET_POS_KALMAN_DIM; i += 2) {
    // kinematic equation of the dynamic model X = F*X (or X = F*X + B*U if acceleration is used)
    #if TARGET_POS_KALMAN_USE_ACCEL
    kalman->state[i] += kalman->state[i + 1] * kalman->dt + kalman->B[i][i/2] * accel[i/2];
    kalman->state[i + 1] += kalman->B[i + 1][i/2] * accel[i/2];
    #else
    kalman->state[i] += kalman->state[i + 1] * kalman->dt;
    #endif

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

  char step[5];
  int rc = snprintf(step, sizeof(step), "p");
  // send debug message
  #if !USE_NPS
  pprz_msg_send_TARGET_POS_KALMAN_DEBUG(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 
                                  rc, step, state_in, state_out, P, zeros, zeros);
  #endif
  RunOnceEvery(100, {
  DOWNLINK_SEND_TARGET_POS_KALMAN_DEBUG(DefaultChannel, DefaultDevice,
                                  rc, step, state_in, state_out, P, zeros, zeros);
  });
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
enum TargetPosKalmanUpdateResult target_pos_kalman_update(struct TargetPosKalman *kalman, struct KalmanSensor *sensor)
{

  if (kalman == NULL || sensor == NULL) {
    return TARGET_POS_KALMAN_UPDATE_NULL_PTR;
  }

  if (sensor->n_meas > TARGET_POS_KALMAN_DIM) {
    return TARGET_POS_KALMAN_UPDATE_DIM;
  }

#if TARGET_POS_KALMAN_DEBUG
  float state_in[TARGET_POS_KALMAN_DIM];
  float meas[TARGET_POS_KALMAN_DIM] = {0};
  float Hmat[TARGET_POS_KALMAN_DIM] = {0};

  for (int i = 0; i < sensor->n_meas; i++) {
    meas[i] = sensor->meas[i];
    Hmat[i] = sensor->Hmat[i];
  }

  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    state_in[i] = kalman->state[i];
  }
#endif // TARGET_POS_KALMAN_DEBUG

  // prepare variables and pointers
  float H[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x n
  float Ht[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // n x m
  float HP[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x n
  float S[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x m
  float invS[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x m
  float HtinvS_tmp[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x n
  float K[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // n x m
  float K_T[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x n
  float HX_tmp[TARGET_POS_KALMAN_DIM]; // m x 1
  float Z_HX[TARGET_POS_KALMAN_DIM]; // m x 1
  float K_ZHX_tmp[TARGET_POS_KALMAN_DIM];
  float KH[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // n x m
  float P_tmp[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];
  float I[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];
  float IKH[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];
  float IKH_T[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM];
  float KRK_T[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // n x m
  float KR[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // n x m
  float R[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x m
  float IKHP[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // n x m

  // Generate the H matrix
  for (int i = 0; i < sensor->n_meas; i++) {
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      H[i][j] = 0;
    }
  }

  // Generate the identity matrix
  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      I[i][j] = (i == j) ? 1.0f : 0.0f; // identity matrix
    }
  }

  // Fill in R matrix
  for (int i = 0; i < sensor->n_meas; i++) {
    for (int j = 0; j < sensor->n_meas; j++) {
      R[i][j] = (i == j) ? sensor->noise[i] : 0.0f;
    }
  }

  for (int i = 0; i < sensor->n_meas; i++) {
    if (sensor->Hmat[i] >= TARGET_POS_KALMAN_DIM) {
      return TARGET_POS_KALMAN_UPDATE_OOB;
    }
    H[i][sensor->Hmat[i]] = 1.0f;
  }

  MAKE_MATRIX_PTR(_H, H, sensor->n_meas);
  MAKE_MATRIX_PTR(_Ht, Ht, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_HP, HP, sensor->n_meas);
  MAKE_MATRIX_PTR(_S, S, sensor->n_meas);
  MAKE_MATRIX_PTR(_invS, invS, sensor->n_meas);
  MAKE_MATRIX_PTR(_HtinvS_tmp, HtinvS_tmp, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_K, K, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_K_T, K_T, sensor->n_meas);
  MAKE_MATRIX_PTR(_KH, KH, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_P, kalman->P, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_P_TMP, P_tmp, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_I, I, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_IKH, IKH, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_IKHP, IKHP, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_IKH_T, IKH_T, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_KRK_T, KRK_T, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_KR, KR, TARGET_POS_KALMAN_DIM);
  MAKE_MATRIX_PTR(_R, R, sensor->n_meas);

  // Make S matrix
  float_mat_transpose(_Ht, _H, sensor->n_meas, TARGET_POS_KALMAN_DIM);                                        // Ht nxm
  float_mat_mul(_HP, _H, _P, sensor->n_meas, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);                   // S = H * P mxn
  float_mat_mul(_S, _HP, _Ht, sensor->n_meas, TARGET_POS_KALMAN_DIM, sensor->n_meas);                         // S = H * P * Ht mxm
  
  for (int i = 0; i < sensor->n_meas; i++) {
    _S[i][i] += sensor->noise[i];                                                                             // S = H * P * Ht + R
  }

  // Check if S is invertible with SVD
  float w[TARGET_POS_KALMAN_DIM]; // m x 1
  float v[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x m
  float SC[TARGET_POS_KALMAN_DIM][TARGET_POS_KALMAN_DIM]; // m x m

  MAKE_MATRIX_PTR(_SC, SC, sensor->n_meas);
  MAKE_MATRIX_PTR(_V, v, sensor->n_meas);
  float_mat_copy(_SC, _S, sensor->n_meas, sensor->n_meas);

  int solved = pprz_svd_float(_SC, w, _V, sensor->n_meas, sensor->n_meas);
  
  if (!solved) {
    return TARGET_POS_KALMAN_UPDATE_SVD;
  }

  // Find the Condition number of the matrix
  float min_singular_value = w[0];
  float max_singular_value = w[0];
  for (int i = 0; i < sensor->n_meas; i++) {
    if (w[i] < min_singular_value) {
      min_singular_value = w[i];
    } else if (w[i] > max_singular_value) {
      max_singular_value = w[i];
    }
  }

  if (min_singular_value / max_singular_value < TARGET_POS_KALMAN_MIN_RCOND) {
    // Condition number is too large, don't invert S matrix
    return TARGET_POS_KALMAN_UPDATE_RCOND;
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
  float_mat_mul(_KH, _K, _H, TARGET_POS_KALMAN_DIM, sensor->n_meas, TARGET_POS_KALMAN_DIM);
  float_mat_copy(_P_TMP, _P, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);

  // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T (Joseph stabilized form)
  float_mat_diff(_IKH, _I, _KH, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);
  float_mat_transpose(_IKH_T, _IKH, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);
  float_mat_mul(_IKHP, _IKH, _P_TMP, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);
  float_mat_transpose(_K_T, _K, TARGET_POS_KALMAN_DIM, sensor->n_meas);
  float_mat_mul(_P_TMP, _IKHP, _IKH_T, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);
  float_mat_mul(_KR, _K, _R, TARGET_POS_KALMAN_DIM, sensor->n_meas, sensor->n_meas);
  float_mat_mul(_KRK_T, _KR, _K_T, TARGET_POS_KALMAN_DIM, sensor->n_meas, TARGET_POS_KALMAN_DIM);
  float_mat_sum(_P, _P_TMP, _KRK_T, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM);

  // print_matrix(_H, sensor->n_meas, TARGET_POS_KALMAN_DIM, "H");
  // print_matrix(_Ht, TARGET_POS_KALMAN_DIM, sensor->n_meas, "Ht");
  // print_matrix(_HP, sensor->n_meas, TARGET_POS_KALMAN_DIM, "HP");
  // print_matrix(_S, sensor->n_meas, sensor->n_meas, "S");
  // print_matrix(_invS, sensor->n_meas, sensor->n_meas, "invS");
  // print_matrix(_HtinvS_tmp, TARGET_POS_KALMAN_DIM, sensor->n_meas, "HtinvS_tmp");
  // print_matrix(_K, TARGET_POS_KALMAN_DIM, sensor->n_meas, "K");
  // print_matrix(_K_T, sensor->n_meas, TARGET_POS_KALMAN_DIM, "K_T");
  // print_matrix(_KH, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "KH");
  // print_matrix(_P, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "P");
  // print_matrix(_P_TMP, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "P_TMP");
  // print_matrix(_I, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "I");
  // print_matrix(_IKH, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "IKH");
  // print_matrix(_IKHP, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "IKHP");
  // print_matrix(_IKH_T, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "IKH_T");
  // print_matrix(_KRK_T, TARGET_POS_KALMAN_DIM, TARGET_POS_KALMAN_DIM, "KRK_T");
  // print_matrix(_KR, TARGET_POS_KALMAN_DIM, sensor->n_meas, "KR");
  // print_matrix(_R, sensor->n_meas, sensor->n_meas, "R");

#if TARGET_POS_KALMAN_DEBUG
  float P_debug[TARGET_POS_KALMAN_DIM * TARGET_POS_KALMAN_DIM];
  float state_out[TARGET_POS_KALMAN_DIM];

  for (int i = 0; i < TARGET_POS_KALMAN_DIM; i++) {
    state_out[i] = kalman->state[i];
    for (int j = 0; j < TARGET_POS_KALMAN_DIM; j++) {
      P_debug[i * TARGET_POS_KALMAN_DIM + j] = kalman->P[i][j];
    }
  }

  char step[5];
  int rc = snprintf(step, sizeof(step), "u");
  // send debug message
  #if !USE_NPS
  pprz_msg_send_TARGET_POS_KALMAN_DEBUG(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 
                                  rc, step, state_in, state_out, P_debug, Hmat, meas);
  #endif
  RunOnceEvery(100, {
  DOWNLINK_SEND_TARGET_POS_KALMAN_DEBUG(DefaultChannel, DefaultDevice,
                                  rc, step, state_in, state_out, P_debug, Hmat, meas);
  });
#endif // TARGET_POS_KALMAN_DEBUG

  return TARGET_POS_KALMAN_UPDATE_SUCCESS;
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