/**
 * Copyright (C) 2026 OpenUAS <info@openuas.org>
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
 *
 * @file modules/ins/ins_eskf.c
 * @brief Error-State Kalman Filter (ESKF) implementation in pure C no reliance on another compiler
 *
 * @ingroup ins_eskf
 *
 * This filter fuses IMU (accelerometer and gyroscope) measurements with GPS
 * and magnetometer data to estimate the attitude, velocity, and position
 * of the vehicle, alongside gyroscope and accelerometer biases.
 *
 * Operating Model:
 * 1. Nominal State Update: Predicts state forward using kinematic equations.
 * 2. Error State Update: Propagates error covariance matrix forward in time.
 * 3. Measurement Update: Corrects the nominal state by computing an error state
 *    from exteroceptive sensors (GPS/Mag) and applying it to the nominal state.
 */

/* Make naming better e.g.no fixed 15x15, refactor, add sensors optiflow abd whatnot */

#include "modules/ins/ins_eskf.h"

#include "math/pprz_isa.h"
#include "state.h"

/** For SITL and NPS we need special includes */
#if defined SITL && USE_NPS
  #include "nps_autopilot.h"
  #include <stdio.h>
#endif

/** INS reference from flight plan, true by default */
#ifndef USE_INS_NAV_INIT
  #define USE_INS_NAV_INIT TRUE
#endif

struct eskf_t eskf_state;

#ifndef INS_ESKF_GPS_P_NOISE
  #define INS_ESKF_GPS_P_NOISE 0.5f
#endif

#ifndef INS_ESKF_GPS_V_NOISE
  #define INS_ESKF_GPS_V_NOISE 0.3f
#endif

float ins_eskf_gps_p_noise = INS_ESKF_GPS_P_NOISE;
float ins_eskf_gps_v_noise = INS_ESKF_GPS_V_NOISE;

// Magic generic earth magnetic field vector (normalized approximation)
// This should best be initialized based on GPS location.
static struct FloatVect3 mag_earth_ref = {0.413550f, -0.010176f, 0.910424f};//FIXME take aircraft defines, for now these are the default for if there is no GNSS lock

/* ABI Bindings Configuration defaults from XML */
#ifndef INS_ESKF_GYRO_ID
  #define INS_ESKF_GYRO_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_ACCEL_ID
  #define INS_ESKF_ACCEL_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_MAG_ID
  #define INS_ESKF_MAG_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_GPS_ID
  #define INS_ESKF_GPS_ID GPS_MULTI_ID
#endif
#ifndef INS_ESKF_AIRSPEED_ID
  #define INS_ESKF_AIRSPEED_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_BARO_ID
  #define INS_ESKF_BARO_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_INCIDENCE_ID
  #define INS_ESKF_INCIDENCE_ID ABI_BROADCAST
#endif
#ifndef INS_ESKF_AGL_ID
  #define INS_ESKF_AGL_ID ABI_BROADCAST
#endif

/**
 * @name ABI Callbacks
 * @brief Handlers for asynchronous sensory data parsed from ABI messages.
 * @{
 */
/**
 * @brief Gyroscope ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param gyro Raw gyroscope data
 */
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro)
{
  (void)sender_id; /* unused in basic filter */
  static uint32_t last_stamp = 0;
  if (last_stamp > 0) {
    eskf_state.gyro_dt = stamp - last_stamp;
  } else {
    eskf_state.gyro_dt = 10000; // default 10ms initial
  }
  last_stamp = stamp;

  RATES_FLOAT_OF_BFP(eskf_state.delta_gyro, *gyro);
  eskf_state.gyro_valid = true;
}

/**
 * @brief Accelerometer ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param accel Raw accel data
 */
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel)
{
  (void)sender_id; /* unused in basic filter */
  static uint32_t last_stamp = 0;
  if (last_stamp > 0) {
    eskf_state.accel_dt = stamp - last_stamp;
  } else {
    eskf_state.accel_dt = 10000;
  }
  last_stamp = stamp;

  ACCELS_FLOAT_OF_BFP(eskf_state.delta_accel, *accel);
  eskf_state.accel_valid = true;
  eskf_state.got_imu_data = eskf_state.gyro_valid && eskf_state.accel_valid;
}

/**
 * @brief Magnetometer ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param mag Raw mag data
 */
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag)
{
  (void)sender_id;
  (void)stamp;
  MAGS_FLOAT_OF_BFP(eskf_state.mag, *mag);
  eskf_state.mag_valid = true;
  ins_eskf_mea_mag();
}

/**
 * @brief GPS ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param gps_s Standard GPS struct
 */
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s)
{
  (void)sender_id; (void)stamp;
  if (gps_s->fix >= GPS_FIX_3D) {
    /* Populate pos from GPS ned_vel / hmsl / etc and run measure */
    struct FloatVect3 pos_meas;
    pos_meas.x = gps_s->ecef_pos.x / 100.0f; /* cm to m */
    pos_meas.y = gps_s->ecef_pos.y / 100.0f;
    pos_meas.z = gps_s->ecef_pos.z / 100.0f;
    struct FloatVect3 pos_noise = {ins_eskf_gps_p_noise, ins_eskf_gps_p_noise, ins_eskf_gps_p_noise};
    ins_eskf_mea_pos(&pos_meas, &pos_noise);
  }
}

/**
 * @brief Airspeed ABI callback.
 * @param sender_id ABI sender ID
 * @param airspeed Calculated airspeed float
 */
static void airspeed_cb(uint8_t sender_id, float airspeed)
{
  (void)sender_id;
  eskf_state.airspeed = airspeed;
  eskf_state.airspeed_valid = true;
  ins_eskf_mea_airspeed(airspeed, 1.0f);
}

/**
 * @brief Barometer ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param pressure Pressure derived altitude/float
 */
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure)
{
  (void)sender_id; (void)stamp;
  static float baro_qfe =
    0.0f;  // see eg https://aeropeep.com/qnh-qfe-and-qne   Ground level pressure reference for altitude calculation, set on first valid reading
  if (pressure <= 0.0f) return;
  if (baro_qfe == 0.0f) baro_qfe = pressure; // Zero ground upon boot

  float alt = pprz_isa_height_of_pressure(pressure, baro_qfe);

  eskf_state.baro_valid = true;
  // Note: NED Z is Down, meaning positive upward altitude is strongly negative Z.
  ins_eskf_mea_baro(-alt,
                    2.0f);//TODO no magic numbers and noise model, maybe also add pressure as a separate measurement for better low altitude performance and less sensitivity to QFE errors
}

/**
 * @brief Incidence / Sideslip ABI callback.
 * @param sender_id ABI sender ID
 * @param flag Quality/availability flag
 * @param aoa Derived Angle of Attack
 * @param sideslip Derived Sideslip Angle
 */
static void incidence_cb(uint8_t sender_id, uint8_t flag, float aoa, float sideslip)
{
  (void)sender_id; (void)flag; (void)aoa;
  /* Convert sideslip angle flag if valid */
  eskf_state.sideslip_valid = true;
  // Assume sideslip noise around 0.1 rad
  ins_eskf_mea_sideslip(sideslip, 0.1f);
}

/**
 * @brief Rangefinder / AGL ABI callback.
 * @param sender_id ABI sender ID
 * @param stamp Timestamp
 * @param distance Distance scalar facing down
 */
static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance)
{
  (void)sender_id; (void)stamp;
  /* Feed AGL distance into the solution */
  eskf_state.agl = distance;
  eskf_state.agl_valid = true;
  ins_eskf_mea_agl(distance, 0.2f); // 0.2m noise std dev approximation
}

/**
 * @brief Geomagnetic Field ABI callback.
 * Updates the static generic Earth magnetic field reference if the geo_mag.xml module runs.
 * @param sender_id ABI sender ID
 * @param h Calculated Geomagnetic vector from geo_mag module based on current GPS loc
 */
static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{
  float n = float_vect3_norm(h);
  if (n > 0.01f) {
    mag_earth_ref.x = h->x / n;
    mag_earth_ref.y = h->y / n;
    mag_earth_ref.z = h->z / n;
  }
}

/** @} */

/**
 * Filter dimensions:
 * 0-2: Attitude error (delta rotation vector)
 * 3-5: Velocity error
 * 6-8: Position error
 * 9-11: Gyroscope bias error
 * 12-14: Accelerometer bias error
 */
#define EKF_N 15 //TODO: extend to more states like sideslip, wind, etc

// State error covariance matrix (P) and Process Noise covariance (Q)
static float P[EKF_N][EKF_N];
static float Q[EKF_N];

// Constants
static const float g_earth = 9.81f; // Standard local gravity

/* ------------------------------------------------------------------------- *
 * Helper Matrix Operations
 * Fixed 15x15 sizes to avoid dynamic allocation overhead and improve speed.
 * ------------------------------------------------------------------------- */

/**
 * @brief Multiply two 15x15 matrices: C = A * B
 * @param C Output result matrix
 * @param A Left operand matrix
 * @param B Right operand matrix
 */
static void mat_mult_15x15(float C[15][15], const float A[15][15], const float B[15][15])
{
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 15; j++) {
      C[i][j] = 0.0f;
    }
  }
  for (int i = 0; i < 15; i++) {
    for (int k = 0; k < 15; k++) {
      float a_ik = A[i][k];
      if (a_ik != 0.0f) {
        for (int j = 0; j < 15; j++) {
          C[i][j] += a_ik * B[k][j];
        }
      }
    }
  }
}

/**
 * @brief Multiply 15x15 matrices with the second matrix transposed: C = A * B^T
 * @param C Output result matrix
 * @param A Left operand matrix
 * @param B Right operand matrix (will be multiplied as B^T)
 */
static void mat_mult_15x15_transB(float C[15][15], const float A[15][15], const float B[15][15])
{
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 15; j++) {
      C[i][j] = 0.0f;
    }
  }
  for (int j = 0; j < 15; j++) {
    for (int k = 0; k < 15; k++) {
      float b_jk = B[j][k];
      if (b_jk != 0.0f) {
        for (int i = 0; i < 15; i++) {
          C[i][j] += A[i][k] * b_jk;
        }
      }
    }
  }
}


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
//TODO:
//maybe should have a proper function to send the full state, but for now we can reuse the INS message which has pos/vel/accel and is commonly used for EKF outputs. We can also add more messages later for more specific outputs like attitude, biases, etc.
//pprz_msg_send_INS(trans, dev, AC_ID, &pos.x, &pos.y, &pos.z, &speed.x, &speed.y, &speed.z, &accel.x, &accel.y, &accel.z);
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
// TODO:
// pprz_msg_send_INS_Z(trans, dev, AC_ID, &baro_z, &pos_z, &speed_z, &accel_z);
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
//FIXME: this is a bit hacky, we should have a proper function to send the reference and also include the qfe
//USE available message for now, but it is not ideal since it is meant for the local origin and not the full reference.
//     pprz_msg_send_INS_REF(trans, dev, AC_ID,
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_ins_eskf(struct transport_tx *trans, struct link_device *dev)
{
//Re-use the available EKF2 message
//   pprz_msg_send_INS_EKF2(trans, dev, AC_ID,
//                          &control_mode, &filter_fault_status, &gps_check_status, &soln_status,
//                          &innov_test_status, &mag, &vel, &pos, &hgt, &tas, &hagl, &flow, &beta,
//                          &mag_decl, &terrain_valid, &dead_reckoning);
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_ins_eskf_ext(struct transport_tx *trans, struct link_device *dev)
{
//Nothing yet, we can add more detailed ESKF outputs like GPS drift, vibration metrics, etc in a custom message or reuse an existing one if it fits.
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  //uint8_t ahrs_eskf_id = AHRS_COMP_ID_EKF2; // Reuse EKF2 ID for now, we can define a new one if needed
  //uint16_t filter_fault_status = 0; //TODO: we can define some fault status based on the innovation test failures, covariance limits, etc. For now we set it to 0 which means no fault.
  //uint16_t filter_fault_status_16 = filter_fault_status;
// pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_eskf_id, &mde, &filter_fault_status_16);
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_wind_info_ret(struct transport_tx *trans, struct link_device *dev)
{
//pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID, &flags, &wind(1), &wind(0), &f_zero, &tas);
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_ahrs_bias(struct transport_tx *trans, struct link_device *dev)
{
//pprz_msg_send_
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_ahrs_quat(struct transport_tx *trans, struct link_device *dev)
{
//pprz_msg_send_
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

static void send_external_pose_down(struct transport_tx *trans, struct link_device *dev)
{
// TODO: Send the estimated pose down to the ground for visualization.
// We can use the EXTERNAL_POSE_DOWN message which includes position and attitude in NED frame.
//pprz_msg_send_
  pprz_msg_send_PONG(trans, dev, AC_ID); //Placeholder until we implement the actual message
}

#endif

/**
 * @brief Initializes the ESKF.
 * Sets the initial nominal state, prediction covariances (P), and process noise (Q).
 */
void ins_eskf_init(void)
{
  struct eskf_t zero_state = {0};
  eskf_state = zero_state;

  // Initialize standard kinematics to zero/identity
  float_quat_identity(&eskf_state.quat);
  FLOAT_VECT3_ZERO(eskf_state.vel);
  FLOAT_VECT3_ZERO(eskf_state.pos);
  FLOAT_RATES_ZERO(eskf_state.gyro_bias);
  FLOAT_VECT3_ZERO(eskf_state.accel_bias);

  //TODO: less magic numbers overall everywhere, these are just rough initial guesses and should be tuned based on the expected dynamics and sensor noise characteristics of the specific platform.
  // TODO Use vaslues of very common sensors
  // Init state covariance matrix (P) with sensible initial uncertainty bounds
  for (int i = 0; i < 15; i++) { for (int j = 0; j < 15; j++) { P[i][j] = 0.0f; } }
  for (int i = 0; i < 3; i++) P[i][i] = 0.01f; // Attitude uncertainty
  for (int i = 3; i < 6; i++) P[i][i] = 1.0f;  // Velocity uncertainty
  for (int i = 6; i < 9; i++) P[i][i] = 1.0f;  // Position uncertainty
  for (int i = 9; i < 12; i++) P[i][i] = 0.0001f; // Gyro Bias uncertainty
  for (int i = 12; i < 15; i++) P[i][i] = 0.01f; // Accel Bias uncertainty


  // Init Process noise (Q) diagonals.
  // Represents confidence in our mathematical model vs sensor integration noise.
  for (int i = 0; i < 3; i++) Q[i] = 1e-4f;    // Attitude noise
  for (int i = 3; i < 6; i++) Q[i] = 1e-3f;    // Velocity noise
  for (int i = 6; i < 9; i++) Q[i] = 1e-5f;    // Position noise
  for (int i = 9; i < 12; i++) Q[i] = 1e-7f;   // Gyro bias wander
  for (int i = 12; i < 15; i++) Q[i] = 1e-5f;  // Accel bias wander

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_EKF2,
                              send_ins_eskf);//Re-Used EKF2 messages for now since we can reuse them, but we can define new ones if needed for more specific outputs.
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_EKF2_EXT, send_ins_eskf_ext);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_INFO_RET, send_wind_info_ret);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_BIAS, send_ahrs_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_QUAT_INT, send_ahrs_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EXTERNAL_POSE_DOWN, send_external_pose_down);
#endif

  /* Binding ABI messages */
  AbiBindMsgIMU_GYRO(INS_ESKF_GYRO_ID, &eskf_state.gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL(INS_ESKF_ACCEL_ID, &eskf_state.accel_ev, accel_cb);
  AbiBindMsgIMU_MAG(INS_ESKF_MAG_ID, &eskf_state.mag_ev, mag_cb);
  AbiBindMsgGPS(INS_ESKF_GPS_ID, &eskf_state.gps_ev, gps_cb);
  AbiBindMsgAIRSPEED(INS_ESKF_AIRSPEED_ID, &eskf_state.airspeed_ev, airspeed_cb);
  AbiBindMsgBARO_ABS(INS_ESKF_BARO_ID, &eskf_state.baro_ev, baro_cb);
  AbiBindMsgINCIDENCE(INS_ESKF_INCIDENCE_ID, &eskf_state.incidence_ev, incidence_cb);
  AbiBindMsgAGL(INS_ESKF_AGL_ID, &eskf_state.agl_ev, agl_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &eskf_state.geo_mag_ev, geo_mag_cb);
}

/**
 * @brief Normalizes a quaternion to ensure it represents a valid rotation.
 * @param q Pointer to the quaternion to normalize
 * Optimized for systems without an FPU by calculating the inverse once.
 */
static void normalize_quat(struct FloatQuat *q)
{
  float n = sqrtf(q->qi * q->qi + q->qx * q->qx + q->qy * q->qy + q->qz * q->qz);
  if (n > 1e-7f) {
    float inv_n = 1.0f / n; // One division, multiplicative application is faster
    q->qi *= inv_n; q->qx *= inv_n; q->qy *= inv_n; q->qz *= inv_n;
  } else {
    /* Fallback if totally degraded to prevent NaN/DivZero explosion */
    q->qi = 1.0f; q->qx = 0.0f; q->qy = 0.0f; q->qz = 0.0f;
  }
}

/**
 * @brief Skew-symmetric matrix generator.
 * @param m 3x3 output matrix
 * @param v 3D input vector
 * Converts a 3D vector [x, y, z] into a 3x3 anti-symmetric cross-product matrix.
 */
static void skew_symmetric(float m[3][3], const struct FloatVect3 *v)
{
  m[0][0] = 0;      m[0][1] = -v->z;  m[0][2] = v->y;
  m[1][0] = v->z;   m[1][1] = 0;      m[1][2] = -v->x;
  m[2][0] = -v->y;  m[2][1] = v->x;   m[2][2] = 0;
}

/**
 * @brief Core Periodic Update (Predict Step).
 * Should be called whenever new IMU measurements (Gyro & Accel) are ready.
 * Integrates the IMU readings to update the nominal state (Position, Velocity, Attitude)
 * and propagates the state error covariance matrix (P) forward in time.
 */
void ins_eskf_update(void)
{
  if (!eskf_state.gyro_valid || !eskf_state.accel_valid) return;

  // dt measured in seconds
  float dt = (float)eskf_state.gyro_dt * 0.000001f;

  /* SAFEGUARD: Limit maximum prediction step and protect against negative time */
  if (dt <= 0.0f) {
    eskf_state.gyro_valid = false;
    eskf_state.accel_valid = false;
    return;
  }
  if (dt > 0.2f) dt = 0.2f; // Assuming 5Hz is lowest practical IMU frequency

  // 1. Subtract estimated biases from raw IMU readings
  struct FloatRates omega;
  omega.p = eskf_state.delta_gyro.p - eskf_state.gyro_bias.p;
  omega.q = eskf_state.delta_gyro.q - eskf_state.gyro_bias.q;
  omega.r = eskf_state.delta_gyro.r - eskf_state.gyro_bias.r;

  struct FloatVect3 acc;
  acc.x = eskf_state.delta_accel.x - eskf_state.accel_bias.x;
  acc.y = eskf_state.delta_accel.y - eskf_state.accel_bias.y;
  acc.z = eskf_state.delta_accel.z - eskf_state.accel_bias.z;

  /* --------------------------------------------------------------------- *
   * NOMINAL STATE UPDATE (Kinematics Integration)
   * --------------------------------------------------------------------- */

  // Integrate Attitude (Quaternion)
  // q_new = q_old + 0.5 * q \otimes [0, omega] * dt
  float dt_half = 0.5f * dt;
  struct FloatQuat dq;
  dq.qi = (-eskf_state.quat.qx * omega.p - eskf_state.quat.qy * omega.q - eskf_state.quat.qz * omega.r) * dt_half;
  dq.qx = (eskf_state.quat.qi * omega.p + eskf_state.quat.qy * omega.r - eskf_state.quat.qz * omega.q) * dt_half;
  dq.qy = (eskf_state.quat.qi * omega.q - eskf_state.quat.qx * omega.r + eskf_state.quat.qz * omega.p) * dt_half;
  dq.qz = (eskf_state.quat.qi * omega.r + eskf_state.quat.qx * omega.q - eskf_state.quat.qy * omega.p) * dt_half;

  // First order Euler integration
  eskf_state.quat.qi += dq.qi;
  eskf_state.quat.qx += dq.qx;
  eskf_state.quat.qy += dq.qy;
  eskf_state.quat.qz += dq.qz;
  normalize_quat(&eskf_state.quat); // Must remain normalized

  // Get rotational matrix C (Body to NED) from our updated quaternion
  struct FloatRMat C;
  float_rmat_of_quat(&C, &eskf_state.quat);

  // Transform specific force into Navigation frame (NED) and subtract gravity
  struct FloatVect3 acc_ned;
  float_rmat_vmult(&acc_ned, &C, &acc);
  acc_ned.z += g_earth; // Gravity down, so we add to neutralize constant upward specific force

  // Integrate Velocity and Position
  eskf_state.vel.x += acc_ned.x * dt;
  eskf_state.vel.y += acc_ned.y * dt;
  eskf_state.vel.z += acc_ned.z * dt;

  eskf_state.pos.x += eskf_state.vel.x * dt;
  eskf_state.pos.y += eskf_state.vel.y * dt;
  eskf_state.pos.z += eskf_state.vel.z * dt;

  /* --------------------------------------------------------------------- *
   * ERROR STATE COVARIANCE UPDATE (F = Jacobian of Error Dynamics)
   * --------------------------------------------------------------------- */

  // F is the Error-State Transition Matrix: F = I + f_cont * dt
  float F[EKF_N][EKF_N] = {0};

  for (int i = 0; i < EKF_N; i++) F[i][i] = 1.0f; // Start with Identity

  // Error Jacobian Blocks:
  // Attitude error wrt Attitude (delta_theta_dot = -skew(omega) * delta_theta)
  struct FloatVect3 v_om = {omega.p, omega.q, omega.r};
  float omega_skew[3][3];
  skew_symmetric(omega_skew, &v_om);
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) F[0 + i][0 + j] -= omega_skew[i][j] * dt;

  // Attitude error wrt Gyro Bias (delta_theta_dot = -delta_bg)
  for (int i = 0; i < 3; i++) F[0 + i][9 + i] = -dt;

  // Velocity error wrt Attitude (delta_v_dot = -C * skew(acc) * delta_theta)
  float acc_skew[3][3];
  skew_symmetric(acc_skew, &acc);
  float C_acc_skew[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C_acc_skew[i][j] = C.m[i * 3 + 0] * acc_skew[0][j] + C.m[i * 3 + 1] * acc_skew[1][j] + C.m[i * 3 + 2] * acc_skew[2][j];
      F[3 + i][0 + j] = -C_acc_skew[i][j] * dt;
    }
  }

  // Velocity error wrt Accel Bias (delta_v_dot = -C * delta_ba)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      F[3 + i][12 + j] = -C.m[i * 3 + j] * dt;
    }
  }

  // Position error wrt Velocity (delta_p_dot = delta_v)
  for (int i = 0; i < 3; i++) F[6 + i][3 + i] = dt;

  // Propagate Covariance: P_new = F * P_old * F^T + Q
  float F_P[EKF_N][EKF_N];
  mat_mult_15x15(F_P, F, P);   // F * P

  float F_P_FT[EKF_N][EKF_N];
  mat_mult_15x15_transB(F_P_FT, F_P, F); // (F * P) * F^T

  // Save to P and add process noise Q
  for (int i = 0; i < EKF_N; i++) {
    for (int j = i; j < EKF_N; j++) {
      // Add structural exact symmetry enforcement alongside process noise
      float sym = 0.5f * (F_P_FT[i][j] + F_P_FT[j][i]);
      if (i == j) {
        sym += Q[i] * dt; // Add process noise
        if (sym < 1e-9f) sym = 1e-9f;
      }
      P[i][j] = sym;
      P[j][i] = sym; // Perfect symmetry bounds EKF drift mathematically
    }
  }

  // Reset IMU availability flags for the next run
  eskf_state.gyro_valid = false;
  eskf_state.accel_valid = false;

  /* SAFEGUARD: NaN Infestation Check */
  if (isnan(eskf_state.pos.x) || isnan(eskf_state.quat.qi) || isnan(eskf_state.vel.x)) {
    ins_eskf_init(); // Exploded, re-init.
    return;
  }

  // Output mapped states to the generic Paparazzi framework
  struct NedCoor_f ned_pos, ned_vel;
  ned_pos.x = eskf_state.pos.x; ned_pos.y = eskf_state.pos.y; ned_pos.z = eskf_state.pos.z;
  ned_vel.x = eskf_state.vel.x; ned_vel.y = eskf_state.vel.y; ned_vel.z = eskf_state.vel.z;

  stateSetPositionNed_f(0, &ned_pos);
  stateSetSpeedNed_f(0, &ned_vel);
  stateSetNedToBodyQuat_f(0, &eskf_state.quat);
}

/**
 * @brief Analytically computes the inverse of a 3x3 matrix using the determinant.
 * @param minv 3x3 inverted output matrix
 * @param m 3x3 input matrix
 * @return True if successful, False if matrix is singular
 * Fails safely (returns false) if the determinant is practically zero (singular).
 */
static bool invert_3x3(float minv[3][3], const float m[3][3])
{
  float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
              m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
              m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  if (fabsf(det) < 1e-6f) return false; // Matrix is non-invertible

  float invdet = 1.0f / det; // Singular division operation for performance

  // Compute the adjugate / determinant
  minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
  minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
  minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
  minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
  minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
  minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
  minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
  minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
  minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
  return true;
}

/**
 * @brief State Injection.
 * @param err_X Float array of error states representing delta corrections
 * Modifies the nominal states based on the calculated error state dx.
 */
static void apply_error_state(const float err_X[EKF_N])
{
  struct FloatVect3 delta_or = {err_X[0], err_X[1], err_X[2]};
  struct FloatQuat dq;

  // Convert angle error into quaternion error term (small angle approximation)
  // dq = [1, 0.5 * delta_or]
  dq.qi = 1.0f;
  dq.qx = 0.5f * delta_or.x;
  dq.qy = 0.5f * delta_or.y;
  dq.qz = 0.5f * delta_or.z;
  normalize_quat(&dq);

  // Inject attitude error (q_new = q_old * dq)
  struct FloatQuat q_new;
  float_quat_comp(&q_new, &eskf_state.quat, &dq);
  eskf_state.quat = q_new;
  normalize_quat(&eskf_state.quat);

  // Directly apply linear error to Velocity & Position
  eskf_state.vel.x += err_X[3];
  eskf_state.vel.y += err_X[4];
  eskf_state.vel.z += err_X[5];

  eskf_state.pos.x += err_X[6];
  eskf_state.pos.y += err_X[7];
  eskf_state.pos.z += err_X[8];

  // Update estimated biases
  eskf_state.gyro_bias.p += err_X[9];
  eskf_state.gyro_bias.q += err_X[10];
  eskf_state.gyro_bias.r += err_X[11];

  eskf_state.accel_bias.x += err_X[12];
  eskf_state.accel_bias.y += err_X[13];
  eskf_state.accel_bias.z += err_X[14];
}

/**
 * @brief Standard Multi-dimensional ESKF Kalman Update.
 * Computes the Kalman Gain and applies the calculated error to both the State and Covariance.
 * Uses the mathematically stable "Joseph Form" covariance update: P = (I - KH) P.
 *
 * @param H Observability Matrix relating measurement to state error.
 * @param R Measurement Noise Matrix.
 * @param z Measurement Innovation/Residual (Actual measurement - Expected measurement).
 */
static void eskf_update_3d(float H[3][EKF_N], float R[3][3], float z[3])
{

  // Calculate Innovation Covariance S: S = H * P * H^T + R
  float HP[3][EKF_N] = {0}; // Stores result of (H * P)
  for (int i = 0; i < 3; i++) {
    for (int k = 0; k < EKF_N; k++) {
      float H_ik = H[i][k];
      if (H_ik != 0.0f) { // Accelerate sparse observability multiplication
        for (int j = 0; j < EKF_N; j++) {
          HP[i][j] += H_ik * P[k][j];
        }
      }
    }
  }

  float S[3][3]; // Stores result of (HP * H^T + R)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      S[i][j] = R[i][j]; // Add R
      // Multiply HP by H^T (which is just multiplying by H with swapped indices)
      for (int k = 0; k < EKF_N; k++) {
        if (H[j][k] != 0.0f) {
          S[i][j] += HP[i][k] * H[j][k];
        }
      }
    }
  }

  // Invert the Innovation Covariance matrix
  float S_inv[3][3];
  if (!invert_3x3(S_inv, S)) return; // If inversion fails due to singularity, drop update

  // Calculate Kalman Gain K: K = P * H^T * S_inv
  float P_HT[EKF_N][3] = {0}; // Stores (P * H^T)
  for (int j = 0; j < 3; j++) {
    for (int k = 0; k < EKF_N; k++) {
      float H_jk = H[j][k];
      if (H_jk != 0.0f) {
        for (int i = 0; i < EKF_N; i++) {
          P_HT[i][j] += P[i][k] * H_jk;
        }
      }
    }
  }

  float K[EKF_N][3]; // Stores final Kalman Gain
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < 3; j++) {
      K[i][j] = 0.0f;
      for (int k = 0; k < 3; k++) K[i][j] += P_HT[i][k] * S_inv[k][j];
    }
  }

  // Calculate Error State vector: dx = K * z
  float dx[EKF_N] = {0};
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < 3; j++) {
      dx[i] += K[i][j] * z[j];
    }
  }

  // Inject computed error state into Nominal States
  apply_error_state(dx);

  // Covariance Matrix Update: TRUE mathematically stable Joseph Form
  // P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
  // This computationally prevents P from ever losing positive semi-definiteness.
  float I_KH[EKF_N][EKF_N];
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      float kh = 0.0f;
      for (int k = 0; k < 3; k++) kh += K[i][k] * H[k][j];
      I_KH[i][j] = (i == j ? 1.0f : 0.0f) - kh;
    }
  }

  float I_KH_P[EKF_N][EKF_N] = {0};
  for (int i = 0; i < EKF_N; i++) {
    for (int k = 0; k < EKF_N; k++) {
      if (I_KH[i][k] != 0.0f) {
        for (int j = 0; j < EKF_N; j++) {
          I_KH_P[i][j] += I_KH[i][k] * P[k][j];
        }
      }
    }
  }

  float P_new[EKF_N][EKF_N] = {0};
  for (int i = 0; i < EKF_N; i++) {
    for (int j = i; j < EKF_N; j++) {
      float val = 0.0f;
      // (I_KH * P) * I_KH^T
      for (int k = 0; k < EKF_N; k++) {
        val += I_KH_P[i][k] * I_KH[j][k];
      }
      // + K * R * K^T
      for (int r1 = 0; r1 < 3; r1++) {
        for (int r2 = 0; r2 < 3; r2++) {
          val += K[i][r1] * R[r1][r2] * K[j][r2];
        }
      }
      P_new[i][j] = val;
      P_new[j][i] = val;
    }
  }

  // Final swap into array & apply minimum covariances (Variance bounding)
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) P[i][j] = P_new[i][j];
    if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;

    // Pearson correlation coefficient bounding [-1, 1] to stop exploding covariance correlation rounding failures
    for (int j = i + 1; j < EKF_N; j++) {
      float max_cov = sqrtf(P[i][i] * P[j][j]);
      if (P[i][j] > max_cov) { P[i][j] = max_cov; P[j][i] = max_cov; }
      else if (P[i][j] < -max_cov) { P[i][j] = -max_cov; P[j][i] = -max_cov; }
    }
  }
}

/**
 * @brief 3D Position Measurement Update.
 * @param pos_meas Measured position float vector
 * @param pos_noise Float vector representing measurement covariance noise
 * Often fed by GPS or visual odometry. Provides observability into NED Position
 * vectors driving the internal biases into convergence.
 */
void ins_eskf_mea_pos(struct FloatVect3 *pos_meas, struct FloatVect3 *pos_noise)
{
  float H[3][EKF_N] = {0};

  // Jacobian mapping states to expected measurements.
  // Positions map 1:1 on indexes 6, 7, 8 in our Error State
  H[0][6] = 1.0f;
  H[1][7] = 1.0f;
  H[2][8] = 1.0f;

  // Fill the Measurement Noise covariance (R) from dynamic config
  float R[3][3] = {0};
  R[0][0] = pos_noise->x;
  R[1][1] = pos_noise->y;
  R[2][2] = pos_noise->z;

  // Calculate Innovation / Residual (difference between measured & expected Pos)
  float z[3];
  z[0] = pos_meas->x - eskf_state.pos.x;
  z[1] = pos_meas->y - eskf_state.pos.y;
  z[2] = pos_meas->z - eskf_state.pos.z;

  eskf_update_3d(H, R, z);
}

/**
 * @brief 3D Magnetometer Measurement Update.
 * Connects Earth's magnetic forces to our internal State. Heavily relies
 * on Attitude error states since compass rotations provide direct attitude constraints.
 */
void ins_eskf_mea_mag(void)
{
  if (!eskf_state.mag_valid) return;

  float H[3][EKF_N] = {0};

  // Retrieve the Rotation Matrix C (Body into Frame)
  struct FloatRMat C;
  float_rmat_of_quat(&C, &eskf_state.quat);

  // Project the expected mathematical Mag Earth Reference back into Body Frame
  // Expected Measurement: m_hat = C^T * m_earth
  struct FloatVect3 m_hat;
  m_hat.x = C.m[0] * mag_earth_ref.x + C.m[3] * mag_earth_ref.y + C.m[6] * mag_earth_ref.z;
  m_hat.y = C.m[1] * mag_earth_ref.x + C.m[4] * mag_earth_ref.y + C.m[7] * mag_earth_ref.z;
  m_hat.z = C.m[2] * mag_earth_ref.x + C.m[5] * mag_earth_ref.y + C.m[8] * mag_earth_ref.z;

  float m_hat_skew[3][3];
  skew_symmetric(m_hat_skew, &m_hat);

  // Fill the H matrix. The magnetometer primarily corrects Attitude Error
  // H(mag, attitude) = m_hat_skew
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      H[i][j] = m_hat_skew[i][j];
    }
  }

  // Magnetometer noise characteristics (Hardcoded approx for compass)
  float R[3][3] = {0};
  R[0][0] = 0.05f; R[1][1] = 0.05f; R[2][2] = 0.05f;

  // Computes measurement residuals: z = actual measurements - expected measurements
  float z[3];
  z[0] = eskf_state.mag.x - m_hat.x;
  z[1] = eskf_state.mag.y - m_hat.y;
  z[2] = eskf_state.mag.z - m_hat.z;

  eskf_update_3d(H, R, z);

  // Flag to wait for next reading
  eskf_state.mag_valid = false;
}

/**
 * @brief 1D ESKF Kalman Update.
 * Used for scalar measurements like Airspeed or Barometer Altitude.
 *
 * @param H Observability Vector (1xN).
 * @param R Measurement Noise Variance (scalar).
 * @param z Measurement Innovation/Residual (scalar).
 */
static void eskf_update_1d(const float H[EKF_N], float R, float z)
{
  // Calculate Innovation Variance S: S = H * P * H^T + R
  float HP[EKF_N] = {0}; // Stores result of (H * P)
  for (int k = 0; k < EKF_N; k++) {
    float h_k = H[k];
    if (h_k != 0.0f) { // Accelerate sparse observability multiplication
      for (int j = 0; j < EKF_N; j++) {
        HP[j] += h_k * P[k][j];
      }
    }
  }

  float S = R;
  for (int j = 0; j < EKF_N; j++) {
    if (H[j] != 0.0f) {
      S += HP[j] * H[j];
    }
  }

  // Invert Innovation Variance
  if (S < 1e-6f) return;
  float S_inv = 1.0f / S;

  // Calculate Kalman Gain K: K = P * H^T * S_inv
  float K[EKF_N] = {0};
  for (int k = 0; k < EKF_N; k++) {
    float h_k = H[k];
    if (h_k != 0.0f) {
      for (int i = 0; i < EKF_N; i++) {
        K[i] += P[i][k] * h_k;
      }
    }
  }
  for (int i = 0; i < EKF_N; i++) {
    K[i] *= S_inv;
  }

  // Calculate Error State vector: dx = K * z
  float dx[EKF_N];
  for (int i = 0; i < EKF_N; i++) {
    dx[i] = K[i] * z;
  }

  // Inject computed error state into Nominal States
  apply_error_state(dx);

  // TRUE Joseph Form mathematically stable covariance update for 1D: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
  float P_new[EKF_N][EKF_N] = {0};
  for (int i = 0; i < EKF_N; i++) {
    for (int j = i; j < EKF_N; j++) {
      // Instead of storing I_kH, compute elements on the fly for 1D since it's a vector multiplication
      // P_new_ij = P_ij - K_i(H*P)_j - (P*H^T)_i K_j + K_i * (H*P*H^T + R) * K_j
      // Since H*P*H^T + R is actually our Innovation scalar Variable "S" we already computed!
      float term = P[i][j] - K[i] * HP[j] - HP[i] * K[j] + K[i] * S * K[j];
      P_new[i][j] = term;
      P_new[j][i] = term;
    }
  }

  // Apply changes with variance and correlation safety limitations
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) P[i][j] = P_new[i][j];
    if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;

    // Bounds correlation matrix constraints safely
    for (int j = i + 1; j < EKF_N; j++) {
      float max_cov = sqrtf(P[i][i] * P[j][j]);
      if (P[i][j] > max_cov) { P[i][j] = max_cov; P[j][i] = max_cov; }
      else if (P[i][j] < -max_cov) { P[i][j] = -max_cov; P[j][i] = -max_cov; }
    }
  }
}

/**
 * @brief 1D Airspeed Measurement Update.
 * Uses a forward-facing Pitot tube reading to correct velocity and attitude.
 * Assumes the aircraft is flying with strictly forward velocity in its body frame
 * (minimal side-slip) and zero wind (or treats wind as sensor noise).
 *
 * @param airspeed_meas True forward airspeed measurement from sensor.
 * @param airspeed_noise Uncertainty/Variance in the airspeed reading.
 */
void ins_eskf_mea_airspeed(float airspeed_meas, float airspeed_noise)
{
  if (!eskf_state.airspeed_valid) return;

  float H[EKF_N] = {0};

  // Retrieve Rotation Matrix C (Body to NED) -> C^T is NED to Body
  struct FloatRMat C;
  float_rmat_of_quat(&C, &eskf_state.quat);

  // Expected forward airspeed in Body Frame (v_x_body)
  // v^B = C^T * v^N
  // v_x^B = C_00 * v_N + C_10 * v_E + C_20 * v_D
  float v_x_body = C.m[0] * eskf_state.vel.x + C.m[3] * eskf_state.vel.y + C.m[6] * eskf_state.vel.z;
  float v_y_body = C.m[1] * eskf_state.vel.x + C.m[4] * eskf_state.vel.y + C.m[7] * eskf_state.vel.z;
  float v_z_body = C.m[2] * eskf_state.vel.x + C.m[5] * eskf_state.vel.y + C.m[8] * eskf_state.vel.z;

  // Jacobian wrt Velocity (indexes 3, 4, 5)
  H[3] = C.m[0];
  H[4] = C.m[3];
  H[5] = C.m[6];

  // Jacobian wrt Attitude (indexes 0, 1, 2)
  // Derivative of v_x^B wrt delta_theta is the first row of skew(v^B)
  // skew(v^B) row 0 = [0, -v_z^B, v_y^B]
  H[0] = 0.0f;
  H[1] = -v_z_body;
  H[2] = v_y_body;

  // Innovation: Measured Airspeed - Expected Forward Velocity
  float z = airspeed_meas - v_x_body;

  // Perform Kalman update
  eskf_update_1d(H, airspeed_noise, z);

  // Reset flag for next reading
  eskf_state.airspeed_valid = false;
}

/**
 * @brief 1D Barometer Altitude Measurement Update.
 * @param baro_alt_meas Altitude measurement from barometric sensors
 * @param baro_alt_noise Measurement uncertainty/variance
 */
void ins_eskf_mea_baro(float baro_alt_meas, float baro_alt_noise)
{
  if (!eskf_state.baro_valid) return;

  eskf_state.baro_alt = baro_alt_meas;

  float H[EKF_N] = {0};
  // Maps to global Z Position (NED downward altitude: index 8)
  H[8] = 1.0f;

  float z = baro_alt_meas - eskf_state.pos.z;
  eskf_update_1d(H, baro_alt_noise * baro_alt_noise, z);
  eskf_state.baro_valid = false;
}

/**
 * @brief 1D Rangefinder / AGL Measurement Update.
 * @param agl_meas Distance measurement facing downwards to terrain
 * @param agl_noise Measurement uncertainty/variance
 */
void ins_eskf_mea_agl(float agl_meas, float agl_noise)
{
  if (!eskf_state.agl_valid) return;

  // Transform rangefinder distance pointing down (Body Z axis) to Earth Z axis (Alt)
  // Range * cos(pitch) * cos(roll) gives delta Altitude in generic approximation,
  // more accurately mathematically derived from C_33 (Rotation Matrix Z dot Z)
  struct FloatRMat C;
  float_rmat_of_quat(&C, &eskf_state.quat);
  float z_measured = -agl_meas * C.m[8]; // Negative because ground is down, and AGL is positive distance.

  float H[EKF_N] = {0};
  // Maps to global Z Position (NED downward altitude: index 8)
  H[8] = 1.0f;

  float z_innovation = z_measured - eskf_state.pos.z;
  eskf_update_1d(H, agl_noise * agl_noise, z_innovation);
  eskf_state.agl_valid = false;
}

/**
 * @brief Fuses sideslip angle measurement.
 * @param sideslip_meas Measurement angle in radians
 * @param sideslip_noise Estimate variance for sideslip constraint
 * Sideslip (beta) relates to the lateral body velocity (v_y^B).
 * v_y^B = V_airspeed * sin(beta) ~ V_airspeed * beta.
 * Often, this is used as a synthetic "zero sideslip" measurement (beta=0)
 * to correct lateral estimation and heading when flying forward.
 */
void ins_eskf_mea_sideslip(float sideslip_meas, float sideslip_noise)
{
  /* Current attitude matrix C: Body to NED */
  struct FloatRMat C;
  float_rmat_of_quat(&C, &eskf_state.quat);

  /* Map NED velocity to Body Frame */
  float v_bx = C.m[0] * eskf_state.vel.x + C.m[3] * eskf_state.vel.y + C.m[6] * eskf_state.vel.z;
  float v_by = C.m[1] * eskf_state.vel.x + C.m[4] * eskf_state.vel.y + C.m[7] * eskf_state.vel.z;
  float v_bz = C.m[2] * eskf_state.vel.x + C.m[5] * eskf_state.vel.y + C.m[8] * eskf_state.vel.z;

  /* Determine total airspeed estimate to scale sideslip angle appropriately */
  float V_est = sqrtf(v_bx * v_bx + v_by * v_by + v_bz * v_bz);
  if (V_est < 1.0f) return; // Discard fusion at near-zero speeds to avoid singularity

  /* Estimated sideslip observation y_est = v_by */
  float y_est = v_by;
  float y_meas = V_est * sinf(sideslip_meas);

  /* Jacobian H for v_by. H_theta = [-v_bz, 0, v_bx]. H_vel = C^T row 1 (Y axis) */
  float H[EKF_N] = {0};
  H[0] = -v_bz;
  H[2] = v_bx;
  H[3] = C.m[1]; H[4] = C.m[4]; H[5] = C.m[7];

  float R = (V_est * sideslip_noise) * (V_est * sideslip_noise);
  eskf_update_1d(H, R, y_meas - y_est);
}
