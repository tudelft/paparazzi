/**
 * @file modules/ins/ins_eskf_c.h
 * @brief Error-State Kalman Filter (ESKF) implementation in pure C, header.
 *
 * @defgroup ins_eskf_c Error-State Kalman Filter (ESKF)
 * @ingroup math
 * @{
 */

#ifndef INS_EKF2_C_H
#define INS_EKF2_C_H

#include "modules/ahrs/ahrs.h"
#include "modules/ins/ins.h"
#include "modules/core/abi.h"

/**
 * @brief ESKF State Structure.
 * Contains the nominal state, sensor variables, boolean flags for asynchronous execution, and ABI handlers.
 */
struct ekf2_c_t {
  struct FloatRates delta_gyro;   ///< Last gyroscope measurements
  struct FloatVect3 delta_accel;  ///< Last accelerometer measurements
  struct FloatVect3 mag;          ///< Magnetometer measurements
  float airspeed;                 ///< Airspeed measurements (e.g. from Pitot)
  float baro_alt;                 ///< Barometer altitude measurements
  float agl;                      ///< Above Ground Level from rangefinder
  uint32_t gyro_dt;
  uint32_t accel_dt;
  bool gyro_valid;
  bool accel_valid;
  bool mag_valid;                 ///< Flag if mag data is received
  bool airspeed_valid;            ///< Flag if airspeed data is received
  bool baro_valid;                ///< Flag if barometer data is received
  bool sideslip_valid;            ///< Flag if sideslip data is received
  bool agl_valid;                 ///< Flag if AGL/range data is received

  // Output states
  struct FloatQuat quat;
  struct FloatVect3 vel;
  struct FloatVect3 pos;
  struct FloatRates gyro_bias;
  struct FloatVect3 accel_bias;

  bool got_imu_data;

  abi_event gyro_ev;
  abi_event accel_ev;
  abi_event mag_ev;
  abi_event gps_ev;
  abi_event airspeed_ev;
  abi_event baro_ev;
  abi_event incidence_ev;
  abi_event agl_ev;
  abi_event geo_mag_ev;
};

extern struct ekf2_c_t ekf2_c_state;

extern float ins_eskf_c_gps_p_noise;
extern float ins_eskf_c_gps_v_noise;

extern void ins_ekf2_c_init(void);
extern void ins_ekf2_c_update(void);
extern void ins_ekf2_c_mea_pos(struct FloatVect3 *pos_meas, struct FloatVect3 *pos_noise);
extern void ins_ekf2_c_mea_mag(void);
extern void ins_ekf2_c_mea_airspeed(float airspeed_meas, float airspeed_noise);
extern void ins_ekf2_c_mea_baro(float baro_alt_meas, float baro_alt_noise);
extern void ins_ekf2_c_mea_sideslip(float sideslip_meas, float sideslip_noise);
extern void ins_ekf2_c_mea_agl(float agl_meas, float agl_noise);

#endif

/** @} */
