/**
 * @file modules/ins/ins_eskf.h
 * @brief Error-State Kalman Filter (ESKF) implementation in pure C, header.
 *
 * @defgroup ins_eskf Error-State Kalman Filter (ESKF)
 * @ingroup math
 * @{
 */

#ifndef INS_ESKF_H
#define INS_ESKF_H

#include "modules/ahrs/ahrs.h"
#include "modules/ins/ins.h"
#include "modules/core/abi.h"

/**
 * @brief ESKF State Structure.
 * Contains the nominal state, sensor variables, boolean flags for asynchronous execution, and ABI handlers.
 */
struct eskf_t {
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

extern struct eskf_t eskf_state;

extern float ins_eskf_gps_p_noise;
extern float ins_eskf_gps_v_noise;

extern void ins_eskf_init(void);
extern void ins_eskf_update(void);
extern void ins_eskf_mea_pos(struct FloatVect3 *pos_meas, struct FloatVect3 *pos_noise);
extern void ins_eskf_mea_mag(void);
extern void ins_eskf_mea_airspeed(float airspeed_meas, float airspeed_noise);
extern void ins_eskf_mea_baro(float baro_alt_meas, float baro_alt_noise);
extern void ins_eskf_mea_sideslip(float sideslip_meas, float sideslip_noise);
extern void ins_eskf_mea_agl(float agl_meas, float agl_noise);

#endif

/** @} */
