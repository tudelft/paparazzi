/*
 * Copyright (C) 2021 Guido de Croon <g.c.h.e.decroon@tudelft.nl>
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
 * @file subsystems/ins/ins_flow.c
 *
 * "Inertial" navigation system, which is actually mostly based on optical flow measurements.
 * Goal is actually to make it work in the end without any inertial sensors (with the possibility for using gyros).
 * Starting point should be the ahrs_*_wrappers
 */

#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"

/* default Gyro to use in INS */
#ifndef INS_FLOW_GYRO_ID
#define INS_FLOW_GYRO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FLOW_GYRO_ID)

/* default Accelerometer to use in INS */
#ifndef INS_FLOW_ACCEL_ID
#define INS_FLOW_ACCEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FLOW_ACCEL_ID)

/* default GPS to use in INS */
#ifndef INS_FLOW_GPS_ID
#define INS_FLOW_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_FLOW_GPS_ID)

/* All registered ABI events */
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event gps_ev;
static abi_event body_to_imu_ev;
struct gps_message gps_msg = {};

/* All ABI callbacks */
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);

/* Main FLOW structure for keeping track of the status - for now unused */
struct ins_flow_t {
  uint32_t gyro_stamp;
  uint32_t gyro_dt;
  uint32_t accel_stamp;
  uint32_t accel_dt;
  FloatRates gyro;
  FloatVect3 accel;
  bool gyro_valid;
  bool accel_valid;

  uint8_t quat_reset_counter;

  uint64_t ltp_stamp;
  struct LtpDef_i ltp_def;

  struct OrientationReps body_to_imu;
  bool got_imu_data;
};

/* Static local functions */
static bool ahrs_icq_output_enabled;
static uint32_t ahrs_icq_last_stamp;
static uint8_t ahrs_flow_id = AHRS_COMP_ID_FLOW;  ///< Component ID for FLOW
struct ins_flow_t ins_flow;

static void set_body_state_from_quat(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
// No telemetry yet...
#endif

static bool ahrs_icq_enable_output(bool enable)
{
  ahrs_icq_output_enabled = enable;
  return ahrs_icq_output_enabled;
}

/* Initialize the flow ins */
void ins_flow_init(void)
{

  ahrs_icq_output_enabled = AHRS_ICQ_OUTPUT_ENABLED;
  ahrs_icq_init();
  ahrs_register_impl(ahrs_icq_enable_output);

  /* Initialize struct */
  ins_flow.ltp_stamp = 0;
  ins_flow.accel_stamp = 0;
  ins_flow.gyro_stamp = 0;
  ins_flow.gyro_valid = false;
  ins_flow.accel_valid = false;
  ins_flow.got_imu_data = false;
  ins_flow.quat_reset_counter = 0;


#if PERIODIC_TELEMETRY
 // ...
#endif

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(INS_FLOW_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(INS_FLOW_ACCEL_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_FLOW_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
}

void ins_flow_update(void)
{
  /*
      // Normally, run the EKF with the sensor measurements buffered with the associated time stamps.

      // Publish to the state
      stateSetPositionNed_f(&pos);

      // Publish to state
      stateSetSpeedNed_f(&speed);

      // Publish to state
      stateSetAccelNed_f(&accel);
  */
}

static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  ahrs_icq_last_stamp = stamp;
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_ICQ propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_ICQ propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_icq.status == AHRS_ICQ_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();
  }
#endif
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t __attribute__((unused)) stamp,
                     struct Int32Vect3 *accel)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS int_cmpl_quat accel update.")
  PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
#endif
}

/** Rotate angles and rates from imu to body frame and set state */
static void set_body_state_from_quat(void)
{
  if (ahrs_icq_output_enabled) {
    /* Compute LTP to BODY quaternion */
    struct Int32Quat ltp_to_body_quat;
    struct Int32Quat *body_to_imu_quat = orientationGetQuat_i(&ahrs_icq.body_to_imu);
    int32_quat_comp_inv(&ltp_to_body_quat, &ahrs_icq.ltp_to_imu_quat, body_to_imu_quat);
    /* Set state */
    stateSetNedToBodyQuat_i(&ltp_to_body_quat);

    /* compute body rates */
    struct Int32Rates body_rate;
    struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&ahrs_icq.body_to_imu);
    int32_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_icq.imu_rate);
    /* Set state */
    stateSetBodyRates_i(&body_rate);
  }
}

/* Update INS based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
  uint32_t time_usec = stamp;
  int32_t lat = gps_s->lla_pos.lat;
  int32_t lon = gps_s->lla_pos.lon;
  int32_t alt = gps_s->hmsl;
  uint8_t fix_type = gps_s->fix;
  float eph = gps_s->hacc / 100.0;
  float epv = gps_s->vacc / 100.0;
  float sacc = gps_s->sacc / 100.0;
  float vel_m_s = gps_s->gspeed / 100.0;

  struct NedCoor_i ned_pos;
  ned_of_ecef_point_i(&ned_pos, &state.ned_origin_i, &gps_s->ecef_pos);
  stateSetPositionNed_i(&ned_pos);

  // immediately believe velocity and publish it:
  struct NedCoor_f vel_ned;
  vel_ned.x = (gps_s->ned_vel.x) / 100.0;
  vel_ned.y = (gps_s->ned_vel.y) / 100.0;
  vel_ned.z = (gps_s->ned_vel.z) / 100.0;
  stateSetSpeedNed_f(&vel_ned);

  bool vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  uint8_t nsats = gps_s->num_sv;

}

/* Save the Body to IMU information */
static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  orientationSetQuat_f(&ins_flow.body_to_imu, q_b2i_f);
}
