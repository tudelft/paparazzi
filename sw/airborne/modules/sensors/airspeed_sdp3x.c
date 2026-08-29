/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file modules/sensors/airspeed_sdp3x.c
 *  Airspeed driver for the SDP3X pressure sensor via I2C.
 */

#include "std.h"
#include "mcu_periph/i2c.h"
#include "modules/sensors/airspeed_sdp3x.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_isa.h"
#include "modules/core/abi.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#ifndef USE_AIRSPEED_SDP3X
#if USE_AIRSPEED
#define USE_AIRSPEED_SDP3X TRUE
PRINT_CONFIG_MSG("USE_AIRSPEED_SDP3X set to TRUE since this is set USE_AIRSPEED")
#endif
#endif

/** Use low pass filter on pressure values
 */
#ifndef USE_AIRSPEED_LOWPASS_FILTER
#define USE_AIRSPEED_LOWPASS_FILTER TRUE
#endif

/** Commands and scales
 */
#define SDP3X_SCALE_TEMPERATURE   200.0f
#define SDP3X_RESET_ADDR          0x00
#define SDP3X_RESET_CMD           0x06

#define SDP3X_CONT_MEAS_AVG_MODE  0x3615
#define SDP3X_CONT_NONE_MODE      0x361E
#define SDP3X_CONT_MODE_STOP      0x3FF9

/** Sensor I2C slave address (existing defaults 0x42, 0x44 and 0x46)
 */
#ifndef SDP3X_I2C_ADDR
#define SDP3X_I2C_ADDR 0x42
#endif

#ifdef SDP3X_PRESSURE_SCALE
#define SDP3X_PRESSURE_SCALE_CONFIGURED TRUE
#else
#define SDP3X_PRESSURE_SCALE_CONFIGURED FALSE
#define SDP3X_PRESSURE_SCALE 0.f
#endif

#ifndef SDP3X_VERIFY_PRESSURE_SCALE
#define SDP3X_VERIFY_PRESSURE_SCALE FALSE
#endif

/* Default offset
 */
#ifndef SDP3X_PRESSURE_OFFSET
#define SDP3X_PRESSURE_OFFSET 0.f
#endif

#ifndef SDP3X_DYNAMIC_PRESSURE_SWAPPED
#define SDP3X_DYNAMIC_PRESSURE_SWAPPED FALSE
#endif

#ifndef SDP3X_ENABLE_BIDIRECTIONAL
#define SDP3X_ENABLE_BIDIRECTIONAL FALSE
#endif

#if SDP3X_DYNAMIC_PRESSURE_SWAPPED != TRUE && SDP3X_DYNAMIC_PRESSURE_SWAPPED != FALSE
#error "SDP3X_DYNAMIC_PRESSURE_SWAPPED must be TRUE or FALSE"
#endif

#if SDP3X_ENABLE_BIDIRECTIONAL != TRUE && SDP3X_ENABLE_BIDIRECTIONAL != FALSE
#error "SDP3X_ENABLE_BIDIRECTIONAL must be TRUE or FALSE"
#endif

PRINT_CONFIG_VAR(SDP3X_PRESSURE_SCALE)
PRINT_CONFIG_VAR(SDP3X_PRESSURE_OFFSET)
PRINT_CONFIG_VAR(SDP3X_DYNAMIC_PRESSURE_SWAPPED)
PRINT_CONFIG_VAR(SDP3X_ENABLE_BIDIRECTIONAL)
PRINT_CONFIG_VAR(SDP3X_VERIFY_PRESSURE_SCALE)

/** Send each acquired sample in an AIRSPEED_RAW message.
 * This diagnostic mode also selects non-averaged sensor acquisition.
 */
#ifndef SDP3X_SYNC_SEND
#define SDP3X_SYNC_SEND FALSE
#endif

#if SDP3X_SYNC_SEND
#ifdef SDP3X_MODE
#undef SDP3X_MODE
#endif
#define SDP3X_MODE SDP3X_CONT_NONE_MODE
#elif !defined SDP3X_MODE
#define SDP3X_MODE SDP3X_CONT_MEAS_AVG_MODE
#endif

PRINT_CONFIG_VAR(SDP3X_SYNC_SEND)
PRINT_CONFIG_VAR(SDP3X_MODE)

/** Quadratic scale factor for indicated airspeed.
 * airspeed = sqrt(2*p_diff/density)
 * With p_diff in Pa and standard air density of 1.225 kg/m^3,
 * default airspeed scale is 2/1.225
 */
#ifndef SDP3X_AIRSPEED_SCALE
#define SDP3X_AIRSPEED_SCALE (2.0f / PPRZ_ISA_AIR_DENSITY)
#endif

/** Time constant for second order Butterworth low pass filter
 * Default of 0.15 should give cut-off freq of 1/(2*pi*tau) ~= 1Hz
 */
#ifndef SDP3X_LOWPASS_TAU
#define SDP3X_LOWPASS_TAU 0.15
#endif

struct AirspeedSdp3x sdp3x;
static struct i2c_transaction sdp3x_trans;

enum Sdp3xState {
  SDP3X_STATE_STOP,
  SDP3X_STATE_WAIT_AFTER_STOP,
  SDP3X_STATE_START,
  SDP3X_STATE_WAIT_AFTER_START,
  SDP3X_STATE_READ_SCALE,
  SDP3X_STATE_READ_DATA
};

#define SDP3X_MAX_CONSECUTIVE_CRC_ERRORS 5U

static enum Sdp3xState sdp3x_state;
static uint8_t sdp3x_autoset_count;
static float sdp3x_autoset_sum;
static uint8_t sdp3x_crc_error_count;

#if USE_AIRSPEED_LOWPASS_FILTER
static Butterworth2LowPass sdp3x_filter;
static bool sdp3x_filter_reset_pending;
#endif

static bool sdp3x_config_is_valid(void)
{
  return isfinite(sdp3x.pressure_scale) && sdp3x.pressure_scale > 0.f &&
         isfinite(sdp3x.pressure_offset) &&
         isfinite(sdp3x.airspeed_scale) && sdp3x.airspeed_scale > 0.f;
}

static float sdp3x_airspeed_from_pressure(float pressure)
{
  return sdp3x_eas_from_pressure(pressure, sdp3x.airspeed_scale);
}

static void sdp3x_restart(void)
{
  sdp3x_state = SDP3X_STATE_STOP;
  sdp3x_trans.status = I2CTransDone;
  sdp3x_autoset_count = 0;
  sdp3x_autoset_sum = 0.f;
  sdp3x_crc_error_count = 0;
#if USE_AIRSPEED_LOWPASS_FILTER
  sdp3x_filter_reset_pending = true;
#endif
}

#if PERIODIC_TELEMETRY || SDP3X_SYNC_SEND
static void sdp3x_downlink(struct transport_tx *trans, struct link_device *dev,
                           float pressure, float airspeed)
{
  uint8_t dev_id = SDP3X_SENDER_ID;
  pprz_msg_send_AIRSPEED_RAW(trans,dev,AC_ID,
                                &dev_id,
                                &sdp3x.raw_p,
                                &sdp3x.pressure_offset,
                                &pressure,
                                &sdp3x.temperature,
                                &airspeed);
}

#if PERIODIC_TELEMETRY && !SDP3X_SYNC_SEND
static void sdp3x_downlink_filtered(struct transport_tx *trans, struct link_device *dev)
{
  sdp3x_downlink(trans, dev, sdp3x.pressure, sdp3x.airspeed);
}
#endif
#endif

void sdp3x_init(void)
{
  sdp3x.pressure = 0.f;
  sdp3x.temperature = 0.f;
  sdp3x.airspeed = 0.f;
  sdp3x.pressure_scale = SDP3X_PRESSURE_SCALE;
  sdp3x.pressure_offset = SDP3X_PRESSURE_OFFSET;
  sdp3x.airspeed_scale = SDP3X_AIRSPEED_SCALE;
  sdp3x.autoset_offset = false;

  sdp3x_restart();
  // setup low pass filter with time constant and 100Hz sampling freq
#if USE_AIRSPEED_LOWPASS_FILTER
  init_butterworth_2_low_pass(&sdp3x_filter, SDP3X_LOWPASS_TAU,
                              SDP3X_PERIODIC_PERIOD, 0);
#endif

#if PERIODIC_TELEMETRY && !SDP3X_SYNC_SEND
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_RAW, sdp3x_downlink_filtered);
#endif
}

void sdp3x_periodic(void)
{
  if (sdp3x_trans.status != I2CTransDone) {
    return;
  }

  switch (sdp3x_state) {
    case SDP3X_STATE_STOP:
      sdp3x_trans.buf[0] = SDP3X_CONT_MODE_STOP >> 8;
      sdp3x_trans.buf[1] = SDP3X_CONT_MODE_STOP & 0xff;
      i2c_transmit(&SDP3X_I2C_DEV, &sdp3x_trans, SDP3X_I2C_ADDR, 2);
      break;
    case SDP3X_STATE_WAIT_AFTER_STOP:
      sdp3x_state = SDP3X_STATE_START;
      break;
    case SDP3X_STATE_START:
      sdp3x_trans.buf[0] = SDP3X_MODE >> 8;
      sdp3x_trans.buf[1] = SDP3X_MODE & 0xff;
      i2c_transmit(&SDP3X_I2C_DEV, &sdp3x_trans, SDP3X_I2C_ADDR, 2);
      break;
    case SDP3X_STATE_WAIT_AFTER_START:
      sdp3x_state = SDP3X_STATE_READ_SCALE;
      break;
    case SDP3X_STATE_READ_SCALE:
      i2c_receive(&SDP3X_I2C_DEV, &sdp3x_trans, SDP3X_I2C_ADDR, 9);
      break;
    case SDP3X_STATE_READ_DATA:
      i2c_receive(&SDP3X_I2C_DEV, &sdp3x_trans, SDP3X_I2C_ADDR, 6);
      break;
  }
}

#define AUTOSET_NB_MAX 20

void sdp3x_event(void)
{
  if (sdp3x_trans.status == I2CTransSuccess) {
    if (sdp3x_state == SDP3X_STATE_STOP) {
      sdp3x_state = SDP3X_STATE_WAIT_AFTER_STOP;
    } else if (sdp3x_state == SDP3X_STATE_START) {
      sdp3x_state = SDP3X_STATE_WAIT_AFTER_START;
    } else if (sdp3x_state == SDP3X_STATE_READ_SCALE) {
      uint8_t buf[9];
      for (uint8_t i = 0; i < 9; i++) {
        buf[i] = sdp3x_trans.buf[i];
      }

        if (!sdp3x_crc_valid(&buf[0], 2, buf[2]) ||
          !sdp3x_crc_valid(&buf[3], 2, buf[5]) ||
          !sdp3x_crc_valid(&buf[6], 2, buf[8])) {
        sdp3x_restart();
        return;
      }

      uint16_t sensor_scale = ((uint16_t)buf[6] << 8) | (uint16_t)buf[7];
      if (!sdp3x_scale_is_valid(sensor_scale)) {
        sdp3x_restart();
        return;
      }

#if SDP3X_PRESSURE_SCALE_CONFIGURED && SDP3X_VERIFY_PRESSURE_SCALE
    if ((float)sensor_scale != (float)SDP3X_PRESSURE_SCALE) {
        sdp3x_restart();
        return;
      }
#endif
#if SDP3X_PRESSURE_SCALE_CONFIGURED
    sdp3x.pressure_scale = (float)SDP3X_PRESSURE_SCALE;
#else
      sdp3x.pressure_scale = (float)sensor_scale;
#endif
      sdp3x_state = SDP3X_STATE_READ_DATA;
    } else {
      uint8_t buf[6];
      for (uint8_t i = 0; i < 6; i++) {
        buf[i] = sdp3x_trans.buf[i];
      }

      // Check the CRC
      if (!sdp3x_crc_valid(&buf[0], 2, buf[2]) || !sdp3x_crc_valid(&buf[3], 2, buf[5])) {
        if (++sdp3x_crc_error_count >= SDP3X_MAX_CONSECUTIVE_CRC_ERRORS) {
          sdp3x_restart();
        } else {
          sdp3x_trans.status = I2CTransDone;
        }
        return;
      }
      sdp3x_crc_error_count = 0;

      int16_t p_raw = sdp3x_decode_int16(buf[0], buf[1]);
      int16_t t_raw = sdp3x_decode_int16(buf[3], buf[4]);

      sdp3x.raw_p = (uint16_t)p_raw;
      sdp3x.temperature = (float)t_raw / SDP3X_SCALE_TEMPERATURE;

      if (!sdp3x_config_is_valid()) {
        sdp3x_restart();
        return;
      }

      float pressure_raw_pa = sdp3x_pressure_from_raw(p_raw, sdp3x.pressure_scale,
                      SDP3X_DYNAMIC_PRESSURE_SWAPPED);
      if (sdp3x.autoset_offset) {
        sdp3x_autoset_sum += pressure_raw_pa;
        sdp3x_autoset_count++;
        if (sdp3x_autoset_count >= AUTOSET_NB_MAX) {
          sdp3x.pressure_offset = sdp3x_autoset_sum / (float)sdp3x_autoset_count;
          sdp3x.autoset_offset = false;
          sdp3x_autoset_count = 0;
          sdp3x_autoset_sum = 0.f;
#if USE_AIRSPEED_LOWPASS_FILTER
          sdp3x_filter_reset_pending = true;
#endif
        }
      } else {
        sdp3x_autoset_count = 0;
        sdp3x_autoset_sum = 0.f;
      }

      float p_out = pressure_raw_pa - sdp3x.pressure_offset;
      float pressure_airspeed = sdp3x_pressure_for_airspeed(p_out, SDP3X_ENABLE_BIDIRECTIONAL);

#if USE_AIRSPEED_LOWPASS_FILTER
      if (sdp3x_filter_reset_pending) {
        sdp3x.pressure = reset_butterworth_2_low_pass(&sdp3x_filter, pressure_airspeed);
        sdp3x_filter_reset_pending = false;
      } else {
        sdp3x.pressure = update_butterworth_2_low_pass(&sdp3x_filter, pressure_airspeed);
      }
    #if !SDP3X_ENABLE_BIDIRECTIONAL
      if (sdp3x.pressure < 0.f) {
        sdp3x.pressure = reset_butterworth_2_low_pass(&sdp3x_filter, 0.f);
      }
    #endif
#else
      sdp3x.pressure = pressure_airspeed;
#endif

  // A second-order filter can undershoot after an abrupt pressure drop.
  sdp3x.pressure = sdp3x_pressure_for_airspeed(sdp3x.pressure, SDP3X_ENABLE_BIDIRECTIONAL);

      // Send (differential) pressure via ABI
      AbiSendMsgBARO_DIFF(SDP3X_SENDER_ID, sdp3x.pressure);
      // Send temperature as float in deg Celcius via ABI
      AbiSendMsgTEMPERATURE(SDP3X_SENDER_ID, sdp3x.temperature);
      // Equivalent airspeed in m/s at ISA sea-level density.
      sdp3x.airspeed = sdp3x_airspeed_from_pressure(sdp3x.pressure);

#if USE_AIRSPEED_SDP3X
      AbiSendMsgAIRSPEED(AIRSPEED_SDP3X_ID, sdp3x.airspeed);
#endif
#if SDP3X_SYNC_SEND
  float pressure_unfiltered = sdp3x_pressure_for_airspeed(p_out, SDP3X_ENABLE_BIDIRECTIONAL);
  float airspeed_unfiltered = sdp3x_airspeed_from_pressure(pressure_unfiltered);
      sdp3x_downlink(&(DefaultChannel).trans_tx, &(DefaultDevice).device,
         pressure_unfiltered, airspeed_unfiltered);
#endif
    }

    sdp3x_trans.status = I2CTransDone;
  } else if (sdp3x_trans.status == I2CTransFailed) {
    sdp3x_restart();
  }
}
