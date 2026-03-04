/*
 * Copyright (C) 2026 OpenUAS <noreply@openuas.org>
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
 */


/** @file modules/sensors/rangefinder_i2c.h
 *  @brief Driver for a rangfinder sensor when used via I2C bus
 */

#include "generated/airframe.h"
#include "modules/sensors/rangefinder_i2c.h"
#include "modules/core/abi.h"
#if defined(SITL) || (defined(RANGEFINDER_I2C_COMPENSATE_ROTATION) && (RANGEFINDER_I2C_COMPENSATE_ROTATION == 1))
#include "state.h"
#endif
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif

#ifdef RANGEFINDER_I2C_SYNC_SEND
#include "modules/datalink/downlink.h"
#endif

// Check if an I2C device is selected
#ifndef RANGEFINDER_I2C_PORT
#error RANGEFINDER_I2C_PORT needs to be defined
#endif
PRINT_CONFIG_VAR(RANGEFINDER_I2C_PORT)

// Default base address on 8 bits
#ifndef RANGEFINDER_I2C_ADDR
//#define RANGEFINDER_I2C_ADDR 0xE0 // Equals 0x70 if notated as 7bit address
#error RANGEFINDER_I2C_ADDR needs to be defined and als be in 8bit format address e.g. 0xE0 for 7bit address 0x70
#endif

#ifndef RANGEFINDER_I2C_READ_MODE_SINGLE
#define RANGEFINDER_I2C_READ_MODE_SINGLE 0x51 // Deducted from sparse random information and thereafter testing with GY-US42V2 sensor, other sensors may differ
#endif

// The minimum and maximum range what we want in our output e.g. sensor can do 7.2m but we only want to get dat less than 4m set MAX_RANGE to 4.0

/// The minimum chosen distance for the device to be give readings
#ifndef RANGEFINDER_I2C_MIN_RANGE
#define RANGEFINDER_I2C_MIN_RANGE 0.24f //Default for rangefinder is 0.24m since many sensors cannot measure a range closer than that reliably
#endif

/// The maximum chosen distance for the device to be give readings
#ifndef RANGEFINDER_I2C_MAX_RANGE
#define RANGEFINDER_I2C_MAX_RANGE 4.0f //Reasonable default value in meters with still usable readings for common rangefinder sensors
#endif

/// Rangefinder distance offset value for what should be considered zero distance, e.g. high landing gear of 1.1m to tarmac still could be considered zero
#ifndef RANGEFINDER_I2C_OFFSET
#define RANGEFINDER_I2C_OFFSET 0.0f
#endif

/// Send AGL data over ABI
#ifndef RANGEFINDER_I2C_USE_FOR_AGL
#define RANGEFINDER_I2C_USE_FOR_AGL 0
#endif

/// Filter the raw measuread sonar data
#ifndef RANGEFINDER_I2C_USE_FILTER
#define RANGEFINDER_I2C_USE_FILTER 1 // Enable filtering for rangefinder per default, sensors tend to give noisy spiking readings
#endif

#ifdef RANGEFINDER_I2C_USE_FILTER
#include "filters/median_filter.h"
///The amount of sensor samples to keep in the median filter buffer
#ifndef RANGEFINDER_I2C_MEDIAN_SIZE
#define RANGEFINDER_I2C_MEDIAN_SIZE 7 // Default median filter length of 7 is a good fit for most rangefinder sensors
#endif
struct MedianFilterFloat rangefinder_i2c_filter;
#endif

#ifndef RANGEFINDER_I2C_COMPENSATE_ROTATION
#define RANGEFINDER_I2C_COMPENSATE_ROTATION 0
#endif

// Gain for sonar sensors to get from raw measuread value to meters
#ifndef RANGEFINDER_I2C_SCALE
#define RANGEFINDER_I2C_SCALE 0.000044f  // Experimentally determined gain for famous GY-US42V2 sensor, since there is no datasheet afaik
#endif

struct RangefinderI2C rangefinder_i2c;

/**
 * Send measured value and status information so it can be read back in e.g. log file for debugging
 */
static void rangefinder_i2c_send_rangefinder(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_RANGEFINDER(trans, dev, AC_ID, &rangefinder_i2c.addr, &rangefinder_i2c.raw, &rangefinder_i2c.distance);
}

/**
 * Set the default values at initialization
 */
void rangefinder_i2c_init(void)
{
  rangefinder_i2c.trans.status = I2CTransDone;
  rangefinder_i2c.addr = RANGEFINDER_I2C_ADDR;

  //Init with defaults that do not cause harm
  rangefinder_i2c.distance = (float)RANGEFINDER_I2C_MIN_RANGE; // Start with minimum range as default distance
  rangefinder_i2c.raw = (uint16_t)(rangefinder_i2c.distance / RANGEFINDER_I2C_SCALE);
  rangefinder_i2c.update_agl = RANGEFINDER_I2C_USE_FOR_AGL;

  rangefinder_i2c.status = RANGEFINDER_I2C_REQ_DATA;

#ifdef RANGEFINDER_I2C_USE_FILTER
  init_median_filter_f(&rangefinder_i2c_filter, RANGEFINDER_I2C_MEDIAN_SIZE);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RANGEFINDER, rangefinder_i2c_send_rangefinder);
#endif

}

/**
 * Rangefinder event function
 * Basically just check the progress of the transation
 * to prevent overruns during high speed operation
 * (ie. polling the sensor at more than maximum, e.g. > 10Hz)
 */
void rangefinder_i2c_event(void)
{
  switch (rangefinder_i2c.trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
    case I2CTransFailed:
      // set to done
      rangefinder_i2c.trans.status = I2CTransDone;
      break;
    default:
      // do nothing
      break;
  }
#if RANGEFINDER_I2C_SYNC_SEND
  rangefinder_i2c_report();
#endif
}

/**
 *
 * Get the ranger current distance value
 * 
 * Note that if *_READ_MODE_SINGLE is defined something else than 0 meaning the sensor need a command to read
 * before it give a new updated range value.
 * Some sensors just emit the range data whenever the buffer is read, then no need for a request byte marker
 */
void rangefinder_i2c_periodic(void)
{
#ifndef SITL
  switch (rangefinder_i2c.status) {

    //Blocking I2C Transceive did not work for some of those I2C devices, therefore a state machine to handle the I2C transactions
    case RANGEFINDER_I2C_REQ_DATA:
      #if ( RANGEFINDER_I2C_READ_MODE_SINGLE == 0x00 )
        rangefinder_i2c.status = RANGEFINDER_I2C_READ_DATA; /* falls through */
        __attribute__((__fallthrough__));
      #else
        if (rangefinder_i2c.trans.status == I2CTransDone) {
          rangefinder_i2c.trans.buf[0] = RANGEFINDER_I2C_READ_MODE_SINGLE;
          if (i2c_transmit(&RANGEFINDER_I2C_PORT, &rangefinder_i2c.trans, rangefinder_i2c.addr, 1)) {
            rangefinder_i2c.status = RANGEFINDER_I2C_READ_DATA;
          }
        }
        break;   
      #endif 
    case RANGEFINDER_I2C_READ_DATA:
      if (rangefinder_i2c.trans.status == I2CTransDone) {
        #if ( RANGEFINDER_I2C_READ_MODE_SINGLE == 0x00 )
          rangefinder_i2c.trans.buf[0] = 0;
          rangefinder_i2c.trans.buf[1] = 0;
        #else
          rangefinder_i2c.trans.buf[1] = 0;
          rangefinder_i2c.trans.buf[2] = 0;
        #endif 

        if (i2c_blocking_receive(&RANGEFINDER_I2C_PORT, &rangefinder_i2c.trans, rangefinder_i2c.addr, 2, 0.5)) {
          rangefinder_i2c.status = RANGEFINDER_I2C_PARSE_DATA;
        }
      }
      break;
    case RANGEFINDER_I2C_PARSE_DATA: {
      #if ( RANGEFINDER_I2C_READ_MODE_SINGLE == 0x00 )
        rangefinder_i2c.raw = (uint16_t)((rangefinder_i2c.trans.buf[0] << 8) | rangefinder_i2c.trans.buf[1]);
      #else
        rangefinder_i2c.raw = (uint16_t)((rangefinder_i2c.trans.buf[1] << 8) | rangefinder_i2c.trans.buf[2]);
      #endif
      
      // Time of when measurement was taken, not when ABI message was send, tiny delay can occur between those two events
      uint32_t now_ts = get_sys_time_usec();

      // Convert the raw value to meters, optionally filter and apply the offset
      rangefinder_i2c.distance = (((float)(rangefinder_i2c.raw)) * RANGEFINDER_I2C_SCALE);
#ifdef RANGEFINDER_I2C_USE_FILTER
      rangefinder_i2c.distance = update_median_filter_f(&rangefinder_i2c_filter, rangefinder_i2c.distance);
#endif

      if (rangefinder_i2c.distance <= (float)RANGEFINDER_I2C_MAX_RANGE) {  //Discard non reliable readings that are out of range
        if (rangefinder_i2c.distance < (float)RANGEFINDER_I2C_MIN_RANGE) { rangefinder_i2c.distance = (float)RANGEFINDER_I2C_MIN_RANGE; }

        // Compensate range measurement for body rotation
        #if RANGEFINDER_I2C_COMPENSATE_ROTATION    
        float phi = stateGetNedToBodyEulers_f()->phi;
        float theta = stateGetNedToBodyEulers_f()->theta;
        float gain = cosf(phi) * cosf(theta);
        rangefinder_i2c.distance = rangefinder_i2c.distance * gain;
        // Too much attitude difference from neutral, e.g. 50deg roll and 40deg pitch, or even negative upside-down for the sensor to be really useful in AGL perspective, set distance to NAN
        if (gain < 0.4f) {rangefinder_i2c.distance = NAN; } //The magic 0.4f is for regular ultrasonic sensors about maximum range
        #endif

        if (!isnan(rangefinder_i2c.distance)) { 
          rangefinder_i2c.distance = rangefinder_i2c.distance - (float)RANGEFINDER_I2C_OFFSET; // Must be applied after rotation compensation
          // Send AGL message
          // Only send valid AGL distance values and positive distances, negative distances do not make sense in AGL perspective as it would mean the aircraft is underground
          if (rangefinder_i2c.distance > 0.0f && rangefinder_i2c.update_agl) {
            AbiSendMsgAGL(AGL_RANGEFINDER_I2C_ID, now_ts, rangefinder_i2c.distance);
          } 
        }
      } else {
        // Out of range, set to NAN
        rangefinder_i2c.distance = NAN;
      }

      // Reset status as so to start reading new distance value again
      rangefinder_i2c.status = RANGEFINDER_I2C_REQ_DATA;
      break;
    }
    default:
      break;
  }

#else // SITL
  rangefinder_i2c.distance = stateGetPositionEnu_f()->z;
#endif // SITL
}

/**
 *
 * Option to send debug informative values over telemetry if you do not want sonar message in telemetry
 */
void rangefinder_i2c_report(void)
{
  DOWNLINK_SEND_RANGEFINDER(DefaultChannel, DefaultDevice, &rangefinder_i2c.addr, &rangefinder_i2c.raw, &rangefinder_i2c.distance);
}
