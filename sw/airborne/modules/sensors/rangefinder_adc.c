/*
 * Copyright (C) 2010  Gautier Hattenberger, 2013 Tobias Münch, 2025 OpenUAS
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

/** @file modules/sensors/rangefinder_adc.h
*  @brief Driver for an analog type of rangfinder sensor which provides distance measurements when used via an ADC channel
*/

#include "generated/airframe.h"
#include "modules/sensors/rangefinder_adc.h"
#include "modules/core/abi.h"
#if defined(SITL) || (defined(RANGEFINDER_ADC_COMPENSATE_ROTATION) && (RANGEFINDER_ADC_COMPENSATE_ROTATION == 1))
#include "state.h"
#endif
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif

#if defined(RANGEFINDER_ADC_SYNC_SEND) && (!defined(RANGEFINDER_ADC_SYNC_SEND))
#define RANGEFINDER_ADC_SYNC_SEND 1
#endif

#ifdef RANGEFINDER_ADC_SYNC_SEND
#include "modules/datalink/downlink.h"
#endif

// Check if an ADC device port is in the airframe configuration
#ifndef RANGEFINDER_ADC_PORT
#error  rangefinder_adc module error: please add and configure RANGEFINDER_ADC_PORT, the port your sensor is connected to e.g. ADC_3 and save in your airframe file.
#endif
PRINT_CONFIG_VAR(RANGEFINDER_ADC_PORT)

#include "mcu_periph/adc.h"

// The minimum and maximum range what we want in our output e.g. sensor can do 7.2m but we only want to get less than 4m set MAX_RANGE to 4.0

/// The minimum chosen distance for the device to be give readings
#ifndef RANGEFINDER_ADC_MIN_RANGE
#define RANGEFINDER_ADC_MIN_RANGE 0.24f //Default is 0.24m since many older analog sensors cannot measure a range closer than that reliably
#endif

/// The maximum chosen distance for the device to be give readings
#ifndef RANGEFINDER_ADC_MAX_RANGE
#define RANGEFINDER_ADC_MAX_RANGE 4.0f //Reasonable default value in meters with still usable readings for it is maximum value for common analog range sensors
#endif

// Just in the rare case the raw ADC value has a deviation on neutral readings. Do not use this for AGL to ground offsets, this is only for the raw ADC value to be correct
#ifndef RANGEFINDER_ADC_RAW_OFFSET
#define RANGEFINDER_ADC_RAW_OFFSET 0  ///< Analog sensor value offset in ADC units
#endif

/// Rangefinder distance offset value for what should be considered zero distance, e.g. high landing gear of 1.1m to tarmac still could be considered zero
/// This is NOT the same as the RAW ADC value offset
#ifndef RANGEFINDER_ADC_OFFSET
#define RANGEFINDER_ADC_OFFSET 0.0f // in meters, default is 0.0f
#endif

/// Filter the raw measuread sonar data
#ifndef RANGEFINDER_ADC_USE_FILTER
#define RANGEFINDER_ADC_USE_FILTER 1 // Enable filtering for sonar per default is best, sonars tend to give noisy spiking readings
#endif

/// Option to send AGL data over ABI
#ifndef USE_RANGEFINDER_ADC_AGL
#define USE_RANGEFINDER_ADC_AGL 0
#endif

#ifdef RANGEFINDER_ADC_USE_FILTER
#include "filters/low_pass_filter.h"
#ifndef RANGEFINDER_ADC_LOWPASS_TAU
#define RANGEFINDER_ADC_LOWPASS_TAU 0.16 // Time constant for second order Butterworth low pass filter. Default of 0.16 should give cut-off freq of 1/(2*pi*tau) ~= 1Hz
#endif
static Butterworth2LowPass rangefinder_filt;
#endif

// Distance compensation if the aircraft is in large rolled or pitched state to get the real distance to the ground
#ifndef RANGEFINDER_ADC_COMPENSATE_ROTATION
#define RANGEFINDER_ADC_COMPENSATE_ROTATION 0
#endif

/** Scale
 *  Sensor sensitivity in m/adc (float)
 */
#ifndef RANGEFINDER_ADC_SCALE
#define RANGEFINDER_ADC_SCALE 0.001855f // Default value for e.g. a ultrasonic sensor scale like a Maxbotix MB1340 LV-EZ1
#endif

struct RangefinderADC rangefinder_adc;

#ifndef SITL
static struct adc_buf rangefinder_adc_buf;
#endif

/** Sending
 * Send measured value and status information so it can be read back in e.g. log file for debugging
 */
static void rangefinder_adc_send_range(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t nul = 0;
  pprz_msg_send_RANGEFINDER(trans, dev, AC_ID, &nul, &rangefinder_adc.raw, &rangefinder_adc.distance);
}
void rangefinder_adc_init(void)
{
  rangefinder_adc.raw = 0;//RANGEFINDER_ADC_RAW_OFFSET;
  rangefinder_adc.scale = RANGEFINDER_ADC_SCALE; // Added to struct as to be able to change the scale live via setting GUI for easier tuning
  rangefinder_adc.distance = 0.0;
  rangefinder_adc.update_agl = RANGEFINDER_ADC_USE_FOR_AGL; // Can be switched live if AGL should be updated or not

#ifdef RANGEFINDER_ADC_USE_FILTER
  init_butterworth_2_low_pass(&rangefinder_filt, RANGEFINDER_ADC_LOWPASS_TAU, RANGEFINDER_ADC_PERIODIC_PERIOD, rangefinder_adc.raw);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RANGEFINDER, rangefinder_adc_send_range);
#endif

#ifndef SITL
  adc_buf_channel(RANGEFINDER_ADC_PORT, &rangefinder_adc_buf, DEFAULT_AV_NB_SAMPLE);
#endif
}

/** Reading
 * Read ADC value to update range measurement
 */
void rangefinder_adc_periodic(void)
{
#ifndef SITL

  // Here the spot where the sonar ADC value is read
  rangefinder_adc.raw = (rangefinder_adc_buf.sum / rangefinder_adc_buf.av_nb_sample) -
                  RANGEFINDER_ADC_RAW_OFFSET; // Get the average value from the ADC buffer and apply the offset to it

  // Time of when measurement was taken, not when ABI message was send, to be as close as possible to real measurement time data
  uint32_t now_ts = get_sys_time_usec();

#ifdef RANGEFINDER_ADC_USE_FILTER
  rangefinder_adc.distance = update_butterworth_2_low_pass(&rangefinder_filt, (float)(rangefinder_adc.raw) * rangefinder_adc.scale);
#else
  rangefinder_adc.distance = (float)rangefinder_adc.raw * rangefinder_adc.scale; // Convert the raw ADC value to a distance in meters
#endif

  if (rangefinder_adc.distance <= (float)RANGEFINDER_ADC_MAX_RANGE) {  // Discard non reliable readings that are out of range
    if (rangefinder_adc.distance < (float)RANGEFINDER_ADC_MIN_RANGE) { rangefinder_adc.distance = (float)RANGEFINDER_ADC_MIN_RANGE; }
    // Compensate range measurement for body rotation
#if RANGEFINDER_ADC_COMPENSATE_ROTATION
    float phi = stateGetNedToBodyEulers_f()->phi;
    float theta = stateGetNedToBodyEulers_f()->theta;
    float gain = cosf(phi) * cosf(theta);
    rangefinder_adc.distance = rangefinder_adc.distance * gain;
    // To much attitude difference from neutral, e.g. 50deg roll and 40deg pitch, or even negative upside-down for the sensor to be really useful in AGL perspective, set distance to NAN
    if (gain < 0.4f) { rangefinder_adc.distance = NAN; } // The magic 0.4f is for regular ultrasonic sensors about maximum range
#endif

    if (!isnan(rangefinder_adc.distance)) {
      rangefinder_adc.distance = rangefinder_adc.distance - (float)RANGEFINDER_ADC_OFFSET; // Must be applied after rotation compensation
      // Note that one could get negative values here if offset is larger than measured distance. This is valid and means the sensor is closer to ground than what is considered zero
      // Mostly negative number behaviour is not wanted, so now bound to zero. Note that one looses the information that the sensor is actually closer than what is considered zero
      Bound(rangefinder_adc.distance, 0.0f, (RANGEFINDER_ADC_MAX_RANGE - RANGEFINDER_ADC_OFFSET));
      // Send AGL message
      // Only send valid AGL distance values and positive distances, negative distances do not make sense in AGL perspective as it would mean the aircraft is underground
      if (rangefinder_adc.update_agl) {
        AbiSendMsgAGL(AGL_RANGEFINDER_ADC_ID, now_ts, rangefinder_adc.distance);
      }
    }
  } else {
    // Out of range, set to NAN
    rangefinder_adc.distance = NAN;
  }

#else // SITL
  rangefinder_adc.distance = stateGetPositionEnu_f()->z;
  Bound(rangefinder_adc.distance, 0.0f,
        (float)RANGEFINDER_ADC_MAX_RANGE); // Sim should also use airframe defined limits since can interact with AGL in simmed flightplan
#endif // SITL

// Fast data output for debugging purposes, one can it have temporary enabled to see what the sensor give as output in detail
#if RANGEFINDER_ADC_SYNC_SEND
  rangefinder_adc_report();
#endif

}

/** Debug report
 * Option to send debug informative values over telemetry
 */
void rangefinder_adc_report(void)
{
#if RANGEFINDER_ADC_SYNC_SEND
  uint8_t id = 0;//RANGEFINDER_ADC_PORT;
  DOWNLINK_SEND_RANGEFINDER(DefaultChannel, DefaultDevice, &id, &rangefinder_adc.raw, &rangefinder_adc.distance);
#endif
}

