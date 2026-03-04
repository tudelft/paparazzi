/*
* Copyright (C) 2020 OpenuAS
*
* Thanks to Jean-François Erdelyi & Gautier Hattenberger for ADC one
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

/**
* @file "modules/sensors/rangefinder_pwm.h"
* @author OpenUAS
* @brief Driver for Rangefinder sensors that output a PWM signal
*
* Reads a sensor using PWM input and outputs distance to object in meters
*
* Sensor example: Maxbotix EZ1
* https://www.maxbotix.com/033-using-pulse-width-pin-2.htm
*
*/

#include "modules/sensors/rangefinder_pwm.h"
#include "mcu_periph/pwm_input.h"
#include "modules/core/abi.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include "pprzlink/messages.h"
#endif
#ifdef RANGEFINDER_PWM_SYNC_SEND
#include "modules/datalink/downlink.h"
#endif
#include "generated/airframe.h"
#ifdef SITL
#include "state.h"
#endif

#ifdef RANGEFINDER_PWM_USE_FILTER
#include "filters/median_filter.h"
struct MedianFilterFloat rangefinder_pwm_filt;
#endif

/* Config parameters for rangefinder_pwm sensor */

/// The input port of the PWM based sensor
#ifndef RANGEFINDER_PWM_PORT
#error "No RANGEFINDER_PWM_PORT defined to use rangefinder_pwm module, add a line like define=RANGEFINDER_PWM_PORT value=PWM_INPUT1"
#endif

/// Rangefinder offset value for what considered the distance should be zero to the object, e.g. tarmac when landing with high landing gear
#ifndef RANGEFINDER_PWM_OFFSET
#define RANGEFINDER_PWM_OFFSET 0
#endif

/// Default to a 12 bit PWM sensor
#ifndef RANGEFINDER_PWM_PWM_PERIOD
#define RANGEFINDER_PWM_PWM_PERIOD 4096
#endif

/// Rangefinder scale or sensitivity as you wish
#ifndef RANGEFINDER_PWM_SCALE
#define RANGEFINDER_PWM_SCALE 0.001f //TODO: in relation to RANGEFINDER_PWM_PWM_PERIOD
#endif

/// The Median Filter strength
#ifndef RANGEFINDER_PWM_MEDIAN_SIZE
#define RANGEFINDER_PWM_MEDIAN_SIZE 7 //Good default for a noisy ultrasonic type of rangefinder
#endif

/// The minimum range for the device to be able to measure
#ifndef RANGEFINDER_PWM_MIN_RANGE
#define RANGEFINDER_PWM_MIN_RANGE 0.15f //Default is a common value in meters for regular PWM type rangefinders. Most cannot measure closer than that
#endif

/// The maximum range for the device to be able to measure
#ifndef RANGEFINDER_PWM_MAX_RANGE
#define RANGEFINDER_PWM_MAX_RANGE 7.0f //Reasonable maximum value in meters for regular rangefinders.
#endif

/// Some PWM type of sensor may need an initial PWM offset (823 usec in the case of an EZ1 sensor)
#ifndef RANGEFINDER_PWM_PWM_OFFSET // Yes, correct naming it is PWM offset for the Rangefinder PWM sensor
#define RANGEFINDER_PWM_PWM_OFFSET 820
#endif

// Enable telemetry report
#ifndef RANGEFINDER_PWM_SYNC_SEND
#define RANGEFINDER_PWM_SYNC_SEND 1
#endif

struct RangefinderPwm rangefinder_pwm;

/* Set the default values at initialization */
void rangefinder_pwm_init(void)
{
  rangefinder_pwm.offset = RANGEFINDER_PWM_OFFSET; //Offset in meters
  rangefinder_pwm.scale = RANGEFINDER_PWM_SCALE;
  rangefinder_pwm.raw = 0.0f;
  #ifdef RANGEFINDER_PWM_USE_FILTER
    init_median_filter_f(&rangefinder_pwm_filt, RANGEFINDER_PWM_MEDIAN_SIZE);
  #endif
}

/* Read the sensor current measured value */
void rangefinder_pwm_read(void) {

#ifndef SITL
  // raw duty cycle in usec
  uint16_t rangefinder_pwm_duty_raw = get_pwm_input_duty_in_usec(RANGEFINDER_PWM_PORT);

  // remove PWM offset where needed
  rangefinder_pwm.raw = rangefinder_pwm_duty_raw - RANGEFINDER_PWM_PWM_OFFSET;

#ifdef RANGEFINDER_PWM_USE_FILTER
  rangefinder_pwm.distance = update_median_filter_f(&rangefinder_pwm_filt, ((float)rangefinder_pwm.raw * rangefinder_pwm.scale) - rangefinder_pwm.offset);
#else
  rangefinder_pwm.distance = (((float)rangefinder_pwm.raw * rangefinder_pwm.scale) - rangefinder_pwm.offset);
#endif

  Bound(rangefinder_pwm.distance, (float)RANGEFINDER_PWM_MIN_RANGE, (float)RANGEFINDER_PWM_MAX_RANGE);

#if RANGEFINDER_PWM_COMPENSATE_ROTATION
  float phi = stateGetNedToBodyEulers_f()->phi;
  float theta = stateGetNedToBodyEulers_f()->theta;
  float gain = (float)fabs( (double) (cosf(phi) * cosf(theta)));
  rangefinder_pwm.distance =  rangefinder_pwm.distance * gain;
#endif

#else // SITL
  rangefinder_pwm.distance = stateGetPositionEnu_f()->z;
#endif // SITL

#if USE_SONAR || USE_RANGEFINDER_PWM //USE_SONAR is defined for backward compatibility
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgAGL(AGL_RANGEFINDER_PWM_ID, now_ts, rangefinder_pwm.distance);
#endif

#ifdef RANGEFINDER_PWM_SYNC_SEND
  uint8_t id = AGL_RANGEFINDER_PWM_ID;
  DOWNLINK_SEND_RANGEFINDER(DefaultChannel, DefaultDevice, &id, &rangefinder_pwm.raw, &rangefinder_pwm.distance);
#endif
}
