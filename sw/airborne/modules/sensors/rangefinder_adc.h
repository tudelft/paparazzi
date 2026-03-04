/*
 * Copyright (C) 2010  Gautier Hattenberger
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

/**
 * @file rangefinder_adc.h
 * @brief Driver module for an analog rangefinder sensor connected to an ADC port of the flightcontroller.
 *
 * This driver module reads the ADC values from a rangefinder sensor and converts those values to a distance in meters or fractions thereof.
 *
 * Options include:
 * - Using a low-pass filter to smooth the distance output
 * - Updating AGL (Above Ground Level) in the state with the distance value.
 * - Rotation compensation, which compensates the AGL distance based on the current attitude of the aircraft.
 * - Use sensor simulation SITL (Software In The Loop).
 * - Optionally sending periodic telemetry debug messages with the raw ADC value and the calculated distance.
 */

#ifndef RANGEFINDER_ADC_H
#define RANGEFINDER_ADC_H

#include "std.h"
struct RangefinderADC {
  uint16_t raw;    ///< raw measuread non scaled range value from sensor
  float scale;     ///< Scaling factor to convert raw value to a distance in SI unit (meters)
  float distance;  ///< Distance measured in meters
  bool update_agl; ///< Do or don't update AGL ABI message
};

extern struct RangefinderADC rangefinder_adc;

extern void rangefinder_adc_init(void);
extern void rangefinder_adc_periodic(void);
extern void rangefinder_adc_report(void);
#endif  /* RANGEFINDER_ADC_H */
