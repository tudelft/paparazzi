/*
 * Copyright (C) 2025 Justin Dubois
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

/** @file filters/complementary_filter.h
 *  @brief Implementation of complementary filters (first order, second order, and Butterworth variants) for sensor fusion and signal processing
 *
 */

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "std.h"
#include "filters/low_pass_filter.h"



struct FirstOrderComplementary
{
  struct FirstOrderLowPass x_lp_filter; // Low pass filter instance for high pass path
  struct FirstOrderLowPass y_lp_filter; // Low pass filter instance for low pass path
};

/** Initialize the second order complementary filter.
 *
 * @param filter Complementary filter struct
 * @param tau Time constant of the low-pass filter
 * @param Q Q factor of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_first_order_complementary(
    struct FirstOrderComplementary *filter,
    float tau, float sample_time,
    float value)
{
  init_first_order_low_pass(&filter->x_lp_filter, tau, sample_time, value);
  init_first_order_low_pass(&filter->y_lp_filter, tau, sample_time, value);
}

/** Update the second order complementary filter with new input values.
 *
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_first_order_complementary(
    struct FirstOrderComplementary *filter,
    float value_x, float value_y)
{
  float x_lp_output = update_first_order_low_pass(&filter->x_lp_filter, value_x);
  float y_lp_output = update_first_order_low_pass(&filter->y_lp_filter, value_y);
  return filter->x_lp_filter.last_in - x_lp_output + y_lp_output;
}

/** 
 * @brief Reset the first order complementary filter to a specific value.
 * 
 * @param filter Complementary filter struct
 * @param value Value to reset the filter to
 * @return The reset value
 */
static inline float reset_first_order_complementary(
    struct FirstOrderComplementary *filter,
    float value)
{
  reset_first_order_low_pass(&filter->x_lp_filter, value);
  reset_first_order_low_pass(&filter->y_lp_filter, value);
  return value;
}

/** Get current value of the second order complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_first_order_complementary(const struct FirstOrderComplementary *filter)
{
  float x_lp_output = get_first_order_low_pass(&filter->x_lp_filter);
  float y_lp_output = get_first_order_low_pass(&filter->y_lp_filter);
  return filter->x_lp_filter.last_in - x_lp_output + y_lp_output;
}


struct SecondOrderComplementary
{
  struct SecondOrderLowPass x_lp_filter; // Low pass filter instance for high pass path
  struct SecondOrderLowPass y_lp_filter; // Low pass filter instance for low pass path
};

/** Initialize the second order complementary filter.
 *
 * @param filter Complementary filter struct
 * @param tau Time constant of the low-pass filter
 * @param Q Q factor of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_second_order_complementary(
    struct SecondOrderComplementary *filter,
    float tau, float Q, float sample_time,
    float value)
{
  init_second_order_low_pass(&filter->x_lp_filter, tau, Q, sample_time, value);
  init_second_order_low_pass(&filter->y_lp_filter, tau, Q, sample_time, value);
}

/** Update the second order complementary filter with new input values.
 *
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_second_order_complementary(
    struct SecondOrderComplementary *filter,
    float value_x, float value_y)
{
  float x_lp_output = update_second_order_low_pass(&filter->x_lp_filter, value_x);
  float y_lp_output = update_second_order_low_pass(&filter->y_lp_filter, value_y);
  return filter->x_lp_filter.i[0] - x_lp_output + y_lp_output;
}

/**
 * @brief Reset the second order complementary filter to a specific value.
 * 
 * @param filter Complementary filter struct
 * @param value Value to reset the filter to
 * @return The reset value
 */
static inline float reset_second_order_complementary(
    struct SecondOrderComplementary *filter,
    float value)
{
  reset_second_order_low_pass(&filter->x_lp_filter, value);
  reset_second_order_low_pass(&filter->y_lp_filter, value);
  return value;
}
/** Get current value of the second order complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_second_order_complementary(const struct SecondOrderComplementary *filter)
{
  float x_lp_output = get_second_order_low_pass(&filter->x_lp_filter);
  float y_lp_output = get_second_order_low_pass(&filter->y_lp_filter);
  return filter->x_lp_filter.i[0] - x_lp_output + y_lp_output;
}

typedef struct SecondOrderComplementary Butterworth2Complementary;

/** Initialize the Butterworth 2nd order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @param tau Time constant of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_butterworth_2_complementary(
    Butterworth2Complementary *filter,
    float tau, float sample_time,
    float value)
{
  init_second_order_complementary(
      (struct SecondOrderComplementary *)filter,
      tau, 0.7071, sample_time,
      value);
}

/** Update the Butterworth 2nd order low-pass complementary filter with new input values.
 *
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_butterworth_2_complementary(Butterworth2Complementary *filter, float value_x, float value_y)
{
  return update_second_order_complementary((struct SecondOrderComplementary *)filter, value_x, value_y);
}

/** Get current value of the Butterworth 2nd order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_butterworth_2_complementary(const Butterworth2Complementary *filter)
{
  return get_second_order_complementary((const struct SecondOrderComplementary *)filter);
}

typedef struct
{
  Butterworth4LowPass x_lp_filter; // Low pass filter instance for high pass path
  Butterworth4LowPass y_lp_filter; // Low pass filter instance for low pass path
} Butterworth4Complementary;

/** Initialize the Butterworth 4th order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @param tau Time constant of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_butterworth_4_complementary(Butterworth4Complementary *filter, float tau, float sample_time, float value)
{
  init_butterworth_4_low_pass(&filter->x_lp_filter, tau, sample_time, value);
  init_butterworth_4_low_pass(&filter->y_lp_filter, tau, sample_time, value);
}

/** Update the Butterworth 4th order low-pass complementary filter with new input values.
 *
 * @param filter Complementary filter struct
 * @param value_x New input value from the high-pass path
 * @param value_y New input value from the low-pass path
 * @return New filtered output value
 */
static inline float update_butterworth_4_complementary(Butterworth4Complementary *filter, float value_x, float value_y)
{
  float x_lp_output = update_butterworth_4_low_pass(&filter->x_lp_filter, value_x);
  float y_lp_output = update_butterworth_4_low_pass(&filter->y_lp_filter, value_y);
  return filter->x_lp_filter.lp1.i[0] - x_lp_output + y_lp_output;
}

/** Get current value of the Butterworth 4th order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_butterworth_4_complementary(const Butterworth4Complementary *filter)
{
  float x_lp_output = get_butterworth_4_low_pass(&filter->x_lp_filter);
  float y_lp_output = get_butterworth_4_low_pass(&filter->y_lp_filter);
  return filter->x_lp_filter.lp1.i[0] - x_lp_output + y_lp_output;
}

#endif /* COMPLEMENTARY_FILTER_H */