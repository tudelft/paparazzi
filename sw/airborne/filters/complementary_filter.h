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
  float time_const;
  float x_last_in;
  float y_last_in;
  float last_out;
};

/** Init first order complementary filter.
 *
 * @param filter first order complementary filter structure
 * @param tau time constant of the complementary filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
static inline void init_first_order_complementary(struct FirstOrderComplementary *filter, float tau,
                                                  float sample_time, float value)
{
  filter->x_last_in = value;
  filter->y_last_in = value;
  filter->last_out = value;
  filter->time_const = 2.0f * tau / sample_time;
}

/** Update first order complementary filter state with a new value.
 *
 * @param filter first order complementary filter structure
 * @param value_x new input value of the filter from the high pass path
 * @param value_y new input value of the filter from the low pass path
 * @return new filtered value
 */
static inline float update_first_order_complementary(struct FirstOrderComplementary *filter, float value_x, float value_y)
{
  float out = (value_x + filter->x_last_in + filter->time_const * (value_y - filter->y_last_in) + (filter->time_const - 1.0f) * filter->last_out) / (filter->time_const + 1.0f);

  filter->x_last_in = value_x;
  filter->y_last_in = value_y;
  filter->last_out = out;

  return out;
}

/** Get current value of the first order complementary filter.
 *
 * @param filter first order complementary filter structure
 * @return current value of the filter
 */
static inline float get_first_order_complementary(const struct FirstOrderComplementary *filter)
{
  return filter->last_out;
}

/** Update time constant of the first order complementary filter.
 *
 * @param filter first order complementary filter structure
 * @param tau new time constant of the complementary filter
 * @param sample_time sampling period of the signal
 */
static inline void update_first_order_complementary_tau(struct FirstOrderComplementary *filter, float tau, float sample_time)
{
  filter->time_const = 2.0f * tau / sample_time;
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

typedef struct SecondOrderComplementary Butterworth2LowPassComplementary;

/** Initialize the Butterworth 2nd order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @param tau Time constant of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_butterworth_2_low_pass_complementary(
    Butterworth2LowPassComplementary *filter,
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
static inline float update_butterworth_2_low_pass_complementary(Butterworth2LowPassComplementary *filter, float value_x, float value_y)
{
  return update_second_order_complementary((struct SecondOrderComplementary *)filter, value_x, value_y);
}

/** Get current value of the Butterworth 2nd order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @return Current output value of the filter
 */
static inline float get_butterworth_2_low_pass_complementary(const Butterworth2LowPassComplementary *filter)
{
  return get_second_order_complementary((const struct SecondOrderComplementary *)filter);
}

typedef struct
{
  Butterworth4LowPass x_lp_filter; // Low pass filter instance for high pass path
  Butterworth4LowPass y_lp_filter; // Low pass filter instance for low pass path
} Butterworth4LowPassComplementary;

/** Initialize the Butterworth 4th order low-pass complementary filter.
 *
 * @param filter Complementary filter struct
 * @param tau Time constant of the low-pass filter
 * @param sample_time Sampling period
 * @param value Initial value for filter history
 */
static inline void init_butterworth_4_low_pass_complementary(Butterworth4LowPassComplementary *filter, float tau, float sample_time, float value)
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
static inline float update_butterworth_4_low_pass_complementary(Butterworth4LowPassComplementary *filter, float value_x, float value_y)
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
static inline float get_butterworth_4_low_pass_complementary(const Butterworth4LowPassComplementary *filter)
{
  float x_lp_output = get_butterworth_4_low_pass(&filter->x_lp_filter);
  float y_lp_output = get_butterworth_4_low_pass(&filter->y_lp_filter);
  return filter->x_lp_filter.lp1.i[0] - x_lp_output + y_lp_output;
}

#endif /* COMPLEMENTARY_FILTER_H */