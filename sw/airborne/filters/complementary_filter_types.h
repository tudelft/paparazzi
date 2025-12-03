#ifndef COMPLEMENTARY_FILTER_TYPES_H
#define COMPLEMENTARY_FILTER_TYPES_H

#include "std.h"
#include "filters/complementary_filter.h"
#include "math/pprz_algebra_float.h"


/**
 * @brief Initialize a 1st order complementary filter.
 *
 * @param[out] filter Pointer to the FirstOrderComplementary filter instance.
 * @param[in] freq Cutoff frequency of the filter in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_first_order_complementary(struct FirstOrderComplementary *filter, float freq, float sample_time)
{
  init_first_order_low_pass_complementary(filter, 1.0f / freq, sample_time, 0.0f);
}

/**
 * @brief Update the 1st order complementary filter with new input values.
 *
 * @param[in,out] filter Pointer to the FirstOrderComplementary filter instance.
 * @param[in] value_x Input value x for the filter update.
 * @param[in] value_y Input value y for the filter update.
 */
static inline void update_first_order_complementary(struct FirstOrderComplementary *filter, float value_x, float value_y)
{
  update_first_order_low_pass_complementary(filter, value_x, value_y);
}

/**
 * @brief Get the current output of the 1st order complementary filter.
 *
 * @param[in] filter Pointer to the FirstOrderComplementary filter instance.
 * @return Current filtered output value.
 */
static inline float get_first_order_complementary(const struct FirstOrderComplementary *filter)
{
  return get_first_order_low_pass_complementary(filter);
}

struct FirstOrderComplementaryVect3
{
  struct FirstOrderComplementary x;
  struct FirstOrderComplementary y;
  struct FirstOrderComplementary z;
};

/**
 * @brief Initialize 3D vector of 1st order complementary filters.
 *
 * @param[out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_first_order_complementary_vect3(struct FirstOrderComplementaryVect3 *filter, float freq, float sample_time)
{
  init_first_order_low_pass_complementary(&filter->x, 1.0f / freq, sample_time, 0.0f);
  init_first_order_low_pass_complementary(&filter->y, 1.0f / freq, sample_time, 0.0f);
  init_first_order_low_pass_complementary(&filter->z, 1.0f / freq, sample_time, 0.0f);
}

/**
 * @brief Update 3D vector of 1st order complementary filters.
 *
 * @param[in,out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatVect3 input vector x values.
 * @param[in] value_y Pointer to FloatVect3 input vector y values.
 */
static inline void update_first_order_complementary_vect3(struct FirstOrderComplementaryVect3 *filter, const struct FloatVect3 *value_x, const struct FloatVect3 *value_y)
{
  update_first_order_low_pass_complementary(&filter->x, value_x->x, value_y->x);
  update_first_order_low_pass_complementary(&filter->y, value_x->y, value_y->y);
  update_first_order_low_pass_complementary(&filter->z, value_x->z, value_y->z);
}

/**
 * @brief Update 3D vector of 1st order complementary filters for rates.
 *
 * @param[in,out] filter Pointer to the FirstOrderComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatRates input p, q, r values.
 * @param[in] value_y Pointer to FloatRates input p, q, r values.
 */
static inline void update_first_order_complementary_rates(struct FirstOrderComplementaryVect3 *filter, const struct FloatRates *value_x, const struct FloatRates *value_y)
{
  update_first_order_low_pass_complementary(&filter->x, value_x->p, value_y->p);
  update_first_order_low_pass_complementary(&filter->y, value_x->q, value_y->q);
  update_first_order_low_pass_complementary(&filter->z, value_x->r, value_y->r);
}

/**
 * @brief Get current output 3D vector from 1st order complementary filters.
 *
 * @param[in] filter Pointer to FirstOrderComplementaryVect3 struct.
 * @return FloatVect3 containing filtered outputs.
 */
static inline struct FloatVect3 get_first_order_complementary_vect3(const struct FirstOrderComplementaryVect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_first_order_low_pass_complementary(&filter->x);
  output.y = get_first_order_low_pass_complementary(&filter->y);
  output.z = get_first_order_low_pass_complementary(&filter->z);
  return output;
}

/**
 * @brief Get current output rates from 1st order complementary filters.
 *
 * @param[in] filter Pointer to FirstOrderComplementaryVect3 struct.
 * @return FloatRates containing filtered outputs p, q, r.
 */
static inline struct FloatRates get_first_order_complementary_rates(const struct FirstOrderComplementaryVect3 *filter)
{
  struct FloatRates output;
  output.p = get_first_order_low_pass_complementary(&filter->x);
  output.q = get_first_order_low_pass_complementary(&filter->y);
  output.r = get_first_order_low_pass_complementary(&filter->z);
  return output;
}

/**
 * @brief Initialize an array of 1st order complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of FirstOrderComplementary filters.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_first_order_complementary_array(uint8_t n, struct FirstOrderComplementary filter_array[restrict n], float freq, float sample_time)
{
  float tau = 1.0f / freq;
  for (uint8_t i = 0; i < n; i++)
  {
    init_first_order_low_pass_complementary(&filter_array[i], tau, sample_time, 0.0f);
  }
}

/**
 * @brief Update an array of 1st order complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of FirstOrderComplementary filters.
 * @param[in] value_x_array Array of input x values.
 * @param[in] value_y_array Array of input y values.
 */
static inline void update_first_order_complementary_array(uint8_t n, struct FirstOrderComplementary filter_array[restrict n], const float value_x_array[restrict n], const float value_y_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_first_order_low_pass_complementary(&filter_array[i], value_x_array[i], value_y_array[i]);
  }
}

/**
 * @brief Get current outputs from an array of 1st order complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of FirstOrderComplementary filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_first_order_complementary_array(uint8_t n, const struct FirstOrderComplementary filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_first_order_low_pass_complementary(&filter_array[i]);
  }
}

/**
 * @brief Initialize a 2nd order Butterworth low-pass complementary filter.
 *
 * @param[out] filter Pointer to the Butterworth2Complementary filter instance.
 * @param[in] freq Cutoff frequency of the filter in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_2_complementary(Butterworth2Complementary *filter, float freq, float sample_time)
{
  init_butterworth_2_low_pass_complementary(filter, 1.0f / freq, sample_time, 0.0f);
}

/**
 * @brief Update the 2nd order Butterworth complementary filter with new input values.
 *
 * @param[in,out] filter Pointer to the Butterworth2Complementary filter instance.
 * @param[in] value_x Input value x for the filter update.
 * @param[in] value_y Input value y for the filter update.
 */
static inline void update_butterworth_2_complementary(Butterworth2Complementary *filter, float value_x, float value_y)
{
  update_butterworth_2_low_pass_complementary(filter, value_x, value_y);
}

/**
 * @brief Get the current output of the 2nd order Butterworth complementary filter.
 *
 * @param[in] filter Pointer to the Butterworth2Complementary filter instance.
 * @return Current filtered output value.
 */

static inline float get_butterworth_2_complementary(const Butterworth2Complementary *filter)
{
  return get_butterworth_2_low_pass_complementary(filter);
}

/**
 * @brief Initialize a 4th order Butterworth low-pass complementary filter.
 *
 * @param[out] filter Pointer to the Butterworth4Complementary filter instance.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_4_complementary(Butterworth4Complementary *filter, float freq, float sample_time)
{
  init_butterworth_4_low_pass_complementary(filter, 1.0f / freq, sample_time, 0.0f);
}

/**
 * @brief Update the 4th order Butterworth complementary filter with new input values.
 *
 * @param[in,out] filter Pointer to the Butterworth4Complementary filter instance.
 * @param[in] value_x Input value x for the filter update.
 * @param[in] value_y Input value y for the filter update.
 */
static inline void update_butterworth_4_complementary(Butterworth4Complementary *filter, float value_x, float value_y)
{
  update_butterworth_4_low_pass_complementary(filter, value_x, value_y);
}

/**
 * @brief Get the current output of the 4th order Butterworth complementary filter.
 *
 * @param[in] filter Pointer to the Butterworth4Complementary filter instance.
 * @return Current filtered output value.
 */
static inline float get_butterworth_4_complementary(const Butterworth4Complementary *filter)
{
  return get_butterworth_4_low_pass_complementary(filter);
}


struct Butterworth2ComplementaryVect3
{
  Butterworth2Complementary x;
  Butterworth2Complementary y;
  Butterworth2Complementary z;
};

/**
 * @brief Initialize 3D vector of 2nd order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_2_complementary_vect3(struct Butterworth2ComplementaryVect3 *filter, float freq, float sample_time)
{
  init_butterworth_2_low_pass_complementary(&filter->x, 1.0f / freq, sample_time, 0.0f);
  init_butterworth_2_low_pass_complementary(&filter->y, 1.0f / freq, sample_time, 0.0f);
  init_butterworth_2_low_pass_complementary(&filter->z, 1.0f / freq, sample_time, 0.0f);
}

/**
 * @brief Initialize 3D vector of 2nd order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void update_butterworth_2_complementary_vect3(struct Butterworth2ComplementaryVect3 *filter, const struct FloatVect3 *value_x, const struct FloatVect3 *value_y)
{
  update_butterworth_2_low_pass_complementary(&filter->x, value_x->x, value_y->x);
  update_butterworth_2_low_pass_complementary(&filter->y, value_x->y, value_y->y);
  update_butterworth_2_low_pass_complementary(&filter->z, value_x->z, value_y->z);
}

/**
 * @brief Update 3D vector of 2nd order Butterworth complementary filters.
 *
 * @param[in,out] filter Pointer to the Butterworth2ComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatVect3 input vector x values.
 * @param[in] value_y Pointer to FloatVect3 input vector y values.
 */
static inline void update_butterworth_2_complementary_rates(struct Butterworth2ComplementaryVect3 *filter, const struct FloatRates *value_x, const struct FloatRates *value_y)
{
  update_butterworth_2_low_pass_complementary(&filter->x, value_x->p, value_y->p);
  update_butterworth_2_low_pass_complementary(&filter->y, value_x->q, value_y->q);
  update_butterworth_2_low_pass_complementary(&filter->z, value_x->r, value_y->r);
}

/**
 * @brief Get current output 3D vector from 2nd order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth2ComplementaryVect3 struct.
 * @return FloatVect3 containing filtered outputs.
 */
static inline struct FloatVect3 get_butterworth_2_complementary_vect3(const struct Butterworth2ComplementaryVect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_butterworth_2_low_pass_complementary(&filter->x);
  output.y = get_butterworth_2_low_pass_complementary(&filter->y);
  output.z = get_butterworth_2_low_pass_complementary(&filter->z);
  return output;
}

/**
 * @brief Get current output rates from 2nd order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth2ComplementaryVect3 struct.
 * @return FloatRates containing filtered outputs p, q, r.
 */
static inline struct FloatRates get_butterworth_2_complementary_rates(const struct Butterworth2ComplementaryVect3 *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_2_low_pass_complementary(&filter->x);
  output.q = get_butterworth_2_low_pass_complementary(&filter->y);
  output.r = get_butterworth_2_low_pass_complementary(&filter->z);
  return output;
}

/**
 * @brief Initialize an array of 2nd order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2Complementary filters.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_2_complementary_array(uint8_t n, Butterworth2Complementary filter_array[restrict n], float freq, float sample_time)
{
  float tau = 1.0f / freq;
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_2_low_pass_complementary(&filter_array[i], tau, sample_time, 0.0f);
  }
}

/**
 * @brief Update an array of 2nd order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth2Complementary filters.
 * @param[in] value_x_array Array of input x values.
 * @param[in] value_y_array Array of input y values.
 */
static inline void update_butterworth_2_complementary_array(uint8_t n, Butterworth2Complementary filter_array[restrict n], const float value_x_array[restrict n], const float value_y_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_butterworth_2_low_pass_complementary(&filter_array[i], value_x_array[i], value_y_array[i]);
  }
}
/**
 * @brief Get current outputs from an array of 2nd order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth2Complementary filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_2_complementary_array(uint8_t n, const Butterworth2Complementary filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_butterworth_2_low_pass_complementary(&filter_array[i]);
  }
}


struct Butterworth4ComplementaryVect3
{
  Butterworth4Complementary x;
  Butterworth4Complementary y;
  Butterworth4Complementary z;
};

/**
 * @brief Initialize 3D vector of 4th order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void init_butterworth_4_complementary_vect3(struct Butterworth4ComplementaryVect3 *filter, float freq, float sample_time)
{
  init_butterworth_4_low_pass_complementary(&filter->x, 1.0f / freq, sample_time, 0.0f);
  init_butterworth_4_low_pass_complementary(&filter->y, 1.0f / freq, sample_time, 0.0f);
  init_butterworth_4_low_pass_complementary(&filter->z, 1.0f / freq, sample_time, 0.0f);
}

/**
 * @brief Initialize 3D vector of 4th order Butterworth complementary filters.
 *
 * @param[out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] freq Cutoff frequency in rad/s.
 * @param[in] sample_time Sampling period in seconds.
 */
static inline void update_butterworth_4_complementary_vect3(struct Butterworth4ComplementaryVect3 *filter, const struct FloatVect3 *value_x, const struct FloatVect3 *value_y)
{
  update_butterworth_4_low_pass_complementary(&filter->x, value_x->x, value_y->x);
  update_butterworth_4_low_pass_complementary(&filter->y, value_x->y, value_y->y);
  update_butterworth_4_low_pass_complementary(&filter->z, value_x->z, value_y->z);
}

/**
 * @brief Update 3D vector of 4th order Butterworth complementary filters for rates.
 *
 * @param[in,out] filter Pointer to the Butterworth4ComplementaryVect3 struct.
 * @param[in] value_x Pointer to FloatRates input p, q, r values.
 * @param[in] value_y Pointer to FloatRates input p, q, r values.
 */
static inline void update_butterworth_4_complementary_rates(struct Butterworth4ComplementaryVect3 *filter, const struct FloatRates *value_x, const struct FloatRates *value_y)
{
  update_butterworth_4_low_pass_complementary(&filter->x, value_x->p, value_y->p);
  update_butterworth_4_low_pass_complementary(&filter->y, value_x->q, value_y->q);
  update_butterworth_4_low_pass_complementary(&filter->z, value_x->r, value_y->r);
}

/**
 * @brief Get current output 3D vector from 4th order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth4ComplementaryVect3 struct.
 * @return FloatVect3 containing filtered outputs.
 */
static inline struct FloatVect3 get_butterworth_4_complementary_vect3(const struct Butterworth4ComplementaryVect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_butterworth_4_low_pass_complementary(&filter->x);
  output.y = get_butterworth_4_low_pass_complementary(&filter->y);
  output.z = get_butterworth_4_low_pass_complementary(&filter->z);
  return output;
}

/**
 * @brief Get current output rates from 4th order Butterworth complementary filters.
 *
 * @param[in] filter Pointer to Butterworth4ComplementaryVect3 struct.
 * @return FloatRates containing filtered outputs p, q, r.
 */
static inline struct FloatRates get_butterworth_4_complementary_rates(const struct Butterworth4ComplementaryVect3 *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_4_low_pass_complementary(&filter->x);
  output.q = get_butterworth_4_low_pass_complementary(&filter->y);
  output.r = get_butterworth_4_low_pass_complementary(&filter->z);
  return output;
}

/**
 * @brief Initialize an array of 4th order Butterworth complementary filters.
 *
 * @param[in] n Number of filters to initialize.
 * @param[out] filter_array Array of Butterworth4Complementary filters.
 * @param[in] freq Cutoff frequency for the filters (rad/s).
 * @param[in] sample_time Sampling period (seconds).
 */
static inline void init_butterworth_4_complementary_array(uint8_t n, Butterworth4Complementary filter_array[restrict n], float freq, float sample_time)
{
  float tau = 1.0f / freq;
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_4_low_pass_complementary(&filter_array[i], tau, sample_time, 0.0f);
  }
}

/**
 * @brief Update an array of 4th order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth4Complementary filters.
 * @param[in] value_x_array Array of input x values.
 * @param[in] value_y_array Array of input y values.
 */
static inline void update_butterworth_4_complementary_array(uint8_t n, Butterworth4Complementary filter_array[restrict n], const float value_x_array[restrict n], const float value_y_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_butterworth_4_low_pass_complementary(&filter_array[i], value_x_array[i], value_y_array[i]);
  }
}

/**
 * @brief Get current outputs from an array of 4th order Butterworth complementary filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth4Complementary filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_4_complementary_array(uint8_t n, const Butterworth4Complementary filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_butterworth_4_low_pass_complementary(&filter_array[i]);
  }
}

#endif /* COMPLEMENTARY_FILTER_TYPES_H */