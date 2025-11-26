#ifndef LOW_PASS_FILTER_TYPES_H
#define LOW_PASS_FILTER_TYPES_H

#include "std.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra_float.h"


struct Butterworth2Vect3
{
  Butterworth2LowPass x;
  Butterworth2LowPass y;
  Butterworth2LowPass z;
};

struct Butterworth4Vect3
{
  Butterworth4LowPass x;
  Butterworth4LowPass y;
  Butterworth4LowPass z;
};


/**
 * @brief Initialize a set of Butterworth low-pass filters to zero for 3D vector data.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] freq Cutoff frequency for the filters (rads/).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_2_vect3(struct Butterworth2Vect3 *filter, float freq, float dt)
{
  init_butterworth_2_low_pass(&filter->x, 1.0f / freq, dt, 0.0f);
  init_butterworth_2_low_pass(&filter->y, 1.0f / freq, dt, 0.0f);
  init_butterworth_2_low_pass(&filter->z, 1.0f / freq, dt, 0.0f);
}

/**
 * @brief Initialize a Butterworth low-pass filter to zero.
 *
 * @param[out] filter Butterworth2LowPass filter instance.
 * @param[in] freq Cutoff frequency of the filter (rad/s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_2(Butterworth2LowPass *filter, float freq, float dt)
{
  init_butterworth_2_low_pass(filter, 1.0f / freq, dt, 0.0f);
}

/**
 * @brief Initialize an array of Butterworth low-pass filters to zero.
 *
 * @param[in] n Number of filters to initialize.
 * @param[out] filter_array Array of Butterworth2LowPass filters to initialize.
 * @param[in] freq Cutoff frequency for the filters (rad/s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_2_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], float freq, float dt)
{
  float tau = 1.0f / freq;
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_2_low_pass(&filter_array[i], tau, dt, 0.0f);
  }
}

/**
 * @brief Update a Butterworth low-pass filter with new input data.
 *
 * @param[in,out] filter Butterworth2LowPass filter instance to update.
 * @param[in] input New input data to feed into the filter.
 */
static inline void update_butterworth_2(Butterworth2LowPass *filter, float input)
{
  update_butterworth_2_low_pass(filter, input);
}

/**
 * @brief Update 3D vector Butterworth filters with new input data.
 *
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatVect3 struct containing new input data.
 */
static inline void update_butterworth_2_vect3(struct Butterworth2Vect3 *filter, const struct FloatVect3 *input)
{
  update_butterworth_2_low_pass(&filter->x, input->x);
  update_butterworth_2_low_pass(&filter->y, input->y);
  update_butterworth_2_low_pass(&filter->z, input->z);
}

/**
 * @brief Update 3D vector Butterworth filters with new rate input data.
 *
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatRates struct containing new input rate data.
 */
static inline void update_butterworth_2_rates(struct Butterworth2Vect3 *filter, const struct FloatRates *input)
{
  update_butterworth_2_low_pass(&filter->x, input->p);
  update_butterworth_2_low_pass(&filter->y, input->q);
  update_butterworth_2_low_pass(&filter->z, input->r);
}

/**
 * @brief Update an array of Butterworth low-pass filters with new input data.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth2LowPass filters to update.
 * @param[in] input_array Array containing new input data for each filter.
 */
static inline void update_butterworth_2_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_butterworth_2_low_pass(&filter_array[i], input_array[i]);
  }
}

/**
 * @brief Reset a Butterworth low-pass filter to a specific value.
 *
 * @param[out] filter Butterworth2LowPass filter instance to reset.
 * @param[in] value Value to reset the filter to.
 */
static inline void reset_butterworth_2(Butterworth2LowPass *filter, float value)
{
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}

/**
 * @brief Reset 3D vector Butterworth filters to a specific value.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatVect3 struct containing the reset value.
 */
static inline void reset_butterworth_2_vect3(struct Butterworth2Vect3 *filter, const struct FloatVect3 *value)
{
  filter->x.i[0] = filter->x.i[1] = filter->x.o[0] = filter->x.o[1] = value->x;
  filter->y.i[0] = filter->y.i[1] = filter->y.o[0] = filter->y.o[1] = value->y;
  filter->z.i[0] = filter->z.i[1] = filter->z.o[0] = filter->z.o[1] = value->z;
}

/**
 * @brief Reset 3D vector Butterworth filters to specific rate values.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatRates struct containing the reset values.
 */
static inline void reset_butterworth_2_rates(struct Butterworth2Vect3 *filter, const struct FloatRates *value)
{
  filter->x.i[0] = filter->x.i[1] = filter->x.o[0] = filter->x.o[1] = value->p;
  filter->y.i[0] = filter->y.i[1] = filter->y.o[0] = filter->y.o[1] = value->q;
  filter->z.i[0] = filter->z.i[1] = filter->z.o[0] = filter->z.o[1] = value->r;
}

/**
 * @brief Reset an array of Butterworth low-pass filters to specific values.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2LowPass filters to reset.
 * @param[in] value_array Array containing reset values for each filter.
 */
static inline void reset_butterworth_2_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    filter_array[i].i[0] = filter_array[i].i[1] = filter_array[i].o[0] = filter_array[i].o[1] = value_array[i];
  }
}

/**
 * @brief Reinitialize a Butterworth low-pass filter with new frequency and time step.
 *
 * @param[out] filter Butterworth2LowPass filter instance to reinitialize.
 * @param[in] freq New cutoff frequency for the filter (rad/s).
 * @param[in] dt New sampling time interval (seconds).
 */
static inline void reinit_butterworth_2(Butterworth2LowPass *filter, float freq, float dt)
{
  init_butterworth_2_low_pass(filter, 1.0f / freq, dt, get_butterworth_2_low_pass(filter));
}

/**
 * @brief Reinitialize 3D vector Butterworth filters with new frequency and time step.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] freq New cutoff frequency for the filters (rad/s).
 * @param[in] dt New sampling time interval (seconds).
 */
static inline void reinit_butterworth_2_vect3(struct Butterworth2Vect3 *filter, float freq, float dt)
{
  init_butterworth_2_low_pass(&filter->x, 1.0f / freq, dt, get_butterworth_2_low_pass(&filter->x));
  init_butterworth_2_low_pass(&filter->y, 1.0f / freq, dt, get_butterworth_2_low_pass(&filter->y));
  init_butterworth_2_low_pass(&filter->z, 1.0f / freq, dt, get_butterworth_2_low_pass(&filter->z));
}

/**
 * @brief Reinitialize an array of Butterworth low-pass filters with new frequency and time step.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2LowPass filters to reinitialize.
 * @param[in] freq New cutoff frequency for the filters (rad/s).
 * @param[in] dt New sampling time interval (seconds).
 */
static inline void reinit_butterworth_2_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], float freq, float dt)
{
  float tau = 1.0f / freq;
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_2_low_pass(&filter_array[i], tau, dt, get_butterworth_2_low_pass(&filter_array[i]));
  }
}


/**
 * @brief Retrieve the filtered output from a Butterworth low-pass filter.
 *
 * @param[in] filter Butterworth2LowPass filter instance.
 * @return Filtered output value.
 */
static inline float get_butterworth_2(const Butterworth2LowPass *filter)
{
  return get_butterworth_2_low_pass(filter);
}

/**
 * @brief Retrieve the filtered output from 3D vector Butterworth filters.
 *
 * @param[in] filter Struct containing Butterworth filters for x, y, z components.
 * @return FloatVect3 struct containing the filtered output values.
 */
static inline struct FloatVect3 get_butterworth_2_vect3(const struct Butterworth2Vect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_butterworth_2_low_pass(&filter->x);
  output.y = get_butterworth_2_low_pass(&filter->y);
  output.z = get_butterworth_2_low_pass(&filter->z);
  return output;
}

/**
 * @brief Retrieve the filtered output from a Butterworth low-pass filter.
 *
 * @param[in] filter Butterworth2LowPass filter instance.
 * @return Filtered output value.
 */
static inline struct FloatRates get_butterworth_2_rates(const struct Butterworth2Vect3 *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_2_low_pass(&filter->x);
  output.q = get_butterworth_2_low_pass(&filter->y);
  output.r = get_butterworth_2_low_pass(&filter->z);
  return output;
}

/**
 * @brief Retrieve the filtered outputs from an array of Butterworth low-pass filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth2LowPass filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_2_array(uint8_t n, const Butterworth2LowPass filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_butterworth_2_low_pass(&filter_array[i]);
  }
}

/**
 * @brief Initialize a set of Butterworth low-pass filters to zero for 3D vector data.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] freq Cutoff frequency for the filters (rad/s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_4_vect3(struct Butterworth4Vect3 *filter, float freq, float dt)
{
  init_butterworth_4_low_pass(&filter->x, 1.0f / freq, dt, 0.0f);
  init_butterworth_4_low_pass(&filter->y, 1.0f / freq, dt, 0.0f);
  init_butterworth_4_low_pass(&filter->z, 1.0f / freq, dt, 0.0f);
}

/**
 * @brief Initialize a Butterworth low-pass filter to zero.
 *
 * @param[out] filter Butterworth4LowPass filter instance.
 * @param[in] freq Cutoff frequency of the filter (rads/).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_4(Butterworth4LowPass *filter, float freq, float dt)
{
  init_butterworth_4_low_pass(filter, 1.0f / freq, dt, 0.0f);
}

/**
 * @brief Initialize an array of Butterworth low-pass filters to zero.
 *
 * @param[in] n Number of filters to initialize.
 * @param[out] filter_array Array of Butterworth2LowPass filters to initialize.
 * @param[in] freq Cutoff frequency for the filters (rad/s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_4_array(uint8_t n, Butterworth4LowPass filter_array[restrict n], float freq, float dt)
{
  float tau = 1.0f / freq;
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_4_low_pass(&filter_array[i], tau, dt, 0.0f);
  }
}

/**
 * @brief Update a Butterworth low-pass filter with new input data.
 *
 * @param[in,out] filter Butterworth2LowPass filter instance to update.
 * @param[in] input New input data to feed into the filter.
 */
static inline void update_butterworth_4(Butterworth4LowPass *filter, float input)
{
  update_butterworth_4_low_pass(filter, input);
}

/**
 * @brief Update 3D vector Butterworth filters with new input data.
 *
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatVect3 struct containing new input data.
 */
static inline void update_butterworth_4_vect3(struct Butterworth4Vect3 *filter, const struct FloatVect3 *input)
{
  update_butterworth_4_low_pass(&filter->x, input->x);
  update_butterworth_4_low_pass(&filter->y, input->y);
  update_butterworth_4_low_pass(&filter->z, input->z);
}

/**
 * @brief Update 3D vector Butterworth filters with new rate input data.
 *
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatRates struct containing new input rate data.
 */
static inline void update_butterworth_4_rates(struct Butterworth4Vect3 *filter, const struct FloatRates *input)
{
  update_butterworth_4_low_pass(&filter->x, input->p);
  update_butterworth_4_low_pass(&filter->y, input->q);
  update_butterworth_4_low_pass(&filter->z, input->r);
}

/**
 * @brief Update an array of Butterworth low-pass filters with new input data.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth2LowPass filters to update.
 * @param[in] input_array Array containing new input data for each filter.
 */
static inline void update_butterworth_4_array(uint8_t n, Butterworth4LowPass filter_array[restrict n], const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_butterworth_4_low_pass(&filter_array[i], input_array[i]);
  }
}

/**
 * @brief Reset a Butterworth low-pass filter to a specific value.
 *
 * @param[out] filter Butterworth2LowPass filter instance to reset.
 * @param[in] value Value to reset the filter to.
 */
static inline void reset_butterworth_4(Butterworth4LowPass *filter, float value)
{
  filter->lp1.i[0] = filter->lp1.i[1] = filter->lp1.o[0] = filter->lp1.o[1] = filter->lp2.i[0] = filter->lp2.i[1] = filter->lp2.o[0] = filter->lp2.o[1] = value;
}

/**
 * @brief Reset 3D vector Butterworth filters to a specific value.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatVect3 struct containing the reset value.
 */
static inline void reset_butterworth_4_vect3(struct Butterworth4Vect3 *filter, const struct FloatVect3 *value)
{
  filter->x.lp1.i[0] = filter->x.lp1.i[1] = filter->x.lp1.o[0] = filter->x.lp1.o[1] = filter->x.lp2.i[0] = filter->x.lp2.i[1] = filter->x.lp2.o[0] = filter->x.lp2.o[1] = value->x;
  filter->y.lp1.i[0] = filter->y.lp1.i[1] = filter->y.lp1.o[0] = filter->y.lp1.o[1] = filter->y.lp2.i[0] = filter->y.lp2.i[1] = filter->y.lp2.o[0] = filter->y.lp2.o[1] = value->y;
  filter->z.lp1.i[0] = filter->z.lp1.i[1] = filter->z.lp1.o[0] = filter->z.lp1.o[1] = filter->z.lp2.i[0] = filter->z.lp2.i[1] = filter->z.lp2.o[0] = filter->z.lp2.o[1] = value->z;
}

/**
 * @brief Reset 3D vector Butterworth filters to specific rate values.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatRates struct containing the reset values.
 */
static inline void reset_butterworth_4_rates(struct Butterworth4Vect3 *filter, const struct FloatRates *value)
{
  filter->x.lp1.i[0] = filter->x.lp1.i[1] = filter->x.lp1.o[0] = filter->x.lp1.o[1] = filter->x.lp2.i[0] = filter->x.lp2.i[1] = filter->x.lp2.o[0] = filter->x.lp2.o[1] = value->p;
  filter->y.lp1.i[0] = filter->y.lp1.i[1] = filter->y.lp1.o[0] = filter->y.lp1.o[1] = filter->y.lp2.i[0] = filter->y.lp2.i[1] = filter->y.lp2.o[0] = filter->y.lp2.o[1] = value->q;
  filter->z.lp1.i[0] = filter->z.lp1.i[1] = filter->z.lp1.o[0] = filter->z.lp1.o[1] = filter->z.lp2.i[0] = filter->z.lp2.i[1] = filter->z.lp2.o[0] = filter->z.lp2.o[1] = value->r;
}

/**
 * @brief Reset an array of Butterworth low-pass filters to specific values.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2LowPass filters to reset.
 * @param[in] value_array Array containing reset values for each filter.
 */
static inline void reset_butterworth_4_array(uint8_t n, Butterworth4LowPass filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    filter_array[i].lp1.i[0] = filter_array[i].lp1.i[1] = filter_array[i].lp1.o[0] = filter_array[i].lp1.o[1] = filter_array[i].lp2.i[0] = filter_array[i].lp2.i[1] = filter_array[i].lp2.o[0] = filter_array[i].lp2.o[1] = value_array[i];
  }
}

/**
 * @brief Reinitialize a Butterworth low-pass filter with new frequency and time step.
 *
 * @param[out] filter Butterworth2LowPass filter instance to reinitialize.
 * @param[in] freq New cutoff frequency for the filter (rad/s).
 * @param[in] dt New sampling time interval (seconds).
 */
static inline void reinit_butterworth_4(Butterworth4LowPass *filter, float freq, float dt)
{
  init_butterworth_4_low_pass(filter, 1.0f / freq, dt, get_butterworth_4_low_pass(filter));
}

/**
 * @brief Reinitialize 3D vector Butterworth filters with new frequency and time step.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] freq New cutoff frequency for the filters (rad/s).
 * @param[in] dt New sampling time interval (seconds).
 */
static inline void reinit_butterorth_4_vect3(struct Butterworth4Vect3 *filter, float freq, float dt)
{
  init_butterworth_4_low_pass(&filter->x, 1.0f / freq, dt, get_butterworth_4_low_pass(&filter->x));
  init_butterworth_4_low_pass(&filter->y, 1.0f / freq, dt, get_butterworth_4_low_pass(&filter->y));
  init_butterworth_4_low_pass(&filter->z, 1.0f / freq, dt, get_butterworth_4_low_pass(&filter->z));
}

/**
 * @brief Reinitialize an array of Butterworth low-pass filters with new frequency and time step.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2LowPass filters to reinitialize.
 * @param[in] freq New cutoff frequency for the filters (rad/s).
 * @param[in] dt New sampling time interval (seconds).
 */
static inline void reinit_butterworth_4_array(uint8_t n, Butterworth4LowPass filter_array[restrict n], float freq, float dt)
{
  float tau = 1.0f / freq;
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_4_low_pass(&filter_array[i], tau, dt, get_butterworth_4_low_pass(&filter_array[i]));
  }
}

/**
 * @brief Retrieve the filtered output from a Butterworth low-pass filter.
 *
 * @param[in] filter Butterworth2LowPass filter instance.
 * @return Filtered output value.
 */
static inline float get_butterworth_4(const Butterworth4LowPass *filter)
{
  return get_butterworth_4_low_pass(filter);
}

/**
 * @brief Retrieve the filtered output from 3D vector Butterworth filters.
 *
 * @param[in] filter Struct containing Butterworth filters for x, y, z components.
 * @return FloatVect3 struct containing the filtered output values.
 */
static inline struct FloatVect3 get_butterworth_4_vect3(const struct Butterworth4Vect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_butterworth_4_low_pass(&filter->x);
  output.y = get_butterworth_4_low_pass(&filter->y);
  output.z = get_butterworth_4_low_pass(&filter->z);
  return output;
}

/**
 * @brief Retrieve the filtered output from a Butterworth low-pass filter.
 *
 * @param[in] filter Butterworth2LowPass filter instance.
 * @return Filtered output value.
 */
static inline struct FloatRates get_butterworth_4_rates(const struct Butterworth4Vect3 *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_4_low_pass(&filter->x);
  output.q = get_butterworth_4_low_pass(&filter->y);
  output.r = get_butterworth_4_low_pass(&filter->z);
  return output;
}

/**
 * @brief Retrieve the filtered outputs from an array of Butterworth low-pass filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth2LowPass filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_4_array(uint8_t n, const Butterworth4LowPass filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_butterworth_4_low_pass(&filter_array[i]);
  }
}

#endif // LOW_PASS_FILTER_TYPES_H