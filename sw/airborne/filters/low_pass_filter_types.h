#ifndef LOW_PASS_FILTER_TYPES_H
#define LOW_PASS_FILTER_TYPES_H

#include "std.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra_float.h"

struct FirstOrderLowPassVect3
{
  struct FirstOrderLowPass x;
  struct FirstOrderLowPass y;
  struct FirstOrderLowPass z;
};

/** 
 * @brief Initialize a set of first order low-pass filters to zero for 3D vector data.
 * 
 * @param[out] filter Struct containing first order low-pass filters for x, y, z components.
 * @param[in] tau Time constant for the filters (s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_first_order_low_pass_vect3(struct FirstOrderLowPassVect3 *filter, float tau, float dt)
{
  init_first_order_low_pass(&filter->x, tau, dt, 0.0f);
  init_first_order_low_pass(&filter->y, tau, dt, 0.0f);
  init_first_order_low_pass(&filter->z, tau, dt, 0.0f);
}

/**
 * @brief Update 3D vector first order low-pass filters with new input data.
 * 
 * @param[in,out] filter Struct containing first order low-pass filters for x, y, z components.
 * @param[in] input Pointer to FloatVect3 struct containing new input data.
 */
static inline void update_first_order_low_pass_vect3(struct FirstOrderLowPassVect3 *filter, const struct FloatVect3 *input)
{
  update_first_order_low_pass(&filter->x, input->x);
  update_first_order_low_pass(&filter->y, input->y);
  update_first_order_low_pass(&filter->z, input->z);
}

/** 
 * @brief Update 3D vector first order low-pass filters with new rate input data.
 * 
 * @param[in,out] filter Struct containing first order low-pass filters for x, y, z components.
 * @param[in] input Pointer to FloatRates struct containing new input rate data.
 */
static inline void update_first_order_low_pass_rates(struct FirstOrderLowPassVect3 *filter, const struct FloatRates *input)
{
  update_first_order_low_pass(&filter->x, input->p);
  update_first_order_low_pass(&filter->y, input->q);
  update_first_order_low_pass(&filter->z, input->r);
}

/** 
 * @brief Reset 3D vector first order low-pass filters to a specific value.
 * 
 * @param[out] filter Struct containing first order low-pass filters for x, y, z components.
 * @param[in] value Pointer to FloatVect3 struct containing the reset value.
 */
static inline void reset_first_order_low_pass_vect3(struct FirstOrderLowPassVect3 *filter, const struct FloatVect3 *value)
{
  reset_first_order_low_pass(&filter->x, value->x);
  reset_first_order_low_pass(&filter->y, value->y);
  reset_first_order_low_pass(&filter->z, value->z);
}

/** 
 * @brief Reset 3D vector first order low-pass filters to specific rate values.
 * 
 * @param[out] filter Struct containing first order low-pass filters for x, y, z components.
 * @param[in] value Pointer to FloatRates struct containing the reset values.
 */
static inline void reset_first_order_low_pass_rates(struct FirstOrderLowPassVect3 *filter, const struct FloatRates *value)
{
  reset_first_order_low_pass(&filter->x, value->p);
  reset_first_order_low_pass(&filter->y, value->q);
  reset_first_order_low_pass(&filter->z, value->r);
}

/**
 * @brief Retrieve the filtered output from 3D vector first order low-pass filters.
 * 
 * @param[in] filter Struct containing first order low-pass filters for x, y, z components.
 * @return FloatVect3 struct containing the filtered output values.
 */
static inline struct FloatVect3 get_first_order_low_pass_vect3(const struct FirstOrderLowPassVect3 *filter)
{
  struct FloatVect3 output;
  output.x = get_first_order_low_pass(&filter->x);
  output.y = get_first_order_low_pass(&filter->y);
  output.z = get_first_order_low_pass(&filter->z);
  return output;
}

/**
 * @brief Retrieve the filtered output from 3D vector first order low-pass filters.
 * 
 * @param[in] filter Struct containing first order low-pass filters for x, y, z components.
 * @return FloatRates struct containing the filtered output rate values.
 */
static inline struct FloatRates get_first_order_low_pass_rates(const struct FirstOrderLowPassVect3 *filter)
{
  struct FloatRates output;
  output.p = get_first_order_low_pass(&filter->x);
  output.q = get_first_order_low_pass(&filter->y);
  output.r = get_first_order_low_pass(&filter->z);
  return output;
}

/**
 * @brief Initialize an array of first order low-pass filters.
 * 
 * @param n Number of filters in the array. 
 * @param filter_array Array of FirstOrderLowPass filters to initialize.
 * @param tau Time constant for the filters (s).
 * @param dt Sampling time interval (seconds).
 */
static inline void init_first_order_low_pass_array(uint8_t n, struct FirstOrderLowPass filter_array[restrict n], float tau, float dt)
{
  for (uint8_t i = 0; i < n; i++)
  {
    init_first_order_low_pass(&filter_array[i], tau, dt, 0.0f);
  }
}

/**
 * @brief Update an array of first order low-pass filters.
 * 
 * @param n Number of filters in the array.
 * @param filter_array Array of FirstOrderLowPass filters to update.
 * @param input_array Array of input values for the filters.
 */
static inline void update_first_order_low_pass_array(uint8_t n, struct FirstOrderLowPass filter_array[restrict n], const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_first_order_low_pass(&filter_array[i], input_array[i]);
  }
}

/**
 * @brief Reset an array of first order low-pass filters to specific values.
 * 
 * @param n Number of filters in the array.
 * @param filter_array Array of FirstOrderLowPass filters to reset.
 * @param value_array Array of values to reset the filters to.
 */
static inline void reset_first_order_low_pass_array(uint8_t n, struct FirstOrderLowPass filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    reset_first_order_low_pass(&filter_array[i], value_array[i]);
  }
}

/**
 * @brief Retrieve the filtered outputs from an array of first order low-pass filters.
 * 
 * @param n Number of filters in the array.
 * @param filter_array Array of FirstOrderLowPass filters.
 * @param output_array Array to store the filtered output values.
 */
static inline void get_first_order_low_pass_array(uint8_t n, const struct FirstOrderLowPass filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_first_order_low_pass(&filter_array[i]);
  }
}


struct Butterworth2Vect3
{
  Butterworth2LowPass x;
  Butterworth2LowPass y;
  Butterworth2LowPass z;
};

/**
 * @brief Initialize a set of Butterworth low-pass filters to zero for 3D vector data.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] tau Time constant for the filters (s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_2_low_pass_vect3(struct Butterworth2Vect3 *filter, float tau, float dt)
{
  init_butterworth_2_low_pass(&filter->x, tau, dt, 0.0f);
  init_butterworth_2_low_pass(&filter->y, tau, dt, 0.0f);
  init_butterworth_2_low_pass(&filter->z, tau, dt, 0.0f);
}

/**
 * @brief Update 3D vector Butterworth filters with new input data.
 *
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatVect3 struct containing new input data.
 */
static inline void update_butterworth_2_low_pass_vect3(struct Butterworth2Vect3 *filter, const struct FloatVect3 *input)
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
static inline void update_butterworth_2_low_pass_rates(struct Butterworth2Vect3 *filter, const struct FloatRates *input)
{
  update_butterworth_2_low_pass(&filter->x, input->p);
  update_butterworth_2_low_pass(&filter->y, input->q);
  update_butterworth_2_low_pass(&filter->z, input->r);
}

/**
 * @brief Reset 3D vector Butterworth filters to a specific value.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatVect3 struct containing the reset value.
 */
static inline void reset_butterworth_2_low_pass_vect3(struct Butterworth2Vect3 *filter, const struct FloatVect3 *value)
{
  reset_butterworth_2_low_pass(&filter->x, value->x);
  reset_butterworth_2_low_pass(&filter->y, value->y);
  reset_butterworth_2_low_pass(&filter->z, value->z);
}

/**
 * @brief Reset 3D vector Butterworth filters to specific rate values.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatRates struct containing the reset values.
 */
static inline void reset_butterworth_2_low_pass_rates(struct Butterworth2Vect3 *filter, const struct FloatRates *value)
{
  reset_butterworth_2_low_pass(&filter->x, value->p);
  reset_butterworth_2_low_pass(&filter->y, value->q);
  reset_butterworth_2_low_pass(&filter->z, value->r);
}

/**
 * @brief Retrieve the filtered output from 3D vector Butterworth filters.
 *
 * @param[in] filter Struct containing Butterworth filters for x, y, z components.
 * @return FloatVect3 struct containing the filtered output values.
 */
static inline struct FloatVect3 get_butterworth_2_low_pass_vect3(const struct Butterworth2Vect3 *filter)
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
static inline struct FloatRates get_butterworth_2_low_pass_rates(const struct Butterworth2Vect3 *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_2_low_pass(&filter->x);
  output.q = get_butterworth_2_low_pass(&filter->y);
  output.r = get_butterworth_2_low_pass(&filter->z);
  return output;
}

/**
 * @brief Initialize an array of Butterworth low-pass filters to zero.
 *
 * @param[in] n Number of filters to initialize.
 * @param[out] filter_array Array of Butterworth2LowPass filters to initialize.
 * @param[in] tau Time constant for the filters (s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_2_low_pass_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], float tau, float dt)
{
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_2_low_pass(&filter_array[i], tau, dt, 0.0f);
  }
}

/**
 * @brief Update an array of Butterworth low-pass filters with new input data.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth2LowPass filters to update.
 * @param[in] input_array Array containing new input data for each filter.
 */
static inline void update_butterworth_2_low_pass_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_butterworth_2_low_pass(&filter_array[i], input_array[i]);
  }
}

/**
 * @brief Reset an array of Butterworth low-pass filters to specific values.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth2LowPass filters to reset.
 * @param[in] value_array Array containing reset values for each filter.
 */
static inline void reset_butterworth_2_low_pass_array(uint8_t n, Butterworth2LowPass filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    reset_butterworth_2_low_pass(&filter_array[i], value_array[i]);
  }
}

/**
 * @brief Retrieve the filtered outputs from an array of Butterworth low-pass filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth2LowPass filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_2_low_pass_array(uint8_t n, const Butterworth2LowPass filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_butterworth_2_low_pass(&filter_array[i]);
  }
}

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
 * @param[in] tau Time constant for the filters (s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_4_low_pass_vect3(struct Butterworth4Vect3 *filter, float tau, float dt)
{
  init_butterworth_4_low_pass(&filter->x, tau, dt, 0.0f);
  init_butterworth_4_low_pass(&filter->y, tau, dt, 0.0f);
  init_butterworth_4_low_pass(&filter->z, tau, dt, 0.0f);
}

/**
 * @brief Update 3D vector Butterworth filters with new input data.
 *
 * @param[in,out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] input Pointer to FloatVect3 struct containing new input data.
 */
static inline void update_butterworth_4_low_pass_vect3(struct Butterworth4Vect3 *filter, const struct FloatVect3 *input)
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
static inline void update_butterworth_4_low_pass_rates(struct Butterworth4Vect3 *filter, const struct FloatRates *input)
{
  update_butterworth_4_low_pass(&filter->x, input->p);
  update_butterworth_4_low_pass(&filter->y, input->q);
  update_butterworth_4_low_pass(&filter->z, input->r);
}

/**
 * @brief Reset 3D vector Butterworth filters to a specific value.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatVect3 struct containing the reset value.
 */
static inline void reset_butterworth_4_low_pass_vect3(struct Butterworth4Vect3 *filter, const struct FloatVect3 *value)
{
  reset_butterworth_4_low_pass(&filter->x, value->x);
  reset_butterworth_4_low_pass(&filter->y, value->y);
  reset_butterworth_4_low_pass(&filter->z, value->z);
}

/**
 * @brief Reset 3D vector Butterworth filters to specific rate values.
 *
 * @param[out] filter Struct containing Butterworth filters for x, y, z components.
 * @param[in] value Pointer to FloatRates struct containing the reset values.
 */
static inline void reset_butterworth_4_low_pass_rates(struct Butterworth4Vect3 *filter, const struct FloatRates *value)
{
  reset_butterworth_4_low_pass(&filter->x, value->p);
  reset_butterworth_4_low_pass(&filter->y, value->q);
  reset_butterworth_4_low_pass(&filter->z, value->r);
}

/**
 * @brief Retrieve the filtered output from 3D vector Butterworth filters.
 *
 * @param[in] filter Struct containing Butterworth filters for x, y, z components.
 * @return FloatVect3 struct containing the filtered output values.
 */
static inline struct FloatVect3 get_butterworth_4_low_pass_vect3(const struct Butterworth4Vect3 *filter)
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
 * @param[in] filter Butterworth4LowPass filter instance.
 * @return Filtered output value.
 */
static inline struct FloatRates get_butterworth_4_low_pass_rates(const struct Butterworth4Vect3 *filter)
{
  struct FloatRates output;
  output.p = get_butterworth_4_low_pass(&filter->x);
  output.q = get_butterworth_4_low_pass(&filter->y);
  output.r = get_butterworth_4_low_pass(&filter->z);
  return output;
}

/**
 * @brief Initialize an array of Butterworth low-pass filters to zero.
 *
 * @param[in] n Number of filters to initialize.
 * @param[out] filter_array Array of Butterworth4LowPass filters to initialize.
 * @param[in] tau Time constant for the filters (s).
 * @param[in] dt Sampling time interval (seconds).
 */
static inline void init_butterworth_4_low_pass_array(uint8_t n, Butterworth4LowPass filter_array[restrict n], float tau, float dt)
{
  for (uint8_t i = 0; i < n; i++)
  {
    init_butterworth_4_low_pass(&filter_array[i], tau, dt, 0.0f);
  }
}

/**
 * @brief Update an array of Butterworth low-pass filters with new input data.
 *
 * @param[in] n Number of filters in the array.
 * @param[in,out] filter_array Array of Butterworth4LowPass filters to update.
 * @param[in] input_array Array containing new input data for each filter.
 */
static inline void update_butterworth_4_low_pass_array(uint8_t n, Butterworth4LowPass filter_array[restrict n], const float input_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    update_butterworth_4_low_pass(&filter_array[i], input_array[i]);
  }
}

/**
 * @brief Reset an array of Butterworth low-pass filters to specific values.
 *
 * @param[in] n Number of filters in the array.
 * @param[out] filter_array Array of Butterworth4LowPass filters to reset.
 * @param[in] value_array Array containing reset values for each filter.
 */
static inline void reset_butterworth_4_low_pass_array(uint8_t n, Butterworth4LowPass filter_array[restrict n], const float value_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    reset_butterworth_4_low_pass(&filter_array[i], value_array[i]);
  }
}

/**
 * @brief Retrieve the filtered outputs from an array of Butterworth low-pass filters.
 *
 * @param[in] n Number of filters in the array.
 * @param[in] filter_array Array of Butterworth4LowPass filters.
 * @param[out] output_array Array to store the filtered output values.
 */
static inline void get_butterworth_4_low_pass_array(uint8_t n, const Butterworth4LowPass filter_array[restrict n], float output_array[restrict n])
{
  for (uint8_t i = 0; i < n; i++)
  {
    output_array[i] = get_butterworth_4_low_pass(&filter_array[i]);
  }
}

#endif // LOW_PASS_FILTER_TYPES_H
