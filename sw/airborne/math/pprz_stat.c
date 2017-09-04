/*
 * Copyright (C) 2017 Kirk Scheper
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file pprz_stat.c
 * @brief Statistics functions.
 *
 */

#include "math/pprz_stat.h"
#include <stdlib.h>

/*********
 * Integer implementations
 *********/

/** Compute the mean value of an array
 *  This is implemented using floats to handle scaling of all variables
 *  @param[in] *array The array
 *  @param[in] n_elements Number of elements in the array
 *  @return mean
 */
int32_t mean_i(int32_t *array, uint32_t n_elements)
{
  // determine the mean for the vector:
  float sum = 0.f;
  uint32_t i;
  for (i = 0; i < n_elements; i++) {
    sum += (float)array[i];
  }

  return (int32_t)(sum / n_elements);
}

/** Compute the variance of an array of values (integer).
 *  The variance is a measure of how far a set of numbers is spread out
 *  V(X) = E[(X-E[X])^2] = E[X^2] - E[X]^2
 *  where E[X] is the expected value of X
 *  This is implemented using floats to handle scaling of all variables
 *  @param array pointer to an array of integer
 *  @param n_elements number of elements in the array
 *  @return variance
 */
int32_t variance_i(int32_t *array, uint32_t n_elements)
{
  return (covariance_i(array, array, n_elements));
}

/** Compute the covariance of two arrays
 *  V(X) = E[(X-E[X])(Y-E[Y])] = E[XY] - E[X]E[Y]
 *  where E[X] is the expected value of X
 *  This is implemented using floats to handle scaling of all variables
 *  @param[in] *array1 The first array
 *  @param[in] *array2 The second array
 *  @param[in] n_elements Number of elements in the arrays
 *  @return covariance
 */
int32_t covariance_i(int32_t *array1, int32_t *array2, uint32_t n_elements)
{
  // Determine means for each vector:
  float sumX = 0.f, sumY = 0.f, sumXY = 0.f;

  // Determine the covariance:
  uint32_t i;
  for (i = 0; i < n_elements; i++) {
    sumX += (float)array1[i];
    sumY += (float)array2[i];
    sumXY += (float)(array1[i]) * (float)(array2[i]);
  }

  return (int32_t)(sumXY / n_elements - sumX * sumY / (n_elements * n_elements));
}

static int int_compare(const void *a, const void *b)
{
  int fa = *((const int*)a);
  int fb = *((const int*)b);

  return fa - fb;
}

/* Compute mean value of arr with n_element values
 *  @param[in] *arr The array
 *  @param[in] n_elements Number of elements in arr
 *  @return median of arr
 */
float median_i(int32_t *arr, uint32_t n_elements)
{
  int32_t result = 0;
  if (n_elements == 0)
  {
    return result;
  }

  // Get the median flow
  qsort(arr, n_elements, sizeof(int), int_compare);
  if (n_elements % 2)
  {
    result = arr[n_elements / 2];
  } else
  {
    // Take the average of the 2 median points
    result = (arr[n_elements / 2 - 1] + arr[n_elements / 2]) / 2;
  }
  return result;
}

/*********
 * Float implementations
 *********/

/** Compute the mean value of an array (float)
 *  @param[in] *array The array
 *  @param[in] n_elements Number of elements in the array
 *  @return mean
 */
float mean_f(float *array, uint32_t n_elements)
{
  // determine the mean for the vector:
  float sum = 0.f;
  uint32_t i;
  for (i = 0; i < n_elements; i++) {
    sum += array[i];
  }

  return (sum / n_elements);
}

/** Compute the variance of an array of values (float).
 *  The variance is a measure of how far a set of numbers is spread out
 *  V(X) = E[(X-E[X])^2] = E[X^2] - E[X]^2
 *  where E[X] is the expected value of X
 *  @param array Pointer to an array of float
 *  @param n_elements Number of values in the array
 *  @return variance
 */
float variance_f(float *array, uint32_t n_elements)
{
  return covariance_f(array, array, n_elements);
}

/** Compute the covariance of two arrays
 *  V(X) = E[(X-E[X])(Y-E[Y])] = E[XY] - E[X]E[Y]
 *  where E[X] is the expected value of X
 *  @param[in] *array1 The first array
 *  @param[in] *array2 The second array
 *  @param[in] n_elements Number of elements in the arrays
 *  @return covariance
 */
float covariance_f(float *arr1, float *arr2, uint32_t n_elements)
{
  // Determine means for each vector:
  float sumX = 0.f, sumY = 0.f, sumXY = 0.f;

  // Determine the covariance:
  uint32_t i;
  for (i = 0; i < n_elements; i++) {
    sumX += arr1[i];
    sumY += arr2[i];
    sumXY += arr1[i] * arr2[i];
  }

  return (sumXY / n_elements - sumX * sumY / (n_elements * n_elements));
}

static int float_compare(const void *a, const void *b)
{
  float fa = *((const float*)a);
  float fb = *((const float*)b);
  if( fabs(fa - fb) <= 1e-7 )
    return 0;
  else
    return fa < fb ? -1 : 1;
}

/* Compute mean value of arr with n_element values
 *  @param[in] *arr The array
 *  @param[in] n_elements Number of elements in arr
 *  @return median of arr
 */
float median_f(float *arr, uint32_t n_elements)
{
  float result = 0.f;
  if (n_elements == 0)
  {
    return result;
  }

  // Get the median flow
  qsort(arr, n_elements, sizeof(float), float_compare);
  if (n_elements % 2)
  {
    result = arr[n_elements / 2];
  } else
  {
    // Take the average of the 2 median points
    result = (arr[n_elements / 2 - 1] + arr[n_elements / 2]) / 2;
  }
  return result;
}
