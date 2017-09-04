/*
 * Copyright (C) Kirk Scheper
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
 * @file "modules/imav2017/imav2017.c"
 * @author Kirk Scheper
 * 
 */

#include "modules/imav2017/imav2017.h"

#include "filters/median_filter.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/abi.h"

// output
float gate_distance, gate_x_offset, gate_y_offset;

// median filter
struct MedianFilterFloat psi_filter, theta_filter, depth_filter, w_filter;
float psi_f, theta_f, depth_f, w_f;

void imav2017_init(void)
{
  init_median_filter_f(&psi_filter, MEDIAN_DEFAULT_SIZE);
  init_median_filter_f(&theta_filter, MEDIAN_DEFAULT_SIZE);
  init_median_filter_f(&depth_filter, MEDIAN_DEFAULT_SIZE);
  init_median_filter_f(&w_filter, MEDIAN_DEFAULT_SIZE);
}

static const float gate_size_m = 1.1f;
void imav2017_set_gate(uint8_t quality, float w, float h,
    float psi, float theta, float depth, uint8_t gate_detected)
{
  // filter incoming angles and depth
  psi_f = update_median_filter_f(&psi_filter, psi);
  theta_f = update_median_filter_f(&theta_filter, theta);
  depth_f = update_median_filter_f(&depth_filter, depth);
  w_f = update_median_filter_f(&depth_filter, w);

  gate_distance = gate_size_m / w_f;
  gate_x_offset = gate_distance * sinf(theta_f);
  gate_y_offset = gate_distance * sinf(psi_f);

  AbiSendMsgGATE_DETECTION(ABI_BROADCAST, gate_detected, gate_x_offset, gate_y_offset,gate_distance);

  DOWNLINK_SEND_TEMP_ADC(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE, &psi_f, &gate_y_offset, &gate_distance);
}



void imav2017_histogram_obstacle_detection(uint8_t *stereo_distance_per_column, uint8_t *stereo_distance_filtered,
		uint8_t *closest_average_distance, uint8_t *pixel_location_of_closest_object, int32_t size)
{
  int32_t x, c;
  int32_t max_distance_obstacle = 255; // in [cm]
  uint8_t obstc_thres = 5;

  // Measure where obstacles are within detection range and turn it into a booleean
  for (x = 0; x < size; x++) {
    if (stereo_distance_per_column[x] < max_distance_obstacle && stereo_distance_per_column[x] != 0) {
      stereo_distance_filtered[x] = 1;
    } else {
      stereo_distance_filtered[x] = 0;
    }
  }
  // Erosion of binary array
  uint8_t min_value;
  const uint8_t morph_value = 10;
  uint8_t stereo_distance_filtered_temp[128] = {0};

  // Dilation
  uint8_t max_value;
  for (x = morph_value; x < size - morph_value - 1; x++) {
    max_value = 0;
    for (c = -morph_value; c <= morph_value; c++) {
      if (max_value < stereo_distance_filtered[x + c]) {
        max_value = stereo_distance_filtered[x + c];
      }
    }
    stereo_distance_filtered_temp[x] =  max_value;
  }
  memcpy(stereo_distance_filtered, stereo_distance_filtered_temp, sizeof(stereo_distance_filtered_temp));

  for (x = morph_value; x < size - morph_value-1; x++) {
    min_value = 1;
    for (c = -morph_value; c <= morph_value; c++) {
      if (min_value > stereo_distance_filtered[x + c]) {
        min_value = stereo_distance_filtered[x + c];
      }
    }
    stereo_distance_filtered_temp[x] =  min_value;
  }

  memcpy(stereo_distance_filtered, stereo_distance_filtered_temp, sizeof(stereo_distance_filtered_temp));

  /*
      // Seperate obstacles with a large distance difference
      for (x = border; x < size - border; x++) {

        if (stereo_distance_per_column[x]!=0&&stereo_distance_per_column[x+2]!=0&&abs(stereo_distance_per_column[x] - stereo_distance_per_column[x + 1]) > 55) {
          stereo_distance_filtered[x] = 0;
        }
      }
  */

  //calculate the distance of the closest obstacle
  int32_t counter = 0;
  int32_t distance_sum = 0;
  int32_t average_distance = 0;

  int32_t start_pixel = 0;
  int8_t first_hit = 0;

  for (x = 0; x < size - 0; x++) {
    //if obstacle is detected, start counting how many you see in a row, sum op the distances
    if (stereo_distance_filtered[x] == 1) {
    	if(stereo_distance_per_column[x]!=0){
      counter ++;
      distance_sum += (int32_t)stereo_distance_per_column[x];
    	}
    	if (first_hit == 0) {start_pixel = x; first_hit = 1;}

      if (counter > obstc_thres) {
        average_distance = distance_sum / counter;
        if (*closest_average_distance > average_distance) {
          *closest_average_distance = (uint8_t)average_distance;
          *pixel_location_of_closest_object = (uint8_t)(start_pixel - obstc_thres + counter / 2);
        }
      }
    } else {
      counter = 0;
      distance_sum = 0;
      first_hit = 0;
    }
  }
}
