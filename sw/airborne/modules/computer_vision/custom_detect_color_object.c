/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/custom_detect_color_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/custom_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS
#define COLOR_OBJECT_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

struct object_position_t {
  uint8_t left;
  uint8_t middle;
  uint8_t right;
  bool updated;
};

struct object_position_t detectedObjectsThreadShared;

// Filter Settings

uint8_t orange_lum_min = 0;
uint8_t orange_lum_max = 0;
uint8_t orange_cb_min = 0;
uint8_t orange_cb_max = 0;
uint8_t orange_cr_min = 0;
uint8_t orange_cr_max = 0;

uint8_t green_lum_min = 0;
uint8_t green_lum_max = 0;
uint8_t green_cb_min = 0;
uint8_t green_cb_max = 0;
uint8_t green_cr_min = 0;
uint8_t green_cr_max = 0;

uint8_t orange_percent = 0;
uint8_t green_percent = 0;

bool cod_draw = false; 


struct image_t *object_detector(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{

  // Create 3x3 grid, focus on middle column (column 1)
  uint16_t grid_width = img->w / 3;
  uint16_t grid_height = img->h / 3;
  uint16_t col_start = grid_width;
  uint16_t col_end = grid_width * 2;
  if (col_start & 1) {
    col_start++;
  }

  uint32_t top_orange = 0, middle_orange = 0, bottom_orange = 0;
  uint32_t top_green = 0, middle_green = 0, bottom_green = 0;
  uint8_t *img_buf = (uint8_t *)img->buf;
  uint16_t pixel_threshold_orange = (float)grid_width * (float)grid_height * (float)orange_percent / 100.f; // 10% threshold
  uint16_t pixel_threshold_green = (float)grid_width * (float)grid_height * (float)green_percent / 100.f; // 10% threshold

  // Scan middle column of grid using per-pixel UYVY access.
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = col_start; x < col_end; x++) {
      uint8_t *yp, *up, *vp;
      uint32_t row_offset = y * 2 * img->w;

      if ((x % 2) == 0) {
        // Even x: U,Y0,V,Y1
        up = &img_buf[row_offset + 2 * x];
        yp = &img_buf[row_offset + 2 * x + 1];
        vp = &img_buf[row_offset + 2 * x + 2];
      } else {
        // Odd x uses same U/V as previous even pixel
        up = &img_buf[row_offset + 2 * x - 2];
        vp = &img_buf[row_offset + 2 * x];
        yp = &img_buf[row_offset + 2 * x + 1];
      }

      bool orange_match = ((*yp >= orange_lum_min) && (*yp <= orange_lum_max) &&
                           (*up >= orange_cb_min) && (*up <= orange_cb_max) &&
                           (*vp >= orange_cr_min) && (*vp <= orange_cr_max));
      bool green_match = ((*yp >= green_lum_min) && (*yp <= green_lum_max) &&
                          (*up >= green_cb_min) && (*up <= green_cb_max) &&
                          (*vp >= green_cr_min) && (*vp <= green_cr_max));

      if (orange_match) {
        if (y < grid_height) { top_orange++; }
        else if (y < grid_height * 2) { middle_orange++; }
        else { bottom_orange++; }
        *yp = 255; // make pixel brighter in image for visualization
      }

      if (green_match) {
        if (y < grid_height) { top_green++; }
        else if (y < grid_height * 2) { middle_green++; }
        else { bottom_green++; }
        *yp = 255; // make pixel brighter in image for visualization
      }
    }
  }

  struct object_position_t localDetections; 

  // ABI keeps left/middle/right fields; map them to top/middle/bottom regions.
  if((top_orange > pixel_threshold_orange) || top_green > pixel_threshold_green){
    localDetections.left = 1;
  } else {
    localDetections.left = 0;
  }

  if((middle_orange > pixel_threshold_orange) || middle_green > pixel_threshold_green){
    localDetections.middle = 1;
  } else {
    localDetections.middle = 0;
  }

  if((bottom_orange > pixel_threshold_orange) || bottom_green > pixel_threshold_green){
    localDetections.right = 1;
  } else {
    localDetections.right = 0;
  }

  localDetections.updated = true;

  pthread_mutex_lock(&mutex);

  // Update shared data
  memcpy(&detectedObjectsThreadShared, &localDetections, sizeof(struct object_position_t));

  pthread_mutex_unlock(&mutex);

  return img;
}


void color_object_detector_init(void)
{
  memset(&detectedObjectsThreadShared, 0, sizeof(struct object_position_t));

  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA
#ifdef ORANGE_OBJECT_DETECTOR_LUM_MIN
  orange_lum_min = ORANGE_OBJECT_DETECTOR_LUM_MIN;
  orange_lum_max = ORANGE_OBJECT_DETECTOR_LUM_MAX;
  orange_cb_min = ORANGE_OBJECT_DETECTOR_CB_MIN;
  orange_cb_max = ORANGE_OBJECT_DETECTOR_CB_MAX;
  orange_cr_min = ORANGE_OBJECT_DETECTOR_CR_MIN;
  orange_cr_max = ORANGE_OBJECT_DETECTOR_CR_MAX;
#endif
#ifdef GREEN_OBJECT_DETECTOR_LUM_MIN
  green_lum_min = GREEN_OBJECT_DETECTOR_LUM_MIN;
  green_lum_max = GREEN_OBJECT_DETECTOR_LUM_MAX;
  green_cb_min = GREEN_OBJECT_DETECTOR_CB_MIN;
  green_cb_max = GREEN_OBJECT_DETECTOR_CB_MAX;
  green_cr_min = GREEN_OBJECT_DETECTOR_CR_MIN;
  green_cr_max = GREEN_OBJECT_DETECTOR_CR_MAX;
#endif

#ifdef ORANGE_PERCENT
  orange_percent = ORANGE_PERCENT;
#endif
#ifdef GREEN_PERCENT
  green_percent = GREEN_PERCENT;
#endif


  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA, object_detector, COLOR_OBJECT_DETECTOR_FPS, 0);
#endif
}




void color_object_detector_periodic(void)
{
  static struct object_position_t detectedObjectsLocal;

  pthread_mutex_lock(&mutex);

  memcpy(&detectedObjectsLocal, &detectedObjectsThreadShared, sizeof(struct object_position_t));

  pthread_mutex_unlock(&mutex);

  if(detectedObjectsLocal.updated){
    AbiSendMsgCUSTOM_DETECTION(CUSTOM_AVOIDER_CUSTOM_DETECTION_ID, detectedObjectsLocal.left, detectedObjectsLocal.middle, detectedObjectsLocal.right);
    detectedObjectsLocal.updated = false;
  }

}
