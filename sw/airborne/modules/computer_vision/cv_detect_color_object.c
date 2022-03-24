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
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
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

#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw2 = true;

// define global variables
// Struct type from the ABI message, ideally moved to header file
struct color_object_t {
  int32_t Nobs;
  int32_t obstacle_data[100][4];
  uint32_t y_max;
  uint32_t x_max;
  bool updated;
};
struct color_object_t global_filters[2];

// Function
uint32_t create_obstacle_matrix(int (*obstacle_data)[4], struct image_t *img, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  lum_min = cod_lum_min2;
  lum_max = cod_lum_max2;
  cb_min = cod_cb_min2;
  cb_max = cod_cb_max2;
  cr_min = cod_cr_min2;
  cr_max = cod_cr_max2;
  draw = cod_draw2;

  // Filter and find centroid
  int obstacle_data[100][4];
  uint32_t Nobs = create_obstacle_matrix(obstacle_data, img, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].Nobs = Nobs;
  for (int i=0; i<100; ++i){
    for (int j=0; j<4; ++j){
      global_filters[filter-1].obstacle_data[i][j] = obstacle_data[i][j];
    }
  }
  global_filters[filter-1].y_max = img->h;
  global_filters[filter-1].x_max = img->w;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}


// Define detector struct
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * create_obstacle_matrix
 *
 * Creates a matrix with the pixel coordinates of the lower left and lower rigth corner of every obstacle
 *
 * @param img - input image to process formatted as YUV422.
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t create_obstacle_matrix(int (*obstacle_data)[4], struct image_t *img, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint8_t *buffer = img->buf;

  uint8_t count_obstacle = 0;
  uint8_t count_green = 0;

  int y_pix_left;
  int y_pix_right;
  int x_pix_left;
  int x_pix_right;

  int Nobs = 0;

  // Go through all the pixels
  for (uint16_t x = 0; x < (0.75*(img->w)); x ++) {
    bool in_zoo = false;
    bool obstacle = false;
    bool skip = false;
    uint8_t count_green = 0;
    uint8_t count_obstacle = 0;
    for (uint16_t y = 0; y < img->h; y++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;

      skip = false;
      for (int i=0; i<Nobs; ++i){
        if (y>obstacle_data[i][1]-6 && y<obstacle_data[i][3]+6){
          skip = true;
        }
      }
      if (!skip){ // Skip if above an obstacle that is already known
        if (x % 2 == 0) {
          // Even x
          up = &buffer[y * 2 * img->w + 2 * x];      // U
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
          vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
          //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
        } else {
          // Uneven x
          up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
          //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
          vp = &buffer[y * 2 * img->w + 2 * x];      // V
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
        }
        if ( (*yp >= lum_min) && (*yp <= lum_max) &&
            (*up >= cb_min ) && (*up <= cb_max ) &&
            (*vp >= cr_min ) && (*vp <= cr_max )) {
          count_green ++;
          if (count_green==3 && !in_zoo){ // First time the algorithm is actually on the green mat
            count_obstacle = 0;
            in_zoo = true;
          }
          if (count_green==3 && obstacle && in_zoo){ // End of obstacle detected
            count_obstacle = 0;
            obstacle = false;
            Nobs ++;
            y_pix_right = y;
            x_pix_right = x;
            obstacle_data[Nobs-1][0] = x_pix_left;
            obstacle_data[Nobs-1][1] = y_pix_left;
            obstacle_data[Nobs-1][2] = x_pix_right;
            obstacle_data[Nobs-1][3] = y_pix_right;
          }
        }
        else{ // Not a green pixel
          count_obstacle ++;
          if (count_obstacle==3 && !obstacle && in_zoo){ //Beginning obstacle detected
            count_green = 0;
            obstacle = true;
            // *yp = 255;
            y_pix_left = y;
            x_pix_left = x;
          }
        }
      }
    }
  }

  // Loop through pixels again to highligh selected obstacle corners (comment for faster runtime)
  if (draw){
    for (uint16_t x = 0; x < (0.75*(img->w)); x ++) {
      for (uint16_t y = 0; y < img->h; y++) {
        uint8_t *yp, *up, *vp;
        if (x % 2 == 0) {
          // Even x
          up = &buffer[y * 2 * img->w + 2 * x];      // U
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
          vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
          //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
        } else {
          // Uneven x
          up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
          //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
          vp = &buffer[y * 2 * img->w + 2 * x];      // V
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
        }

        for (int i=0; i<Nobs; ++i){
          if (abs(y-obstacle_data[i][1]) < 4 && abs(x-obstacle_data[i][0]) < 4){
            *yp = 255;
          }else if (abs(y-obstacle_data[i][3]) < 4 && abs(x-obstacle_data[i][2]) < 4){
            *yp = 255;
          }
        }
      }
    }
  }

  return Nobs;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[1].updated){
    AbiSendMsgOF_OBSTACLE_DATA(OPTIC_FLOW_OBSTACLE_DATA1_ID, &local_filters[1]);
    local_filters[1].updated = false;
  }
  return;
}
