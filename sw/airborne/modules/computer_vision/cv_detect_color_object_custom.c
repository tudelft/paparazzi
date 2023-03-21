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
#include "modules/computer_vision/cv_detect_color_object_custom.h"
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

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

//NEW GLOBAL VARIABLES

// #define k_size 25.0f
// #define T_x 50.0f
// #define T_y 100.0f
// #ifndef alpha
// #define alpha 1.5/2.0
// #endif


//NEW

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  
  //new
  int16_t vector_x;
  int16_t vector_y;
  //new
  
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
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

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  // Filter and find centroid
  uint32_t count;
  int16_t vx;
  int16_t vy;
  count, vx, vy = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  
  //new
  global_filters[filter-1].vector_x = vx;
  global_filters[filter-1].vector_y = vy;
  //new
  
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

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
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */

uint16_t triangle_shape(uint16_t y);

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // // Go through all the pixels
  // for (uint16_t y = 0; y < img->h; y++) {
  //   for (uint16_t x = 0; x < img->w; x ++) {
  //     // Check if the color is inside the specified values
  //     uint8_t *yp, *up, *vp;
  //     if (x % 2 == 0) {
  //       // Even x
  //       up = &buffer[y * 2 * img->w + 2 * x];      // U
  //       yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
  //       vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
  //       //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
  //     } else {
  //       // Uneven x
  //       up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
  //       //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
  //       vp = &buffer[y * 2 * img->w + 2 * x];      // V
  //       yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
  //     }

      
  // //     if ( (*yp >= lum_min) && (*yp <= lum_max) &&
  // //          (*up >= cb_min ) && (*up <= cb_max ) &&
  // //          (*vp >= cr_min ) && (*vp <= cr_max )) {
  // //       cnt ++;
  // //       tot_x += x;
  // //       tot_y += y;
  // //       if (draw){
  // //         *yp = 255;  // make pixel brighter in image
  // //       }
  // //     }
  // //   }
  // // }
  // // if (cnt > 0) {
  // //   *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
  // //   *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  // // } else {
  // //   *p_xc = 0;
  // //   *p_yc = 0;
  // // }

  //       if ( (*yp >= lum_min) && (*yp <= lum_max) &&
  //          (*up >= cb_min ) && (*up <= cb_max ) &&
  //          (*vp >= cr_min ) && (*vp <= cr_max )) {
  //       cnt ++;
  //       tot_x += x;
  //       tot_y += y;
  //       if (draw){
  //         *yp = 255;  // make pixel brighter in image
  //       }
  //     }
  //   }
  // }
  // if (cnt > 0) {
  //   *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
  //   *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  // } else {
  //   *p_xc = 0;
  //   *p_yc = 0;
  // }

  int8_t kernel_size = 25;
  int16_t heigth = img->h;
  int16_t width = img->w;
  int16_t kernel_cnt = 0;

  int8_t sensitivity = 170;
  int8_t lower_black = 41;
  int8_t upper_white = 183;
  int16_t threshold = 255;

  int16_t x = 0;
  int16_t y = 0;
  int16_t kur_x = 0;
  int16_t kur_y = 0;

  int16_t kernel_centroid = 0;

  int16_t kernel_w_cnt = floor(width/kernel_size);
  int16_t kernel_h_cnt = floor(heigth/kernel_size);

  int16_t vector_array[20] = {0};

  for (int8_t y_k = 0; y_k < kernel_h_cnt; y_k++){
    // for (int8_t x_k = 0; x_k < kernel_w_cnt; x_k++){

      int8_t state = 0;

      for (int8_t x_k = kernel_w_cnt-1; x_k >= 0; x_k--){

      kernel_cnt = 0;
      kur_x = kernel_size*x_k;
      kur_y = kernel_size*y_k;
      for (int8_t i = 0; i < kernel_size; i++){
        for (int8_t j = 0; j < kernel_size; j++){
          x = kur_x + j;
          y = kur_y + i;

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

          // if ((*up < sensitivity) && (*vp < sensitivity) && (*yp > lower_black) && (*yp < upper_white)){
          if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
            if (draw){
              *yp = 255;  // make pixel brighter in image
            }
            kernel_cnt++;
          }          
        }
      }
      //add break when ready
      //TODO
      if (kernel_cnt > threshold){
        if (state == 0) {
          state = 1;
          kernel_centroid = kernel_size * x_k + floor(kernel_size/2);
          // PRINT("Vector length %d\n", kernel_centroid);
          // PRINT("Yk value %d\n", y_k);
          vector_array[y_k] = kernel_centroid;
        }
      }
    }
    if (state == 0) {
      state = 1;
      kernel_centroid = 0;
      // PRINT("Vector length %d\n", kernel_centroid);
      // PRINT("Yk value %d\n", y_k);
      vector_array[y_k] = kernel_centroid;
    }
  }
  if (draw){
    int16_t max = 0;
    int8_t vector_count = 0;
    int8_t flr = floor(kernel_size/2);
    for (int16_t y = flr; y < (img->h - kernel_size); y+= kernel_size){
      max = vector_array[vector_count];
      if (max<0) {
        max = 0;
      }
      if (max > img->w) {
        max = img->w;
      }
      for(int16_t x = 0; x < max; x++){
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
        *up = 0;
        *vp = 255;
        *yp = 125;
      }
      vector_count++;
    }
  }
    //add 0,0 to list
    // kernel_centroid = 0;
    // vector_array[y_k] = kernel_centroid;


  //DEBUG LOOP



  cnt = 1;
  // PRINT("CNT IN CV_OBS = %d", cnt);

  int16_t vector_x = 4;
  int16_t vector_y = 20;

  // PRINT("Vector x IN CV_OBS = %d", vector_x);
  // PRINT("Vector y IN CV_OBS = %d", vector_y);

  cnt = vector_array[10];

  int16_t T_x = 50;
  int16_t T_y = 100;
  float T_mid = 10.0*kernel_size - 12;
  float alpha = T_x/(0.5 * T_y);
  float beta1 = T_x - alpha*(T_mid);
  float beta2 = T_x + alpha*(T_mid);

  if (draw){
    for (int16_t y = int(T_mid - T_y/2); y < int(T_mid + T_y/2); y++){
      if (y > T_mid) {
        int16_t x = int(y*-1.0*alpha + beta2);
      }
      else {
        int16_t x = int(y*alpha + beta1);
      }
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
        *up = 128;
        *vp = 128;
        *yp = 128;
      }
    }
  }


  int16_t length = 0;
  int16_t pos = 0;
  for (int8_t i = 0; i < 20; i++){
    int8_t pot_max = vector_array[i];
    if (pot_max > length){
      length = pot_max;
      pos = i;
    }
  }
  vector_y = (pos * kernel_size) + floor(kernel_size/2);
  return cnt, length, vector_y;
}

uint16_t triangle_shape(uint16_t y) {

  // uint16_t T_x = 100;
  // uint16_t T_y = 200;
  // uint16_t mid = 9*25 + 12;
  // if 



  return 0;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  //local_filters[0].vector_x = 3;
  //local_filters[0].vector_y = 4;
  //local_filters[0].color_count = 60;

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        local_filters[0].vector_x, local_filters[0].vector_y, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        local_filters[1].vector_x, local_filters[1].vector_y, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}
