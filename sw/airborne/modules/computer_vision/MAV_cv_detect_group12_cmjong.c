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
#include "modules/computer_vision/MAV_cv_detect_group12_cmjong.h"
#include "modules/computer_vision/MAV_cv_color_group12_cmjong.h"

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

// define global variables: this is the function that the information is stored that is send to the fast controller.
struct cv_detect_message {
  int16_t  loss_left;
  int16_t  loss_middle;
  int16_t  loss_right;
  bool updated;
};
struct cv_detect_message global_message[2];



/*
------------function that is called for every time a new image is made----------------------------------------------

includes:
  - weighted function
  - calls the color detection function
  - calls the edge detection function
  - ...
 */
static struct image_t *object_detector(struct image_t *img, uint8_t camera_id)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (camera_id){
    case 0:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 1:
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


  /*
  ----------------------------------------------------------------------------------------------------------------
  Add you function below here
  ----------------------------------------------------------------------------------------------------------------
  */

  PixelCount count = orange_detection(img, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, TRUE);
  // count.left, count.middle, count.right
  //VERBOSE_PRINT("Orange pixel count: %u left , %u middle , %u right", Count.left , Count.middle, Count.right);


   /*
  ----------------------------------------------------------------------------------------------------------------
  Weighted function below here 
  ----------------------------------------------------------------------------------------------------------------
  */
  int16_t weighted_left  = 1 * (int16_t)count.left;
  int16_t weighted_middle = 1 * (int16_t)count.middle;
  int16_t weighted_right  = 1 * (int16_t)count.right;

  pthread_mutex_lock(&mutex);
  global_message[camera_id].loss_left   = weighted_left;
  global_message[camera_id].loss_middle = weighted_middle;
  global_message[camera_id].loss_right  = weighted_right;
  global_message[camera_id].updated = TRUE;
  pthread_mutex_unlock(&mutex);

  return img;
}





/*
------------init for the function that is called every time a new image is made/recieved----------------------------
*/
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 0);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

/*
-------------function that is called once--------------------------------------------------------------------------
*/
void MAV_cv_detect_group12_cmjong_init(void)
{
  memset(global_message, 0, 2*sizeof(struct cv_detect_message));
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
-------------------Function that is cald every .. hz that send info to the fast controller--------------------------
*/
void MAV_cv_detect_group12_cmjong_periodic(void)
{
  static struct cv_detect_message local_message[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_message, global_message, 2*sizeof(struct cv_detect_message));
  global_message[0].updated = false;           
  global_message[1].updated = false;
  pthread_mutex_unlock(&mutex);

  if(local_message[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID,
    local_message[0].loss_left,
    local_message[0].loss_middle,
    local_message[0].loss_right,
    0, 0, 0);

    local_message[0].updated = false;
  }

  if(local_message[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID,
    local_message[1].loss_left,
    local_message[1].loss_middle,
    local_message[1].loss_right,
    0, 0, 0);
    
    local_message[1].updated = false;
  }
}
