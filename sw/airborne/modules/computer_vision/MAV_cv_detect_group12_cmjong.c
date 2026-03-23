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
#include "modules/computer_vision/MAV_plant_avoider.h"

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

/* Keep plant mask visible by default, even if generic detector draw is toggled off at runtime. */
#ifndef GREEN_OBJECT_DETECTOR_DRAW
#define GREEN_OBJECT_DETECTOR_DRAW false
#endif

#ifndef ORANGE_OBJECT_DETECTOR_DRAW
#define ORANGE_OBJECT_DETECTOR_DRAW false
#endif

// Filter Settings
uint8_t orange_lum_min = 0;
uint8_t orange_lum_max = 0;
uint8_t orange_cb_min = 0;
uint8_t orange_cb_max = 0;
uint8_t orange_cr_min = 0;
uint8_t orange_cr_max = 0;

uint8_t blue_lum_min = 0;
uint8_t blue_lum_max = 0;
uint8_t blue_cb_min = 0;
uint8_t blue_cb_max = 0;
uint8_t blue_cr_min = 0;
uint8_t blue_cr_max = 0;

uint8_t green_lum_min = 0;
uint8_t green_lum_max = 0;
uint8_t green_cb_min = 0;
uint8_t green_cb_max = 0;
uint8_t green_cr_min = 0;
uint8_t green_cr_max = 0;

bool cod_draw = false;

float weight_orange_detector = 0;
float weight_green_detector = 0;
float weight_optical_flow = 0;


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
  /*
  ----------------------------------------------------------------------------------------------------------------
  Add you function below here
  ----------------------------------------------------------------------------------------------------------------
  */

  const bool draw_orange_mask = cod_draw || ORANGE_OBJECT_DETECTOR_DRAW;
  PixelCount orange_count = color_detection(img,
                                            orange_lum_min, orange_lum_max,
                                            orange_cb_min, orange_cb_max,
                                            orange_cr_min, orange_cr_max,
                                            draw_orange_mask);

  uint32_t plant_left = 0U;
  uint32_t plant_middle = 0U;
  uint32_t plant_right = 0U;
  const bool draw_green_mask = cod_draw || GREEN_OBJECT_DETECTOR_DRAW;
  plant_avoider_detect_losses(img, draw_green_mask, &plant_left, &plant_middle, &plant_right);


   /*
  ----------------------------------------------------------------------------------------------------------------
  Weighted function below here 
  ----------------------------------------------------------------------------------------------------------------
  */
  int16_t weighted_left = (int16_t)(weight_orange_detector * (float)orange_count.left +
                                    weight_green_detector * (float)plant_left);
  int16_t weighted_middle = (int16_t)(weight_orange_detector * (float)orange_count.middle +
                                      weight_green_detector * (float)plant_middle);
  int16_t weighted_right = (int16_t)(weight_orange_detector * (float)orange_count.right +
                                     weight_green_detector * (float)plant_right);

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


/*
-------------function that is called once--------------------------------------------------------------------------
*/
void MAV_cv_detect_group12_cmjong_init(void)
{
  memset(global_message, 0, 2*sizeof(struct cv_detect_message));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA
#ifdef ORANGE_OBJECT_DETECTOR_LUM_MIN
  orange_lum_min = ORANGE_OBJECT_DETECTOR_LUM_MIN;
  orange_lum_max = ORANGE_OBJECT_DETECTOR_LUM_MAX;
  orange_cb_min  = ORANGE_OBJECT_DETECTOR_CB_MIN;
  orange_cb_max  = ORANGE_OBJECT_DETECTOR_CB_MAX;
  orange_cr_min  = ORANGE_OBJECT_DETECTOR_CR_MIN;
  orange_cr_max  = ORANGE_OBJECT_DETECTOR_CR_MAX;
#endif

#ifdef BLUE_OBJECT_DETECTOR_LUM_MIN
  blue_lum_min = BLUE_OBJECT_DETECTOR_LUM_MIN;
  blue_lum_max = BLUE_OBJECT_DETECTOR_LUM_MAX;
  blue_cb_min  = BLUE_OBJECT_DETECTOR_CB_MIN;
  blue_cb_max  = BLUE_OBJECT_DETECTOR_CB_MAX;
  blue_cr_min  = BLUE_OBJECT_DETECTOR_CR_MIN;
  blue_cr_max  = BLUE_OBJECT_DETECTOR_CR_MAX;
#endif

#ifdef GREEN_OBJECT_DETECTOR_LUM_MIN
  green_lum_min = GREEN_OBJECT_DETECTOR_LUM_MIN;
  green_lum_max = GREEN_OBJECT_DETECTOR_LUM_MAX;
  green_cb_min  = GREEN_OBJECT_DETECTOR_CB_MIN;
  green_cb_max  = GREEN_OBJECT_DETECTOR_CB_MAX;
  green_cr_min  = GREEN_OBJECT_DETECTOR_CR_MIN;
  green_cr_max  = GREEN_OBJECT_DETECTOR_CR_MAX;
#endif

#ifdef WEIGHT_ORANGE_DETECTOR
  weight_orange_detector = WEIGHT_ORANGE_DETECTOR;
  weight_green_detector = WEIGHT_GREEN_DETECTOR;
  weight_optical_flow = WEIGHT_OPTICAL_FLOW;
#endif

#ifdef COLOR_OBJECT_DETECTOR_DRAW
  cod_draw = COLOR_OBJECT_DETECTOR_DRAW;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
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
