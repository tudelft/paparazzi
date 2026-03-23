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
 * @file modules/computer_vision/cv_detect_color_object.h
 * Assumes the color_object consists of a continuous color and checks
 * if you are over the defined color_object or not
 */

#ifndef MAV_CV_DETECT_GROUP12_CMJONG_H
#define MAV_CV_DETECT_GROUP12_CMJONG_H

#include <stdint.h>
#include <stdbool.h>
#include "modules/computer_vision/opticflow/opticflow_calculator.h"

// Color filter settings
extern uint8_t orange_lum_min;
extern uint8_t orange_lum_max;
extern uint8_t orange_cb_min;
extern uint8_t orange_cb_max;
extern uint8_t orange_cr_min;
extern uint8_t orange_cr_max;

extern uint8_t blue_lum_min;
extern uint8_t blue_lum_max;
extern uint8_t blue_cb_min;
extern uint8_t blue_cb_max;
extern uint8_t blue_cr_min;
extern uint8_t blue_cr_max;

extern uint8_t green_lum_min;
extern uint8_t green_lum_max;
extern uint8_t green_cb_min;
extern uint8_t green_cb_max;
extern uint8_t green_cr_min;
extern uint8_t green_cr_max;

extern bool cod_draw;

extern float threshold_orange_detector;
extern float threshold_green_detector;

// Crop settings (GCS-tunable)
extern float crop_h_frac;
extern float crop_w_frac;

// Optical flow settings
extern float luke_of_divergence_threshold;
extern bool  luke_of_show_stream_overlay;
extern bool  luke_of_derotation;
extern float luke_of_ema_alpha;
extern float luke_of_smoothed_divergence;
extern struct opticflow_t luke_of_opticflow[];

// OF reset request (set by fast controller after large avoidance turn)
extern bool luke_of_request_reset;

// Module functions
extern void MAV_cv_detect_group12_cmjong_init(void);
extern void MAV_cv_detect_group12_cmjong_periodic(void);

#endif /* COLOR_OBJECT_DETECTOR_CV_H */
