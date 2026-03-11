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
 * Assumes the color_object consists of a continuous color and checks
 * if you are over the defined color_object or not
 */

#ifndef COLOR_OBJECT_DETECTOR_CV_H
#define COLOR_OBJECT_DETECTOR_CV_H

#include <stdint.h>
#include <stdbool.h>

// Module settings
extern uint8_t orange_lum_min;  
extern uint8_t orange_lum_max;
extern uint8_t orange_cb_min;
extern uint8_t orange_cb_max;
extern uint8_t orange_cr_min;
extern uint8_t orange_cr_max;

extern uint8_t green_lum_min;
extern uint8_t green_lum_max;
extern uint8_t green_cb_min;
extern uint8_t green_cb_max;
extern uint8_t green_cr_min;
extern uint8_t green_cr_max;

extern uint8_t orange_percent;
extern uint8_t green_percent;

extern bool cod_draw;

// Module functions
extern void color_object_detector_init(void);
extern void color_object_detector_periodic(void);

#endif /* COLOR_OBJECT_DETECTOR_CV_H */
