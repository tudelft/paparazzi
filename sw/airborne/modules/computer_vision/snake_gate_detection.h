/*
 * Copyright (C) 2016
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
 * @file modules/computer_vision/snake_gate_detection.h
 */

#ifndef SNAKE_GATE_DETECTION_CV_PLUGIN_H
#define SNAKE_GATE_DETECTION_CV_PLUGIN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "lib/vision/gate_detection.h"

// Module functions
extern void snake_gate_detection_init(void);
extern void snake_gate_detection_start(void);
extern void snake_gate_detection_stop(void);
extern void snake_gate_detection_periodic(void);

#define SNAKE_DOOR 0
#define SNAKE_WINDOW 1
//uint16_t image_yuv422_set_color(struct image_t *input, struct image_t *output, int x, int y);
//void check_color_center(struct image_t *im, uint8_t *y_c, uint8_t *cb_c, uint8_t *cr_c)

extern float gate_x_dist;
extern float gate_y_dist;
extern float gate_z_dist;
extern float angle_to_gate;
extern float filtered_x_gate;
extern float filtered_y_gate;
extern float filtered_z_gate;
extern int gate_detected;
extern int gate_processed;
extern int ready_pass_through;
extern int init_pos_filter;
extern float decide_time_to_go_through_window;
extern float gate_pos_margin;
extern float gate_speed_margin;

extern void set_snake_gate_color_filter(uint8_t color);

#endif /* SNAKE_GATE_DETECTION_CV_PLUGIN_H */
