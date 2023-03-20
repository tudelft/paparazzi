/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"


#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#define WIDTH_2_PROCESS 200
#define HEIGHT_2_PROCESS 50


float output_flow[2];

struct image_t *optical_flow_func(struct image_t *img, int camera_id);
struct image_t *optical_flow_func(struct image_t *img, int camera_id)
{
    // action act = STANDBY;
    if (img->type == IMAGE_YUV422) {
        farneback((char *) img->buf, output_flow, WIDTH_2_PROCESS, HEIGHT_2_PROCESS, img->w, img->h);
    }
    return img;
}

void calc_action_optical_flow_init(void)
{
    cv_add_to_device(&OPENCVDEMO_CAMERA, optical_flow_func, OPENCVDEMO_FPS, 0);
    output_flow[0] = 0.0;
    output_flow[1] = 0.0;
}

void calc_action_optical_flow_periodic(void)
{
    printf("out: %f",output_flow[0]);
}

// PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

// // Function
// struct image_t *opencv_func(struct image_t *img, uint8_t camera_id);
// struct image_t *opencv_func(struct image_t *img, uint8_t camera_id)
// {

//   if (img->type == IMAGE_YUV422) {
//     // Call OpenCV (C++ from paparazzi C function)
//     opencv_example((char *) img->buf, img->w, img->h);
//   }

// // opencv_example(NULL, 10,10);

//   return NULL;
// }

// void opencvdemo_init(void)
// {
//   cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func, OPENCVDEMO_FPS, 0);
// }

