/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "lib/vision/image.h"
#include "depth_estimation.h"
#include "modules/computer_vision/cv.h"

#include <stdio.h>

// Define variables from config file
#ifndef DEPTH_ESTIMATION_FPS
#define DEPTH_ESTIMATION_FPS 0
#endif

#ifndef INPUT_DOWN_SAMPLE_FACTOR
#define INPUT_DOWN_SAMPLE_FACTOR 1
#endif

#define PRINT(string, ...) fprintf(stderr, "[depth_estimation->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
  Struct for keeping track of all the module settings
*/
struct depth_estimation depth_estimation = {
  .in_ds_factor = INPUT_DOWN_SAMPLE_FACTOR,
  .in_cam_fps = DEPTH_ESTIMATION_FPS,
};

/*
  Video callback. Processes a camera image when available, and returns a depth map
*/
struct image_t *depth_estimation_cb(struct image_t *img) {
  // Down sample image
  image_create(
    img,
    img->w / depth_estimation.in_ds_factor,
    img->h / depth_estimation.in_ds_factor,
    IMAGE_YUV422
  );

  // Run  neural network using img


  return img; // Return (original / modified) image
}

/*
  Initialize the module
*/
void depth_estimation_init(void) {
  // bind our colorfilter callbacks to receive the color filter outputs
  cv_add_to_device(&DEPTH_ESTIMATION_CAMERA, depth_estimation_cb, depth_estimation.in_cam_fps, 0);
}
