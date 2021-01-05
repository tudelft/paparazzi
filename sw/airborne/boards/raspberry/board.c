/*
 * Copyright (C) 2020
 *
 * This file is part of paparazzi.
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
 * @file boards/raspberry/board.c
 * Raspberry specific camera settings.
 * TODO: should this even be a .c file?
 *
 */

// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include "mcu.h"

#include "modules/computer_vision/lib/v4l/v4l2.h"
#include "peripherals/video_device.h"

struct video_config_t front_camera = {
  .output_size = {
    .w = 1280,
    .h = 720
  },
  .sensor_size = {
    .w = 1280,
    .h = 720
  },
  .crop = {
    .x = 0,
    .y = 0,
    .w = 1280,
    .h = 720
  },
  .dev_name = "/dev/video0",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 10,
  .filters = 0,
  .cv_listener=NULL,
  .fps = 0
};
