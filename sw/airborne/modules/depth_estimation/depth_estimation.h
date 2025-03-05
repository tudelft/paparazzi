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

#ifndef PAPARAZZI_DEPTH_ESTIMATION_H
#define PAPARAZZI_DEPTH_ESTIMATION_H

struct depth_estimation {
  uint8_t in_ds_factor;
  uint8_t in_cam_fps;
};
extern struct depth_estimation depth_estimation;

struct image_t *depth_estimation_cb(struct image_t *img);

extern void depth_estimation_init(void);

#endif //PAPARAZZI_DEPTH_ESTIMATION_H
