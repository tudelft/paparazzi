/*
 * Copyright (C) 2018 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/mission/mission_path.h
 *  @brief mission path planner
 *
 *  Efficient mission path planner for
 *  avoidance algorithms
 */

#ifndef MISSION_PATH_H
#define MISSION_PATH_H

#include "std.h"
#include "subsystems/datalink/datalink.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"

struct mission_path_elem_t {
  struct EnuCoor_i wp;
  uint16_t id;
};

extern void mission_path_init(void);
extern bool mission_path_run(void);
extern bool mission_path_parse_ADD(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);
extern bool mission_path_parse_DELETE(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

#endif /* MISSION_PATH_H */