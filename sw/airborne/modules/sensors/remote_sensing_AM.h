/*
 * Copyright (C) 2025 Alessandro Mancinelli
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
#include "std.h"

#ifndef REMOTE_SENSING_AM_H
#define REMOTE_SENSING_AM_H

#define FALCON_MODE_NONE 0x00       ///< 
#define FALCON_MODE_SIXDOF 0x01     ///< 
#define FALCON_MODE_RELANGLE 0x02   ///< 
#define FALCON_MODE_RELBEACON 0x03  ///< 

//define the AzimuthElevation structure: 
struct AzimuthElevation {
  float azimuth; // azimuth angle in rad
  float elevation; // elevation angle in rad
};

extern float falcon_relangle_distance; // Falcon sensor distance in meters
extern uint8_t falcon_mode; 

extern void remote_sensing_AM_init(void); 
extern void remote_sensing_AM_periodic(void);
extern void remote_sensing_parse_target_pos(uint8_t *buf);
extern void remote_sensing_AM_send_falcon_cmd(uint8_t unk);
extern void remote_sensing_parse_falcon_sixdof(uint8_t *buf);
extern void remote_sensing_parse_falcon_relangle(uint8_t *buf);
extern void remote_sensing_parse_falcon_relbeacon(uint8_t *buf);
extern void remote_sensing_parse_opencv_aruco(uint8_t *buf);

#endif /* REMOTE_SENSING_H */

