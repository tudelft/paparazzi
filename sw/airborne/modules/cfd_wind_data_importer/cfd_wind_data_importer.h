/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
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

/** @file "modules/world_env_req_sender/world_env_req_sender.h"
 * @author Sunyou Hwang <S.Hwang-1@tudelft.nl>
 * Sending IVY request to import CFD wind data
 */

#ifndef CFD_WIND_DATA_IMPORTER_H
#define CFD_WIND_DATA_IMPORTER_H

#include "std.h"

extern int wind_speed;

extern void init_cfd_wind_data_importer(void);
//extern void init_cfd_wind_data_importer_ivy(void);
extern void cfd_wind_data_importer_periodic(void);
extern void cfd_wind_importer_parse_wind_msg(uint8_t *buf);
extern void cfd_wind_importer_move_waypoint_msg_cb(uint8_t *buf);

#endif  // CFD_WIND_DATA_IMPORTER_H
