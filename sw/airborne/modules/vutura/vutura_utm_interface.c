/*
 * Copyright (C) Dennis van Wijngaarden
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
 * @file "modules/vutura/vutura_utm_interface.c"
 * @author Dennis van Wijngaarden
 * Sends over position and speed data to vutura utm module
 */

#include "modules/vutura/vutura_utm_interface.h"
#include "state.h"

#include <stdio.h>
#include <stdlib.h>

#define VUTURA_UTM_INTERFACE_VERBOSE 1

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if VUTURA_UTM_INTERFACE_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

void parse_gps(void)
{
	struct LlaCoor_i *lonlatalt = stateGetPositionLla_i();
	struct NedCoor_f *speed_ned = stateGetSpeedNed_f();
	int32_t lon = lonlatalt->lon; // [deg e7]
	int32_t lat = lonlatalt->lat; // [deg e7]]
	int32_t alt = lonlatalt->alt; // [m]
	int32_t Vn  = speed_ned->x * 1000.; // [mm/s]
	int32_t Ve  = speed_ned->y * 1000.; // [mm/s]
	int32_t Vd  = speed_ned->z * 1000.; // [mm/s]

	VERBOSE_PRINT("lon %i [degE7], lat %i [degE7], alt %i [mm] \n", lon, lat, alt);
	VERBOSE_PRINT("Vn %i [mm/s], Ve %i [mm/s], Vd %i [mm/s] \n", Vn, Ve, Vd);
	return;
}


