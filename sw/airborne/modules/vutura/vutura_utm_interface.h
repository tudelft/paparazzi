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
 * @file "modules/vutura/vutura_utm_interface.h"
 * @author Dennis van Wijngaarden
 * Sends over position and speed data to vutura utm module
 */

#ifndef VUTURA_UTM_INTERFACE_H
#define VUTURA_UTM_INTERFACE_H

#include <stdint.h>

union paparazzi_to_vutura_msg_t {
	struct {
		int32_t lon;
		int32_t lat;
		int32_t alt;
		int32_t Vn;
		int32_t Ve;
		int32_t Vd;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union paparazzi_to_vutura_msg_t PaparazziToVuturaMsg;

extern void init_vutura_utm_interface(void);
extern void parse_gps(void);
extern void avoid_check(void);

#endif

