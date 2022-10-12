/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/status/status_nederdrone.h
 * Status information for the Nederdrone logging and datalink
 *
 */

#ifndef STATUS_NEDERDRONE_H
#define STATUS_NEDERDRONE_H

#include "std.h"

void status_nederdrone_init(void);
void status_nederdrone_period(void);

#endif /* STATUS_NEDERDRONE_H */
