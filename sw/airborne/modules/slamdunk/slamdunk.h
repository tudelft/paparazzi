/*
 * Copyright (C) Charlelie M
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
 * @file "modules/slamdunk/slamdunk.h"
 * @author Charlelie M
 * Uses the information from the slamdunk's depth map to move forward without hitting walls
 */

#ifndef SLAMDUNK_H
#define SLAMDUNK_H

#include "pprzlink/pprz_transport.h"

/* Main slamdunk strcuture */
struct slamdunk_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                   ///< If we received a message
};

extern void slamdunk_init(void);
extern void slamdunk_parse_IMCU_LR_MEAN_DIST(void);

#endif

