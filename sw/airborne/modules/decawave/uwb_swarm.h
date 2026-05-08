/*
 * Copyright (C) 2026 OpenUAS
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
 * @file "modules/decawave/uwb_swarm.h"
 * @author OpenUAS
 * UWB(Ultra Wide Band) swarm of aircraft inter-communication and relative localization.
 * This without using any Anchors, but instead using the UWB modules on each drone to directly communicate with each other and get relative distance measurements.
 * This module must be used together with a MiniTag PCB with added Decawave DWM1000 running the appropriate Serial Communication code, which can be flashed on the Arduino board.
 * The proper arduino library can be found at:
 *  http//github.com/tudelft/uwb-dw1000-pprz/
 * The accompanying file to flash to the MCU Atmega can be found in
 *  examples/UWB_localization/UWB_localization.ino
 */

#ifndef UWB_SWARM_H_
#define UWB_SWARM_H_

extern void uwb_swarm_init(void);
extern void uwb_swarm_periodic(void);
extern void uwb_swarm_event(void);
extern void uwb_swarm_report(void);

#endif /* UWB_SWARM_H_ */