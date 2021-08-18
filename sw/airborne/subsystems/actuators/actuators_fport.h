/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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
 */

/** @file actuators_fport.h
 *  Fport actuator driver, which can output as 7 fport channels at ~11ms.
 *  Channels min, average and maximum should be: min 0 neutral 1023  max 2047
 */

#ifndef ACTUATORS_FPORT_H
#define ACTUATORS_FPORT_H

#include "std.h"

/* Maximum amount of fport actuator channels */
#define ACTUATORS_FPORT_MAX_NB   16

/* Main actuator structure */
struct ActuatorsFport {
  int32_t cmds[ACTUATORS_FPORT_MAX_NB];
  struct link_device *device;
};

/* Functions used in actuator macros */
extern struct ActuatorsFport actuators_fport;
extern void actuators_fport_init(void);
extern void actuators_fport_set(void);

/* Actuator macros */
#define ActuatorFportSet(_i, _v) { actuators_fport.cmds[_i] = _v; }
#define ActuatorsFportInit() actuators_fport_init()
#define ActuatorsFportCommit() actuators_fport_set()


#endif /* ACTUATORS_FPORT_H */
