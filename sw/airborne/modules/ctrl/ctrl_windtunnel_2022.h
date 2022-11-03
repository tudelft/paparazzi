/*
 * Copyright (C) 2022 Tomaso De Ponti <tomasodp@gmail.com>
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

/** @file "modules/ctrl/ctrl_windtunnel_2022.h"
 * @author Tomaso De Ponti <tomasodp@gmail.com>
 * Module to test actuator effectiveness of rotwingdrone
 */

#ifndef CTRL_WINDTUNNEL_2022_H
#define CTRL_WINDTUNNEL_2022_H
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
//extern void windtunnel_control_init(void);
//extern void windtunnel_message_init(void);
extern void manual_test_periodic(void);
extern bool windtunnel_control(void);
extern bool skew_moment(void);
void sync_procedure(void);
bool excitation_control(void);
bool as_control(void);
bool mot_status_control(void);
bool wing_skew_control(void);
extern bool test_active;
extern bool test_skew_active;
//extern int16_t mot_status[3][4];
//extern int16_t as_static[];
extern bool static_test;
extern bool manual_test;
extern int16_t mot0_static;
extern int16_t mot1_static;
extern int16_t mot2_static;
extern int16_t mot3_static;
extern int16_t ailL_static;
extern int16_t ailR_static;
extern int16_t ele_static ;
extern int16_t rud_static ;
extern int16_t push_static ;

#endif  // CTRL_WINDTUNNEL_2022_H
