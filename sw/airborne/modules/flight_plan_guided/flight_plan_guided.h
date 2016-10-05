/*
 * Copyright (C) 2016 - IMAV 2016
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file modules/computer_vision/marker_tracking.h
 * @author IMAV 2016
 */

#ifndef FLIGHT_PLAN_GUIDED_PLUGIN_H
#define FLIGHT_PLAN_GUIDED_PLUGIN_H

#include <std.h>

struct range_finders_ {
  int16_t front;
  int16_t right;
  int16_t left;
  int16_t back;
};

extern bool marker_lost;

// Module functions
void flight_plan_guided_init(void);

// Flight Plan functions
extern uint8_t KillEngines(void);
extern uint8_t StartEngines(void);
extern uint8_t ResetAlt(void);
extern uint8_t Hover(float altitude);
extern uint8_t MoveForward(float vx);
extern uint8_t MoveRight(float vy);

extern bool RotateToHeading(float heading);

extern bool TakeOff(float climb_rate);
extern bool WaitUntilAltitude(float altitude);
extern bool Land(float end_altitude);

extern bool close_gripper(void);
extern bool open_gripper(void);

bool bucket_heading_change(void);
bool bucket_approach(void);
bool bucket_center(void);

extern bool range_sensors_avoid(void);
extern bool range_sensors_wall_following(uint8_t direction);
extern void range_sensor_force_field(float *vel_body_x, float *vel_body_y, struct range_finders_ range_finders,
                                     int16_t avoid_inner_border, int16_t avoid_outer_border, float min_vel_command, float max_vel_command);

#endif
