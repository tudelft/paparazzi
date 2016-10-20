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
  int16_t front;  // mm
  int16_t right;  // mm
  int16_t left;   // mm
  int16_t back;   // mm
};

extern float marker_err;
extern bool marker_lost;
extern bool approach_white_building;
extern uint32_t max_pixel_building;

extern struct range_finders_ range_finders;

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
extern bool LiftOff(float throttle);

extern bool WaitUntilAltitude(float altitude);
extern bool WaitUntilSpeedOrAltitude(float speed, float fail_altitude);
extern bool ResetSpecialTimer(void);
extern bool WaitUntilTimerOrAltitude(float sec, float fail_altitude);
extern bool Land(float end_altitude);

extern bool close_gripper(void);
extern bool open_gripper(void);

extern bool front_marker_heading_change(void);
extern bool front_marker_approach(void);

extern bool marker_center_descent(float x_offset, float z_speed, float end_altitude);

extern bool fly_through_window(uint8_t color);
extern int8_t win_state;

extern bool go_to_object(bool descent);
extern int8_t object_state;
extern int8_t object_retries;


extern bool front_cam_set_x_offset(int offset);

extern bool front_wall_detected;
extern bool do_wall_following;
extern bool disable_sideways_forcefield;
extern bool range_sensor_wall_following_between_doors(float travel_time);

#endif
