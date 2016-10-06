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
 * @file modules/computer_vision/flight_plan_guided.c
 * @author IMAV 2016
 */

#include "modules/flight_plan_guided/flight_plan_guided.h"
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>

#include "modules/computer_vision/marker/detector.h"

#include "mcu_periph/uart.h"
#include "modules/stereocam/stereocam.h"
#include "modules/stereocam/stereoprotocol.h"

bool marker_lost;

#include "subsystems/abi.h"
#ifndef RANGE_SENSORS_ABI_ID
#define RANGE_SENSORS_ABI_ID ABI_BROADCAST
#endif
static abi_event range_sensors_ev;
static void range_sensors_cb(uint8_t sender_id __attribute__((unused)),
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left);


void flight_plan_guided_init(void)
{
  marker_lost = true;

  AbiBindMsgRANGE_SENSORS(RANGE_SENSORS_ABI_ID, &range_sensors_ev, &range_sensors_cb);
} // Dummy


/* Kill throttle */
uint8_t KillEngines(void)
{
  autopilot_set_motors_on(FALSE);

  return false;
}


/* Start throttle */
uint8_t StartEngines(void)
{
  autopilot_set_motors_on(TRUE);

  return false;
}


/* Reset the altitude reference to the current GPS alt if GPS is used */
uint8_t ResetAlt(void) {if (autopilot_mode == AP_MODE_GUIDED) { ins_reset_altitude_ref(); } return false;}


bool TakeOff(float climb_rate)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_v_set_guided_vz(-climb_rate);
  guidance_h_set_guided_body_vel(0, 0);

  return false;
}

bool WaitUntilAltitude(float altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z < altitude) { return true; }

  return false;
}

bool RotateToHeading(float heading)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_h_set_guided_heading(heading);

  return false;
}

uint8_t Hover(float altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }
  // Horizontal velocities are set to zero
  guidance_h_set_guided_body_vel(0, 0);
  guidance_v_set_guided_z(-altitude);

  return false;
}

/* Move forward */
uint8_t MoveForward(float vx)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(vx, 0);
  }
  return false;
}

/* Move Right */
uint8_t MoveRight(float vy)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(0, vy);
  }
  return false;
}



bool Land(float end_altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  // return true if not completed

  //For bucket
//    guidance_v_set_guided_vz(0.2);
  //For landing pad
  guidance_v_set_guided_vz(1.5);
  guidance_h_set_guided_body_vel(0, 0);


  if (stateGetPositionEnu_f()->z > end_altitude) {
    return true;
  }

  return false;
}

static int BUCKET_HEADING_MARGIN = 60;  // px
static float BUCKET_HEADING_RATE = 0.5; // rad/s

bool bucket_heading_change(void)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_h_set_guided_body_vel(0., 0.);

  if (marker2.detected) {
    marker_lost = false;
    if (!marker2.processed) {
      // Marker detected and not processed
      marker2.processed = true;
      int relative_heading = marker2.pixel.y - 320;

      if (relative_heading > BUCKET_HEADING_MARGIN) {
        // Marker is to the right
        guidance_h_set_guided_heading_rate(BUCKET_HEADING_RATE);
      } else if (relative_heading < -BUCKET_HEADING_MARGIN) {
        // Marker is to the left
        guidance_h_set_guided_heading_rate(-BUCKET_HEADING_RATE);
      } else {
        // Marker is more or less centered
        guidance_h_set_guided_heading_rate(0.);
        return false;
      }
    } else {
      // Marker detected but already processed
      // ** just wait **
    }
  } else {
    // Marker not detected
    guidance_h_set_guided_heading_rate(BUCKET_HEADING_RATE);
  }

  return true;
}

static int BUCKET_POSITION_MARGIN = 45;
static int BUCKET_POSITION_MARGIN_LOST = 200;
static float BUCKET_DRIFT_CORRECTION_RATE = 0.1;
static float BUCKET_APPROACH_SPEED_HIGH = 0.1;
static float BUCKET_APPROACH_SPEED_LOW = 0.05;

bool bucket_approach(void)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (marker1.detected) {
    // Hand over control to next stage
    return false;
  }

  if (marker2.detected) {
    marker_lost = false;
    if (!marker2.processed) {
      marker2.processed = true;
      int relative_pos = marker2.pixel.y - 320;

      if (abs(relative_pos) > BUCKET_POSITION_MARGIN_LOST) {
        fprintf(stderr, "[bucket] OUT OF FRAME.\n");
        marker_lost = true;
      } else if (relative_pos > BUCKET_POSITION_MARGIN) {
        fprintf(stderr, "[bucket] RIGHT.\n");
        guidance_h_set_guided_body_vel(BUCKET_APPROACH_SPEED_LOW, BUCKET_DRIFT_CORRECTION_RATE);
      } else if (relative_pos < -BUCKET_POSITION_MARGIN) {
        fprintf(stderr, "[bucket] LEFT.\n");
        guidance_h_set_guided_body_vel(BUCKET_APPROACH_SPEED_LOW, -BUCKET_DRIFT_CORRECTION_RATE);
      } else {
        fprintf(stderr, "[bucket] CENTER.\n");
        guidance_h_set_guided_body_vel(BUCKET_APPROACH_SPEED_HIGH, 0.0);
      }
    } else {
      fprintf(stderr, "[bucket] ALREADY PROCESSED.\n");
      // Marker detected but already processed
      // ** just wait **
    }
  } else {
    fprintf(stderr, "[bucket] NOT DETECTED.\n");
    // Marker not detected
    marker_lost = true;
  }

  if (marker_lost) {
    guidance_h_set_guided_body_vel(0., 0.);
  }

  // Loop this function
  return true;
}

bool bucket_center(void)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (marker1.detected) {
    if (!marker1.processed) {
      marker1.processed = true;

      struct EnuCoor_f *speed = stateGetSpeedEnu_f();

      if (marker1.found_time > 5 && marker1.mid && abs(speed->x) < 0.1 && abs(speed->y) < 0.1) {
        return false;
      }

      guidance_h_set_guided_pos(marker1.geo_location.x, marker1.geo_location.y);
    }
  } else {
    marker_lost = true;
  }

  // Loop this function
  return true;
}

bool open_gripper(void)
{
  uint8_t msg[1]; msg[0] = 0;
  stereoprot_sendArray(&((UART_LINK).device), msg, 1, 1);
  return false;
}

bool close_gripper(void)
{
  uint8_t msg[1]; msg[0] = 1;
  stereoprot_sendArray(&((UART_LINK).device), msg, 1, 1);
  return false;
}


void range_sensor_force_field(float *vel_body_x, float *vel_body_y, struct range_finders_ range_finders,
                              int16_t avoid_inner_border, int16_t avoid_outer_border, float min_vel_command, float max_vel_command)
{

  int16_t difference_inner_outer = avoid_outer_border - avoid_inner_border;


  // Velocity commands
  float avoid_x_command = *vel_body_x;
  float avoid_y_command = *vel_body_y;

  // Balance avoidance command for y direction (sideways)

  if (range_finders.right < avoid_outer_border) {
    if (range_finders.right > avoid_inner_border) {
      avoid_y_command -= (max_vel_command - min_vel_command) *
                         ((float)avoid_outer_border - (float)range_finders.right)
                         / (float)difference_inner_outer;
    } else {
      avoid_y_command -= max_vel_command;
    }
  }
  if (range_finders.left < avoid_outer_border) {
    if (range_finders.left > avoid_inner_border) {
      avoid_y_command += (max_vel_command - min_vel_command) *
                         ((float)avoid_outer_border - (float)range_finders.left)
                         / (float)difference_inner_outer;
    } else {
      avoid_y_command += max_vel_command;
    }
  }

  // balance avoidance command for x direction (forward/backward)
  if (range_finders.front < avoid_outer_border) {
    //from stereo camera TODO: add this once the stereocamera is attached
    if (range_finders.front > avoid_inner_border)
      avoid_y_command += (max_vel_command - min_vel_command) *
                         ((float)avoid_outer_border - (float)range_finders.front)
                         / (float)difference_inner_outer;
  } else {
    avoid_y_command += max_vel_command;
  }


  if (range_finders.back < avoid_outer_border) {
    if (range_finders.back > avoid_inner_border) {
      avoid_x_command += (max_vel_command - min_vel_command) *
                         ((float)avoid_outer_border - (float)range_finders.back)
                         / (float)difference_inner_outer;
    } else {
      avoid_x_command += max_vel_command;
    }
  }

  *vel_body_x = avoid_x_command;
  *vel_body_y = avoid_y_command;
}

static void range_sensors_cb(uint8_t sender_id __attribute__((unused)),
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left)
{

  struct range_finders_ range_finders;
// save range finders values
  range_finders.front = range_front;
  range_finders.right = range_right;
  range_finders.left = range_left;
  range_finders.back = range_back;


//add extra velocity command to avoid walls based on range sensors
  float vel_offset_body_x = 0.0f;
  float vel_offset_body_y = 0.0f;

  range_sensor_force_field(&vel_offset_body_x, &vel_offset_body_y, range_finders, 800, 1200, 0.0f, 0.3f);

// calculate velocity offset for guidance
  guidance_h_set_speed_offset(vel_offset_body_x, vel_offset_body_y);

}
