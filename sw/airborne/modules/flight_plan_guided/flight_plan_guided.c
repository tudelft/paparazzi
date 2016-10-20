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
#include "modules/stereocam/stereocam2state/stereocam2state.h"

// start and stop modules
#include "generated/modules.h"

#define ANGLE_BUILDING_ENTRY  -0.594  // angle of drone to enter building
#define ANGLE_ROOM_1_ENTRY     0.  // angle to enter room 1
#define ANGLE_ROOM_2_ENTRY     0.  // angle to enter room 2
#define ANGLE_ROOM_3_ENTRY     0.  // angle to enter room 3

float marker_err = 0;
bool marker_lost;
bool approach_white_building = false;
uint32_t max_pixel_building = 1000; //TODO: CALIBRATE THIS!

#include "subsystems/abi.h"
struct range_finders_ range_finders;
bool do_lr_avoidance = true;
bool do_wall_following = false;
bool front_wall_detected = false;
bool disable_sideways_forcefield = false;

#ifndef RANGE_SENSORS_ABI_ID
#define RANGE_SENSORS_ABI_ID ABI_BROADCAST
#endif
static abi_event range_sensors_ev;
static void range_sensors_cb(uint8_t sender_id,
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left);

void flight_plan_guided_init(void)
{
  marker_lost = true;
  AbiBindMsgRANGE_SENSORS(RANGE_SENSORS_ABI_ID, &range_sensors_ev, range_sensors_cb);
} // Dummy


/* Kill throttle */
uint8_t KillEngines(void) {
    autopilot_set_motors_on(FALSE);

    return false;
}


/* Start throttle */
uint8_t StartEngines(void) {
    autopilot_set_motors_on(TRUE);

    return false;
}


/* Reset the altitude reference to the current GPS alt if GPS is used */
uint8_t ResetAlt(void) {ins_reset_altitude_ref(); return false;}

bool TakeOff(float climb_rate) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    guidance_v_set_guided_vz(-climb_rate);
    guidance_h_set_guided_body_vel(0, 0);

    return false;
}

bool WaitUntilAltitude(float altitude) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (stateGetPositionEnu_f()->z < altitude) { return true; }

    return false;
}

bool RotateToHeading(float heading) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_h_set_guided_heading(heading);

  return false;
}

uint8_t Hover(float altitude) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }
    // Horizontal velocities are set to zero
    guidance_h_set_guided_body_vel(0, 0);
    guidance_v_set_guided_z(-altitude);

    return false;
}

/* Move forward */
uint8_t MoveForward(float vx) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (autopilot_mode == AP_MODE_GUIDED) {
        guidance_h_set_guided_body_vel(vx, 0);
    }
    return false;
}

/* Move Right */
uint8_t MoveRight(float vy) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (autopilot_mode == AP_MODE_GUIDED) {
        guidance_h_set_guided_body_vel(0, vy);
    }
    return false;
}



bool Land(float end_altitude) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  //gh_set_max_speed(float max_speed)
  //gh_set_max_speed(GUIDANCE_H_REF_MAX_SPEED);

    // return true if not completed

    //For bucket
    guidance_v_set_guided_vz(0.4);
    //For landing pad
//    guidance_v_set_guided_vz(1.5);
    guidance_h_set_guided_pos(marker1.geo_location.x, marker1.geo_location.y);

    if (stateGetPositionEnu_f()->z > end_altitude) {
        return true;
    }

    return false;
}

bool front_cam_set_x_offset(int offset) {
  mt9f002.offset_x = offset;
  mt9f002_set_resolution(&mt9f002);

  return false;
}

static int FRONT_MARKER_HEADING_MARGIN = 60;  // px
static float FRONT_MARKER_HEADING_RATE = 0.5; // rad/s

bool front_marker_heading_change(void) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (marker2.detected) {
    marker_lost = false;
    if (!marker2.processed) {
      // Marker detected and not processed
      marker2.processed = true;
      int relative_heading = marker2.pixel.y - 320;

      if (relative_heading > FRONT_MARKER_HEADING_MARGIN) {
        // Marker is to the right
        guidance_h_set_guided_heading_rate(FRONT_MARKER_HEADING_RATE);
      } else if (relative_heading < -FRONT_MARKER_HEADING_MARGIN) {
        // Marker is to the left
        guidance_h_set_guided_heading_rate(-FRONT_MARKER_HEADING_RATE);
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
    marker_lost = true;
    guidance_h_set_guided_heading_rate(FRONT_MARKER_HEADING_RATE);
  }

  return true;
}

static int FRONT_MARKER_POSITION_MARGIN = 100; // > 50, SO DISABLED
static int FRONT_MARKER_POSITION_MARGIN_LOST = 50;
static float FRONT_MARKER_DRIFT_CORRECTION_RATE = 0.05;
static float FRONT_MARKER_APPROACH_SPEED_HIGH = 0.2;
static float FRONT_MARKER_APPROACH_SPEED_LOW = 0.05;

bool front_marker_approach(void) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (marker2.detected) {
    marker_lost = false;
    if (!marker2.processed) {
      marker2.processed = true;
      int relative_pos = marker2.pixel.y - 320;

      if (abs(relative_pos) > FRONT_MARKER_POSITION_MARGIN_LOST) {
        fprintf(stderr, "[FRONT_MARKER] OUT OF FRAME.\n");
        marker_lost = true;
      } else if (relative_pos > FRONT_MARKER_POSITION_MARGIN) {
        fprintf(stderr, "[FRONT_MARKER] RIGHT.\n");
        guidance_h_set_guided_body_vel(FRONT_MARKER_APPROACH_SPEED_LOW, FRONT_MARKER_DRIFT_CORRECTION_RATE);
      } else if (relative_pos < -FRONT_MARKER_POSITION_MARGIN) {
        fprintf(stderr, "[FRONT_MARKER] LEFT.\n");
        guidance_h_set_guided_body_vel(FRONT_MARKER_APPROACH_SPEED_LOW, -FRONT_MARKER_DRIFT_CORRECTION_RATE);
      } else {
        fprintf(stderr, "[FRONT_MARKER] CENTER.\n");
        guidance_h_set_guided_body_vel(FRONT_MARKER_APPROACH_SPEED_HIGH, 0.0);

        return false;
      }
    } else {
      fprintf(stderr, "[FRONT_MARKER] ALREADY PROCESSED.\n");
      // Marker detected but already processed
      // ** just wait **
    }
  } else {
    fprintf(stderr, "[FRONT_MARKER] NOT DETECTED.\n");
    // Marker not detected
    marker_lost = true;
  }

  // Loop this function
  return true;
}


#define ERR_MIN 0.05
#define ERR_MAX 0.30
#define ERR_GAIN (1.0 / (ERR_MAX - ERR_MIN))

bool marker_center_descent(float x_offset, float z_speed, float end_altitude) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (end_altitude != 0 && stateGetPositionEnu_f()->z < end_altitude) {
      return false;
  }

  struct Marker *marker = &marker1;

  guidance_h_set_guided_heading_rate(0.);

  if (marker->detected) {
    if (!marker->processed) {
      marker->processed = true;

      float psi = stateGetNedToBodyEulers_f()->psi;

      float offset_x = cosf(-psi) * x_offset;
      float offset_y = sinf(-psi) * x_offset;

      float rel_x = marker->geo_relative.x + offset_x;
      float rel_y = marker->geo_relative.y + offset_y;

      // err < 0.05 = acceptable when landed
      // err < 0.10 = acceptable at low altitude
      // err < 0.30 = caution when landing
      // err > 0.30 = NOT GOOD
      // err > 0.50 = probably lost marker
      marker_err = fabsf(rel_x) + fabsf(rel_y);

      if (z_speed != 0) {
//      fprintf(stderr, "[landing] %.2f, %.2f, %.2f\n", rel_x, rel_y, err);
        float bounded_err = Chop(marker_err, ERR_MIN, ERR_MAX);
        guidance_v_set_guided_vz(z_speed - z_speed * (bounded_err - ERR_MIN) * ERR_GAIN);
      }

      float pos_x = marker->geo_location.x + offset_x;
      float pos_y = marker->geo_location.y - offset_y;

      guidance_h_set_guided_pos(pos_x, pos_y);
    }
  } else {
    guidance_v_set_guided_vz(0);
  }

  // Loop this function
  return true;
}

bool open_gripper(void) {
  uint8_t msg[1]; msg[0] = 0;
  stereoprot_sendArray(&((UART_LINK).device), msg, 1, 1);
  return false;
}

bool close_gripper(void) {
  uint8_t msg[1]; msg[0] = 1;
  stereoprot_sendArray(&((UART_LINK).device), msg, 1, 1);
  return false;
}


int8_t object_state;
int8_t object_retries = 0;

bool go_to_object(bool descent) {
  // If we are not in guided mode
  if (autopilot_mode != AP_MODE_GUIDED) {
    // Reset the approach strategy and loop
    object_state = 0;
    return true;
  }

  fprintf(stderr, "[go_to_object] State %i.\n", object_state);

  switch (object_state) {
    case 0:
      // Initialize

      guidance_v_set_guided_z(-NOM_FLIGHT_ALT);
      object_retries--;

      object_state++; // Go to next state + switch fallthrough
    case 1:
      // Search for the marker with the front camera
      guidance_h_set_guided_body_vel(0., 0.);

      if (marker1.found_time > 0.5) {
        object_state = 3;
        break;
      }

      if (front_marker_heading_change()) {
        // TODO: after 360 degrees of mindless turning move a bit first
        break;
      }

      object_state++; // Go to next state + switch fallthrough
    case 2:
      // Approach marker straight on

      if (marker_lost) {
        object_state = 1;
        break;
      }

      front_marker_approach();

      if (marker1.found_time < 1) {
        break;
      }

      if(approach_white_building)
      {
        if(marker1.pixel_cnt > max_pixel_building) //TODO: calibrate this
        {
          return false; // if returned false it will stop the approach

        }
      }

      object_state++; // Go to next state + switch fallthrough
    case 3:
      // Hover over marker

      if (marker1.found_time < 1) {
        object_state = 1;
        break;
      }

      marker_center_descent(0.1, 0, 0);

      if (marker1.found_time < 4) {
        break;
      }

      // If we don't want to land, job is done and we can continue
      if (!descent && marker_err < ERR_MAX) { return false; }

      object_state++; // Go to next state + switch fallthrough
    case 4:
      // Land on top of marker

      if (marker1.found_time < 4) {
        object_state = 3;
        break;
      }

      marker_center_descent(0.1, 0.4, 0);

      if (marker1.found_time < 1) {
        object_state = 0;
        break;
      }

  }

  return true;
}

int8_t win_state;
// color 0 = red, 1 = blue
bool fly_through_window(uint8_t color) {
  static float mytime = 0;

  if (autopilot_mode != AP_MODE_GUIDED) { win_state = 0; return true; }

  // TODO window lost recovery
  //if (!win_processed) { // this will limit updates to the speed of the stereocam ~12Hz
  //  win_processed = 1;
    switch (win_state){
      case 0:
        //guidance_h_set_guided_heading(ANGLE_BUILDING_ENTRY);
        //guidance_h_set_guided_heading();
        guidance_h_set_guided_pos(stateGetPositionNed_f()->x, stateGetPositionNed_f()->y);
        guidance_v_set_guided_z(-1.7);
        mytime = get_sys_time_float();
        init_pos_filter = 1;
        set_snake_gate_color_filter(color);
        snake_gate_detection_snake_gate_detection_periodic_status = MODULES_START;
        do_lr_avoidance = false;

        win_state++;
        break;
      // centre drone in front of window at about 2m away
      case 1:
        // todo, filter... do I need this?
        /*if(range_finders.front > 0 && range_finders.front < 30) {  // if I get closer than 30cm, move away and retry
          win_state = 4;
          break;
        }*/
        if(gate_detected && gate_processed == 0) {
          if (ready_pass_through){
            win_state++;
            break;
          }

          // position drone 1.5m in front of window, add small low pass filter on position command
          guidance_h_set_guided_pos_relative(0.9*(filtered_x_gate - 1.5), 0.9*filtered_y_gate);
          // align drone perpendicular to gate

           // Way too agressive behavior, need a stable measurement??
           // guidance_h_set_guided_heading_relative(angle_to_gate);
          gate_processed = 1;
        }
        break;
      // fly forward with active control till >0.5m in front of window
      case 2:
        guidance_h_set_guided_pos_relative(filtered_x_gate + 0.5, filtered_y_gate);
        snake_gate_detection_snake_gate_detection_periodic_status = MODULES_STOP;
        mytime = get_sys_time_float();
        win_state++;
        break;
      case 3:
        if (get_sys_time_float() - mytime > 6.) {
          win_state = 0;
          do_lr_avoidance = true;
          return false;
        }
        break;
      case 4: // missed approach, recycle and try again
        guidance_h_set_guided_pos_relative(-1.5, 0.);
        gate_processed = 1;
        win_state = 1;  // try again
        break;
      default:
        mytime = get_sys_time_float();
        win_state = 0;
        snake_gate_detection_snake_gate_detection_periodic_status = MODULES_STOP;
        break;
    }

  return true;
}

static void range_sensor_force_field(float *vel_body_x, float *vel_body_y, int16_t avoid_inner_border, int16_t avoid_outer_border,
    int16_t tinder_range, float min_vel_command, float max_vel_command)
{
  static const int16_t max_sensor_range = 2000;

  int16_t difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_x_command = *vel_body_x;
  float avoid_y_command = *vel_body_y;

  // Balance avoidance command for y direction (sideways)
  if (range_finders.right < 1 || range_finders.right > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.right < avoid_inner_border){
    avoid_y_command -= max_vel_command;
  } else if (range_finders.right < avoid_outer_border) {
    // Linear
    avoid_y_command -= (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.right)
        / (float)difference_inner_outer;
  } else {}

  if (range_finders.left < 1 || range_finders.left > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.left < avoid_inner_border){
    avoid_y_command -= max_vel_command;
  } else if (range_finders.left < avoid_outer_border) {
    // Linear
    avoid_y_command -= (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.left)
        / (float)difference_inner_outer;
  } else {}

  // balance avoidance command for x direction (forward/backward)
  if (range_finders.front < 1 || range_finders.front > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.front < avoid_inner_border){
    avoid_x_command -= max_vel_command;
  } else if (range_finders.front < avoid_outer_border) {
    // Linear
    avoid_x_command -= (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.front)
        / (float)difference_inner_outer;
  } else if(range_finders.front > tinder_range){
    if(do_wall_following){
      avoid_x_command += max_vel_command;
    }
  } else {}


  if (range_finders.back < 1 || range_finders.back > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.back < avoid_inner_border){
    avoid_x_command += max_vel_command;
  } else if (range_finders.back < avoid_outer_border) {
    // Linear
    avoid_x_command += (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.back)
        / (float)difference_inner_outer;
  } else {}

  *vel_body_x = avoid_x_command;
  *vel_body_y = avoid_y_command;
}

static void range_sensors_cb(uint8_t sender_id,
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left)
{
  static int32_t front_wall_detect_counter = 0;
  static const int32_t max_sensor_range = 2000;

  // save range finders values
  range_finders.front = range_front;
  range_finders.right = range_right;
  range_finders.left = range_left;
  range_finders.back = range_back;

  if (range_finders.front > 1) {  // good sensor reading
    if(range_finders.front < max_sensor_range) {  // wall in view
      if(front_wall_detect_counter > 5) { // outlier detection for positive wall detection
        front_wall_detected = true;
      } else {
        front_wall_detect_counter++;
      }
    } else if(--front_wall_detect_counter < 0){  // outlier detection for negative wall detection
      front_wall_detected = false;
      front_wall_detect_counter = 0;
    }
  }

  // add extra velocity command to avoid walls based on range sensors
  float vel_offset_body_x = 0.0f;
  float vel_offset_body_y = 0.0f;

  range_sensor_force_field(&vel_offset_body_x, &vel_offset_body_y, 500, 1000, 1600, 0.0f, 0.3f);
  // printf("front %d, back %d, right %d, left %d\n, vel x %f, vel_y %f",range_finders.front,range_finders.back,range_finders.right,range_finders.left, vel_offset_body_x,vel_offset_body_y);

  if(disable_sideways_forcefield) // disable forcefield for the side if the drone is going through a door for instance
  {
    vel_offset_body_y = 0.0f;
  }
  // calculate velocity offset for guidance
  guidance_h_set_speed_offset(vel_offset_body_x, vel_offset_body_y);
}
