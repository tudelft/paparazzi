/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/MAV_fast_controller_group12_cmjong/MAV_fast_controller_group12_cmjong.h"
#include "modules/computer_vision/MAV_cv_detect_group12_cmjong.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

#include "generated/flight_plan.h"

#define MAV_FAST_CONTROLLER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[MAV_fast_controller_group12_cmjong->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if MAV_FAST_CONTROLLER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t move_waypoint_forward(uint8_t waypoint, float distance_m);
static uint8_t calculate_forward_position(struct EnuCoor_i *new_coor, float distance_m);
static uint8_t set_waypoint_position(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t rotate_drone_heading(float degrees);
static void hold_current_waypoints(void);

enum navigation_state_t {
  SAFE_AND_WAIT,
  TURN_AVOID,
  SEARCH_FOR_SAFE_HEADING,
  MOVE_FORWARD_WITH_FIXED_DISTANCE,
  GATE_DETECTED,
  OUT_OF_BOUNDS,
};


#define AVOIDANCE_TURN_DEGREES 10.f
#define MOVE_DISTANCE       0.5f
#define AVOIDANCE_TURN_DEGREES_OutOfBound 5.f
#define OF_AVOIDANCE_TURN_DEGREES 120.f
#define GYRO_YAW_RATE_THRESHOLD 0.15f
#define OF_STARTUP_IGNORE_TIME 2.0f

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE_AND_WAIT;

static bool of_obstacle_ahead = false;
static float of_turn_remaining = 0.0f;
static bool was_in_flight = false;
static float of_ignore_until = 0.0f;
uint16_t detected_local = 1;
uint16_t color_count_local = 0;

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef MAV_cmjong_VISUAL_DETECTION_ID
#define MAV_cmjong_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
#ifndef MAV_cmjong_OF_VISUAL_DETECTION_ID
#define MAV_cmjong_OF_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event cv_detect_event;
static abi_event luke_of_event;

static void cv_detection_message_callback(
    uint8_t  __attribute__((unused)) sender_id,
    int16_t  detected,
    int16_t  color_count,
    int16_t  __attribute__((unused)) extra5,
    int16_t  __attribute__((unused)) extra1,
    int32_t  __attribute__((unused)) extra2,
    int16_t  __attribute__((unused)) extra3)
{
  // Safe cast: ABI guarantees int16, loss is always >= 0
  detected_local = detected;
  color_count_local = color_count;
}

static void luke_of_message_callback(
    uint8_t  __attribute__((unused)) sender_id,
    int16_t  __attribute__((unused)) pixel_x,
    int16_t  __attribute__((unused)) pixel_y,
    int16_t  __attribute__((unused)) pixel_width,
    int16_t  __attribute__((unused)) pixel_height,
    int32_t  quality,
    int16_t  __attribute__((unused)) extra)
{
  of_obstacle_ahead = (quality > 0);
}

/*
-------------function that is called once--------------------------------------------------------------------------
*/
void MAV_fast_controller_group12_cmjong_init(void)
{
  AbiBindMsgVISUAL_DETECTION(MAV_cmjong_VISUAL_DETECTION_ID, &cv_detect_event, cv_detection_message_callback);
  AbiBindMsgVISUAL_DETECTION(MAV_cmjong_OF_VISUAL_DETECTION_ID, &luke_of_event, luke_of_message_callback);
}

/*
-------------------Function that is cald every .. hz that send info to the fast controller--------------------------
*/
void MAV_fast_controller_group12_cmjong_periodic(void)
{
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight()) {
    was_in_flight = false;
    navigation_state = SAFE_AND_WAIT;
    of_obstacle_ahead = false;
    of_turn_remaining = 0.0f;
    return;
  }

  float now = get_sys_time_float();
  if (!was_in_flight) {
    was_in_flight = true;
    of_ignore_until = now + OF_STARTUP_IGNORE_TIME;
    of_obstacle_ahead = false;
    of_turn_remaining = 0.0f;
    luke_of_request_reset = true;
  }

  if (now < of_ignore_until) {
    of_obstacle_ahead = false;
  }

  // Gyro-based rotation lock: suppress new OF triggers while already rotating
  if (fabsf(stateGetBodyRates_f()->r) > GYRO_YAW_RATE_THRESHOLD) {
    of_obstacle_ahead = false;
  }

  VERBOSE_PRINT("State: %d | detected: %u, %u | of: %d | of_turn_rem: %.1f | of_grace: %.1f\n",
    navigation_state, detected_local, color_count_local, of_obstacle_ahead, of_turn_remaining,
    fmaxf(0.0f, of_ignore_until - now));


  switch (navigation_state) {

    case SAFE_AND_WAIT:
      // Hold position, wait for cv_detect to get a reading
      hold_current_waypoints();

      if (detected_local == 0 && !of_obstacle_ahead) {
        navigation_state = MOVE_FORWARD_WITH_FIXED_DISTANCE;
      } else if (detected_local > 0 || of_obstacle_ahead) {
        if (of_obstacle_ahead && detected_local == 0) {
          of_turn_remaining = OF_AVOIDANCE_TURN_DEGREES;
        }
        navigation_state = TURN_AVOID;
      }
      break;

    case TURN_AVOID:
      if (of_turn_remaining > 0.f) {
        // OF-triggered turn: execute fixed rotation incrementally
        float step = (of_turn_remaining < AVOIDANCE_TURN_DEGREES)
                     ? of_turn_remaining : AVOIDANCE_TURN_DEGREES;
        rotate_drone_heading(step);
        of_turn_remaining -= step;
        if (of_turn_remaining <= 0.f) {
          of_turn_remaining = 0.f;
          luke_of_request_reset = true;
          navigation_state = SAFE_AND_WAIT;
        }
      } else if (of_obstacle_ahead) {
        // New OF detection: start large turn
        of_turn_remaining = OF_AVOIDANCE_TURN_DEGREES;
        rotate_drone_heading(AVOIDANCE_TURN_DEGREES);
        of_turn_remaining -= AVOIDANCE_TURN_DEGREES;
      } else if (detected_local > 0) {
        // Color-triggered: small incremental turn
        rotate_drone_heading(AVOIDANCE_TURN_DEGREES);
      } else {
        navigation_state = SAFE_AND_WAIT;
      }
      break;


    case MOVE_FORWARD_WITH_FIXED_DISTANCE:
    {
      struct EnuCoor_i next_coor;

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;

      } else if (detected_local > 0 || of_obstacle_ahead) {
        navigation_state = SAFE_AND_WAIT;

      } else {
        calculate_forward_position(&next_coor, MOVE_DISTANCE);

        if (!InsideObstacleZone(POS_FLOAT_OF_BFP(next_coor.x), POS_FLOAT_OF_BFP(next_coor.y))) {
          navigation_state = OUT_OF_BOUNDS;

        } else {
          set_waypoint_position(WP_TRAJECTORY, &next_coor);
          set_waypoint_position(WP_GOAL, &next_coor);
        }
      }
      break;
    }

    case OUT_OF_BOUNDS:
      // Rotate and probe until back inside arena
      rotate_drone_heading(AVOIDANCE_TURN_DEGREES_OutOfBound);
      move_waypoint_forward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        /* CHANGED: removed second rotate_drone_heading call here —
           the single rotation per tick is enough; double-rotating on the 
           re-entry tick caused unpredictable heading jumps */
        navigation_state = SAFE_AND_WAIT;
      }
      break;

    default:
      break;
  }
}


static uint8_t rotate_drone_heading(float degrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(degrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  // VERBOSE_PRINT("Rotating heading by %.1f deg, new heading: %.1f deg\n",
  //   degrees, DegOfRad(new_heading));
  return false;
}


static uint8_t move_waypoint_forward(uint8_t waypoint, float distance_m)
{
  struct EnuCoor_i new_coor;
  calculate_forward_position(&new_coor, distance_m);
  set_waypoint_position(waypoint, &new_coor);
  return false;
}

static uint8_t calculate_forward_position(struct EnuCoor_i *new_coor, float distance_m)
{
  float heading = stateGetNedToBodyEulers_f()->psi;
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distance_m);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distance_m);
  // VERBOSE_PRINT("Calculated %.2f m forward: x=%.2f y=%.2f from pos(%.2f, %.2f) heading=%.1f deg\n",
  //   distance_m,
  //   POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
  //   stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y,
  //   DegOfRad(heading));
  return false;
}


static uint8_t set_waypoint_position(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // VERBOSE_PRINT("Setting waypoint %d to x=%.2f y=%.2f\n",
    // waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

static void hold_current_waypoints(void)
{
  waypoint_move_here_2d(WP_GOAL);
  waypoint_move_here_2d(WP_TRAJECTORY);
}
