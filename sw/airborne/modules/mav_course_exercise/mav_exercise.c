/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "mav_exercise.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot_static.h"
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#ifndef BIGDIV;
#define BIGDIV 1;
#endif

//green threshold
#ifndef GREE ;
#define GREE 5000;
#endif

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

uint8_t increase_nav_heading(float incrementDegrees);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  OUT_OF_BOUNDS,
  HOLD,
  SAFE_HEADING
};

// define and initialise global variables
float oa_color_count_frac = 0.18f;
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
// float moveDistance = 2;   // waypoint displacement [m]
float maxDistance = 2.25;           
float oob_haeding_increment = 30.f;      // heading angle increment if out of bounds [deg]
float heading_increment = 30.f;
const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
float divergence_imp= 0.f;
int bigdiv = BIGDIV;
int32_t div_counter = 0;
float div_thresh = 0.06f;
int32_t floor_count = 0;
int32_t floor_centroid = 0;
int32_t obstacle_free_confidence_orange = 0;
float green_thresh = GREE;
// float oa_color_count_frac = 0.18f;

//float div_thresholder = 5.f;
// needed to receive output from a separate module running on a parallel process
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef ORANGE_AVOIDER_OPTICAL_FLOW_ID
#define ORANGE_AVOIDER_OPTICAL_FLOW_ID ABI_BROADCAST
#endif


//static abi_event color_detection_ev;
//static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
//                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
 //                              int16_t __attribute__((unused)) pixel_width,
  //                             int16_t __attribute__((unused)) pixel_height,
   //                            int32_t quality, int16_t __attribute__((unused)) extra) {
  //color_count = quality;
//}

static abi_event optics_flower_ev;
static void optics_flower_cb(uint8_t __attribute__((unused)) sender_id,
                             uint32_t __attribute__((unused))  flow_x, uint32_t __attribute__((unused))  flow_y,
                             uint32_t __attribute__((unused))  flow_der_x, uint32_t __attribute__((unused))  flow_der_y,
                             float __attribute__((unused)) quality, float size_divergence) {
  divergence_imp = size_divergence;                            
}

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}



void mav_exercise_init(void) {
  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgOPTICAL_FLOW(ORANGE_AVOIDER_OPTICAL_FLOW_ID, &optics_flower_ev, optics_flower_cb);
}

void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying
  
  if (!autopilot_in_flight()) {
    return;
  }

  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  //int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  
  //PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  PRINT("Divergence: %.3f  threshold: %.3f state: %d \n", divergence_imp, div_thresh ,navigation_state);
  //int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  //update our safe confidence using color threshold

  // if(color_count < color_count_threshold){
  //   obstacle_free_confidence_orange++;
  // } else {
  //   obstacle_free_confidence_orange -= 2;  // be more cautious with positive obstacle detections
  // }

  if (divergence_imp < div_thresh) {
    obstacle_free_confidence ++;
  }
  else if (color_count < green_thresh){
    obstacle_free_confidence -=1;
  }
   else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);
PRINT("Green count: %d \n threshold: %d \n", color_count, green_thresh);

  switch (navigation_state) {
    case SAFE:
      // printf("periodic fucntions fucks up SAFE");
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;}
      else if (color_count < green_thresh){
        moveWaypointForward(WP_TRAJECTORY, 0.f);
          div_counter = 0;
          navigation_state = OBSTACLE_FOUND;
      }
      else if (obstacle_free_confidence <= 1) {
        moveWaypointForward(WP_TRAJECTORY, 0.f);
        navigation_state = OBSTACLE_FOUND;
      }else if (divergence_imp > div_thresh) {
         div_counter++; 
            if (div_counter >= bigdiv){
              div_counter = 0;
              navigation_state=OBSTACLE_FOUND;}
      }
      
       else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;
    case OBSTACLE_FOUND:
      // TODO Change behavior
      // stop as soon as obstacle is found
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      //moveWaypointForward(WP_TRAJECTORY, 0.f);
      chooseRandomIncrementAvoidance();

      
      navigation_state = SAFE_HEADING;
      
      break;
    case SAFE_HEADING:
      increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      // if (divergence_imp < div_thresh){
      //   navigation_state = SAFE;
      // }
      
      if (color_count < green_thresh){
        moveWaypointForward(WP_TRAJECTORY, 0.f);
        navigation_state = OBSTACLE_FOUND;
      }
      else if (divergence_imp > div_thresh){
        moveWaypointForward(WP_TRAJECTORY, 0.f);
        navigation_state = OBSTACLE_FOUND;
      }
      else{navigation_state = SAFE;}
      break;
    case OUT_OF_BOUNDS:
      // stop
      // waypoint_move_here_2d(WP_GOAL);
      // waypoint_move_here_2d(WP_TRAJECTORY);

      increase_nav_heading(oob_haeding_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        // add offset to head back into arena
        increase_nav_heading(oob_haeding_increment);
        navigation_state = SAFE;
      }
      break;
    case HOLD:
    default:
      break;
  }
  // printf("periodic fucntions fucks up");
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees) {
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

uint8_t chooseRandomIncrementAvoidance(void)
{
  
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = -20.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
     heading_increment = -20.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}
