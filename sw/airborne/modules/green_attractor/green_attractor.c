/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/green_attractor/green_attractor.c"
 * @author Group2
 * Example on how to use the colours detected to avoid obstacles in the cyberzoo
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of green pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the GREEN_ATTRACTOR_VISUAL_DETECTION_ID setting.
 */

#include "modules/green_attractor/green_attractor.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

// #define GREEN_AVOIDER_VERBOSE TRUE
#define GREEN_ATTRACTOR_VERBOSE TRUE


#define PRINT(string,...) fprintf(stderr, "[green_attractor->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GREEN_ATTRACTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t moveWaypointAcross(uint8_t waypoint, float distanceMeters, float heading_increment);
static uint8_t calculateAcross(struct EnuCoor_i *new_coor, float distanceMeters, float heading_increment);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseIncrementAvoidance(void);
static uint8_t MeanderIncrement(void);
static int8_t chooseIncrementSign(void);


enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

/* Green Detection
 * oa_color_count_frac: threshold fraction of green that triggers loss in confidence
 * meander_frac: threshold fraction ar which to already change heading regardless of confidence
 * meander_increment: heading increment for meandering 
 */


float oa_color_count_frac = 0.87f;
float meander_frac = 0.57f;
int meander_increment = 7;

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;                // green color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 6; // number of consecutive negative object detections to be sure we are obstacle free

// global vars for logging distance covered
float last_pos_x = 0;
float last_pos_y = 0;
float dx = 0;
float dy = 0;
float d_covered = 0;

// global vars for object center identification
int16_t green_center_x = 0;
int16_t green_center_y = 0;

// global vars for FPS logging
float current_time = 0;
float last_time = 0;
float FPS_green_attractor = 0;

// global var for meander maneuvre
bool safeflight = false;

// global vars for confidence tuning
int confidence_increment = 2;
int confidence_decrement = 4;

// track state
int current_state = 2;

// turn 90 deg when losing all color
float strong_turn_threshold = 0.1;

int SFSH_increment = 5;


/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */

// Decide later whether to add a new msg for green_attractor
#ifndef GREEN_ATTRACTOR_VISUAL_DETECTION_ID
#define GREEN_ATTRACTOR_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

// callback - extracts color_count and center pixels from object
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
  green_center_x = pixel_x;
  green_center_y = pixel_y;

  // Get FPS
  current_time = get_sys_time_float();
  FPS_green_attractor = 1/(current_time-last_time);
  last_time = current_time;
}


/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void green_attractor_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(GREEN_ATTRACTOR_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void green_attractor_periodic(void)
{
  current_state = (int)navigation_state;
  VERBOSE_PRINT("center of green  y = %i\n", green_center_y);
  VERBOSE_PRINT("FPS = %f\n", FPS_green_attractor);
  VERBOSE_PRINT("obstacle_free_confidence = %i\n", obstacle_free_confidence);
  VERBOSE_PRINT("current_state = %i\n", current_state);

  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }
  int px_filterbox = (filterbox_ymax-filterbox_ymin) * (filterbox_xmax-filterbox_xmin);
  // compute current color thresholds
  // for entire image: int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  // for current filterbox: int32_t color_count_threshold = oa_color_count_frac * 10 * 320;
  int32_t color_count_threshold = oa_color_count_frac * px_filterbox;

  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);


  // update our safe confidence using color threshold
  if(color_count > color_count_threshold){
    obstacle_free_confidence += confidence_increment;
  } else if(color_count > meander_frac*px_filterbox && safeflight == true){ // if we already see object, start yawing in flight
    MeanderIncrement();
    increase_nav_heading(heading_increment);
    moveWaypointForward(WP_TRAJECTORY, 1.5f * fminf(maxDistance, (0.2f * 6) + 0.2));
    // moveWaypointAcross(WP_TRAJECTORY, 1.5f , heading_increment);
    safeflight = false;
  } else {
    obstacle_free_confidence -= confidence_decrement;  // be more cautious with positive obstacle detections
  }
  

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, (0.2f * obstacle_free_confidence)); // what's the 0.2? (was added to 2nd argument)

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // select new search direction
      chooseIncrementAvoidance();
      increase_nav_heading(heading_increment);
      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:

      if(color_count < strong_turn_threshold*px_filterbox)
      {
        VERBOSE_PRINT("TURN 90");
        increase_nav_heading(heading_increment/abs(heading_increment)*90);
      }
      else{
        heading_increment = heading_increment/abs(heading_increment)*SFSH_increment;

        increase_nav_heading(heading_increment);
        moveWaypointAcross(WP_TRAJECTORY, 1.5f , heading_increment); // check +10
        // make sure we have a couple of good readings before declaring the way safe
      }


      if (obstacle_free_confidence >= 2){ // tweak
        safeflight = true;
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:

      // moveWaypointForward(WP_TRAJECTORY, 0.2f);

      // Test
      heading_increment = heading_increment/abs(heading_increment)*5;
      increase_nav_heading(heading_increment);

      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

void log_distance_covered_periodic(void)
{
  if(!autopilot_in_flight()){
    return;
  }
  // calculate distance covered
  dx = fabs(stateGetPositionEnu_f()->x - last_pos_x);
  dy = fabs(stateGetPositionEnu_f()->y - last_pos_y);
  d_covered = d_covered + sqrt(dx*dx+dy*dy);
  VERBOSE_PRINT("distance covered: d = %f \n", d_covered);

  // Update position
  last_pos_x = stateGetPositionEnu_f()->x;
  last_pos_y = stateGetPositionEnu_f()->y;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}
uint8_t moveWaypointAcross(uint8_t waypoint, float distanceMeters, float heading_increment)
{
  struct EnuCoor_i new_coor;
  calculateAcross(&new_coor, distanceMeters, heading_increment);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}
uint8_t calculateAcross(struct EnuCoor_i *new_coor, float distanceMeters, float heading_increment)
{
  float heading  = stateGetNedToBodyEulers_f()->psi + RadOfDeg(heading_increment*0.5f);
  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m across position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' to positive/negative.
 * if 
 */
uint8_t chooseIncrementAvoidance(void)
{ 
  if (green_center_y > 0 && green_center_y < 25) {
    heading_increment = -15.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  
  if (green_center_y > 25 && green_center_y < 50){
    heading_increment = -10.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }

  if (green_center_y < 0 && green_center_y > -25) {
    heading_increment = 15.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  
  if (green_center_y < -25 && green_center_y > -50){
    heading_increment = 10.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }

  return false;
}


uint8_t MeanderIncrement(void)
{
  if (green_center_y > 0) {
    heading_increment = -1*meander_increment;
    VERBOSE_PRINT("Meander increment to: %f\n", heading_increment);
  } else {
    heading_increment = meander_increment;
    VERBOSE_PRINT("Meander increment to: %f\n", heading_increment);
  }
  return false;
}

int8_t chooseIncrementSign(void)
{
  return (green_center_y <= 0) ? 1 : -1;
}