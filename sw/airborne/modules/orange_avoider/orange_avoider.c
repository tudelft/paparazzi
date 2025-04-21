/**
 * @file   modules/orange_avoider/orange_avoider.c
 * @author Roland Meertens
 * @author Kevin Malkow (modifications)
 *
 * @brief  Obstacle avoidance using orange color detection.
 *
 * Implements a color-based obstacle avoidance strategy for the AE4317 Autonomous Flight of Micro Air Vehicles course at 
 * TU Delft. Relies on the `cv_detect_color_object` module to detect orange objects (e.g., poles) in the environment.
 *
 * Functionality:
 *  - Subscribes to pixel count data via ABI (AirBorne Ivy) messaging from a selected `cv_detect_color_object` filter.
 *  - Compares detected orange pixel count to `oa_color_count_frac` threshold for obstacle detection.
 *  - Tracks obstacle-free confidence (`obstacle_free_confidence`).
 *  - Uses a navigation state machine (`SAFE`, `OBSTACLE_FOUND`, etc.) to manage avoidance maneuvers.
 *  - Adjusts navigation (heading/waypoints) based on state and confidence.
 *
 * Configuration:
 *  Requires the `cv_detect_color_object` module (`sw/airborne/modules/computer_vision/cv_detect_color_object.c`)
 *  to be active and configured for orange detection. Color configuration involves setting Y (Luminance/Brightness), 
 *  Cb (Chrominance blue/Blue–yellow deviation), and Cr (Chrominance red/Red–cyan deviation) thresholds.
 *
 *  These thresholds are defined in the `bebop_course_orangeavoid.xml` airframe file 
 *  (`conf/airframes/tudelft/bebop_course_orangeavoid.xml`) and should be adjusted according to the conditions. For example:
 *    ```
 *    <define name="COLOR_OBJECT_DETECTOR_LUM_MIN1" value="30"/>
 *    <define name="COLOR_OBJECT_DETECTOR_LUM_MAX1" value="190"/>
 *    <define name="COLOR_OBJECT_DETECTOR_CB_MIN1" value="70"/>
 *    <define name="COLOR_OBJECT_DETECTOR_CB_MAX1" value="130"/>
 *    <define name="COLOR_OBJECT_DETECTOR_CR_MIN1" value="150"/>
 *    <define name="COLOR_OBJECT_DETECTOR_CR_MAX1" value="190"/>
 *    ```
 *
 *  `cv_detect_color_object` can run multiple filters, each publishing results on a distinct ABI message ID 
 *  (e.g., `COLOR_OBJECT_DETECTION1_ID`). The `ORANGE_AVOIDER_VISUAL_DETECTION_ID` directive in the airframe file selects which
 *  ABI message stream this module subscribes to. In `bebop_course_orangeavoid.xml`, this is set to `COLOR_OBJECT_DETECTION1_ID`:
 *    ```
 *    <define name="ORANGE_AVOIDER_VISUAL_DETECTION_ID" value="COLOR_OBJECT_DETECTION1_ID"/>
 *    ```
 *  Ensure this ID matches the filter configured for orange detection.
 *
 * Copyright (C) Roland Meertens, Kevin Malkow (modifications)
 * This module is part of Paparazzi UAV.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C
#include "generated/flight_plan.h"


// Verbose output toggle
#define ORANGE_AVOIDER_VERBOSE TRUE
#define PRINT(string, ...) fprintf(stderr, "[orange_avoider->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Internal helper function prototypes
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

// Navigation states
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// Define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0; // Orange color detection count
int16_t obstacle_free_confidence = 0; // Confidence measure of obstacle-free path

// Configurable variables
float oa_color_count_frac = 0.18f; // Fraction of image required to be orange to count as obstacle
const int16_t max_trajectory_confidence = 5; // Number of consecutive zero object detections required to confirm obstacle-free path
float heading_increment = 5.f; // Heading angle increment in degrees
float maxDistance = 2.25f; // Maximum waypoint displacement in meters

// ABI binding for visual detection
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
  #define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;

/**
 * @brief  
 * Callback function triggered by an ABI (AirBorne Ivy) messaging event (http://wiki.paparazziuav.org/wiki/ABI) 
 * whenever new data is published by the subscribed module, in this case, orange detections by the `cv_detect_color_object` 
 * module. In the Paparazzi architecture, modules operate independently and asynchronously. ABI provides a standardized, 
 * event-driven mechanism for inter-module communication. This callback is automatically triggered upon each 
 * new detection by the `cv_detect_color_object` module, but only after the ABI event has been bound 
 * to this function in the initialization function of the `orange_avoider` module.
 * 
 * @param  sender_id     ID of the module that triggered the event (unused)
 * @param  pixel_x       X-coordinate of the detected blob in the image (unused)
 * @param  pixel_y       Y-coordinate of the detected blob in the image (unused)
 * @param  pixel_width   Width of the detected blob in pixels (unused)
 * @param  pixel_height  Height of the detected blob in pixels (unused)
 * @param  quality       Detection quality metric (orange-pixel count)
 * @param  extra         Additional metadata (unused)
 */ 
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

/**
 * @brief  Initializes the orange-avoider module. This function binds the module to the ABI message for color detection, 
 * sets the color filter, initializes the random seed, and selects a random heading increment for avoidance behavior.
 * @param  None
 * @return void
 */
void orange_avoider_init(void)
{
  srand(time(NULL)); // Seed the random number generator
  chooseRandomIncrementAvoidance(); // Randomly select a heading increment for avoidance
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID,
                             &color_detection_ev,
                             color_detection_cb); // Bind the ABI message to the callback function
}
/**
 * @brief  Periodic state machine for obstacle avoidance. 
 * The function evaluates the current state of the system and determines whether it is safe to move forward.
 * It adjusts the waypoint, heading, or navigational state accordingly based on the presence of obstacles detected 
 * through the color filter. The state machine transitions between four states: SAFE, OBSTACLE_FOUND, 
 * SEARCH_FOR_SAFE_HEADING, and OUT_OF_BOUNDS, and it handles waypoint movement and heading adjustments.
 * 
 * @param  None
 * 
 * @return void
 */
void orange_avoider_periodic(void)
{
  if (!autopilot_in_flight()) {
    return;
  }

  // Compute colour thresholds
  int32_t color_count_threshold = oa_color_count_frac 
                                * front_camera.output_size.w 
                                * front_camera.output_size.h;

  VERBOSE_PRINT("Color Count: %d  Threshold: %d State: %d \n", 
                 color_count, color_count_threshold, navigation_state);

  // Update obstacle-free confidence based on color detection
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2; // Decrease confidence if obstacle detected (the higher the decrease, 
                                   // the more sensitive the system is to obstacles)
  }
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence); // Ensure confidence is within bounds (should not be negative 
                                                                 // or above max_trajectory_confidence)

  // Calculate the distance to move based on confidence
  float moveDist = fminf(maxDistance,
    0.2f * obstacle_free_confidence); 

  // State machine for navigation
  switch (navigation_state){
    case SAFE: {
      // Advance trajectory waypoint based on confidence
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDist);

      // Check 1: Is the new trajectory waypoint outside the obstacle zone?
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),
                              WaypointY(WP_TRAJECTORY)))
      {
        // Outside zone, transition to recovery state
        navigation_state = OUT_OF_BOUNDS;
      }
      // Check 2: If inside zone, is obstacle detection confidence zero?
      else if (obstacle_free_confidence == 0) {
        // Confidence is zero (obstacle detected), transition to avoidance state
        navigation_state = OBSTACLE_FOUND;
      }
      // If inside zone and confidence > 0 (path appears clear):
      else {
        // Continue advancing GOAL and RETREAT waypoints.
        moveWaypointForward(WP_GOAL,    moveDist);
        moveWaypointForward(WP_RETREAT, -moveDist); // Retreat moves backward
      }
      break;
    } 

    case OBSTACLE_FOUND: {
      // Stop movement by setting all navigation waypoints to the current position
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // Choose a random direction (clockwise or counter-clockwise) for the heading search
      chooseRandomIncrementAvoidance();

      // Transition to the state where the drone rotates to find a safe direction
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;
    }

    case SEARCH_FOR_SAFE_HEADING: {
      // Increment the navigation heading by the chosen avoidance increment
      increase_nav_heading(heading_increment);

      // Check if obstacle-free confidence is high enough to consider the current heading safe 
      // If yes, transition back to the SAFE state to resume forward movement
      if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      break;
    } 

    case OUT_OF_BOUNDS: {
      // Continue rotating and moving waypoints to navigate back towards the zone
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      moveWaypointForward(WP_RETREAT, -1.0f);

      // Check if the trajectory waypoint is now inside the obstacle zone
      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // Add an additional heading change to help point back into the arena
        increase_nav_heading(heading_increment);

        // Reset confidence as the safety of the path ahead from this new position is unknown
        obstacle_free_confidence = 0;

        // Ensure the direction is safe before resuming normal movement
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    } 

    default: {
      // Default case to catch any unexpected navigation_state values. No action is taken, just break out of the switch.
      break;
    }
  }
}

/**
 * @brief  Adjust navigation heading by a given increment.
 * @param  incrementDegrees  Degrees to turn (signed).
 * @return Always returns false.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  // Calculate new heading by adding increment to current heading
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // Normalize heading to the range [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // Update the navigation system's target heading (defined in navigation.h)
  nav.heading = new_heading;

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/**
 * @brief  Calculate coordinates forward and move a waypoint.
 * @param  waypoint        The ID of the waypoint to move.
 * @param  distanceMeters  Distance to move forward in meters.
 * @return Always returns false.
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  // Calculate the target coordinates at the specified distance forward
  calculateForwards(&new_coor, distanceMeters);
  // Move the specified waypoint to the calculated coordinates
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/**
 * @brief  Calculate new coordinates at a distance forward based on current position and heading.
 * @param  new_coor        Pointer to structure to store the calculated coordinates.
 * @param  distanceMeters  Distance forward in meters.
 * @return Always returns false.
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  // Get the current heading
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Calculate the new position using trigonometry based on current position and heading
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/**
 * @brief  Move the specified waypoint to the new coordinates calculated based on the current position and heading.
 * @param  waypoint  The ID of the waypoint to move.
 * @param  new_coor  Pointer to structure containing the new coordinates.
 * @return Always returns false.
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  // Move the waypoint to the new coordinates
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/**
 * @brief  Randomly set the heading increment for avoidance.
 * @return Always returns false.
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose either a positive (clockwise) or negative (counter-clockwise) increment
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}