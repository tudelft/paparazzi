/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/GROUP_11/GROUP_11.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>
#include "modules/computer_vision/cv_detect_color_object.h"

#define GROUP_11_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUP_11_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  TURNING,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  PLANT_FOUND
};

// define settings
float oag_color_count_frac = 0.9f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
float oag_central_floor_frac = 0.35f;     // central floor threshold to detect object
float oag_plant_frac = 0.08f;              // plant threshold for plant detection

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
int32_t floor_central_count = 0;        // green color count from color filter for floor detection
int32_t plant_count = 0;                  // green color for plant detection
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
int16_t ground_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
int16_t plant_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.

int16_t heading_new = 0;              // used to define if to turn left or right based on the amount of green in the defined areas
const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// Define a new boolean variable at the global scope of the module
static bool ground_calibration_started = false;

// This call back will be used to receive the color count from the orange detector
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

#ifndef FLOOR_VISUAL_DETECTION_ID
#define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

#ifndef GROUND_CENTRAL_VISUAL_DETECTION_ID
#define GROUND_CENTRAL_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event ground_central_detection_ev;
static void ground_central_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_central_count = quality;
}

#ifndef PLANT_VISUAL_DETECTION_ID
#define PLANT_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event plant_detection_ev;
static void plant_count_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  plant_count = quality;
}

#ifndef GROUND_SPLIT_ID
#define GROUND_SPLIT_ID ABI_BROADCAST
#endif
static abi_event ground_detection_ev;
static void ground_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t new_direction)
{
  heading_new = new_direction;
}
/*
 * Initialisation function
 */
void group_11_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  AbiBindMsgVISUAL_DETECTION(GROUND_CENTRAL_VISUAL_DETECTION_ID, &ground_central_detection_ev, ground_central_cb);
  AbiBindMsgVISUAL_DETECTION(PLANT_VISUAL_DETECTION_ID, &plant_detection_ev, plant_count_cb);
  AbiBindMsgGROUND_DETECTION(GROUND_SPLIT_ID, &ground_detection_ev, ground_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void group_11_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    ground_free_confidence = 0;
    plant_free_confidence = 0;
    ground_calibration_started = false;  // Reset this flag if we are not in guided mode
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t central_floor_count_threshold = oag_central_floor_frac * (front_camera.output_size.w * 0.4) * (front_camera.output_size.h * 0.2); // Adjust based on the central area size
  int32_t plant_count_threshold = oag_plant_frac * (front_camera.output_size.w * 0.25) * (front_camera.output_size.h * 0.3); // Adjust based on the central area size
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  // VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor central count: %d, threshold: %d, state: %d\n", floor_central_count, central_floor_count_threshold, navigation_state);
  VERBOSE_PRINT("Plant count: %d, threshold: %d\n", plant_count, plant_count_threshold);
  // VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);
  // Add your debug print statements here
  // VERBOSE_PRINT("Ground Detection Thresholds - Luminance: min=%d, max=%d\n", cod_lum_min2, cod_lum_max2);
  // VERBOSE_PRINT("Ground Detection Thresholds - Chrominance Blue: min=%d, max=%d\n", cod_cb_min2, cod_cb_max2);
  // VERBOSE_PRINT("Ground Detection Thresholds - Chrominance Red: min=%d, max=%d\n", cod_cr_min2, cod_cr_max2);
  // VERBOSE_PRINT("Frame Counter: %d\n", frame_counter);
  // VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  // VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);
  VERBOSE_PRINT("largest green count: %d\n", heading_new);

  // Example condition: Start calibration when in SAFE state and calibration has not yet started
  if (navigation_state == SAFE && !ground_calibration_started) {
      // Call the function from cv_detect_color_object module to start ground calibration
      start_ground_calibration();
      ground_calibration_started = true;  // Prevent this block from running again
  }

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // Update the ground confidence based on the central floor count
  if(floor_central_count < central_floor_count_threshold){
    ground_free_confidence -= 2; // Decrease confidence if not enough ground is detected
  } else {
    ground_free_confidence++; // Increase confidence if enough ground is detected
  }

  // Update the plant confidence based on the plant count
  if(plant_count < plant_count_threshold){
     plant_free_confidence++;
  } else {
    plant_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
  Bound(ground_free_confidence, 0, max_trajectory_confidence);
  Bound(plant_free_confidence, 0, max_trajectory_confidence);

  // float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);
  // Adjust speed based on the lower of obstacle and ground confidences
  // Calculate the minimum confidence among obstacle, ground, and plant detections
  float min_confidence = fminf(fminf(obstacle_free_confidence, ground_free_confidence), plant_free_confidence);
  float speed_sp = fminf(oag_max_speed, 0.2f * min_confidence);
  // float speed_sp = fminf(oag_max_speed, 0.2f * ground_free_confidence);
  float steering_bias = 0.f; // Determines turning direction based on ground detection

  // obstacle_free_confidence == 0 ||
  switch (navigation_state){
    case SAFE:
      VERBOSE_PRINT("Navigation state = SAFE\n");
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0 || ground_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else if (plant_free_confidence == 0){
        navigation_state = PLANT_FOUND;
      } else {
          // Only set steering bias and change to TURNING state if needed
          if (heading_new == 0) {
              steering_bias = -3.f; // Turn left
              guidance_h_set_heading_rate(steering_bias * oag_heading_rate);
              navigation_state = TURNING;
          } else if (heading_new == 2) {
              steering_bias = 3.f; // Turn right
              guidance_h_set_heading_rate(steering_bias * oag_heading_rate);
              navigation_state = TURNING;
          }
          
          guidance_h_set_body_vel(speed_sp, 0);
      }
      break;

    case TURNING:
      VERBOSE_PRINT("Navigation state = TURNING\n");
      // Include obstacle detection logic even during turning
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12) {
          navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0 || ground_free_confidence == 0) {
          navigation_state = OBSTACLE_FOUND;
      } else if (plant_free_confidence == 0) {
          navigation_state = PLANT_FOUND;
      } else if (heading_new == 1) {
          guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
          navigation_state = SAFE; // Return to SAFE once realigned
      } else {
          // Continue turning as per the previous bias
          guidance_h_set_heading_rate(steering_bias * oag_heading_rate);
      }
      guidance_h_set_body_vel(speed_sp, 0); // Maintain speed during turning
      break;

    case OBSTACLE_FOUND:
      VERBOSE_PRINT("Navigation state = OBSTACLE FOUND\n");
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      VERBOSE_PRINT("Navigation state = SEARCH FOR SAFE HEADING\n");
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      // changed to ground_free_confidence
      if (obstacle_free_confidence >= 2 && ground_free_confidence >=3 && plant_free_confidence >=4){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      VERBOSE_PRINT("Navigation state = OUT OF BOUNDS\n");
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(30));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      VERBOSE_PRINT("Navigation state = REENTER ARENA\n");
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;
        ground_free_confidence = 0;
        plant_free_confidence = 0;
        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      break;
    case PLANT_FOUND:
    VERBOSE_PRINT("Navigation state = PLANT FOUND\n");
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}
