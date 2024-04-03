/*
 * This file is part of the paparazzi project, specifically tailored for GROUP 11's custom module.
 * 
 * This module, developed for the course AE4317 Autonomous Flight of Micro Air Vehicles at TU Delft, 
 * demonstrates an advanced obstacle avoidance algorithm using guided mode flight and color detection.
 * It enhances the orange_avoider_guided module by integrating additional logic for improved 
 * obstacle, floor, and plant detection, utilizing visual cues from the onboard camera system.
 *
 * The module employs color detection for navigation: identifying obstacles (orange), the floor (green),
 * and specific targets or plants. By calculating the proportion of these colors visible to the camera,
 * it decides whether to proceed, avoid obstacles, or execute specialized maneuvers. This approach 
 * allows for nuanced flight behavior in environments with varying elements, aiming for robust 
 * autonomous navigation within the cyberzoo or similar setups.
 *
 * Adaptation and additional functionality were contributed by the students of GROUP 11.
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

#define PRINT(string,...) fprintf(stderr, "[GROUP_11->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUP_11_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Function declaration for choosing random direction for obstacle avoidance
uint8_t chooseRandomIncrementAvoidance(void);

// Enumeration for various navigation states in the state machine
enum navigation_state_t {
  SAFE,
  TURNING,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  PLANT_FOUND
};

// Settings for obstacle and floor detection thresholds, flight speed, and heading change rate
float oag_color_count_frac = 0.18f;       // Obstacle (orange) detection threshold as a fraction of the total image area
float oag_floor_count_frac = 0.05f;       // Floor (green) detection threshold as a fraction of the total image area
float oag_max_speed = 0.5f;               // Maximum flight speed in meters per second (m/s)
float oag_heading_rate = RadOfDeg(20.f);  // Heading change setpoint for avoidance in radians per second (rad/s)
float oag_central_floor_frac = 0.35f;     // Central floor threshold to detect objects in the middle of the image
float oag_plant_frac = 0.08f;             // Plant detection threshold as a fraction of a predefined central image area

// Global variables for state machine, color counts, and confidence levels
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING; // Current state in the state machine
int32_t color_count = 0;                  // Orange color count from color filter for obstacle detection
int32_t floor_count = 0;                  // Green color count from color filter for floor detection
int32_t floor_centroid = 0;               // Floor detector centroid in the y direction (along the horizon)
int32_t floor_central_count = 0;          // Green color count from color filter for central floor detection
int32_t plant_count = 0;                  // Green color count for plant detection
float avoidance_heading_direction = 0;    // Heading change direction for avoidance (positive for right, negative for left)
int16_t obstacle_free_confidence = 0;     // Confidence measure of obstacle absence ahead
int16_t ground_free_confidence = 0;       // Confidence measure of ground presence ahead
int16_t plant_free_confidence = 0;        // Confidence measure of plant absence ahead
int16_t heading_new = 0;                  // Direction for turning based on ground detection (0 for left, 2 for right)
float steering_bias = 0.f;                // Determines turning direction based on ground detection analysis
int8_t navigation_state_msg = 0;     // nav state msg for logging

const int16_t max_trajectory_confidence = 5; // Number of consecutive negative detections needed to be sure of obstacle absence

// Boolean variable to track whether ground calibration is performed
static bool ground_calibration_started = false;

// Callback functions for color detection
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
 * Module initialization function
 * Sets up ABI message bindings and initializes random seed
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
 * Periodic update function
 * Implements the logic for obstacle avoidance and navigation state management
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

  // Add your debug print statements here
  VERBOSE_PRINT("Color_count: %d  threshold: %d \n", color_count, color_count_threshold);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor central count: %d, threshold: %d\n", floor_central_count, central_floor_count_threshold);
  VERBOSE_PRINT("Plant count: %d, threshold: %d\n", plant_count, plant_count_threshold);
  VERBOSE_PRINT("largest green count: %d\n", heading_new);
  // VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);
  // VERBOSE_PRINT("Ground Detection Thresholds - Luminance: min=%d, max=%d\n", cod_lum_min2, cod_lum_max2);
  // VERBOSE_PRINT("Ground Detection Thresholds - Chrominance Blue: min=%d, max=%d\n", cod_cb_min2, cod_cb_max2);
  // VERBOSE_PRINT("Ground Detection Thresholds - Chrominance Red: min=%d, max=%d\n", cod_cr_min2, cod_cr_max2);
  
  AbiSendMsgGROUP11_GROUND_DETECTION(GROUP11_GROUND_DETECT_ID, navigation_state_msg, central_floor_count_threshold);

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

  // Calculate the minimum confidence among obstacle, ground, and plant detections
  float min_confidence = fminf(fminf(obstacle_free_confidence, ground_free_confidence), plant_free_confidence);
  // Adjust speed based on the lower of obstacle and ground confidences
  float speed_sp = fminf(oag_max_speed, 0.2f * min_confidence);
  float steering_bias = 0.f; // Determines turning direction based on ground detection

  // obstacle_free_confidence == 0 ||
  switch (navigation_state){
    case SAFE:
      VERBOSE_PRINT("Navigation state = SAFE\n");
      navigation_state_msg = 0;
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
      navigation_state_msg = 1;
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
      navigation_state_msg = 2;
      VERBOSE_PRINT("Navigation state = OBSTACLE FOUND\n");
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      navigation_state_msg = 3;
      VERBOSE_PRINT("Navigation state = SEARCH FOR SAFE HEADING\n");
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      // changed to ground_free_confidence
      if (obstacle_free_confidence >= 2 && ground_free_confidence >=2 && plant_free_confidence >=3){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      navigation_state_msg = 4;
      VERBOSE_PRINT("Navigation state = OUT OF BOUNDS\n");
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(30));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      navigation_state_msg = 5;
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
      navigation_state_msg = 6;
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
