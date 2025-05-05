/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 *
 * This module, developed for the AE4317 Autonomous Flight of Micro Air Vehicles course at TU Delft,
 * implements an obstacle avoidance strategy using guided mode, combined with color detection from
 * the `cv_detect_color_object` module. It counts orange pixels to detect obstacles, and green pixels
 * to detect the floor boundary of the arena.
 *
 * When the number of orange pixels exceeds a threshold (`oag_color_count_frac`), an avoidance maneuver
 * is triggered. If the green floor detection drops below a threshold (`oag_floor_count_frac`), the system
 * assumes it is near the edge of the Cyberzoo and turns back.
 *
 * The color detection settings must be defined using ORANGE_AVOIDER_VISUAL_DETECTION_ID and
 * FLOOR_VISUAL_DETECTION_ID in the airframe configuration.
 *
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 * This module is part of Paparazzi UAV.
 */

 #include "modules/orange_avoider/orange_avoider_guided.h"
 #include "firmwares/rotorcraft/guidance/guidance_h.h"
 #include "generated/airframe.h"
 #include "state.h"
 #include "modules/core/abi.h"
 #include <stdio.h>
 #include <time.h>
 
 #define ORANGE_AVOIDER_VERBOSE TRUE
 
 #define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string, __FUNCTION__ , ##__VA_ARGS__)
 #if ORANGE_AVOIDER_VERBOSE
 #define VERBOSE_PRINT PRINT
 #else
 #define VERBOSE_PRINT(...)
 #endif
 
 // Define navigation states
 enum navigation_state_t {
   SAFE,
   OBSTACLE_FOUND,
   SEARCH_FOR_SAFE_HEADING,
   OUT_OF_BOUNDS,
   REENTER_ARENA,
   RETREAT
 };
 
 // Define and initialise variables
 enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING; // Initial navigation state
 float oag_color_count_frac = 0.18f;                                 // Threshold for obstacle detection
 float oag_floor_count_frac = 0.05f;                                 // Threshold for floor detection
 float oag_max_speed = 0.5f;                                         // Maximum flight speed [m/s]
 float oag_heading_rate = RadOfDeg(20.f);                            // Heading change rate for avoidance [rad/s]
 int32_t color_count = 0;                                            // Initial orange color count
 int32_t floor_count = 0;                                            // Initial green color count
 int32_t floor_centroid = 0;                                         // Initial y-position of detected floor center
 float avoidance_heading_direction = 0.f;                            // Initial turn direction for avoidance (+1 or -1)
 int16_t obstacle_free_confidence = 0;                               // Initial confidence that path ahead is clear
 const int16_t max_trajectory_confidence = 5;                        // Maximum confidence value
 
 // ABI bindings
 #ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
 #error This module requires ORANGE_AVOIDER_VISUAL_DETECTION_ID defined in the airframe.
 #endif
 static abi_event color_detection_ev;
 
 #ifndef FLOOR_VISUAL_DETECTION_ID
 #error This module requires FLOOR_VISUAL_DETECTION_ID defined in the airframe.
 #endif
 static abi_event floor_detection_ev;
 
 /* ============================ */
 /*        Helper Functions      */
 /* ============================ */
 
 /**
  * @brief Randomly selects an avoidance heading direction.
  */
 static void chooseRandomIncrementAvoidance(void)
 {
   avoidance_heading_direction = (rand() % 2 == 0) ? 1.f : -1.f;
   VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
 }

 /**
  * @brief Set state to RETREAT so the drone will fly backwards for 2 seconds.
  */
 void orange_avoider_guided_retreat(void)
 {
   navigation_state = RETREAT;
 }
 
 /* ================================= */
 /*    ABI Callback & Main Logic      */
 /* ================================= */
 
 /**
  * @brief Callback function triggered on orange detection events.
  */
 static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t __attribute__((unused)) pixel_x,
                                int16_t __attribute__((unused)) pixel_y,
                                int16_t __attribute__((unused)) pixel_width,
                                int16_t __attribute__((unused)) pixel_height,
                                int32_t quality,
                                int16_t __attribute__((unused)) extra)
 {
   color_count = quality;
 }
 
 /**
  * @brief Callback function triggered on green detection events.
  */
 static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t __attribute__((unused)) pixel_x,
                                int16_t pixel_y,
                                int16_t __attribute__((unused)) pixel_width,
                                int16_t __attribute__((unused)) pixel_height,
                                int32_t quality,
                                int16_t __attribute__((unused)) extra)
 {
   floor_count = quality;
   floor_centroid = pixel_y;
 }
 
 /**
  * @brief Initializes the Orange Avoider Guided module.
  */
 void orange_avoider_guided_init(void)
 {
   srand(time(NULL));
   chooseRandomIncrementAvoidance();
 
   AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
   AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
 }
 
 /**
  * @brief Main (periodic) update function for obstacle and arena boundary avoidance.
  */
 void orange_avoider_guided_periodic(void)
 {
   // Only run the module if in the correct flight mode (GUIDED)
   if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
     navigation_state = SEARCH_FOR_SAFE_HEADING;
     obstacle_free_confidence = 0;
     return;
   }
 
   // Compute color detection thresholds
   int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
   int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
   float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;
 
   VERBOSE_PRINT("Color count: %d  Threshold: %d  State: %d\n", color_count, color_count_threshold, navigation_state);
   VERBOSE_PRINT("Floor count: %d  Threshold: %d\n", floor_count, floor_count_threshold);
   VERBOSE_PRINT("Floor centroid fraction: %f\n", floor_centroid_frac);
 
   // Update obstacle-free confidence
   if (color_count < color_count_threshold) {
     obstacle_free_confidence++;
   } else {
     obstacle_free_confidence -= 2;
   }
   Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
 
   // Calculate forward speed based on confidence
   float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);
 
   // State machine for navigation
   switch (navigation_state) {
     case SAFE: {
       if (floor_count < floor_count_threshold ||
           fabsf(floor_centroid_frac) > 0.12f) {
         navigation_state = OUT_OF_BOUNDS;
       }
       else if (obstacle_free_confidence == 0) {
         navigation_state = OBSTACLE_FOUND;
       }
       else {
         guidance_h_set_body_vel(speed_sp, 0);
       }
       break;
     }
 
     case OBSTACLE_FOUND: {
       guidance_h_set_body_vel(0, 0);
       chooseRandomIncrementAvoidance();
       navigation_state = SEARCH_FOR_SAFE_HEADING;
       break;
     }
 
     case SEARCH_FOR_SAFE_HEADING: {
       guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);
       if (obstacle_free_confidence >= 2) {
         guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
         navigation_state = SAFE;
       }
       break;
     }
 
     case OUT_OF_BOUNDS: {
       guidance_h_set_body_vel(0, 0);
       guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15.f));
       navigation_state = REENTER_ARENA;
       break;
     }
 
     case REENTER_ARENA: {
       if (floor_count >= floor_count_threshold &&
           avoidance_heading_direction * floor_centroid_frac >= 0.f) {
         guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
         obstacle_free_confidence = 0;
         navigation_state = SAFE;
       }
       break;
     }
 
     case RETREAT: {
       guidance_h_set_body_vel(-oag_max_speed, 0);
       break;
     }
 
     default:
       break;
   }
 } 