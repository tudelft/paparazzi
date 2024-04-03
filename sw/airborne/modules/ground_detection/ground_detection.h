/**
 * @file modules/computer_vision/ground_detection.h
 * This module is designed for detecting ground areas to enhance UAV's navigation
 * by avoiding obstacles and ensuring safe landings by analyzing the camera feed
 * for ground characteristics.
 */
#ifndef GROUND_DETECTION_H
#define GROUND_DETECTION_H

#include <stdint.h>
#include <stdbool.h>


// Ground detection settings
extern uint8_t ground_y_min;
extern uint8_t ground_y_max;
extern uint8_t ground_u_min;
extern uint8_t ground_u_max;
extern uint8_t ground_v_min;
extern uint8_t ground_v_max;

extern bool draw_ground_indicator;

// Global variables for navigation direction based on ground detection
extern int16_t direction_new;

// Module functions
extern void ground_detection_init(void);
extern void ground_detection_periodic(void);

#endif /* GROUND_DETECTION_H */
