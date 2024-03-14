#ifndef GROUND_SPLIT_H
#define GROUND_SPLIT_H

#include <stdint.h>
#include <stdbool.h>
#include "modules/computer_vision/lib/vision/image.h"

// Ground detection settings
extern uint8_t ground_y_min;
extern uint8_t ground_y_max;
extern uint8_t ground_u_min;
extern uint8_t ground_u_max;
extern uint8_t ground_v_min;
extern uint8_t ground_v_max;

extern bool draw_ground_indicator;

// Global variables for navigation direction based on ground detection
extern int direction;

// Module functions
extern void ground_split_init(void);
extern void ground_detection(struct image_t *img);
extern void ground_split_periodic(void);

#endif /* GROUND_SPLIT_H */
