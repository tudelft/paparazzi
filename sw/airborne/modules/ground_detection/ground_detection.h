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
extern int direction;

// Module functions
extern void ground_detection_init(void);
extern void ground_detection_periodic(void);

// // Function prototypes
// void ground_detection(struct image_t *img);
// int decide_navigation_direction(int green_percentage_history[HISTORY_LENGTH][5]);
// void count_green_pixels(struct image_t *img, int *green_counts);
// void update_green_history(int green_history[HISTORY_LENGTH][5], int *green_counts);
// bool detect_color(uint8_t y, uint8_t u, uint8_t v);
// int get_quintant(int x, int y, int img_width, int img_height);
// void calculate_pixels_per_quintant(struct image_t *img, int *total_pixels_per_quintant);


#endif /* GROUND_DETECTION_H */
