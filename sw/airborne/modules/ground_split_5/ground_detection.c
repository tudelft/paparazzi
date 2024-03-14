// Standard headers for Paparazzi modules and computer vision
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

// Include the header for the ground split functionality
#include "modules/ground_split_5/ground_detection.h"

// Standard C libraries needed for the module
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>  // For thread safety

// Debugging and verbose output settings
#define PRINT(string,...) fprintf(stderr, "[ground_detection->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#ifdef GROUND_SPLIT_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Mutex for thread safety
static pthread_mutex_t mutex;

// Frame rate settings
#ifndef GROUND_DETECTION_FPS
#define GROUND_DETECTION_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

// Define the YUV color bounds for the color we want to detect
#define LUM_MIN 60
#define LUM_MAX 160
#define CB_MIN 75
#define CB_MAX 110
#define CR_MIN 120
#define CR_MAX 140

// Box dimensions and position
#define HEIGHT_THRESHOLD 40
#define WIDTH_THRESHOLD 50

#define HISTORY_LENGTH 5

/////////////////////////   TO DO   ///////////////////////////////////
//  width en height zijn omgedraaid dus width = 240 en height = 520  //
///////////////////////////////////////////////////////////////////////

// Function prototypes
void ground_detection(struct image_t *img);
int decide_navigation_direction(int green_percentage_history[HISTORY_LENGTH][5]);
void count_green_pixels(struct image_t *img, int *green_counts);
void update_green_history(int green_history[HISTORY_LENGTH][5], int *green_counts);
bool detect_color(uint8_t y, uint8_t u, uint8_t v);
int get_quintant(int x, int y, int img_width, int img_height);
void calculate_pixels_per_quintant(struct image_t *img, int *total_pixels_per_quintant);

void ground_detection(struct image_t *img) {
    int green_counts[5];
    int total_pixels_per_quintant[5];
    int green_percentage_history[HISTORY_LENGTH][5] = {0}; // Initialize all to 0

    // Assuming the frame size is fixed as per your constants (520x240)
    calculate_pixels_per_quintant(&img, total_pixels_per_quintant);
    count_green_pixels(&img, green_counts);

    // Calculate the percentage of green pixels in each quintant
    for (int i = 0; i < 5; i++) {
        if (total_pixels_per_quintant[i] > 0) { // Avoid division by zero
            green_percentage_history[0][i] = (green_counts[i] * 100) / total_pixels_per_quintant[i];
        } else {
            green_percentage_history[0][i] = 0;
        }
    }

    // Decide the navigation direction based on the first (and only) entry in green_percentage_history
    int direction = decide_navigation_direction(green_percentage_history);

    printf("Ground detection and navigation image processing complete.\n");
}

int decide_navigation_direction(int green_percentage_history[HISTORY_LENGTH][5]) {
    int sum[5] = {0};
    int moving_averages[5];
    int threshold = 5; // Threshold percentage (multiplied by 100 to avoid floating point)

    // Calculate sum for each quintant across all history
    for (int i = 0; i < HISTORY_LENGTH; i++) {
        for (int j = 0; j < 5; j++) {
            sum[j] += green_percentage_history[i][j];
        }
    }

    // Calculate moving average for each quintant
    for (int i = 0; i < 5; i++) {
        moving_averages[i] = sum[i] / HISTORY_LENGTH;
    }

    // Decide navigation direction based on the highest moving average that exceeds the threshold
    int max_index = -1;
    int max_value = threshold;
    for (int i = 0; i < 5; i++) {
        if (moving_averages[i] > max_value) {
            max_value = moving_averages[i];
            max_index = i;
        }
    }

    return max_index; // Returns -1 if no quintant exceeds the threshold
}

int get_quintant(int x, int y, int img_width, int img_height) {
    int origin_x = img_width / 2;
    int origin_y = img_height;
    double angle = atan2(origin_y - y, x - origin_x);
    int quintant = (int)(angle / (M_PI/5));
    return quintant;
}

void count_green_pixels(struct image_t *img, int *green_counts) {
    uint8_t *buffer = img->buf;
    int start_x = (img->w / 2) - WIDTH_THRESHOLD;
    int end_x = start_x + (2 * WIDTH_THRESHOLD);
    int start_y = img->h - HEIGHT_THRESHOLD;
    int end_y = img->h;

    // Initialize green_counts array
    for (int i = 0; i < 5; i++) {
        green_counts[i] = 0;
    }

    for (int y = start_y; y < end_y; y++) {
        for (int x = start_x; x < end_x; x++) {
            uint8_t *yp, *up, *vp;
            if (x % 2 == 0) {
                up = &buffer[y * 2 * img->w + 2 * x];
                yp = &buffer[y * 2 * img->w + 2 * x + 1];
                vp = &buffer[y * 2 * img->w + 2 * x + 2];
            } else {
                up = &buffer[y * 2 * img->w + 2 * x - 2];
                vp = &buffer[y * 2 * img->w + 2 * x];
                yp = &buffer[y * 2 * img->w + 2 * x + 1];
            }

            if (detect_color(*yp, *up, *vp)) {
                int quintant = get_quintant(x, y, img->w, img->h);
                green_counts[quintant]++;
                // Optionally mark detected green pixels on the image
                // Note: This might alter the original image significantly
                *yp = 255; // Y channel to maximum brightness
                *up = 0;   // U channel to mid-range (green)
                *vp = 0;   // V channel to mid-range (green)
            }
        }
    }
}

void update_green_history(int green_history[HISTORY_LENGTH][5], int *green_counts) {
    static int current_index = 0;

    // Calculate green percentages for the current frame and update the history
    for (int i = 0; i < 5; i++) {
        green_history[current_index][i] = (green_counts[i] * 100) / ; // Assuming frame_size is the total pixel count in the detection area
    }

    // Update the index for the next frame, wrapping around if necessary
    current_index = (current_index + 1) % HISTORY_LENGTH;
}

bool detect_color(uint8_t y, uint8_t u, uint8_t v) {
    return y >= LUM_MIN && y <= LUM_MAX &&
           u >= CB_MIN && u <= CB_MAX &&
           v >= CR_MIN && v <= CR_MAX;
}

void calculate_pixels_per_quintant(struct image_t *img, int *total_pixels_per_quintant) {
    int start_x = (img->w / 2) - WIDTH_THRESHOLD;
    int end_x = start_x + (2 * WIDTH_THRESHOLD);
    int start_y = img->h - HEIGHT_THRESHOLD;
    int end_y = img->h;

    // Initialize total_pixels_per_quintant array
    for (int i = 0; i < 5; i++) {
        total_pixels_per_quintant[i] = 0;
    }

    for (int y = start_y; y < end_y; y++) {
        for (int x = start_x; x < end_x; x++) {
            int quintant = get_quintant(x, y, img->w, img->h);
            total_pixels_per_quintant[quintant]++;
        }
    }
}
