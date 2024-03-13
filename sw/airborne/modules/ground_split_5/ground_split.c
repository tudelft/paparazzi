// Standard headers for Paparazzi modules and computer vision
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

// Standard C libraries needed for the module
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>  // For thread safety

// Include the header for the ground split functionality
#include "modules/ground_split_5/ground_split.h"

// Debugging and verbose output settings
#define PRINT(string,...) fprintf(stderr, "[ground_split->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#ifdef GROUND_SPLIT_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Mutex for thread safety
static pthread_mutex_t mutex;

// Frame rate settings
#ifndef GROUND_SPLIT_FPS
#define GROUND_SPLIT_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

// Parameters for ground detection
#define FRAME_WIDTH 520    // Expected frame width for ground detection processing
#define FRAME_HEIGHT 240   // Expected frame height for ground detection processing
#define FPS 5              // Processing frame rate for ground detection


/// Parameters for ground color detection
uint8_t ground_y_min = 0;   // Minimum Y value for ground color detection in YUV color space
uint8_t ground_y_max = 0; // Maximum Y value for ground color detection in YUV color space
uint8_t ground_u_min = 0;   // Minimum U value for ground color detection in YUV color space
uint8_t ground_u_max = 0; // Maximum U value for ground color detection in YUV color space
uint8_t ground_v_min = 0;   // Minimum V value for ground color detection in YUV color space
uint8_t ground_v_max = 0; // Maximum V value for ground color detection in YUV color space

bool draw_ground_indicator = false; // Whether to draw the ground detection indicator

// Global variables for ground detection
static struct image_t *img;
static int direction = 0; // Current navigation direction based on ground detection

// Forward declarations
static bool detect_color(uint8_t y, uint8_t u, uint8_t v);
static void draw_direction_indicator(struct image_t *img, int direction);
static void draw_quintants(struct image_t *img);

/**
 * Performs ground detection on the provided image.
 * This function analyzes the image to find the direction with the most detected ground color.
 * 
 * @param input_img - input image to process
 */
void ground_detection(struct image_t *input_img) {
    // Ensure the input image is valid
    if (input_img == NULL) {
        return;
    }

    // Convert to YUV color space if necessary
    struct image_t yuv_img;
    img_copy(input_img, &yuv_img);
    image_switch_color(&yuv_img, IMAGE_YUV422);

    draw_quintants(input_img);
    int green_counts[5] = {0};

    // Analyze the ground based on color detection
    for (int y = 0; y < FRAME_HEIGHT - 200; y++) { // Check bottom pixels only
        for (int x = 0; x < FRAME_WIDTH; x++) {
            struct color_t pixel = get_color(&yuv_img, x, y);
            double angle = atan2(y, x - FRAME_WIDTH / 2);
            if (angle < 0) {
                angle += 2 * M_PI;
            }
            int quintant = (int)(5 * angle / M_PI);
            if (detect_color(pixel.y, pixel.u, pixel.v)) {
                green_counts[quintant]++;
            }
        }
    }

    // Determine the direction with the highest green count
    int max_count = 0;
    for (int i = 0; i < 5; i++) {
        if (green_counts[i] > max_count) {
            max_count = green_counts[i];
            direction = i + 1;
        }
    }

    draw_direction_indicator(input_img, direction);
    // You can add further processing based on the direction, e.g., update state variables or send messages
}

/**
 * Checks if the provided YUV pixel is within the target green color range for ground detection.
 * 
 * @param y - Y component of the pixel
 * @param u - U component of the pixel
 * @param v - V component of the pixel
 * @return true if the pixel is within the target color range, false otherwise
 */
static bool detect_color(uint8_t y, uint8_t u, uint8_t v) {
    return (y >= ground_y_min && y <= ground_y_max) &&
           (u >= ground_u_min && u <= ground_u_max) &&
           (v >= ground_v_min && v <= ground_v_max);
}

/**
 * Draws an indicator for the direction to navigate based on the detected ground.
 * 
 * @param img - The image to draw onto
 * @param direction - The direction to indicate
 */
static void draw_direction_indicator(struct image_t *img, int direction) {
    double angle_offset = (2 * M_PI / 5) / 2;
    double quintant_start_angles[5];
    for (int i = 0; i < 5; i++) {
        quintant_start_angles[i] = i * (2 * M_PI / 5);
    }
    double middle_angle = quintant_start_angles[direction - 1] + angle_offset;
    int arrow_length = 100;
    int end_x = FRAME_WIDTH / 2 + arrow_length * cos(middle_angle);
    int end_y = FRAME_HEIGHT - 1 - arrow_length * sin(middle_angle);
    draw_line(img, FRAME_WIDTH / 2, FRAME_HEIGHT - 1, end_x, end_y, UV_GREEN_COLOR);
}

/**
 * Draws quintants on the ground detection image for visual reference.
 * 
 * @param img - The image to draw onto
 */
static void draw_quintants(struct image_t *img) {
    double angles[5];
    for (int i = 0; i < 5; i++) {
        angles[i] = i * (M_PI / 5);
    }
    for (int i = 0; i < 5; i++) {
        int end_x = FRAME_WIDTH / 2 + cos(angles[i]) * FRAME_WIDTH;
        int end_y = FRAME_HEIGHT - 1 - sin(angles[i]) * FRAME_HEIGHT;
        draw_line(img, FRAME_WIDTH / 2, FRAME_HEIGHT - 1, end_x, end_y, UV_WHITE_COLOR);
    }
}


void ground_split_init(void) {
    // Initialize any required global variables here.
    // Since we're working with predefined constants for color thresholds, 
    // there might not be direct analogs for individual filter settings as in the color object detector.

    // However, if there are adjustable parameters or settings you wish to initialize from configuration,
    // this is the place to do it. For example, you might have configurable thresholds or frame settings.

    // Example:
    ground_y_min = COLOR_OBJECT_DETECTOR_LUM_MIN2;
    ground_y_max = COLOR_OBJECT_DETECTOR_LUM_MAX2;
    ground_u_min = COLOR_OBJECT_DETECTOR_CB_MIN2;
    ground_u_max = COLOR_OBJECT_DETECTOR_CB_MAX2;
    ground_v_min = COLOR_OBJECT_DETECTOR_CR_MIN2;
    ground_v_max = COLOR_OBJECT_DETECTOR_CR_MAX2;

    // Register the ground detection function with the computer vision system, if applicable.
    // This typically involves adding the ground detection function to a processing pipeline or setting it as a callback.
    // Replace 'GROUND_SPLIT_CAMERA' with the actual camera identifier used in your system and
    // 'ground_detection' with the name of your ground detection function.
    // Example:
    cv_add_to_device(&GROUND_SPLIT_CAMERA, ground_detection, GROUND_SPLIT_FPS, 0);
}

// Direction determination function, based on color segmentation results
static int determine_navigation_direction(int green_counts[5]) {
    // Determine which segment has the most green (or navigable area)
    // and return the corresponding direction.
    int max_count = 0, max_index = -1;
    for (int i = 0; i < 5; i++) {
        if (green_counts[i] > max_count) {
            max_count = green_counts[i];
            max_index = i;
        }
    }
    return max_index + 1; // +1 to convert from index to human-readable direction
}

void ground_split_periodic(void) {
    static int last_direction = -1; // Store the last direction for comparison
    pthread_mutex_lock(&mutex); // Assuming you have a similar mutex for thread safety
    int current_direction = direction; // Assuming 'direction' is updated by your main detection logic
    pthread_mutex_unlock(&mutex);

    // Only send a message if the direction has changed or some time has passed
    if (current_direction != last_direction) {
        // Send ABI message with the new direction
        AbiSendMsgGROUND_SPLIT(GROUND_SPLIT_ID, current_direction);
        last_direction = current_direction; // Update the last known direction
    }

    // You can also add other periodic tasks here, like logging or additional message sending
}
