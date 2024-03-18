// Standard headers for Paparazzi modules and computer vision
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

// Include the header for the ground split functionality
#include "modules/ground_detection/ground_detection.h"

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

// // Define the YUV color bounds for the color we want to detect
// #define LUM_MIN 60
// #define LUM_MAX 160
// #define CB_MIN 75
// #define CB_MAX 110
// #define CR_MIN 120
// #define CR_MAX 140

// Box dimensions and position
#define HEIGHT_THRESHOLD 50
#define WIDTH_THRESHOLD 40

#define HISTORY_LENGTH 3

uint8_t cod_lum_min = 0;
uint8_t cod_lum_max = 0;
uint8_t cod_cb_min = 0;
uint8_t cod_cb_max = 0;
uint8_t cod_cr_min = 0;
uint8_t cod_cr_max = 0;

int16_t direction_new = 0;

static bool pixels_per_quintant_calculated = false;


/////////////////////////   TO DO   ///////////////////////////////////
//  width en height zijn omgedraaid dus width = 240 en height = 520  //
///////////////////////////////////////////////////////////////////////

// Function prototypes
uint32_t ground_detection(struct image_t *img, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
int decide_navigation_direction(int green_percentage_history[HISTORY_LENGTH][5]);
void count_green_pixels(struct image_t *img, int *green_counts, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
void update_green_history(int green_history[HISTORY_LENGTH][5], int *green_counts, int *total_pixels_per_quintant);
bool detect_color(uint8_t y, uint8_t u, uint8_t v, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
int get_quintant(int x, int y, int img_height);
void calculate_pixels_per_quintant(struct image_t *img, int *total_pixels_per_quintant);

static struct image_t *object_detector(struct image_t *img)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;

    lum_min =cod_lum_min;
    lum_max =cod_lum_max;
    cb_min = cod_cb_min;
    cb_max = cod_cb_max;
    cr_min = cod_cr_min;
    cr_max = cod_cr_max;

    // Obtain the direction based on ground detection
    int16_t direction_output = ground_detection(img, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);

    // Lock the mutex, update the global direction, and unlock the mutex
    pthread_mutex_lock(&mutex);
    direction_new = direction_output;
    pthread_mutex_unlock(&mutex);

    // Return the original image
    return img;
  };
struct image_t *ground_detection1(struct image_t *img, uint8_t camera_id);
struct image_t *ground_detection1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img);
}

uint32_t ground_detection(struct image_t *img, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max) {
    int green_counts[5];
    int total_pixels_per_quintant[5];
    int green_percentage_history[HISTORY_LENGTH][5] = {0}; // Initialize all to 0

    // Assuming the frame size is fixed as per your constants (520x240)
    calculate_pixels_per_quintant(img, total_pixels_per_quintant);
    count_green_pixels(img, green_counts, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
    update_green_history(green_percentage_history, green_counts, total_pixels_per_quintant);

    // Decide the navigation direction based on the first (and only) entry in green_percentage_history
    int direction = decide_navigation_direction(green_percentage_history);

    // printf("lum_min: %u, lum_max: %u, cb_min: %u, cb_max: %u, cr_min: %u, cr_max: %u.\n", lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
    return direction;
}

int decide_navigation_direction(int green_percentage_history[HISTORY_LENGTH][5]) {
    int sum[5] = {0};
    float moving_averages[5]; // Use float for averages
    float threshold = 5; // 5% threshold, assuming your percentages are actually integers like 5 for 5%

    // Calculate sum for each quintant across all history
    for (int i = 0; i < HISTORY_LENGTH; i++) {
        for (int j = 0; j < 5; j++) {
            sum[j] += green_percentage_history[i][j];
        }
    }

    // Calculate moving average for each quintant
    for (int i = 0; i < 5; i++) {
        moving_averages[i] = (float)sum[i] / HISTORY_LENGTH; // Convert to float before division
    }

    // Decide navigation direction based on the highest moving average that exceeds the threshold
    int max_index = -1;
    float max_value = threshold; // Use float for comparison
    for (int i = 0; i < 5; i++) {
        if (moving_averages[i] > max_value) {
            max_value = moving_averages[i];
            max_index = i;
        }
    }

    // Debug print statements to see the moving averages and final decision
    printf("Moving averages:\n");
    for (int i = 0; i < 5; i++) {
        printf("Quintant %d: %.2f\n", i, moving_averages[i]);
    }
    printf("Final decision: Quintant %d with average: %.2f\n", max_index, max_value);

    return max_index; // Returns -1 if no quintant exceeds the threshold
}


int get_quintant(int x, int y, int img_height) {
    // int origin_x = img_width / 2;
    // int origin_y = img_height;
    // double angle = atan2(origin_y - y, x - origin_x);
    // int quintant = (int)(angle / (M_PI/5));
    int origin_y = img_height / 2;
    double angle = atan2(x, origin_y - y);
    int quintant = (int)(angle / (M_PI/5));
    return quintant;
}

void count_green_pixels(struct image_t *img, int *green_counts, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max) {
    uint8_t *buffer = img->buf;
    int start_x = 0;
    int end_x = WIDTH_THRESHOLD;
    int start_y = (img->h/2) - HEIGHT_THRESHOLD;
    int end_y = (img->h/2) + HEIGHT_THRESHOLD;

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

            if (detect_color(*yp, *up, *vp, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max)) {
                int quintant = get_quintant(x, y, img->h);
                green_counts[quintant]++;
                // Optionally mark detected green pixels on the image
                // Note: This might alter the original image significantly
                // *yp = 255; // Y channel to maximum brightness
                // *up = 0;   // U channel to mid-range (green)
                // *vp = 0;   // V channel to mid-range (green)
            }
        }
    }

    // Print out the green pixel counts per quintant after counting
    printf("Green pixel counts per quintant:\n");
    for (int i = 0; i < 5; i++) {
        printf("Quintant %d: %d green pixels\n", i, green_counts[i]);
    }
}

void update_green_history(int green_history[HISTORY_LENGTH][5], int *green_counts, int *total_pixels_per_quintant) {
    static int current_index = 0;

    // Calculate green percentages for the current frame and update the history
    for (int i = 0; i < 5; i++) {
        if (total_pixels_per_quintant[i] > 0) { // Avoid division by zero
            green_history[current_index][i] = (green_counts[i] * 100) / total_pixels_per_quintant[i];
        } else {
            green_history[current_index][i] = 0; // Set percentage to 0 if no pixels are in the quintant
        }
    }

    // Update the index for the next frame, wrapping around if necessary
    current_index = (current_index + 1) % HISTORY_LENGTH;
}


bool detect_color(uint8_t y, uint8_t u, uint8_t v, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max) {
    return y >= lum_min && y <= lum_max &&
           u >= cb_min && u <= cb_max &&
           v >= cr_min && v <= cr_max;
}

void calculate_pixels_per_quintant(struct image_t *img, int *total_pixels_per_quintant) {
    // Check if we've already calculated for this frame size
    if (pixels_per_quintant_calculated) {
        return; // Skip calculation if already done
    }

    int start_x = 0;
    int end_x = WIDTH_THRESHOLD;
    int start_y = (img->h/2) - HEIGHT_THRESHOLD;
    int end_y = (img->h/2) + HEIGHT_THRESHOLD;

    // Initialize total_pixels_per_quintant array
    for (int i = 0; i < 5; i++) {
        total_pixels_per_quintant[i] = 0;
    }

    for (int y = start_y; y < end_y; y++) {
        for (int x = start_x; x < end_x; x++) {
            int quintant = get_quintant(x, y, img->h);
            total_pixels_per_quintant[quintant]++;
        }
    }

    pixels_per_quintant_calculated = true;
}

void ground_detection_init(void)
{
    pthread_mutex_init(&mutex, NULL);
    // Initialize any required global variables here.
    // Since we're working with predefined constants for color thresholds, 
    // there might not be direct analogs for individual filter settings as in the color object detector.

    // However, if there are adjustable parameters or settings you wish to initialize from configuration,
    // this is the place to do it. For example, you might have configurable thresholds or frame settings.

    // Example:
    cod_lum_min = COLOR_OBJECT_DETECTOR_LUM_MIN2;
    cod_lum_max = COLOR_OBJECT_DETECTOR_LUM_MAX2;
    cod_cb_min = COLOR_OBJECT_DETECTOR_CB_MIN2;
    cod_cb_max = COLOR_OBJECT_DETECTOR_CB_MAX2;
    cod_cr_min = COLOR_OBJECT_DETECTOR_CR_MIN2;
    cod_cr_max = COLOR_OBJECT_DETECTOR_CR_MAX2;

    // Register the ground detection function with the computer vision system, if applicable.
    // This typically involves adding the ground detection function to a processing pipeline or setting it as a callback.
    // Replace 'GROUND_SPLIT_CAMERA' with the actual camera identifier used in your system and
    // 'ground_detection' with the name of your ground detection function.
    // Example:
    cv_add_to_device(&GROUND_DETECTION_CAMERA, ground_detection1, GROUND_DETECTION_FPS, 0);
}

void ground_detection_periodic(void) 
{   
    // static struct color_object_t;
    // pthread_mutex_lock(&mutex);
    // memcpy(2*sizeof(struct color_object_t));
    // pthread_mutex_unlock(&mutex);


    // Send ABI message with the new direction
    AbiSendMsgGROUND_DETECTION(GROUND_DETECTION_ID, direction_new);
}