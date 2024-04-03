/*
 * This file is part of the Paparazzi project and has been modified by the contributors
 * to include advanced ground detection functionalities for the AE4317 Autonomous Flight
 * of Micro Air Vehicles course at TU Delft.
 *
 * This module employs computer vision to enhance UAV navigation through dynamic ground detection,
 * supporting autonomous operations in complex environments. It leverages the YUV color space
 * for efficient ground analysis, enabling the UAV to adapt its flight path based on real-time
 * ground visibility. This capability is crucial for obstacle avoidance, boundary awareness,
 * and safe landing operations.
 *
 * Key features include:
 * - Dynamic calibration for ground color detection to adapt to varying lighting conditions.
 * - Detection of specific ground areas for improved spatial awareness and boundary detection.
 * - Integration with the Paparazzi UAV system for real-time operation and decision-making.
 *
 * The implementation focuses on real-time image analysis to identify ground areas, calculate
 * centroid positions for detected regions, and inform navigation decisions based on the
 * visual characteristics of the ground.
 */

// Standard headers for Paparazzi modules and computer vision
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

// header for color thresholds
#include "modules/computer_vision/cv_detect_color_object.h"

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

// Box dimensions and position
#define HEIGHT_THRESHOLD 40
#define WIDTH_THRESHOLD 30

#define HISTORY_LENGTH 3
#define NUMB_QUINT 3

int16_t direction_new = 0;

static bool pixels_per_quintant_calculated = false;

// Function prototypes
uint32_t ground_detection(struct image_t *img, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
int decide_navigation_direction(int green_percentage_history[HISTORY_LENGTH][NUMB_QUINT]);
void count_green_pixels(struct image_t *img, int *green_counts, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
void update_green_history(int green_history[HISTORY_LENGTH][NUMB_QUINT], int *green_counts, int *total_pixels_per_quintant);
bool detect_color(uint8_t y, uint8_t u, uint8_t v, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
int get_quintant(int x, int y, int img_height);
void calculate_pixels_per_quintant(struct image_t *img, int *total_pixels_per_quintant);

/**
 * This function processes the input image to determine the direction based on ground detection. It sets up the color
 * thresholds for detection and calculates the navigation direction based on the analysis of the ground visibility.
 *
 * @param img The input image to be processed.
 * @return The processed image.
 */
static struct image_t *object_detector(struct image_t *img) {
    // Define local variables for color thresholding based on predefined settings
    uint8_t lum_min, lum_max;
    uint8_t cb_min, cb_max;
    uint8_t cr_min, cr_max;

    // Assign color threshold values for ground detection from cv_detect_color_object
    lum_min = cod_lum_min2;
    lum_max = cod_lum_max2;
    cb_min = cod_cb_min2;
    cb_max = cod_cb_max2;
    cr_min = cod_cr_min2;
    cr_max = cod_cr_max2;

    // Perform ground detection on the image to obtain navigation direction
    int16_t direction_output = ground_detection(img, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);

    // Safely update the global direction variable with mutex for thread safety
    pthread_mutex_lock(&mutex);
    direction_new = direction_output;
    pthread_mutex_unlock(&mutex);

    // Return the unmodified image, as this function's purpose is to affect navigation, not image manipulation
    return img;
}

struct image_t *ground_detection1(struct image_t *img, uint8_t camera_id);
struct image_t *ground_detection1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img);
}

/**
 * Main ground detection function that analyzes the image and determines the preferred navigation direction.
 * It divides the image into quintants, counts green pixels in each quintant, and decides the navigation direction
 * based on green pixel distribution and historical data.
 *
 * @param img The input image to process.
 * @param lum_min Minimum Y value for green detection.
 * @param lum_max Maximum Y value for green detection.
 * @param cb_min Minimum Cb value for green detection.
 * @param cb_max Maximum Cb value for green detection.
 * @param cr_min Minimum Cr value for green detection.
 * @param cr_max Maximum Cr value for green detection.
 * @return The chosen direction based on ground detection analysis.
 */
uint32_t ground_detection(struct image_t *img, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max) {

    static int total_pixels_per_quintant[NUMB_QUINT]; // Make this static to retain its value across calls
    static int initialization_done = 0; // Add a flag to track the initialization
    static int green_percentage_history[HISTORY_LENGTH][NUMB_QUINT] = {{0}}; // Static ensures it retains values between calls
    int green_counts[NUMB_QUINT] = {0};
    
    if (!initialization_done) {
        // Initialize your arrays only once
        for (int i = 0; i < NUMB_QUINT; i++) {
            green_counts[i] = 0;
            total_pixels_per_quintant[i] = 0; // Now only initialized if not done
        }
        initialization_done = 1; // Mark as initialized
    }

    // Assuming the frame size is fixed as per your constants (520x240)
    calculate_pixels_per_quintant(img, total_pixels_per_quintant);
    count_green_pixels(img, green_counts, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
    update_green_history(green_percentage_history, green_counts, total_pixels_per_quintant);

    // Decide the navigation direction based on the first (and only) entry in green_percentage_history
    int direction = decide_navigation_direction(green_percentage_history);

    return direction;
}

/**
 * Decides navigation direction based on the historical distribution of green pixels across different quintants of the image.
 * This decision-making process utilizes moving averages of green pixel percentages to smooth out transient variations.
 *
 * @param green_percentage_history Historical green pixel percentage data for each quintant.
 * @return The index of the chosen quintant as the navigation direction.
 */
int decide_navigation_direction(int green_percentage_history[HISTORY_LENGTH][NUMB_QUINT]) {
    float moving_averages[NUMB_QUINT] = {0};
    for (int quintant = 0; quintant < NUMB_QUINT; quintant++) {
        int sum = 0;
        for (int frame = 0; frame < HISTORY_LENGTH; frame++) {
            sum += green_percentage_history[frame][quintant];
        }
        moving_averages[quintant] = (float)sum / HISTORY_LENGTH;
    }

    // Check if quintant 2's moving average is above the threshold
    const float percentage_threshold = 90.0;
    int mid_index = (int)floor(NUMB_QUINT / 2.0); // Ensure division is floating-point
    if (moving_averages[mid_index] >= percentage_threshold) {
        return mid_index; // Return the mid quintant as the chosen direction if it meets the threshold
    }

    // Logic to choose the quintant with the highest moving average
    int max_index = -1;
    float max_average = 0; // Start with zero, assuming at least one average should be above zero
    bool quintant_two_is_max = false; // Flag to check if quintant 2 is one of the max values

    for (int i = 0; i < NUMB_QUINT; i++) {
        if (moving_averages[i] > max_average) {
            max_index = i;
            max_average = moving_averages[i];
            quintant_two_is_max = (i == mid_index); // Check if the new maximum is quintant 2
        } else if (moving_averages[i] == max_average) {
            if (i == mid_index) {
                quintant_two_is_max = true; // Set flag if quintant 2 shares the max
            }
        }
    }

    // If quintant 2 is among the quintants with the maximum average, choose it.
    if (quintant_two_is_max) {
        max_index = mid_index; // Default to quintant 2 if it is one of the maximums
    }

    return max_index; // Return the index of the chosen quintant
}

/**
 * Calculates the quintant of a given pixel in the image. This function divides the image
 * into equal segments (quintants) and determines which quintant a pixel belongs to
 * based on its x-coordinate. It's used to help segment the image for more detailed analysis,
 * particularly in functions that need to operate on specific areas of the image.
 *
 * @param x The x-coordinate of the pixel.
 * @param y The y-coordinate of the pixel, used here for symmetry in API but not in calculation.
 * @param img_height The height of the image, used to calculate the origin for angle computation.
 * @return The quintant number (starting from 0) in which the pixel is located.
 */
int get_quintant(int x, int y, int img_height) {
    // Calculate the middle line of the image to use as a reference point for angle calculation
    int origin_y = img_height / 2;
    // Calculate the angle of the point from the central line, which is used to determine the quintant
    double angle = atan2(x, origin_y - y);
    // Map the angle to a quintant, ensuring the return value is within bounds
    int quintant = (int)(NUMB_QUINT * angle / M_PI);
    return quintant < 0 ? 0 : (quintant >= NUMB_QUINT ? NUMB_QUINT - 1 : quintant);
}

/**
 * Counts the number of green pixels within a defined threshold in each quintant of the image.
 * This is a critical step for ground detection, as it quantifies the amount of "green" (which
 * could be indicative of grass, trees, or other vegetation) and thereby assists in the UAV's
 * decision-making process for navigation and obstacle avoidance.
 *
 * @param img The image on which pixel counting is performed.
 * @param green_counts An array to store the count of green pixels found in each quintant.
 * @param lum_min Minimum luminance value for a pixel to be considered "green".
 * @param lum_max Maximum luminance value for a pixel to be considered "green".
 * @param cb_min Minimum Cb (chroma blue) value for a pixel to be considered "green".
 * @param cb_max Maximum Cb (chroma blue) value for a pixel to be considered "green".
 * @param cr_min Minimum Cr (chroma red) value for a pixel to be considered "green".
 * @param cr_max Maximum Cr (chroma red) value for a pixel to be considered "green".
 *
 * This function iterates over a central band of the image, applying color thresholding to
 * identify green pixels. Detected green pixels are then categorized by their quintant for
 * subsequent analysis. This method allows for a granular approach to understanding the
 * distribution of navigable space within the UAV's visual field.
 */
void count_green_pixels(struct image_t *img, int *green_counts, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max) {
    uint8_t *buffer = img->buf;
    // Define the region of interest boundaries within the image
    int start_x = 0;
    int end_x = WIDTH_THRESHOLD;
    int start_y = (img->h/2) - HEIGHT_THRESHOLD;
    int end_y = (img->h/2) + HEIGHT_THRESHOLD;

    // Initialize green_counts array
    for (int i = 0; i < NUMB_QUINT; i++) {
        green_counts[i] = 0;
    }

    for (int y = start_y; y < end_y; y++) {
        for (int x = start_x; x < end_x; x++) {
            uint8_t *yp, *up, *vp;
            // Determine the memory location of Y, U, and V components for the pixel
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
            }
        }
    }
}

/**
 * Updates the history of green pixel percentages across several frames for each quintant.
 * This is used to smooth out the navigation decision over time, reducing the impact of temporary fluctuations.
 *
 * @param green_history Array containing the history of green pixel percentages for each quintant.
 * @param green_counts Array containing the current frame's count of green pixels for each quintant.
 * @param total_pixels_per_quintant Array containing the total pixel count for each quintant, used for percentage calculation.
 */
void update_green_history(int green_history[HISTORY_LENGTH][NUMB_QUINT], int *green_counts, int *total_pixels_per_quintant) {
    // Shift history data to make room for the new data (moving older data up and losing the oldest)
    for (int i = HISTORY_LENGTH - 2; i >= 0; i--) {
        for (int j = 0; j < NUMB_QUINT; j++) {
            green_history[i + 1][j] = green_history[i][j];
        }
    }

    // Add new data (percentage of green pixels for each quintant)
    for (int i = 0; i < NUMB_QUINT; i++) {
        green_history[0][i] = (total_pixels_per_quintant[i] > 0) ? ((green_counts[i] * 100) / total_pixels_per_quintant[i]) : 0;
    }
}

/**
 * Determines if a given pixel color falls within specified YUV color thresholds.
 *
 * @param y Y component of the pixel.
 * @param u U component of the pixel.
 * @param v V component of the pixel.
 * @param lum_min Minimum acceptable Y value.
 * @param lum_max Maximum acceptable Y value.
 * @param cb_min Minimum acceptable U value.
 * @param cb_max Maximum acceptable U value.
 * @param cr_min Minimum acceptable V value.
 * @param cr_max Maximum acceptable V value.
 * @return True if the pixel color is within the thresholds, false otherwise.
 */
bool detect_color(uint8_t y, uint8_t u, uint8_t v, uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max) {
    return y >= lum_min && y <= lum_max &&
           u >= cb_min && u <= cb_max &&
           v >= cr_min && v <= cr_max;
}

/**
 * Calculates the distribution of pixels across quintants in a specified area of the image.
 * This function helps in determining the focus area for ground detection based on spatial distribution.
 *
 * @param img The input image to analyze.
 * @param total_pixels_per_quintant Array to store the pixel count for each quintant.
 */
void calculate_pixels_per_quintant(struct image_t *img, int *total_pixels_per_quintant) {
    // Only calculate once per session if all frames have the same dimensions
    if (pixels_per_quintant_calculated) {
        return; // Skip calculation if already done
    }

    int start_x = 0; // Start from the left side of the image
    int end_x = WIDTH_THRESHOLD; // End at the width threshold
    int start_y = (img->h / 2) - HEIGHT_THRESHOLD; // Start from the middle of the image minus the height threshold
    int end_y = (img->h / 2) + HEIGHT_THRESHOLD; // End at the middle of the image plus the height threshold

    // Initialize total_pixels_per_quintant array
    for (int i = 0; i < NUMB_QUINT; i++) {
        total_pixels_per_quintant[i] = 0;
    }

    // Count pixels in each quintant within the specified area
    for (int y = start_y; y < end_y; y++) {
        for (int x = start_x; x < end_x; x++) {
            int quintant = get_quintant(x, y, img->h);
            if (quintant >= 0 && quintant < NUMB_QUINT) { // Check that the quintant is valid
                total_pixels_per_quintant[quintant]++;
            }
        }
    }

    pixels_per_quintant_calculated = true; // Mark that we've calculated this to avoid redundant calculations
}

/**
 * Initializes the ground detection module, setting up necessary configurations and registering
 * the detection function with the computer vision system.
 */
void ground_detection_init(void)
{
    pthread_mutex_init(&mutex, NULL);
    cv_add_to_device(&GROUND_DETECTION_CAMERA, ground_detection1, GROUND_DETECTION_FPS, 0);
}

/**
 * Performs periodic updates based on ground detection results and communicates these results to the rest of the system.
 * This function is part of the module's periodic execution, ensuring up-to-date navigation information is available.
 */
void ground_detection_periodic(void) 
{   
    // Send ABI message with the new direction
    AbiSendMsgGROUND_DETECTION(GROUND_DETECTION_ID, direction_new);
}