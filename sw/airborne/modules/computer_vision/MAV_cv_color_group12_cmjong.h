#ifndef MAV_CV_COLOR_GROUP12_CMJONG_H_
#define MAV_CV_COLOR_GROUP12_CMJONG_H_


#include <stdint.h>
#include <stdbool.h>
#include "lib/vision/image.h"  // for struct image_t

/**
 * @file cv_color_filter.h
 * @brief Utility for YUV422 color filtering and centroid detection.
 *
 * Scans an image for pixels within a given YCbCr color range,
 * computes the centroid of matching pixels, and returns the count.
 */

/**
 * find_object_centroid
 *
 * Finds the centroid of pixels within the specified YCbCr color bounds.
 *
 * @param img     - Input image in YUV422 format
 * @param p_xc    - Output: x coordinate of centroid, relative to image center (positive = right)
 * @param p_yc    - Output: y coordinate of centroid, relative to image center (positive = up)
 * @param draw    - If true, matching pixels are brightened in the image (for debugging)
 * @param lum_min - Minimum Y (luminance) value
 * @param lum_max - Maximum Y (luminance) value
 * @param cb_min  - Minimum Cb (blue chroma) value
 * @param cb_max  - Maximum Cb (blue chroma) value
 * @param cr_min  - Minimum Cr (red chroma) value
 * @param cr_max  - Maximum Cr (red chroma) value
 * @return Number of pixels that matched the color filter
 */
uint32_t find_object_centroid(struct image_t *img,
                              int32_t *p_xc, int32_t *p_yc,
                              bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min,  uint8_t cb_max,
                              uint8_t cr_min,  uint8_t cr_max);

typedef struct {
  uint32_t left;
  uint32_t middle;
  uint32_t right;
} PixelCount;

PixelCount orange_detection(struct image_t *img,
                             uint8_t lum_min, uint8_t lum_max,
                             uint8_t cb_min,  uint8_t cb_max,
                             uint8_t cr_min,  uint8_t cr_max);

#endif