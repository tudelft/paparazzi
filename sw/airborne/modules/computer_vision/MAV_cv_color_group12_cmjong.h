#ifndef MAV_CV_COLOR_GROUP12_CMJONG_H_
#define MAV_CV_COLOR_GROUP12_CMJONG_H_

#include <stdint.h>
#include <stdbool.h>
#include "lib/vision/image.h"  // for struct image_t

/**
 * Per-column pixel counts for the 3-column split:
 *   LEFT (0–30%), CENTER (25–75%), RIGHT (70–100%) of the image h-dimension.
 *   Overlap zones (25–30% and 70–75%) are counted in both adjacent columns.
 */
struct column_counts {
  uint16_t left;
  uint16_t center;
  uint16_t right;
};

uint16_t color_detection(struct image_t *img,
                         uint8_t lum_min, uint8_t lum_max,
                         uint8_t cb_min,  uint8_t cb_max,
                         uint8_t cr_min,  uint8_t cr_max,
                         bool draw);

struct column_counts color_detection_columns(struct image_t *img,
                                             uint8_t lum_min, uint8_t lum_max,
                                             uint8_t cb_min,  uint8_t cb_max,
                                             uint8_t cr_min,  uint8_t cr_max,
                                             bool draw);

#endif