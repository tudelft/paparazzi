
#include "MAV_cv_color_group12_cmjong.h"
#include <math.h>

/*
 * YUV422 memory layout (4 bytes per 2 pixels):
 *
 *   [ U | Y1 | V | Y2 ]
 *     0    1   2    3
 *
 * Even pixel x reads:  U @ 2x,   Y1 @ 2x+1, V @ 2x+2
 * Odd  pixel x reads:  U @ 2x-2, V @ 2x,    Y2 @ 2x+1
 */

uint32_t find_object_centroid(struct image_t *img,
                              int32_t *p_xc, int32_t *p_yc,
                              bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min,  uint8_t cb_max,
                              uint8_t cr_min,  uint8_t cr_max)
{
  uint32_t cnt   = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;

  uint8_t *buffer = img->buf;

  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {

      uint8_t *yp, *up, *vp;

      if (x % 2 == 0) {
        // Even pixel: use Y1
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
      } else {
        // Odd pixel: use Y2
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U (shared with previous even pixel)
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }

      // Check if pixel is within the color bounds
      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min)  && (*up <= cb_max)  &&
          (*vp >= cr_min)  && (*vp <= cr_max)) {

        cnt++;
        tot_x += x;
        tot_y += y;

        if (draw) {
          *yp = 255;  // brighten pixel for visual debugging
        }
      }
    }
  }

  // Compute centroid relative to image center
  // x: positive = right of center
  // y: positive = above center (image y-axis is flipped)
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / (float)cnt - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / (float)cnt);
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }

  return cnt;
}





/*
 * YUV422 memory layout (4 bytes per 2 pixels):
 *
 *   [ U | Y1 | V | Y2 ]
 *     0    1   2    3
 *
 * Even pixel x reads:  U @ 2x,   Y1 @ 2x+1, V @ 2x+2
 * Odd  pixel x reads:  U @ 2x-2, V @ 2x,    Y2 @ 2x+1
 *
 * Region layout (with overlap = img->w / 6):
 *
 *   |<------- left ------->|
 *   0              w/3 + overlap
 *              |<------- middle ------->|
 *              w/3 - overlap      2w/3 + overlap
 *                         |<------- right ------->|
 *                         2w/3 - overlap          w
 */

PixelCount orange_detection(struct image_t *img,
                             uint8_t lum_min, uint8_t lum_max,
                             uint8_t cb_min,  uint8_t cb_max,
                             uint8_t cr_min,  uint8_t cr_max)
{
  uint8_t *buffer = img->buf;

  uint16_t overlap      = img->w / 12;
  uint16_t one_third    = img->w / 3;

  uint16_t left_end     = one_third + overlap;
  uint16_t middle_start = one_third - overlap;
  uint16_t middle_end   = 2 * one_third + overlap;
  uint16_t right_start  = 2 * one_third - overlap;

  PixelCount result = {0, 0, 0};

  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {
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

      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min)  && (*up <= cb_max)  &&
          (*vp >= cr_min)  && (*vp <= cr_max)) {

        if (x < left_end)                        { result.left++;   }
        if (x >= middle_start && x < middle_end) { result.middle++; }
        if (x >= right_start)                    { result.right++;  }
      }
    }
  }

  return result;
}