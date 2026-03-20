#include "MAV_cv_color_group12_cmjong.h"

#ifndef COLOR_OBJECT_DETECTOR_DRAW_GUIDES
#define COLOR_OBJECT_DETECTOR_DRAW_GUIDES 0
#endif

/*
YUV422 memory layout (4 bytes per 2 pixels):
   [ U | Y1 | V | Y2 ]
     0    1   2    3
 Even pixel x reads:  U @ 2x,   Y1 @ 2x+1, V @ 2x+2
 Odd  pixel x reads:  U @ 2x-2, V @ 2x,    Y2 @ 2x+1

 CHANGED: regions are now split on Y axis (rows) instead of X axis (columns)
 because the Bebop front camera is mounted rotated 90 degrees CW.
 "left" = top of physical image, "right" = bottom of physical image.

Region layout (overlap = img->h / 12, half_overlap = img->h / 24):
   |<------- left ------->|
   0              h/3 + overlap
           |<-- middle -->|
           h/3 - half_overlap    2h/3 + half_overlap
                      |<------- right ------->|
                      2h/3 - overlap          h
*/


static void draw_horizontal_line(uint8_t *buffer, uint16_t img_w,
                                  uint16_t y_row, uint8_t y_val, uint8_t u_val, uint8_t v_val)
{
  for (uint16_t x = 0; x < img_w; x++) {
    uint16_t x_even = (x % 2 == 0) ? x : x - 1;
    uint32_t base = y_row * 2 * img_w + 2 * x_even;
    buffer[base]     = u_val;
    buffer[base + 1] = y_val;
    buffer[base + 2] = v_val;
    buffer[base + 3] = y_val;
  }
}

PixelCount color_detection(struct image_t *img,
                             uint8_t lum_min, uint8_t lum_max,
                             uint8_t cb_min,  uint8_t cb_max,
                             uint8_t cr_min,  uint8_t cr_max,
                             bool draw)
{
  uint8_t *buffer = img->buf;

  uint16_t overlap      = img->h / 12;
  uint16_t half_overlap = img->h / 24;
  uint16_t one_third    = img->h / 3;

  uint16_t left_end     = one_third + overlap;
  uint16_t middle_start = one_third - half_overlap;
  uint16_t middle_end   = 2 * one_third + half_overlap;
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

        if (y < left_end)                        { result.left++;   }
        if (y >= middle_start && y < middle_end) { result.middle++; }
        if (y >= right_start)                    { result.right++;  }
      }
    }
  }

  if (draw && COLOR_OBJECT_DETECTOR_DRAW_GUIDES) {
    draw_horizontal_line(buffer, img->w, left_end,     255, 128, 128); // white
    draw_horizontal_line(buffer, img->w, middle_start, 255, 128, 128); // white
    draw_horizontal_line(buffer, img->w, middle_end,   255, 128, 128); // white
    draw_horizontal_line(buffer, img->w, right_start,  255, 128, 128); // white

    draw_horizontal_line(buffer, img->w, img->h / 6,       150, 90,  90);  // green  = left center
    draw_horizontal_line(buffer, img->w, img->h / 2,       150, 80,  180); // orange = middle center
    draw_horizontal_line(buffer, img->w, 5 * img->h / 6,   150, 90,  90);  // green  = right center
  }

  return result;
}