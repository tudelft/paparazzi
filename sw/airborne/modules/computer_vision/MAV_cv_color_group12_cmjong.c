#include "modules/computer_vision/MAV_cv_color_group12_cmjong.h"

// Scan-band configuration
#define SCAN_NUM_LINES   5   // number of horizontal scan bands
#define SCAN_THICKNESS   1   // width of each band in pixels
#define SCAN_SPACING     20  // gap between band edges in pixels

static void draw_horizontal_line(uint8_t *buffer, uint16_t img_w, uint16_t img_h,
                                uint16_t x_col, uint8_t y_val, uint8_t u_val, uint8_t v_val)
{
  if (!buffer || img_w < 2 || x_col >= img_w) {
    return;
  }

  uint16_t x_even = x_col & ~1u;
  if (x_even + 1 >= img_w) {
    x_even = img_w - 2;
  }

  for (uint16_t y = 0; y < img_h; y++) {
    uint32_t base = y * 2u * img_w + 2u * x_even;
    buffer[base]     = u_val;
    buffer[base + 1] = y_val;
    buffer[base + 2] = v_val;
    buffer[base + 3] = y_val;
  }
}

uint16_t color_detection(struct image_t *img,
                         uint8_t lum_min, uint8_t lum_max,
                         uint8_t cb_min,  uint8_t cb_max,
                         uint8_t cr_min,  uint8_t cr_max,
                         bool draw)
{
  uint32_t total   = 0;
  uint8_t *buffer  = img->buf;
  uint16_t w       = img->w;
  uint16_t h       = img->h;
  uint16_t scan_w  = w & ~1u;

  if (!buffer || scan_w < 2 || h == 0) {
    return 0;
  }

  // YUV422 stores pixels in even/odd pairs; skip any trailing odd column.
  uint16_t block    = SCAN_THICKNESS + SCAN_SPACING;
  uint16_t total_w  = block * SCAN_NUM_LINES - SCAN_SPACING;
  uint16_t x_offset = (scan_w > total_w) ? (scan_w - total_w) / 2 : 0;

  uint16_t band_start[SCAN_NUM_LINES];
  uint16_t band_end[SCAN_NUM_LINES];

  for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
    band_start[i] = x_offset + i * block;
    band_end[i] = band_start[i] + SCAN_THICKNESS;

    if (band_start[i] >= scan_w) {
      band_start[i] = scan_w;
      band_end[i] = scan_w;
      continue;
    }
    if (band_end[i] > scan_w) {
      band_end[i] = scan_w;
    }
  }

  bool in_band[scan_w];
  for (uint16_t x = 0; x < scan_w; x++) {
    in_band[x] = false;
    for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
      if (x >= band_start[i] && x < band_end[i]) {
        in_band[x] = true;
        break;
      }
    }
  }

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < scan_w; x++) {
      if (!in_band[x]) {
        continue;
      }

      uint32_t pair_base = y * 2u * w + 2u * (x & ~1u);
      uint8_t *up = &buffer[pair_base];
      uint8_t *vp = &buffer[pair_base + 2u];
      uint8_t *yp = &buffer[pair_base + ((x & 1u) ? 3u : 1u)];

      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min)  && (*up <= cb_max)  &&
          (*vp >= cr_min)  && (*vp <= cr_max)) {
        total++;
      }
    }
  }

  if (draw) {
    for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
      if (band_start[i] >= scan_w || band_end[i] <= band_start[i]) {
        continue;
      }
      draw_horizontal_line(buffer, w, h, band_start[i], 235, 128, 128);
      draw_horizontal_line(buffer, w, h, band_end[i] - 1u, 235, 128, 128);
    }
  }

  return (uint16_t)(total > 65535 ? 65535 : total);
}
