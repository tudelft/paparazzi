#include "modules/computer_vision/MAV_cv_color_group12_cmjong.h"

// Scan-band configuration
#define SCAN_NUM_LINES   5   // number of horizontal scan bands
#define SCAN_THICKNESS   1   // width of each band in pixels
#define SCAN_SPACING     10  // gap between band edges in pixels
#ifndef COLOR_OBJECT_DETECTOR_DRAW_GUIDES
#define COLOR_OBJECT_DETECTOR_DRAW_GUIDES 0
#endif

/*
YUV422 memory layout (4 bytes per 2 pixels):
   [ U | Y1 | V | Y2 ]
     0    1   2    3
 Even pixel x reads:  U @ 2x,   Y1 @ 2x+1, V @ 2x+2
 Odd  pixel x reads:  U @ 2x-2, V @ 2x,    Y2 @ 2x+1

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
  if (draw && COLOR_OBJECT_DETECTOR_DRAW_GUIDES) {
    draw_horizontal_line(buffer, img->w, left_end,     255, 128, 128); // white
    draw_horizontal_line(buffer, img->w, middle_start, 255, 128, 128); // white
    draw_horizontal_line(buffer, img->w, middle_end,   255, 128, 128); // white
    draw_horizontal_line(buffer, img->w, right_start,  255, 128, 128); // white

    draw_horizontal_line(buffer, img->w, img->h / 6,       150, 90,  90);  // green  = left center
    draw_horizontal_line(buffer, img->w, img->h / 2,       150, 80,  180); // orange = middle center
    draw_horizontal_line(buffer, img->w, 5 * img->h / 6,   150, 90,  90);  // green  = right center
  }

  return (uint16_t)(total > 65535 ? 65535 : total);
}

// Column boundary fractions along the h-dimension (physical left-right).
// LEFT: 0–30%, CENTER: 25–75%, RIGHT: 70–100%.  5% overlap on each side.
#define COL_LEFT_FRAC    0.30f
#define COL_CENTER_START 0.25f
#define COL_CENTER_END   0.75f
#define COL_RIGHT_FRAC   0.70f

static void draw_row_line(uint8_t *buffer, uint16_t img_w, uint16_t img_h,
                          uint16_t y_row, uint8_t y_val, uint8_t u_val, uint8_t v_val)
{
  if (!buffer || img_w < 2 || y_row >= img_h) return;
  uint32_t row_base = (uint32_t)y_row * 2u * img_w;
  for (uint16_t x = 0; x < (img_w & ~1u); x += 2) {
    uint32_t base = row_base + 2u * x;
    buffer[base]     = u_val;
    buffer[base + 1] = y_val;
    buffer[base + 2] = v_val;
    buffer[base + 3] = y_val;
  }
}

struct column_counts color_detection_columns(struct image_t *img,
                                             uint8_t lum_min, uint8_t lum_max,
                                             uint8_t cb_min,  uint8_t cb_max,
                                             uint8_t cr_min,  uint8_t cr_max,
                                             bool draw)
{
  struct column_counts counts = {0, 0, 0};
  uint8_t *buffer = img->buf;
  uint16_t w      = img->w;
  uint16_t h      = img->h;
  uint16_t scan_w = w & ~1u;

  if (!buffer || scan_w < 2 || h == 0) return counts;

  // Column boundaries along h (y-axis = physical left-right)
  uint16_t left_end     = (uint16_t)(h * COL_LEFT_FRAC);
  uint16_t center_start = (uint16_t)(h * COL_CENTER_START);
  uint16_t center_end   = (uint16_t)(h * COL_CENTER_END);
  uint16_t right_start  = (uint16_t)(h * COL_RIGHT_FRAC);

  // Scan bands along x (same as color_detection)
  uint16_t block    = SCAN_THICKNESS + SCAN_SPACING;
  uint16_t total_w  = block * SCAN_NUM_LINES - SCAN_SPACING;
  uint16_t x_offset = (scan_w > total_w) ? (scan_w - total_w) / 2 : 0;

  uint16_t bs[SCAN_NUM_LINES], be[SCAN_NUM_LINES];
  for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
    bs[i] = x_offset + i * block;
    be[i] = bs[i] + SCAN_THICKNESS;
    if (bs[i] >= scan_w) { bs[i] = scan_w; be[i] = scan_w; continue; }
    if (be[i] > scan_w) be[i] = scan_w;
  }

  bool x_in_band[scan_w];
  for (uint16_t x = 0; x < scan_w; x++) {
    x_in_band[x] = false;
    for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
      if (x >= bs[i] && x < be[i]) { x_in_band[x] = true; break; }
    }
  }

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < scan_w; x++) {
      if (!x_in_band[x]) continue;

      uint32_t pair_base = y * 2u * w + 2u * (x & ~1u);
      uint8_t *up = &buffer[pair_base];
      uint8_t *vp = &buffer[pair_base + 2u];
      uint8_t *yp = &buffer[pair_base + ((x & 1u) ? 3u : 1u)];

      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min)  && (*up <= cb_max)  &&
          (*vp >= cr_min)  && (*vp <= cr_max)) {
        if (y < left_end)                         counts.left++;
        if (y >= center_start && y < center_end)  counts.center++;
        if (y >= right_start)                      counts.right++;
      }
    }
  }

  if (draw) {
    // Draw column boundary lines (horizontal lines in buffer = vertical in physical scene)
    draw_row_line(buffer, w, h, left_end,     0, 200, 128);   // left|center boundary (blue-ish)
    draw_row_line(buffer, w, h, center_start, 0, 200, 128);   // center overlap start
    draw_row_line(buffer, w, h, center_end,   0, 128, 200);   // center|right boundary (red-ish)
    draw_row_line(buffer, w, h, right_start,  0, 128, 200);   // right overlap start
  }

  return counts;
}
