#include "modules/computer_vision/MAV_cv_color_group12_cmjong.h"

// ── Scan-band configuration ────────────────────────────────────────────────
#define SCAN_NUM_LINES   5      // number of horizontal scan bands
#define SCAN_THICKNESS   1    // width of each band in pixels
#define SCAN_SPACING     20  // gap between band edges in pixels
// ──────────────────────────────────────────────────────────────────────────

static void draw_horizontal_line(uint8_t *buffer, uint16_t img_w, uint16_t img_h,
                                uint16_t x_col, uint8_t y_val, uint8_t u_val, uint8_t v_val)
{
  uint16_t x_even = (x_col % 2 == 0) ? x_col : x_col - 1;
  for (uint16_t y = 0; y < img_h; y++) {
    uint32_t base = y * 2 * img_w + 2 * x_even;
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
  uint8_t  *buffer = img->buf;
  uint16_t  w      = img->w;
  uint16_t  h      = img->h;

  // Total block = one band + one gap, bands are centred as a group in the image
  // Layout per band:  |<- SCAN_THICKNESS ->|<- SCAN_SPACING ->| (last gap ignored)
  uint16_t block     = SCAN_THICKNESS + SCAN_SPACING;
  uint16_t total_w   = block * SCAN_NUM_LINES - SCAN_SPACING; // drop trailing gap
  uint16_t x_offset  = (w > total_w) ? (w - total_w) / 2 : 0; // centre the group

  uint16_t band_start[SCAN_NUM_LINES];
  uint16_t band_end[SCAN_NUM_LINES];

  for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
    band_start[i] = x_offset + i * block;
    band_end[i]   = band_start[i] + SCAN_THICKNESS;
    // Clamp to image width
    if (band_start[i] >= w) band_start[i] = w;
    if (band_end[i]   >= w) band_end[i]   = w;
  }

  // Precompute which x columns are inside any band
  bool in_band[w];
  for (uint16_t x = 0; x < w; x++) {
    in_band[x] = false;
    for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
      if (x >= band_start[i] && x < band_end[i]) { in_band[x] = true; break; }
    }
  }

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      if (!in_band[x]) continue;

      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        up = &buffer[y * 2 * w + 2 * x];
        yp = &buffer[y * 2 * w + 2 * x + 1];
        vp = &buffer[y * 2 * w + 2 * x + 2];
      } else {
        up = &buffer[y * 2 * w + 2 * x - 2];
        vp = &buffer[y * 2 * w + 2 * x];
        yp = &buffer[y * 2 * w + 2 * x + 1];
      }

      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min)  && (*up <= cb_max)  &&
          (*vp >= cr_min)  && (*vp <= cr_max)) {
        total++;
      }
    }
  }

  if (draw) {
    for (uint8_t i = 0; i < SCAN_NUM_LINES; i++) {
      draw_horizontal_line(buffer, w, h, band_start[i],   235, 128, 128);
      draw_horizontal_line(buffer, w, h, band_end[i] - 1, 235, 128, 128);
    }
  }

  return (uint16_t)(total > 65535 ? 65535 : total);
}