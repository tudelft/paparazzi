/** @file ear_heatmap.c @brief JPEG heatmap rendering for geotagged EARcam sample sessions. */
#include "ear_heatmap.h"

#include <math.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <jpeglib.h>

#define HEATMAP_MIN_CELL_M 1.0
#define HEATMAP_MAX_CELLS 96U
#define HEATMAP_MAX_OUTPUT_PX 768U
#define HEATMAP_QUALITY 90
#define HEATMAP_MIN_SAMPLES 3U
#define EARTH_RADIUS_M 6378137.0
#define DEG_TO_RAD 0.017453292519943295

struct heatmap_jpeg_error {
  struct jpeg_error_mgr manager;
  jmp_buf recovery;
};

static void heatmap_jpeg_error_exit(j_common_ptr info)
{
  struct heatmap_jpeg_error *error = (struct heatmap_jpeg_error *)info->err;
  longjmp(error->recovery, 1);
}

// Dark blue -> cyan -> yellow -> red; t in 0..1.
static void colormap(double t, uint8_t *rgb)
{
  if (t < 0.0) { t = 0.0; }
  if (t > 1.0) { t = 1.0; }
  double r, g, b;
  if (t < 0.33) {
    double u = t / 0.33;
    r = 0.05; g = 0.10 + 0.70 * u; b = 0.35 + 0.55 * u;
  } else if (t < 0.66) {
    double u = (t - 0.33) / 0.33;
    r = 0.05 + 0.90 * u; g = 0.80 + 0.15 * u; b = 0.90 - 0.85 * u;
  } else {
    double u = (t - 0.66) / 0.34;
    r = 0.95; g = 0.95 - 0.80 * u; b = 0.05;
  }
  rgb[0] = (uint8_t)(r * 255.0 + 0.5);
  rgb[1] = (uint8_t)(g * 255.0 + 0.5);
  rgb[2] = (uint8_t)(b * 255.0 + 0.5);
}

static void draw_disc(uint8_t *pixels, unsigned int width, unsigned int height,
                      double cx, double cy, double radius, const uint8_t *rgb, bool ring)
{
  int x0 = (int)floor(cx - radius - 1.0);
  int x1 = (int)ceil(cx + radius + 1.0);
  int y0 = (int)floor(cy - radius - 1.0);
  int y1 = (int)ceil(cy + radius + 1.0);
  for (int y = y0; y <= y1; y++) {
    if (y < 0 || (unsigned int)y >= height) { continue; }
    for (int x = x0; x <= x1; x++) {
      if (x < 0 || (unsigned int)x >= width) { continue; }
      double d = hypot(x + 0.5 - cx, y + 0.5 - cy);
      bool inside = ring ? (d <= radius && d >= radius - 2.0) : d <= radius;
      if (inside) {
        uint8_t *p = &pixels[((size_t)y * width + x) * 3];
        p[0] = rgb[0]; p[1] = rgb[1]; p[2] = rgb[2];
      }
    }
  }
}

static double clamp01(double v)
{
  return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
}

// One-pixel white dot with a one-pixel black ring, antialiased by radial coverage.
static void draw_dot(uint8_t *pixels, unsigned int width, unsigned int height, double cx, double cy)
{
  int x0 = (int)floor(cx - 2.5), x1 = (int)ceil(cx + 2.5);
  int y0 = (int)floor(cy - 2.5), y1 = (int)ceil(cy + 2.5);
  for (int y = y0; y <= y1; y++) {
    if (y < 0 || (unsigned int)y >= height) { continue; }
    for (int x = x0; x <= x1; x++) {
      if (x < 0 || (unsigned int)x >= width) { continue; }
      double d = hypot(x + 0.5 - cx, y + 0.5 - cy);
      double white = clamp01(1.0 - d);          // core: radius 0.5 px, soft edge
      double black = clamp01(2.0 - d) - white;  // ring: 1 px wide around the core
      if (white + black <= 0.0) { continue; }
      uint8_t *p = &pixels[((size_t)y * width + x) * 3];
      for (int c = 0; c < 3; c++) {
        double v = p[c] * (1.0 - white - black) + 255.0 * white;
        p[c] = (uint8_t)(v + 0.5);
      }
    }
  }
}

// Georeference sidecar next to the JPEG so ground tools can overlay it on a map.
static void write_georef(const char *filename, unsigned int width, unsigned int height,
                         double lat_c, double lon_c, double cos_lat, double half_e, double half_n,
                         double m_per_px, const struct ear_loudest_spot *spot)
{
  char path[512];
  size_t n = strlen(filename);
  if (n + 5 > sizeof(path)) {
    return;
  }
  size_t stem = (n > 4 && strcmp(filename + n - 4, ".jpg") == 0) ? n - 4 : n;
  for (size_t i = 0; i < stem; i++) {
    path[i] = filename[i];
  }
  path[stem] = '\0';
  strcat(path, ".geo");
  FILE *f = fopen(path, "w");
  if (f == NULL) {
    return;
  }
  double m_per_deg_lat = DEG_TO_RAD * EARTH_RADIUS_M;
  double m_per_deg_lon = m_per_deg_lat * cos_lat;
  fprintf(f, "# ear heatmap georeference: equirectangular about the centre, north up\n");
  fprintf(f, "width_px=%u\nheight_px=%u\nm_per_px=%.6f\n", width, height, m_per_px);
  fprintf(f, "lat_center=%.8f\nlon_center=%.8f\n", lat_c, lon_c);
  fprintf(f, "lat_north=%.8f\nlon_west=%.8f\n", lat_c + half_n / m_per_deg_lat, lon_c - half_e / m_per_deg_lon);
  fprintf(f, "lat_south=%.8f\nlon_east=%.8f\n", lat_c + (half_n - height * m_per_px) / m_per_deg_lat,
          lon_c + (width * m_per_px - half_e) / m_per_deg_lon);
  if (spot != NULL && spot->valid) {
    fprintf(f, "spot_lat=%.8f\nspot_lon=%.8f\nspot_level_db=%.2f\nspot_confidence=%.3f\n",
            spot->lat_deg, spot->lon_deg, spot->level_db, spot->confidence);
  }
  fclose(f);
}

static int encode_jpeg(const char *filename, const uint8_t *pixels,
                       unsigned int width, unsigned int height)
{
  FILE *output = fopen(filename, "wb");
  if (output == NULL) {
    return -1;
  }
  struct jpeg_compress_struct encoder;
  struct heatmap_jpeg_error error;
  encoder.err = jpeg_std_error(&error.manager);
  error.manager.error_exit = heatmap_jpeg_error_exit;
  if (setjmp(error.recovery) != 0) {
    jpeg_destroy_compress(&encoder);
    fclose(output);
    return -1;
  }
  jpeg_create_compress(&encoder);
  jpeg_stdio_dest(&encoder, output);
  encoder.image_width = width;
  encoder.image_height = height;
  encoder.input_components = 3;
  encoder.in_color_space = JCS_RGB;
  jpeg_set_defaults(&encoder);
  jpeg_set_quality(&encoder, HEATMAP_QUALITY, TRUE);
  jpeg_start_compress(&encoder, TRUE);
  while (encoder.next_scanline < encoder.image_height) {
    JSAMPROW row[1] = {(JSAMPROW)&pixels[(size_t)encoder.next_scanline * width * 3]};
    jpeg_write_scanlines(&encoder, row, 1);
  }
  jpeg_finish_compress(&encoder);
  jpeg_destroy_compress(&encoder);
  return fclose(output) == 0 ? 0 : -1;
}

static int heatmap_write(const char *filename, const struct ear_sample *samples, size_t count,
                         const struct ear_loudest_spot *spot, bool decorate)
{
  if (filename == NULL || samples == NULL || count < HEATMAP_MIN_SAMPLES) {
    return -1;
  }

  // Bounding box and typical AGL over the usable samples.
  double lat_min = 1e9, lat_max = -1e9, lon_min = 1e9, lon_max = -1e9;
  double agl_sum = 0.0;
  size_t usable = 0;
  for (size_t index = 0; index < count; index++) {
    const struct ear_sample *s = &samples[index];
    if (s->clipped || !isfinite(s->lat_deg) || !isfinite(s->lon_deg)
        || !isfinite(s->level_db) || fabs(s->lat_deg) <= 1e-6) {
      continue;
    }
    if (s->lat_deg < lat_min) { lat_min = s->lat_deg; }
    if (s->lat_deg > lat_max) { lat_max = s->lat_deg; }
    if (s->lon_deg < lon_min) { lon_min = s->lon_deg; }
    if (s->lon_deg > lon_max) { lon_max = s->lon_deg; }
    agl_sum += (isfinite(s->agl_m) && s->agl_m > 0.5) ? s->agl_m : 10.0;
    usable++;
  }
  if (usable < HEATMAP_MIN_SAMPLES) {
    return -1;
  }
  double agl = agl_sum / (double)usable;
  double cell_m = fmax(HEATMAP_MIN_CELL_M, agl / 4.0);
  double sigma_m = fmax(cell_m, agl / 2.0);   // acoustic footprint of one sample

  // Local ENU about the box centre, with one footprint of margin.
  double lat_c = 0.5 * (lat_min + lat_max);
  double lon_c = 0.5 * (lon_min + lon_max);
  double cos_lat = cos(lat_c * DEG_TO_RAD);
  double half_e = (lon_max - lon_min) * 0.5 * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat + agl;
  double half_n = (lat_max - lat_min) * 0.5 * DEG_TO_RAD * EARTH_RADIUS_M + agl;
  if (half_e < 3.0 * cell_m) { half_e = 3.0 * cell_m; }
  if (half_n < 3.0 * cell_m) { half_n = 3.0 * cell_m; }
  unsigned int cells_x = (unsigned int)ceil(2.0 * half_e / cell_m);
  unsigned int cells_y = (unsigned int)ceil(2.0 * half_n / cell_m);
  if (cells_x > HEATMAP_MAX_CELLS || cells_y > HEATMAP_MAX_CELLS) {
    double scale = fmax((double)cells_x, (double)cells_y) / HEATMAP_MAX_CELLS;
    cell_m *= scale;
    cells_x = (unsigned int)ceil(2.0 * half_e / cell_m);
    cells_y = (unsigned int)ceil(2.0 * half_n / cell_m);
  }

  double *field = calloc((size_t)cells_x * cells_y, sizeof(*field));
  double *weight = calloc((size_t)cells_x * cells_y, sizeof(*weight));
  if (field == NULL || weight == NULL) {
    free(field);
    free(weight);
    return -1;
  }

  // Gaussian footprint deposit of each sample's level (in linear power).
  double level_min = 1e9, level_max = -1e9;
  double inv_two_sigma2 = 1.0 / (2.0 * sigma_m * sigma_m);
  int reach = (int)ceil(3.0 * sigma_m / cell_m);
  for (size_t index = 0; index < count; index++) {
    const struct ear_sample *s = &samples[index];
    if (s->clipped || !isfinite(s->lat_deg) || !isfinite(s->lon_deg)
        || !isfinite(s->level_db) || fabs(s->lat_deg) <= 1e-6) {
      continue;
    }
    double e = (s->lon_deg - lon_c) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
    double n = (s->lat_deg - lat_c) * DEG_TO_RAD * EARTH_RADIUS_M;
    double gx = (e + half_e) / cell_m;
    double gy = (half_n - n) / cell_m;   // north up
    double power = pow(10.0, s->level_db / 10.0);
    int cx0 = (int)floor(gx), cy0 = (int)floor(gy);
    for (int y = cy0 - reach; y <= cy0 + reach; y++) {
      if (y < 0 || (unsigned int)y >= cells_y) { continue; }
      for (int x = cx0 - reach; x <= cx0 + reach; x++) {
        if (x < 0 || (unsigned int)x >= cells_x) { continue; }
        double dx = (x + 0.5 - gx) * cell_m;
        double dy = (y + 0.5 - gy) * cell_m;
        double w = exp(-(dx * dx + dy * dy) * inv_two_sigma2);
        size_t k = (size_t)y * cells_x + x;
        field[k] += w * power;
        weight[k] += w;
      }
    }
  }
  for (size_t k = 0; k < (size_t)cells_x * cells_y; k++) {
    if (weight[k] > 1e-3) {
      field[k] = 10.0 * log10(field[k] / weight[k]);
      if (field[k] < level_min) { level_min = field[k]; }
      if (field[k] > level_max) { level_max = field[k]; }
    } else {
      field[k] = NAN;
    }
  }
  if (!(level_max > level_min)) {
    level_max = level_min + 1.0;
  }
  // Emphasize the top of the dynamic range; the quiet floor is not informative.
  double span = fmin(level_max - level_min, 20.0);
  double floor_db = level_max - span;

  // Output image: upsample cells bilinearly so the viewer sees smooth blobs.
  // The grid (not the pixel count) carries the microphone's resolution limit.
  unsigned int cells_max = cells_x > cells_y ? cells_x : cells_y;
  double scale = (double)HEATMAP_MAX_OUTPUT_PX / (double)cells_max;
  unsigned int width = (unsigned int)ceil(cells_x * scale);
  unsigned int height = (unsigned int)ceil(cells_y * scale);
  if (width < 1U) { width = 1U; }
  if (height < 1U) { height = 1U; }
  // The long side is HEATMAP_MAX_OUTPUT_PX by construction; bound both anyway.
  if (width > HEATMAP_MAX_OUTPUT_PX) { width = HEATMAP_MAX_OUTPUT_PX; }
  if (height > HEATMAP_MAX_OUTPUT_PX) { height = HEATMAP_MAX_OUTPUT_PX; }
  uint8_t *pixels = malloc((size_t)width * height * 3);
  if (pixels == NULL) {
    free(field);
    free(weight);
    return -1;
  }
  for (unsigned int y = 0; y < height; y++) {
    double fy = (y + 0.5) / scale - 0.5;
    int y0 = (int)floor(fy); double ty = fy - y0;
    for (unsigned int x = 0; x < width; x++) {
      double fx = (x + 0.5) / scale - 0.5;
      int x0 = (int)floor(fx); double tx = fx - x0;
      double sum = 0.0, wsum = 0.0;
      for (int j = 0; j < 2; j++) {
        int yy = y0 + j;
        if (yy < 0 || (unsigned int)yy >= cells_y) { continue; }
        for (int i = 0; i < 2; i++) {
          int xx = x0 + i;
          if (xx < 0 || (unsigned int)xx >= cells_x) { continue; }
          double v = field[(size_t)yy * cells_x + xx];
          if (isnan(v)) { continue; }
          double w = (i ? tx : 1.0 - tx) * (j ? ty : 1.0 - ty);
          sum += w * v;
          wsum += w;
        }
      }
      uint8_t *p = &pixels[((size_t)y * width + x) * 3];
      if (wsum > 1e-6) {
        colormap((sum / wsum - floor_db) / span, p);
      } else {
        p[0] = 18; p[1] = 18; p[2] = 24;   // no data
      }
    }
  }

  // Track dots and the fused loudest-spot marker.
  for (size_t index = 0; decorate && index < count; index++) {
    const struct ear_sample *s = &samples[index];
    if (!isfinite(s->lat_deg) || !isfinite(s->lon_deg) || fabs(s->lat_deg) <= 1e-6) {
      continue;
    }
    double e = (s->lon_deg - lon_c) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
    double n = (s->lat_deg - lat_c) * DEG_TO_RAD * EARTH_RADIUS_M;
    double px = (e + half_e) / cell_m * scale;
    double py = (half_n - n) / cell_m * scale;
    draw_dot(pixels, width, height, px, py);
  }
  if (decorate && spot != NULL && spot->valid) {
    // Loudest spot: magenta crosshair with a small ring, black casing.
    const uint8_t magenta[3] = {255, 40, 255};
    const uint8_t black[3] = {0, 0, 0};
    double e = (spot->lon_deg - lon_c) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
    double n = (spot->lat_deg - lat_c) * DEG_TO_RAD * EARTH_RADIUS_M;
    double px = (e + half_e) / cell_m * scale;
    double py = (half_n - n) / cell_m * scale;
    const double ring = 9.0, arm = 22.0, gap = 4.0;
    for (int pass = 0; pass < 2; pass++) {
      const uint8_t *rgb = pass == 0 ? black : magenta;
      double r = pass == 0 ? 2.4 : 1.2;
      for (int t = (int)gap; t <= (int)arm; t++) {
        draw_disc(pixels, width, height, px + t, py, r, rgb, false);
        draw_disc(pixels, width, height, px - t, py, r, rgb, false);
        draw_disc(pixels, width, height, px, py + t, r, rgb, false);
        draw_disc(pixels, width, height, px, py - t, r, rgb, false);
      }
    }
    draw_disc(pixels, width, height, px, py, ring + 2.0, black, true);
    draw_disc(pixels, width, height, px, py, ring, magenta, true);
    draw_disc(pixels, width, height, px, py, 1.5, magenta, false);
  }

  // 50 m scale bar, bottom left, with end ticks.
  if (decorate) {
    const uint8_t white[3] = {255, 255, 255};
    const uint8_t black[3] = {0, 0, 0};
    double bar_px = 50.0 / cell_m * scale;
    if (bar_px > 0.6 * width) { bar_px = 0.6 * width; }
    int bar_steps = (int)floor(bar_px);
    double x0 = 0.04 * width;
    double y0 = height - 0.06 * height;
    for (int t = 0; t <= bar_steps; t++) {
      draw_disc(pixels, width, height, x0 + t, y0, 2.5, black, false);
    }
    for (int t = 0; t <= bar_steps; t++) {
      draw_disc(pixels, width, height, x0 + t, y0, 1.2, white, false);
    }
    for (int dy = -6; dy <= 6; dy++) {
      draw_disc(pixels, width, height, x0, y0 + dy, 1.2, white, false);
      draw_disc(pixels, width, height, x0 + bar_px, y0 + dy, 1.2, white, false);
    }
  }

  int status = encode_jpeg(filename, pixels, width, height);
  if (status == 0) {
    write_georef(filename, width, height, lat_c, lon_c, cos_lat, half_e, half_n, cell_m / scale, spot);
  }
  free(pixels);
  free(field);
  free(weight);
  return status;
}

int ear_heatmap_write(const char *filename, const struct ear_sample *samples, size_t count,
                      const struct ear_loudest_spot *spot)
{
  return heatmap_write(filename, samples, count, spot, true);
}

int ear_heatmap_write_field(const char *filename, const struct ear_sample *samples, size_t count,
                            const struct ear_loudest_spot *spot)
{
  return heatmap_write(filename, samples, count, spot, false);
}
