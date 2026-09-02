#include "image_mock_transform.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <jpeglib.h>

#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232
#define JPEG_COMPONENTS 3U
#define JPEG_QUALITY 95
#define FOCAL_LENGTH_FACTOR 0.9
#define PROJECTION_EPSILON 1e-9
#define PROJECTION_MIN_DEPTH 1e-4
#define PROJECTION_FOV_SAMPLES 192
#define PROJECTION_REFINE_STEPS 48
#define PROJECTION_MIN_RAY_SCALE 1e-4
#define COVER_SAFETY_FACTOR 0.999

#ifndef near_horizon_case_black
#define near_horizon_case_black 0
#endif

#ifndef near_horizon_roll_deg
#define near_horizon_roll_deg 75.0
#endif

#ifndef near_horizon_pitch_deg
#define near_horizon_pitch_deg 75.0
#endif

#ifndef near_horizon_combined_tilt_deg
#define near_horizon_combined_tilt_deg 70.0
#endif

#if near_horizon_case_black != 0 && near_horizon_case_black != 1
#error "near_horizon_case_black must be 0 or 1"
#endif

struct projection_fit {
  double ray_scale;
  double image_scale;
  double offset_x;
  double offset_y;
};

struct transform_jpeg_error {
  struct jpeg_error_mgr manager;
  jmp_buf recovery;
};

struct decoded_image {
  unsigned char *pixels;
  size_t width;
  size_t height;
};

static void transform_jpeg_error_exit(j_common_ptr jpeg_info);
static bool is_near_horizon(double roll_rad, double pitch_rad);
static int read_jpeg_dimensions(FILE *input, struct decoded_image *image);
static int decode_jpeg(FILE *input, struct decoded_image *image);
static int encode_jpeg(FILE *output, const struct decoded_image *image);
static int transform_pixels(const struct decoded_image *source, struct decoded_image *destination,
                            double roll_rad, double pitch_rad, double yaw_rad,
                            bool synthesize_terrain);
static void build_rotation(double roll_rad, double pitch_rad, double yaw_rad,
                           double rotation[3][3]);
static bool project_point(double rotation[3][3], double focal_length,
                          double center_x, double center_y, double ray_scale,
                          double destination_x, double destination_y,
                          double *source_x, double *source_y);
static bool projection_corners_valid(const struct decoded_image *source,
                                     double rotation[3][3], double focal_length,
                                     double center_x, double center_y, double ray_scale);
static int compute_projection_fit_at_scale(const struct decoded_image *source,
                                           double rotation[3][3], double focal_length,
                                           double center_x, double center_y, double ray_scale,
                                           struct projection_fit *fit);
static int compute_projection_fit(const struct decoded_image *source,
                                  double rotation[3][3], double focal_length,
                                  double center_x, double center_y,
                                  struct projection_fit *fit);
static void bilinear_sample_rgb(const struct decoded_image *image, double source_x,
                                double source_y, unsigned char output[JPEG_COMPONENTS]);
static double mirror_coordinate(double coordinate, size_t extent);
static int create_black_frame(const char *filename);
static int replace_image_atomically(const char *filename, const struct decoded_image *image);

int image_mock_transform(const char *filename, const union dc_shot_union *shot)
{
  if (filename == NULL || shot == NULL) {
    errno = EINVAL;
    return -1;
  }
  if (!isfinite(near_horizon_roll_deg) || near_horizon_roll_deg < 0.0
      || near_horizon_roll_deg > 180.0
      || !isfinite(near_horizon_pitch_deg) || near_horizon_pitch_deg < 0.0
      || near_horizon_pitch_deg > 180.0
      || !isfinite(near_horizon_combined_tilt_deg)
      || near_horizon_combined_tilt_deg < 0.0
      || near_horizon_combined_tilt_deg > 180.0) {
    fprintf(stderr, "MOCK_TRANSFORM:\tnear-horizon thresholds must be between 0 and 180 degrees\n");
    errno = EINVAL;
    return -1;
  }

  double altitude_m = shot->data.alt / 1000.0;
  double ground_altitude_m = shot->data.groundalt / POSITION_BFP_SCALE;
  double roll_rad = shot->data.phi / ANGLE_BFP_SCALE;
  double pitch_rad = shot->data.theta / ANGLE_BFP_SCALE;
  double yaw_rad = shot->data.psi / ANGLE_BFP_SCALE;
  double speed_m_s = shot->data.vground / SPEED_BFP_SCALE;
  double course_deg = shot->data.course / ANGLE_BFP_SCALE * RAD_TO_DEG;
  double blur_factor = speed_m_s / 100.0;
  bool near_horizon = is_near_horizon(roll_rad, pitch_rad);

  printf("MOCK_TRANSFORM:\talt %.3f m AGL %.3f m | roll %.3f pitch %.3f yaw %.3f deg | "
         "speed %.3f m/s course %.3f deg | blur factor %.6f (not applied)\n",
         altitude_m, ground_altitude_m,
         roll_rad * RAD_TO_DEG, pitch_rad * RAD_TO_DEG, yaw_rad * RAD_TO_DEG,
         speed_m_s, course_deg, blur_factor);

  if (near_horizon && near_horizon_case_black) {
    printf("MOCK_TRANSFORM:\tnear-horizon attitude: generating black frame\n");
    return create_black_frame(filename);
  }

  if (!near_horizon && fabs(roll_rad) < PROJECTION_EPSILON
      && fabs(pitch_rad) < PROJECTION_EPSILON && fabs(yaw_rad) < PROJECTION_EPSILON) {
    return IMAGE_MOCK_TRANSFORM_SKIPPED;
  }

  FILE *input = fopen(filename, "rb");
  if (input == NULL) {
    fprintf(stderr, "MOCK_TRANSFORM:\tfailed to open %s: %s\n", filename, strerror(errno));
    return -1;
  }

  struct decoded_image source = {NULL, 0, 0};
  if (decode_jpeg(input, &source) != 0) {
    fprintf(stderr, "MOCK_TRANSFORM:\tfailed to decode JPEG %s\n", filename);
    fclose(input);
    return -1;
  }
  if (fclose(input) != 0) {
    free(source.pixels);
    return -1;
  }

  struct decoded_image transformed = {NULL, source.width, source.height};
  int result = transform_pixels(&source, &transformed, roll_rad, pitch_rad, yaw_rad,
                                near_horizon);
  free(source.pixels);
  if (result < 0) {
    return -1;
  }
  if (result == IMAGE_MOCK_TRANSFORM_SKIPPED) {
    return result;
  }

  result = replace_image_atomically(filename, &transformed);
  free(transformed.pixels);
  return result;
}

static void transform_jpeg_error_exit(j_common_ptr jpeg_info)
{
  struct transform_jpeg_error *error = (struct transform_jpeg_error *)jpeg_info->err;
  longjmp(error->recovery, 1);
}

static bool is_near_horizon(double roll_rad, double pitch_rad)
{
  double absolute_roll_deg = fabs(roll_rad * RAD_TO_DEG);
  double absolute_pitch_deg = fabs(pitch_rad * RAD_TO_DEG);
  double optical_axis_cosine = cos(roll_rad) * cos(pitch_rad);
  optical_axis_cosine = fmax(-1.0, fmin(1.0, optical_axis_cosine));
  double combined_tilt_deg = acos(optical_axis_cosine) * RAD_TO_DEG;
  return absolute_roll_deg >= near_horizon_roll_deg
         || absolute_pitch_deg >= near_horizon_pitch_deg
         || combined_tilt_deg >= near_horizon_combined_tilt_deg;
}

static int read_jpeg_dimensions(FILE *input, struct decoded_image *image)
{
  struct jpeg_decompress_struct *decoder = calloc(1, sizeof(*decoder));
  if (decoder == NULL) {
    return -1;
  }
  struct transform_jpeg_error error;
  decoder->err = jpeg_std_error(&error.manager);
  error.manager.error_exit = transform_jpeg_error_exit;
  if (setjmp(error.recovery) != 0) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    return -1;
  }

  jpeg_create_decompress(decoder);
  jpeg_stdio_src(decoder, input);
  if (jpeg_read_header(decoder, TRUE) != JPEG_HEADER_OK
      || decoder->image_width < 2 || decoder->image_height < 2) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    errno = EINVAL;
    return -1;
  }
  image->width = decoder->image_width;
  image->height = decoder->image_height;
  jpeg_destroy_decompress(decoder);
  free(decoder);
  return 0;
}

static int decode_jpeg(FILE *input, struct decoded_image *image)
{
  struct jpeg_decompress_struct *decoder = calloc(1, sizeof(*decoder));
  if (decoder == NULL) {
    return -1;
  }
  struct transform_jpeg_error error;
  decoder->err = jpeg_std_error(&error.manager);
  error.manager.error_exit = transform_jpeg_error_exit;
  if (setjmp(error.recovery) != 0) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    free(image->pixels);
    image->pixels = NULL;
    return -1;
  }

  jpeg_create_decompress(decoder);
  jpeg_stdio_src(decoder, input);
  if (jpeg_read_header(decoder, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    return -1;
  }
  decoder->out_color_space = JCS_RGB;
  jpeg_start_decompress(decoder);

  image->width = decoder->output_width;
  image->height = decoder->output_height;
  if (decoder->output_components != JPEG_COMPONENTS || image->width < 2 || image->height < 2
      || image->width > SIZE_MAX / JPEG_COMPONENTS
      || image->height > SIZE_MAX / (image->width * JPEG_COMPONENTS)) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    errno = EOVERFLOW;
    return -1;
  }

  size_t row_size = image->width * JPEG_COMPONENTS;
  image->pixels = malloc(row_size * image->height);
  if (image->pixels == NULL) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    return -1;
  }

  while (decoder->output_scanline < decoder->output_height) {
    JSAMPROW row[1] = {&image->pixels[(size_t)decoder->output_scanline * row_size]};
    if (jpeg_read_scanlines(decoder, row, 1) != 1) {
      jpeg_destroy_decompress(decoder);
      free(decoder);
      free(image->pixels);
      image->pixels = NULL;
      return -1;
    }
  }

  jpeg_finish_decompress(decoder);
  jpeg_destroy_decompress(decoder);
  free(decoder);
  return 0;
}

static int encode_jpeg(FILE *output, const struct decoded_image *image)
{
  struct jpeg_compress_struct *encoder = calloc(1, sizeof(*encoder));
  if (encoder == NULL) {
    return -1;
  }
  struct transform_jpeg_error error;
  encoder->err = jpeg_std_error(&error.manager);
  error.manager.error_exit = transform_jpeg_error_exit;
  if (setjmp(error.recovery) != 0) {
    jpeg_destroy_compress(encoder);
    free(encoder);
    return -1;
  }

  jpeg_create_compress(encoder);
  jpeg_stdio_dest(encoder, output);
  encoder->image_width = (JDIMENSION)image->width;
  encoder->image_height = (JDIMENSION)image->height;
  encoder->input_components = JPEG_COMPONENTS;
  encoder->in_color_space = JCS_RGB;
  jpeg_set_defaults(encoder);
  jpeg_set_quality(encoder, JPEG_QUALITY, TRUE);
  jpeg_start_compress(encoder, TRUE);

  size_t row_size = image->width * JPEG_COMPONENTS;
  while (encoder->next_scanline < encoder->image_height) {
    JSAMPROW row[1] = {(JSAMPROW)&image->pixels[(size_t)encoder->next_scanline * row_size]};
    if (jpeg_write_scanlines(encoder, row, 1) != 1) {
      jpeg_destroy_compress(encoder);
      free(encoder);
      return -1;
    }
  }

  jpeg_finish_compress(encoder);
  jpeg_destroy_compress(encoder);
  free(encoder);
  return 0;
}

static int transform_pixels(const struct decoded_image *source, struct decoded_image *destination,
                            double roll_rad, double pitch_rad, double yaw_rad,
                            bool synthesize_terrain)
{
  double center_x = ((double)source->width - 1.0) / 2.0;
  double center_y = ((double)source->height - 1.0) / 2.0;
  double largest_dimension = source->width > source->height ? source->width : source->height;
  double focal_length = largest_dimension * FOCAL_LENGTH_FACTOR;
  double rotation[3][3];
  build_rotation(roll_rad, pitch_rad, yaw_rad, rotation);
  if (rotation[2][2] <= PROJECTION_MIN_DEPTH) {
    synthesize_terrain = true;
  }
  struct projection_fit fit;
  if (synthesize_terrain) {
    fit.ray_scale = 1.0;
    fit.image_scale = 1.0;
    fit.offset_x = 0.0;
    fit.offset_y = 0.0;
    printf("MOCK_TRANSFORM:\tnear-horizon attitude: synthesizing extended terrain\n");
  } else if (compute_projection_fit(source, rotation, focal_length, center_x, center_y, &fit) != 0) {
    fprintf(stderr, "MOCK_TRANSFORM:\tattitude is outside the usable single-image camera model\n");
    return -1;
  }

  size_t pixel_count = source->width * source->height;
  if (pixel_count > SIZE_MAX / JPEG_COMPONENTS) {
    errno = EOVERFLOW;
    return -1;
  }
  destination->pixels = malloc(pixel_count * JPEG_COMPONENTS);
  if (destination->pixels == NULL) {
    return -1;
  }
  if (!synthesize_terrain) {
    double cover_zoom = 1.0 / (fit.ray_scale * fit.image_scale);
    double pan_x = fit.offset_x + (fit.image_scale - 1.0) * center_x;
    double pan_y = fit.offset_y + (fit.image_scale - 1.0) * center_y;
    printf("MOCK_TRANSFORM:\tborderless cover crop: %.3fx zoom, pan %.1f px x %.1f px\n",
           cover_zoom, pan_x, pan_y);
  }

  double ray_x_start = -fit.ray_scale * center_x / focal_length;
  double ray_x_increment = fit.ray_scale / focal_length;
  double source_ray_x_increment = rotation[0][0] * ray_x_increment;
  double source_ray_y_increment = rotation[0][1] * ray_x_increment;
  double source_ray_z_increment = rotation[0][2] * ray_x_increment;

  for (size_t destination_y = 0; destination_y < destination->height; destination_y++) {
    double ray_y = fit.ray_scale * ((double)destination_y - center_y) / focal_length;
    double source_ray_x = rotation[0][0] * ray_x_start + rotation[1][0] * ray_y + rotation[2][0];
    double source_ray_y = rotation[0][1] * ray_x_start + rotation[1][1] * ray_y + rotation[2][1];
    double source_ray_z = rotation[0][2] * ray_x_start + rotation[1][2] * ray_y + rotation[2][2];

    for (size_t destination_x = 0; destination_x < destination->width; destination_x++) {
      double projection_depth = source_ray_z;
      if (projection_depth <= PROJECTION_MIN_DEPTH) {
        if (!synthesize_terrain) {
          free(destination->pixels);
          destination->pixels = NULL;
          return -1;
        }
        projection_depth = projection_depth < 0.0
                           ? fmin(projection_depth, -PROJECTION_MIN_DEPTH)
                           : PROJECTION_MIN_DEPTH;
      }
      double inverse_depth = focal_length / projection_depth;
      double source_x = source_ray_x * inverse_depth + center_x;
      double source_y = source_ray_y * inverse_depth + center_y;

      source_x = fit.image_scale * source_x + fit.offset_x;
      source_y = fit.image_scale * source_y + fit.offset_y;
      if (synthesize_terrain) {
        source_x = mirror_coordinate(source_x, source->width);
        source_y = mirror_coordinate(source_y, source->height);
      } else {
        source_x = fmax(0.0, fmin((double)(source->width - 1), source_x));
        source_y = fmax(0.0, fmin((double)(source->height - 1), source_y));
      }

      size_t destination_offset = (destination_y * destination->width + destination_x) * JPEG_COMPONENTS;
      bilinear_sample_rgb(source, source_x, source_y,
                          &destination->pixels[destination_offset]);
      source_ray_x += source_ray_x_increment;
      source_ray_y += source_ray_y_increment;
      source_ray_z += source_ray_z_increment;
    }
  }
  return IMAGE_MOCK_TRANSFORM_APPLIED;
}

static double mirror_coordinate(double coordinate, size_t extent)
{
  double maximum = (double)(extent - 1);
  double period = 2.0 * maximum;
  double mirrored = fmod(coordinate, period);
  if (mirrored < 0.0) {
    mirrored += period;
  }
  return mirrored <= maximum ? mirrored : period - mirrored;
}

static int create_black_frame(const char *filename)
{
  FILE *input = fopen(filename, "rb");
  if (input == NULL) {
    return -1;
  }
  struct decoded_image black = {NULL, 0, 0};
  int result = read_jpeg_dimensions(input, &black);
  if (fclose(input) != 0) {
    result = -1;
  }
  if (result != 0 || black.width > SIZE_MAX / JPEG_COMPONENTS
      || black.height > SIZE_MAX / (black.width * JPEG_COMPONENTS)) {
    return -1;
  }
  black.pixels = calloc(black.width * black.height, JPEG_COMPONENTS);
  if (black.pixels == NULL) {
    return -1;
  }
  result = replace_image_atomically(filename, &black);
  free(black.pixels);
  return result;
}

static void build_rotation(double roll_rad, double pitch_rad, double yaw_rad,
                           double rotation[3][3])
{
  double cosine_roll = cos(roll_rad);
  double sine_roll = sin(roll_rad);
  double cosine_pitch = cos(pitch_rad);
  double sine_pitch = sin(pitch_rad);
  double cosine_yaw = cos(yaw_rad);
  double sine_yaw = sin(yaw_rad);

  rotation[0][0] = cosine_yaw * cosine_pitch;
  rotation[0][1] = cosine_yaw * sine_pitch * sine_roll - sine_yaw * cosine_roll;
  rotation[0][2] = cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll;
  rotation[1][0] = sine_yaw * cosine_pitch;
  rotation[1][1] = sine_yaw * sine_pitch * sine_roll + cosine_yaw * cosine_roll;
  rotation[1][2] = sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll;
  rotation[2][0] = -sine_pitch;
  rotation[2][1] = cosine_pitch * sine_roll;
  rotation[2][2] = cosine_pitch * cosine_roll;
}

static bool project_point(double rotation[3][3], double focal_length,
                          double center_x, double center_y, double ray_scale,
                          double destination_x, double destination_y,
                          double *source_x, double *source_y)
{
  double ray_x = ray_scale * (destination_x - center_x) / focal_length;
  double ray_y = ray_scale * (destination_y - center_y) / focal_length;
  double source_ray_x = rotation[0][0] * ray_x + rotation[1][0] * ray_y + rotation[2][0];
  double source_ray_y = rotation[0][1] * ray_x + rotation[1][1] * ray_y + rotation[2][1];
  double source_ray_z = rotation[0][2] * ray_x + rotation[1][2] * ray_y + rotation[2][2];
  if (source_ray_z <= PROJECTION_MIN_DEPTH) {
    return false;
  }

  *source_x = focal_length * source_ray_x / source_ray_z + center_x;
  *source_y = focal_length * source_ray_y / source_ray_z + center_y;
  return isfinite(*source_x) && isfinite(*source_y);
}

static bool projection_corners_valid(const struct decoded_image *source,
                                     double rotation[3][3], double focal_length,
                                     double center_x, double center_y, double ray_scale)
{
  const double corners[4][2] = {
    {0.0, 0.0},
    {(double)(source->width - 1), 0.0},
    {(double)(source->width - 1), (double)(source->height - 1)},
    {0.0, (double)(source->height - 1)}
  };
  for (size_t index = 0; index < 4; index++) {
    double source_x;
    double source_y;
    if (!project_point(rotation, focal_length, center_x, center_y, ray_scale,
                       corners[index][0], corners[index][1], &source_x, &source_y)) {
      return false;
    }
  }
  return true;
}

static int compute_projection_fit(const struct decoded_image *source,
                                  double rotation[3][3], double focal_length,
                                  double center_x, double center_y,
                                  struct projection_fit *fit)
{
  if (rotation[2][2] <= PROJECTION_MIN_DEPTH) {
    errno = ERANGE;
    return -1;
  }

  double minimum_log_scale = log(PROJECTION_MIN_RAY_SCALE);
  double best_log_scale = minimum_log_scale;
  double best_zoom = INFINITY;
  bool fit_found = false;
  for (int sample = 0; sample <= PROJECTION_FOV_SAMPLES; sample++) {
    double fraction = (double)sample / PROJECTION_FOV_SAMPLES;
    double log_scale = minimum_log_scale * (1.0 - fraction);
    double ray_scale = exp(log_scale);
    struct projection_fit candidate;
    if (compute_projection_fit_at_scale(source, rotation, focal_length,
                                        center_x, center_y, ray_scale, &candidate) != 0) {
      continue;
    }
    double zoom = 1.0 / (candidate.ray_scale * candidate.image_scale);
    if (zoom < best_zoom) {
      best_zoom = zoom;
      best_log_scale = log_scale;
      *fit = candidate;
      fit_found = true;
    }
  }
  if (!fit_found) {
    errno = ERANGE;
    return -1;
  }

  double coarse_step = -minimum_log_scale / PROJECTION_FOV_SAMPLES;
  double lower_log_scale = fmax(minimum_log_scale, best_log_scale - coarse_step);
  double upper_log_scale = fmin(0.0, best_log_scale + coarse_step);
  for (int step = 0; step < PROJECTION_REFINE_STEPS; step++) {
    double first_log_scale = (2.0 * lower_log_scale + upper_log_scale) / 3.0;
    double second_log_scale = (lower_log_scale + 2.0 * upper_log_scale) / 3.0;
    struct projection_fit first_fit;
    struct projection_fit second_fit;
    bool first_valid = compute_projection_fit_at_scale(source, rotation, focal_length,
                                                       center_x, center_y, exp(first_log_scale),
                                                       &first_fit) == 0;
    bool second_valid = compute_projection_fit_at_scale(source, rotation, focal_length,
                                                        center_x, center_y, exp(second_log_scale),
                                                        &second_fit) == 0;
    double first_zoom = first_valid ? 1.0 / (first_fit.ray_scale * first_fit.image_scale) : INFINITY;
    double second_zoom = second_valid ? 1.0 / (second_fit.ray_scale * second_fit.image_scale) : INFINITY;
    if (first_zoom < best_zoom) {
      best_zoom = first_zoom;
      *fit = first_fit;
    }
    if (second_zoom < best_zoom) {
      best_zoom = second_zoom;
      *fit = second_fit;
    }
    if (first_zoom <= second_zoom) {
      upper_log_scale = second_log_scale;
    } else {
      lower_log_scale = first_log_scale;
    }
  }
  return 0;
}

static int compute_projection_fit_at_scale(const struct decoded_image *source,
                                           double rotation[3][3], double focal_length,
                                           double center_x, double center_y, double ray_scale,
                                           struct projection_fit *fit)
{
  if (!projection_corners_valid(source, rotation, focal_length, center_x, center_y,
                                ray_scale)) {
    return -1;
  }
  fit->ray_scale = ray_scale;

  const double corners[4][2] = {
    {0.0, 0.0},
    {(double)(source->width - 1), 0.0},
    {(double)(source->width - 1), (double)(source->height - 1)},
    {0.0, (double)(source->height - 1)}
  };
  double minimum_x = INFINITY;
  double minimum_y = INFINITY;
  double maximum_x = -INFINITY;
  double maximum_y = -INFINITY;
  for (size_t index = 0; index < 4; index++) {
    double source_x;
    double source_y;
    if (!project_point(rotation, focal_length, center_x, center_y, fit->ray_scale,
                       corners[index][0], corners[index][1], &source_x, &source_y)) {
      errno = ERANGE;
      return -1;
    }
    minimum_x = fmin(minimum_x, source_x);
    minimum_y = fmin(minimum_y, source_y);
    maximum_x = fmax(maximum_x, source_x);
    maximum_y = fmax(maximum_y, source_y);
  }

  double span_x = maximum_x - minimum_x;
  double span_y = maximum_y - minimum_y;
  if (span_x <= PROJECTION_EPSILON || span_y <= PROJECTION_EPSILON) {
    errno = ERANGE;
    return -1;
  }

  double horizontal_fit = (double)(source->width - 1) / span_x;
  double vertical_fit = (double)(source->height - 1) / span_y;
  bool already_covered = minimum_x >= 0.0 && minimum_y >= 0.0
                         && maximum_x <= (double)(source->width - 1)
                         && maximum_y <= (double)(source->height - 1);
  if (already_covered && fit->ray_scale == 1.0) {
    fit->image_scale = 1.0;
    fit->offset_x = 0.0;
    fit->offset_y = 0.0;
    return 0;
  }

  fit->image_scale = fmin(1.0, fmin(horizontal_fit, vertical_fit)) * COVER_SAFETY_FACTOR;
  fit->offset_x = center_x - fit->image_scale * (minimum_x + maximum_x) / 2.0;
  fit->offset_y = center_y - fit->image_scale * (minimum_y + maximum_y) / 2.0;
  return 0;
}

static void bilinear_sample_rgb(const struct decoded_image *image, double source_x,
                                double source_y, unsigned char output[JPEG_COMPONENTS])
{
  size_t left = (size_t)floor(source_x);
  size_t top = (size_t)floor(source_y);
  size_t right = left + 1 < image->width ? left + 1 : left;
  size_t bottom = top + 1 < image->height ? top + 1 : top;
  double horizontal = source_x - (double)left;
  double vertical = source_y - (double)top;

  size_t top_left = (top * image->width + left) * JPEG_COMPONENTS;
  size_t top_right = (top * image->width + right) * JPEG_COMPONENTS;
  size_t bottom_left = (bottom * image->width + left) * JPEG_COMPONENTS;
  size_t bottom_right = (bottom * image->width + right) * JPEG_COMPONENTS;
  for (size_t component = 0; component < JPEG_COMPONENTS; component++) {
    double top_value = image->pixels[top_left + component] * (1.0 - horizontal)
                       + image->pixels[top_right + component] * horizontal;
    double bottom_value = image->pixels[bottom_left + component] * (1.0 - horizontal)
                          + image->pixels[bottom_right + component] * horizontal;
    double value = top_value * (1.0 - vertical) + bottom_value * vertical;
    output[component] = (unsigned char)(value + 0.5);
  }
}

static int replace_image_atomically(const char *filename, const struct decoded_image *image)
{
  struct stat original_status;
  if (stat(filename, &original_status) != 0) {
    return -1;
  }

  size_t filename_length = strlen(filename);
  const char suffix[] = ".transform.XXXXXX";
  if (filename_length > SIZE_MAX - sizeof(suffix)) {
    errno = EOVERFLOW;
    return -1;
  }
  char *temporary_name = malloc(filename_length + sizeof(suffix));
  if (temporary_name == NULL) {
    return -1;
  }
  int name_length = snprintf(temporary_name, filename_length + sizeof(suffix),
                             "%s%s", filename, suffix);
  if (name_length < 0 || (size_t)name_length >= filename_length + sizeof(suffix)) {
    free(temporary_name);
    errno = EOVERFLOW;
    return -1;
  }

  int temporary_fd = mkstemp(temporary_name);
  if (temporary_fd < 0) {
    free(temporary_name);
    return -1;
  }
  FILE *output = fdopen(temporary_fd, "wb");
  if (output == NULL) {
    close(temporary_fd);
    unlink(temporary_name);
    free(temporary_name);
    return -1;
  }

  int result = encode_jpeg(output, image);
  if (result == 0 && fflush(output) != 0) {
    result = -1;
  }
  if (result == 0 && fsync(temporary_fd) != 0) {
    result = -1;
  }
  if (result == 0 && fchmod(temporary_fd, original_status.st_mode & 0777) != 0) {
    result = -1;
  }
  if (fclose(output) != 0) {
    result = -1;
  }
  if (result == 0 && rename(temporary_name, filename) != 0) {
    result = -1;
  }
  if (result != 0) {
    unlink(temporary_name);
    fprintf(stderr, "MOCK_TRANSFORM:\tfailed to replace %s: %s\n", filename, strerror(errno));
  }
  free(temporary_name);
  return result;
}