#include "modules/computer_vision/gate_cnn_detector.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include "gate_cnn_weights.h"

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#ifndef GATE_CNN_DETECTOR_CAMERA
#error "Define GATE_CNN_DETECTOR_CAMERA in the airframe/module settings"
#endif

#ifndef GATE_CNN_DETECTOR_FPS
#define GATE_CNN_DETECTOR_FPS 0
#endif

#ifndef GATE_CNN_VISUAL_DETECTION_ID
#define GATE_CNN_VISUAL_DETECTION_ID 1
#endif

#define C1 8
#define C2 16
#define C3 24

#ifndef GATE_CNN_DEBUG
#define GATE_CNN_DEBUG true
#endif

#ifndef GATE_CNN_DEBUG_EVERY_N_FRAMES
#define GATE_CNN_DEBUG_EVERY_N_FRAMES 5
#endif


/* --------------------------------------------------------- */
/* Shared detector result                                    */
/* --------------------------------------------------------- */

static pthread_mutex_t gate_cnn_mutex;
static gate_prediction_t g_pred;
static struct video_listener *gate_cnn_listener;

static uint32_t g_frame_counter = 0u;
static float g_last_inference_ms = 0.0f;

#if GATE_CNN_DEBUG
#define GATE_CNN_PRINT(...) printf(__VA_ARGS__)
#else
#define GATE_CNN_PRINT(...) do { } while (0)
#endif

static inline double gate_cnn_now_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

/* --------------------------------------------------------- */
/* Scratch buffers: fixed-size, no malloc                    */
/* --------------------------------------------------------- */

static float g_in[3 * GATE_CNN_INPUT_H * GATE_CNN_INPUT_W];

static float g_c1[C1 * GATE_CNN_INPUT_H * GATE_CNN_INPUT_W];
static float g_p1[C1 * 6 * 24];

static float g_c2[C2 * 6 * 24];
static float g_p2[C2 * 3 * 12];

static float g_c3[C3 * 3 * 12];
static float g_ap[C3 * 3 * 2];

static float g_fc1[64];
static float g_fc2[32];
static float g_out[5];

/* --------------------------------------------------------- */
/* Cached resize map                                         */
/* --------------------------------------------------------- */

static int map_w = -1;
static int map_h = -1;

static int x0_map[GATE_CNN_INPUT_W];
static int x1_map[GATE_CNN_INPUT_W];
static float wx_map[GATE_CNN_INPUT_W];

static int y0_map[GATE_CNN_INPUT_H];
static int y1_map[GATE_CNN_INPUT_H];
static float wy_map[GATE_CNN_INPUT_H];

/* --------------------------------------------------------- */
/* Small helpers                                             */
/* --------------------------------------------------------- */

static inline float fast_sigmoid(float x)
{
  return 1.0f / (1.0f + expf(-x));
}

static inline float clampf_local(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline void relu_inplace(float *x, int n)
{
  for (int i = 0; i < n; i++) {
    if (x[i] < 0.0f) {
      x[i] = 0.0f;
    }
  }
}

/* --------------------------------------------------------- */
/* UYVY pixel access from Paparazzi image buffer             */
/* --------------------------------------------------------- */

static inline void uyvy_get_pixel(const uint8_t *frame, int width, int x, int y,
                                  float *Y, float *U, float *V)
{
  const uint8_t *row = frame + (size_t)y * (size_t)width * 2u;
  int pair = x >> 1;
  int idx = pair * 4;

  uint8_t u = row[idx + 0];
  uint8_t v = row[idx + 2];
  uint8_t yy = (x & 1) ? row[idx + 3] : row[idx + 1];

  *Y = (float)yy * (1.0f / 255.0f);
  *U = (float)u  * (1.0f / 255.0f);
  *V = (float)v  * (1.0f / 255.0f);
}

/* --------------------------------------------------------- */
/* Resize map cache                                          */
/* --------------------------------------------------------- */

static void build_resize_map_if_needed(int width, int height)
{
  if (width == map_w && height == map_h) {
    return;
  }

  map_w = width;
  map_h = height;

  for (int ox = 0; ox < GATE_CNN_INPUT_W; ox++) {
    float gx = ((float)ox * (float)(width - 1)) / (float)(GATE_CNN_INPUT_W - 1);
    int x0 = (int)gx;
    int x1 = x0 + 1;
    if (x1 >= width) {
      x1 = width - 1;
    }
    x0_map[ox] = x0;
    x1_map[ox] = x1;
    wx_map[ox] = gx - (float)x0;
  }

  for (int oy = 0; oy < GATE_CNN_INPUT_H; oy++) {
    float gy = ((float)oy * (float)(height - 1)) / (float)(GATE_CNN_INPUT_H - 1);
    int y0 = (int)gy;
    int y1 = y0 + 1;
    if (y1 >= height) {
      y1 = height - 1;
    }
    y0_map[oy] = y0;
    y1_map[oy] = y1;
    wy_map[oy] = gy - (float)y0;
  }
}

/* --------------------------------------------------------- */
/* Direct UYVY -> resized CHW input                          */
/* --------------------------------------------------------- */

static void uyvy_to_resized_input(const uint8_t *frame, int width, int height, float *out_chw)
{
  build_resize_map_if_needed(width, height);

  for (int oy = 0; oy < GATE_CNN_INPUT_H; oy++) {
    int y0 = y0_map[oy];
    int y1 = y1_map[oy];
    float wy = wy_map[oy];
    float wy0 = 1.0f - wy;

    for (int ox = 0; ox < GATE_CNN_INPUT_W; ox++) {
      int x0 = x0_map[ox];
      int x1 = x1_map[ox];
      float wx = wx_map[ox];
      float wx0 = 1.0f - wx;

      float Y00, U00, V00;
      float Y01, U01, V01;
      float Y10, U10, V10;
      float Y11, U11, V11;

      uyvy_get_pixel(frame, width, x0, y0, &Y00, &U00, &V00);
      uyvy_get_pixel(frame, width, x1, y0, &Y01, &U01, &V01);
      uyvy_get_pixel(frame, width, x0, y1, &Y10, &U10, &V10);
      uyvy_get_pixel(frame, width, x1, y1, &Y11, &U11, &V11);

      float topY = Y00 * wx0 + Y01 * wx;
      float botY = Y10 * wx0 + Y11 * wx;
      float topU = U00 * wx0 + U01 * wx;
      float botU = U10 * wx0 + U11 * wx;
      float topV = V00 * wx0 + V01 * wx;
      float botV = V10 * wx0 + V11 * wx;

      int p = oy * GATE_CNN_INPUT_W + ox;
      out_chw[0 * (GATE_CNN_INPUT_H * GATE_CNN_INPUT_W) + p] = topY * wy0 + botY * wy;
      out_chw[1 * (GATE_CNN_INPUT_H * GATE_CNN_INPUT_W) + p] = topU * wy0 + botU * wy;
      out_chw[2 * (GATE_CNN_INPUT_H * GATE_CNN_INPUT_W) + p] = topV * wy0 + botV * wy;
    }
  }
}

/* --------------------------------------------------------- */
/* Conv 3x3 same                                             */
/* --------------------------------------------------------- */

static void conv3x3_same(const float *x, int cin, int h, int w,
                         const float *weight, const float *bias, int cout,
                         float *out)
{
  for (int co = 0; co < cout; co++) {
    for (int oy = 0; oy < h; oy++) {
      for (int ox = 0; ox < w; ox++) {
        float acc = bias[co];

        for (int ci = 0; ci < cin; ci++) {
          const float *xci = x + ci * h * w;
          const float *wco = weight + co * cin * 9 + ci * 9;

          for (int ky = 0; ky < 3; ky++) {
            int iy = oy + ky - 1;
            if ((unsigned)iy >= (unsigned)h) {
              continue;
            }

            int base = iy * w;
            for (int kx = 0; kx < 3; kx++) {
              int ix = ox + kx - 1;
              if ((unsigned)ix >= (unsigned)w) {
                continue;
              }
              acc += xci[base + ix] * wco[ky * 3 + kx];
            }
          }
        }

        out[co * h * w + oy * w + ox] = acc;
      }
    }
  }
}

/* --------------------------------------------------------- */
/* MaxPool 2x2 stride 2                                      */
/* --------------------------------------------------------- */

static void maxpool2x2_s2(const float *x, int c, int h, int w, float *out)
{
  int oh = h >> 1;
  int ow = w >> 1;

  for (int ch = 0; ch < c; ch++) {
    const float *xc = x + ch * h * w;
    float *oc = out + ch * oh * ow;

    for (int oy = 0; oy < oh; oy++) {
      int iy = oy << 1;
      for (int ox = 0; ox < ow; ox++) {
        int ix = ox << 1;

        float a = xc[iy * w + ix];
        float b = xc[iy * w + ix + 1];
        float c0 = xc[(iy + 1) * w + ix];
        float d = xc[(iy + 1) * w + ix + 1];

        float m = a;
        if (b > m) m = b;
        if (c0 > m) m = c0;
        if (d > m) m = d;

        oc[oy * ow + ox] = m;
      }
    }
  }
}

/* --------------------------------------------------------- */
/* Fixed pool: (24,3,12) -> (24,3,2)                         */
/* --------------------------------------------------------- */

static void avgpool_width_12_to_2(const float *x, float *out)
{
  for (int ch = 0; ch < C3; ch++) {
    const float *xc = x + ch * 3 * 12;
    float *oc = out + ch * 3 * 2;

    for (int row = 0; row < 3; row++) {
      const float *r = xc + row * 12;

      float s0 = r[0] + r[1] + r[2] + r[3] + r[4] + r[5];
      float s1 = r[6] + r[7] + r[8] + r[9] + r[10] + r[11];

      oc[row * 2 + 0] = s0 * (1.0f / 6.0f);
      oc[row * 2 + 1] = s1 * (1.0f / 6.0f);
    }
  }
}

/* --------------------------------------------------------- */
/* Linear                                                    */
/* --------------------------------------------------------- */

static void linear_layer(const float *x, int in_dim,
                         const float *weight, const float *bias,
                         int out_dim, float *out)
{
  for (int o = 0; o < out_dim; o++) {
    const float *wo = weight + o * in_dim;
    float acc = bias[o];
    for (int i = 0; i < in_dim; i++) {
      acc += wo[i] * x[i];
    }
    out[o] = acc;
  }
}

/* --------------------------------------------------------- */
/* Core CNN inference                                        */
/* --------------------------------------------------------- */

static int gate_cnn_predict_uyvy_core(const uint8_t *frame, int width, int height, gate_prediction_t *out)
{
  if (frame == NULL || out == NULL) {
    return -1;
  }
  if (width <= 1 || height <= 1) {
    return -2;
  }
  if (width & 1) {
    return -3;
  }

  uyvy_to_resized_input(frame, width, height, g_in);

  conv3x3_same(g_in, 3, 12, 48,
               &features_0_weight[0][0][0][0], &features_0_bias[0], C1, g_c1);
  relu_inplace(g_c1, C1 * 12 * 48);
  maxpool2x2_s2(g_c1, C1, 12, 48, g_p1);

  conv3x3_same(g_p1, C1, 6, 24,
               &features_3_weight[0][0][0][0], &features_3_bias[0], C2, g_c2);
  relu_inplace(g_c2, C2 * 6 * 24);
  maxpool2x2_s2(g_c2, C2, 6, 24, g_p2);

  conv3x3_same(g_p2, C2, 3, 12,
               &features_6_weight[0][0][0][0], &features_6_bias[0], C3, g_c3);
  relu_inplace(g_c3, C3 * 3 * 12);

  avgpool_width_12_to_2(g_c3, g_ap);

  linear_layer(g_ap, 144, &head_1_weight[0][0], &head_1_bias[0], 64, g_fc1);
  relu_inplace(g_fc1, 64);

  linear_layer(g_fc1, 64, &head_3_weight[0][0], &head_3_bias[0], 32, g_fc2);
  relu_inplace(g_fc2, 32);

  linear_layer(g_fc2, 32, &head_5_weight[0][0], &head_5_bias[0], 5, g_out);

  float present_prob = fast_sigmoid(g_out[0]);
  float cx_n = fast_sigmoid(g_out[1]);
  float cy_n = fast_sigmoid(g_out[2]);
  float bw_n = fast_sigmoid(g_out[3]);
  float bh_n = fast_sigmoid(g_out[4]);

  float cx = cx_n * (float)width;
  float cy = cy_n * (float)height;
  float bw = bw_n * (float)width;
  float bh = bh_n * (float)height;

  out->present_prob = present_prob;
  out->present = (present_prob >= GATE_CNN_THRESHOLD) ? 1u : 0u;
  out->center_x = cx;
  out->center_y = cy;
  out->bbox_width = bw;
  out->bbox_height = bh;
  out->bbox_xyxy[0] = clampf_local(cx - 0.5f * bw, 0.0f, (float)(width - 1));
  out->bbox_xyxy[1] = clampf_local(cy - 0.5f * bh, 0.0f, (float)(height - 1));
  out->bbox_xyxy[2] = clampf_local(cx + 0.5f * bw, 0.0f, (float)(width - 1));
  out->bbox_xyxy[3] = clampf_local(cy + 0.5f * bh, 0.0f, (float)(height - 1));
  out->updated = 1u;

  return 0;
}

static void gate_cnn_debug_print_prediction(const gate_prediction_t *pred, int width, int height, uint32_t frame_idx)
{
#if GATE_CNN_DEBUG
  if (pred == NULL) {
    return;
  }
  if ((frame_idx % GATE_CNN_DEBUG_EVERY_N_FRAMES) != 0u) {
    return;
  }

  GATE_CNN_PRINT("[gate_cnn_detector.c][cnn] frame=%lu time=%.3f ms prob=%.3f detected=%u center=(%.1f,%.1f) size=(%.1f,%.1f) bbox=(%.1f,%.1f,%.1f,%.1f) img=%dx%d\n",
                 (unsigned long)frame_idx,
                 (double)g_last_inference_ms,
                 (double)pred->present_prob,
                 pred->present,
                 (double)pred->center_x,
                 (double)pred->center_y,
                 (double)pred->bbox_width,
                 (double)pred->bbox_height,
                 (double)pred->bbox_xyxy[0],
                 (double)pred->bbox_xyxy[1],
                 (double)pred->bbox_xyxy[2],
                 (double)pred->bbox_xyxy[3],
                 width,
                 height);

  if (pred->present) {
    GATE_CNN_PRINT("[gate_cnn_detector.c][detection] frame=%lu DETECTED gate -> will send VISUAL_DETECTION with quality=%ld\n",
                   (unsigned long)frame_idx,
                   (long)(1000.0f * pred->present_prob));
  }
#endif
}

/* --------------------------------------------------------- */
/* Paparazzi video callback                                  */
/* --------------------------------------------------------- */

static struct image_t *gate_cnn_detector_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  gate_prediction_t local_pred;
  const uint32_t npix = (uint32_t)img->w * (uint32_t)img->h;

  if (img->type != IMAGE_YUV422) {
    return img;
  }

  if (npix == 0U) {
    return img;
  }

  memset(&local_pred, 0, sizeof(local_pred));

  double t0_ms = gate_cnn_now_ms();
  int status = gate_cnn_predict_uyvy_core((const uint8_t *)img->buf, img->w, img->h, &local_pred);
  double t1_ms = gate_cnn_now_ms();
  g_last_inference_ms = (float)(t1_ms - t0_ms);
  g_frame_counter++;

  if (status == 0) {
    gate_cnn_debug_print_prediction(&local_pred, img->w, img->h, g_frame_counter);

    pthread_mutex_lock(&gate_cnn_mutex);
    memcpy(&g_pred, &local_pred, sizeof(local_pred));
    pthread_mutex_unlock(&gate_cnn_mutex);
  } else {
    GATE_CNN_PRINT("[gate_cnn_detector.c][cnn] frame=%lu prediction_failed status=%d time=%.3f ms\n",
                   (unsigned long)g_frame_counter,
                   status,
                   (double)g_last_inference_ms);
  }

  return img;
}

/* --------------------------------------------------------- */
/* Public module API                                         */
/* --------------------------------------------------------- */

void gate_cnn_detector_init(void)
{
  memset(&g_pred, 0, sizeof(g_pred));
  pthread_mutex_init(&gate_cnn_mutex, NULL);

  map_w = -1;
  map_h = -1;
  g_frame_counter = 0u;
  g_last_inference_ms = 0.0f;

  gate_cnn_listener = cv_add_to_device(&GATE_CNN_DETECTOR_CAMERA,
                                       gate_cnn_detector_func,
                                       GATE_CNN_DETECTOR_FPS,
                                       0);
  (void)gate_cnn_listener;
}

void gate_cnn_detector_periodic(void)
{
  gate_prediction_t local_pred;

  pthread_mutex_lock(&gate_cnn_mutex);
  memcpy(&local_pred, &g_pred, sizeof(local_pred));
  g_pred.updated = 0u;
  pthread_mutex_unlock(&gate_cnn_mutex);

  if (local_pred.updated && local_pred.present) {
    int16_t msg_px = (int16_t)local_pred.center_x;
    int16_t msg_py = (int16_t)local_pred.center_y;
    int16_t msg_w = (int16_t)local_pred.bbox_width;
    int16_t msg_h = (int16_t)local_pred.bbox_height;
    int32_t msg_q = (int32_t)(1000.0f * local_pred.present_prob);

    GATE_CNN_PRINT("[gate_cnn_detector.c][tx VISUAL_DETECTION] px=%d py=%d w=%d h=%d quality=%ld time=%.3f ms\n",
                   msg_px,
                   msg_py,
                   msg_w,
                   msg_h,
                   (long)msg_q,
                   (double)g_last_inference_ms);

    AbiSendMsgVISUAL_DETECTION(
      GATE_CNN_VISUAL_DETECTION_ID,
      msg_px,
      msg_py,
      msg_w,
      msg_h,
      msg_q,
      0
    );
  }
}
