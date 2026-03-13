/*
 * custom_detector.cpp
 *
 * Paparazzi/OpenCV module
 *
 * Detects obstacles from color masks and publishes:
 *   CUSTOM_DETECTION(uint8_t left, uint8_t middle, uint8_t right)
 *
 * Message rule:
 *   obstacle = 1 if region risk_score >= 2
 *   obstacle = 0 otherwise
 *
 * Structure:
 *   - image callback processes frame and stores latest result
 *   - periodic function publishes stored result through ABI
 */

extern "C" {
#include "modules/computer_vision/custom_detector.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
}

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <vector>
#include <string>
#include <algorithm>

using namespace cv;
using namespace std;

#define PRINT(string, ...) fprintf(stderr, "[custom_detector->%s()] " string, __FUNCTION__, ##__VA_ARGS__)

#ifndef CUSTOM_DETECT_COLOR_OBJECT_VERBOSE
#define CUSTOM_DETECT_COLOR_OBJECT_VERBOSE TRUE
#endif

#if CUSTOM_DETECT_COLOR_OBJECT_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

#ifndef CUSTOM_DETECT_COLOR_OBJECT_ID
#define CUSTOM_DETECT_COLOR_OBJECT_ID ABI_BROADCAST
#endif

#ifndef CUSTOM_DETECT_COLOR_OBJECT_FPS
#define CUSTOM_DETECT_COLOR_OBJECT_FPS 0
#endif

#ifndef CUSTOM_DETECT_COLOR_OBJECT_ROTATION
#define CUSTOM_DETECT_COLOR_OBJECT_ROTATION cv::ROTATE_90_COUNTERCLOCKWISE
#endif

static pthread_mutex_t detector_mutex;

// HSV thresholds for orange
static const Scalar ORANGE_LOWER(5, 80, 120);
static const Scalar ORANGE_UPPER(25, 255, 255);

// Orange geometry filter
static const float ORANGE_MIN_ASPECT_RATIO = 1.0f;   // height / width
static const int ORANGE_MIN_HEIGHT = 50;

// HSV thresholds for green
static const Scalar GREEN_LOWER(25, 40, 40);
static const Scalar GREEN_UPPER(90, 255, 255);

// Minimum contour area
static const int MIN_AREA_ORANGE = 300;
static const int MIN_AREA_GREEN = 120;

// Occupancy thresholds
static const float ORANGE_OCC_THRESHOLD = 0.20f;
static const float GREEN_OCC_THRESHOLD = 0.10f;
static const float WINDOW_LOW_THRESHOLD = 0.10f;

// ----------------------------------------------------------------------------
// Global published result
// ----------------------------------------------------------------------------

struct custom_detection_result_t {
  uint8_t left;
  uint8_t middle;
  uint8_t right;
  bool updated;
};

static struct custom_detection_result_t global_result;

// ----------------------------------------------------------------------------
// Helper structs
// ----------------------------------------------------------------------------

struct WindowCandidate {
  int x;
  int y;
  int w;
  int h;
  double area;
};

struct RegionMetrics {
  float orange_occ;
  float green_occ;
  float window_occ;

  bool orange_blocked;
  bool green_blocked;
  bool window_blocked;
  bool blocked;

  int risk_score;
  Rect bbox;
};

// ----------------------------------------------------------------------------
// Utility functions
// ----------------------------------------------------------------------------

static Mat preprocess_mask(const Mat &mask_in, int kernel_size)
{
  Mat mask = mask_in.clone();
  Mat kernel = Mat::ones(kernel_size, kernel_size, CV_8U);

  morphologyEx(mask, mask, MORPH_OPEN, kernel);
  morphologyEx(mask, mask, MORPH_CLOSE, kernel);

  return mask;
}

// These functions expect HSV input
static Mat build_orange_mask_from_hsv(const Mat &image_hsv)
{
  Mat mask;
  inRange(image_hsv, ORANGE_LOWER, ORANGE_UPPER, mask);
  return preprocess_mask(mask, 5);
}

static Mat build_green_mask_from_hsv(const Mat &image_hsv)
{
  Mat mask;
  inRange(image_hsv, GREEN_LOWER, GREEN_UPPER, mask);
  return preprocess_mask(mask, 3);
}

// ----------------------------------------------------------------------------
// Geometry-validated orange mask
// ----------------------------------------------------------------------------

static Mat build_valid_orange_mask(const Mat &orange_mask)
{
  Mat valid_mask = Mat::zeros(orange_mask.size(), CV_8UC1);

  vector<vector<Point> > contours;
  findContours(orange_mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area < MIN_AREA_ORANGE) {
      continue;
    }

    Rect r = boundingRect(contours[i]);

    if (r.height < ORANGE_MIN_HEIGHT) {
      continue;
    }

    float aspect_ratio = (float)r.height / (float)max(r.width, 1);
    if (aspect_ratio < ORANGE_MIN_ASPECT_RATIO) {
      continue;
    }

    drawContours(valid_mask, contours, (int)i, Scalar(255), FILLED);
  }

  return valid_mask;
}

// ----------------------------------------------------------------------------
// Split green into floor and obstacles
// ----------------------------------------------------------------------------

static void split_green_floor_and_obstacles(const Mat &green_mask,
                                            Mat &floor_mask,
                                            Mat &obstacle_mask)
{
  int h = green_mask.rows;
  int w = green_mask.cols;

  Mat labels, stats, centroids;
  int num_labels = connectedComponentsWithStats(green_mask, labels, stats, centroids, 8);

  floor_mask = Mat::zeros(green_mask.size(), CV_8UC1);
  obstacle_mask = Mat::zeros(green_mask.size(), CV_8UC1);

  for (int label = 1; label < num_labels; label++) {
    int y    = stats.at<int>(label, CC_STAT_TOP);
    int ww   = stats.at<int>(label, CC_STAT_WIDTH);
    int hh   = stats.at<int>(label, CC_STAT_HEIGHT);
    int area = stats.at<int>(label, CC_STAT_AREA);

    if (area < 50) {
      continue;
    }

    Mat component = (labels == label);
    component.convertTo(component, CV_8UC1, 255);

    bool touches_bottom = false;
    for (int col = 0; col < w; col++) {
      if (component.at<uint8_t>(h - 1, col) > 0) {
        touches_bottom = true;
        break;
      }
    }

    bool wide_and_low = ((y + hh) > (int)(0.75f * h)) && (ww > (int)(0.25f * w));

    if (touches_bottom || wide_and_low) {
      bitwise_or(floor_mask, component, floor_mask);
    } else {
      bitwise_or(obstacle_mask, component, obstacle_mask);
    }
  }
}

// ----------------------------------------------------------------------------
// Geometry-validated green obstacle mask
// ----------------------------------------------------------------------------

static Mat build_valid_green_obstacle_mask(const Mat &green_obstacle_mask)
{
  Mat valid_mask = Mat::zeros(green_obstacle_mask.size(), CV_8UC1);

  vector<vector<Point> > contours;
  findContours(green_obstacle_mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area < MIN_AREA_GREEN) {
      continue;
    }

    drawContours(valid_mask, contours, (int)i, Scalar(255), FILLED);
  }

  return valid_mask;
}

// ----------------------------------------------------------------------------
// Window candidates from edges
// ----------------------------------------------------------------------------

static vector<WindowCandidate> get_window_candidates_from_edges(const Mat &image_bgr, Mat &edges_out)
{
  Mat gray, blur_img, edges;
  cvtColor(image_bgr, gray, COLOR_BGR2GRAY);
  GaussianBlur(gray, blur_img, Size(5, 5), 0);

  Canny(blur_img, edges, 50, 150);

  Mat kernel = Mat::ones(3, 3, CV_8U);
  dilate(edges, edges, kernel, Point(-1, -1), 1);
  morphologyEx(edges, edges, MORPH_CLOSE, kernel);

  vector<vector<Point> > contours;
  findContours(edges.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<WindowCandidate> candidates;
  int h_img = image_bgr.rows;

  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area < 200 || area > 3000) {
      continue;
    }

    double peri = arcLength(contours[i], true);
    vector<Point> approx;
    approxPolyDP(contours[i], approx, 0.02 * peri, true);

    if (approx.size() < 4 || approx.size() > 8) {
      continue;
    }

    Rect r = boundingRect(approx);

    if (r.width < 15 || r.height < 15) {
      continue;
    }

    float aspect = (float)r.width / (float)max(r.height, 1);
    if (!(aspect > 0.6f && aspect < 1.6f)) {
      continue;
    }

    if (r.y > (int)(0.5f * h_img)) {
      continue;
    }

    Mat roi = gray(r);
    if (roi.empty()) {
      continue;
    }

    Scalar mean_val = mean(roi);
    if (mean_val[0] > 170.0) {
      continue;
    }

    WindowCandidate c;
    c.x = r.x;
    c.y = r.y;
    c.w = r.width;
    c.h = r.height;
    c.area = area;
    candidates.push_back(c);
  }

  edges_out = edges.clone();
  return candidates;
}

// ----------------------------------------------------------------------------
// Door mask from best window candidate
// ----------------------------------------------------------------------------

static Mat build_door_mask_from_window(const Size &frame_size,
                                       const vector<WindowCandidate> &window_candidates)
{
  Mat door_mask = Mat::zeros(frame_size, CV_8UC1);

  if (window_candidates.empty()) {
    return door_mask;
  }

  WindowCandidate best = window_candidates[0];
  for (size_t i = 1; i < window_candidates.size(); i++) {
    if (window_candidates[i].area > best.area) {
      best = window_candidates[i];
    }
  }

  int h = frame_size.height;
  int w = frame_size.width;

  int door_x = max(0, best.x - (int)(0.5f * best.w));
  int door_y = max(0, best.y - (int)(0.1f * best.h));
  int door_w = min(w - door_x, (int)(1.5f * best.w));
  int door_h = min(h - door_y, (int)(3.0f * best.h));

  rectangle(door_mask,
            Point(door_x, door_y),
            Point(door_x + door_w, door_y + door_h),
            Scalar(255),
            FILLED);

  return door_mask;
}

// ----------------------------------------------------------------------------
// Drawing helpers
// ----------------------------------------------------------------------------

static void draw_bboxes(Mat &frame,
                        const Mat &mask,
                        const Scalar &color,
                        const string &label,
                        int min_area,
                        bool require_tall = false,
                        float min_aspect_ratio = 1.0f,
                        int min_height = 0)
{
  vector<vector<Point> > contours;
  findContours(mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area < min_area) {
      continue;
    }

    Rect r = boundingRect(contours[i]);

    if (require_tall) {
      if (r.height < min_height) {
        continue;
      }
      float aspect_ratio = (float)r.height / (float)max(r.width, 1);
      if (aspect_ratio < min_aspect_ratio) {
        continue;
      }
    }

    rectangle(frame, r, color, 2);
    putText(frame, label,
            Point(r.x, max(r.y - 8, 20)),
            FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
  }
}

static void draw_door_from_window(Mat &frame, const vector<WindowCandidate> &window_candidates)
{
  if (window_candidates.empty()) {
    return;
  }

  WindowCandidate best = window_candidates[0];
  for (size_t i = 1; i < window_candidates.size(); i++) {
    if (window_candidates[i].area > best.area) {
      best = window_candidates[i];
    }
  }

  int h_img = frame.rows;
  int w_img = frame.cols;

  rectangle(frame,
            Point(best.x, best.y),
            Point(best.x + best.w, best.y + best.h),
            Scalar(255, 0, 255), 2);

  putText(frame, "Window",
          Point(best.x, max(best.y - 8, 20)),
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 2);

  int door_x = max(0, best.x - (int)(0.5f * best.w));
  int door_y = max(0, best.y - (int)(0.1f * best.h));
  int door_w = min(w_img - door_x, (int)(1.5f * best.w));
  int door_h = min(h_img - door_y, (int)(3.0f * best.h));

  rectangle(frame,
            Point(door_x, door_y),
            Point(door_x + door_w, door_y + door_h),
            Scalar(255, 140, 0), 2);

  putText(frame, "Door",
          Point(door_x, max(door_y - 8, 20)),
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 140, 0), 2);
}

static void draw_navigation_columns(Mat &frame,
                                    const RegionMetrics &left_m,
                                    const RegionMetrics &center_m,
                                    const RegionMetrics &right_m,
                                    const string &decision)
{
  auto draw_one = [&](const string &name, const RegionMetrics &m) {
    Scalar color = m.blocked ? Scalar(0, 0, 255) : Scalar(0, 255, 0);
    string text = m.blocked ? (name + ": NO") : (name + ": FREE");

    rectangle(frame, m.bbox, color, 3);

    putText(frame, text,
            Point(m.bbox.x + 10, m.bbox.y + 35),
            FONT_HERSHEY_SIMPLEX, 0.8, color, 2);

    char risk_text[32];
    snprintf(risk_text, sizeof(risk_text), "risk=%d", m.risk_score);

    putText(frame, risk_text,
            Point(m.bbox.x + 10, m.bbox.y + 65),
            FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
  };

  draw_one("L", left_m);
  draw_one("C", center_m);
  draw_one("R", right_m);

  putText(frame,
          "Decision: " + decision,
          Point(20, 40),
          FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2);
}

// ----------------------------------------------------------------------------
// Risk computation
// ----------------------------------------------------------------------------

static void compute_column_metrics(const Mat &orange_mask,
                                   const Mat &door_mask,
                                   const Mat &green_obstacle_mask,
                                   RegionMetrics &left_m,
                                   RegionMetrics &center_m,
                                   RegionMetrics &right_m)
{
  int h = orange_mask.rows;
  int w = orange_mask.cols;

  int col_w = w / 3;
  int row_h = h / 4;

  int bottom_y1 = 3 * row_h;
  int bottom_y2 = h;

  Rect left_rect(0, bottom_y1, col_w, bottom_y2 - bottom_y1);
  Rect center_rect(col_w, bottom_y1, col_w, bottom_y2 - bottom_y1);
  Rect right_rect(2 * col_w, bottom_y1, w - 2 * col_w, bottom_y2 - bottom_y1);

  auto eval_region = [&](const Rect &r, RegionMetrics &m) {
    int total_pixels = max(1, r.width * r.height);

    float orange_occ = (float)countNonZero(orange_mask(r)) / (float)total_pixels;
    float green_occ  = (float)countNonZero(green_obstacle_mask(r)) / (float)total_pixels;
    float window_occ = (float)countNonZero(door_mask(r)) / (float)total_pixels;

    bool orange_blocked = orange_occ >= ORANGE_OCC_THRESHOLD;
    bool green_blocked  = green_occ  >= GREEN_OCC_THRESHOLD;
    bool window_blocked = window_occ <= WINDOW_LOW_THRESHOLD;

    bool blocked = orange_blocked || green_blocked;

    int risk_score = 0;
    if (orange_blocked) risk_score += 2;
    if (green_blocked)  risk_score += 2;
    if (window_blocked) risk_score += 1;

    m.orange_occ = orange_occ;
    m.green_occ = green_occ;
    m.window_occ = window_occ;
    m.orange_blocked = orange_blocked;
    m.green_blocked = green_blocked;
    m.window_blocked = window_blocked;
    m.blocked = blocked;
    m.risk_score = risk_score;
    m.bbox = r;
  };

  eval_region(left_rect, left_m);
  eval_region(center_rect, center_m);
  eval_region(right_rect, right_m);
}

static string decide_direction(const RegionMetrics &left_m,
                               const RegionMetrics &center_m,
                               const RegionMetrics &right_m)
{
  int left_risk = left_m.risk_score;
  int center_risk = center_m.risk_score;
  int right_risk = right_m.risk_score;

  VERBOSE_PRINT("Left Risk: %d\n", left_risk);
  VERBOSE_PRINT("Center Risk: %d\n", center_risk);
  VERBOSE_PRINT("Right Risk: %d\n", right_risk);

  if (left_risk == 0 && center_risk == 0 && right_risk == 0) {
    return "CENTER";
  } else if (center_risk < 2) {
    return "CENTER";
  } else if (!left_m.blocked && right_m.blocked) {
    return "LEFT";
  } else if (!right_m.blocked && left_m.blocked) {
    return "RIGHT";
  }

  return (left_risk <= right_risk) ? "LEFT" : "RIGHT";
}

static void risk_to_binary_obstacles(const RegionMetrics &left_m,
                                     const RegionMetrics &center_m,
                                     const RegionMetrics &right_m,
                                     uint8_t *left_obs,
                                     uint8_t *center_obs,
                                     uint8_t *right_obs)
{
  *left_obs   = (left_m.risk_score   >= 2) ? 1 : 0;
  *center_obs = (center_m.risk_score >= 2) ? 1 : 0;
  *right_obs  = (right_m.risk_score  >= 2) ? 1 : 0;
}

// ----------------------------------------------------------------------------
// Main image callback processing
// ----------------------------------------------------------------------------

static struct image_t *object_detector(struct image_t *img, uint8_t camera_id);
static struct image_t *object_detector(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  if (img == NULL || img->buf == NULL) {
    return img;
  }

  // Raw Paparazzi camera image is YUV422 packed
  Mat src_yuv(img->h, img->w, CV_8UC2, img->buf);

  // Convert YUV422 -> BGR
  // If colors look wrong, try COLOR_YUV2BGR_YUY2 instead
  Mat frame_bgr;
  cvtColor(src_yuv, frame_bgr, COLOR_YUV2BGR_UYVY);

  // Rotate
  rotate(frame_bgr, frame_bgr, CUSTOM_DETECT_COLOR_OBJECT_ROTATION);

  // Convert BGR -> HSV once
  Mat frame_hsv;
  cvtColor(frame_bgr, frame_hsv, COLOR_BGR2HSV);

  // Raw masks from HSV
  Mat orange_mask_raw = build_orange_mask_from_hsv(frame_hsv);
  Mat green_mask = build_green_mask_from_hsv(frame_hsv);

  // Window/door from BGR
  Mat edge_mask;
  vector<WindowCandidate> window_candidates = get_window_candidates_from_edges(frame_bgr, edge_mask);
  Mat door_mask = build_door_mask_from_window(frame_bgr.size(), window_candidates);

  // Split green into floor and obstacle
  Mat green_floor_mask, green_obstacle_mask_raw;
  split_green_floor_and_obstacles(green_mask, green_floor_mask, green_obstacle_mask_raw);

  // Geometry-validated masks
  Mat orange_mask_valid = build_valid_orange_mask(orange_mask_raw);
  Mat green_obstacle_mask_valid = build_valid_green_obstacle_mask(green_obstacle_mask_raw);

  // Risk computation
  RegionMetrics left_m, center_m, right_m;
  compute_column_metrics(orange_mask_valid, door_mask, green_obstacle_mask_valid,
                         left_m, center_m, right_m);

  string decision = decide_direction(left_m, center_m, right_m);

  uint8_t left_obs, center_obs, right_obs;
  risk_to_binary_obstacles(left_m, center_m, right_m,
                           &left_obs, &center_obs, &right_obs);

  VERBOSE_PRINT("Decision: %s\n", decision.c_str());
  VERBOSE_PRINT("Binary obstacle message -> Left: %d Center: %d Right: %d\n",
                left_obs, center_obs, right_obs);

  // Store result for periodic ABI publication
  pthread_mutex_lock(&detector_mutex);
  global_result.left = left_obs;
  global_result.middle = center_obs;
  global_result.right = right_obs;
  global_result.updated = true;
  pthread_mutex_unlock(&detector_mutex);

  // Optional debug drawing on frame_bgr
  draw_bboxes(frame_bgr,
              green_obstacle_mask_valid,
              Scalar(0, 255, 0),
              "Green obstacle",
              MIN_AREA_GREEN);

  draw_bboxes(frame_bgr,
              orange_mask_valid,
              Scalar(0, 140, 255),
              "Orange object",
              MIN_AREA_ORANGE,
              true,
              ORANGE_MIN_ASPECT_RATIO,
              ORANGE_MIN_HEIGHT);

  draw_door_from_window(frame_bgr, window_candidates);
  draw_navigation_columns(frame_bgr, left_m, center_m, right_m, decision);

  // Optional debug windows
  imshow("Detection", frame_bgr);
  // imshow("HSV", frame_hsv);
  // imshow("Edges", edge_mask);
  // imshow("Orange Raw", orange_mask_raw);
  // imshow("Orange Valid", orange_mask_valid);
  // imshow("Green Mask", green_mask);
  // imshow("Green Obstacle Raw", green_obstacle_mask_raw);
  // imshow("Green Obstacle Valid", green_obstacle_mask_valid);
  // imshow("Door Mask", door_mask);
  waitKey(1);

  return img;
}

// ----------------------------------------------------------------------------
// Paparazzi hooks
// ----------------------------------------------------------------------------

extern "C" {

void custom_detect_color_object_init(void)
{
  memset(&global_result, 0, sizeof(global_result));
  pthread_mutex_init(&detector_mutex, NULL);

#ifdef CUSTOM_DETECT_COLOR_OBJECT_CAMERA
  cv_add_to_device(&CUSTOM_DETECT_COLOR_OBJECT_CAMERA, object_detector, CUSTOM_DETECT_COLOR_OBJECT_FPS, 0);
#else
  PRINT("ERROR: CUSTOM_DETECT_COLOR_OBJECT_CAMERA is not defined in XML/airframe.\n");
#endif

  VERBOSE_PRINT("Initialized\n");
}

void custom_detect_color_object_periodic(void)
{
  struct custom_detection_result_t local_result;

  pthread_mutex_lock(&detector_mutex);
  memcpy(&local_result, &global_result, sizeof(local_result));
  global_result.updated = false;
  pthread_mutex_unlock(&detector_mutex);

  if (local_result.updated) {
    AbiSendMsgCUSTOM_DETECTION(CUSTOM_DETECT_COLOR_OBJECT_ID,
                               local_result.left,
                               local_result.middle,
                               local_result.right);

    VERBOSE_PRINT("Sent CUSTOM_DETECTION: left=%d middle=%d right=%d\n",
                  local_result.left, local_result.middle, local_result.right);
  }
}

}