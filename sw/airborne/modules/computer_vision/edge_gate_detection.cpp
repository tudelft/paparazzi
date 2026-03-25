/*
 * Edge-based gate and panel detection for drone racing.
 *
 * This is Jort's (lukemykon) Edge_detectV7 algorithm, adapted and tuned
 * for Paparazzi runtime use on a live YUV422 camera buffer instead of reading
 * JPG files from disk.
 *
 * Original: Edge_detectV7.cpp (edge_detection branch of lukemykon/paparazzi)
 *
 * Algorithm steps:
 *  1. Green field masking   — HSV (30,100,80)-(55,255,180)
 *  2. Line detection        — Canny + HoughLinesP, EMA smoothing (alpha=0.8)
 *  3. Panel detection       — grayscale < 75, pentagon/hexagon contours
 *  4. Gate detection orange  — HSV (5,100,100)-(35,255,255), approxPolyDP, angle
 */

#include "edge_gate_detection.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cmath>

using namespace cv;
using namespace std;

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

// Sorting points by Y-value  (exact copy from Edge_detectV7)
static bool sortByY(const Point &a, const Point &b) { return a.y < b.y; }
// Sorting points by X-value  (exact copy from Edge_detectV7)
static bool sortByX(const Point &a, const Point &b) { return a.x < b.x; }

/* Convert YUV422 (UYVY) buffer to a BGR Mat so OpenCV can process it */
static Mat yuv422_to_bgr(const char *buf, int w, int h)
{
    Mat yuv(h, w, CV_8UC2, (void *)buf);
    Mat bgr;
    cvtColor(yuv, bgr, COLOR_YUV2BGR_UYVY);
    return bgr;
}

/* Draw a line into the YUV422 buffer (Bresenham) */
static void draw_line_yuv422(char *buf, int w, int h,
                              int x0, int y0, int x1, int y1,
                              uint8_t yc, uint8_t uc, uint8_t vc)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
            int idx = y0 * w + x0;
            buf[2 * idx + 1] = yc;
            buf[2 * idx] = (x0 & 1) ? vc : uc;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* ------------------------------------------------------------------ */
/*  EMA state — persistent across frames, just like Edge_detectV7     */
/* ------------------------------------------------------------------ */
static double alpha = 0.8;
static Vec4f smoothed_left(0, 0, 0, 0), smoothed_right(0, 0, 0, 0);
static bool init_left = false, init_right = false;

/* ------------------------------------------------------------------ */
/*  Main entry point                                                  */
/* ------------------------------------------------------------------ */
extern "C" int edge_gate_detection(char *img_buf, int width, int height,
                                    int x_corners[4], int y_corners[4],
                                    float *quality, int *n_sides,
                                    int draw)
{
    /* ====== Convert live YUV422 buffer to BGR (replaces imread) ====== */
    Mat img = yuv422_to_bgr(img_buf, width, height);
    if (img.empty()) return 0;

    /* ====== Exact Edge_detectV7 algorithm from here ====== */

    Mat blurred, hsv, mask;
    GaussianBlur(img, blurred, Size(5, 5), 0);
    cvtColor(blurred, hsv, COLOR_BGR2HSV);

    // 1. Green Field Masking  (exact thresholds from Edge_detectV7)
    Scalar lower_green(30, 100, 80), upper_green(55, 255, 180);
    inRange(hsv, lower_green, upper_green, mask);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat clean_mask = Mat::zeros(img.size(), CV_8UC1);
    int top_y = 0;

    if (!contours.empty()) {
        double max_area = 0;
        int max_idx = -1;
        for (int j = 0; j < (int)contours.size(); j++) {
            double area = contourArea(contours[j]);
            if (area > max_area) { max_area = area; max_idx = j; }
        }

        if (max_idx != -1 && max_area > (img.rows * img.cols * 0.05)) {
            vector<Point> hull;
            convexHull(contours[max_idx], hull);
            fillConvexPoly(clean_mask, hull, Scalar(255));

            Rect br = boundingRect(hull);
            top_y = br.y;
            clean_mask(Rect(0, 0, img.cols, max(0, top_y - 10))).setTo(0);
        }
    }

    // 2. Line Detection  (exact from Edge_detectV7)
    Mat edges;
    Canny(clean_mask, edges, 50, 150);
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 40, 60, 100);

    Vec4f curr_left, curr_right;
    bool found_left = false, found_right = false;

    for (auto &l : lines) {
        float dx = (float)(l[2] - l[0]);
        float dy = (float)(l[3] - l[1]);
        if (dx == 0) continue;
        float slope = dy / dx;

        if (slope > 0.2)       { curr_left  = Vec4f((float)l[0],(float)l[1],(float)l[2],(float)l[3]); found_left  = true; }
        else if (slope < -0.2) { curr_right = Vec4f((float)l[0],(float)l[1],(float)l[2],(float)l[3]); found_right = true; }
    }

    // Smoothing (EMA)  (exact from Edge_detectV7)
    if (found_left) {
        if (!init_left) { smoothed_left = curr_left; init_left = true; }
        else smoothed_left = alpha * curr_left + (1.0 - alpha) * smoothed_left;
    }
    if (found_right) {
        if (!init_right) { smoothed_right = curr_right; init_right = true; }
        else smoothed_right = alpha * curr_right + (1.0 - alpha) * smoothed_right;
    }

    // Draw boundary lines (replaces cv::line with YUV422 drawing)
    if (draw) {
        // Yellow for left line (Y:226 U:0 V:149)
        if (init_left)
            draw_line_yuv422(img_buf, width, height,
                              (int)smoothed_left[0], (int)smoothed_left[1],
                              (int)smoothed_left[2], (int)smoothed_left[3],
                              226, 0, 149);
        // Magenta for right line (Y:105 U:212 V:234)
        if (init_right)
            draw_line_yuv422(img_buf, width, height,
                              (int)smoothed_right[0], (int)smoothed_right[1],
                              (int)smoothed_right[2], (int)smoothed_right[3],
                              105, 212, 234);
    }

    // 3. Panel Detection  (exact from Edge_detectV7)
    Mat gray, dark, dark_on_mat;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    threshold(gray, dark, 75, 255, THRESH_BINARY_INV);
    bitwise_and(dark, clean_mask, dark_on_mat);

    Mat kernel_p = getStructuringElement(MORPH_RECT, Size(9, 9));
    morphologyEx(dark_on_mat, dark_on_mat, MORPH_CLOSE, kernel_p);

    vector<vector<Point>> p_contours;
    findContours(dark_on_mat, p_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (auto &pcnt : p_contours) {
        if (contourArea(pcnt) > 1000) {
            vector<Point> approx;
            approxPolyDP(pcnt, approx, 0.03 * arcLength(pcnt, true), true);
            if (approx.size() >= 5 && approx.size() <= 6) {
                // pentagon/hexagon = panel  (exact from Edge_detectV7)
                if (draw) {
                    RotatedRect rr = minAreaRect(pcnt);
                    Point2f vtx[4];
                    rr.points(vtx);
                    // Draw panel in blue (Y:41 U:240 V:110)
                    for (int j = 0; j < 4; j++)
                        draw_line_yuv422(img_buf, width, height,
                                          (int)vtx[j].x, (int)vtx[j].y,
                                          (int)vtx[(j+1)%4].x, (int)vtx[(j+1)%4].y,
                                          41, 240, 110);
                }
                // Panel detected — no log spam, just draw
            }
        }
    }

    // 4. Gate Detection (Orange)  (exact from Edge_detectV7)
    Mat gate_mask;
    inRange(hsv, Scalar(5, 100, 100), Scalar(35, 255, 255), gate_mask);
    morphologyEx(gate_mask, gate_mask, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));

    vector<vector<Point>> g_contours;
    findContours(gate_mask, g_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Find the best gate among all qualifying contours
    int best_idx = -1;
    double best_area = 0;
    double best_score = -1.0;
    vector<Point> best_approx;
    Rect best_br;

    for (int i = 0; i < (int)g_contours.size(); i++) {
        Rect br = boundingRect(g_contours[i]);
        float ar = (float)br.width / (float)br.height;

        // Exact thresholds from Edge_detectV7: area > 800, aspect 0.6..1.6
        if (contourArea(g_contours[i]) > 800 && ar > 0.6 && ar < 1.6) {
            vector<Point> approx;
            approxPolyDP(g_contours[i], approx, 0.04 * arcLength(g_contours[i], true), true);
            if (approx.size() >= 4 && approx.size() <= 6) {
                double area = contourArea(g_contours[i]);
                double bbox_area = (double)br.width * (double)br.height;
                double fill_ratio = (bbox_area > 0.0) ? (area / bbox_area) : 0.0;
                if (fill_ratio < 0.08) {
                    continue;
                }

                int sides = (int)approx.size();
                double side_weight = (sides == 4) ? 1.0 : ((sides == 5) ? 0.75 : 0.50);
                double score = (side_weight * fill_ratio) + (0.00002 * area);

                if (score > best_score) {
                    best_area = area;
                    best_idx = i;
                    best_approx = approx;
                    best_br = br;
                    best_score = score;
                }
            }
        }
    }

    if (best_idx < 0) {
        *quality = 0.0f;
        *n_sides = 0;
        return 0;
    }

    // Angle calculation  (exact from Edge_detectV7)
    vector<Point> pts = best_approx;
    sort(pts.begin(), pts.end(), sortByY);
    vector<Point> top_pts = { pts[0], pts[1] };
    sort(top_pts.begin(), top_pts.end(), sortByX);

    float dx = (float)(top_pts[1].x - top_pts[0].x);
    float dy = (float)(top_pts[1].y - top_pts[0].y);
    double angle = (dx != 0) ? atan2(dy, dx) * 180.0 / CV_PI : 0;

    string status = (abs(angle) < 2) ? "Straight" : "Angle: " + to_string(angle).substr(0, 4) + " deg";

    // Draw gate  (replaces polylines + putText with YUV422 drawing)
    if (draw) {
        // Cyan polylines (Y:188 U:154 V:16)
        for (int j = 0; j < (int)best_approx.size(); j++) {
            int j2 = (j + 1) % (int)best_approx.size();
            draw_line_yuv422(img_buf, width, height,
                              best_approx[j].x, best_approx[j].y,
                              best_approx[j2].x, best_approx[j2].y,
                              188, 154, 16);
        }
        // If angled, draw red cross (Y:76 U:84 V:255)
        if (abs(angle) >= 2) {
            draw_line_yuv422(img_buf, width, height,
                              best_br.x, best_br.y,
                              best_br.x + best_br.width, best_br.y + best_br.height,
                              76, 84, 255);
            draw_line_yuv422(img_buf, width, height,
                              best_br.x + best_br.width, best_br.y,
                              best_br.x, best_br.y + best_br.height,
                              76, 84, 255);
        }
    }

    /* ====== End of Edge_detectV7 algorithm ====== */

    /* ====== Fill output for detect_gate.c PnP ====== */
    // Snake gate convention (set_gate_points): x_corners = COLUMNS, y_corners = ROWS
    //   Corner order: "bottom-right CCW" in Bebop coordinates:
    //     0: (x-sz, y+sz)  1: (x+sz, y+sz)  2: (x+sz, y-sz)  3: (x-sz, y-sz)
    //
    // detect_gate.c (CAMERA_ROTATED_90DEG_RIGHT) uses:
    //   sz1 = x_corners[2] - x_corners[0]   → col span (must be positive)
    //   sz2 = y_corners[1] - y_corners[0]   → row span (0 for square gate)
    //   pix_y = (x_corners[1] + x_corners[0]) / 2  → center column
    //   pix_x = (y_corners[2] + y_corners[1]) / 2  → center row

    int center_col = best_br.x + best_br.width / 2;
    int center_row = best_br.y + best_br.height / 2;
    // Use the larger dimension as sz (gate is approximately square)
    int sz = (best_br.width > best_br.height) ? best_br.width / 2 : best_br.height / 2;

    x_corners[0] = center_col - sz;   // left col
    x_corners[1] = center_col + sz;   // right col
    x_corners[2] = center_col + sz;   // right col
    x_corners[3] = center_col - sz;   // left col

    y_corners[0] = center_row + sz;   // bottom row
    y_corners[1] = center_row + sz;   // bottom row
    y_corners[2] = center_row - sz;   // top row
    y_corners[3] = center_row - sz;   // top row

    double bbox_area = (double)(best_br.width * best_br.height);
    *quality = (bbox_area > 0) ? (float)(best_area / bbox_area) : 0.0f;
    if (*quality > 1.0f) *quality = 1.0f;
    *n_sides = (int)best_approx.size();

    fprintf(stderr, "[edge_detect] GATE: bbox=(%d,%d,%d,%d) q=%.2f sides=%d sz=%d angle=%.1f %s\n",
            best_br.x, best_br.y, best_br.width, best_br.height,
            *quality, *n_sides, sz, angle, status.c_str());

    return 1;
}
