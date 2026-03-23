/*
 * Edge-based gate and panel detection for drone racing.
 * Algorithm by Jort (lukemykon) — adapted for live YUV422 camera feed.
 *
 * Original: Edge_detectV7.cpp (edge_detection branch)
 */

#ifndef EDGE_GATE_DETECTION_H
#define EDGE_GATE_DETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Detect gates and panels in a YUV422 image using the Edge_detectV7 algorithm.
 *
 * Fills the gate_img best_gate struct (x_corners, y_corners, quality, n_sides)
 * with the best detected orange gate.
 *
 * @param img_buf       Pointer to YUV422 (UYVY) image buffer
 * @param width         Image width in pixels
 * @param height        Image height in pixels
 * @param x_corners     Output: 4 corner x-coords (rows for CAMERA_ROTATED_90DEG_RIGHT)
 * @param y_corners     Output: 4 corner y-coords (cols for CAMERA_ROTATED_90DEG_RIGHT)
 * @param quality       Output: detection quality [0..1]
 * @param n_sides       Output: number of polygon sides detected
 * @param draw          If nonzero, draw detections back into img_buf
 * @return 1 if gate found, 0 otherwise
 */
int edge_gate_detection(char *img_buf, int width, int height,
                        int x_corners[4], int y_corners[4],
                        float *quality, int *n_sides,
                        int draw);

#ifdef __cplusplus
}
#endif

#endif /* EDGE_GATE_DETECTION_H */
