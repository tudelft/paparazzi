#ifndef LOWRES_BLUE_H
#define LOWRES_BLUE_H

#include <stddef.h>

typedef struct {
    int window;
    float alpha;
    int hold_frames;
    float *history; /* size window * 2 (x,y) */
    int hist_count;
    int hist_pos;
    float smoothed_x;
    float smoothed_y;
    int smoothed_set;
    int missed;
} TargetTracker;

/* Convert BGR image (uint8, interleaved B,G,R) to HSV (uint8: H [0..179], S [0..255], V [0..255]) */
void bgr_to_hsv_uint8(const unsigned char *bgr, unsigned char *hsv, int w, int h);

/* Produce a binary blue mask (0 or 255) from a BGR image given HSV bounds */
void merged_mask_from_frame(const unsigned char *bgr, unsigned char *blue_mask, int w, int h,
                            const unsigned char lower[3], const unsigned char upper[3]);

/* Compute centroid of non-zero pixels in mask. Returns 1 if centroid found, 0 otherwise. */
int get_centroid(const unsigned char *mask, int w, int h, int *out_cx, int *out_cy);

/* Tracker lifecycle */
void tracker_init(TargetTracker *t, int window, float alpha, int hold_frames);
void tracker_free(TargetTracker *t);

/* Update tracker. If has_detection==1, detected_x/y used. Returns 1 if a center is available (either detected or held), 0 if none.
   When returning 1, out_cx/out_cy are set. out_seen is set to 1 when detection occurred this frame, 0 when held. */
int tracker_update(TargetTracker *t, int has_detection, int detected_x, int detected_y,
                   int *out_cx, int *out_cy, int *out_seen);

#endif /* LOWRES_BLUE_H */
