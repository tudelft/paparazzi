#include "lowres_blue.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

void bgr_to_hsv_uint8(const unsigned char *bgr, unsigned char *hsv, int w, int h) {
    int total = w * h;
    for (int i = 0; i < total; ++i) {
        float b = bgr[3*i + 0] / 255.0f;
        float g = bgr[3*i + 1] / 255.0f;
        float r = bgr[3*i + 2] / 255.0f;

        float cmax = r;
        if (g > cmax) cmax = g;
        if (b > cmax) cmax = b;
        float cmin = r;
        if (g < cmin) cmin = g;
        if (b < cmin) cmin = b;
        float delta = cmax - cmin;

        float h_deg = 0.0f;
        if (delta > 1e-8f) {
            if (cmax == r) {
                h_deg = fmodf((g - b) / delta, 6.0f) * 60.0f;
                if (h_deg < 0) h_deg += 360.0f;
            } else if (cmax == g) {
                h_deg = (((b - r) / delta) + 2.0f) * 60.0f;
            } else {
                h_deg = (((r - g) / delta) + 4.0f) * 60.0f;
            }
        } else {
            h_deg = 0.0f;
        }

        float s = 0.0f;
        if (cmax > 1e-8f) s = delta / cmax;
        float v = cmax;

        int h8 = (int)roundf(h_deg / 2.0f);
        if (h8 < 0) h8 = 0;
        if (h8 > 179) h8 = 179;
        int s8 = (int)roundf(s * 255.0f);
        if (s8 < 0) s8 = 0; if (s8 > 255) s8 = 255;
        int v8 = (int)roundf(v * 255.0f);
        if (v8 < 0) v8 = 0; if (v8 > 255) v8 = 255;

        hsv[3*i + 0] = (unsigned char)h8;
        hsv[3*i + 1] = (unsigned char)s8;
        hsv[3*i + 2] = (unsigned char)v8;
    }
}

void merged_mask_from_frame(const unsigned char *bgr, unsigned char *blue_mask, int w, int h,
                            const unsigned char lower[3], const unsigned char upper[3]) {
    int total = w * h;
    unsigned char *hsv = (unsigned char *)malloc((size_t)total * 3);
    if (!hsv) return;
    bgr_to_hsv_uint8(bgr, hsv, w, h);

    for (int i = 0; i < total; ++i) {
        unsigned char H = hsv[3*i + 0];
        unsigned char S = hsv[3*i + 1];
        unsigned char V = hsv[3*i + 2];
        if (H >= lower[0] && H <= upper[0] &&
            S >= lower[1] && S <= upper[1] &&
            V >= lower[2] && V <= upper[2]) {
            blue_mask[i] = 255;
        } else {
            blue_mask[i] = 0;
        }
    }

    free(hsv);
}

int get_centroid(const unsigned char *mask, int w, int h, int *out_cx, int *out_cy) {
    long long sumx = 0;
    long long sumy = 0;
    long long count = 0;
    for (int y = 0; y < h; ++y) {
        int row = y * w;
        for (int x = 0; x < w; ++x) {
            if (mask[row + x] > 0) {
                sumx += x;
                sumy += y;
                ++count;
            }
        }
    }
    if (count == 0) return 0;
    *out_cx = (int)((sumx + count/2) / count);
    *out_cy = (int)((sumy + count/2) / count);
    return 1;
}

void tracker_init(TargetTracker *t, int window, float alpha, int hold_frames) {
    if (!t) return;
    t->window = window > 0 ? window : 1;
    t->alpha = alpha;
    t->hold_frames = hold_frames;
    t->history = (float *)malloc(sizeof(float) * t->window * 2);
    if (t->history) memset(t->history, 0, sizeof(float) * t->window * 2);
    t->hist_count = 0;
    t->hist_pos = 0;
    t->smoothed_x = 0.0f;
    t->smoothed_y = 0.0f;
    t->smoothed_set = 0;
    t->missed = 0;
}

void tracker_free(TargetTracker *t) {
    if (!t) return;
    if (t->history) free(t->history);
    t->history = NULL;
    t->hist_count = 0;
}

int tracker_update(TargetTracker *t, int has_detection, int detected_x, int detected_y,
                   int *out_cx, int *out_cy, int *out_seen) {
    if (!t) return 0;
    if (has_detection) {
        t->missed = 0;
        /* append */
        int pos = t->hist_pos;
        t->history[2*pos + 0] = (float)detected_x;
        t->history[2*pos + 1] = (float)detected_y;
        t->hist_pos = (t->hist_pos + 1) % t->window;
        if (t->hist_count < t->window) t->hist_count++;

        /* compute local average */
        float avgx = 0.0f, avgy = 0.0f;
        for (int i = 0; i < t->hist_count; ++i) {
            avgx += t->history[2*i + 0];
            avgy += t->history[2*i + 1];
        }
        avgx /= (float)t->hist_count;
        avgy /= (float)t->hist_count;

        if (!t->smoothed_set) {
            t->smoothed_x = avgx;
            t->smoothed_y = avgy;
            t->smoothed_set = 1;
        } else {
            t->smoothed_x = (1.0f - t->alpha) * t->smoothed_x + t->alpha * avgx;
            t->smoothed_y = (1.0f - t->alpha) * t->smoothed_y + t->alpha * avgy;
        }

        if (out_cx) *out_cx = (int)roundf(t->smoothed_x);
        if (out_cy) *out_cy = (int)roundf(t->smoothed_y);
        if (out_seen) *out_seen = 1;
        return 1;
    }

    /* no detection */
    t->missed += 1;
    if (t->smoothed_set && t->missed <= t->hold_frames) {
        if (out_cx) *out_cx = (int)roundf(t->smoothed_x);
        if (out_cy) *out_cy = (int)roundf(t->smoothed_y);
        if (out_seen) *out_seen = 0;
        return 1;
    }

    /* give up */
    t->hist_count = 0;
    t->hist_pos = 0;
    t->smoothed_set = 0;
    return 0;
}
