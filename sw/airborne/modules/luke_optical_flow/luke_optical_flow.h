/*
 * MAV Course 2026 - Luke Optical Flow Module
 *
 * Public interface for the opticflow-calculator wrapper.
 */

#ifndef LUKE_OPTICAL_FLOW_H
#define LUKE_OPTICAL_FLOW_H

#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/computer_vision/lib/vision/image.h"

// The divergence threshold used to decide "obstacle detected".
extern float luke_of_divergence_threshold;
extern bool luke_of_show_stream_overlay;
extern bool luke_of_derotation;
extern float luke_of_ema_alpha;
extern float luke_of_smoothed_divergence;
extern struct opticflow_t luke_of_opticflow[];

extern void luke_optical_flow_init(void);
extern void luke_optical_flow_periodic(void);

// Annotate an image copy in the RTP streamer with the latest divergence debug
// overlay. This is a no-op outside NPS and when the runtime toggle is off.
extern void luke_optical_flow_annotate_stream(struct image_t *img, uint8_t camera_id);

#endif /* LUKE_OPTICAL_FLOW_H */
