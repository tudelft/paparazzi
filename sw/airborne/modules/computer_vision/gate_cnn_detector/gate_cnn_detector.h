#ifndef GATE_CNN_DETECTOR_H
#define GATE_CNN_DETECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GATE_CNN_INPUT_W 48
#define GATE_CNN_INPUT_H 12
#define GATE_CNN_THRESHOLD 0.2f       //TUNING FOR CONFIDENCE THRESHOLD


typedef struct {
  float present_prob;
  uint8_t present;
  float center_x;
  float center_y;
  float bbox_width;
  float bbox_height;
  float bbox_xyxy[4];
  uint8_t updated;
} gate_prediction_t;

void gate_cnn_detector_init(void);
void gate_cnn_detector_periodic(void);

#ifdef __cplusplus
}
#endif

#endif