#include "fifo.h"


// VISION LATENCY COMPENSATION

#define VISION_LATENCY_TIME_STEPS    10    // Time steps: note: must be at least 1

struct dronerace_vision_struct
{
  float x[VISION_LATENCY_TIME_STEPS];
  float y[VISION_LATENCY_TIME_STEPS];
  float z[VISION_LATENCY_TIME_STEPS];
  int index;
} dr_past_state;


void fifo_reset(void)
{
  int i;
  for (i=0; i<VISION_LATENCY_TIME_STEPS;i++) {
    dr_past_state.x[i] = 0;
    dr_past_state.y[i] = 0;
    dr_past_state.z[i] = 0;
  }
  dr_past_state.index = 0;
}

void fifo_push(float x, float y, float z)
{
  dr_past_state.index++;
  if (dr_past_state.index >= VISION_LATENCY_TIME_STEPS) {
    dr_past_state.index = 0;
  }
  dr_past_state.x[dr_past_state.index] = x;
  dr_past_state.y[dr_past_state.index] = y;

}
void fifo_pop(float *x, float *y, float *z)
{
  int index = dr_past_state.index + 1;
  if (index >= VISION_LATENCY_TIME_STEPS) {
    index = 0;
  }
  *x = dr_past_state.x[index];
  *y = dr_past_state.y[index];
  *z = dr_past_state.z[index];
}
