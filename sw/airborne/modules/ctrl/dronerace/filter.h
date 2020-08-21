#ifndef DRONERACE_FILTER_H_
#define DRONERACE_FILTER_H_
#include <stdio.h>
struct dronerace_vision_struct {
  int cnt;
  float dx;
  float dy;
  float dz;
};

// store for logging purposes
extern struct dronerace_vision_struct dr_vision;

#define NR_SAMPLES_AVERAGE 10

extern float buffer_vx[NR_SAMPLES_AVERAGE];
extern float buffer_vy[NR_SAMPLES_AVERAGE];
extern float buffer_vz[NR_SAMPLES_AVERAGE];

struct dronerace_state_struct {
  // Time
  float time;

  // Positon
  float x;
  float y;
  float z; 

  // Speed
  float vx;
  float vy;

  // Heading
  float theta;
  float phi;
  float psi;

  // Logic
  int assigned_gate_index;
};

extern float log_mx, log_my;

extern struct dronerace_state_struct dr_state;


extern void filter_reset(void);

extern void filter_predict(float phi, float theta, float psi, float dt);
extern void filter_predict1(float phi, float theta, float psi, float dt);
extern float filter_moving_avg(float x, float *buf);
extern void filter_correct(void);
extern void complementary_filter_speed(float alpha,float Cd,float m,float VxGPS, float VyGPS,float dt);
extern float mx;
extern float my;
extern float filteredX;
extern float filteredY;

extern float filter_az;
extern float filter_abx;
extern float filter_aby;
extern float filter_ax;
extern float filter_ay;
extern float compl_V[2];
FILE *filter_log_t;
#endif