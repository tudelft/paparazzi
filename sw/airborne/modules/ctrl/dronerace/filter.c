

#include "filter.h"
#include "fifo.h"
#include "ransac.h"
#include "flightplan.h"
#include <math.h>
#include "std.h"
#include "stdio.h"
#include "state.h"

#include "math/pprz_algebra_float.h"

// to know if we are simulating:
#include "generated/airframe.h"

struct dronerace_state_struct dr_state = {0};
struct dronerace_vision_struct dr_vision = {0};

void filter_reset()
{
  // Time
  dr_state.time = 0.0f;

  struct NedCoor_f *pos = stateGetPositionNed_f();

  dr_state.x = pos->x;
  dr_state.y = pos->y;
  dr_state.z = pos->z; 

  // Speed
  dr_state.vx = 0.0f;
  dr_state.vy = 0.0f;

  printf("filter reset \n\n\n");
  // Heading
  dr_state.psi = 0.0f;
}


// PREDICTION MODEL

#define DR_FILTER_GRAVITY  9.81
#if SIMULATE
#define DR_FILTER_DRAG  0.95
#define DR_FILTER_THRUSTCORR  0.8
#else
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  0.8
#endif
float filteredX, filteredY, filter_abx, filter_aby, filter_ax, filter_ay, filter_az;
void filter_predict(float phi, float theta, float psi, float dt)
{
  ////////////////////////////////////////////////////////////////////////
  // Body accelerations
  BoundAbs(phi, RadOfDeg(50));
  BoundAbs(theta, RadOfDeg(50));
  float az = DR_FILTER_GRAVITY / (cosf(theta * DR_FILTER_THRUSTCORR) * cosf(phi * DR_FILTER_THRUSTCORR));
  float abx =  sinf(-theta) * az;
  float aby =  sinf(phi)   * az;

  // Earth accelerations
  float ax =  cosf(psi) * abx - sinf(psi) * aby - dr_state.vx * DR_FILTER_DRAG ; //TODO shouldnt the drag term be divided by mass? assuming Drag force = v*DR_FILTER_DRAG
  float ay =  sinf(psi) * abx + cosf(psi) * aby - dr_state.vy * DR_FILTER_DRAG;


  // Velocity and Position
  dr_state.vx += ax * dt;
  dr_state.vy += ay * dt;
  dr_state.x += dr_state.vx * dt;
  dr_state.y += dr_state.vy * dt;

  // Time
  dr_state.time += dt;

  // Store psi for local corrections
  dr_state.psi = psi; //TODO: use psi command?

  // Store old states for latency compensation
  fifo_push(dr_state.x, dr_state.y, 0);

  // Check if Ransac buffer is empty
  //ransac_propagate();

  filteredX = dr_state.x;
  filteredY = dr_state.y;

  filter_az=az;
  filter_abx=abx;
  filter_aby=aby;
  filter_ax=ax;
  filter_ay=ay;

}
