

#include "filter.h"
#include "fifo.h"
#include "ransac.h"
#include "flightplan.h"

#include "std.h"



struct dronerace_state_struct dr_state;
struct dronerace_vision_struct dr_vision;



void filter_reset()
{
  // Time
  dr_state.time = 0;

  // Position
  dr_state.x = 0;
  dr_state.y = 0;

  // Speed
  dr_state.vx = 0;
  dr_state.vy = 0;

  // Heading
  dr_state.psi = 0;

  // Vision latency
  fifo_reset();
  ransac_reset();
}




// PREDICTION MODEL

#define DR_FILTER_GRAVITY  9.81
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  0.8

void filter_predict(float phi, float theta, float psi, float dt)
{
  ////////////////////////////////////////////////////////////////////////
  // Body accelerations
  float abx =  sin(-theta) * DR_FILTER_GRAVITY / cos(theta * DR_FILTER_THRUSTCORR) / cos(phi * DR_FILTER_THRUSTCORR);
  float aby =  sin( phi)   * DR_FILTER_GRAVITY / cos(theta * DR_FILTER_THRUSTCORR) / cos(phi * DR_FILTER_THRUSTCORR);

  // Earth accelerations
  float ax =  cos(psi) * abx - sin(psi) * aby - dr_state.vx * DR_FILTER_DRAG ;
  float ay =  sin(psi) * abx + cos(psi) * aby - dr_state.vy * DR_FILTER_DRAG;


  // Velocity and Position
  dr_state.vx += ax * dt;
  dr_state.vy += ay * dt;
  dr_state.x += dr_state.vx * dt;
  dr_state.y += dr_state.vy * dt;

  // Time
  dr_state.time += dt;

  // Store psi for local corrections
  dr_state.psi = psi;

  // Store old states for latency compensation
  fifo_push(dr_state.x, dr_state.y, 0);
}

void filter_correct(void)
{
  // Retrieve oldest element of state buffer (that corresponds to current vision measurement)
  float sx, sy, sz;
  float mx, my;

  fifo_pop(&sx, &sy, &sz);

  // Compute current absolute position
  // TODO: check: this is probably wrong!
  mx = dr_fp.gate_x - dr_vision.dx;
  my = dr_fp.gate_y - dr_vision.dy;

  // Push to RANSAC
  ransac_push(dr_state.time, dr_state.x, dr_state.y, mx, my);
}
