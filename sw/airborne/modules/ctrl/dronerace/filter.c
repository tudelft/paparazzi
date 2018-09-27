

#include "filter.h"
#include "fifo.h"
#include "ransac.h"
#include "flightplan.h"
#include <math.h>
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

float filteredX, filteredY;


// PREDICTION MODEL

#define DR_FILTER_GRAVITY  9.81
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  0.8

void filter_predict(float phi, float theta, float psi, float dt)
{
  ////////////////////////////////////////////////////////////////////////
  // Body accelerations
  float az = DR_FILTER_GRAVITY / cosf(theta * DR_FILTER_THRUSTCORR) / cosf(phi * DR_FILTER_THRUSTCORR);
  float abx =  sinf(-theta) * az;
  float aby =  sinf( phi)   * az;

  // Earth accelerations
  float ax =  cosf(psi) * abx - sinf(psi) * aby - dr_state.vx * DR_FILTER_DRAG ;
  float ay =  sinf(psi) * abx + cosf(psi) * aby - dr_state.vy * DR_FILTER_DRAG;


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

  // Check if Ransac buffer is empty
  ransac_propagate(ax,ay,dt);

  filteredX = dr_state.x;
  filteredY = dr_state.y;

}

float log_mx, log_my;
float mx, my;
void transfer_measurement_local_2_global(float * mx,float *my,float dx,float dy);

void pushJungleGateDetection();

void filter_correct(void)
{
  // Retrieve oldest element of state buffer (that corresponds to current vision measurement)
  float sx, sy, sz;
  float rotx, roty;

  float vision_scale = 1.0f;

  fifo_pop(&sx, &sy, &sz);

  pushJungleGateDetection();

  /*
  // Compute current absolute position
  rotx =  cosf(dr_fp.gate_psi) * dr_vision.dx - sinf(dr_fp.gate_psi) * dr_vision.dy;
  roty = sinf(dr_fp.gate_psi) * dr_vision.dx + cosf(dr_fp.gate_psi) * dr_vision.dy;

  mx = dr_fp.gate_x + rotx * vision_scale;
  my = dr_fp.gate_y + roty * vision_scale;
   */

  transfer_measurement_local_2_global(&mx,&my,dr_vision.dx,dr_vision.dy);

  log_mx = dr_fp.gate_x;
  log_my = dr_fp.gate_y;

  // Push to RANSAC
  ransac_push(dr_state.time, dr_state.x, dr_state.y, mx, my);

  // for logging the filtering result  Shuo add
  filteredX = dr_state.x+dr_ransac.corr_x;
  filteredY = dr_state.y+dr_ransac.corr_y;
}


void transfer_measurement_local_2_global(float * mx,float *my,float dx,float dy)
{
    float min_distance = 9999;
    for(int i = 0;i<MAX_GATES;i++)
    {
        float rotx = cosf(gates[i].psi) * dx - sinf(gates[i].psi) * dy;
        float roty = sinf(gates[i].psi) * dx + cosf(gates[i].psi) * dy;
        float x = gates[i].x + rotx;
        float y = gates[i].y + roty;
        float distance_measured_2_drone = 0;
        distance_measured_2_drone = (x-(dr_state.x+dr_ransac.corr_x))*(x-(dr_state.x+dr_ransac.corr_x))+
                                    (y-(dr_state.y+dr_ransac.corr_y))*(y-(dr_state.y+dr_ransac.corr_y));
        if(distance_measured_2_drone < min_distance)
        {
            min_distance = distance_measured_2_drone;
            *mx = x;
            *my = y;
        }
    }
}

void pushJungleGateDetection()
{
    if(gates[dr_fp.gate_nr].type == JUNGLE && jungleGate.flagJungleGateDetected == false && jungleGate.numJungleGateDetection < MAX_DETECTION)
    {
        jungleGate.jungleGateDetection[jungleGate.numJungleGateDetection] = dr_vision.dz;
        jungleGate.sumJungleGateHeight += dr_vision.dz;
        jungleGate.numJungleGateDetection++;
        jungleGate.jungleGateHeight = jungleGate.sumJungleGateHeight / jungleGate.numJungleGateDetection;
        if(jungleGate.numJungleGateDetection == MAX_DETECTION)
        {
            jungleGate.flagJungleGateDetected = true;
            if(jungleGate.jungleGateHeight > -1.0)
                flagHighOrLowGate = UPPER_GATE;
            else
                flagHighOrLowGate = LOWER_GATE;
        }
    }
}
