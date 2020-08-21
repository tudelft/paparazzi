

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

float compl_V[3]={0};
float compl_pos[3]={0};

float buffer_vx[NR_SAMPLES_AVERAGE]={0};
float buffer_vy[NR_SAMPLES_AVERAGE]={0};
float buffer_vz[NR_SAMPLES_AVERAGE]={0};
void filter_reset()
{
  // Time
  dr_state.time = 0.0f;

  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();

  dr_state.x = pos->x;
  dr_state.y = pos->y;
  dr_state.z = pos->z; 

  // Speed
  dr_state.vx = vel->x;
  dr_state.vy = vel->y;

  compl_pos[0]= dr_state.x;
  compl_pos[1]= dr_state.y;
  compl_V[0]=dr_state.vx;
  compl_V[1]=dr_state.vy;
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

float filter_moving_avg(float x, float *buf){
  int i; 
  float valx=0;

  for(i=0;i<NR_SAMPLES_AVERAGE-1;i++)
  {
    buf[i]=buf[i+1];
    valx+=buf[i+1];
    // printf("buf[%d]: %f ",i,buf[i]);
  }
  buf[NR_SAMPLES_AVERAGE-1]=x;
  // printf("buf[%d]: %f ",NR_SAMPLES_AVERAGE-1,buf[NR_SAMPLES_AVERAGE-1]);
  valx+=x;
  // printf("total vx: %f, average: %f\n",valx,valx/(float)NR_SAMPLES_AVERAGE);
  return valx/(float)NR_SAMPLES_AVERAGE;

}

void complementary_filter_speed(float alpha, float beta, float Cd,float m,float VxGPS, float VyGPS, float VzGPS, float xGPS, float yGPS, float zGPS, float dt){
    float axvel=-DR_FILTER_GRAVITY*tanf(dr_state.theta); 
    float ayvel= DR_FILTER_GRAVITY*tanf(dr_state.phi)/cosf(dr_state.theta); 

    float axE=axvel*cosf(dr_state.psi)-ayvel*sinf(dr_state.psi)-compl_V[0]*Cd;
    float ayE=axvel*sinf(dr_state.psi)+ayvel*cosf(dr_state.psi)-compl_V[1]*Cd;

    compl_V[0]=alpha*(compl_V[0]+axE*dt) + (1-alpha)*VxGPS;
    compl_V[1]=alpha*(compl_V[1]+ayE*dt) + (1-alpha)*VyGPS;

    compl_pos[0]=alpha*(compl_pos[0]+compl_V[0]*dt)+(1-alpha)*xGPS;
    compl_pos[1]=alpha*(compl_pos[1]+compl_V[1]*dt)+(1-alpha)*yGPS;
    compl_pos[2]=beta*(compl_pos[2]+VzGPS*dt)+(1-beta)*zGPS; // not sure if it's a good idea to use complementary filter on Vz since the model assumes hover condition. Could it break when there are altitude variations?

}