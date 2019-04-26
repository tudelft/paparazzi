

#include "filter.h"
#include "fifo.h"
#include "ransac.h"
#include "flightplan.h"
#include <math.h>
#include "std.h"
#include "stdio.h"
#include "state.h"

// to know if we are simulating:
#include "generated/airframe.h"


struct dronerace_state_struct dr_state;
struct dronerace_vision_struct dr_vision;



void filter_reset()
{
  // Time
  dr_state.time = 0.0f;

  // Position
  dr_state.x = 0.0f;
  dr_state.y = 0.0f;

  // Speed
  dr_state.vx = 0.0f;
  dr_state.vy = 0.0f;

  // Heading
  dr_state.psi = 0.0f;

  // Vision latency
  fifo_reset();
  //ransac_reset();
}

float filteredX, filteredY;


// PREDICTION MODEL

#define DR_FILTER_GRAVITY  9.81
#if SIMULATE
#define DR_FILTER_DRAG  0.95
#define DR_FILTER_THRUSTCORR  0.8
#else
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  0.8
#endif

#define VERSION_SUPERSTATE 1

#ifdef VERSION_SUPERSTATE

bool first_call = true;

float *velWorld;
float *posWorld;

void filter_predict(float phi, float theta, float psi, float dt)
{
  ////////////////////////////////////////////////////////////////////////
  // Body accelerations
  if (first_call) {
    // init is imp, dead reckon
    velWorld = (float [3]) {0,0,0};
    posWorld = (float [3]) {stateGetPositionNed_f()->x, stateGetPositionNed_f()->y, stateGetPositionNed_f()->z};

    first_call = false;
  }

  float az = -9.81 / (cosf(theta) * cosf(phi));

  // float bodyVel[3] = {dr_state.vx, dr_state.vy, 0}; // TODO: replace to vz

  float world2body[3][3] ={{cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)},
      {sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta)},
      {cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)}};
  
  float rpmAvg = (actuators_bebop.rpm_obs[0] + actuators_bebop.rpm_obs[1] + actuators_bebop.rpm_obs[2] + actuators_bebop.rpm_obs[3])/4;
  
  float kdx = 1.0428;
  float kdy = 1.0095;

  float kd[3][3] = {{-kdx * rpmAvg, 0, 0}, {0, -kdy * rpmAvg, 0}, {0,0,0}};


  //  accBody = kd * world2body * worldVel;
  float accBody_temp[3] = {0};
  float bodyVel_temp[3] = {0};
  float_mat_vect_mul(bodyVel_temp, world2body, velWorld, 3, 3); // TODO: but this is scaled :( )
  float_mat_vect_mul(accBody_temp, kd, bodyVel_temp, 3, 3);
  
  float accBody[3] = {accBody_temp[1], accBody_temp[2], az};

  // acc_t = ([0;0;9.8] + R'* [0;0;thrust(i,3)] + R'* [a_body(i,1); a_body(i,2); 0])';
  float grav[3] = {0, 0, 9.8};
  float body2world[3][3] = {0};
  float_mat_transpose(body2world, world2body, 3, 3); 

  float accWorld[3] = {0};
  float_mat_vect_mul(accWorld, body2world, accBody, 3, 3);
  float_vect_add(accWorld, grav, 3);

  float tmp[3] = {0};
  float_vect_smul(tmp, accWorld, dt, 3);    // tmp  = accWorld*dt

  float_vect_add(velWorld, tmp, 3);   // velWorld += accWorld*dt
  
  float_vect_smul(tmp, velWorld, dt, 3);    // tmp = velWorld * dt

  float_vect_add(posWorld, tmp, 3);   // posWorld += velWorld * dt
  
  //TODO:  + 0.5 * accWorld * dt * dt; 
  dr_state.vx += accWorld[1]; // * dt; // already scaled by dt
  dr_state.vy += accWorld[2]; // * dt; // already scaled by dt
  dr_state.x +=  dr_state.vx * dt; //TODO: + 0.5 * dr_state.accWorld[1] * dt *dt;
  dr_state.y +=  dr_state.vy * dt; //TODO: + 0.5 * dr_state.accWorld[2] * dt *dt;


    /*
    % Kdrag - Kumar: work here
    kd = rpmAvg(i,1) * [-kdx 0 0; 0 -kdy 0; 0 0 0];
    a_body(i, 1:3) = (kd * R * vel_w2(i-1, 1:3)')'; % plus some bias. 
    
    acc_t = ([0;0;9.8] + R'* [0;0;thrust(i,3)] + R'* [a_body(i,1); a_body(i,2); 0])';
    acc_w2(i,1) = acc_t(1);
    acc_w2(i,2) = acc_t(2);
    acc_w2(i,3) = acc_t(3);
    vel_w2(i,1:3) = vel_w2(i-1,1:3) + (acc_w2(i,1:3) .* dt);
    pos_w2(i,1:3) = pos_w2(i-1,1:3) + (vel_w2(i,1:3) .* dt); 
    */

  // Store psi for local corrections
  dr_state.psi = psi; // TODO: use psi command?

  // Store old states for latency compensation
  fifo_push(dr_state.x, dr_state.y, 0);

  // Check if Ransac buffer is empty
  //ransac_propagate();

  filteredX = dr_state.x;
  filteredY = dr_state.y;

}



#else

void filter_predict(float phi, float theta, float psi, float dt)
{
  ////////////////////////////////////////////////////////////////////////
  // Body accelerations
  BoundAbs(phi, RadOfDeg(50));
  BoundAbs(theta, RadOfDeg(50));
  float az = DR_FILTER_GRAVITY / cosf(theta * DR_FILTER_THRUSTCORR) / cosf(phi * DR_FILTER_THRUSTCORR);
  float abx =  sinf(-theta) * az;
  float aby =  sinf(phi)   * az;

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
  dr_state.psi = psi; // TODO: use psi command?

  // Store old states for latency compensation
  fifo_push(dr_state.x, dr_state.y, 0);

  // Check if Ransac buffer is empty
  //ransac_propagate();

  filteredX = dr_state.x;
  filteredY = dr_state.y;

}

#endif

float log_mx, log_my;
float mx, my;
// int transfer_measurement_local_2_global(float *mx, float *my, float dx, float dy);

// void pushJungleGateDetection(void);

/*void filter_correct(void)
{
  // Retrieve oldest element of state buffer (that corresponds to current vision measurement) // TODO: should we not empirically determine the delay (is it now just guessed?)
  float sx, sy, sz;

  fifo_pop(&sx, &sy, &sz);

  // TODO: we should actually check that the determined height is not so different from the gate height, given that we are not looking at the jungle gate
  // With the check on dr_vision.dz, we want to exclude the detection of the gate botom part.
  //  && dr_vision.dz > -2.5
  if (gates[dr_fp.gate_nr].type != VIRTUAL) {

    int assigned_gate = transfer_measurement_local_2_global(&mx, &my, dr_vision.dx, dr_vision.dy);

    //printf("assigned gate = %d, gate nr = %d.\n", assigned_gate, dr_fp.gate_nr);

    if (assigned_gate == dr_fp.gate_nr) {

      pushJungleGateDetection();

      log_mx = dr_fp.gate_x;
      log_my = dr_fp.gate_y;

      // Push to RANSAC
      ransac_push(dr_state.time, dr_state.x, dr_state.y, mx, my);

      // for logging the filtering result  Shuo add
      filteredX = dr_state.x + dr_ransac.corr_x;
      filteredY = dr_state.y + dr_ransac.corr_y;

      return;
    }
  }

  filteredX = dr_state.x;
  filteredY = dr_state.y;
  return;
}*/

#if 0
int transfer_measurement_local_2_global(float *_mx, float *_my, float dx, float dy)
{
  int i, j;
  // TODO: reintroduce vision scale?
  float min_distance = 9999;

  dr_state.assigned_gate_index = -1;

  for (i = 0; i < MAX_GATES; i++) {
    if (gates[i].type != VIRTUAL) {
      float exp_dx = gates[i].x - dr_state.x;
      float exp_dy = gates[i].y - dr_state.y;
      float exp_yaw = gates[i].psi - dr_state.psi;
      float exp_dist = sqrtf(exp_dx * exp_dx + exp_dy * exp_dy);
      if (exp_dist == 0.0) {
        exp_dist = 0.0001f;
      }
      float exp_size =  1.4f * 340.0f / exp_dist;
      // dist = 1.4f * 340.0f / ((float)size);
      float exp_bearing = atan2(exp_dy, exp_dx);
      float exp_view = exp_bearing - dr_state.psi;
      if ((exp_view > -320.0f / 340.0f) && (exp_view < 320.0f / 340.0f)
          && ((exp_yaw > -RadOfDeg(60.0f)) && (exp_yaw < RadOfDeg(60.0f)))
         ) {

        float rot_dx = cosf(dr_state.psi) * dx -sinf(dr_state.psi) * dy;
        float rot_dy = sinf(dr_state.psi) * dx + cosf(dr_state.psi) * dy;

        float x = gates[i].x + rot_dx;
        float y = gates[i].y + rot_dy;
        float distance_measured_2_drone = 0;
        distance_measured_2_drone = (x - (dr_state.x + dr_ransac.corr_x)) * (x - (dr_state.x + dr_ransac.corr_x)) +
                                    (y - (dr_state.y + dr_ransac.corr_y)) * (y - (dr_state.y + dr_ransac.corr_y));
        if (distance_measured_2_drone < min_distance) {
          dr_state.assigned_gate_index = i;
          min_distance = distance_measured_2_drone;
          *_mx = x;
          *_my = y;
          //printf("Mx = %f, my = %f\n", *_mx, *_my);
        }
          //printf("Expected gates: %d  %.1f s=%.1f heading %.1f rot %.1f\n", i, dist, size, px, yaw * 57.6f);
      }
    }
  }

  if(dr_state.assigned_gate_index == -1) {
    dr_state.assigned_gate_index = dr_fp.gate_nr;
  }

  //printf("Final assigned gate = %d: mx,my = %f,%f\n", dr_state.assigned_gate_index, *_mx, *_my);



  /*
  for (i = 0; i < MAX_GATES; i++) {
    if (gates[i].type == VIRTUAL) {
      continue;
    }
    // we can detect the gate from the back side, so not only check one gate in front. But also check back
    for (j = 0; j < 2; j++) {
      if (j == 1 && !gates[dr_fp.gate_nr].both_side) {
        break;
        //if the drone are at the back side of the gate and there is white paper on the gate,
        // we should not consider this gate
      }
      {
        float psi = gates[i].psi - j * RadOfDeg(180);
        float rotx = cosf(psi) * dx - sinf(psi) * dy;
        float roty = sinf(psi) * dx + cosf(psi) * dy;

        float x = gates[i].x + rotx;
        float y = gates[i].y + roty;
        float distance_measured_2_drone = 0;
        distance_measured_2_drone = (x - (dr_state.x + dr_ransac.corr_x)) * (x - (dr_state.x + dr_ransac.corr_x)) +
                                    (y - (dr_state.y + dr_ransac.corr_y)) * (y - (dr_state.y + dr_ransac.corr_y));
        if (distance_measured_2_drone < min_distance) {
          dr_state.assigned_gate_index = i;
          min_distance = distance_measured_2_drone;
          *_mx = x;
          *_my = y;
        }
      }
    }
  }*/
  // printf("Assigned gate = %d, (dx,dy) = (%f,%f), (mx,my) = (%f,%f).\n", dr_state.assigned_gate_index, dx, dy, (*_mx), (*_my));
  return dr_state.assigned_gate_index;
}


void pushJungleGateDetection(void)
{
  if (gates[dr_fp.gate_nr].type == JUNGLE && jungleGate.flagJungleGateDetected == false
      && jungleGate.numJungleGateDetection < MAX_DETECTION) {
    jungleGate.jungleGateDetectionZ[jungleGate.numJungleGateDetection] = dr_vision.dz;
    jungleGate.jungleGateDetectionY[jungleGate.numJungleGateDetection] = dr_vision.dy;
    jungleGate.sumJungleGateHeight += dr_vision.dz;
    jungleGate.numJungleGateDetection++;
    jungleGate.jungleGateHeight = jungleGate.sumJungleGateHeight / jungleGate.numJungleGateDetection;
    if (jungleGate.numJungleGateDetection == MAX_DETECTION) {
      jungleGate.flagJungleGateDetected = true;
      if (jungleGate.jungleGateHeight > 0.0) {
        flagHighOrLowGate = UPPER_GATE;
      } else {
        flagHighOrLowGate = LOWER_GATE;
      }
    }
  }

#endif
