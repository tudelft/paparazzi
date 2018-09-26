
#include "flightplan.h"
#include "filter.h"
#include "ransac.h"
#include "std.h"

struct dronerace_fp_struct dr_fp;

// X, Y, ALT, PSI
/*
#define MAX_GATES 1
const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    {0.0, 0.0, 1.5, RadOfDeg(0)},
};
*/


const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    {4.0, 0.0, -1.5, RadOfDeg(0)},
    {10.0, 0, -1.5, RadOfDeg(0)},
    {15, 0, -1.5, RadOfDeg(0)},
    {0.0, 0.0, 1.5, RadOfDeg(-270)},
};


const struct dronerace_flightplan_item_struct waypoints[MAX_GATES] = {
        {5.0, 0.0, -1.5, RadOfDeg(0)},
        {11.0, 0.0, -1.5, RadOfDeg(-0)},
        {16, 0.0, -1.5, RadOfDeg(-225)},
        {0.0, 0.0, 1.5, RadOfDeg(-270)},
};

static void update_gate_setpoints(void)
{
  dr_fp.gate_x   = gates[dr_fp.gate_nr].x;
  dr_fp.gate_y   = gates[dr_fp.gate_nr].y;
  dr_fp.gate_alt = gates[dr_fp.gate_nr].alt;
  dr_fp.gate_psi = gates[dr_fp.gate_nr].psi;
}



void flightplan_reset()
{
  // Current Gate
  dr_fp.gate_nr = 0;
  update_gate_setpoints();

  // Navigation Setpoint
  dr_fp.x_set = 3;
  dr_fp.y_set = 0;
  dr_fp.alt_set = 0;
  dr_fp.psi_set = 0;
}


#define DISTANCE_GATE_NOT_IN_SIGHT  1.5f
#define DISTANCE_ARRIVED_AT_WP    1.0f

void flightplan_run(void)
{
  float dist = 0.0;

  // Get current gate position
  update_gate_setpoints();

  dr_fp.x_set = waypoints[dr_fp.gate_nr].x;
  dr_fp.y_set = waypoints[dr_fp.gate_nr].y;
  dr_fp.alt_set = dr_fp.gate_alt;

  // Estimate distance to the gate
  float correctedX,correctedY;
  correctedX = dr_state.x+dr_ransac.corr_x;
  correctedY = dr_state.y+dr_ransac.corr_y;
  dist = (waypoints[dr_fp.gate_nr].x - correctedX)*(waypoints[dr_fp.gate_nr].x- correctedX) + (waypoints[dr_fp.gate_nr].y- correctedY)*(waypoints[dr_fp.gate_nr].y - correctedY);
  // Align with current gate
  dr_fp.psi_set = dr_fp.gate_psi;

  float dist_2_gate =  (dr_fp.gate_x - correctedX)*(dr_fp.gate_x - correctedX) + (dr_fp.gate_y - correctedY)*(dr_fp.gate_y - correctedY);

  // If too close to the gate to see the gate, heading to next gate
  if (dist_2_gate < DISTANCE_GATE_NOT_IN_SIGHT * DISTANCE_GATE_NOT_IN_SIGHT)
  {
    if ((dr_fp.gate_nr+1) < MAX_GATES)
    {
      dr_fp.psi_set = gates[dr_fp.gate_nr+1].psi;
    }
  }


  // If close to desired position, switch to next
  if (dist < DISTANCE_ARRIVED_AT_WP * DISTANCE_ARRIVED_AT_WP)
  {
    dr_fp.gate_nr ++;
    if (dr_fp.gate_nr >= MAX_GATES)
    {
      dr_fp.gate_nr = (MAX_GATES -1);
    }
  }
}
