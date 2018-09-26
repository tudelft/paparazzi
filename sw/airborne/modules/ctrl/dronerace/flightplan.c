
#include "flightplan.h"
#include "filter.h"

#include "std.h"

struct dronerace_fp_struct dr_fp;

struct dronerace_flightplan_item_struct
{
	float x;
	float y;
	float alt;
	float psi;
};

// X, Y, ALT, PSI
/*
#define MAX_GATES 1
const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    {0.0, 0.0, 1.5, RadOfDeg(0)},
};
*/

#define MAX_GATES 4
const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    {4.0, 0.0, 1.5, RadOfDeg(0)},
    {4.0, -2.0, 1.5, RadOfDeg(90)},
    {0.0, -2.0, 1.5, RadOfDeg(0)},
    {0.0, 0.0, 1.5, RadOfDeg(0)},
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


void flightplan_run(void)
{
  float dist = 0.0;

  // Get current gate position
  update_gate_setpoints();

  dr_fp.x_set = dr_fp.gate_x;
  dr_fp.y_set = dr_fp.gate_y;
  dr_fp.alt_set = dr_fp.gate_alt;

  // Estimate distance to the gate
  dist = (dr_fp.gate_x - dr_state.x)*(dr_fp.gate_x - dr_state.x) + (dr_fp.gate_y - dr_state.y)*(dr_fp.gate_y - dr_state.y);

  // Align with current gate
  dr_fp.psi_set = dr_fp.gate_psi;

  // If too close to the gate to see the gate, heading to next gate
  if (dist < 1.4 * 1.4)
  {
    if ((dr_fp.gate_nr+1) < MAX_GATES)
    {
      dr_fp.psi_set = gates[dr_fp.gate_nr+1].psi;
    }
  }

  // If close to desired position, switch to next
  if (dist < 0.5 * 0.5)
  {
    dr_fp.gate_nr ++;
    if (dr_fp.gate_nr >= MAX_GATES)
    {
      dr_fp.gate_nr = (MAX_GATES -1);
    }
  }

}
