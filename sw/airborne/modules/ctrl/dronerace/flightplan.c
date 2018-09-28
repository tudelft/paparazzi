
#include "flightplan.h"
#include "filter.h"
#include "ransac.h"
#include "std.h"

struct dronerace_fp_struct dr_fp;

struct JungleGate jungleGate;
void checkJungleGate();
int flagHighOrLowGate;

// X, Y, ALT, PSI
/*
#define MAX_GATES 1
const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    {0.0, 0.0, 1.5, RadOfDeg(0)},
};
*/


const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    {4.0, 0.0, 1.0, RadOfDeg(0), REGULAR, NO_BRAKE},
    {10.0, 0.0, 1.0, RadOfDeg(0), REGULAR, BRAKE},
    {11.5, 5, 1.0, RadOfDeg(90), REGULAR, BRAKE},
    {4.0, 8.0, 1.0, RadOfDeg(180), REGULAR, NO_BRAKE},
    {0.0, 8.0, 1.0, RadOfDeg(180), VIRTUAL, BRAKE},
    {0.0, 5.0, 1.0, RadOfDeg(180), VIRTUAL, BRAKE},
    {4.0, 4.0, 1.0, RadOfDeg(0), JUNGLE, BRAKE}
};


const struct dronerace_flightplan_item_struct waypoints_dr[MAX_GATES] = {
    {6.0, 0.0, 1.0, RadOfDeg(0), REGULAR, NO_BRAKE},
    {10.5, 0.0, 1.0, RadOfDeg(-0), REGULAR, BRAKE},
    {11.5, 6.0, 1.0, RadOfDeg(90), REGULAR, BRAKE},
    {3.0, 8.0, 1.0, RadOfDeg(180), REGULAR, NO_BRAKE},
    {0.0, 8.0, 1.0, RadOfDeg(180), VIRTUAL, BRAKE},
    {0.0, 5.0, 1.0, RadOfDeg(180), VIRTUAL, BRAKE},
    {6.0, 4.0, 1.0, RadOfDeg(0), JUNGLE, BRAKE},
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

  resetJungleGate();
}


#define DISTANCE_GATE_NOT_IN_SIGHT  1.5f
#define DISTANCE_ARRIVED_AT_WP    1.0f

void flightplan_run(void)
{
  float dist = 0.0;

  // Get current gate position
  update_gate_setpoints();

  dr_fp.x_set = waypoints_dr[dr_fp.gate_nr].x;
  dr_fp.y_set = waypoints_dr[dr_fp.gate_nr].y;
  dr_fp.alt_set = dr_fp.gate_alt;

  checkJungleGate();

  // Estimate distance to the gate
  float correctedX,correctedY;
  correctedX = dr_state.x+dr_ransac.corr_x;
  correctedY = dr_state.y+dr_ransac.corr_y;
  dist = (waypoints_dr[dr_fp.gate_nr].x - correctedX)*(waypoints_dr[dr_fp.gate_nr].x- correctedX) + (waypoints_dr[dr_fp.gate_nr].y- correctedY)*(waypoints_dr[dr_fp.gate_nr].y - correctedY);
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


#define MAX_TIME_JUNGLE_GATE_DETECTION 1.0
void checkJungleGate()
{
  // get the time when enter jungle gate logic
  if(gates[dr_fp.gate_nr].type == JUNGLE && jungleGate.flagInJungleGate == false)
  {
    jungleGate.flagInJungleGate = 1;
    jungleGate.timeStartJungleGate = dr_state.time; // TODO: this will compile but don't know if it is correct
  }


  // if there is no detection within 1s, it is likely to be a low gate
    /*
  if((mav::getCurrentTimeMillis() - jungleGate.timeStartJungleGate)/1000.0 > MAX_TIME_JUNGLE_GATE_DETECTION && jungleGate.flagJungleGateDetected == false)
  {
    jungleGate.flagJungleGateDetected = 1;
    flagHighOrLowGate = LOWER_GATE;
  }
     */


  // When determine the gate is in high or low position, send controller desired altitude
  if(gates[dr_fp.gate_nr].type == JUNGLE && jungleGate.flagJungleGateDetected == 1)
  {
    if(flagHighOrLowGate == UPPER_GATE)
      dr_fp.alt_set = -1.6;
    else
      dr_fp.alt_set = -0.6;
  }

}


void resetJungleGate()
{
  jungleGate.flagJungleGateDetected = false;
  jungleGate.numJungleGateDetection = 0;
  jungleGate.jungleGateHeight = 0;
  jungleGate.sumJungleGateHeight = 0;
  jungleGate.flagInJungleGate = false;
}
