#include "modules/divergence_stop/divergence_stop.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>
#include "mcu_periph/sys_time.h"

#ifndef DIVERGENCE_STOP_THRESHOLD
#error Error! divergence_stop threshold not defined
#endif

#ifndef DIVERGENCE_STOP_AVOIDANCE_PITCH
#error Error! divergence_stop avoidance pitch not defined
#endif

#ifndef DIVERGENCE_STOP_AVOIDANCE_DURATION
#error Error! divergence_stop avoidance duration not defined
#endif




#ifndef OFH_OPTICAL_FLOW_ID
#define OFH_OPTICAL_FLOW_ID ABI_BROADCAST
#endif

#define PRINT(string,...) fprintf(stderr, "[divergence_stop->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "autopilot.h"

// Own Variables
float divergence_stop_threshold;
float divergence_stop_avoidance_pitch;
float divergence_stop_avoidance_duration;

float size_divergence;

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;

static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y,
                         int16_t flow_der_x, int16_t flow_der_y, float quality, float size_div)
{
  size_divergence = size_div;
}

void divergence_stop_init(void)
{
  // Initialize by flying forward
  last_time_exceeded =get_sys_time_float() - 99;
  divergence_stop_threshold = DIVERGENCE_STOP_THRESHOLD;
  divergence_stop_avoidance_pitch = DIVERGENCE_STOP_AVOIDANCE_PITCH;
  divergence_stop_avoidance_duration = DIVERGENCE_STOP_AVOIDANCE_DURATION;

  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(OFH_OPTICAL_FLOW_ID, &optical_flow_ev, optical_flow_cb);

}

void divergence_stop_periodic(void)
{
  if (size_divergence > divergence_stop_threshold) {
          last_time_exceeded = get_sys_time_float();
	  PRINT("THRESHOLD EXCEEDED\n");
  }
  return;
}

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
}

void guidance_h_module_enter(void)
{
  // Store current heading
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

void guidance_h_module_read_rc(void)
{
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}


void guidance_h_module_run(bool in_flight)
{
  float pitch = RadOfDeg(0.);
  if (get_sys_time_float() - last_time_exceeded < divergence_stop_avoidance_duration) {
      pitch = RadOfDeg(divergence_stop_avoidance_pitch);
  }

  ctrl.cmd.phi = ctrl.rc_sp.phi;
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch) + ctrl.rc_sp.theta;

  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}
