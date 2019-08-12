
#include "std.h"


#include "control.h"
#include "filter.h"
#include "flightplan.h"
#include "ransac.h"
#include <math.h>

// Variables
struct dronerace_control_struct dr_control;

// Settings


// Slow speed
#define CTRL_MAX_SPEED  6.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(28)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(28)    // rad
#define CTRL_MAX_R      RadOfDeg(45)    // rad/sec


void control_reset(void)
{
  // Reset flight plan logic
  flightplan_reset();

  // Reset own variables
  dr_control.psi_ref = 0;
  dr_control.psi_cmd = 0;
}

static float angle180(float r)
{
  if (r < RadOfDeg(-180))
  {
    r += RadOfDeg(360.0f);
  }
  else if (r > RadOfDeg(180.0f))
  {
    r -= RadOfDeg(360.0f);
  }

  return r;
}

void control_run(float dt)
{
  /*
  float psi, vxcmd, vycmd, r_cmd, ax, ay;
  // Propagate the flightplan
  flightplan_run();

  // Variables
  psi = dr_state.psi;

  // Heading controller
  r_cmd = dr_fp.psi_set - dr_control.psi_ref;

  // Find shortest turn
  r_cmd = angle180(r_cmd);

  // Apply rate limit
  Bound(r_cmd, -CTRL_MAX_R, CTRL_MAX_R);
  dr_control.psi_ref += r_cmd * dt;

  // Position error to Speed
  // TODO: interestingly, we don't use the velocity correction for control: t_fit * dr_ransac.corr_vx
  if(gates[dr_fp.gate_nr].psi == RadOfDeg(0.0) || gates[dr_fp.gate_nr].psi == RadOfDeg(180))
  {
    vxcmd = (dr_fp.x_set - (dr_state.x)) * P_FORWARD - dr_state.vx * D_FORWARD;
    vycmd = (dr_fp.y_set - (dr_state.y)) * P_LATERAL - dr_state.vy * D_LATERAL;
  }
  else
  {
    vxcmd = (dr_fp.x_set - (dr_state.x )) * P_LATERAL- dr_state.vx * D_LATERAL;
    vycmd = (dr_fp.y_set - (dr_state.y)) * P_FORWARD- dr_state.vy * D_FORWARD;
  }

  if(!waypoints_dr[dr_fp.gate_nr].brake) {
      vxcmd += 10.0f * cosf(waypoints_dr[dr_fp.gate_nr].psi);
      vycmd += 10.0f * sinf(waypoints_dr[dr_fp.gate_nr].psi);
  }

  Bound(vxcmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);
  Bound(vycmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);

  vxcmd *= dr_fp.gate_speed;
  vycmd *= dr_fp.gate_speed;

  // Speed to Attitude
  ax = (vxcmd - dr_state.vx) * 1.0f + vxcmd * RadOfDeg(10.0f) / 3.0f;
  ay = (vycmd - dr_state.vy) * 1.0f + vycmd * RadOfDeg(10.0f) / 3.0f;

  Bound(ax, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);
  Bound(ay, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);

  dr_control.phi_cmd   = -sinf(psi) * ax + cosf(psi) * ay;
  dr_control.theta_cmd = -cosf(psi) * ax - sinf(psi) * ay;
  dr_control.psi_cmd   = dr_control.psi_ref;*/
  dr_control.z_cmd   = dr_fp.z_set;

//-------------------------------------------------------
  	static double old_error_pos_x_velFrame = 0;
	static double old_error_pos_y_velFrame = 0;
  static double desired_vx_gate=0;

	double curr_error_pos_w_x = dr_fp.x_set - dr_state.x;
	double curr_error_pos_w_y = dr_fp.y_set - dr_state.y;

	double curr_error_pos_x_velFrame =  cos(dr_state.psi)*curr_error_pos_w_x + sin(dr_state.psi)*curr_error_pos_w_y;
	double curr_error_pos_y_velFrame = -sin(dr_state.psi)*curr_error_pos_w_x + cos(dr_state.psi)*curr_error_pos_w_y;

	double diff_pos_x_velframe = curr_error_pos_x_velFrame - old_error_pos_x_velFrame;
	double diff_pos_y_velframe = curr_error_pos_y_velFrame - old_error_pos_y_velFrame;

	double vel_x_cmd_velFrame = curr_error_pos_x_velFrame * kp_pos_x + diff_pos_x_velframe * dt * kd_pos_x;
	double vel_y_cmd_velFrame = curr_error_pos_y_velFrame * kp_pos_y + diff_pos_y_velframe * dt * kd_pos_y;

//    double vel_x_sp_gate_velFrame =  cos(dr_state.psi) * desired_vx_gate + sin(dr_state.psi) * desired_vy_gate;
//    double vel_y_sp_gate_velFrame = -sin(dr_state.psi) * desired_vx_gate + cos(dr_state.psi) * desired_vy_gate;

    vel_x_cmd_velFrame += desired_vx_gate;
//    vel_y_cmd_velFrame += vel_y_sp_gate_velFrame;

    // Actual speed
	double vel_x_est_velFrame =  cos(dr_state.psi) * dr_state.vx + sin(dr_state.psi) * dr_state.vy;
	double vel_y_est_velFrame = -sin(dr_state.psi) * dr_state.vx + cos(dr_state.psi) * dr_state.vy;
  Bound(vel_x_cmd_velFrame, -MAX_VEL_X, MAX_VEL_X);
	Bound(vel_y_cmd_velFrame, -MAX_VEL_Y, MAX_VEL_Y);

	double curr_error_vel_x = vel_x_cmd_velFrame - vel_x_est_velFrame;
	double curr_error_vel_y = vel_y_cmd_velFrame - vel_y_est_velFrame;

	dr_control.theta_cmd =   curr_error_vel_x * KP_VEL_X + K_FF_THETA * (vel_x_cmd_velFrame); 
	dr_control.phi_cmd  = -(curr_error_vel_y * KP_VEL_Y + K_FF_PHI   * (vel_y_cmd_velFrame));

    old_error_pos_x_velFrame = curr_error_pos_x_velFrame;
    old_error_pos_y_velFrame = curr_error_pos_y_velFrame;




}

