
#include "std.h"


#include "control.h"
#include "filter.h"
#include "flightplan.h"
#include "ransac.h"
#include <math.h>
#include "state.h"

// Variables
struct dronerace_control_struct dr_control;

/** The file pointer */
FILE *file_logger_t = NULL;

static void open_log(void) 
{
  char filename[512];
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "lllllog_file");
  printf("\n\n*** chosen filename log drone race: %s ***\n\n", filename);
  file_logger_t = fopen(filename, "w+"); 
}


// Settings


// Slow speed
#define CTRL_MAX_SPEED  6.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(18)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(18)    // rad
#define CTRL_MAX_R      RadOfDeg(45)    // rad/sec


void control_reset(void)
{ 
  POSI = 0;
  // Reset flight plan logic
  flightplan_reset();
  open_log();
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


float bound_f(float val, float min, float max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}

#define KP_POS    0.05
#define KI_POS 0.001
#define KP_VEL_X  0.1
#define KP_VEL_Y  0.1
#define KD_VEL_X  0.05
#define KD_VEL_Y  0.05
#define radius_des 4.0 
#define lookahead  10 * PI/180.0


void control_run(float dt)
{

  dr_control.z_cmd = dr_fp.z_set;
  float Vx_velframe; 

  // outer loop velocity control
	static float prev_acc_x_error = 0;
	static float prev_acc_y_error = 0;

  struct NedCoor_f *pos_gps = stateGetPositionNed_f();
  dr_state.x = pos_gps->x;
  dr_state.y = pos_gps->y;
  dr_state.z = pos_gps->z;

	// float vel_cmd_w_x = KP_POS * (0 - dr_state.x)/dt;
	// float vel_cmd_w_y = KP_POS * (0 - dr_state.y)/dt;
  float dist2target = sqrtf(dr_state.x * dr_state.x + dr_state.y*dr_state.y);
  float radiuserror = (radius_des - dist2target);
  dr_control.phi_cmd   = KP_POS* radiuserror + POSI*KI_POS; 
  POSI = POSI + radiuserror*dt;

  float phase_angle = atan2f(dr_state.x,dr_state.y); 

  float psi_cmd = phase_angle + 0.5*PI + lookahead;
  dr_control.psi_cmd = angle180(psi_cmd*180.0/PI)*PI/180.0;


// //	vel_cmd_w_x -= 5 * cos(flightplan->wp.psi[flightplan->wp_selector]);
// //	vel_cmd_w_y -= 5 * sin(flightplan->wp.psi[flightplan->wp_selector]);

// 	vel_cmd_w_x = bound_f(vel_cmd_w_x, -2, 2);
// 	vel_cmd_w_y = bound_f(vel_cmd_w_y, -2, 2);

// //	double vel_x_cmd_velFrame =  cos(this->dr_state.yaw) * vel_cmd_w_x + sin(this->dr_state.yaw) * vel_cmd_w_y;
// //	double vel_y_cmd_velFrame = -sin(this->dr_state.yaw) * vel_cmd_w_x + cos(this->dr_state.yaw) * vel_cmd_w_y;

// 	float acc_x_cmd_bodyFrame =  cosf(dr_state.psi) * (vel_cmd_w_x - dr_state.vx) + sinf(dr_state.psi) * (vel_cmd_w_y - dr_state.vy);
// 	float acc_y_cmd_bodyFrame = -sinf(dr_state.psi) * (vel_cmd_w_x - dr_state.vx) + cosf(dr_state.psi) * (vel_cmd_w_y - dr_state.vy);

// 	float d_acc_x_bodyFrame = (acc_x_cmd_bodyFrame - prev_acc_x_error) / dt;
// 	float d_acc_y_bodyFrame = (acc_y_cmd_bodyFrame - prev_acc_y_error) / dt;

	// dr_control.theta_cmd = 0;//-(KP_VEL_X * acc_x_cmd_bodyFrame - KD_VEL_X * d_acc_x_bodyFrame); // + K_FF_THETA * vel_x_cmd_velFrame);
  // //(KP_VEL_Y * acc_y_cmd_bodyFrame - KD_VEL_Y * d_acc_y_bodyFrame);  // + K_FF_PHI   * vel_y_cmd_velFrame;

	// prev_acc_x_error = acc_x_cmd_bodyFrame;
	// prev_acc_y_error = acc_y_cmd_bodyFrame;


  static int counter = 0;
  fprintf(file_logger_t, "%d,%f,%f,%f,%f,%f,%f,%f,%f\n", counter, dr_control.theta_cmd, dr_control.phi_cmd, dr_state.theta, dr_state.phi, vel_cmd_w_x, vel_cmd_w_y, dr_state.vx, dr_state.vy);
  counter++;
  

}
