
#include "std.h"


#include "control.h"

#include "flightplan.h"
#include "ransac.h"
#include <math.h>
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "bangbang.h"
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
#define CTRL_MAX_PITCH  RadOfDeg(20)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(30)    // rad
#define CTRL_MAX_R      RadOfDeg(90)    // rad/sec

float bound_angle(float angle, float max_angle){
  if(angle>max_angle){
    angle=max_angle;
  }
  if(angle<(-max_angle)){
    angle=-max_angle;
  }
  return angle;
}

void control_reset(void)
{ 
  POS_I = 0.0; 
  lookI = 0.0;
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
#define KP_look 0.1
#define KI_look 0.0 
#define KP_POS  0.2
#define KI_POS 0.02
#define KP_VEL_X  0.2
#define KP_VEL_Y  0.2 
#define KD_VEL_X  0.05
#define KD_VEL_Y  0.05
#define radius_des 2
float lookahead = 25 * PI/180.0;
#define PITCHFIX  -10 * PI/180.0
#define DIRECTION 1 // 1 for clockwise, -1 for counterclockwise

void control_run(float dt)
{

  float psi_meas=stateGetNedToBodyEulers_f()->psi;
  float theta_meas=stateGetNedToBodyEulers_f()->theta;
  float phi_meas=stateGetNedToBodyEulers_f()->phi;


  dr_control.z_cmd = dr_fp.z_set;

  // outer loop velocity control

  struct NedCoor_f *pos_gps = stateGetPositionNed_f();
  dr_state.x = pos_gps->x;
  dr_state.y = pos_gps->y;
  dr_state.z = pos_gps->z;

   struct NedCoor_f *vel_gps = stateGetSpeedNed_f();
   float vxE = vel_gps->x; // In earth reference frame
   float vyE = vel_gps->y;
   float vzE = vel_gps->z;

// transform to body reference frame 

  float cthet=cosf(theta_meas);
  float sthet=sinf(theta_meas);
  float cphi= cosf(phi_meas);
  float sphi = sinf(phi_meas);
  float cpsi = cosf(psi_meas);
  float spsi = sinf(psi_meas);
  optimize(4,3,0);
  // printf("bang_ctrl: %f %f %f\n",bang_ctrl[0],bang_ctrl[1],bang_ctrl[2]);

  dr_state.vx = (cthet*cpsi)*vxE + (cthet*spsi)*vyE - sthet*vzE;
  dr_state.vy = (sphi*sthet*cpsi-cphi*spsi)*vxE + (sphi*sthet*spsi+cphi*cpsi)*vyE + (sphi*cthet)*vzE;
  float posxVel = (cthet*cpsi)*dr_state.x + (cthet*spsi)*dr_state.y - sthet*dr_state.z;
  float posyVel = (sphi*sthet*cpsi-cphi*spsi)*dr_state.x + (sphi*sthet*spsi+cphi*cpsi)*dr_state.y + (sphi*cthet)*dr_state.z;

  dr_control.psi_cmd = 0;// angle180(psi_cmd*180.0/PI)*PI/180.0;
  dr_control.phi_cmd = 0;
  dr_control.theta_cmd = 0;

  static int counter = 0;
  // fprintf(file_logger_t, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f, %f, %f, %f, %f, %f, %f, %f, %f\n", get_sys_time_float(), dr_control.theta_cmd, dr_control.phi_cmd, theta_meas,phi_meas,dr_state.x,dr_state.y, dist2target, phase_angle,rxb,ryb,centriterm, 
  // dr_control.psi_cmd,psi_meas,dr_state.vx,dr_state.vy,rx,ry,radiuserror,ang1,ang2,ang);
  // counter++;
  

}
