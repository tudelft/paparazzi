
#include "std.h"


#include "control.h"

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
#define CTRL_MAX_ROLL   RadOfDeg(5)    // rad
#define CTRL_MAX_R      RadOfDeg(45)    // rad/sec

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

#define KP_POS    0.08
#define KI_POS 0.00
#define KP_VEL_X  0.1
#define KP_VEL_Y  0.1
#define KD_VEL_X  0.05
#define KD_VEL_Y  0.05
#define radius_des 0.5 
#define lookahead  0 * PI/180.0
#define PITCHFIX  0 * PI/180.0
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

  float dist2target = sqrtf(dr_state.x * dr_state.x + dr_state.y*dr_state.y);
  float absvel = (dr_state.vx * dr_state.vx + dr_state.vy*dr_state.vy);
  float radiuserror = (dist2target-radius_des);

  float phase_angle = atan2f(dr_state.y,dr_state.x); 
  float r_error_x=radiuserror * sinf(phase_angle);
  float r_error_y=radiuserror * cosf(phase_angle);

  float rx = r_error_x;// - dr_state.x; //translate error to body 
  float ry = r_error_y;// - dr_state.y;

  //rotate error to body
  float rxb = rx*cosf(theta_meas)*cosf(psi_meas)+ry*cosf(theta_meas)*sinf(psi_meas);
  float ryb = rx*(sinf(phi_meas)*sinf(theta_meas)*cosf(psi_meas)-cosf(phi_meas)*sinf(psi_meas)) + ry*(sinf(phi_meas)*sinf(theta_meas)*sinf(psi_meas)+cosf(phi_meas)*cosf(psi_meas));
  radiuserror = ryb;

  dr_control.phi_cmd =KP_POS* radiuserror + POS_I*KI_POS + DIRECTION*atan2f(absvel * cosf(dr_state.theta), (abs(GRAVITY) * radius_des)); //fix sign for direction of circle 
  dr_control.phi_cmd = 0;//bound_angle(dr_control.phi_cmd,CTRL_MAX_ROLL);
//   if(dr_control.phi_cmd>CTRL_MAX_ROLL){
//     dr_control.phi_cmd = CTRL_MAX_ROLL;
//   }

// if(dr_control.phi_cmd<(-1*(CTRL_MAX_ROLL))){
//   dr_control.phi_cmd = -CTRL_MAX_ROLL;
// }
  POS_I = POS_I + radiuserror*dt; 


  float psi_cmd = phase_angle +(DIRECTION*0.5*PI) + lookahead;    
  dr_control.psi_cmd = 0;//angle180(0*180.0/PI)*PI/180.0;

  dr_control.theta_cmd = PITCHFIX ;
  dr_control.theta_cmd = bound_angle(dr_control.theta_cmd,CTRL_MAX_PITCH);

  printf("posx: %f, posy: %f, PosR: %f, phaseangle: %f, yaw_angle: %f, yaw_cmd: %f, radius error: %f\n",dr_state.x, dr_state.y, dist2target,phase_angle*180./PI,psi_meas*180/PI,dr_control.psi_cmd*180./PI,radiuserror);

  static int counter = 0;
  fprintf(file_logger_t, "%d,%f,%f,%f,%f,%f,%f\n", counter, dr_control.theta_cmd, dr_control.phi_cmd, dr_state.theta, dr_state.phi, dist2target, phase_angle);
  counter++;
  

}
