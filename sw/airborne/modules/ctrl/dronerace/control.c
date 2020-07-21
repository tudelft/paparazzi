
#include "std.h"


#include "control.h"

#include "flightplan_Bang.h"
#include "ransac.h"
#include <math.h>
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "bangbang.h"


#define r2d 180./M_PI
#define d2r M_PI/180.0
// Variables
struct dronerace_control_struct dr_control;

/** The file pointer */
FILE *file_logger_t = NULL;
FILE *bang_bang_t = NULL;

static void open_log(void) 
{
  char filename[512];
  char filename2[512];
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "pid_outer_log_file");
  sprintf(filename2, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "bangbang_log");
  printf("\n\n*** chosen filename log drone race: %s ***\n\n", filename);
  file_logger_t = fopen(filename, "w+"); 
  bang_bang_t = fopen(filename2,"w+");
  fprintf(bang_bang_t,"time,satdim, brake, t_s, t_target, error_x, error_y, posx, posy, vxvel, vyvel, c1_sat,c2_sat, c1_sat_brake, c2_sat_brake, c1_sec, c2_sec\n");
  fprintf(file_logger_t,"time, gate_nr, gate_type, controller_type\n");
}


// Settings


// Slow speed
#define CTRL_MAX_SPEED  3.0             // m/s
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
  #ifdef LOG
  open_log();
  #endif
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
  flightplan_run();
  float psi_meas=stateGetNedToBodyEulers_f()->psi;
  float theta_meas=stateGetNedToBodyEulers_f()->theta;
  float phi_meas=stateGetNedToBodyEulers_f()->phi;

  dr_state.phi=phi_meas;
  dr_state.theta=theta_meas;
  dr_state.psi=psi_meas;
  
  dr_control.z_cmd = dr_bang.gate_z;

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

  float cthet=cosf(dr_control.theta_cmd);
  float sthet=sinf(dr_control.theta_cmd);
  float cphi = cosf(dr_control.phi_cmd);
  float sphi = sinf(dr_control.phi_cmd);
  float cpsi = cosf(dr_control.psi_cmd);
  float spsi = sinf(dr_control.psi_cmd);
  
  
  dr_state.vx = (cthet*cpsi)*vxE + (cthet*spsi)*vyE ;//- sthet*vzE;
  dr_state.vy = (sphi*sthet*cpsi-cphi*spsi)*vxE + (sphi*sthet*spsi+cphi*cpsi)*vyE ;//+ (sphi*cthet)*vzE;
  // float posxVel = (cthet*cpsi)*dr_state.x + (cthet*spsi)*dr_state.y - sthet*dr_state.z;
  // float posyVel = (sphi*sthet*cpsi-cphi*spsi)*dr_state.x + (sphi*sthet*spsi+cphi*cpsi)*dr_state.y + (sphi*cthet)*dr_state.z;
  float posx_cmd =  dr_bang.gate_x; //position command in earth reference frame
  float posy_cmd =  dr_bang.gate_y;
  float vx_cmd =0;
  float vy_cmd =0;
  float error_posx_E=posx_cmd-dr_state.x; //error in earth frame
  float error_posy_E=posy_cmd-dr_state.y;
  float error_posx_vel =(cthet*cpsi)*error_posx_E+(cthet*spsi)*error_posy_E;//error in velocity frame
  float error_posy_vel = (sphi*sthet*cpsi-cphi*spsi)*error_posx_E+(sphi*sthet*spsi+cphi*cpsi)*error_posy_E;
  float dist2target = sqrtf(error_posx_vel*error_posx_vel+error_posy_vel*error_posy_vel);

  if(dr_bang.controller_type==BANGBANG)
  {
    optimizeBangBang(error_posx_vel,error_posy_vel,0.2); // function writes to bang_ctrl 

    if(abs(error_posx_vel)>1){ //freeze yaw cmd when it gets close to wp
      dr_control.psi_cmd =atan2f(error_posy_vel,error_posx_vel);// angle180(psi_cmd*180.0/PI)*PI/180.0;
    }
    dr_control.phi_cmd =0;// bang_ctrl[1];
    dr_control.theta_cmd = bang_ctrl[0];
  }
  else{ // USE a PID controller if not BANGBANG
        
      dr_state.vx = (cthet*cpsi)*vxE + (cthet*spsi)*vyE - sthet*vzE;
      dr_state.vy = (sphi*sthet*cpsi-cphi*spsi)*vxE + (sphi*sthet*spsi+cphi*cpsi)*vyE + (sphi*cthet)*vzE;
      float posxVel = (cthet*cpsi)*dr_state.x + (cthet*spsi)*dr_state.y - sthet*dr_state.z;
      float posyVel = (sphi*sthet*cpsi-cphi*spsi)*dr_state.x + (sphi*sthet*spsi+cphi*cpsi)*dr_state.y + (sphi*cthet)*dr_state.z;

      float vx_des = bound_angle(error_posx_vel,2);
      float vy_des = bound_angle(error_posy_vel,2);
      if(abs(error_posx_vel)>1){ //freeze yaw cmd when it gets close to wp
        dr_control.psi_cmd =atan2(error_posy_E,error_posx_E);// angle180(psi_cmd*180.0/PI)*PI/180.0;
      }
      else{
        dr_control.psi_cmd=dr_bang.gate_psi;
      }

      printf("psicmd: %f,atan: %f, error_posx: %f, error_posy: %f, error_posx_vel: %f\n",dr_control.psi_cmd,(error_posy_E,error_posx_E),error_posx_E,error_posy_E, error_posx_vel);
      dr_control.phi_cmd= bound_angle(KP_VEL_Y * (vy_des-dr_state.vy),CTRL_MAX_ROLL);
      dr_control.theta_cmd=bound_angle(KP_VEL_X *-1* (vx_des-dr_state.vx),CTRL_MAX_PITCH); 
      #ifdef LOG
      fprintf(file_logger_t,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",get_sys_time_float(),posx_cmd,posy_cmd,dr_state.x,dr_state.y,error_posx_vel,error_posy_vel,vx_cmd,vy_cmd,dr_state.vx,dr_state.vy,dr_control.phi_cmd,dr_control.theta_cmd,posxVel,posyVel);
      #endif
  } 

  // dr_control.psi_cmd=0;
  // dr_control.phi_cmd=0;
  // dr_control.theta_cmd=0;
  
 

}
