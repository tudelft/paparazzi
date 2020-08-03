
#include "std.h"


#include "control.h"

#include "flightplan_Bang.h"
#include "ransac.h"
#include <math.h>
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "bangbang.h"
#include "filter.h"
#define LOG

#define r2d 180./M_PI
#define d2r M_PI/180.0
// Variables
struct dronerace_control_struct dr_control;

/** The file pointer */
FILE *file_logger_t = NULL;
FILE *bang_bang_t = NULL;
FILE *fp_logger_t = NULL;
FILE *brake_log_t = NULL;
FILE *filter_log_t = NULL;
FILE *comp_log_t = NULL;
static void open_log(void) 
{
  char filename[512];
  char filename2[512];
  char filename3[512];
  char filename4[512];
  
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "state_log");
  sprintf(filename2, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "bangbang_log");
  sprintf(filename3, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "flightplan_log");
  sprintf(filename4, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "filter_log");
  sprintf(filename5, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "comp_measure_log");
  // sprintf(filename4, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "brake_log");
  
  file_logger_t = fopen(filename, "w+"); 
  bang_bang_t = fopen(filename2,"w+");
  fp_logger_t=fopen(filename3,"w+"); 
  comp_log_t=fopen(filename5,"w+");
  filter_log_t=fopen(filename4,"w+");
  

  // brake_log_t=fopen(filename4,"w+");
  // fprintf(comp_measure_log_t,"satdim, v0, ang_0, ang_1, delta_t_meas, delta_y_meas, delta_v_meas\n");
  // fprintf(comp_measure_log_t,"time, gps_x, gps_y, gps_z, gps_vx, gps_vy, gps_vz, az, abx, aby, ax, ay\n");
  fprintf(bang_bang_t,"time, satdim, brake, t_s, t_target, pos_error_vel_x, pos_error_vel_y, dr_state.x, dr_state.y, v0[0], v0[1], constant_sat_accel.c1, constant_sat_accel.c2, constant_sat_brake.c1, constant_sat_brake.c2, constant_sec.c1, constant_sec.c2, T_sat, T_sec, apply_compensation, in_transition, delta_t, delta_y, delta_v, ys, vs\n");
  fprintf(file_logger_t,"time, dr_state.x, dr_state.y, posxVel, posyVel, dr_state.z, vxE, vyE, vzE, dr_state.vx, dr_state.vy, dr_state.phi, dr_state.theta, dr_state.psi, phi_cmd, theta_cmd, psi_cmd\n");
  fprintf(fp_logger_t,"time, gate_nr, gate_type, controller_type, gate_x, gate_y, gate_z, gate_psi \n");
  fprintf(comp_log_t,"satdim, v0, ang_0, ang_1, delta_t_meas, delta_y_meas, delta_v_meas\n");
  fprintf(filter_log_t,"time, gps_x, gps_y, gps_z, gps_vx, gps_vy, gps_vz, az, abx, aby, ax, ay\n");
 fclose(comp_log_t);
  // fprintf(brake_log_t,"time, y0, v0, c1, c2, ang0, ang1, angc, Epos\n");
}


// Settings

float vx_des_vel ;
float vy_des_vel ;

// Slow speed
#define CTRL_MAX_SPEED  5.0             // m/s
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
  dr_bang.controller_type=PID;
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

  // struct NedCoor_f *pos_gps = stateGetPositionNed_f();
  // float pos_x = pos_gps->x;
  // float pos_y = pos_gps->y;
  // float pos_z = pos_gps->z;

   
   

// transform to body reference frame 

  // float cthet=cosf(dr_control.theta_cmd);
  // float sthet=sinf(dr_control.theta_cmd);
  // float cphi = cosf(dr_control.phi_cmd);
  // float sphi = sinf(dr_control.phi_cmd);
  float cpsi = cosf(dr_control.psi_cmd);
  float spsi = sinf(dr_control.psi_cmd);
    
  // float cthet=cosf(dr_state.theta);
  // float sthet=sinf(dr_state.theta);
  // float cphi = cosf(dr_state.phi);
  // float sphi = sinf(dr_state.phi);
  // float cpsi = cosf(dr_state.psi);
  // float spsi = sinf(dr_state.psi);


  

  float posx_cmd =  dr_bang.gate_x; //position command in earth reference frame
  float posy_cmd =  dr_bang.gate_y;

  float error_posx_E=posx_cmd-dr_state.x; //position error in earth frame
  float error_posy_E=posy_cmd-dr_state.y;
  float error_posx_vel =cpsi*error_posx_E+spsi*error_posy_E;//position error in velocity frame
  float error_posy_vel = -spsi*error_posx_E+cpsi*error_posy_E;

  dist2gate = sqrtf(error_posx_vel*error_posx_vel+error_posy_vel*error_posy_vel);

  float posxVel = cpsi*dr_state.x + spsi*dr_state.y;
  float posyVel = -spsi*dr_state.x + cpsi*dr_state.y ;

  float vx_vel=dr_state.vx*cpsi +dr_state.vy*spsi;
  float vy_vel=-dr_state.vx*spsi + dr_state.vy*cpsi;


  if(dr_bang.controller_type==BANGBANG)
  {
    psi_command=dr_control.psi_cmd; //used in bangbang.c 
    optimizeBangBang(error_posx_vel,error_posy_vel,0.2); // function writes to bang_ctrl 

    if(fabs(error_posx_vel)>1){ //freeze yaw cmd when it gets close to wp
      dr_control.psi_cmd = atan2f(error_posy_E,error_posx_E);// angle180(psi_cmd*180.0/PI)*PI/180.0;
    }
    else{
      dr_control.psi_cmd=dr_bang.gate_psi;
    }
    dr_control.phi_cmd = bang_ctrl[1];
    vy_des_vel = bound_angle(error_posy_vel,CTRL_MAX_SPEED);
    dr_control.phi_cmd= bound_angle(KP_VEL_Y * (vy_des_vel-vy_vel),CTRL_MAX_ROLL);//TODO
    dr_control.theta_cmd = bang_ctrl[0];
  }
  else{ // USE a PID controller if not BANGBANG
        
      
       vx_des_vel = bound_angle(error_posx_vel,CTRL_MAX_SPEED); //saturate to max velocity
       vy_des_vel = bound_angle(error_posy_vel,CTRL_MAX_SPEED);
      
      if(fabs(error_posx_vel)>1){ //freeze yaw cmd when it gets close to wp
        dr_control.psi_cmd =atan2f(error_posy_E,error_posx_E); // yaw towards gate when distance is large enough. 
      }
      else{
        dr_control.psi_cmd=dr_bang.gate_psi;
      }
      if(dr_bang.turning==TURNING){
        dr_control.psi_cmd=dr_bang.gate_psi;
      }
      // printf("psicmd: %f,atan: %f, error_posx: %f, error_posy: %f, error_posx_vel: %f\n",dr_control.psi_cmd,(error_posy_E,error_posx_E),error_posx_E,error_posy_E, error_posx_vel);
      dr_control.phi_cmd= bound_angle(KP_VEL_Y * (vy_des_vel-vy_vel),CTRL_MAX_ROLL);
      dr_control.theta_cmd=bound_angle(KP_VEL_X *-1* (vx_des_vel-vx_vel),CTRL_MAX_PITCH); 
      
  } 
    if (dr_bang.overwrite_psi){
      dr_control.psi_cmd=dr_bang.psi_forced; //
    }

    // printf("psi_cmd: %f, psi_forced %d\n",dr_control.psi_cmd,dr_bang.psi_forced);
  
    // if(dr_bang.controller_type==BANGBANG){
    //   dr_control.theta_cmd=bang_ctrl[0]; //TODO for now only pitch can be affected by bangbang
    // }
  // dr_control.psi_cmd=0;
  // dr_control.phi_cmd=0;
  // dr_control.theta_cmd=0;
  
  #ifdef LOG
      fprintf(file_logger_t,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",get_sys_time_float(),dr_state.x,dr_state.y,posxVel,posyVel,dr_state.z,vxE,vyE,vzE,dr_state.vx,dr_state.vy,dr_state.phi,dr_state.theta,dr_state.psi,dr_control.phi_cmd,dr_control.theta_cmd,dr_control.psi_cmd);
      // fprintf(comp_log_t,"test1 : %f",dr_state.x);
  #endif

}
