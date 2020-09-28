
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
// float dt = 1.0f / 512.f;
float vx_vel_old=0;
float vy_vel_old=0;


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
  
  
  file_logger_t = fopen(filename, "w+"); 
  bang_bang_t = fopen(filename2,"w+");
  fp_logger_t=fopen(filename3,"w+"); 
  comp_log_t=fopen(filename5,"w+");
  filter_log_t=fopen(filename4,"w+");
  


  fprintf(bang_bang_t,"time, satdim, brake, t_s, t_target, pos_error_vel_x, pos_error_vel_y, dr_state.x, dr_state.y, v0[0], v0[1], constant_sat_accel.c1, constant_sat_accel.c2, constant_sat_brake.c1, constant_sat_brake.c2, constant_sec.c1, constant_sec.c2, T_sat, T_sec, apply_compensation, in_transition, delta_t, delta_y, delta_v, ys, vs, delta_angle_in, delta_posx, delta_posy, direction\n");
  fprintf(file_logger_t,"time, dr_state.x, dr_state.y, posxVel, posyVel, dr_state.z, vxE, vyE, vzE, dr_state.vx, dr_state.vy, dr_state.phi, dr_state.theta, dr_state.psi, phi_cmd, theta_cmd, psi_cmd, vx_des, vy_des\n");
  fprintf(fp_logger_t,"time, gate_nr, gate_type, controller_type, gate_x, gate_y, gate_z, gate_psi, target_reached, vel_desired_speed\n");
  fprintf(comp_log_t,"time,satdim, v0, ang_0, ang_1, delta_t_meas, delta_y_meas, delta_v_meas\n");
  fprintf(filter_log_t,"time, gps_x, gps_y, gps_z, gps_vx, gps_vy, gps_vz, az, abx, aby, ax, ay, vx_avg, vy_avg, vx_compl, vy_compl, x_compl,y_compl, z_compl\n");
  
 fclose(comp_log_t);
  
}


// Settings

float vx_des_vel ;
float vy_des_vel ;

// Slow speed
#define CTRL_MAX_SPEED  10.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(50)    // rad  (overrules dr_bang.satangle if smaller)
#define CTRL_MAX_ROLL   RadOfDeg(50)    // rad
#define CTRL_MAX_R      RadOfDeg(90)    // rad/sec


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
  dr_bang.gate_speed_sat=0;
  vx_vel_old=0;
  vy_vel_old=0;
}

float checksign(float val){
  if(val<0){
    return -1;
  }
  else{
    return 1;
}
}

float bound_f(float val, float min, float max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}

// Normal PID gains
#define KP_POS  0.2
#define KI_POS 0.02
#define KP_VEL_X  0.2
#define KP_VEL_Y  0.2 
#define KD_VEL_X  0.05
#define KD_VEL_Y  0.05

// High gain PID gains
#define KP_POS_HIGH  1.8  
#define KI_POS_HIGH  0//0.02
#define KP_VEL_X_HIGH  1.5 // 1
#define KP_VEL_Y_HIGH  1.5 // 1 
#define KD_VEL_X_HIGH  0.1//0.05
#define KD_VEL_Y_HIGH  0.1//0.05

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
  float error_posz = dr_bang.gate_z - dr_state.z;

  dist2gate = sqrtf(error_posx_vel*error_posx_vel+error_posy_vel*error_posy_vel+error_posz*error_posz);

  float posxVel = cpsi*dr_state.x + spsi*dr_state.y;
  float posyVel = -spsi*dr_state.x + cpsi*dr_state.y ;

  float vx_vel=dr_state.vx*cpsi +dr_state.vy*spsi;
  float vy_vel=-dr_state.vx*spsi + dr_state.vy*cpsi;


  if(dr_bang.controller_type==BANGBANG)
  {
    psi_command=dr_control.psi_cmd; //used in bangbang.c 
    optimizeBangBang(error_posx_vel,error_posy_vel,dr_bang.gate_speed_sat); // function writes to bang_ctrl 

    if(fabs(error_posx_vel)>1){ //freeze yaw cmd when it gets close to wp
      dr_control.psi_cmd = atan2f(error_posy_E,error_posx_E);// angle180(psi_cmd*180.0/PI)*PI/180.0;
    }
    else{
      dr_control.psi_cmd=dr_bang.gate_psi; // note that below it may get overwritten if dr_bang.overwrite_psi is true
    }
    // dr_control.psi_cmd = atan2f(error_posy_E,error_posx_E);// angle180(psi_cmd*180.0/PI)*PI/180.0;
    dr_control.phi_cmd = bang_ctrl[1];
    dr_control.theta_cmd = bang_ctrl[0];
    vy_des_vel = bound_f(KP_POS_HIGH*error_posy_vel,-CTRL_MAX_SPEED, CTRL_MAX_SPEED); 
    vx_des_vel = bound_f(KP_POS_HIGH*error_posx_vel,-CTRL_MAX_SPEED,CTRL_MAX_SPEED);
    if(dist2gate<1 && fabs(dr_bang.gate_speed_sat)<0.5){ // when close to the waypoint the lateral control switches to PD control. 
      if(satdim==0){
          dr_control.phi_cmd= bound_f(KP_VEL_Y * (vy_des_vel-vy_vel)-KD_VEL_Y*(vy_vel-vy_vel_old)/dt,-dr_bang.sat_angle/r2d,dr_bang.sat_angle/r2d);
      }
      else{
          dr_control.theta_cmd= bound_f(-KP_VEL_X * (vx_des_vel-vx_vel)+KD_VEL_X*(vx_vel-vx_vel_old)/dt,-dr_bang.sat_angle/r2d,dr_bang.sat_angle/r2d);//Uncomment to overwrite with pd values
      }
    }

    // 
    // 
    
  }
    else if(dr_bang.controller_type==HIGHPID){ 
        if(fabs(error_posx_vel)>fabs(error_posy_vel)){
          vx_des_vel=dr_bang.gate_speed_sat*(float)checksign(error_posx_vel);
          vy_des_vel=0;
        }
        else{
          vy_des_vel=dr_bang.gate_speed_sat*(float)checksign(error_posy_vel);
          vx_des_vel=0;
        }
      vx_des_vel = bound_f(vx_des_vel+KP_POS_HIGH*error_posx_vel,-CTRL_MAX_SPEED,CTRL_MAX_SPEED); //saturate to max velocity
      vy_des_vel = bound_f(vy_des_vel+KP_POS_HIGH*error_posy_vel,-CTRL_MAX_SPEED,CTRL_MAX_SPEED);
      
      if(fabs(error_posx_vel)>1){ //freeze yaw cmd when it gets close to wp
        dr_control.psi_cmd =atan2f(error_posy_E,error_posx_E); // yaw towards gate when distance is large enough. 
      }
      else{
        dr_control.psi_cmd=dr_bang.gate_psi;
      }
      if(dr_bang.turning==TURNING){
        dr_control.psi_cmd=dr_bang.gate_psi;
      }
      // dr_control.psi_cmd =atan2f(error_posy_E,error_posx_E);
      
      dr_control.phi_cmd= KP_VEL_Y_HIGH * (vy_des_vel-vy_vel)-KD_VEL_Y_HIGH*(vy_vel-vy_vel_old)/dt;
      dr_control.theta_cmd=KP_VEL_X_HIGH *-1* (vx_des_vel-vx_vel)+KD_VEL_X_HIGH*(vx_vel-vx_vel_old)/dt;

      dr_control.phi_cmd=bound_f(dr_control.phi_cmd,-d2r*fabs(dr_bang.sat_angle),d2r*fabs(dr_bang.sat_angle));
      dr_control.theta_cmd=bound_f(dr_control.theta_cmd,-d2r*fabs(dr_bang.sat_angle),d2r*fabs(dr_bang.sat_angle));
      vx_vel_old=vx_vel;
      vy_vel_old=vy_vel;
  }    
  else{ // USE a PID controller 
            
       vx_des_vel = bound_f(error_posx_vel,-CTRL_MAX_SPEED,CTRL_MAX_SPEED); //saturate to max velocity
       vy_des_vel = bound_f(error_posy_vel,-CTRL_MAX_SPEED,CTRL_MAX_SPEED);
      
      if(fabs(error_posx_vel)>1){ //freeze yaw cmd when it gets close to wp
        dr_control.psi_cmd =atan2f(error_posy_E,error_posx_E); // yaw towards gate when distance is large enough. 
      }
      else{
        dr_control.psi_cmd=dr_bang.gate_psi;
      }
      if(dr_bang.turning==TURNING){
        dr_control.psi_cmd=dr_bang.gate_psi;
      }

      dr_control.phi_cmd= bound_f(KP_VEL_Y * (vy_des_vel-vy_vel),-d2r*30,d2r*30);
      dr_control.theta_cmd=bound_f(KP_VEL_X *-1* (vx_des_vel-vx_vel),-d2r*30,d2r*30);
  }
    if (dr_bang.overwrite_psi){
      dr_control.psi_cmd=dr_bang.psi_forced; //
    }

  
  #ifdef LOG
      fprintf(file_logger_t,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",get_sys_time_float(),dr_state.x,dr_state.y,posxVel,posyVel,dr_state.z,vxE,vyE,vzE,dr_state.vx,dr_state.vy,dr_state.phi,dr_state.theta,dr_state.psi,dr_control.phi_cmd,dr_control.theta_cmd,dr_control.psi_cmd,vx_des_vel,vy_des_vel);
  #endif

}
