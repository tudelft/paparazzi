#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/time.h>
#include <math.h>
#include <stddef.h>
#include "filters/low_pass_filter.h"

int verbose_output = 1; 

struct timeval current_time, time_last_opt_run, init_time;

float Psi_old = 0.0f, psi_dot_filtered = 0.0f, speed_ref_out_old[3] = {0.0f, 0.0f, 0.0f}, acc_ref_out_old[3] = {0.0f, 0.0f, 0.0f}; 

float PERIODIC_FREQUENCY = 500; //Hz
float OVERACTUATED_MIXING_FIRST_ORDER_FILTER_COEFF_ANG_RATES = 0.0488;
float LIMITS_FWD_MIN_FWD_SPEED = -4.0, LIMITS_FWD_MAX_FWD_SPEED = 15.0;
float LIMITS_FWD_MAX_LAT_SPEED = 8.0; 
float LIMITS_FWD_MAX_VERT_SPEED = 3.0; 
float LIMITS_FWD_MAX_FWD_ACC = 7.0; 
float LIMITS_FWD_MIN_FWD_ACC = -2.0; 
float LIMITS_FWD_MAX_LAT_ACC = 7.0; 
float LIMITS_FWD_MAX_VERT_ACC = 4.0; 
float REF_MODEL_D_GAIN = 6.0; 
float REF_MODEL_P_GAIN = 2.0;

void compute_rm_speed_and_acc_control_rf(float * speed_ref_in, float * speed_ref_out, float * acc_ref_out, float Psi, float Vy_control){
    float desired_internal_acc_rm[3] = {0.0f, 0.0f, 0.0f}, desired_internal_jerk_rm[3] = {0.0f, 0.0f, 0.0f}; 
    //Compute Psi_dot
    float Psi_dot = (Psi - Psi_old)*PERIODIC_FREQUENCY; 
    Psi_old = Psi; 
    psi_dot_filtered = psi_dot_filtered + OVERACTUATED_MIXING_FIRST_ORDER_FILTER_COEFF_ANG_RATES * (Psi_dot - psi_dot_filtered);

    //Compute speed and acc ref based on the REF_MODEL_GAINS: 
    //First, bound the speed_ref_in with the max and min values: 
    Bound(speed_ref_in[0],LIMITS_FWD_MIN_FWD_SPEED,LIMITS_FWD_MAX_FWD_SPEED);
    BoundAbs(speed_ref_in[1],LIMITS_FWD_MAX_LAT_SPEED);
    BoundAbs(speed_ref_in[2],LIMITS_FWD_MAX_VERT_SPEED);

    desired_internal_acc_rm[0] = (speed_ref_in[0] - speed_ref_out_old[0])*REF_MODEL_P_GAIN; 
    Bound(desired_internal_acc_rm[0],LIMITS_FWD_MIN_FWD_ACC,LIMITS_FWD_MAX_FWD_ACC);

    desired_internal_acc_rm[2] = (speed_ref_in[2] - speed_ref_out_old[2])*REF_MODEL_P_GAIN; 
    BoundAbs(desired_internal_acc_rm[2],LIMITS_FWD_MAX_VERT_ACC);
    
    desired_internal_jerk_rm[0] = (desired_internal_acc_rm[0] - acc_ref_out_old[0])*REF_MODEL_D_GAIN; 
    desired_internal_jerk_rm[2] = (desired_internal_acc_rm[2] - acc_ref_out_old[2])*REF_MODEL_D_GAIN; 

    //Integrate jerk to get acc_ref_out: 
    acc_ref_out[0] = acc_ref_out_old[0] + desired_internal_jerk_rm[0]/PERIODIC_FREQUENCY;
    acc_ref_out[2] = acc_ref_out_old[2] + desired_internal_jerk_rm[2]/PERIODIC_FREQUENCY;

    //Save acc_ref variables
    for(int i=0; i<3; i++){
        acc_ref_out_old[i] = acc_ref_out[i]; 
    }

    //Add the non-intertial term to the acc_x component: 
    acc_ref_out[0] = acc_ref_out[0] + psi_dot_filtered * Vy_control;

    //Integrate acc to get speed_ref_out: 
    speed_ref_out[0] = speed_ref_out_old[0] + acc_ref_out[0]/PERIODIC_FREQUENCY;
    speed_ref_out[2] = speed_ref_out_old[2] + acc_ref_out[2]/PERIODIC_FREQUENCY;

    //Save speed_ref variables
    for(int i=0; i<3; i++){
        speed_ref_out_old[i] = speed_ref_out[i]; 
    }

    //Compute reference for lateral speed: 
    acc_ref_out[1] = 0.0f; 
    speed_ref_out[1] = speed_ref_in[1]; 
}

void test_init(void){
  gettimeofday(&time_last_opt_run, NULL);
  gettimeofday(&init_time, NULL);

    //Init reference model integrators: 
  for(int i=0; i<3; i++){
      speed_ref_out_old[i] = 0.0f;
      acc_ref_out_old[i] = 0.0f;
  }
}

void main() {

  //Initialize the serial 
  test_init();

  float speed_ref_in[3] = {0, 0, 0};

  while(1){

    while(((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_opt_run.tv_sec*1e6 + time_last_opt_run.tv_usec)) < (1/PERIODIC_FREQUENCY)*1e6){gettimeofday(&current_time, NULL);}


    float speed_ref_out[3], acc_ref_out[3];
    float Psi = 0; 
    float Vy_control = 0; 

    compute_rm_speed_and_acc_control_rf(speed_ref_in, speed_ref_out, acc_ref_out, Psi, Vy_control);

    gettimeofday(&current_time, NULL);
    float sim_time = ((current_time.tv_sec + current_time.tv_usec*1e-6) - (init_time.tv_sec + init_time.tv_usec*1e-6));
    if(sim_time >= 5){
      speed_ref_in[0] = 15;
    }
    if(sim_time >= 10){
      speed_ref_in[0] = 0;
    }

    //Print results if needed
    if(verbose_output){
      
      printf("\n speed_ref_out = [ %f %f %f] ; acc_ref_out = [ %f %f %f] ; sim_time = %.2f \n",(float) speed_ref_out[0], (float) speed_ref_out[1], (float) speed_ref_out[2], 
      (float) acc_ref_out[0], (float) acc_ref_out[1], (float) acc_ref_out[2], (float) sim_time ); 

      // printf("\n Refresh time uS = %f \n", (float) ((current_time.tv_sec*1e6 + current_time.tv_usec) - (time_last_opt_run.tv_sec*1e6 + time_last_opt_run.tv_usec))); 
      fflush(stdout);
    }

    //Reset waiting timer: 
    gettimeofday(&time_last_opt_run, NULL);

  }

}
