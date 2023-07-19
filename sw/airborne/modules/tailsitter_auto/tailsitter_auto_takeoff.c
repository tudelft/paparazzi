#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include <math.h>
#include "math/pprz_algebra_float.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/autopilot_static.h"
#include "autopilot.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

int16_t stage = 0;
int16_t counter = 0;
float Kq = TAKEOFF_Q_GAIN;
int16_t stage_1_thrust = 6400;

int16_t pwm2pprz(float pwm);
int16_t take_off_stage(float theta);
int16_t take_off_thrust(void);
void take_off_enter(void);

void take_off_enter(void){
  stage = 0;
  counter = 0;
  autopilot_static_set_motors_on(true);
}

int16_t take_off_thrust(void){
  float thrust_pwm=1000;
  int16_t thrust_pprz=0;
  struct FloatEulers eulers_zxy;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();
  struct FloatRates * body_rates = stateGetBodyRates_f();
  float_eulers_of_quat_zxy(&eulers_zxy, statequat);
  //int temp_stage = take_off_stage(eulers_zxy.theta);
  if(autopilot.mode == AP_MODE_NAV){
    if(stage == 0){
      thrust_pwm = 1000;
      thrust_pprz = pwm2pprz(thrust_pwm);
    }
    else if(stage == 1){
      //thrust_pwm = 105.1233*eulers_zxy.theta + 1764.7072;
      //thrust_pprz = pwm2pprz(thrust_pwm);
      int16_t pitch_rate_cont = (int16_t)ceil(Kq*(0.3 - body_rates->q));
      thrust_pprz = stage_1_thrust - pitch_rate_cont;
    }
    else if(stage == 2){
      thrust_pwm = -22.9362*eulers_zxy.theta + 1697.9914;
      thrust_pprz = pwm2pprz(thrust_pwm);
    }
    else{
    thrust_pwm = 1542;
    thrust_pprz = pwm2pprz(thrust_pwm);
  }
  }
  else{
    thrust_pprz = radio_control.values[RADIO_THROTTLE];
  }
  return thrust_pprz;

}

int16_t take_off_stage(float theta){
  //if((*stg != 0 || *stg != 3) && fabsf(theta) > RadOfDeg(85)){
    //stg = 0;
 //} 
  counter++;
  if(autopilot.mode == AP_MODE_NAV){
    if(stage == 0 && counter/500 > 1.0 && autopilot_get_motors_on()==true){
      stage = 1;
    }
    else if(stage == 1 && fabsf(theta) < RadOfDeg(30.0)){
      stage++; 
    }
    else if(stage == 2 && fabsf(theta) < RadOfDeg(5.0)){
      stage++;
    }
    else if(stage == 3){
      counter = 0;
    }
  } else{
    if(radio_control.values[RADIO_PIVOT_SWITCH] < -4500 ){
      counter = 0;
      stage = 0;
    }
    else if(radio_control.values[RADIO_PIVOT_SWITCH] < 4500){
      stage = 1;
    }
    else if(radio_control.values[RADIO_PIVOT_SWITCH] > 4500){
      stage = 3;
    }
  }
  return stage;
}
int16_t pwm2pprz(float pwm){
  int16_t pprz_cmd;
  pprz_cmd =  9600/900*pwm - 10666.67; //(int16_t)ceil((pwm - [1560 1350 1150 1150 1500 1500])./[-600 600 750 750 400 400]*9600);
  return pprz_cmd;
}