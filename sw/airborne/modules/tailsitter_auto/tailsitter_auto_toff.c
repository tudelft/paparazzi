#include "generated/airframe.h"
#include "state.h"
#include <stdio.h>
#include <math.h>
#include "math/pprz_algebra_float.h"

float take_off_thrust(void);

float take_off_thrust(void){
  int16_t thrust;
  struct FloatEulers eulers_zxy;
  struct FloatQuat * statequat = stateGetNedToBodyQuat_f();

  float_eulers_of_quat_zxy(&eulers_zxy, statequat);
  
  if (eulers_zxy.theta > RadOfDeg(30.0)){
    thrust = 1800;
    return thrust;
  }
  else{
    thrust = (int16_t)ceil(-(3600-1800)/RadOfDeg(30.0)*eulers_zxy.theta + 3600);
    return thrust;
  }


}