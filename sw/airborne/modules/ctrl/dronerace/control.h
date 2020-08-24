
#include "filter.h"
#include <stdio.h>
struct dronerace_control_struct
{
  // States
  float psi_ref;    ///< maintain a rate limited speed set-point to smooth heading changes

  // Outputs to inner loop
  float phi_cmd;
  float theta_cmd;
  float psi_cmd;
  float z_cmd;
};

//extern struct dronerace_control_struct dr_control;

extern void control_reset(void);
extern void control_run(float dt);
// extern struct dronerace_state_struct dr_state = {0};

// gains from AIRR sim

 float POS_I ;
 float lookI;

 float vxE;
 float vyE;
 float vzE;
 


#define GRAVITY -9.80665
#define PI 3.14159265359
// #define FILE_LOGGER_PATH /data/ftp/internal_000