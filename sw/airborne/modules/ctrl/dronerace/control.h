

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

// gains from AIRR sim

// float POSI = 0; 
float POS_I;
#define HOVERTHRUST 0.53
#define GRAVITY -9.80665
#define PI 3.14159265359

