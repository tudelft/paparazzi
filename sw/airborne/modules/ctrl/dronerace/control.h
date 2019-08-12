#ifndef  P_FORWARD
#define P_FORWARD 1.1
#endif

#ifndef  D_FORWARD
#define D_FORWARD 0.0
#endif

#ifndef  P_LATERAL
#define P_LATERAL 1.1
#endif

#ifndef  D_LATERAL
#define D_LATERAL 0.0
#endif

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


#define HOVERTHRUST 0.53

// ahrs
#define KP_AHRS 0.01
#define KI_AHRS 0.0005

//outer loop
#define kp_pos_x 0.5
#define kd_pos_x 0.

#define kp_pos_y 0.5
#define kd_pos_y 0.

#define kp_pos_z 4.0
#define kd_pos_z 0.0

#define KP_VEL_X -0.15
#define KD_VEL_X -0.00001

#define KP_VEL_Y -0.15
#define KD_VEL_Y -0.00001

#define K_FF_THETA 0.0005
#define K_FF_PHI   0.0005

// Altitude control
#define KP_Z 0.8
#define KI_Z 0
#define KP_ZDOT -0.07
#define KI_ZDOT -0.004

// lateral vel ctrl
#define MAX_VEL_X 60
#define MAX_VEL_Y 60


// inner loop
#define KP_ROLL 0.1 //0.5
#define KD_ROLL 0.0
#define KP_PITCH -0.1 //0.65
#define KD_PITCH -0.0
#define KP_alt  0.

#define KP_YAW -0.1
#define KD_YAW 0.0
#define KP_YAWRATE 0.5