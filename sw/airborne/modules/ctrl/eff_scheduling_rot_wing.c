/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/ctrl/eff_scheduling_rot_wing.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * The control effectiveness scheduler for the rotating wing drone type
 */

#include "modules/ctrl/eff_scheduling_rot_wing.h"
#include "generated/airframe.h"
#include "state.h"

#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"

#define FORCE_ONELOOP
#ifdef FORCE_ONELOOP
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"
float actuator_state_filt_vect[EFF_MAT_COLS_NB] = {0};
#else
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

#ifndef SERVO_ROTATION_MECH_IDX
#error ctrl_eff_sched_rot_wing requires a servo named ROTATION_MECH_IDX
#endif

#ifndef ROT_WING_EFF_SCHED_IXX_BODY
#error "NO ROT_WING_EFF_SCHED_IXX_BODY defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IYY_BODY
#error "NO ROT_WING_EFF_SCHED_IYY_BODY defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IZZ
#error "NO ROT_WING_EFF_SCHED_IZZ defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IXX_WING
#error "NO ROT_WING_EFF_SCHED_IXX_WING defined"
#endif

#ifndef ROT_WING_EFF_SCHED_IYY_WING
#error "NO ROT_WING_EFF_SCHED_IYY_WING defined"
#endif

#ifndef ROT_WING_EFF_SCHED_M
#error "NO ROT_WING_EFF_SCHED_M defined"
#endif

#ifndef ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH
#error "NO ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH defined"
#endif

#ifndef ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL
#error "NO ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL defined"
#endif

#ifndef ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF
#error "NO ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF defined"
#endif

#ifndef ROT_WING_EFF_SCHED_HOVER_ROLL_ROLL_COEF
#error "NO ROT_WING_EFF_SCHED_HOVER_ROLL_ROLL_COEF defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_ELEVATOR
#error "NO ROT_WING_EFF_SCHED_K_ELEVATOR defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_RUDDER
#error "NO ROT_WING_EFF_SCHED_K_RUDDER defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_AILERON
#error "NO ROT_WING_EFF_SCHED_K_AILERON defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_FLAPERON
#error "NO ROT_WING_EFF_SCHED_K_FLAPERON defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_PUSHER
#error "NO ROT_WING_EFF_SCHED_K_PUSHER defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_ELEVATOR_DEFLECTION
#error "NO ROT_WING_EFF_SCHED_K_ELEVATOR_DEFLECTION defined"
#endif

#ifndef ROT_WING_EFF_SCHED_D_RUDDER_D_PPRZ
#error "NO ROT_WING_EFF_SCHED_D_RUDDER_D_PPRZ defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_RPM_PPRZ_PUSHER
#error "NO ROT_WING_EFF_SCHED_K_RPM_PPRZ_PUSHER defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_LIFT_WING
#error "NO ROT_WING_EFF_SCHED_K_LIFT_WING defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_LIFT_FUSELAGE
#error "NO ROT_WING_EFF_SCHED_K_LIFT_FUSELAGE defined"
#endif

#ifndef ROT_WING_EFF_SCHED_K_LIFT_TAIL
#error "NO ROT_WING_EFF_SCHED_K_LIFT_TAIL defined"
#endif

#ifndef EFF_MAT_SIMPLE
#define EFF_MAT_SIMPLE TRUE
#endif

/* Effectiveness Matrix definition */
float G2_RW[EFF_MAT_COLS_NB]                       = {0};//ROT_WING_EFF_SCHED_G2; //scaled by RW_G_SCALE
float G1_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB]      = {0};//{ROT_WING_EFF_SCHED_G1_ZERO, ROT_WING_EFF_SCHED_G1_ZERO, ROT_WING_EFF_SCHED_G1_THRUST, ROT_WING_EFF_SCHED_G1_ROLL, ROT_WING_EFF_SCHED_G1_PITCH, ROT_WING_EFF_SCHED_G1_YAW}; //scaled by RW_G_SCALE 
float EFF_MAT_RW[EFF_MAT_ROWS_NB][EFF_MAT_COLS_NB] = {0};
static float flt_cut = 1.0e-4;

struct FloatEulers eulers_zxy_RW_EFF;

struct rot_wing_eff_sched_param_t eff_sched_p = {
  .Ixx_body                 = ROT_WING_EFF_SCHED_IXX_BODY,
  .Iyy_body                 = ROT_WING_EFF_SCHED_IYY_BODY,
  .Izz                      = ROT_WING_EFF_SCHED_IZZ,
  .Ixx_wing                 = ROT_WING_EFF_SCHED_IXX_WING,
  .Iyy_wing                 = ROT_WING_EFF_SCHED_IYY_WING,
  .m                        = ROT_WING_EFF_SCHED_M,
  .DMdpprz_hover_roll       = ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_ROLL,
  .hover_roll_pitch_coef    = ROT_WING_EFF_SCHED_HOVER_ROLL_PITCH_COEF,
  .hover_roll_roll_coef     = ROT_WING_EFF_SCHED_HOVER_ROLL_ROLL_COEF,
  .k_elevator               = ROT_WING_EFF_SCHED_K_ELEVATOR,
  .k_rudder                 = ROT_WING_EFF_SCHED_K_RUDDER,
  .k_aileron                = ROT_WING_EFF_SCHED_K_AILERON,
  .k_flaperon               = ROT_WING_EFF_SCHED_K_FLAPERON,
  .k_pusher                 = ROT_WING_EFF_SCHED_K_PUSHER,
  .k_elevator_deflection    = ROT_WING_EFF_SCHED_K_ELEVATOR_DEFLECTION,
  .d_rudder_d_pprz          = ROT_WING_EFF_SCHED_D_RUDDER_D_PPRZ,
  .k_rpm_pprz_pusher        = ROT_WING_EFF_SCHED_K_RPM_PPRZ_PUSHER,
  .k_lift_wing              = ROT_WING_EFF_SCHED_K_LIFT_WING,
  .k_lift_fuselage          = ROT_WING_EFF_SCHED_K_LIFT_FUSELAGE,
  .k_lift_tail              = ROT_WING_EFF_SCHED_K_LIFT_TAIL
};

struct rot_wing_eff_sched_var_t eff_sched_var;

/* Define Forces and Moments tructs for each actuator*/
struct RW_Model RW;
//struct RW.attitde RW.att;

inline void eff_scheduling_rot_wing_update_wing_angle(void);
//inline void eff_scheduling_rot_wing_update_MMOI(void);
// inline void eff_scheduling_rot_wing_update_cmd(void);
inline void eff_scheduling_rot_wing_update_airspeed(void);
// inline void eff_scheduling_rot_wing_update_hover_motor_effectiveness(void);
// inline void eff_scheduling_rot_wing_update_elevator_effectiveness(void);
// inline void eff_scheduling_rot_wing_update_rudder_effectiveness(void);
// inline void eff_scheduling_rot_wing_update_aileron_effectiveness(void);
// inline void eff_scheduling_rot_wing_update_flaperon_effectiveness(void);
// inline void eff_scheduling_rot_wing_update_pusher_effectiveness(void);
// inline void eff_scheduling_rot_wing_schedule_liftd(void);
void  update_attitude(void);
void  sum_EFF_MAT_RW(void);
void  init_RW_Model(void);
void  calc_G1_G2_RW(void);
//inline float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED);
//void stabilization_indi_set_wls_settings(void);


/** ABI binding wing position data.
 */
#ifndef WING_ROTATION_CAN_ROT_WING_ID
#define WING_ROTATION_CAN_ROT_WING_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(WING_ROTATION_CAN_ROT_WING_ID)
static abi_event wing_position_ev;

static void wing_position_cb(uint8_t sender_id UNUSED, struct act_feedback_t *pos_msg, uint8_t num_act)
{
  for (int i=0; i<num_act; i++){
    if (pos_msg[i].set.position && (pos_msg[i].idx == SERVO_ROTATION_MECH_IDX))
    {
      // Get wing rotation angle from sensor
      eff_sched_var.wing_rotation_rad = 0.5 * M_PI - pos_msg[i].position;

      // Bound wing rotation angle
      Bound(eff_sched_var.wing_rotation_rad, 0, 0.5 * M_PI);
    }
  }
}
void init_RW_Model(void)
{
  // Inertia and mass
  RW.I.b_xx = 0.0478; // [kgm²]
  RW.I.b_yy = 0.7546; // [kgm²]
  RW.I.w_xx = 0.08099; // [kgm²]
  RW.I.w_yy = 0.1949; // [kgm²]
  RW.I.xx   = RW.I.b_xx + RW.I.w_xx; // [kgm²]
  RW.I.yy   = RW.I.b_yy + RW.I.b_yy; // [kgm²]
  RW.I.zz   = 0.975; // [kgm²]
  RW.m      = 6.670; // [kg]
  // Motor Front
  RW.mF.dFdu     = 3.835 / RW_G_SCALE; // [N  / pprz] 
  RW.mF.dMdu     = 0.390 / RW_G_SCALE; // [Nm / pprz]
  RW.mF.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mF.l        = 0.423             ; // [m]                   
  // Motor Right
  RW.mR.dFdu     = 3.835 / RW_G_SCALE; // [N  / pprz]
  RW.mR.dMdu     = 0.390 / RW_G_SCALE; // [Nm / pprz]
  RW.mR.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mR.l        = 0.408             ; // [m]        
  // Motor Back
  RW.mB.dFdu     = 3.835 / RW_G_SCALE; // [N  / pprz]
  RW.mB.dMdu     = 0.390 / RW_G_SCALE; // [Nm / pprz]
  RW.mB.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mB.l        = 0.423             ; // [m]        
  // Motor Left
  RW.mL.dFdu     = 3.835 / RW_G_SCALE; // [N  / pprz]
  RW.mL.dMdu     = 0.390 / RW_G_SCALE; // [Nm / pprz]
  RW.mL.dMdud    = 0.020 / RW_G_SCALE; // [Nm / pprz]
  RW.mL.l        = 0.408             ; // [m]        
  // Motor Pusher
  RW.mP.dFdu     = 3.468 / RW_G_SCALE; // [N  / pprz]
  RW.mP.dMdu     = 0.000 / RW_G_SCALE; // [Nm / pprz]
  RW.mP.dMdud    = 0.000 / RW_G_SCALE; // [Nm / pprz]
  RW.mP.l        = 0.000             ; // [m]        
  // Elevator
  RW.ele.dFdu    = 24.81 / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz]   
  RW.ele.dMdu    = 0;                                 // [Nm / pprz]
  RW.ele.dMdud   = 0;                                 // [Nm / pprz]
  RW.ele.l       = 0.85;                              // [m]               
  // Rudder
  RW.rud.dFdu   = 1.207 / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz] 
  RW.rud.dMdu   = 0;                                 // [Nm / pprz]
  RW.rud.dMdud  = 0;                                 // [Nm / pprz]
  RW.rud.l      = 0.88;                              // [m]        
  // Aileron
  RW.ail.dFdu   = 4.084 / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz]
  RW.ail.dMdu   = 0;                                 // [Nm / pprz]
  RW.ail.dMdud  = 0;                                 // [Nm / pprz]
  RW.ail.l      = 0.68;                              // [m]        
  // Flaperon
  RW.flp.dFdu   = 5.758 / (RW_G_SCALE * RW_G_SCALE); // [N  / pprz]
  RW.flp.dMdu   = 0;                                 // [Nm / pprz]
  RW.flp.dMdud  = 0;                                 // [Nm / pprz]
  RW.flp.l      = 0.36;                              // [m]     
  // Lift: k0 the v²+ k1 the sin² v² + k2 sin² v²
  RW.wing.k0    = 0.336 + 0.0507 + 0.102;
  RW.wing.k1    = 0.616;
  RW.wing.k2    = 0.121;
}

/*Update the attitude*/
void  update_attitude(void)
{
  float_eulers_of_quat_zxy(&eulers_zxy_RW_EFF, stateGetNedToBodyQuat_f());
  RW.att.phi    = eulers_zxy_RW_EFF.phi;
  RW.att.theta  = eulers_zxy_RW_EFF.theta;
  RW.att.psi    = eulers_zxy_RW_EFF.psi;
  RW.att.sphi   = sinf(eulers_zxy_RW_EFF.phi);
  RW.att.cphi   = cosf(eulers_zxy_RW_EFF.phi);
  RW.att.stheta = sinf(eulers_zxy_RW_EFF.theta);
  RW.att.ctheta = cosf(eulers_zxy_RW_EFF.theta);
  RW.att.spsi   = sinf(eulers_zxy_RW_EFF.psi);
  RW.att.cpsi   = cosf(eulers_zxy_RW_EFF.psi);
}
/* Function to precalculate once some constant effectiveness values to improve efficiency*/
void calc_G1_G2_RW(void)
{
  // Motor Front
  G1_RW[aZ][COMMAND_MOTOR_FRONT]  = -RW.mF.dFdu / RW.m;
  G1_RW[aq][COMMAND_MOTOR_FRONT]  =  (RW.mF.dFdu * RW.mF.l) / RW.I.yy;
  G1_RW[ar][COMMAND_MOTOR_FRONT]  = -RW.mF.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_FRONT]      = -RW.mF.dMdud / RW.I.zz;
  // Motor Right
  G1_RW[aZ][COMMAND_MOTOR_RIGHT]  = -RW.mR.dFdu / RW.m;
  G1_RW[ap][COMMAND_MOTOR_RIGHT]  = -(RW.mR.dFdu * RW.mR.l * eff_sched_var.cosr) / RW.I.xx;
  G1_RW[aq][COMMAND_MOTOR_RIGHT]  =  (RW.mR.dFdu * RW.mR.l * eff_sched_var.sinr) / RW.I.yy;
  G1_RW[ar][COMMAND_MOTOR_RIGHT]  =  RW.mR.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_RIGHT]      =  RW.mR.dMdud / RW.I.zz;
  // Motor Back
  G1_RW[aZ][COMMAND_MOTOR_BACK]   = -RW.mB.dFdu / RW.m;
  G1_RW[aq][COMMAND_MOTOR_BACK]   = -(RW.mB.dFdu * RW.mB.l) / RW.I.yy;
  G1_RW[ar][COMMAND_MOTOR_BACK]   = -RW.mB.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_BACK]       = -RW.mB.dMdud / RW.I.zz;
  // Motor Left
  G1_RW[aZ][COMMAND_MOTOR_LEFT]   = -RW.mL.dFdu / RW.m;
  G1_RW[ap][COMMAND_MOTOR_LEFT]   =  (RW.mL.dFdu * RW.mL.l * eff_sched_var.cosr) / RW.I.xx;
  G1_RW[aq][COMMAND_MOTOR_LEFT]   = -(RW.mL.dFdu * RW.mL.l * eff_sched_var.sinr) / RW.I.yy;
  G1_RW[ar][COMMAND_MOTOR_LEFT]   =  RW.mL.dMdu  / RW.I.zz;
  G2_RW[COMMAND_MOTOR_LEFT]       =  RW.mL.dMdud / RW.I.zz;
  // Motor Pusher
  G1_RW[aX][COMMAND_MOTOR_PUSHER] =  RW.mP.dFdu / RW.m;
  // Elevator
  G1_RW[aq][COMMAND_ELEVATOR]     =  (RW.ele.dFdu * eff_sched_var.airspeed2 * RW.ele.l) / RW.I.yy;
  // Rudder
  G1_RW[ar][COMMAND_RUDDER]       =  (RW.rud.dFdu * eff_sched_var.airspeed2 * RW.rud.l) / RW.I.zz ;
  // Aileron
  G1_RW[ap][COMMAND_AILERONS]     =  (RW.ail.dFdu * eff_sched_var.airspeed2 * RW.ail.l * eff_sched_var.sinr3) / RW.I.xx;
  G1_RW[aq][COMMAND_AILERONS]     =  (RW.ail.dFdu * eff_sched_var.airspeed2 * RW.ail.l * (eff_sched_var.cosr-eff_sched_var.cosr3)) / RW.I.yy;
  
  // Flaperon
  G1_RW[ap][COMMAND_FLAPS]        =  (RW.flp.dFdu * eff_sched_var.airspeed2 * RW.flp.l * eff_sched_var.sinr3) / RW.I.xx;
  G1_RW[aq][COMMAND_FLAPS]        =  (RW.flp.dFdu * eff_sched_var.airspeed2 * RW.flp.l * (eff_sched_var.cosr-eff_sched_var.cosr3)) / RW.I.yy;
  // Lift and thrust
  RW.wing.dLdtheta                =  (RW.wing.k0 + RW.wing.k1 * eff_sched_var.sinr2) * eff_sched_var.airspeed2;
  Bound(RW.wing.dLdtheta, 0.0, 1300.0);
  RW.wing.L                       =  RW.wing.k0 * RW.att.theta * eff_sched_var.airspeed2 + RW.wing.k1 * RW.att.theta * eff_sched_var.sinr2 * eff_sched_var.airspeed2 + RW.wing.k2 * eff_sched_var.sinr2 * eff_sched_var.airspeed2;
  Bound(RW.wing.L, 0.0, 350.0);
  RW.T = actuator_state_1l[COMMAND_MOTOR_FRONT] * RW.mF.dFdu + actuator_state_1l[COMMAND_MOTOR_RIGHT] * RW.mR.dFdu + actuator_state_1l[COMMAND_MOTOR_BACK] * RW.mB.dFdu + actuator_state_1l[COMMAND_MOTOR_LEFT] * RW.mL.dFdu;
  Bound(RW.T, 0.0, 140.0);
  RW.P                            = actuator_state_1l[COMMAND_MOTOR_PUSHER] * RW.mP.dFdu;
  // Inertia
  RW.I.xx = RW.I.b_xx + eff_sched_var.cosr2 * RW.I.w_xx + eff_sched_var.sinr2 * RW.I.w_yy;
  RW.I.yy = RW.I.b_yy + eff_sched_var.sinr2 * RW.I.w_xx + eff_sched_var.cosr2 * RW.I.w_yy;
  Bound(RW.I.xx, 0.01, 100.);
  Bound(RW.I.yy, 0.01, 100.);
}

void eff_scheduling_rot_wing_init(void)
{
  // Initialize variables to quad values
  eff_sched_var.Ixx               = eff_sched_p.Ixx_body + eff_sched_p.Ixx_wing;
  eff_sched_var.Iyy               = eff_sched_p.Iyy_body + eff_sched_p.Iyy_wing;
  eff_sched_var.wing_rotation_rad = 0; // ABI input
  eff_sched_var.wing_rotation_deg = 0;
  eff_sched_var.cosr              = 1;
  eff_sched_var.sinr              = 0;
  eff_sched_var.cosr2             = 1;
  eff_sched_var.sinr2             = 0;
  eff_sched_var.sinr3             = 0;

  // Initialize attitude
  RW.att.phi    = 0.0;
  RW.att.theta  = 0.0; 
  RW.att.psi    = 0.0; 
  RW.att.sphi   = 0.0; 
  RW.att.cphi   = 0.0; 
  RW.att.stheta = 0.0; 
  RW.att.ctheta = 0.0; 
  RW.att.spsi   = 0.0; 
  RW.att.cpsi   = 0.0; 
  // Set moment derivative variables
  eff_sched_var.pitch_motor_dMdpprz = ROT_WING_EFF_SCHED_DM_DPPRZ_HOVER_PITCH;
  eff_sched_var.roll_motor_dMdpprz  = (eff_sched_p.DMdpprz_hover_roll[0] + (MAX_PPRZ/2.) * eff_sched_p.DMdpprz_hover_roll[1]) / 10000.; // Dmdpprz hover roll for hover thrust

  eff_sched_var.cmd_elevator = 0;
  eff_sched_var.cmd_pusher = 0;
  eff_sched_var.cmd_pusher_scaled = 0;
  eff_sched_var.cmd_T_mean_scaled = 0;

  eff_sched_var.airspeed = 0;
  eff_sched_var.airspeed2 = 0;
  // Run initialization of the RW model
  init_RW_Model();
  update_attitude();
  // Get wing angle
  AbiBindMsgACT_FEEDBACK(WING_ROTATION_CAN_ROT_WING_ID, &wing_position_ev, wing_position_cb);
}

void eff_scheduling_rot_wing_periodic(void)
{
#if EFF_MAT_SIMPLE
  update_attitude();
  eff_scheduling_rot_wing_update_wing_angle();
  eff_scheduling_rot_wing_update_airspeed();
  //eff_scheduling_rot_wing_update_MMOI();
  calc_G1_G2_RW();
  sum_EFF_MAT_RW();
#else
  // your periodic code here.
  // freq = 10.0 Hz
  // eff_scheduling_rot_wing_update_wing_angle();
  // eff_scheduling_rot_wing_update_MMOI();
  // eff_scheduling_rot_wing_update_cmd();
  // eff_scheduling_rot_wing_update_airspeed();

  // Update the effectiveness values
  // eff_scheduling_rot_wing_update_hover_motor_effectiveness();
  // eff_scheduling_rot_wing_update_elevator_effectiveness();
  // eff_scheduling_rot_wing_update_rudder_effectiveness();
  // eff_scheduling_rot_wing_update_aileron_effectiveness();
  // eff_scheduling_rot_wing_update_flaperon_effectiveness();
  // eff_scheduling_rot_wing_update_pusher_effectiveness();
  // eff_scheduling_rot_wing_schedule_liftd();
#endif
}


/**
 * @brief Function that sums g1 and g2 to obtain the g1_g2 matrix. It also undoes the scaling that was done to make the values readable
 * FIXME: make this function into a for loop to make it more adaptable to different configurations
 */


void sum_EFF_MAT_RW(void) {
  
  // Thrust and Pusher force estimation
  float L      = RW.wing.L / RW.m;          // Lift specific force
  float dLdtheta = RW.wing.dLdtheta / RW.m; // Lift specific force derivative with pitch
  float T      = RW.T / RW.m;             //  Thrust specific force. Minus gravity is a guesstimate.
  float P      = RW.P / RW.m;               // Pusher specific force
  float T_L    = T + L * RW.att.ctheta;     // Thrust and Lift term
  float P_L    = P + L * RW.att.stheta;     // Pusher and Lift term
  int i = 0;
  int j = 0;

  for (i = 0; i < EFF_MAT_COLS_NB; i++) {
    switch (i) {
    case (COMMAND_MOTOR_FRONT):
    case (COMMAND_MOTOR_BACK):
    case (COMMAND_MOTOR_RIGHT):     
    case (COMMAND_MOTOR_LEFT):
      EFF_MAT_RW[aN][i] = (RW.att.cpsi * RW.att.stheta + RW.att.ctheta * RW.att.sphi   * RW.att.spsi) * G1_RW[aZ][i];
      EFF_MAT_RW[aE][i] = (RW.att.spsi * RW.att.stheta - RW.att.cpsi   * RW.att.ctheta * RW.att.sphi) * G1_RW[aZ][i];
      EFF_MAT_RW[aD][i] = (RW.att.cphi * RW.att.ctheta                                              ) * G1_RW[aZ][i];
      EFF_MAT_RW[ap][i] = (G1_RW[ap][i])                                       ;
      EFF_MAT_RW[aq][i] = (G1_RW[aq][i])                                       ;
      EFF_MAT_RW[ar][i] = (G1_RW[ar][i] + G2_RW[i])                            ;
      break;
    case (COMMAND_MOTOR_PUSHER): 
      EFF_MAT_RW[aN][i] = (RW.att.cpsi   * RW.att.ctheta - RW.att.sphi * RW.att.spsi * RW.att.stheta) * G1_RW[aX][i];
      EFF_MAT_RW[aE][i] = (RW.att.ctheta * RW.att.spsi   + RW.att.cpsi * RW.att.sphi * RW.att.stheta) * G1_RW[aX][i];
      EFF_MAT_RW[aD][i] = (- RW.att.cphi * RW.att.stheta                                            ) * G1_RW[aX][i];
      EFF_MAT_RW[ap][i] = 0.0;
      EFF_MAT_RW[aq][i] = 0.0;
      EFF_MAT_RW[ar][i] = 0.0;
      break;
    case (COMMAND_ELEVATOR):
    case (COMMAND_RUDDER): 
    case (COMMAND_AILERONS):
    case (COMMAND_FLAPS):  
      EFF_MAT_RW[aN][i] = 0.0;
      EFF_MAT_RW[aE][i] = 0.0;
      EFF_MAT_RW[aD][i] = 0.0;
      EFF_MAT_RW[ap][i] = G1_RW[ap][i];
      EFF_MAT_RW[aq][i] = G1_RW[aq][i];
      EFF_MAT_RW[ar][i] = G1_RW[ar][i];
      break;     
    case (COMMAND_ROLL):
      EFF_MAT_RW[aN][i] = (-RW.att.cphi * RW.att.ctheta * RW.att.spsi * T_L - RW.att.cphi * RW.att.spsi * RW.att.stheta * P_L);
      EFF_MAT_RW[aE][i] = ( RW.att.cphi * RW.att.ctheta * RW.att.cpsi * T_L + RW.att.cphi * RW.att.cpsi * RW.att.stheta * P_L);
      EFF_MAT_RW[aD][i] = ( RW.att.sphi * RW.att.ctheta * T_L + RW.att.sphi * RW.att.stheta * P_L);
      EFF_MAT_RW[ap][i] = 0.0;
      EFF_MAT_RW[aq][i] = 0.0;
      EFF_MAT_RW[ar][i] = 0.0;  
      break;
    case (COMMAND_PITCH):
      EFF_MAT_RW[aN][i] = (-(RW.att.ctheta * RW.att.cpsi - RW.att.sphi * RW.att.stheta * RW.att.spsi) * T - (RW.att.cpsi * RW.att.stheta + RW.att.ctheta * RW.att.sphi   * RW.att.spsi) * P - RW.att.sphi * RW.att.spsi * dLdtheta);
      EFF_MAT_RW[aE][i] = (-(RW.att.ctheta * RW.att.spsi + RW.att.sphi * RW.att.stheta * RW.att.cpsi) * T - (RW.att.spsi * RW.att.stheta - RW.att.cpsi   * RW.att.ctheta * RW.att.sphi) * P + RW.att.sphi * RW.att.cpsi * dLdtheta);
      EFF_MAT_RW[aD][i] = ( RW.att.stheta * RW.att.cphi * T - RW.att.cphi * RW.att.ctheta * P - RW.att.cphi * dLdtheta)                                           ;
      EFF_MAT_RW[ap][i] = 0.0;
      EFF_MAT_RW[aq][i] = 0.0;
      EFF_MAT_RW[ar][i] = 0.0;
      break;
    default:
      break;
    }
  }
  for (i = 0; i < EFF_MAT_ROWS_NB; i++) {
    for(j = 0; j < EFF_MAT_COLS_NB; j++) {
      float abs = fabs(EFF_MAT_RW[i][j]);
      if (abs < flt_cut) {
        EFF_MAT_RW[i][j] = 0.0;
      }
    }
  }
}

void eff_scheduling_rot_wing_update_wing_angle(void)
{
  // Calculate sin and cosines of rotation
  eff_sched_var.wing_rotation_deg = eff_sched_var.wing_rotation_rad / M_PI * 180.;

  eff_sched_var.cosr = cosf(eff_sched_var.wing_rotation_rad);
  eff_sched_var.sinr = sinf(eff_sched_var.wing_rotation_rad);

  eff_sched_var.cosr2 = eff_sched_var.cosr * eff_sched_var.cosr;
  eff_sched_var.sinr2 = eff_sched_var.sinr * eff_sched_var.sinr;

  eff_sched_var.sinr3 = eff_sched_var.sinr2 * eff_sched_var.sinr;
  eff_sched_var.cosr3 = eff_sched_var.cosr2 * eff_sched_var.cosr;

}

void eff_scheduling_rot_wing_update_airspeed(void)
{
  eff_sched_var.airspeed = stateGetAirspeed_f();
  Bound(eff_sched_var.airspeed, 0. , 30.);
  eff_sched_var.airspeed2 = eff_sched_var.airspeed * eff_sched_var.airspeed;
  Bound(eff_sched_var.airspeed2, 0. , 900.);
}

// void eff_scheduling_rot_wing_update_MMOI(void)
// {
//   eff_sched_var.Ixx = eff_sched_p.Ixx_body + eff_sched_var.cosr2 * eff_sched_p.Ixx_wing + eff_sched_var.sinr2 * eff_sched_p.Iyy_wing;
//   eff_sched_var.Iyy = eff_sched_p.Iyy_body + eff_sched_var.sinr2 * eff_sched_p.Ixx_wing + eff_sched_var.cosr2 * eff_sched_p.Iyy_wing;

//   // Bound inertia
//   Bound(eff_sched_var.Ixx, 0.01, 100.);
//   Bound(eff_sched_var.Iyy, 0.01, 100.);
// }

// Implemented ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// void eff_scheduling_rot_wing_update_cmd(void)
// {
//   // These indexes depend on the INDI sequence, not the actuator IDX
//   eff_sched_var.cmd_elevator = actuator_state_filt_vect[5];
//   eff_sched_var.cmd_pusher = actuator_state_filt_vect[8];
//   eff_sched_var.cmd_pusher_scaled = actuator_state_filt_vect[8] * 0.000853229; // Scaled with 8181 / 9600 / 1000
//   eff_sched_var.cmd_T_mean_scaled = (actuator_state_filt_vect[0] + actuator_state_filt_vect[1] + actuator_state_filt_vect[2] + actuator_state_filt_vect[3]) / 4. * 0.000853229; // Scaled with 8181 / 9600 / 1000
// }

// void eff_scheduling_rot_wing_update_elevator_effectiveness(void)
// {
//   float de = eff_sched_p.k_elevator_deflection[0] + eff_sched_p.k_elevator_deflection[1] * eff_sched_var.cmd_elevator;

//   float dMyde = (eff_sched_p.k_elevator[0] * de * eff_sched_var.airspeed2 +
//                  eff_sched_p.k_elevator[1] * eff_sched_var.cmd_pusher_scaled * eff_sched_var.cmd_pusher_scaled * eff_sched_var.airspeed +
//                  eff_sched_p.k_elevator[2] * eff_sched_var.airspeed2) / 10000.;

//   float dMydpprz = dMyde * eff_sched_p.k_elevator_deflection[1];

//   // Convert moment to effectiveness
//   float eff_y_elev = dMydpprz / eff_sched_var.Iyy;

//   Bound(eff_y_elev, 0.00001, 0.1);

//   EFF_MAT_RW[1][5] = eff_y_elev;
// }


// float eff_scheduling_rot_wing_lift_d = 0.0f;

// void eff_scheduling_rot_wing_schedule_liftd(void)
// {
//   float lift_d_wing = (eff_sched_p.k_lift_wing[0] + eff_sched_p.k_lift_wing[1] * eff_sched_var.sinr2) * eff_sched_var.airspeed2 / eff_sched_p.m;
//   float lift_d_fuselage = eff_sched_p.k_lift_fuselage * eff_sched_var.airspeed2 / eff_sched_p.m;
//   float lift_d_tail = eff_sched_p.k_lift_tail * eff_sched_var.airspeed2 / eff_sched_p.m;

//   float lift_d = lift_d_wing + lift_d_fuselage + lift_d_tail;
//   if (eff_sched_var.wing_rotation_deg < 60.) {
//     lift_d = 0.0;
//   }
//   Bound(lift_d, -130., 0.);
//   eff_scheduling_rot_wing_lift_d = lift_d;
// }

// // Override standard LIFT_D function
// float guidance_indi_get_liftd(float pitch UNUSED, float theta UNUSED) {
//   return eff_scheduling_rot_wing_lift_d;
// }

// void eff_scheduling_rot_wing_update_rudder_effectiveness(void)
// {
//   float dMzdr = (eff_sched_p.k_rudder[0] * eff_sched_var.cmd_pusher_scaled * eff_sched_var.cmd_T_mean_scaled +
//                  eff_sched_p.k_rudder[1] * eff_sched_var.cmd_T_mean_scaled * eff_sched_var.airspeed2 * eff_sched_var.cosr +
//                  eff_sched_p.k_rudder[2] * eff_sched_var.airspeed2) / 10000.;

//   // Convert moment to effectiveness

//   float dMzdpprz = dMzdr * eff_sched_p.d_rudder_d_pprz;

//   // Convert moment to effectiveness
//   float eff_z_rudder = dMzdpprz / eff_sched_p.Izz;

//   Bound(eff_z_rudder, 0.000001, 0.1);

//   EFF_MAT_RW[2][4] = eff_z_rudder;
// }

// void eff_scheduling_rot_wing_update_pusher_effectiveness(void)
// {
//   float rpmP = eff_sched_p.k_rpm_pprz_pusher[0] + eff_sched_p.k_rpm_pprz_pusher[1] * eff_sched_var.cmd_pusher + eff_sched_p.k_rpm_pprz_pusher[2] * eff_sched_var.cmd_pusher * eff_sched_var.cmd_pusher;

//   float dFxdrpmP = eff_sched_p.k_pusher[0]*rpmP + eff_sched_p.k_pusher[1] * eff_sched_var.airspeed;
//   float drpmPdpprz = eff_sched_p.k_rpm_pprz_pusher[1] + 2. * eff_sched_p.k_rpm_pprz_pusher[2] * eff_sched_var.cmd_pusher;

//   float eff_pusher = (dFxdrpmP * drpmPdpprz / eff_sched_p.m) / 10000.;

//   Bound(eff_pusher, 0.00030, 0.0015);
//   EFF_MAT_RW[4][8] = eff_pusher;
// }

// void eff_scheduling_rot_wing_update_hover_motor_effectiveness(void)
// {
//   // Pitch motor effectiveness

//   float pitch_motor_q_eff = eff_sched_var.pitch_motor_dMdpprz / eff_sched_var.Iyy;

//   // Roll motor effectiveness
//   float dM_dpprz_right  = (eff_sched_p.DMdpprz_hover_roll[0] + actuator_state_filt_vect[1] * eff_sched_p.DMdpprz_hover_roll[1]) / 10000.;
//   float dM_dpprz_left   = (eff_sched_p.DMdpprz_hover_roll[0] + actuator_state_filt_vect[3] * eff_sched_p.DMdpprz_hover_roll[1]) / 10000.;

//   // Bound dM_dpprz to half and 2 times the hover effectiveness
//   Bound(dM_dpprz_right, eff_sched_var.roll_motor_dMdpprz * 0.5, eff_sched_var.roll_motor_dMdpprz * 2.0);
//   Bound(dM_dpprz_left,  eff_sched_var.roll_motor_dMdpprz * 0.5, eff_sched_var.roll_motor_dMdpprz * 2.0);

//   float roll_motor_p_eff_right = -(dM_dpprz_right * eff_sched_var.cosr + eff_sched_p.hover_roll_roll_coef[0] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.airspeed * eff_sched_var.cosr) / eff_sched_var.Ixx;
//   Bound(roll_motor_p_eff_right, -1, -0.00001);

//   float roll_motor_p_eff_left = (dM_dpprz_left * eff_sched_var.cosr + eff_sched_p.hover_roll_roll_coef[0] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.airspeed * eff_sched_var.cosr) / eff_sched_var.Ixx;
//   float roll_motor_airspeed_compensation = eff_sched_p.hover_roll_roll_coef[1] * eff_sched_var.airspeed * eff_sched_var.cosr / eff_sched_var.Ixx;
//   roll_motor_p_eff_left += roll_motor_airspeed_compensation;
//   Bound(roll_motor_p_eff_left, 0.00001, 1);

//   float roll_motor_q_eff = (eff_sched_p.hover_roll_pitch_coef[0] * eff_sched_var.wing_rotation_rad + eff_sched_p.hover_roll_pitch_coef[1] * eff_sched_var.wing_rotation_rad * eff_sched_var.wing_rotation_rad * eff_sched_var.sinr) / eff_sched_var.Iyy;
//   Bound(roll_motor_q_eff, 0, 1);

//   // Update front pitch motor q effectiveness
//   EFF_MAT_RW[1][0] = pitch_motor_q_eff;   // pitch effectiveness front motor

//   // Update back motor q effectiveness
//   EFF_MAT_RW[1][2] = -pitch_motor_q_eff;  // pitch effectiveness back motor

//   // Update right motor p and q effectiveness
//   EFF_MAT_RW[0][1] = roll_motor_p_eff_right;   // roll effectiveness right motor (no airspeed compensation)
//   EFF_MAT_RW[1][1] = roll_motor_q_eff;    // pitch effectiveness right motor

//   // Update left motor p and q effectiveness
//   EFF_MAT_RW[0][3] = roll_motor_p_eff_left;  // roll effectiveness left motor
//   EFF_MAT_RW[1][3] = -roll_motor_q_eff;   // pitch effectiveness left motor
// }

// void eff_scheduling_rot_wing_update_aileron_effectiveness(void)
// {
//   float dMxdpprz = (eff_sched_p.k_aileron * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / (RW_G_SCALE * RW_G_SCALE).;
//   float eff_x_aileron = dMxdpprz / eff_sched_var.Ixx;
//   Bound(eff_x_aileron, 0, 0.005)
//   // EFF_MAT_RW[0][6] = eff_x_aileron;
// }

// void eff_scheduling_rot_wing_update_flaperon_effectiveness(void)
// {
//   float dMxdpprz = (eff_sched_p.k_flaperon * eff_sched_var.airspeed2 * eff_sched_var.sinr3) / 1000000.;
//   float eff_x_flap_aileron = dMxdpprz / eff_sched_var.Ixx;
//   Bound(eff_x_flap_aileron, 0, 0.005)
//   EFF_MAT_RW[0][7] = eff_x_flap_aileron;
// }

//void stabilization_indi_set_wls_settings(void)
//{
   //// Calculate the min and max increments
  //   for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
  //     u_min_stab_indi[i] = -MAX_PPRZ * act_is_servo[i];
  //     u_max_stab_indi[i] = MAX_PPRZ;
  //     u_pref_stab_indi[i] = act_pref[i];
  //     if (i == 5) {
  //       u_pref_stab_indi[i] = actuator_state_filt_vect[i]; // Set change in prefered state to 0 for elevator
  //       u_min_stab_indi[i] = 0; // cmd 0 is lowest position for elevator
  //     }
  // }
//}
