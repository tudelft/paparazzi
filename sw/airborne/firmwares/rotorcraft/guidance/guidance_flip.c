/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2015 Ewoud Smeur
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_flip.c
 *
 * Open Loop guidance for making a flip. You need to tune this before using.
 * When entering this mode it saves the previous guidance mode and changes AUTO2 back to
 * the previous mode after finishing the flip.
 * Use it with caution!
 */

#include "guidance_flip.h"

#include "firmwares/rotorcraft/autopilot_firmware.h"
#include "guidance_h.h"
#include "stabilization/stabilization_attitude_rc_setpoint.h"
#include "stabilization/stabilization_attitude.h"

#include "paparazzi.h"
#include "math/pprz_algebra_int.h"
#include "subsystems/radio_control.h"


//// Single pitch/roll flip - reliable (light MXS Transformer)
//
//#define STOP_ACCELERATE_CMD_ANGLE 90
//#define START_DECELERATE_CMD_ANGLE 255.0
//#define START_RECOVER_CMD_ANGLE 255.0
//#define FIRST_THRUST_LEVEL 9000
//#define FIRST_THRUST_DURATION 0.4
//#define FINAL_THRUST_LEVEL 9000
//#define FINAL_THRUST_DURATION 0.5
//#define FLIP_PITCH 1

//// Single pitch/roll flip - for heavy FPV Transformer
//
//#define STOP_ACCELERATE_CMD_ANGLE 90
//#define START_DECELERATE_CMD_ANGLE 255.0
//#define START_RECOVER_CMD_ANGLE 255.0
//#define FIRST_THRUST_LEVEL 9000
//#define FIRST_THRUST_DURATION 0.7
//#define FINAL_THRUST_LEVEL 9000
//#define FINAL_THRUST_DURATION 0.9
//#define FLIP_ROLL 1

// Double roll flip - stil some overshoot
//
//#define STOP_ACCELERATE_CMD_ANGLE 550.0
//#define START_DECELERATE_CMD_ANGLE 560.0
//#define START_RECOVER_CMD_ANGLE 615.0
//#define FIRST_THRUST_LEVEL 9000
//#define FIRST_THRUST_DURATION 0.6
//#define FINAL_THRUST_LEVEL 9000
//#define FINAL_THRUST_DURATION 0.8
//#define FLIP_ROLL 1

// Single roll flip - some overshoot
//
//#define STOP_ACCELERATE_CMD_ANGLE 170
//#define START_DECELERATE_CMD_ANGLE 190.0
//#define START_RECOVER_CMD_ANGLE 270.0
//#define FIRST_THRUST_LEVEL 9000
//#define FIRST_THRUST_DURATION 0.6
//#define FINAL_THRUST_LEVEL 9000
//#define FINAL_THRUST_DURATION 0.8
//#define FLIP_ROLL 1


// Single pitch flip - decelerates too fast
//
//#define STOP_ACCELERATE_CMD_ANGLE 180
//#define START_DECELERATE_CMD_ANGLE 230.0
//#define START_RECOVER_CMD_ANGLE 270.0
//#define FIRST_THRUST_LEVEL 9000
//#define FIRST_THRUST_DURATION 0.6
//#define FINAL_THRUST_LEVEL 9000
//#define FINAL_THRUST_DURATION 0.8
//#define FLIP_PITCH 1

//// Fast throttle up
//
//#define STOP_ACCELERATE_CMD_ANGLE 1.0
//#define START_DECELERATE_CMD_ANGLE 1.0
//#define START_RECOVER_CMD_ANGLE 2.0
//#define FIRST_THRUST_LEVEL 9000
//#define FIRST_THRUST_DURATION 1.0
//#define FINAL_THRUST_LEVEL 9000
//#define FINAL_THRUST_DURATION 0.1

//// Evasive maneuver - roll & pitch, angular limit
//#define FIRST_THRUST_LEVEL 6500
//#define FIRST_THRUST_DURATION 0.0
//#define STRAIGHT_FLIGHT_DURATION 2.0
//#define STOP_EVADE_ANGLE 30.0
//#define FINAL_THRUST_LEVEL 6500
//#define FINAL_THRUST_DURATION 0.8
//#define EVADE_ROLL 1
//#define ROLL_DELAY 0.0
//#define PITCH_CMD_FINAL 0
//#define PITCH_CMD_NOMINAL -MAX_PPRZ*2/3

//// Evasive maneuver - roll & pitch, time limit
//#define FIRST_THRUST_LEVEL 6500
//#define FIRST_THRUST_DURATION 0.0
//#define STRAIGHT_FLIGHT_DURATION 1.0
//#define STOP_EVADE_TIME 0.25
//#define FINAL_THRUST_LEVEL 6500
//#define FINAL_THRUST_DURATION 0.8
//#define EVADE_ROLL_PITCH 1
//#define ROLL_DELAY 0.0
//#define PITCH_CMD_FINAL 0 //-MAX_PPRZ*1/4 //-MAX_PPRZ*1/3 // max pitch set to 60 deg!!!
//#define PITCH_CMD_NOMINAL -MAX_PPRZ*2/3 //-MAX_PPRZ*1/2 // -MAX_PPRZ*2/3


// Pitch doublets
#define FIRST_THRUST_LEVEL 6500
#define FIRST_THRUST_DURATION 0.0
#define STRAIGHT_FLIGHT_DURATION 10
#define DOUBLET_DURATION 1.0
#define FINAL_THRUST_LEVEL 6500
#define FINAL_THRUST_DURATION 0
#define PITCH_CMD_NOMINAL -MAX_PPRZ*15/60
#define PITCH_CMD_DELTA -MAX_PPRZ/4
#define PITCH_DOUBLET 1
#define DOUBLET_REPETITIONS 1

//// Roll doublets
//#define FIRST_THRUST_LEVEL 6500
//#define FIRST_THRUST_DURATION 0.0
//#define STRAIGHT_FLIGHT_DURATION 0.6
//#define DOUBLET_DURATION 1.0
//#define FINAL_THRUST_LEVEL 6500
//#define FINAL_THRUST_DURATION 0
//#define ROLL_CMD_NOMINAL -MAX_PPRZ*15/60
//#define ROLL_CMD_DELTA -MAX_PPRZ/4
//#define ROLL_DOUBLET 1
//#define DOUBLET_REPETITIONS 10

//// Pitch sweep
//#define FIRST_THRUST_LEVEL 6500
//#define FIRST_THRUST_DURATION 0.0
//#define SWEEP_DURATION 5
//#define FINAL_THRUST_LEVEL 6500
//#define FINAL_THRUST_DURATION 0
//#define PITCH_CMD_DELTA -MAX_PPRZ
//#define PITCH_SWEEP 1

// default values
#ifndef STOP_ACCELERATE_CMD_ANGLE
#define STOP_ACCELERATE_CMD_ANGLE 180.0
#endif

#ifndef START_DECELERATE_CMD_ANGLE
#define START_DECELERATE_CMD_ANGLE 230.0 //190.0 //255.0 // 615 //-115.0
#endif

#ifndef START_RECOVER_CMD_ANGLE
#define START_RECOVER_CMD_ANGLE 270.0 // 270.0 //255.0 // 615 //-115.0
#endif

#ifndef FINAL_THRUST_LEVEL
#define FINAL_THRUST_LEVEL 2000
#endif
#ifndef FINAL_THRUST_DURATION
#define FINAL_THRUST_DURATION 0.8
#endif


#ifndef STRAIGHT_FLIGHT_DURATION
#define STRAIGHT_FLIGHT_DURATION 0.0
#endif
#ifndef DOUBLET_DURATION
#define DOUBLET_DURATION 0.0
#endif
#ifndef DOUBLET_REPETITIONS
#define DOUBLET_REPETITIONS 1
#endif
#ifndef PITCH_CMD_NOMINAL
#define PITCH_CMD_NOMINAL 0
#endif
#ifndef PITCH_CMD_FINAL
#define PITCH_CMD_FINAL 0
#endif
#ifndef PITCH_CMD_DELTA
#define PITCH_CMD_DELTA -MAX_PPRZ/3
#endif

#ifndef ROLL_CMD_NOMINAL
#define ROLL_CMD_NOMINAL 0
#endif
#ifndef ROLL_CMD_FINAL
#define ROLL_CMD_FINAL 0
#endif
#ifndef ROLL_CMD_DELTA
#define ROLL_CMD_DELTA -MAX_PPRZ/3
#endif


#ifndef SWEEP_DURATION
#define SWEEP_DURATION 0
#endif

#ifndef STOP_EVADE_ANGLE
#define STOP_EVADE_ANGLE 30.0
#endif

#ifndef STOP_EVADE_TIME
#define STOP_EVADE_TIME 0.25
#endif


#ifndef FLIP_PITCH
#define FLIP_PITCH 0
#endif
#ifndef FLIP_ROLL
#define FLIP_ROLL 0
#endif
#ifndef EVADE_ROLL
#define EVADE_ROLL 0
#endif
#ifndef EVADE_ROLL_PITCH
#define EVADE_ROLL_PITCH 0
#endif

#ifndef PITCH_DOUBLET
#define PITCH_DOUBLET 0
#endif

#ifndef ROLL_DOUBLET
#define ROLL_DOUBLET 0
#endif

#ifndef PITCH_SWEEP
#define PITCH_SWEEP 0
#endif

#ifndef ROLL_DELAY
#define ROLL_DELAY 0.0
#endif

uint8_t in_flip;
pprz_t auto_pitch = 0;
pprz_t auto_roll = 0;

uint32_t flip_counter;
uint32_t doublet_cnt;
uint8_t sequence_cnt;
uint8_t flip_state;
int32_t heading_save;
uint8_t autopilot_mode_old;
struct Int32Vect2 flip_cmd_earth;

int32_t phi_gyr, theta_gyr;

float timer_fl;

void guidance_flip_enter(void)
{
  flip_counter = 0;
  doublet_cnt = 0;
  flip_state = 0;
  heading_save = stabilization_attitude_get_heading_i();
//  autopilot_mode_old = autopilot_get_mode();
  phi_gyr = 0;
  theta_gyr = 0;
  in_flip = 0;
  auto_pitch = 0;
  auto_roll = 0;
}

void guidance_flip_run(void)
{
  uint32_t timer;
  int32_t phi, theta, p, q; //phiq, thetaq, qi, qx, qy, qz;
  static uint32_t timer_save = 0;
  //  struct Int32Quat q, qg, qprod;

  timer = (flip_counter++ << 12) / PERIODIC_FREQUENCY;
  phi = stateGetNedToBodyEulers_i()->phi;
  theta = stateGetNedToBodyEulers_i()->theta;

  p = stateGetBodyRates_i()->p;
  q = stateGetBodyRates_i()->q;

  //  qi = stateGetNedToBodyQuat_i()->qi;
  //  qx = stateGetNedToBodyQuat_i()->qx;
  //  qy = stateGetNedToBodyQuat_i()->qy;
  //  qz = stateGetNedToBodyQuat_i()->qz;
  //  q = stateGetNedToBodyQuat_i();

  // quaternion multiplication
  // int32_quat_comp(struct Int32Quat *a2c, struct Int32Quat *a2b, struct Int32Quat *b2c)

  //  phiq = 2*int32_atan2(stateGetNedToBodyQuat_i()->qx, stateGetNedToBodyQuat_i()->qi);
  //  thetaq=2*int32_atan2(stateGetNedToBodyQuat_i()->qy, stateGetNedToBodyQuat_i()->qi);


  switch (flip_state) {

    //----------------------------------------------------------------------------------------------------------------------
    //---GAIN-HEIGHT--------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------

    case 0:
      in_flip = 1;

      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      stabilization_attitude_set_earth_cmd_i(&flip_cmd_earth,
                                             heading_save);
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST] = FIRST_THRUST_LEVEL; //Thrust to go up first
      timer_save = 0;

      if (timer >= BFP_OF_REAL(FIRST_THRUST_DURATION, 12)) {
        if (FLIP_ROLL && ~FLIP_PITCH) {
          phi_gyr = phi; // initialize the phi estimate with the current phi
          flip_state = 1;
        }
        else if (FLIP_PITCH && ~FLIP_ROLL) {
          theta_gyr = theta; // initialize the theta estimate with the current theta
          flip_state = 11;
        }
        else if (EVADE_ROLL) {
          flip_state = 21;
        }
        else if (EVADE_ROLL_PITCH) {
          flip_state = 31;
        }
        else if (PITCH_DOUBLET) {
          flip_state = 41;
          doublet_cnt = 2;
          sequence_cnt = 1;
        }
        else if (PITCH_SWEEP) {
          flip_state = 51;
        }
        else flip_state = 101; // return to attitude mode
      }
      break;

    //----------------------------------------------------------------------------------------------------------------------
    //---ROLL-FLIP----------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------

    case 1:
      stabilization_cmd[COMMAND_ROLL]   = 7100; // Rolling command
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 6050; // 5600 // --> Left (5600-8000/2) = 1600, right --> (5600+8000/2) = 9600

      // Integrate gyros for angle estimates
      phi_gyr += p/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (phi_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ACCELERATE_CMD_ANGLE))) {
        flip_state++;
      }
      break;

    case 2:
      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 2500; //1600; // Min. thrust, so that none of the wings stops flapping

      // Integrate gyros for angle estimates
      phi_gyr += p/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (phi_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(START_DECELERATE_CMD_ANGLE))) { // && phi < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ACCELERATE_CMD_ANGLE))) {
        timer_save = timer;
        flip_state++;
      }
      break;

    case 3:
      stabilization_cmd[COMMAND_ROLL]   = -7100;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 6050; // 5600 // --> Left (5600-8000/2) = 1600, right --> (5600+8000/2) = 9600

      // Integrate gyros for angle estimates
      phi_gyr += p/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (phi_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(START_RECOVER_CMD_ANGLE))) { // && phi < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ACCELERATE_CMD_ANGLE))) {
        timer_save = timer;
        flip_state = 100;
      }
      break;

    //----------------------------------------------------------------------------------------------------------------------
    //---PITCH-FLIP---------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------

    case 11:
      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = -9600; // Pitching command
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 9000; // Max. thrust

      // Integrate gyro for pitch estimate
      theta_gyr += -q/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (theta_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ACCELERATE_CMD_ANGLE))) {
    	  flip_state++;
      }
      break;

    case 12:
      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 2500; //1600 //Min thrust?

      // Integrate gyro for pitch estimate
      theta_gyr += -q/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (theta_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(START_DECELERATE_CMD_ANGLE))) { // && theta < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ACCELERATE_CMD_ANGLE))) {
        timer_save = timer;
        flip_state++;
      }
      break;

    case 13:
         stabilization_cmd[COMMAND_ROLL]   = 0;
         stabilization_cmd[COMMAND_PITCH]  = 9600;
         stabilization_cmd[COMMAND_YAW]    = 0;
         stabilization_cmd[COMMAND_THRUST] = 9000; //1600 //Min thrust?

         // Integrate gyro for pitch estimate
         theta_gyr += -q/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

         if (theta_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(START_RECOVER_CMD_ANGLE))) { // && theta < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ACCELERATE_CMD_ANGLE))) {
           timer_save = timer;
           flip_state = 100;
         }
         break;

    //----------------------------------------------------------------------------------------------------------------------
    //---EVADE-ROLL---------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------
    case 21:
         // straight flight
         auto_pitch = PITCH_CMD_NOMINAL;
         stabilization_attitude_run(autopilot_in_flight());
         stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

         if (timer > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
            phi_gyr = phi; // initialize the phi estimate with the current phi
            flip_state++;
            auto_pitch = 0;
			timer_save = timer;
         }
         break;

    case 22:
         // Open loop manoeuver

    	 // Science video: R7000, P2000 ~ 90deg, more pitch/less roll for sharper turns
         if (timer >= BFP_OF_REAL(ROLL_DELAY, 12)) {
             stabilization_cmd[COMMAND_ROLL]   = 7000; // Rolling command (max 7100 with 6050 thrust cmd)
         } else {
         	 stabilization_cmd[COMMAND_ROLL]   = 0;
		 }
         stabilization_cmd[COMMAND_PITCH]  = 2000; //4000; //9600;
         stabilization_cmd[COMMAND_YAW]    = 0;
         stabilization_cmd[COMMAND_THRUST] = 6050; // 5600 // --> Left (5600-8000/2) = 1600, right --> (5600+8000/2) = 9600

         // Integrate gyros for angle estimates
         phi_gyr += p/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

         if (phi_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(STOP_EVADE_ANGLE))) {
             if (PITCH_CMD_FINAL == 0) {
               flip_state = 100;
               auto_pitch = 0;
             }
             else {
               flip_state = 23;
               auto_pitch = PITCH_CMD_FINAL;
             }
			 timer_save = timer;
         }
         break;

    case 23:
          // recovery with straight flight
          auto_pitch = PITCH_CMD_FINAL;
          stabilization_attitude_run(autopilot_in_flight());
          stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];
          stabilization_cmd[COMMAND_YAW] = 0; // no yaw feedback also during the recovery

          stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
          // reset yaw stabilization loop
          att_ref_euler_i.euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
          att_ref_euler_i.rate.r = 0;
          att_ref_euler_i.accel.r = 0;
          stabilization_att_sum_err.psi = 0;

          if ((timer - timer_save) > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
            flip_state = 100;
            timer_save = timer;
            auto_pitch = 0;
          }
          break;

          //----------------------------------------------------------------------------------------------------------------------
          //---EVADE-ROLL-PITCH---------------------------------------------------------------------------------------------------
          //----------------------------------------------------------------------------------------------------------------------
    case 31:
      // straight flight
      auto_pitch = PITCH_CMD_NOMINAL;
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

      if (timer > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
        phi_gyr = phi; // initialize the phi estimate with the current phi
        flip_state++;
        auto_pitch = 0;
        timer_save = timer;
      }
      break;

    case 32:
      // Open loop manoeuver
      if (timer >= BFP_OF_REAL(ROLL_DELAY, 12)) {
        stabilization_cmd[COMMAND_ROLL]   = 2730; // Rolling command (max 7100 with 6050 thrust cmd)
      } else {
        stabilization_cmd[COMMAND_ROLL]   = 0;
      }
      stabilization_cmd[COMMAND_PITCH]  = 4395;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = radio_control.values[RADIO_THROTTLE]; //6050; // 5600 // --> Left (5600-8000/2) = 1600, right --> (5600+8000/2) = 9600

      // Integrate gyros for angle estimates
      phi_gyr += p/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if ((timer - timer_save) > BFP_OF_REAL(STOP_EVADE_TIME, 12)) {
        if (PITCH_CMD_FINAL == 0) {
          flip_state = 100;
          auto_pitch = 0;
        }
        else {
          flip_state = 33;
          auto_pitch = PITCH_CMD_FINAL;
        }
        timer_save = timer;
      }
      break;

    case 33:
      // recovery with straight flight
      auto_pitch = PITCH_CMD_FINAL;
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];
      stabilization_cmd[COMMAND_YAW] = 0; // no yaw feedback also during the recovery

      stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
      // reset yaw stabilization loop
      att_ref_euler_i.euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
      att_ref_euler_i.rate.r = 0;
      att_ref_euler_i.accel.r = 0;
      stabilization_att_sum_err.psi = 0;

      // start turning after a delay
      if ((timer - timer_save) > BFP_OF_REAL(0.6, 12)) {
        stabilization_cmd[COMMAND_YAW] = 0;
      }

      if ((timer - timer_save) > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
        flip_state = 100;
        timer_save = timer;
        auto_pitch = 0;
      }
      break;

    //----------------------------------------------------------------------------------------------------------------------
    //---PITCH-DOUBLET------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------
    case 41:
      // straight flight
      auto_pitch = PITCH_CMD_NOMINAL; //-MAX_PPRZ*2/3;
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

      if ((timer - timer_save) > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
        flip_state++;
        timer_save = timer;
      }
      break;
    case 42:
      // doublet
      auto_pitch = PITCH_CMD_NOMINAL + PITCH_CMD_DELTA; //-MAX_PPRZ*2/3;
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

      if ((timer - timer_save) > BFP_OF_REAL(DOUBLET_DURATION/doublet_cnt, 12)) {
        flip_state++;
        timer_save = timer;
        doublet_cnt++;
      }
      break;

    case 43:
      // doublet
      auto_pitch = PITCH_CMD_NOMINAL - PITCH_CMD_DELTA; //-MAX_PPRZ*2/3;
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

      if ((timer - timer_save) > BFP_OF_REAL(DOUBLET_DURATION/doublet_cnt, 12)) {
        if (doublet_cnt > DOUBLET_REPETITIONS) {
          if (sequence_cnt == 2) {
            flip_state++;
          }
          else {
            flip_state = 42;
            doublet_cnt = 2;
            sequence_cnt++;
          }
        }
        else {
          flip_state = 42;
          doublet_cnt++;
        }
        timer_save = timer;
        }
      break;

    case 44:
      // doublet
      auto_pitch = PITCH_CMD_NOMINAL; //-MAX_PPRZ*2/3;
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

      if ((timer - timer_save) > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
        flip_state = 100;
        timer_save = timer;
        auto_pitch = 0;
      }
      break;

      //----------------------------------------------------------------------------------------------------------------------
      //---PITCH-SWEEP------------------------------------------------------------------------------------------------------
      //----------------------------------------------------------------------------------------------------------------------
    case 51:
      timer_fl = (timer - timer_save) / (1<<12);
      auto_pitch = PITCH_CMD_DELTA*sinf(3*timer_fl); //-MAX_PPRZ*2/3;
      stabilization_attitude_run(autopilot_in_flight());
      stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

      if (timer > BFP_OF_REAL(SWEEP_DURATION, 12)) {
        flip_state=100;
        auto_pitch = 0;
      }
      break;


      //----------------------------------------------------------------------------------------------------------------------
      //---ROLL-DOUBLET------------------------------------------------------------------------------------------------------
      //----------------------------------------------------------------------------------------------------------------------
      case 61:
        // straight flight
        auto_roll = ROLL_CMD_NOMINAL; //-MAX_PPRZ*2/3;
        stabilization_attitude_run(autopilot_in_flight());
        stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

        if ((timer - timer_save) > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
          flip_state++;
          timer_save = timer;
        }
        break;
      case 62:
        // doublet
        auto_roll = ROLL_CMD_NOMINAL + ROLL_CMD_DELTA; //-MAX_PPRZ*2/3;
        stabilization_attitude_run(autopilot_in_flight());
        stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

        if ((timer - timer_save) > BFP_OF_REAL(DOUBLET_DURATION/doublet_cnt, 12)) {
          flip_state++;
          timer_save = timer;
          doublet_cnt++;
        }
        break;

      case 63:
        // doublet
        auto_roll = ROLL_CMD_NOMINAL - ROLL_CMD_DELTA; //-MAX_PPRZ*2/3;
        stabilization_attitude_run(autopilot_in_flight());
        stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

        if ((timer - timer_save) > BFP_OF_REAL(DOUBLET_DURATION/doublet_cnt, 12)) {
          if (doublet_cnt > DOUBLET_REPETITIONS) {
            if (sequence_cnt == 2) {
              flip_state++;
            }
            else {
              flip_state = 62;
              doublet_cnt = 2;
              sequence_cnt++;
            }
          }
          else {
            flip_state = 62;
            doublet_cnt++;
          }
          timer_save = timer;
          }
        break;

      case 64:
        // doublet
        auto_roll = ROLL_CMD_NOMINAL; //-MAX_PPRZ*2/3;
        stabilization_attitude_run(autopilot_in_flight());
        stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

        if ((timer - timer_save) > BFP_OF_REAL(STRAIGHT_FLIGHT_DURATION, 12)) {
          flip_state = 100;
          timer_save = timer;
          auto_roll = 0;
        }
        break;

    //----------------------------------------------------------------------------------------------------------------------
    //---RECOVER------------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------

    case 100: // recovery with stabilization
      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      stabilization_attitude_set_earth_cmd_i(&flip_cmd_earth,heading_save);
      stabilization_attitude_run(autopilot_in_flight());

      if (EVADE_ROLL || EVADE_ROLL_PITCH) {
         stabilization_cmd[COMMAND_YAW] = 0; // no yaw feedback also during the recovery
         stabilization_cmd[COMMAND_THRUST]=radio_control.values[RADIO_THROTTLE];

         stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
         // reset yaw stabilization loop
         att_ref_euler_i.euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
         att_ref_euler_i.rate.r = 0;
         att_ref_euler_i.accel.r = 0;
         stabilization_att_sum_err.psi = 0;
      } else {
        stabilization_cmd[COMMAND_THRUST] = FINAL_THRUST_LEVEL; //Thrust to stop falling
      }

      if ((timer - timer_save) > BFP_OF_REAL(FINAL_THRUST_DURATION, 12)) {
		     flip_state++;
		  }
		  break;

    default:

//      autopilot_mode_auto2 = autopilot_mode_old;
//      autopilot_set_mode(autopilot_mode_old);

      autopilot_mode_auto2 = AP_MODE_ATTITUDE_DIRECT;
      autopilot_set_mode(AP_MODE_ATTITUDE_DIRECT);

      in_flip = 0;
      auto_pitch = 0;

      if (EVADE_ROLL || EVADE_ROLL_PITCH) {
        stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();
        // reset yaw stabilization loop
        att_ref_euler_i.euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
        att_ref_euler_i.rate.r = 0;
        att_ref_euler_i.accel.r = 0;
        stabilization_att_sum_err.psi = 0;
      } else {
        stab_att_sp_euler.psi = heading_save;
      }

      flip_counter = 0;
      timer_save = 0;
      flip_state = 0;

      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 8000; //Some thrust to come out of the roll?
      break;
  }
}
