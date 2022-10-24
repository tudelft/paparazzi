/*
 * Copyright (C) 2021 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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
/**
 * @file "modules/ctrl/approach_moving_target.c"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * Approach a moving target (e.g. ship)
 */

#include "approach_moving_target.h"

#include "generated/modules.h"
#include "modules/core/abi.h"

//float amt_err_slowdown_gain = AMT_ERR_SLOWDOWN_GAIN;
float amt_err_slowdown_gain = 0.001;
bool allways_update_ship_wp = true;

float approach_moving_target_angle_deg;


#define DEBUG_AMT TRUE
#define SELFBUILDCONTROLLER TRUE
#define CYBERZOO
#include <stdio.h>

// settings how drone should approach the ship
struct Amt amt = {
  #ifdef CYBERZOO
  .distance = 4,     // [m], diagonal decent line to ship
  .speed = -0.5,      // [m/s], speed over descent line to ship, inverted because software looks from ship to drone
  .slope_ref = 15.0,  // [deg], slope descent line
  #else
  .distance = 40,     // [m], diagonal decent line to ship
  .speed = -1.5,      // [m/s], speed over descent line to ship, inverted because software looks from ship to drone
  .slope_ref = 19.471,  // [deg], slope descent line
  #endif
  .accel_gain = 1,
  .pos_gain = 1,    // was 200 [-], how aggresive drone tracks the descent line
  .psi_ref = 180.0,   // [deg], descent line direction offset w.r.t. heading ship
  .speed_gain = 1.0,  // [-], how agressive ..................
  .relvel_gain = 0.75, // [-], ................................
  .approach_speed_gain = 1.0,
  .enabled_time = 0,
  .wp_ship_id = 0,
  .wp_approach_id = 0,
  .target_heigth = 0
};

// settings how drone should approach the ship
struct Wave WaveInfl = {
  .RollPitchYaw.x = 0,        // [deg]  the attitude of the deck of the ship
  .RollPitchYaw.y = 0,        // [deg]  the attitude of the deck of the ship
  .RollPitchYaw.z = 0,        // [deg]  the attitude of the deck of the ship
  .WaveOff2StaticPos.x = 0,   // [m]    the offset from the average/static position of the ship
  .WaveOff2StaticPos.y = 0,   // [m]    the offset from the average/static position of the ship
  .WaveOff2StaticPos.z = 0,   // [m]    the offset from the average/static position of the ship
  .estmInterval = 0,          // [sec]  the estimated interval in beteween the waves
  .estmAmp = 0,               // [m]    the estimated amplitude of the waves
  .certainty = 0              // [%]    how certain the system is of the wave shape/interval
};

struct AmtTelem amt_telem;

bool approach_moving_target_enabled = false; // NOT USED

static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);

struct FloatVect3 nav_get_speed_sp_from_diagonal(struct EnuCoor_i target, float pos_gain, float rope_heading);
void update_waypoint(uint8_t wp_id, struct FloatVect3 * target_ned);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_approach_moving_target(struct transport_tx *trans, struct link_device *dev)
{
  // pprz_msg_send_APPROACH_MOVING_TARGET(trans, dev, AC_ID,
  //                             &amt_telem.des_pos.x,
  //                             &amt_telem.des_pos.y,
  //                             &amt_telem.des_pos.z,
  //                             &amt_telem.des_vel.x,
  //                             &amt_telem.des_vel.y,
  //                             &amt_telem.des_vel.z,
  //                             &amt.distance
  //                             );
  int32_t enabled_time_diff = (get_sys_time_msec() - amt.enabled_time);
  
   pprz_msg_send_APPROACH_MOVING_TARGET(trans, dev, AC_ID,
                              &amt_telem.des_pos.x,
                              &amt_telem.des_pos.y,
                              &amt_telem.des_pos.z,
                              &amt_telem.des_vel.x,
                              &amt_telem.des_vel.y,
                              &amt_telem.des_vel.z,

                              &amt_telem.target_pos.x,
                              &amt_telem.target_pos.y,
                              &amt_telem.target_pos.z,
                              &amt_telem.target_vel.x,
                              &amt_telem.target_vel.y,
                              &amt_telem.target_vel.z,

                              &amt_telem.start_distance,
                              &amt.distance,
                              &amt_telem.approach_speed,
                              &amt.slope_ref,
                              &force_forward,
                              &enabled_time_diff
                              );
}
#endif

struct LlaCoor_i gps_lla;

void approach_moving_target_init(void)
{
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

  amt_telem.start_distance = amt.distance;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_APPROACH_MOVING_TARGET, send_approach_moving_target);
#endif
}

/**
 * Receive a RC_4CH message from the ground
 * TODO: this function needs a better place in a more general script
 */
void target_parse_RC_4CH(uint8_t *buf)
{
  int value = DL_RC_4CH_rotating_knob(buf);
  if (SELFBUILDCONTROLLER){
    if (value < 50)       value = 0;
    else if (value > 150) value = 100;
    else                  value = 200;
    amt.approach_speed_gain = (float)value/200; // 0.0 to 1.0
  }
  else{
    // Save the receivd rotating knob value
    //amt.approach_speed_gain = (float)value/100; // 0.0 to 2.0
    amt.approach_speed_gain = (float)value/200; // 0.0 to 1.0
    //printf("knop value: %i \n", value);
  }
}


/* Update INS (internal navigation system) based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
  gps_lla.lat = gps_s->lla_pos.lat;
  gps_lla.lon = gps_s->lla_pos.lon;
  gps_lla.alt = gps_s->lla_pos.alt;
}

// interface with ship position module?

// Update a waypoint such that you can see on the GCS where the drone wants to go
void update_waypoint(uint8_t wp_id, struct FloatVect3 * target_ned) {

  // Update the waypoint
  struct EnuCoor_f target_enu;
  ENU_OF_TO_NED(target_enu, *target_ned);
  waypoint_set_enu(wp_id, &target_enu);

  // Send waypoint update every 0.2 second !!!!!! THIS ONE SEEMD NOT BE USED, if time is long, nothing is different in GCS sim
  //RunOnceEvery(0.2, {
    // Send to the GCS that the waypoint has been moved
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                                &waypoints[wp_id].enu_i.x,
                                &waypoints[wp_id].enu_i.y,
                                &waypoints[wp_id].enu_i.z);
  //} );
}

void init_wp_ids(uint8_t wp_ship_id, uint8_t wp_approach_id){
  amt.wp_ship_id = wp_ship_id;
  amt.wp_approach_id = wp_approach_id;
}

// Function to enable from flight plan (call repeatedly!)
void approach_moving_target_enable(void) { 
  amt.enabled_time = get_sys_time_msec(); // this makes folow_diagonal_approach()
}

// Function to enable from flight plan (call repeatedly!)
void reset_moving_target_distance(void) { 
  amt.distance = amt_telem.start_distance;
}


/*
void waveEstimation(void){
  const int filter_length = 20;
  static bool filter_full = false;
  static int filter_loopcounter = 0;
  //static struct FloatVect3 bufRPY[filter_length];
  //static struct FloatVect3 bufOff[filter_length];
  
  struct FloatVect3 dummyWaveRPY;
  dummyWaveRPY.x = 20; //roll
  dummyWaveRPY.y = 5;  //pitch
  dummyWaveRPY.z = 10;  //yaw

  // WaveInfl.bufRPY[filter_loopcounter] = dummyWaveRPY;

  VECT3_COPY(WaveInfl.bufRPY[filter_loopcounter], dummyWaveRPY)

  //avg calc
  struct FloatVect3 sumBufRPY;
  for (int i=0; i <filter_length; i++){
    VECT3_ADD(sumBufRPY, WaveInfl.bufRPY[i]);
  }
  struct FloatVect3 avgBufRPY;
  printf("avgBuf %.1f %.1f %.1f \n",  sumBufRPY.x,  sumBufRPY.y,  sumBufRPY.z);
  VECT3_SDIV(avgBufRPY, sumBufRPY, filter_length);

  printf("avgBuf %.1f %.1f %.1f \n",  avgBufRPY.x,  avgBufRPY.y,  avgBufRPY.z);

  Bound(WaveInfl.certainty, 0, 100);
  filter_loopcounter++;
  if (filter_loopcounter >=filter_length) filter_loopcounter = 0;
}*/



/**
 * @brief Generates a velocity reference from a diagonal approach path
 *
 */
void follow_diagonal_approach(void) {
  /* STEPS
    - check if approach_moving_target_enable() is called recently by flightplan (otherwise return)
    - get target pos, vel, hdg (target is the landing spot)
    - calculate descent line unit vector (is it nesceserry to calculate this over and over every loop?)
    - 
  */
  uint32_t start_time = get_sys_time_msec();

  // waveEstimation(); // TESTING // NOT WORKING YET

  // Check if the flight plan recently called the enable function
  if ( (get_sys_time_msec() - amt.enabled_time) > (2000 / NAVIGATION_FREQUENCY) && !allways_update_ship_wp) {
    return; // if approach_moving_target_enable is not called recently
  }

  // Get the target position, velocity and heading
  struct NedCoor_f rel_target_pos, target_vel_boat = {0};
  float target_heading;
  if(!target_get_pos(&rel_target_pos, &target_heading)) {
    // TODO: What to do? Same can be checked for the velocity
    return;
  }

  target_get_vel(&target_vel_boat); // [m/s] update ground speed of the ship
  VECT3_SMUL(target_vel_boat, target_vel_boat, amt.speed_gain);

  // Reference model (descent line)
  // Translate angles to unit vector
  float gamma_ref = RadOfDeg(amt.slope_ref); // descent rate 
  float psi_ref = RadOfDeg(target_heading + amt.psi_ref); // how descent line lines up with ship heading
  amt.rel_unit_vec.x = cosf(gamma_ref) * cosf(psi_ref);
  amt.rel_unit_vec.y = cosf(gamma_ref) * sinf(psi_ref); 
  amt.rel_unit_vec.z = -sinf(gamma_ref);

  // Multiply vector with prefered descent dist to get ref point of descent
  // Desired position = rel_pos + target_pos_boat ??????????????
  struct FloatVect3 ref_relpos;
  VECT3_SMUL(ref_relpos, amt.rel_unit_vec, amt.distance); //calculate decscent point

  // Add ref point of descent to ship location to get NED coordinate ref point of descent
  // ATTENTION, target_pos_boat is already relative now!
  struct FloatVect3 rel_des_pos;
  VECT3_SUM(rel_des_pos, ref_relpos, rel_target_pos); 
  //printf("ref_relpos.z %f \n", ref_relpos.z);


  // ------------------------------------------------------------------------- ADD MORE COMMENTS FROM HERE ON

  struct FloatVect3 ref_relvel;
  VECT3_SMUL(ref_relvel, amt.rel_unit_vec, amt.approach_speed_gain * amt.speed * amt.relvel_gain * (int)force_forward); 

  // error controller
  struct FloatVect3 pos_err;
  struct FloatVect3 ec_vel;
  struct NedCoor_f *drone_pos = stateGetPositionNed_f();
  // ATTENTION, target_pos_boat is already relative now!
  // VECT3_DIFF(pos_err, des_pos, *drone_pos);
  VECT3_COPY(pos_err, rel_des_pos);
  VECT3_SMUL(ec_vel, pos_err, amt.pos_gain);

  // TODO improve + test
  float integral_gain = 0.001;
  VECT3_ADD_SCALED(amt.steady_state_error, pos_err, (integral_gain/NAVIGATION_FREQUENCY));
  if (abs(pos_err.x) > 0.5) amt.steady_state_error.x = 0;
  if (abs(pos_err.y) > 0.5) amt.steady_state_error.y = 0;
  //if (abs(pos_err.x) < 0.005) amt.steady_state_error.x = 0;
  //if (abs(pos_err.y) < 0.005) amt.steady_state_error.y = 0;
  if ((pos_err.x <= 0) != (amt.steady_state_error.x <= 0)) amt.steady_state_error.x = 0;
  if ((pos_err.y <= 0) != (amt.steady_state_error.y <= 0)) amt.steady_state_error.y = 0;
  struct FloatVect3 integral_compenstation;
  VECT3_COPY(integral_compenstation, amt.steady_state_error);

  // desired velocity = rel_vel + target_vel_boat + error_controller(using NED position)
  //printf("ref_relvel.x: %f \t target_vel_boat.x: %f \t ec_vel.x: %f \n", ref_relvel.x, target_vel_boat.x, ec_vel.x);
  struct FloatVect3 des_vel = {
    ref_relvel.x + target_vel_boat.x + ec_vel.x, // + integral_compenstation.x,
    ref_relvel.y + target_vel_boat.y + ec_vel.y, // + integral_compenstation.y,
    ref_relvel.z + target_vel_boat.z + ec_vel.z,
  };

  // Bound vertical speed setpoint
  if(stateGetAirspeed_f() > 13.0) {
    Bound(des_vel.z, -4.0, 5.0);
  } else {
    Bound(des_vel.z, -nav_climb_vspeed, -nav_descend_vspeed);
  }

  vect_bound_in_2d(&des_vel, 5.0); 

  /*
  #ifdef CYBERZOO
  Bound(des_vel.x, -0.5, 0.5); // not sure if this works // TEST
  Bound(des_vel.y, -0.5, 0.5); // not sure if this works // TEST
  #else
  Bound(des_vel.x, -2, 2); // not sure if this works // TEST
  Bound(des_vel.y, -2, 2); // not sure if this works // TEST
  #endif
  */

  // TODO: read nav status/block inside this script
  // TODO: place this in a better place with a more robust if statement
  //if (!force_forward){
  if ((get_sys_time_msec() - amt.enabled_time) < 1000) { 
    //AbiSendMsgVEL_SP(VEL_SP_FCR_ID, &des_vel); 
    struct FloatVect3 des_accel;
    struct FloatVect3 current_vel_ned; 
    struct FloatVect3 err_speed;
    current_vel_ned.x = stateGetSpeedNed_f()->x;
    current_vel_ned.y = stateGetSpeedNed_f()->y;
    current_vel_ned.z = stateGetSpeedNed_f()->z;
    VECT3_DIFF(err_speed, des_vel, current_vel_ned);
    VECT3_SMUL(des_accel, err_speed, 1.0);
    //printf("ref_relvel.x: %f \t target_vel_boat.x: %f \t ec_vel.x: %f \t des_accel.x: %f \n", ref_relvel.x, target_vel_boat.x, ec_vel.x, des_accel.x);

    int flag = 1; // 0 is 2d, 1 is 3D
    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &des_accel);
  }


  /* limit the speed such that the vertical component is small enough
  * and doesn't outrun the vehicle
  */
  float min_speed;
  float sin_gamma_ref = sinf(gamma_ref);
  if (sin_gamma_ref > 0.05) {
    min_speed = (nav_descend_vspeed+0.1) / sin_gamma_ref;
  } else {
    min_speed = -5.0; // prevent dividing by zero
  }

  // The upper bound is not very important
  Bound(amt.speed, min_speed, 4.0);

  // Reduce approach speed if the error is large
  float norm_pos_err_sq = VECT3_NORM2(pos_err);
  // int_speed = (default speed / (squared position error [m] * slowdown factor + 1) * speed gain control by joystick
  amt_telem.approach_speed = ((amt.speed) / (norm_pos_err_sq * amt_err_slowdown_gain + 1.0)) * amt.approach_speed_gain * (int)force_forward;
  if (amt.distance < 20) amt_telem.approach_speed = amt_telem.approach_speed / 2;

  // Check if the flight plan recently called the enable function
  // make distance to ship smaller, So descent to ship

  if ( (get_sys_time_msec() - amt.enabled_time) < (2000 / NAVIGATION_FREQUENCY) && force_forward) {
    // integrate speed to get the distance
    //printf("moving_towards_ship \n");
    float dt = FOLLOW_DIAGONAL_APPROACH_PERIOD;
    amt.distance += amt_telem.approach_speed*dt;
    Bound(amt.distance, 0, 100); // approach dist > 0
    //amt.distance -= 1*dt;
  }
  

  // For display purposes
  struct FloatVect3 ned_pos_target;
  VECT3_SUM(ned_pos_target, rel_target_pos, *drone_pos);
  //struct FloatVect3 ne_up_pos_target; // North, East, UP (alt) for Paparazzi
  //VECT3_COPY(ne_up_pos_target, ned_pos_target);
  //ne_up_pos_target.z = ne_up_pos_target.z*-1;
  update_waypoint(amt.wp_ship_id, &ned_pos_target);
  
  // *drone_pos = NED float
  // des_pos = NED float
  // ned _pos_approach = NED float
  struct FloatVect3 ned_pos_approach; // WAS LLA instead of NED

  VECT3_SUM(ned_pos_approach, rel_des_pos, *drone_pos);

  update_waypoint(amt.wp_approach_id, &ned_pos_approach);
  // TEST TO SOLVE HEIGHT STEP PROBLEM
  struct EnuCoor_f target_enu;
  ENU_OF_TO_NED(target_enu, ned_pos_approach);
  amt.target_heigth = ned_pos_approach.x;
  // TEST TO SOLVE HEIGHT STEP PROBLEM

  // Update values for telemetry
  VECT3_COPY(amt_telem.des_pos, rel_des_pos); 
  
  VECT3_COPY(amt_telem.des_vel, des_vel);

  // Update values for telemetry
  VECT3_COPY(amt_telem.target_pos, rel_target_pos); 
  VECT3_COPY(amt_telem.target_vel, target_vel_boat);

  uint32_t end_time = get_sys_time_msec();
  //printf("loop_time = %i %i \n", end_time, start_time);
}
