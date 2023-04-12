/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
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

/** @file "modules/potential_flow_simulator/potential_flow_simulator.c"
 * @author Sunyou Hwang <S.Hwang-1@tudelft.nl>
 * Potential flow simulator for orographic soaring (Never Landing Drone project)

 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <iostream>
#include <time.h>

//extern "C" {
#include "modules/potential_flow_simulator/potential_flow_simulator.h"
#include "state.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"
#include "modules/datalink/telemetry.h"
#ifdef SIM
    #include "nps_atmosphere.h"
#endif
//#include "modules/ctrl/follow_me.h"
//}

#if PF_SET_GRADIENT
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid_nld_soaring.h"
#endif

//using namespace std;

// define radius of the obstacle - TODO: building and boat?
#ifndef PF_OBSTACLE_RADIUS
#define PF_OBSTACLE_RADIUS 40
#endif
// surface roughness
#ifndef PF_SURFACE_ROUGHNESS
#define PF_SURFACE_ROUGHNESS 0.05
#endif
// Initial obstacle direction (starting from North, in degrees)
#ifndef PF_INITIAL_OBSTACLE_DIR
#define PF_INITIAL_OBSTACLE_DIR 0
#endif
// type of the obstacle (0: boat, 1: hill, 2: building, ...)
// for boat, use ground gps. for others, set heading angle and position (center point)
#ifndef PF_OBSTACLE_TYPE
#define PF_OBSTACLE_TYPE 1
#endif
#ifndef PF_OBSTACLE_POSITION_X
#define PF_OBSTACLE_POSITION_X 230
#endif
#ifndef PF_OBSTACLE_POSITION_Y
#define PF_OBSTACLE_POSITION_Y 0
#endif
#ifndef PF_OBSTACLE_POSITION_Z
#define PF_OBSTACLE_POSITION_Z 0
#endif
// reference wind velocity: x dir in the obstacle's body frame
#ifndef PF_REF_WIND_SPD
#define PF_REF_WIND_SPD -12
#endif
// wind direction from north CW in degrees
#ifndef PF_REF_WIND_DIR
#define PF_REF_WIND_DIR 180
#endif
// whether use ground gps or not
#ifndef PF_USE_GROUND_GPS
#define PF_USE_GROUND_GPS FALSE
#endif
// set wind to nps environment?
#ifndef PF_SET_WIND_NPS_ENV
#define PF_SET_WIND_NPS_ENV FALSE
#endif

#ifndef PF_DEFAULT_HEADING
#define PF_DEFAULT_HEADING 0
#endif

#ifndef PF_PREVENT_DOWNLIFT
#define PF_PREVENT_DOWNLIFT FALSE
#endif

//struct FloatVect3 calc_relative_position(struct UtmCoor_f *utm_vehicle, struct UtmCoor_f *utm_ground);
//struct LlaCoor_d *to_lla_d(struct LlaCoor_f *_lla_f);
//struct FloatVect3 rotate_frame(struct FloatVect3 *point, float theta);
//#if PF_USE_GROUND_GPS
//void parse_ground_gps(uint8_t *buf);
//void follow_me_set_heading(void);
//float average_heading(float diffx, float diffy);
//#endif

float ref_wind_spd = PF_REF_WIND_SPD;
float ref_wind_dir = PF_REF_WIND_DIR;

struct NedCoor_f *ltp_vehicle_pos;
struct FloatVect3 rel_dist = {0., 0., 0.};
struct FloatVect3 wind_vel_v3f = {0., 0., 0.};
struct FloatVect3 wind_grad_v3f = {0., 0., 0.};

float grad_spd_sp_x = 0.;
float grad_spd_sp_z = 0.;

float obstacle_position_north = PF_OBSTACLE_POSITION_X;
float obstacle_position_east = PF_OBSTACLE_POSITION_Y;
float obstacle_position_down = PF_OBSTACLE_POSITION_Z;

float obstacle_radius = PF_OBSTACLE_RADIUS;

struct FloatVect3 compute_potential_flow(struct FloatVect3 rel_dist_v3f, float ref_wind_speed);
struct FloatVect3 compute_wind_gradient(struct FloatVect3 rel_dist_v3f, float ref_wind_speed);

inline struct FloatVect3 calc_relative_position_utm(struct UtmCoor_f *utm_vehicle, struct UtmCoor_f *utm_ground){
    struct FloatVect3 rel_pos;
    rel_pos.x = utm_ground->east - utm_vehicle->east;
    rel_pos.y = utm_ground->north - utm_vehicle->north;
    rel_pos.z = utm_ground->alt - utm_vehicle->alt;
    return rel_pos;
}
inline struct FloatVect3 calc_relative_position_v3(struct FloatVect3 *utm_vehicle, struct FloatVect3 *utm_ground){
    struct FloatVect3 rel_pos;
    rel_pos.x = utm_ground->x - utm_vehicle->x;
    rel_pos.y = utm_ground->y - utm_vehicle->y;
    rel_pos.z = utm_ground->z - utm_vehicle->z;
    return rel_pos;
}

inline struct FloatVect3 rotate_frame(struct FloatVect3 *point, float theta){
    // Create return Vector for function
    struct FloatVect3 transformation;

    // Rotate point
    transformation.x = cosf(theta)*point->x + sinf(theta)*point->y;
    transformation.y = -sinf(theta)*point->x + cosf(theta)*point->y;
    transformation.z = point->z;

    // Return
    return transformation;
}

inline struct LlaCoor_d to_lla_d(struct LlaCoor_f *_lla_f) {
    struct LlaCoor_d _lla_d;
    _lla_d.lat = _lla_f->lat;
    _lla_d.lon = _lla_f->lon;
    _lla_d.alt = _lla_f->alt;
    return _lla_d;
}

// utm: ENU -> NED
//inline struct FloatVect3 utm_to_v3(struct UtmCoor_d *_utm_d) {
//    struct FloatVect3 _fv3;
//    _fv3.x = _utm_d->north;
//    _fv3.y = _utm_d->east;
//    _fv3.z = -_utm_d->alt;
//    return _fv3;
//}

inline struct FloatVect3 utm_to_v3(struct UtmCoor_f *_utm_f) {
    struct FloatVect3 _fv3;
    _fv3.x = _utm_f->north;
    _fv3.y = _utm_f->east;
    _fv3.z = -_utm_f->alt;
    return _fv3;
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_potential_flow(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_POTENTIAL_FLOW(trans, dev, AC_ID,
                              &wind_vel_v3f.x, &wind_vel_v3f.z,
        &wind_grad_v3f.x, &wind_grad_v3f.z,
       &ltp_vehicle_pos->x, &ltp_vehicle_pos->z,
       &rel_dist.x, &rel_dist.z,
       &grad_spd_sp_x, &grad_spd_sp_z
                              );
}
#endif

/*
 * the actual potential flow calculation
 * 2d potential flow calculation (ignores y component)
 */
struct FloatVect3 compute_potential_flow(struct FloatVect3 rel_dist_v3f, float ref_wind_speed){
    float _x = rel_dist_v3f.x;
    float _z = rel_dist_v3f.z;

    float _theta = atan2(-_z, _x);
    float _r = sqrt(_x*_x+_z*_z);
    float _R = obstacle_radius;     // cylinder radius

//    cout << "r; " << _r << ", R: " << _R << endl;

    float u_r = (1 - ((_R*_R)/(_r*_r))) * ref_wind_speed * cosf(_theta);      // Vinf * (1-R*R/r*r) * cos(tht)
    float u_th = -(1 + ((_R*_R)/(_r*_r))) * ref_wind_speed * sinf(_theta);    // - Vinf * (1+R*R/r*r) * sin(tht)

//    cout << "t,r,th: " << _theta*180/M_PI << ", " << u_r << ", " << u_th << endl;

    float z_circle = -0.01;
    if (_R*_R - _x*_x > 0){       // what is this for?
        z_circle = -1*sqrt(_R*_R - _x*_x);
    }

//    cout << "; " << (_z - z_circle) << "< " << PF_SURFACE_ROUGHNESS << "< " << endl;
    // 2d potential flow, so there is no x component (x:east)
//    float mult_factor = (logf(-(_z - z_circle) / PF_SURFACE_ROUGHNESS)) / (logf(-(-70 - z_circle) / PF_SURFACE_ROUGHNESS));

//    cout << "z: " << z_circle << ", " << mult_factor << endl;
//    float mult_factor = 1.0;  //TODO: temp

    struct FloatVect3 _wind_vel;
    _wind_vel.x = (cosf(_theta) * u_r) - (sinf(_theta) * u_th);
    _wind_vel.y = 0;
    _wind_vel.z = -((sinf(_theta) * u_r) + (cosf(_theta) * u_th));      // wind down
    // TODO: check mult_factor. what is it for?

//    float tmp_x = (cosf(_theta) * u_r) - (sinf(_theta) * u_th) * mult_factor;
//    float tmp_z = -((sinf(_theta) * u_r) + (cosf(_theta) * u_th)) * mult_factor;      // wind down

//    cout << "mult: " << tmp_x << ", " << tmp_z << endl;

//    cout << rel_dist_v3f.x << ", " << rel_dist_v3f.y << ", " << rel_dist_v3f.z << endl;
//    cout << _wind_vel.y << ", " << _wind_vel.z << endl;

// NED
    return _wind_vel;
}

struct FloatVect3 compute_wind_gradient(struct FloatVect3 rel_dist_v3f, float ref_wind_speed) {
    struct FloatVect3 wind_vals[3];
    struct FloatVect3 dist_tweak;
    float pos_diff = 0.5;

    wind_vals[0] = compute_potential_flow(rel_dist_v3f, ref_wind_speed);

    // (x+0.5,z) (x,z-0.5) NED
    dist_tweak.x = rel_dist_v3f.x+pos_diff; dist_tweak.z = rel_dist_v3f.z;
    wind_vals[1] = compute_potential_flow(dist_tweak, ref_wind_speed);

    dist_tweak.x = rel_dist_v3f.x; dist_tweak.z = rel_dist_v3f.z-pos_diff;
    wind_vals[2] = compute_potential_flow(dist_tweak, ref_wind_speed);

    struct FloatVect3 wind_grad_v3f;
    wind_grad_v3f.x = (wind_vals[1].z-wind_vals[0].z)/pos_diff;
    wind_grad_v3f.z = (wind_vals[2].z-wind_vals[0].z)/pos_diff;
    wind_grad_v3f.y = 0;

    return wind_grad_v3f;
}

void init_potential_flow_simulator(void)
{
    ltp_vehicle_pos = stateGetPositionNed_f();   // local NED

#if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_POTENTIAL_FLOW, send_potential_flow);
#endif
}

void potential_flow_simulator_periodic(void)
{
    // Should I use mutex?
    ltp_vehicle_pos = stateGetPositionNed_f();   // local NED

    // heading
    float vehicle_heading = stateGetNedToBodyEulers_f()->psi*180/M_PI;

    /// then I have ground station position and vehicle position
    /// OBSTACLE's body frame!!! !!! !!!
//    struct FloatVect3 vehicle_position_in_ground_body_frame =
//            rotate_frame(&vehicle_position_v3, follow_me_heading-vehicle_heading);

//    cout << vehicle_position_in_ground_body_frame.x << ", " << vehicle_position_in_ground_body_frame.y << ", "
//    << vehicle_position_in_ground_body_frame.z << endl;

//    utm_of_lla_d(&pf_ground_utm, &_lla); // east north alt;
//    struct FloatVect3 ground_position_v3_enu = utm_to_v3(&pf_ground_utm);

//    struct FloatVect3 ground_position_v3_enu = utm_to_v3(&ground_utm);
//    cout << ground_position_v3_enu.x << ", " << ground_position_v3_enu.y << ", " << ground_position_v3_enu.z << endl;

//    struct FloatVect3 ground_position_v3 = rotate_frame(&ground_position_v3_enu, follow_me_heading);

//    cout << ground_position_v3.x << ", " << ground_position_v3.y << ", " << ground_position_v3.z << endl;

    /// calculate relative distance.. we assume that ground gps position is the obstacle position.
    // TODO: if not using ground gps, this does not make much sense.. bc ground position is in local, not utm.
    // TODO: so, set gp in utm, or add some conditions..
//    struct FloatVect3 dist_from_obstacle_to_vehicle =
//            calc_relative_position_v3(&vehicle_position_in_ground_body_frame, &ground_position_v3);

//    cout << "relative?" << endl;
//    cout << dist_from_obstacle_to_vehicle.x << ", " << dist_from_obstacle_to_vehicle.y << ", " << dist_from_obstacle_to_vehicle.z << endl;


    /// WIND. IN. NED.
//    float state_wind_x = stateGetHorizontalWindspeed_f()->x; // north
//    float state_wind_y = stateGetHorizontalWindspeed_f()->y; // east
//struct FloatVect3 windspeed_v3f = stateGetWindspeed_f();

    /// should be a user input or a measurement
//    float ref_wind_vel = PF_REF_WIND_VEL;

    rel_dist.x = obstacle_position_north - ltp_vehicle_pos->x;
    rel_dist.y = obstacle_position_east - ltp_vehicle_pos->y;
    rel_dist.z = obstacle_position_down - ltp_vehicle_pos->z;

//    cout << "dist: " << rel_dist.x << ", " << rel_dist.y << ", " << rel_dist.z << endl;

    /// potential flow calculation: obstacle's body frame; wind velocity (x, z) and reference wind vel (x dir)
    wind_vel_v3f = compute_potential_flow(rel_dist, ref_wind_spd);

#if PF_SET_GRADIENT
    wind_grad_v3f = compute_wind_gradient(rel_dist, ref_wind_spd);
    guidance_wind_gradient.x = wind_grad_v3f.x;
    guidance_wind_gradient.y = wind_grad_v3f.y;
    guidance_wind_gradient.z = wind_grad_v3f.z;

    grad_spd_sp_x = guidance_grad_gain_rec_x/guidance_wind_gradient.x * guidance_grad_gain_x;
    grad_spd_sp_z = guidance_grad_gain_rec_z/guidance_wind_gradient.z * guidance_grad_gain_z;
//    printf("%f %f\n", wind_grad_v3f.x*1000000, wind_grad_v3f.z*1000000);
#endif

//    cout << wind_vel_v3f.x << ", " << wind_vel_v3f.y << ", " << wind_vel_v3f.z << endl;

//    struct FloatVect3 wind_in_ned = rotate_frame(&wind_vel_v3f, -follow_me_heading);

    struct FloatVect2 hor_wind;
    hor_wind.x = wind_vel_v3f.x;
    hor_wind.y = wind_vel_v3f.y;

    float ver_wind = wind_vel_v3f.z;
#if PF_PREVENT_DOWNLIFT
    if(ver_wind > 1.0) { ver_wind = 1.0; }
#endif
//    cout << "wind: " << hor_wind.x << ", " << hor_wind.y << ", " << ver_wind << endl;
//    cout << endl;

//    // Set wind speed (state)
    stateSetHorizontalWindspeed_f(&hor_wind);
    stateSetVerticalWindspeed_f(ver_wind);

//    // set wind speed as environment
#ifdef SIM
    if (PF_SET_WIND_NPS_ENV){
        nps_atmosphere_set_wind_ned((double)wind_vel_v3f.x, (double)wind_vel_v3f.y, (double)ver_wind);
    }
#endif
}
