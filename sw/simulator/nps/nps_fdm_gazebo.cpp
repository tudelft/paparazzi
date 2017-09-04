/*
 * Copyright (C) 2017 Tom van Dijk
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
 * @file nps_fdm_gazebo.cpp
 * Flight Dynamics Model (FDM) for NPS using Gazebo.
 *
 * This is an FDM for NPS that uses Gazebo as the simulation engine.
 */

// The transition from Gazebo 7 to 8 deprecates a large number of functions.
// Ignore these errors for now...
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"


#ifdef NPS_DEBUG_VIDEO
// Opencv tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sys/time.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo_config.h>

extern "C" {
#include "nps_fdm.h"

#include "generated/airframe.h"
#include "autopilot.h"

#include "math/pprz_isa.h"
#include "math/pprz_algebra_double.h"
}
/******************************************************/

using namespace std;

#ifndef NPS_GAZEBO_WORLD
#define NPS_GAZEBO_WORLD "ardrone.world"
#endif
#ifndef NPS_GAZEBO_AC_NAME
#define NPS_GAZEBO_AC_NAME "paparazzi_uav"
#endif

// Add video handling functions if req'd.
#ifdef NPS_SIMULATE_VIDEO
extern "C" {
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/video_thread_nps.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "mcu_periph/sys_time.h"
}

static void init_gazebo_video(void);
static void gazebo_read_video(void);
static void read_image(
  struct image_t *img,
  gazebo::sensors::CameraSensorPtr cam);
struct gazebocam_t {
  gazebo::sensors::CameraSensorPtr cam;
  gazebo::common::Time last_measurement_time;
};
static struct gazebocam_t gazebo_cams[VIDEO_THREAD_MAX_CAMERAS] =
{ { NULL, 0 } };
#endif

#ifdef NPS_SIMULATE_RANGE_SENSORS
static void gazebo_init_range_sensors(void);
static void gazebo_read_range_sensors(void);
struct gazebo_range_sensors_t {
  gazebo::sensors::RaySensorPtr ray_front;
  gazebo::sensors::RaySensorPtr ray_right;
  gazebo::sensors::RaySensorPtr ray_back;
  gazebo::sensors::RaySensorPtr ray_left;
  gazebo::sensors::RaySensorPtr ray_down;
  gazebo::sensors::RaySensorPtr ray_up;
  gazebo::common::Time last_measurement_time;
};

static struct gazebo_range_sensors_t gazebo_range_sensors;

#endif

#ifdef NPS_SIMULATE_EXTERNAL_STEREO_CAMERA
extern "C" {
#include "stereoboard/stereo_image.h"
#include "stereoboard/math/stereo_math.h"
#include "stereoboard/camera_type.h"
#include "stereoboard/edgeflow.h"
#include "stereoboard/gate_detection.h"
}

static void gazebo_init_stereo_camera(void);
static void gazebo_read_stereo_camera(void);
static void read_stereoimage(
  struct image_t *img,
  gazebo::sensors::MultiCameraSensorPtr  cam);
struct gazebostereocam_t {
  gazebo::sensors::MultiCameraSensorPtr stereocam;
  gazebo::common::Time last_measurement_time;
};
static struct gazebostereocam_t gazebo_stereocam;

#endif


/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

// Pointer to Gazebo data
static bool gazebo_initialized = false;
static gazebo::physics::ModelPtr model = NULL;

// Get contact sensor
static gazebo::sensors::ContactSensorPtr ct;

// Helper functions
static void init_gazebo(void);
static void gazebo_read(void);
static void gazebo_write(double act_commands[], int commands_nb);

// Conversion routines
inline struct EcefCoor_d to_pprz_ecef(ignition::math::Vector3d ecef_i)
{
  struct EcefCoor_d ecef_p;
  ecef_p.x = ecef_i.X();
  ecef_p.y = ecef_i.Y();
  ecef_p.z = ecef_i.Z();
  return ecef_p;
}

inline struct NedCoor_d to_pprz_ned(ignition::math::Vector3d global)
{
  struct NedCoor_d ned;
  ned.x = global.Y();
  ned.y = global.X();
  ned.z = -global.Z();
  return ned;
}

inline struct LlaCoor_d to_pprz_lla(ignition::math::Vector3d lla_i)
{
  struct LlaCoor_d lla_p;
  lla_p.lat = lla_i.X();
  lla_p.lon = lla_i.Y();
  lla_p.alt = lla_i.Z();
  return lla_p;
}

inline struct DoubleVect3 to_pprz_body(gazebo::math::Vector3 body_g)
{
  struct DoubleVect3 body_p;
  body_p.x = body_g.x;
  body_p.y = -body_g.y;
  body_p.z = -body_g.z;
  return body_p;
}

inline struct DoubleRates to_pprz_rates(gazebo::math::Vector3 body_g)
{
  struct DoubleRates body_p;
  body_p.p = body_g.x;
  body_p.q = -body_g.y;
  body_p.r = -body_g.z;
  return body_p;
}

inline struct DoubleEulers to_pprz_eulers(gazebo::math::Quaternion quat)
{
  struct DoubleEulers eulers;
  eulers.psi = -quat.GetYaw() - M_PI / 2;
  eulers.theta = -quat.GetPitch();
  eulers.phi = quat.GetRoll();
  return eulers;
}

inline struct DoubleVect3 to_pprz_ltp(gazebo::math::Vector3 xyz)
{
  struct DoubleVect3 ltp;
  ltp.x = xyz.y;
  ltp.y = xyz.x;
  ltp.z = -xyz.z;
  return ltp;
}

// External functions, interface with Paparazzi's NPS as declared in nps_fdm.h

/**
 * Set JSBsim specific fields that are not used for Gazebo.
 * @param dt
 */
void nps_fdm_init(double dt)
{
  fdm.init_dt = dt; // JSBsim specific
  fdm.curr_dt = dt; // JSBsim specific
  fdm.nan_count = 0; // JSBsim specific
}

/**
 * Update the simulation state.
 * @param launch
 * @param act_commands
 * @param commands_nb
 */
void nps_fdm_run_step(
  bool launch __attribute__((unused)),
  double *act_commands,
  int commands_nb)
{
  // Initialize Gazebo if req'd.
  // Initialization is peformed here instead of in nps_fdm_init because:
  // - Video devices need to added at this point. Video devices have not been
  //   added yet when nps_fdm_init is called.
  // - nps_fdm_init runs on a different thread then nps_fdm_run_step, which
  //   causes problems with Gazebo.
  if (!gazebo_initialized) {
    init_gazebo();
    gazebo_read();
#ifdef NPS_SIMULATE_VIDEO
    init_gazebo_video();
    cout << "video initiated" << endl;
#endif
#ifdef NPS_SIMULATE_RANGE_SENSORS
    gazebo_init_range_sensors();
    cout << "laser range sensor ring initiated" << endl;
#endif
#ifdef NPS_SIMULATE_EXTERNAL_STEREO_CAMERA
    gazebo_init_stereo_camera();
    cout << "stereo camera initiated" << endl;
#endif
    gazebo_initialized = true;
  }

  // Update the simulation for a single timestep.
  gazebo::runWorld(model->GetWorld(), 1);
  gazebo::sensors::run_once();
  gazebo_write(act_commands, commands_nb);
  gazebo_read();
#ifdef NPS_SIMULATE_VIDEO
  gazebo_read_video();
#endif
#ifdef NPS_SIMULATE_RANGE_SENSORS
  gazebo_read_range_sensors();
#endif
#ifdef NPS_SIMULATE_EXTERNAL_STEREO_CAMERA
  gazebo_read_stereo_camera();
#endif

}

// TODO Atmosphere functions have not been implemented yet.
// Starting at version 8, Gazebo has its own atmosphere and wind model.
void nps_fdm_set_wind(
  double speed __attribute__((unused)),
  double dir __attribute__((unused)))
{
}

void nps_fdm_set_wind_ned(
  double wind_north __attribute__((unused)),
  double wind_east __attribute__((unused)),
  double wind_down __attribute__((unused)))
{
}

void nps_fdm_set_turbulence(
  double wind_speed __attribute__((unused)),
  int turbulence_severity __attribute__((unused)))
{
}

/** Set temperature in degrees Celcius at given height h above MSL */
void nps_fdm_set_temperature(
  double temp __attribute__((unused)),
  double h __attribute__((unused)))
{
}

// Internal functions
/**
 * Set up a Gazebo server.
 *
 * Starts a Gazebo server, adds conf/simulator/gazebo/models to its model path
 * and loads the world specified by NPS_GAZEBO_WORLD.
 *
 * This function also obtaines a pointer to the aircraft model, named
 * NPS_GAZEBO_AC_NAME ('paparazzi_uav' by default). This pointer, 'model',
 * is used to read the state and write actuator commands in gazebo_read and
 * _write.
 */
static void init_gazebo(void)
{
  string gazebo_home = "/conf/simulator/gazebo/";
  string pprz_home(getenv("PAPARAZZI_HOME"));
  string gazebodir = pprz_home + gazebo_home;
  cout << "Gazebo directory: " << gazebodir << endl;

  if (!gazebo::setupServer(0, NULL)) {
    cout << "Failed to start Gazebo, exiting." << endl;
    std::exit(-1);
  }

  cout << "Add Paparazzi model path: " << gazebodir + "models/" << endl;
  gazebo::common::SystemPaths::Instance()->AddModelPaths(
    gazebodir + "models/");

  cout << "Load world: " << gazebodir + "world/" + NPS_GAZEBO_WORLD << endl;
  gazebo::physics::WorldPtr world = gazebo::loadWorld(
                                      gazebodir + "world/" + NPS_GAZEBO_WORLD);
  if (!world) {
    cout << "Failed to open world, exiting." << endl;
    std::exit(-1);
  }

  cout << "Get pointer to aircraft: " << NPS_GAZEBO_AC_NAME << endl;
  model = world->GetModel(NPS_GAZEBO_AC_NAME);
  if (!model) {
    cout << "Failed to find '" << NPS_GAZEBO_AC_NAME << "', exiting."
         << endl;
    std::exit(-1);
  }

  // Initialize sensors
  gazebo::sensors::run_once(true);
  gazebo::sensors::run_threads();
  gazebo::runWorld(world, 1);
  cout << "Sensors initialized..." << endl;

  // activate collision sensor
  gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();
  ct = static_pointer_cast < gazebo::sensors::ContactSensor > (mgr->GetSensor("contactsensor"));
  ct->SetActive(true);

  cout << "Gazebo initialized successfully!" << endl;
}

/**
 * Read Gazebo's simulation state and store the results in the fdm struct used
 * by NPS.
 *
 * Not all fields are filled at the moment, as some of them are unused by
 * paparazzi (see comments) and others are not available in Gazebo 7
 * (atmosphere).
 */
static void gazebo_read(void)
{
  gazebo::physics::WorldPtr world = model->GetWorld();
  gazebo::math::Pose pose = model->GetWorldPose(); // In LOCAL xyz frame
  gazebo::math::Vector3 vel = model->GetWorldLinearVel();
  gazebo::math::Vector3 accel = model->GetWorldLinearAccel();
  gazebo::math::Vector3 ang_vel = model->GetWorldAngularVel();
  gazebo::common::SphericalCoordinatesPtr sphere =
    world->GetSphericalCoordinates();
  gazebo::math::Quaternion local_to_global_quat(0, 0,
      -sphere->HeadingOffset().Radian());

  /* Fill FDM struct */
  fdm.time = world->GetSimTime().Double();

  // init_dt: unused
  // curr_dt: unused
  // on_ground: unused
  // nan_count: unused

  /* position */
  fdm.ecef_pos = to_pprz_ecef(
                   sphere->PositionTransform(pose.pos.Ign(),
                       gazebo::common::SphericalCoordinates::LOCAL,
                       gazebo::common::SphericalCoordinates::ECEF));
  fdm.ltpprz_pos = to_pprz_ned(
                     sphere->PositionTransform(pose.pos.Ign(),
                         gazebo::common::SphericalCoordinates::LOCAL,
                         gazebo::common::SphericalCoordinates::GLOBAL));
  fdm.lla_pos = to_pprz_lla(
                  sphere->PositionTransform(pose.pos.Ign(),
                      gazebo::common::SphericalCoordinates::LOCAL,
                      gazebo::common::SphericalCoordinates::SPHERICAL));
  fdm.hmsl = pose.pos.z;
  /* debug positions */
  fdm.lla_pos_pprz = fdm.lla_pos; // Don't really care...
  fdm.lla_pos_geod = fdm.lla_pos;
  fdm.lla_pos_geoc = fdm.lla_pos;
  fdm.agl = pose.pos.z; // TODO Measure with sensor

  /* velocity */
  fdm.ecef_ecef_vel = to_pprz_ecef(
                        sphere->VelocityTransform(vel.Ign(),
                            gazebo::common::SphericalCoordinates::LOCAL,
                            gazebo::common::SphericalCoordinates::ECEF));
  fdm.body_ecef_vel = to_pprz_body(pose.rot.RotateVectorReverse(vel)); // Note: unused
  fdm.ltp_ecef_vel = to_pprz_ned(
                       sphere->VelocityTransform(vel.Ign(),
                           gazebo::common::SphericalCoordinates::LOCAL,
                           gazebo::common::SphericalCoordinates::GLOBAL));
  fdm.ltpprz_ecef_vel = fdm.ltp_ecef_vel; // ???

  /* acceleration */
  fdm.ecef_ecef_accel = to_pprz_ecef(
                          sphere->VelocityTransform(accel.Ign(),
                              gazebo::common::SphericalCoordinates::LOCAL,
                              gazebo::common::SphericalCoordinates::ECEF)); // Note: unused
  fdm.body_ecef_accel = to_pprz_body(pose.rot.RotateVectorReverse(accel));
  fdm.ltp_ecef_accel = to_pprz_ned(
                         sphere->VelocityTransform(accel.Ign(),
                             gazebo::common::SphericalCoordinates::LOCAL,
                             gazebo::common::SphericalCoordinates::GLOBAL)); // Note: unused
  fdm.ltpprz_ecef_accel = fdm.ltp_ecef_accel; // ???
  fdm.body_inertial_accel = fdm.body_ecef_accel; // Approximate, unused.

  // only use accelerometer if no collisions or if not on ground
  if (ct->Contacts().contact_size() || pose.pos.z < 0.03){
    fdm.body_accel = to_pprz_body(
                     pose.rot.RotateVectorReverse(-world->Gravity()));
  } else {
    fdm.body_accel = to_pprz_body(
                     pose.rot.RotateVectorReverse(accel.Ign() - world->Gravity()));
  }
  /* attitude */
  // ecef_to_body_quat: unused
  fdm.ltp_to_body_eulers = to_pprz_eulers(local_to_global_quat * pose.rot);
  double_quat_of_eulers(&(fdm.ltp_to_body_quat), &(fdm.ltp_to_body_eulers));
  fdm.ltpprz_to_body_quat = fdm.ltp_to_body_quat; // unused
  fdm.ltpprz_to_body_eulers = fdm.ltp_to_body_eulers; // unused

  /* angular velocity */
  fdm.body_ecef_rotvel = to_pprz_rates(pose.rot.RotateVectorReverse(ang_vel));
  fdm.body_inertial_rotvel = fdm.body_ecef_rotvel; // Approximate

  /* angular acceleration */
  // body_ecef_rotaccel: unused
  // body_inertial_rotaccel: unused
  /* misc */
  fdm.ltp_g = to_pprz_ltp(
                sphere->VelocityTransform(-1 * world->Gravity(),
                                          gazebo::common::SphericalCoordinates::LOCAL,
                                          gazebo::common::SphericalCoordinates::GLOBAL)); // unused
  fdm.ltp_h = to_pprz_ltp(
                sphere->VelocityTransform(world->MagneticField(),
                                          gazebo::common::SphericalCoordinates::LOCAL,
                                          gazebo::common::SphericalCoordinates::GLOBAL));

  /* atmosphere */
#if GAZEBO_MAJOR_VERSION >= 8 && 0 // TODO implement

#else
  // Gazebo versions < 8 do not have atmosphere or wind support
  // Use placeholder values. Valid for low altitude, low speed flights.
  fdm.wind = {.x = 0, .y = 0, .z = 0};
  fdm.pressure_sl = 101325; // Pa

  fdm.airspeed = 0;
  fdm.pressure = pprz_isa_pressure_of_height(fdm.hmsl, fdm.pressure_sl);
  fdm.dynamic_pressure = fdm.pressure_sl; // Pa
  fdm.temperature = 20.0; // C
  fdm.aoa = 0; // rad
  fdm.sideslip = 0; // rad
#endif
  /* flight controls: unused */
  fdm.elevator = 0;
  fdm.flap = 0;
  fdm.left_aileron = 0;
  fdm.right_aileron = 0;
  fdm.rudder = 0;
  /* engine: unused */
  fdm.num_engines = 0;
}

/**
 * Write actuator commands to Gazebo.
 *
 * This function takes the normalized commands and applies them as forces and
 * torques in Gazebo. Since the commands are normalized in [0,1], their
 * thrusts (NPS_ACTUATOR_THRUSTS) and torques (NPS_ACTUATOR_TORQUES) need to
 * be specified in the airframe file. Their values need to be specified in the
 * same order as the NPS_ACTUATOR_NAMES and should be provided in SI units.
 * See conf/airframes/examples/ardrone2_gazebo.xml for an example.
 *
 * The forces and torques are applied to (the origin of) the links named in
 * NPS_ACTUATOR_NAMES. See conf/simulator/gazebo/models/ardrone/ardrone.sdf
 * for an example.
 *
 * @param act_commands
 * @param commands_nb
 */
static void gazebo_write(double act_commands[], int commands_nb)
{
  const string names[] = NPS_ACTUATOR_NAMES;
  const double thrusts[] = { NPS_ACTUATOR_THRUSTS };
  const double torques[] = { NPS_ACTUATOR_TORQUES };

  for (int i = 0; i < commands_nb; ++i) {
    double thrust = autopilot.motors_on ? thrusts[i] * act_commands[i] : 0.0;
    double torque = autopilot.motors_on ? torques[i] * act_commands[i] : 0.0;
    gazebo::physics::LinkPtr link = model->GetLink(names[i]);
    link->AddRelativeForce(gazebo::math::Vector3(0, 0, thrust));
    link->AddRelativeTorque(gazebo::math::Vector3(0, 0, torque));
  }
}

#ifdef NPS_SIMULATE_VIDEO
/**
 * Set up cameras.
 *
 * This function finds the video devices added through add_video_device
 * (sw/airborne/modules/computer_vision/cv.h). The camera links in the Gazebo AC
 * model should have the same name as the .dev_name field in the corresponding
 * video_config_t struct stored in 'cameras[]' (computer_vision/
 * video_thread_nps.h). Pointers to Gazebo's cameras are stored in gazebo_cams
 * at the same index as their 'cameras[]' counterpart.
 *
 * The video_config_t parameters are updated using the values provided by
 * Gazebo. This should simplify the use of different UAVs with different camera
 * setups.
 */
static void init_gazebo_video(void)
{
  gazebo::sensors::SensorManager *mgr =
    gazebo::sensors::SensorManager::Instance();

  cout << "Initializing cameras..." << endl;
  // Loop over cameras registered in video_thread_nps
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS && cameras[i] != NULL; ++i) {
    // Find link in gazebo model
    cout << "Setting up '" << cameras[i]->dev_name << "'... ";
    gazebo::physics::LinkPtr link = model->GetLink(cameras[i]->dev_name);
    if (!link) {
      cout << "ERROR: Link '" << cameras[i]->dev_name << "' not found!"
           << endl;
      ;
      continue;
    }
    // Get a pointer to the sensor using its full name
    if (link->GetSensorCount() != 1) {
      cout << "ERROR: Link '" << link->GetName()
           << "' should only contain 1 sensor!" << endl;
      continue;
    }
    string name = link->GetSensorName(0);
    gazebo::sensors::CameraSensorPtr cam = static_pointer_cast
                                           < gazebo::sensors::CameraSensor > (mgr->GetSensor(name));
    if (!cam) {
      cout << "ERROR: Could not get pointer to '" << name << "'!" << endl;
      continue;
    }
    // Activate sensor
    cam->SetActive(true);
    // Add to list of cameras
    gazebo_cams[i].cam = cam;
    gazebo_cams[i].last_measurement_time = cam->LastMeasurementTime();
    // Copy video_config settings from Gazebo's camera
    cameras[i]->output_size.w = cam->ImageWidth();
    cameras[i]->output_size.h = cam->ImageHeight();
    cameras[i]->sensor_size.w = cam->ImageWidth();
    cameras[i]->sensor_size.h = cam->ImageHeight();
    cameras[i]->crop.w = cam->ImageWidth();
    cameras[i]->crop.h = cam->ImageHeight();
    cameras[i]->fps = cam->UpdateRate();
    cout << "ok" << endl;
  }
}

/**
 * Read camera images.
 *
 * Polls gazebo cameras. If the last measurement time has been updated, a new
 * frame is available. This frame is converted to Paparazzi's UYVY format
 * and passed to cv_run_device which runs the callbacks registered by various
 * modules.
 */
static void gazebo_read_video(void)
{
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    gazebo::sensors::CameraSensorPtr &cam = gazebo_cams[i].cam;
    // Skip unregistered or unfound cameras
    if (cam == NULL) { continue; }
    // Skip if not updated
    // Also skip when LastMeasurementTime() is zero (workaround)
    if ((cam->LastMeasurementTime() - gazebo_cams[i].last_measurement_time).Float() < 0.005
        || cam->LastMeasurementTime() == 0) { continue; }
    // Grab image, convert and send to video thread
    struct image_t img;
    read_image(&img, cam);

#ifdef NPS_DEBUG_VIDEO
    cv::Mat RGB_cam(cam->ImageHeight(),cam->ImageWidth(),CV_8UC3,(uint8_t *)cam->ImageData());
    cv::cvtColor(RGB_cam, RGB_cam, cv::COLOR_RGB2BGR);
    cv::namedWindow(cameras[i]->dev_name, cv::WINDOW_AUTOSIZE);  // Create a window for display.
    cv::imshow(cameras[i]->dev_name, RGB_cam);
    cv::waitKey(1);
#endif

    cv_run_device(cameras[i], &img);
    // Free image buffer after use.
    image_free(&img);
    // Keep track of last update time.
    gazebo_cams[i].last_measurement_time = cam->LastMeasurementTime();
  }
}

/**
 * Read Gazebo image and convert.
 *
 * Converts the current camera frame to the format used by Paparazzi. This
 * includes conversion to UYVY. Gazebo's simulation time is used for the image
 * timestamp.
 *
 * @param img
 * @param cam
 */
static void read_image(
  struct image_t *img,
  gazebo::sensors::CameraSensorPtr cam)
{
  image_create(img, cam->ImageWidth(), cam->ImageHeight(), IMAGE_YUV422);
  // Convert Gazebo's *RGB888* image to Paparazzi's YUV422
  const uint8_t *data_rgb = cam->ImageData();
  uint8_t *data_yuv = (uint8_t *)(img->buf);
  for (int x = 0; x < img->w; ++x) {
    for (int y = 0; y < img->h; ++y) {
      int idx_rgb = 3 * (img->w * y + x);
      int idx_yuv = 2 * (img->w * y + x);
      int idx_px = img->w * y + x;
      if (idx_px % 2 == 0) { // Pick U or V
        data_yuv[idx_yuv] = -0.148 * data_rgb[idx_rgb]
                            - 0.291 * data_rgb[idx_rgb + 1]
                            + 0.439 * data_rgb[idx_rgb + 2] + 128; // U
      } else {
        data_yuv[idx_yuv] = 0.439 * data_rgb[idx_rgb]
                            - 0.368 * data_rgb[idx_rgb + 1]
                            - 0.071 * data_rgb[idx_rgb + 2] + 128; // V
      }
      data_yuv[idx_yuv + 1] = 0.257 * data_rgb[idx_rgb]
                              + 0.504 * data_rgb[idx_rgb + 1]
                              + 0.098 * data_rgb[idx_rgb + 2] + 16; // Y
    }
  }
  // Fill miscellaneous fields
  gazebo::common::Time ts = cam->LastMeasurementTime();
  img->ts.tv_sec = ts.sec;
  img->ts.tv_usec = ts.nsec / 1000.0;
  img->pprz_ts = ts.Double() * 1e6;
  img->buf_idx = 0; // unused
}
#endif

#ifdef NPS_SIMULATE_RANGE_SENSORS
static void gazebo_init_range_sensors(void)
{
  gazebo::sensors::SensorManager *mgr =
    gazebo::sensors::SensorManager::Instance();

  gazebo_range_sensors.ray_front = static_pointer_cast<gazebo::sensors::RaySensor>(mgr->GetSensor("range_sensors::front_range_sensor"));
  gazebo_range_sensors.ray_right = static_pointer_cast<gazebo::sensors::RaySensor>(mgr->GetSensor("range_sensors::right_range_sensor"));
  gazebo_range_sensors.ray_back = static_pointer_cast<gazebo::sensors::RaySensor>(mgr->GetSensor("range_sensors::back_range_sensor"));
  gazebo_range_sensors.ray_left = static_pointer_cast<gazebo::sensors::RaySensor>(mgr->GetSensor("range_sensors::left_range_sensor"));
  gazebo_range_sensors.ray_up = static_pointer_cast<gazebo::sensors::RaySensor>(mgr->GetSensor("range_sensors::up_range_sensor"));
  gazebo_range_sensors.ray_down = static_pointer_cast<gazebo::sensors::RaySensor>(mgr->GetSensor("range_sensors::down_range_sensor"));

  if (!gazebo_range_sensors.ray_left || !gazebo_range_sensors.ray_right || !gazebo_range_sensors.ray_up ||
      !gazebo_range_sensors.ray_down || !gazebo_range_sensors.ray_front || !gazebo_range_sensors.ray_back) {
    cout << "ERROR: Could not get pointer to raysensor!" << gazebo_range_sensors.ray_left << gazebo_range_sensors.ray_right << gazebo_range_sensors.ray_up <<
         gazebo_range_sensors.ray_down << gazebo_range_sensors.ray_front << gazebo_range_sensors.ray_back << endl;
  }

  gazebo_range_sensors.ray_left->SetActive(true);
  gazebo_range_sensors.ray_right->SetActive(true);
  gazebo_range_sensors.ray_up->SetActive(true);
  gazebo_range_sensors.ray_down->SetActive(true);
  gazebo_range_sensors.ray_front->SetActive(true);
  gazebo_range_sensors.ray_back->SetActive(true);

}
static void gazebo_read_range_sensors(void)
{
  int16_t range_sensors_int16[6];
  range_sensors_int16[0] = (int16_t)(gazebo_range_sensors.ray_front->Range(0) * 1000);
  range_sensors_int16[1] = (int16_t)(gazebo_range_sensors.ray_right->Range(0) * 1000);
  range_sensors_int16[2] = (int16_t)(gazebo_range_sensors.ray_back->Range(0) * 1000);
  range_sensors_int16[3] = (int16_t)(gazebo_range_sensors.ray_left->Range(0) * 1000);
  range_sensors_int16[4] = (int16_t)(gazebo_range_sensors.ray_up->Range(0) * 1000);
  range_sensors_int16[5] = (int16_t)(gazebo_range_sensors.ray_down->Range(0) * 1000);

  int i;
  for(i=0;i<6;i++)
  {
	  if(range_sensors_int16[i] == 0)
		  range_sensors_int16[i] = 32767;

  }

  //SEND ABI MESSAGES
  // Standard range sensor message
  AbiSendMsgRANGE_SENSORS(RANGE_SENSORS_GAZEBO_ID, range_sensors_int16[0], range_sensors_int16[1], range_sensors_int16[2],
                          range_sensors_int16[3], range_sensors_int16[4], range_sensors_int16[5]);
  // Down range sensor as "Sonar"
  AbiSendMsgAGL(AGL_RANGE_SENSORS_GAZEBO_ID, gazebo_range_sensors.ray_down->Range(0));
}
#endif

#ifdef NPS_SIMULATE_EXTERNAL_STEREO_CAMERA
/******************************EDGEFLOW****************************/

struct FloatRMat body_to_cam;
struct FloatEulers cam_angles;

/******************************************************************/

///////PLOTTING FUNCTION/////
static void plot_matlab(int32_t *array, uint16_t size, double scale,char* name )
{
  std::vector<double> X;
  std::vector<double> Y;

  int x;
  for (x = 0; x < size; x++) {
    X.push_back((double)x);
    Y.push_back((double)array[x]*scale);
  }

  plt::plot(X,Y);
	plt::named_plot(name, X, Y);
}
////////////////////////////////////

static void gazebo_init_stereo_camera(void)
{
  gazebo::sensors::SensorManager *mgr =
    gazebo::sensors::SensorManager::Instance();

  gazebo_stereocam.stereocam = static_pointer_cast
                               < gazebo::sensors::MultiCameraSensor > (mgr->GetSensor("stereo_camera::stereo_camera"));

  if (!gazebo_stereocam.stereocam) {
    cout << "ERROR: Could not get pointer to stereocam!" << endl;
  }
  gazebo_stereocam.stereocam->SetActive(true);

  gazebo_stereocam.last_measurement_time = gazebo_stereocam.stereocam->LastMeasurementTime();

  /******************************EDGEFLOW  INIT****************************/
  edgeflow_init(gazebo_stereocam.stereocam->ImageWidth(0), gazebo_stereocam.stereocam->ImageHeight(0), 0);

#ifdef STEREO_BODY_TO_STEREO_PHI
  struct FloatEulers euler = {STEREO_BODY_TO_STEREO_PHI, STEREO_BODY_TO_STEREO_THETA, STEREO_BODY_TO_STEREO_PSI};
#else
  struct FloatEulers euler = {0, 0, 0};
#endif
  float_rmat_of_eulers(&body_to_cam, &euler);

  /******************************************************************/
}

static void gazebo_read_stereo_camera(void)
{
  gazebo::sensors::MultiCameraSensorPtr &stereocam = gazebo_stereocam.stereocam;
  if ((stereocam->LastMeasurementTime() - gazebo_stereocam.last_measurement_time).Float() < 0.005
      || stereocam->LastMeasurementTime() == 0) { return; }

  static struct image_t img;
  read_stereoimage(&img, stereocam);

#ifdef NPS_DEBUG_STEREOCAM
  cv::Mat RGB_left(stereocam->ImageHeight(0),stereocam->ImageWidth(0),CV_8UC3,(uint8_t *)stereocam->ImageData(0));
  cv::Mat RGB_right(stereocam->ImageHeight(1),stereocam->ImageWidth(1),CV_8UC3,(uint8_t *)stereocam->ImageData(1));

  cv::cvtColor(RGB_left, RGB_left, cv::COLOR_RGB2BGR);
  cv::cvtColor(RGB_right, RGB_right, cv::COLOR_RGB2BGR);

  cv::namedWindow("left", cv::WINDOW_AUTOSIZE);  // Create a window for display.
  cv::imshow("left", RGB_left);
  cv::namedWindow("right", cv::WINDOW_AUTOSIZE);  // Create a window for display.
  cv::imshow("right", RGB_right);
  cv::waitKey(1);
#endif

  /******************************************EDGEFLOW*******************************/
  // set angles for derotation
  float_rmat_mult(&cam_angles, &body_to_cam, stateGetNedToBodyEulers_f());
  img.eulers = cam_angles;

  edgeflow_total(&img, 0);

  static struct FloatVect3 vel, dist;

  float res = (float)(edgeflow_params.RES);

  vel.x = edgeflow.vel.x / res;
  vel.y = edgeflow.vel.y / res;
  vel.z = edgeflow.vel.z / res;

  float R2 = (float)edgeflow.flow_quality / res;

  stereocam_parse_vel(vel, R2);

  dist.x = (float)edgeflow_snapshot.dist.x / res;
  dist.y = (float)edgeflow_snapshot.dist.y / res;
  dist.z = (float)edgeflow_snapshot.dist.z / res;

  R2 = (float)edgeflow_snapshot.quality / res;

  stereocam_parse_pos(dist, R2, edgeflow_snapshot.new_keyframe);

  //////Gate Detector//////
  struct image_t left_img, gradient;
  image_create(&left_img, img.w, img.h, IMAGE_GRAYSCALE);
  image_create(&gradient, img.w, img.h, IMAGE_GRAYSCALE);

  getLeftFromStereo(&left_img, &img);
  image_2d_gradients(&left_img, &gradient);
  //image_2d_sobel(&left_img, &gradient);

  gate_set_intensity(50,250);
  static struct gate_t gate;
  bool gate_detected = false;

  //For debugging gate detection
  gate_detected = snake_gate_detection(&gradient, &gate, false, NULL, NULL, NULL);

#ifdef NPS_DEBUG_STEREOCAM
  cv::Mat gradient_cv(stereocam->ImageHeight(0),stereocam->ImageWidth(0),CV_8UC1,(uint8_t *)gradient.buf);
  cv::imshow("gradient", gradient_cv);
  cv::waitKey(1);
#endif

  if (gate.q > 15)
  {
    float w = 2.f * gate.sz * FOVX / IMAGE_WIDTH;
    float h = 2.f * (float)(gate.sz_left > gate.sz_right ? gate.sz_left : gate.sz_right)
        * FOVY / IMAGE_HEIGHT;

    static struct FloatEulers camera_bearing;
    camera_bearing.theta = pix2angle((gate.x - IMAGE_WIDTH/2), 0);
    camera_bearing.phi = -pix2angle((gate.y - IMAGE_HEIGHT/2), 1); // positive y, causes negative phi
    camera_bearing.psi = 0;

    static struct FloatEulers body_bearing;
    float_rmat_transp_mult(&body_bearing, &body_to_cam, &camera_bearing);

    imav2017_set_gate(gate.q, w, h, body_bearing.psi, body_bearing.theta, 0,(uint8_t)gate_detected);
  }

  image_free(&left_img);
  image_free(&gradient);

  // GET obstacle
  /////////////////////////////////
  uint8_t stereo_distance_per_column_uint8[IMAGE_WIDTH] = {0};
  for(int x=0;x<img.w;x++)stereo_distance_per_column_uint8[x]= bounduint8(edgeflow.stereo_distance_per_column[x]);

  const uint8_t width = img.w;
  uint8_t stereo_distance_filtered[width];
  memset( stereo_distance_filtered, 0, width*sizeof(uint8_t) );
  uint8_t closest_average_distance = 255;
  uint8_t pixel_location_of_closest_object = 0;

  //TODO: do this for each stereo image column
  imav2017_histogram_obstacle_detection(stereo_distance_per_column_uint8, stereo_distance_filtered,
      &closest_average_distance, &pixel_location_of_closest_object, img.w);

  /*Copied from stereocam.c*/

  //TODO: automatically get FOV
  float pxtorad=(float)RadOfDeg(59) / 128;

  float heading = (float)(pixel_location_of_closest_object-49)*pxtorad;
  float distance = (float)(closest_average_distance)/100;

// DOWNLINK_SEND_SETTINGS(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE, &distance, &heading);

  //float x_offset_collision = tanf(heading)*distance;

  //AbiSendMsgSTEREO_FORCEFIELD(ABI_BROADCAST, 0, 0,0);

  AbiSendMsgOBSTACLE_DETECTION(AGL_RANGE_SENSORS_GAZEBO_ID, distance, heading);
  /////////////////////////////////
/*
  int32_t stereo_distance_filtered_int32[128];
  for(int x=0;x<img.w;x++)stereo_distance_filtered_int32[x]= (int32_t)stereo_distance_filtered[x];
*/


  ///PLOT STUFF
  /*   plt::clf();
  plot_matlab(edgeflow.stereo_distance_per_column,128,1, "edge_histogram");
  plot_matlab(edgeflow.edge_hist[edgeflow.prev_frame_x].x,128,1, "edge_histogram");
  plot_matlab(edgeflow.disp.x,128,10, "edge_histogram");
  plot_matlab((int32_t*)stereo_distance_filtered_int32,128,100, "edge_histogram");
  plt::pause(0.0001);*/
  /////

/*********************************************************************************/

  image_free(&img);
  gazebo_stereocam.last_measurement_time = stereocam->LastMeasurementTime();
}


static void read_stereoimage(
  struct image_t *img,
  gazebo::sensors::MultiCameraSensorPtr cam)
{
  image_create(img, cam->ImageWidth(0), cam->ImageHeight(0), IMAGE_YUV422);

  // Convert Gazebo's *RGB888* image to stereo pixel muxtexed image
  const uint8_t *data_rgb_left = cam->ImageData(0);
  const uint8_t *data_rgb_right = cam->ImageData(1);

  uint8_t *data = (uint8_t *)(img->buf);

  for (int x = 0; x < img->w; ++x) {
    for (int y = 0; y < img->h; ++y) {
      int idx_rgb = 3 * (img->w * y + x);
      int idx_pix_mux = 2 * (img->w * y + x);

      data[idx_pix_mux] =  0.257 * data_rgb_left[idx_rgb]
                           + 0.504 * data_rgb_left[idx_rgb + 1]
                           + 0.098 * data_rgb_left[idx_rgb + 2] + 16; // Y

      data[idx_pix_mux + 1] = 0.257 * data_rgb_right[idx_rgb]
                              + 0.504 * data_rgb_right[idx_rgb + 1]
                              + 0.098 * data_rgb_right[idx_rgb + 2] + 16; // Y

    }
  }

  // Fill miscellaneous fields
  gazebo::common::Time ts = cam->LastMeasurementTime();
  img->ts.tv_sec = ts.sec;
  img->ts.tv_usec = ts.nsec / 1000.0;
  img->pprz_ts = ts.Double() * 1e6;
  img->buf_idx = 0; // unused*/
}

#endif

#pragma GCC diagnostic pop // Ignore -Wdeprecated-declarations
