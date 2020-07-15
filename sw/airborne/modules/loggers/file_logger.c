/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include "std.h"

#include "subsystems/imu.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"

#include "state.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  // check if log path exists
  struct stat s;
  int err = stat(STRINGIFY(FILE_LOGGER_PATH), &s);

  if(err < 0) {
    // try to make the directory
    mkdir(STRINGIFY(FILE_LOGGER_PATH), 0666);
  }

  // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y-%m-%d_%X", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), date_time);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), date_time, counter);
    counter++;
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,

	  //rotorcraft uses COMMAND_THRUST, fixedwing COMMAND_THROTTLE at this time
#ifdef COMMAND_THRUST
      "counter,rates_p,rates_q,rates_r,stab_cmd_T,stab_cmd_R,stab_cmd_P,stab_cmd_Y,quati,quatx,quaty,quatz,sp_quati,sp_quatx,sp_quaty,sp_quatz,accelx,accely,accelz,posx,posy,posz,speedx,speedy,speedz,v_z_ref,sp_accelx,sp_accely,sp_accelz,speed_sp.x,speed_sp.y,speed_sp.z,desired_airspeed.x,desired_airspeed.y,eulercmdx,eulercmdy,eulercmdz,guidance_euler_cmd_phi,guidance_euler_cmd_theta\n"
#else
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,	h_ctl_aileron_setpoint, h_ctl_elevator_setpoint, qi,qx,qy,qz\n"
#endif
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

/** Log the values to a csv file    */
/** Change the Variable that you are interested in here */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  struct FloatRates *rates = stateGetBodyRates_f();
  struct Int32Vect3 *accel = stateGetAccelBody_i();

#ifdef COMMAND_THRUST //For example rotorcraft
  fprintf(file_logger, "%d,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
          counter,
          rates->p,
          rates->q,
          rates->r,
          stabilization_cmd[COMMAND_THRUST],
          stabilization_cmd[COMMAND_ROLL],
          stabilization_cmd[COMMAND_PITCH],
          stabilization_cmd[COMMAND_YAW],
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz,
          stab_att_sp_quat.qi,
          stab_att_sp_quat.qx,
          stab_att_sp_quat.qy,
          stab_att_sp_quat.qz,
          accel->x,
          accel->y,
          accel->z,
          stateGetPositionNed_f()->x,
          stateGetPositionNed_f()->y,
          stateGetPositionNed_f()->z,
          stateGetSpeedNed_f()->x,
          stateGetSpeedNed_f()->y,
          stateGetSpeedNed_f()->z,
          guidance_v_z_ref,
          sp_accel.x,
          sp_accel.y,
          sp_accel.z,
          speed_sp.x,
          speed_sp.y,
          speed_sp.z,
          desired_airspeed.x,
          desired_airspeed.y,
          euler_cmd.x,
          euler_cmd.y,
          euler_cmd.z,
          guidance_euler_cmd.phi,
          guidance_euler_cmd.theta
         );
#else
  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          counter,
          imu.gyro_unscaled.p,
          imu.gyro_unscaled.q,
          imu.gyro_unscaled.r,
          imu.accel_unscaled.x,
          imu.accel_unscaled.y,
          imu.accel_unscaled.z,
          imu.mag_unscaled.x,
          imu.mag_unscaled.y,
          imu.mag_unscaled.z,
		  h_ctl_aileron_setpoint,
		  h_ctl_elevator_setpoint,
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz
         );
#endif

  counter++;
}
