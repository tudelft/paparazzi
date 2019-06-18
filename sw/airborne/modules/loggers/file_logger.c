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
    "counter,\
    accel.x,\
    accel.y,\
    accel.z,\
    gyro.p,\
    gyro.q,\
    gyro.r,\
    rot->phi,\
    rot->theta,\
    rot->psi,\
    rates->p,\
    rates->q,\
    rates->r,\
    pos->x, pos->y, pos->z,\
    COMMAND_THRUST,\
    COMMAND_ROLL,\
    COMMAND_PITCH,\
    COMMAND_YAW,\
    rpm_obs[0],\
    rpm_obs[1],\
    rpm_obs[2],\
    rpm_obs[3]\n"
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

/** Log the values to a csv file    */
/** Change the Variable that you are interested in here */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter; // no idea how it works. zero init?
  // struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  struct FloatEulers *rot = stateGetNedToBodyEulers_f();
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d\n",
          counter,
          imu.accel.x, 
          imu.accel.y, // /1024 now
          imu.accel.z,
          imu.gyro.p,  // INT32_RATE_FRAC is 4096
          imu.gyro.q,
          imu.gyro.r,
          rot->phi,
          rot->theta,
          rot->psi,
          rates->p, rates->q, rates->r,
          pos->x, pos->y, pos->z,
          stabilization_cmd[COMMAND_THRUST],
          stabilization_cmd[COMMAND_ROLL],
          stabilization_cmd[COMMAND_PITCH],
          stabilization_cmd[COMMAND_YAW],
          actuators_bebop.rpm_obs[0],
          actuators_bebop.rpm_obs[1],
          actuators_bebop.rpm_obs[2],
          actuators_bebop.rpm_obs[3]);
  counter++;
}
