/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2019 Tom van Dijk <tomvand@users.noreply.github.com>
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

/** @file modules/loggers/logger_file.c
 *  @brief File logger for Linux based autopilots
 */

#include "logger_file.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "std.h"
#include "modules/core/abi.h"

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "generated/modules.h"

int32_t color_count_log = 0;
int32_t floor_count_log = 0;
int32_t floor_central_count_log = 0;
int32_t plant_count_log = 0;
int16_t heading_log = 0;


/** Set the default File logger path to the USB drive */
#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/video/usb
#endif

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count_log = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, __attribute__((unused)) int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count_log = quality;
}

#ifndef GROUND_CENTRAL_VISUAL_DETECTION_ID
#define GROUND_CENTRAL_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event ground_central_detection_ev;
static void ground_central_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_central_count_log = quality;
}

#ifndef PLANT_VISUAL_DETECTION_ID
#define PLANT_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event plant_detection_ev;
static void plant_count_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  plant_count_log = quality;
}

#ifndef GROUND_SPLIT_ID
#define GROUND_SPLIT_ID ABI_BROADCAST
#endif
static abi_event ground_detection_ev;
static void ground_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t new_direction)
{
  heading_log = new_direction;
}

// int32_t floor_central_count_logger = 0;   

// #ifndef GROUND_CENTRAL_VISUAL_DETECTION_ID
// #define GROUND_CENTRAL_VISUAL_DETECTION_ID ABI_BROADCAST
// #endif
// static abi_event ground_central_detection_ev;
// static void ground_central_cb(uint8_t __attribute__((unused)) sender_id,
//                                int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
//                                int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
//                                int32_t quality, int16_t __attribute__((unused)) extra)
// {
//   floor_central_count_logger = quality;
// }


#ifndef GROUP11_GROUND_DETECT_ID
#define GROUP11_GROUND_DETECT_ID ABI_BROADCAST
#endif
static abi_event group11_ground_detect_ev;
static void ground_central_cb(uint8_t __attribute__((unused)) sender_id,
                              int8_t navigation_state_msg, int32_t central_floor_count_threshold,
                              )
{
  central_floor_count_threshold_logger = central_floor_count_threshold;
  navigation_state_msg_logger = navigation_state_msg;
}




/** The file pointer */
static FILE *logger_file = NULL;


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in logger_file_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void logger_file_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "color_count,");
  fprintf(file, "floor_count,");
  fprintf(file, "floor_count_central,");
  fprintf(file, "plat_count,");
  fprintf(file, "heading,");


  // fprintf(file, "floor_central_count,");
  fprintf(file, "navigation_state_msg,");
  fprintf(file, "central_floor_count_threshold,");
#ifdef BOARD_BEBOP
  fprintf(file, "rpm_obs_1,rpm_obs_2,rpm_obs_3,rpm_obs_4,");
  fprintf(file, "rpm_ref_1,rpm_ref_2,rpm_ref_3,rpm_ref_4,");
#endif
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_header(file);
#endif
#ifdef COMMAND_THRUST
  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
#else
  fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void logger_file_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  fprintf(file, "%d,", color_count_log);
  fprintf(file, "%d,", floor_count_log);
  fprintf(file, "%d,", floor_central_count_log);
  fprintf(file, "%d,", plant_count_log);
  fprintf(file, "%d,", heading_log);

  // fprintf(file, "%f,", floor_central_count_logger);
  fprintf(file, "%d,", central_floor_count_threshold_logger);      // idem v
  fprintf(file, "%d,", navigation_state_msg_logger);               // UNSURE ABOUT navigation_state_msg OR navigation_state_msg_logger
#ifdef BOARD_BEBOP
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_obs[0],actuators_bebop.rpm_obs[1],actuators_bebop.rpm_obs[2],actuators_bebop.rpm_obs[3]);
  fprintf(file, "%d,%d,%d,%d,",actuators_bebop.rpm_ref[0],actuators_bebop.rpm_ref[1],actuators_bebop.rpm_ref[2],actuators_bebop.rpm_ref[3]);
#endif
#ifdef INS_EXT_POSE_H
  ins_ext_pos_log_data(file);
#endif
#ifdef COMMAND_THRUST
  fprintf(file, "%d,%d,%d,%d\n",
      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif
}


/** Start the file logger and open a new file */
void logger_file_start(void)
{
  // Ensure that the module is running when started with this function
  logger_file_logger_file_periodic_status = MODULES_RUN;

  // AbiBindMsgVISUAL_DETECTION(GROUND_CENTRAL_VISUAL_DETECTION_ID, &ground_central_detection_ev, ground_central_cb);
  
  AbiBindMsgGROUP11_GROUND_DETECTION(GROUP11_GROUND_DETECT_ID, &group11_ground_detect_ev, group11_ground_detect_cb);
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
  AbiBindMsgVISUAL_DETECTION(GROUND_CENTRAL_VISUAL_DETECTION_ID, &ground_central_detection_ev, ground_central_cb);
  AbiBindMsgVISUAL_DETECTION(PLANT_VISUAL_DETECTION_ID, &plant_detection_ev, plant_count_cb);
  AbiBindMsgGROUND_DETECTION(GROUND_SPLIT_ID, &ground_detection_ev, ground_detection_cb);

  // Create output folder if necessary
  if (access(STRINGIFY(LOGGER_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(LOGGER_FILE_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[logger_file] Could not create log file directory %s.\n", STRINGIFY(LOGGER_FILE_PATH));
      return;
    }
  }

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(LOGGER_FILE_PATH), date_time);
  while ((logger_file = fopen(filename, "r"))) {
    fclose(logger_file);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(LOGGER_FILE_PATH), date_time, counter);
    counter++;
  }

  logger_file = fopen(filename, "w");
  if(!logger_file) {
    printf("[logger_file] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[logger_file] Start logging to %s...\n", filename);

  logger_file_write_header(logger_file);
}

/** Stop the logger an nicely close the file */
void logger_file_stop(void)
{
  if (logger_file != NULL) {
    fclose(logger_file);
    logger_file = NULL;
  }
}

/** Log the values to a csv file    */
void logger_file_periodic(void)
{
  if (logger_file == NULL) {
    return;
  }
  logger_file_write_row(logger_file);
}
