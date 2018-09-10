/*
 * Copyright (C) MAVLab
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
 * @file "modules/ctrl/dronerace//dronerace.c"
 * @author MAVLab
 * Autonomous Drone Race
 */


#include "modules/ctrl/dronerace/dronerace.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"

#include "subsystems/abi.h"

#include "state.h"

#include "filter.h"
#include "control.h"
#include "ransac.h"
#include "flightplan.h"


#include "subsystems/datalink/telemetry.h"



float dt = 1.0f / 512.f;


///////////////////////////////////////////////////////////////////////////////////////////////////
// LOGGING

#include <stdio.h>

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/ftp/internal_000
#endif

/** The file pointer */
static FILE *file_logger = NULL;

static void open_log(void) {
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
      file_logger,"Test\n");
  }
}

static void write_log(void)
{
  if (file_logger != 0) {
    fprintf(file_logger, "%f,%f,%f,%f,%d,%f,%f\n",dr_state.x, dr_state.y, dr_state.vx, dr_state.vy,
        dr_vision.cnt,dr_vision.dx,dr_vision.dy);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// TELEMETRY


// sending the divergence message to the ground station:
static void send_dronerace(struct transport_tx *trans, struct link_device *dev)
{
  float fx = dr_state.x; // Flows
  float fy = dr_state.y;
  float fz = dr_state.time;

  float cx = dr_state.vx; // Covariance
  float cy = dr_state.vy;
  float cz = 0;

  float gx = dr_vision.dx; // Gains
  float gy = dr_vision.dy;
  float gz = dr_vision.dz;

  float ex = dr_vision.cnt; // Error
  float ey = 0;
  float ez = POS_FLOAT_OF_BFP(guidance_v_z_sp);

  int32_t ix = dr_ransac.buf_size; // Error
  float iy = (float) dr_ransac.buf_index_of_last;
  float iz = dr_ransac.dt_max;

  pprz_msg_send_OPTICAL_FLOW_HOVER(trans, dev, AC_ID, &fx, &fy, &fz,
                                   &cx, &cy, &cz,
                                   &gx, &gy, &gz,
                                   &ex, &ey, &ez,
                                   &ix, &iy, &iz);
}


// ABI receive gates!
#ifndef DRONE_RACE_ABI_ID
#define DRONE_RACE_ABI_ID ABI_BROADCAST
#endif

static abi_event gate_detected_ev;


static void gate_detected_cb(uint8_t sender_id __attribute__((unused)), int32_t cnt, float dx, float dy, float dz, float vx, float vy, float vz)
{
  // Vision update
  dr_vision.cnt = cnt;
  dr_vision.dx = dx;
  dr_vision.dy = dy;
  dr_vision.dz = dz;
  filter_correct();
}

void dronerace_init(void)
{
  // Receive vision
  AbiBindMsgRELATIVE_LOCALIZATION(DRONE_RACE_ABI_ID, &gate_detected_ev, gate_detected_cb);

  // Send telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW_HOVER, send_dronerace);

  // Start Logging
  open_log();
}

float psi0 = 0;

void dronerace_enter(void)
{
  psi0 = stateGetNedToBodyEulers_f()->psi;
  filter_reset();
  control_reset();
}

void dronerace_periodic(void)
{
  float phi = stateGetNedToBodyEulers_f()->phi - RadOfDeg(-1.5);
  float theta = stateGetNedToBodyEulers_f()->theta - RadOfDeg(2.0);
  float psi = stateGetNedToBodyEulers_f()->psi - psi0;

  filter_predict(phi,theta,psi, dt);

  write_log();

  // Show position on the map
  /*
  struct NedCoor_f pos;
  pos.x = dr_state.x;
  pos.y = dr_state.y;
  pos.z = sonar_bebop.distance; // Hardcoded push of sonar to the altitude
  //stateSetPositionNed_f(&pos);
  */
}

void dronerace_set_rc(UNUSED float rt, UNUSED float rx, UNUSED float ry, UNUSED float rz)
{
}

void dronerace_get_cmd(float* alt, float* phi, float* theta, float* psi_cmd)
{

  control_run(dt);

  *phi = dr_control.phi_cmd;
  *theta = dr_control.theta_cmd;
  *psi_cmd = dr_control.psi_cmd + psi0;

  guidance_v_z_sp = POS_BFP_OF_REAL(-dr_control.alt_cmd);
}

