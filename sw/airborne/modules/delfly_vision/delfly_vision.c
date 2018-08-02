/*
 * Copyright (C) Matej Karasek
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
 * @file "modules/delfly_vision/delfly_vision.c"
 * @author Matej Karasek
 * Vision module for (tail less) DelFlies
 * Include delfly_vision.xml to your airframe file.
 * Define parameters STEREO_PORT, STEREO_BAUD
 */

#include "modules/delfly_vision/delfly_vision.h"

#include "mcu_periph/uart.h"
#include "subsystems/datalink/telemetry.h"
#include "pprzlink/messages.h"
#include "pprzlink/intermcu_msg.h"

#include "mcu_periph/sys_time.h"
//#include "subsystems/abi.h"



struct stereocam_t stereocam = {
  .device = (&((UART_LINK).device)),
  .msg_available = false
};
static uint8_t stereocam_msg_buf[256]  __attribute__((aligned));   ///< The message buffer for the stereocamera



void delfly_vision_init(void)
{
  // Initialize transport protocol
  pprz_transport_init(&stereocam.transport);
}

/* Parse the InterMCU message */
static void delfly_vision_parse_msg(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* Parse the stereocam message */
  uint8_t msg_id = stereocam_msg_buf[1];
  switch (msg_id) {

  case DL_STEREOCAM_VELOCITY: {
    static struct FloatVect3 camera_vel;

    float res = (float)DL_STEREOCAM_VELOCITY_resolution(stereocam_msg_buf);

    camera_vel.x = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_msg_buf)/res;
    camera_vel.y = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_msg_buf)/res;
    camera_vel.z = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_msg_buf)/res;

    float noise = 1-(float)DL_STEREOCAM_VELOCITY_vRMS(stereocam_msg_buf)/res;

    // Rotate camera frame to body frame
    struct FloatVect3 body_vel;
    float_rmat_transp_vmult(&body_vel, &stereocam.body_to_cam, &camera_vel);

//    //todo make setting
//    if (STEREOCAM_USE_MEDIAN_FILTER) {
//      // Use a slight median filter to filter out the large outliers before sending it to state
//      UpdateMedianFilterVect3Float(medianfilter, body_vel);
//    }
//
//    //Send velocities to state
//    AbiSendMsgVELOCITY_ESTIMATE(VEL_STEREOCAM_ID, now_ts,
//                                body_vel.x,
//                                body_vel.y,
//                                body_vel.z,
//                                noise,
//                                noise,
//                                noise
//                               );
    break;
  }

//#ifdef STEREOCAM_FOLLOWME
//  // todo is follow me still used?
//  case DL_STEREOCAM_FOLLOW_ME: {
//    follow_me( DL_STEREOCAM_FOLLOW_ME_headingToFollow(stereocam_msg_buf),
//               DL_STEREOCAM_FOLLOW_ME_heightObject(stereocam_msg_buf),
//               DL_STEREOCAM_FOLLOW_ME_distanceToObject(stereocam_msg_buf));
//    break;
//  }
//#endif

    default:
      break;
  }
}


void delfly_vision_periodic(void) {}


void delfly_vision_event(void)
{
   // Check if we got some message from the stereocam
   pprz_check_and_parse(stereocam.device, &stereocam.transport, stereocam_msg_buf, &stereocam.msg_available);

   // If we have a message we should parse it
   if (stereocam.msg_available) {
     delfly_vision_parse_msg();
     stereocam.msg_available = false;
   }
}


/**
 * Initialization of horizontal guidance module.
 */
void guidance_h_module_init(void) {}


/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
//  /* Reset the integrated errors */
//  opticflow_stab.err_vx_int = 0;
//  opticflow_stab.err_vy_int = 0;
//
//  /* Set rool/pitch to 0 degrees and psi to current heading */
//  opticflow_stab.cmd.phi = 0;
//  opticflow_stab.cmd.theta = 0;
//  opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool in_flight)
{
//  /* Update the setpoint */
//  stabilization_attitude_set_rpy_setpoint_i(&opticflow_stab.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}
