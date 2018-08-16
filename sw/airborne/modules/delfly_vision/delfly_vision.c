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

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "subsystems/radio_control.h"


struct stereocam_t stereocam = {
  .device = (&((UART_LINK).device)),
  .msg_available = false
};
static uint8_t stereocam_msg_buf[256]  __attribute__((aligned));   ///< The message buffer for the stereocamera

static struct Int32Eulers stab_cmd;   ///< The commands that are send to the satbilization loop
static struct Int32Eulers rc_sp;   ///< Euler setpoints given by the rc

struct gate_t gate;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_delfly_vision_msg(struct transport_tx *trans, struct link_device *dev)
{
//  <field name="quality" type="uint8"/>
//        <field name="width"   type="float" unit="rad"/>
//        <field name="hieght"  type="float" unit="rad"/>
//        <field name="phi"     type="float" unit="rad"/>
//        <field name="theta"   type="float" unit="rad"/>
//        <field name="depth"   type="float" unit="m"/>

  pprz_msg_send_DELFLY_VISION(trans, dev, AC_ID,
                                  &gate.quality, &gate.width, &gate.height,
                                  &gate.psi, &gate.theta, &gate.depth);
}

#endif


void delfly_vision_init(void)
{
  // Initialize transport protocol
  pprz_transport_init(&stereocam.transport);

  gate.quality = 0;
  gate.width = 0;
  gate.height = 0;
  gate.psi = 0;
  gate.theta = 0;
  gate.depth = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DELFLY_VISION, send_delfly_vision_msg);
#endif

}

/* Parse the InterMCU message */
static void delfly_vision_parse_msg(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* Parse the stereocam message */
  uint8_t msg_id = pprzlink_get_msg_id(stereocam_msg_buf);
  switch (msg_id) {

    case DL_STEREOCAM_GATE: {

      gate.quality = DL_STEREOCAM_GATE_quality(stereocam_msg_buf);
      gate.width = DL_STEREOCAM_GATE_width(stereocam_msg_buf);
      gate.height = DL_STEREOCAM_GATE_height(stereocam_msg_buf);
      gate.psi = DL_STEREOCAM_GATE_phi(stereocam_msg_buf);
      gate.theta = DL_STEREOCAM_GATE_theta(stereocam_msg_buf);
      gate.depth = DL_STEREOCAM_GATE_depth(stereocam_msg_buf);

      break;
    }

    default:

      break;
  }
}


void delfly_vision_periodic(void) {
}

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
  /* Set rool/pitch to 0 degrees and psi to current heading */
  stab_cmd.phi = 0;
  stab_cmd.theta = 0;
  stab_cmd.psi = stateGetNedToBodyEulers_i()->psi;
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
  stabilization_attitude_read_rc_setpoint_eulers(&rc_sp, false, false, false);

  if (radio_control.values[RADIO_FLAP] > 5000) {
    stab_cmd.phi=rc_sp.phi;
    stab_cmd.theta=rc_sp.theta;
    stab_cmd.psi=stateGetNedToBodyEulers_i()->psi+gate.psi;
  }
  else
  {
    stab_cmd.phi=rc_sp.phi;
    stab_cmd.theta=rc_sp.theta;
    stab_cmd.psi=rc_sp.psi;
  }

  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&stab_cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}
