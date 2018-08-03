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

//  case DL_STEREOCAM_VELOCITY: {
////    <message name="STEREOCAM_VELOCITY" id="81">
////      <description>Velocity measured using optical flow and stereovision. All parameters are in the camera frame</description>
////      <field name="resolution" type="uint8">Resolution of the vel and pos messages</field>
////      <field name="dt_frame"   type="uint8">Time difference to previous frame</field>
////      <field name="dt"         type="uint8">Time difference to previous message, not strictly required</field>
////      <field name="velx"       type="int16" unit="m/s">Velocity estimaed using stereo edgeflow</field>
////      <field name="vely"       type="int16" unit="m/s"/>
////      <field name="velz"       type="int16" unit="m/s"/>
////      <field name="dposx"      type="int16" unit="m">Distance traveled since the last message</field>
////      <field name="dposy"      type="int16" unit="m"/>
////      <field name="dposz"      type="int16" unit="m"/>
////      <field name="vRMS"       type="uint8">RMS of the velocity estimate</field>
////      <field name="posRMS"     type="uint8">RMS of the position</field>
////      <field name="avg_dist"   type="uint16">Average distance to scene</field>
////    </message>
//
//    static struct FloatVect3 camera_vel;
//
//    float res = (float)DL_STEREOCAM_VELOCITY_resolution(stereocam_msg_buf);
//
//    camera_vel.x = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_msg_buf)/res;
//    camera_vel.y = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_msg_buf)/res;
//    camera_vel.z = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_msg_buf)/res;
//
//    float noise = 1-(float)DL_STEREOCAM_VELOCITY_vRMS(stereocam_msg_buf)/res;
//
//    // Rotate camera frame to body frame
//    struct FloatVect3 body_vel;
//    float_rmat_transp_vmult(&body_vel, &stereocam.body_to_cam, &camera_vel);
//
////    //todo make setting
////    if (STEREOCAM_USE_MEDIAN_FILTER) {
////      // Use a slight median filter to filter out the large outliers before sending it to state
////      UpdateMedianFilterVect3Float(medianfilter, body_vel);
////    }
////
////    //Send velocities to state
////    AbiSendMsgVELOCITY_ESTIMATE(VEL_STEREOCAM_ID, now_ts,
////                                body_vel.x,
////                                body_vel.y,
////                                body_vel.z,
////                                noise,
////                                noise,
////                                noise
////                               );
//    break;
//  }

  case DL_STEREOCAM_GATE: {

//    <message name="STEREOCAM_GATE" id="84">
//          <description>Result of the gate detection algorithm on the stereocamera</description>
//          <field name="quality" type="uint8">Measure of how certainty of gate identificaiton, min 15, max 100</field>
//          <field name="width"   type="float" unit="rad"/>
//          <field name="height"  type="float" unit="rad"/>
//          <field name="phi"     type="float" unit="rad">Bearing of the gate in the camera frame</field>
//          <field name="theta"   type="float" unit="rad"/>
//          <field name="depth"   type="float" unit="m">Set to 0 is not known</field>
//        </message>

//      gate.quality = DL_STEREOCAM_GATE_quality(stereocam_msg_buf);
//      gate.width = DL_STEREOCAM_GATE_width(stereocam_msg_buf);
//      gate.height = DL_STEREOCAM_GATE_height(stereocam_msg_buf);
//      gate.psi = DL_STEREOCAM_GATE_phi(stereocam_msg_buf);
//      gate.theta = DL_STEREOCAM_GATE_theta(stereocam_msg_buf);
//      gate.depth = DL_STEREOCAM_GATE_depth(stereocam_msg_buf);
    gate.quality = 4;
    gate.width = DL_STEREOCAM_GATE_width(stereocam_msg_buf);
    gate.height = DL_STEREOCAM_GATE_height(stereocam_msg_buf);
    gate.psi = DL_STEREOCAM_GATE_phi(stereocam_msg_buf);
    gate.theta = DL_STEREOCAM_GATE_theta(stereocam_msg_buf);
    gate.depth = DL_STEREOCAM_GATE_depth(stereocam_msg_buf);

       break;
    }

    default:
      gate.quality = 3;

      break;
  }
}


void delfly_vision_periodic(void) {
}

void delfly_vision_event(void)
{
   // Check if we got some message from the stereocam
   pprz_check_and_parse(stereocam.device, &stereocam.transport, stereocam_msg_buf, &stereocam.msg_available);
//   gate.quality = 1;


   // If we have a message we should parse it
   if (stereocam.msg_available) {
     gate.quality = 2;
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

  // IF vision switch is on
//  stab_cmd.phi=rc_sp.phi;
//  stab_cmd.theta=rc_sp.theta;
//  stab_cmd.psi=rc_sp.psi+gate.psi;
  // ELSE
  stab_cmd.phi=rc_sp.phi;
  stab_cmd.theta=rc_sp.theta;
  stab_cmd.psi=rc_sp.psi;

  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&stab_cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}
