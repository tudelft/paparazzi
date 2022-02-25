//
// Created by matteo on 06/08/2021.
//
#include <math.h>
#include "state.h"

#include "subsystems/navigation/common_nav.h"
#include "subsystems/datalink/telemetry.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"

#include "cyclic_controller.h"

struct cyclic_controller_t cyclic_controller;

int32_t cc_x_ned, cc_y_ned, cc_x_ecef, cc_y_ecef;
int32_t cc_vx_ned, cc_vy_ned, cc_vx_ecef, cc_vy_ecef;
uint32_t cc_hor_norm;
int32_t cc_hor_dir;

// GCS settings
//float cc_kp_gain = 1;
//float cc_kd_gain = 1;
//float max_amplitude = 0.3;

/*
 * <message name="FRISBEE_CONTROL" id="55">
     <field name="x_ned" type="int32">State x position NED</field>
     <field name="y_ned" type="int32">State y position NED</field>
     <field name="x_ecef" type="int32">State x position ECEF</field>
     <field name="y_ecef" type="int32">State y position ECEF</field>
     <field name="vx_ned" type="int32">State x velocity NED</field>
     <field name="vy_ned" type="int32">State y velocity NED</field>
     <field name="vx_ecef" type="int32">State x velocity ECEF</field>
     <field name="vy_ecef" type="int32">State y velocity ECEF</field>
     <field name="hor_norm" type="uint32">Horizontal ground speed norm</field>
     <field name="hor_dir" type="int32">Horizontal ground speed direction</field>
   </message>
 */
static void send_frisbee_control(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_FRISBEE_CONTROL(trans, dev, AC_ID, &cc_x_ned, &cc_y_ned, &cc_x_ecef, &cc_y_ecef, &cc_vx_ned, &cc_vy_ned, &cc_vx_ecef, &cc_vy_ecef, &cc_hor_norm, &cc_hor_dir);
}

void cyclic_controller_init() {
  NavSetWaypointHere(WP_STDBY);

  cc_x_ned = stateGetPositionNed_i() -> x;
  cc_y_ned = stateGetPositionNed_i() -> y;
  cc_x_ecef = stateGetPositionEcef_i() -> x;
  cc_y_ecef = stateGetPositionEcef_i() -> y;

  cc_vx_ned = stateGetSpeedNed_i() -> x;
  cc_vy_ned = stateGetSpeedNed_i() -> y;
  cc_vx_ecef = stateGetSpeedEcef_i() -> x;
  cc_vy_ecef = stateGetSpeedEcef_i() -> y;

  cc_hor_norm = stateGetHorizontalSpeedNorm_i();
  cc_hor_dir  = stateGetHorizontalSpeedDir_i();  // rad

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FRISBEE_CONTROL, send_frisbee_control);
}

void cyclic_controller_run() {
  cc_x_ned = stateGetPositionNed_i() -> x;
  cc_y_ned = stateGetPositionNed_i() -> y;
  cc_x_ecef = stateGetPositionEcef_i() -> x;
  cc_y_ecef = stateGetPositionEcef_i() -> y;

  cc_vx_ned = stateGetSpeedNed_i() -> x;
  cc_vy_ned = stateGetSpeedNed_i() -> y;
  cc_vx_ecef = stateGetSpeedEcef_i() -> x;
  cc_vy_ecef = stateGetSpeedEcef_i() -> y;

  cc_hor_norm = stateGetHorizontalSpeedNorm_i();
  cc_hor_dir  = stateGetHorizontalSpeedDir_i();  // rad
}

