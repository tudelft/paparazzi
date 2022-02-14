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

int32_t x_ned, y_ned, x_ecef, y_ecef;
int32_t vx_ned, vy_ned, vx_ecef, vy_ecef;
uint32_t hor_norm;
int32_t hor_dir;

// GCS settings
float cc_kp_gain = 1;
float cc_kd_gain = 1;
float max_amplitude = 0.3;

/*
 * <message name="FRISBEE_CONTROL" id="55">
     <field name="x_ned" type="int32_t">State x position NED</field>
     <field name="y_ned" type="int32_t">State y position NED</field>
     <field name="x_ecef" type="int32_t">State x position ECEF</field>
     <field name="y_ecef" type="int32_t">State y position ECEF</field>
     <field name="vx_ned" type="int32_t">State x velocity NED</field>
     <field name="vy_ned" type="int32_t">State y velocity NED</field>
     <field name="vx_ecef" type="int32_t">State x velocity ECEF</field>
     <field name="vy_ecef" type="int32_t">State y velocity ECEF</field>
     <field name="hor_norm" type="uint32_t">Horizontal ground speed norm</field>
     <field name="hor_dir" type="int32_t">Horizontal ground speed direction</field>
   </message>
 */
static void send_frisbee_control(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_FRISBEE_CONTROL(trans, dev, AC_ID, &x_ned, &y_ned, &x_ecef, &y_ecef, &vx_ned, &vy_ned, &vx_ecef, &vy_ecef, &hor_norm, &hor_dir);
}

void cyclic_controller_init() {
  NavSetWaypointHere(WP_STDBY);

  x_ned = stateGetPositionNed_i() -> x;
  y_ned = stateGetPositionNed_i() -> y;
  x_ecef = stateGetPositionEcef_i() -> x;
  y_ecef = stateGetPositionEcef_i() -> y;

  vx_ned = stateGetSpeedNed_i() -> x;
  vy_ned = stateGetSpeedNed_i() -> y;
  vx_ecef = stateGetSpeedEcef_i() -> x;
  vy_ecef = stateGetSpeedEcef_i() -> y;

  hor_norm = stateGetHorizontalSpeedNorm_i();
  hor_dir  = stateGetHorizontalSpeedDir_i();  // rad

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FRISBEE_CONTROL, send_frisbee_control);
}

void cyclic_controller_run() {
  x_ned = stateGetPositionNed_i() -> x;
  y_ned = stateGetPositionNed_i() -> y;
  x_ecef = stateGetPositionEcef_i() -> x;
  y_ecef = stateGetPositionEcef_i() -> y;

  vx_ned = stateGetSpeedNed_i() -> x;
  vy_ned = stateGetSpeedNed_i() -> y;
  vx_ecef = stateGetSpeedEcef_i() -> x;
  vy_ecef = stateGetSpeedEcef_i() -> y;

  hor_norm = stateGetHorizontalSpeedNorm_i();
  hor_dir  = stateGetHorizontalSpeedDir_i();  // rad
}

