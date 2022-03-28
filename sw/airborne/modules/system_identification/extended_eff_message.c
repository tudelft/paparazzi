/*
 * Copyright (C) 2022 Tomaso De Ponti <tomasodp@gmail.com>
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

/** @file "modules/system_identification/extended_eff_message.c"
 * @author Tomaso De Ponti <tomasodp@gmail.com>
 * Module to create a comprehensive module which encompasses all the required data to perform effectiveness estimations. This is done to synchronize all signals.
 */

#include "modules/system_identification/extended_eff_message.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "subsystems/actuators.h"
#include "modules/system_identification/sys_id_doublet.h"
#include "modules/system_identification/sys_id_chirp.h"
#include "modules/rot_wing_drone/wing_rotation_controller.h"

#ifndef EFFECTIVENESS_FULL_ID
#define EFFECTIVENESS_FULL_ID ABI_BROADCAST
#endif

//struct FloatRates *body_rates_full;
//struct Int32Vect3 *body_accel_i_full;
//struct FloatVect3 body_accel_f_telem_full;
//float airspeed;
//static abi_event effectiveness_full_ev;

//static void effectiveness_full_cb(uint8_t sender_id, float eas)
//{
  // your abi callback code here
//}



#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_eff_full_indi(struct transport_tx *trans, struct link_device *dev)
{
struct FloatRates *body_rates_full = stateGetBodyRates_f();
struct Int32Vect3 *body_accel_i_full = stateGetAccelBody_i();
struct FloatVect3 body_accel_f_telem_full;
float airspeed = stateGetAirspeed_f();
//body_rates_full
//body_accel_i_full = stateGetAccelBody_i();
ACCELS_FLOAT_OF_BFP(body_accel_f_telem_full, *body_accel_i_full);
struct FloatEulers eulers_zxy;
float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());
int32_t state_psi = ANGLE_BFP_OF_REAL(eulers_zxy.psi);

pprz_msg_send_EFF_FULL_INDI(trans, dev, AC_ID,
                                        &body_rates_full->p,
                                        &body_rates_full->q,
                                        &body_rates_full->r,
                                        &ap_ref_save,
                                        &aq_ref_save,
                                        &ar_ref_save,
                                        &body_accel_f_telem_full.x,
                                        &body_accel_f_telem_full.y,
                                        &body_accel_f_telem_full.z,
                                        &airspeed,
                                        &doublet_active, 
                                        INDI_NUM_ACT, current_doublet_values,
                                        &doublet_axis, 
                                        &doublet_amplitude,
                                        &doub_number,
                                        &chirp_active,                    
                                        &chirp_axis, 
                                        &chirp_amplitude, 
                                        &chirp_number,
                                        &chirp_fstart_hz, 
                                        &chirp_fstop_hz, 
                                        3, current_chirp_values,
                                        &wing_rotation.wing_angle_deg,
                                        &wing_rotation.wing_angle_deg_sp,
                                        &(stateGetNedToBodyEulers_i()->phi),
                                        &(stateGetNedToBodyEulers_i()->theta),
                                        &state_psi,
                                        &actuator_thrust_bx_pprz,
                                        ACTUATORS_NB, actuators
                                        );
}
#endif

void extended_eff_message_init(void)
{
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_EFF_FULL_INDI, send_eff_full_indi);
  #endif 
}

  // Abi messages bindings
  //AbiBindMsgAIRSPEED(EFFECTIVENESS_FULL_ID, &effectiveness_full_ev, effectiveness_full_cb);


