/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rot_wing_drone/wing_rotation_controller_v3a.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module to control wing rotation servo command based on prefered angle setpoint
 */

#include "modules/rot_wing_drone/wing_rotation_controller_v3a.h"
#include "modules/radio_control/radio_control.h"

#include <stdlib.h>
#include "mcu_periph/adc.h"

/*
#ifndef WING_ROTATION_SERVO_IDX
#error "No WING_ROTATION_SERVO_IDX defined"
#endif
*/

#if !USE_NPS

#ifndef ADC_CHANNEL_WING_ROTATION_POSITION
#define ADC_CHANNEL_WING_ROTATION_POSITION ADC_5
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES
#define ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES 16
#endif

#endif // !USE_NPS

#ifndef WING_ROTATION_POSITION_ADC_0
#error "No WING_ROTATION_POSITION_ADC_0 defined"
#endif

#ifndef WING_ROTATION_POSITION_ADC_90
#error "No WING_ROTATION_POSITION_ADC_90 defined"
#endif

#ifndef WING_ROTATION_FIRST_DYN
#define WING_ROTATION_FIRST_DYN 0.002
#endif

#ifndef WING_ROTATION_SECOND_DYN
#define WING_ROTATION_SECOND_DYN 0.006
#endif

// Parameters
struct wing_rotation_controller wing_rotation;

static struct adc_buf buf_wing_rot_pos;

// Inline functions
inline void wing_rotation_to_rad(void);
inline void wing_rotation_update_sp(void);
inline void wing_rotation_compute_pprz_cmd(void);

// Telemetry
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_rot_wing_controller(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROT_WING_CONTROLLER(trans, dev, AC_ID,
                          &wing_rotation.wing_angle_deg,
                          &wing_rotation.wing_angle_deg_sp,
                          &wing_rotation.adc_wing_rotation,
                          &wing_rotation.servo_pprz_cmd);
}
#endif

void wing_rotation_init(void)
{
  // your init code here
  #if !USE_NPS
  adc_buf_channel(ADC_CHANNEL_WING_ROTATION_POSITION, &buf_wing_rot_pos, ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES);
  #endif

  // Init wing_rotation_controller struct
  wing_rotation.servo_pprz_cmd = 0;
  wing_rotation.adc_wing_rotation = 0;
  wing_rotation.wing_angle_rad = 0;
  wing_rotation.wing_angle_deg = 0;
  wing_rotation.wing_angle_rad_sp = 0;
  wing_rotation.wing_angle_deg_sp = 0;
  wing_rotation.wing_rotation_speed = 0;
  wing_rotation.wing_angle_virtual_deg_sp = 0;
  wing_rotation.wing_rotation_first_order_dynamics = WING_ROTATION_FIRST_DYN;
  wing_rotation.wing_rotation_second_order_dynamics = WING_ROTATION_SECOND_DYN;
  wing_rotation.adc_wing_rotation_range = WING_ROTATION_POSITION_ADC_90 - WING_ROTATION_POSITION_ADC_0;

  // Set wing angle to current wing angle
  wing_rotation.initialized = false; 
  wing_rotation.init_loop_count = 0;

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROT_WING_CONTROLLER, send_rot_wing_controller);
  #endif
}

void wing_rotation_periodic(void)
{
  // your periodic code here.
  // freq = 1.0 Hz

  // After 5 loops, set current setpoint and enable wing_rotation
  if (!wing_rotation.initialized) {
    wing_rotation.init_loop_count += 1;
    if (wing_rotation.init_loop_count > 4) {
      wing_rotation.initialized = true;
      wing_rotation.wing_angle_rad_sp = 0;
      wing_rotation.wing_angle_deg_sp = wing_rotation.wing_angle_rad_sp / M_PI * 180.;
    }
  }
}

void wing_rotation_event(void)
{
  // First check if safety switch is triggered
  #ifdef WING_ROTATION_RESET_RADIO_CHANNEL
  // Update wing_rotation deg setpoint when RESET switch triggered
  if (radio_control.values[WING_ROTATION_RESET_RADIO_CHANNEL] > 1750)
  {
      wing_rotation.wing_angle_deg_sp = 0;
  }
  #endif

  // Update Wing position sensor
  wing_rotation_to_rad();

  // Run control if initialized
  if (wing_rotation.initialized) {
    wing_rotation_update_sp();

    //int32_t servo_pprz_cmd;  // Define pprz cmd

    // Control the wing rotation position.
    wing_rotation_compute_pprz_cmd();
    //servo_pprz_cmd = wing_rotation.wing_angle_deg_sp / 90 * MAX_PPRZ;
    //Bound(servo_pprz_cmd, 0, MAX_PPRZ);

    //wing_rotation.servo_pprz_cmd = servo_pprz_cmd;
  }
}

void wing_rotation_to_rad(void)
{ 
  #if !USE_NPS
  wing_rotation.adc_wing_rotation = buf_wing_rot_pos.sum / buf_wing_rot_pos.av_nb_sample;

  wing_rotation.wing_angle_deg =  -2.65302456619e-12 * (float)wing_rotation.adc_wing_rotation * (float)wing_rotation.adc_wing_rotation * (float)wing_rotation.adc_wing_rotation 
                                  + 3.743648588699e-7 * (float)wing_rotation.adc_wing_rotation * (float)wing_rotation.adc_wing_rotation 
                                  - 0.02145571907765 * (float)wing_rotation.adc_wing_rotation
                                  + 511.027281901372;
  wing_rotation.wing_angle_rad = wing_rotation.wing_angle_deg / 180. * M_PI;

  #else
  // Copy setpoint as actual angle in simulation
  wing_rotation.wing_angle_deg = wing_rotation.wing_angle_deg_sp;
  wing_rotation.wing_angle_rad = wing_rotation.wing_angle_rad_sp;
  #endif
}

void wing_rotation_update_sp(void)
{
  wing_rotation.wing_angle_rad_sp = wing_rotation.wing_angle_deg_sp / 180. * M_PI;
}

void wing_rotation_compute_pprz_cmd(void)
{
  #if !USE_NPS
  float angle_error = wing_rotation.wing_angle_deg_sp - wing_rotation.wing_angle_virtual_deg_sp;
  float speed_sp = wing_rotation.wing_rotation_first_order_dynamics * angle_error;
  float speed_error = speed_sp - wing_rotation.wing_rotation_speed;
  wing_rotation.wing_rotation_speed += wing_rotation.wing_rotation_second_order_dynamics * speed_error;
  wing_rotation.wing_angle_virtual_deg_sp += wing_rotation.wing_rotation_speed;
  
  int32_t servo_pprz_cmd;  // Define pprz cmd
  servo_pprz_cmd = (int32_t)(wing_rotation.wing_angle_virtual_deg_sp / 90. * (float)MAX_PPRZ);
  Bound(servo_pprz_cmd, 0, MAX_PPRZ);

  wing_rotation.servo_pprz_cmd = servo_pprz_cmd;
  #else
  int32_t servo_pprz_cmd;  // Define pprz cmd
  servo_pprz_cmd = (int32_t)(wing_rotation.wing_angle_deg_sp / 90. * (float)MAX_PPRZ);
  Bound(servo_pprz_cmd, 0, MAX_PPRZ);

  wing_rotation.servo_pprz_cmd = servo_pprz_cmd;
  #endif
}