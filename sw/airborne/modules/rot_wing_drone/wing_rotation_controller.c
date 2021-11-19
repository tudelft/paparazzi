/*
 * Copyright (C) 2021 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rot_wing_drone/wing_rotation_controller.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module that controls the wing rotation of the rotating wing drone
 */

#include "modules/rot_wing_drone/wing_rotation_controller.h"

#include "mcu_periph/adc.h"
#include "math.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra_float.h"

#ifndef ADC_CHANNEL_WING_ROTATION_POSITION
#define ADC_CHANNEL_WING_ROTATION_POSITION ADC_6
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES
#define ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES 16
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_SERVO
#define ADC_CHANNEL_WING_ROTATION_SERVO ADC_5
#endif

#ifndef ADC_CHANNEL_WING_ROTATION_SERVO_NB_SAMPLES
#define ADC_CHANNEL_WING_ROTATION_SERVO_NB_SAMPLES 16
#endif

#ifndef WING_ROTATION_POSITION_ADC_0
#error "No WING_ROTATION_POSITION_ADC_0 defined"
#endif

#ifndef WING_ROTATION_POSITION_ADC_90
#error "No WING_ROTATION_POSITION_ADC_90 defined"
#endif

#ifndef WING_ROTATION_SERVO_ADC_MAX
#define WING_ROTATION_SERVO_ADC_MAX 3080
#endif

#ifndef ROT_WING_SERVO_CUTOFF_FREQUENCY
#define ROT_WING_SERVO_CUTOFF_FREQUENCY 0.5
#endif

#ifndef WING_ROTATION_MAX_CMD
#define WING_ROTATION_MAX_CMD MAX_PPRZ;
#endif

#ifndef WING_ROTATION_P_GAIN
#define WING_ROTATION_P_GAIN -100
#endif

#ifndef WING_ROTATION_DEADZONE_PPRZ_CMD
#define WING_ROTATION_DEADZONE_PPRZ_CMD 250
#endif

// Endpoint tuning
#ifndef WING_ROTATION_ENDPOINT_MARGIN_DEG
#define WING_ROTATION_ENDPOINT_MARGIN_DEG 30
#endif

#ifndef WING_ROTATION_ENDPOINT_CMD_THRESHOLD_0
#define WING_ROTATION_ENDPOINT_CMD_THRESHOLD_0 400
#endif

#ifndef WING_ROTATION_ENDPOINT_CMD_THRESHOLD_90
#define WING_ROTATION_ENDPOINT_CMD_THRESHOLD_90 400
#endif

#ifndef WING_ROTATION_ENDPOINT_SERVO_RATE_LIMIT
#define WING_ROTATION_ENDPOINT_SERVO_RATE_LIMIT 0.5
#endif

static struct adc_buf buf_wing_rot_pos;
static struct adc_buf buf_wing_rot_servo;

// Parameters
struct wing_rotation_controller wing_rotation;

// Define filter(s)
Butterworth2LowPass rot_wing_servo_rate_filter;

// Inline functions
inline void wing_rotation_to_rad(void);
inline void wing_servo_to_rad(void);
inline void wing_rotation_update_sp(void);

// Telemetry
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_rot_wing_controller(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROT_WING_CONTROLLER(trans, dev, AC_ID,
                          &wing_rotation.wing_angle_deg,
                          &wing_rotation.wing_angle_deg_sp,
                          &wing_rotation.adc_wing_rotation,
                          &wing_rotation.adc_wing_servo,
                          &wing_rotation.servo_pprz_cmd);
}
#endif

void wing_rotation_init(void)
{
  // your init code here
  adc_buf_channel(ADC_CHANNEL_WING_ROTATION_POSITION, &buf_wing_rot_pos, ADC_CHANNEL_WING_ROTATION_POSITION_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_WING_ROTATION_SERVO, &buf_wing_rot_servo, ADC_CHANNEL_WING_ROTATION_SERVO_NB_SAMPLES);

  // Init wing_rotation_controller struct
  wing_rotation.servo_pprz_cmd = 0;
  wing_rotation.pprz_cmd_deadzone = WING_ROTATION_DEADZONE_PPRZ_CMD;
  wing_rotation.adc_wing_rotation = 0;
  wing_rotation.adc_wing_servo = 0;
  wing_rotation.wing_angle_rad = 0;
  wing_rotation.wing_angle_deg = 0;
  wing_rotation.wing_angle_rad_sp = 0;
  wing_rotation.wing_angle_deg_sp = 0;
  wing_rotation.servo_angle_rad = 0;
  wing_rotation.servo_rate = 0;

  wing_rotation.adc_wing_rotation_range = WING_ROTATION_POSITION_ADC_90 - WING_ROTATION_POSITION_ADC_0;

  // Set wing angle to current wing angle
  wing_rotation.initialized = false; 
  wing_rotation.init_loop_count = 0;

  wing_rotation.p_gain = WING_ROTATION_P_GAIN;
  wing_rotation.max_cmd = WING_ROTATION_MAX_CMD;

  // Endpoint tuning
  wing_rotation.cmd_threshold_0 = WING_ROTATION_ENDPOINT_CMD_THRESHOLD_0;
  wing_rotation.cmd_threshold_90 = WING_ROTATION_ENDPOINT_CMD_THRESHOLD_90;
  wing_rotation.threshold_servo_rate_limit = WING_ROTATION_ENDPOINT_SERVO_RATE_LIMIT;

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROT_WING_CONTROLLER, send_rot_wing_controller);
  #endif

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * ROT_WING_SERVO_CUTOFF_FREQUENCY);
  float sample_time = 1. / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&rot_wing_servo_rate_filter, tau, sample_time, 0.0);
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
      wing_rotation.wing_angle_rad_sp = wing_rotation.wing_angle_rad;
      wing_rotation.wing_angle_deg_sp = wing_rotation.wing_angle_rad_sp / M_PI * 180.;
    }
  }
}

void wing_rotation_event(void)
{
  // Update Wing position sensor
  wing_rotation_to_rad();
  wing_servo_to_rad();

  // Propagate filters
  update_butterworth_2_low_pass(&rot_wing_servo_rate_filter, wing_rotation.servo_rate);

  // Run control if initialized
  if (wing_rotation.initialized) {
    wing_rotation_update_sp();

    // Calculate rad error
    float pos_error_rad = wing_rotation.wing_angle_rad_sp - wing_rotation.wing_angle_rad;
    float pos_error_deg = pos_error_rad / M_PI * 180.;

    int32_t servo_pprz_cmd;  // Define pprz cmd

    // Check if in endpoint region
    bool endpoint_reaching = false;

    // Give command until filter rate is below threshold and stop
    /*
    if (wing_rotation.wing_angle_deg_sp < WING_ROTATION_ENDPOINT_MARGIN_DEG && pos_error_deg > -WING_ROTATION_ENDPOINT_MARGIN_DEG)
    {
      endpoint_reaching = true;
      if (fabs(rot_wing_servo_rate_filter.o[0]) < wing_rotation.threshold_servo_rate_limit)
      {
        servo_pprz_cmd = 0;
      } else {
        servo_pprz_cmd = wing_rotation.cmd_threshold_0;
      }
    } 
    else if (wing_rotation.wing_angle_deg_sp > (90 - WING_ROTATION_ENDPOINT_MARGIN_DEG) && pos_error_deg < WING_ROTATION_ENDPOINT_MARGIN_DEG)
    {
      endpoint_reaching = true;
      if (fabs(rot_wing_servo_rate_filter.o[0]) < wing_rotation.threshold_servo_rate_limit)
      {
        servo_pprz_cmd = 0;
      } else {
        servo_pprz_cmd = -wing_rotation.cmd_threshold_90;
      }
    }
    */
    if (!endpoint_reaching) 
    {
      // Control the wing servo using P control and bound command
      servo_pprz_cmd = wing_rotation.p_gain * pos_error_rad;
      servo_pprz_cmd = Max(servo_pprz_cmd, (int32_t)-wing_rotation.max_cmd);
      servo_pprz_cmd = Min(servo_pprz_cmd, (int32_t)wing_rotation.max_cmd);
    }
    
    if (fabs(servo_pprz_cmd) < wing_rotation.pprz_cmd_deadzone) {
      servo_pprz_cmd = 0;
    }

    wing_rotation.servo_pprz_cmd = servo_pprz_cmd;
  }
}

void wing_rotation_to_rad(void)
{
  wing_rotation.adc_wing_rotation = buf_wing_rot_pos.sum / buf_wing_rot_pos.av_nb_sample;

  wing_rotation.wing_angle_rad = (float)(wing_rotation.adc_wing_rotation - WING_ROTATION_POSITION_ADC_0)
                                   / (float)wing_rotation.adc_wing_rotation_range * (0.5 * M_PI);

  wing_rotation.wing_angle_deg = wing_rotation.wing_angle_rad / M_PI * 180.;
}

void wing_servo_to_rad(void)
{
  wing_rotation.adc_wing_servo = buf_wing_rot_servo.sum / buf_wing_rot_servo.av_nb_sample;

  float servo_angle = (float)wing_rotation.adc_wing_servo / (float)WING_ROTATION_SERVO_ADC_MAX * 2. * M_PI;

  // Normalize servo rate
  wing_rotation.servo_rate = (servo_angle - wing_rotation.servo_angle_rad) * PERIODIC_FREQUENCY;
  FLOAT_ANGLE_NORMALIZE(wing_rotation.servo_rate);
  wing_rotation.servo_angle_rad = servo_angle;
}

void wing_rotation_update_sp(void)
{
  wing_rotation.wing_angle_rad_sp = wing_rotation.wing_angle_deg_sp / 180. * M_PI;
}