/*
 * Copyright (C) 2014 OpenUAS
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

#include "ground_detect.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "filters/low_pass_filter.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#define GROUND_DETECT_COUNTER_TRIGGER 10

// Cutoff frequency of accelerometer filter in Hz
#define GROUND_DETECT_FILT_FREQ 5.0

Butterworth2LowPass accel_filter;

bool disarm_on_not_in_flight = false;

int32_t counter = 0;
bool ground_detected = false;

void ground_detect_init() {
  float tau = 1.0 / (2.0 * M_PI * GROUND_DETECT_FILT_FREQ);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_filter, tau, sample_time, 0.0);
}

bool ground_detect(void) {
  return ground_detected;
}

void ground_detect_periodic() {

  // Evaluate thrust given (less than hover thrust)
  // Use the control effectiveness in thrust in order to estimate the thrust delivered (only works for multicopters)
  float specific_thrust = 0.0; // m/s2
  uint8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    specific_thrust += actuator_state_filt_vect[i] * g1g2[3][i] * -((int32_t) act_is_servo[i] - 1);
  }

  // vertical component
  float spec_thrust_down;
  struct FloatRMat *ned_to_body_rmat = stateGetNedToBodyRMat_f();
  spec_thrust_down = ned_to_body_rmat->m[8] * specific_thrust;

  // Evaluate vertical speed (close to zero, not at terminal velocity)
  float vspeed_ned = stateGetSpeedNed_f()->z;

  // Detect free fall (to be done, rearm?)
  bool free_fall = false;

  // Detect noise level (to be done)

  // Detect ground
  if ( (fabsf(vspeed_ned) < 5.0) && (spec_thrust_down > -5.0) && (fabsf(accel_filter.o[0]) < 2.0)  ) {
    counter +=1;
    if (counter > 5) {
      ground_detected = true;

      if (disarm_on_not_in_flight) {
        autopilot_set_motors_on(false);
        disarm_on_not_in_flight = false;
      }
    }
  } else {
    ground_detected = false;
    counter = 0;
  }

#ifdef DEBUG_GROUND_DETECT
  uint8_t test_gd = ground_detected;
  float payload[4];
  payload[0] = vspeed_ned;
  payload[1] = spec_thrust_down;
  payload[2] = accel_filter.o[0];
  payload[3] = stateGetAccelNed_f()->z;

  RunOnceEvery(10, {DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, &test_gd); DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, payload);} );
#endif
}

/**
 * Filter the vertical acceleration with a low cutoff frequency.
 *
 */
void ground_detect_filter_accel(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&accel_filter, accel->z);
}
