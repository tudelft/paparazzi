/*
 * Copyright (C) 2014 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 *               2018 Freek van Tienen <freek.v.tienen@gmail.com>
 * This is code for guidance of the hybrid DelftaCopter UAV.
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
 */

/** @file firmwares/rotorcraft/guidance/guidance_delftacopter.h
 *  Guidance controllers (horizontal and vertical) for the Hybrid Delftacopter UAV configurations.
 *  This is an extension of the normal hybrid controller and replaces most of its functionality.
 */

#ifndef GUIDANCE_DELFTACOPTER_H
#define GUIDANCE_DELFTACOPTER_H

#include "math/pprz_algebra_int.h"
#include "guidance_hybrid.h"
#include "filters/low_pass_filter.h"

extern int32_t guidance_feed_forward_yaw_which_is_delftacopter_roll;  ///< Rate FF in forward mode.

extern int32_t v_control_pitch;
extern float vertical_setpont_outback;
extern int32_t nominal_forward_thrust;
extern float vertical_pgain;
extern float vertical_dgain;
extern float vertical_pitch_of_roll;
extern float low_airspeed_pitch_gain;
enum hybrid_mode_t {HB_HOVER, HB_FORWARD};
extern enum hybrid_mode_t dc_hybrid_mode;
extern struct Int32Eulers guidance_hybrid_ypr_sp;
extern float perpen_dgain;
extern float throttle_from_pitch_up;
extern float forward_max_psi;
extern int16_t transition_throttle_to_forward;
extern int16_t transition_throttle_to_hover;
extern float acc_y_filter_cutoff_hz;
extern float e_psi_deg_from_acc_y;

extern bool delftacopter_fwd_controller_enabled;
extern float delftacopter_fwd_roll;
extern float delftacopter_fwd_pitch;
extern float delftacopter_fwd_yaw;
extern float delftacopter_fwd_roll_pgain;
extern float delftacopter_fwd_roll_dgain;
extern float delftacopter_fwd_pitch_pgain;
extern float delftacopter_fwd_pitch_dgain;
extern float delftacopter_fwd_pitch_igain;
extern float feedforward_yaw_of_turn_rate;
extern float delftacopter_fwd_advance_angle_p;
extern float delftacopter_fwd_advance_angle_q;
extern float delftacopter_fwd_pitch_swp;

extern void guidance_delftacopter_set_acc_y_cutoff_hz(float cutoff_hz);

#endif /* GUIDANCE_DELFTACOPTER_H */
