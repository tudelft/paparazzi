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

extern void set_wind_heading_to_current90(void);

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
extern float wind_heading_deg;
extern bool has_transitioned;

#endif /* GUIDANCE_DELFTACOPTER_H */