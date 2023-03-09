/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid.h
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to ICRA2016 to learn more!
 *
 */

#ifndef GUIDANCE_INDI_HYBRID_H
#define GUIDANCE_INDI_HYBRID_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "filters/high_pass_filter.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"

// TODO change names for _indi_hybrid_

extern void guidance_indi_init(void);
extern void guidance_indi_enter(void);

extern struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accep_sp, float heading_sp);
extern struct StabilizationSetpoint guidance_indi_run_pos(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv);
extern struct StabilizationSetpoint guidance_indi_run_speed(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv);
extern struct StabilizationSetpoint guidance_indi_run_accel(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv);
extern void guidance_indi_propagate_filters(void);

struct guidance_indi_hybrid_params {
  float pos_gain;
  float pos_gainz;
  float speed_gain;
  float speed_gainz;
  float heading_bank_gain;
};

extern struct guidance_indi_hybrid_params gih_params;
extern float guidance_indi_specific_force_gain;
extern float guidance_indi_max_airspeed;
extern float nav_max_speed;
extern bool take_heading_control;
extern bool force_forward;       ///< forward flight for hybrid nav

extern float guidance_indi_max_bank;
extern float push_first_order_constant;
extern float a_diff_limit;
extern float a_diff_limit_z;
extern float rot_wing_max_pitch_limit_deg;
extern float rot_wing_min_pitch_limit_deg;
extern float airspeed_turn_lower_bound;

extern float pitch_priority_factor;
extern float roll_priority_factor;
extern float thrust_priority_factor;
extern float pusher_priority_factor;

#endif /* GUIDANCE_INDI_HYBRID_H */
