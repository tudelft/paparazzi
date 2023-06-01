/*
 * Copyright (C) 2023 Tomaso De Ponti <tmldeponti@tudelft.nl>
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

/** @file "firmwares/rotorcraft/oneloop/oneloop_andi.h"
 * @author Tomaso De Ponti <tmldeponti@tudelft.nl>
 * One loop (Guidance + Stabilization) ANDI controller for the rotating wing drone RW3C
 */

#ifndef ONELOOP_ANDI_H
#define ONELOOP_ANDI_H

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#define ANDI_G_SCALING 1000.0f

extern float att_ref[3];
extern float att_d_ref[3];
extern float att_2d_ref[3];
extern float att_3d_ref[3];
extern float pos_ref[3];
extern float pos_d_ref[3];
extern float pos_2d_ref[3];
extern float pos_3d_ref[3];
extern float att_1l[3];
extern float att_d[3]; 
extern float att_2d[3];
extern float pos_1l[3];
extern float pos_d[3];
extern float pos_2d[3];
extern float act_state_filt_vect_1l[ANDI_NUM_ACT];
extern float nu[6];

extern float g1g2_1l[ANDI_OUTPUTS][ANDI_NUM_ACT];

extern float andi_u[ANDI_NUM_ACT];

extern void oneloop_andi_init(void);
extern void oneloop_andi_enter(void);
extern void oneloop_andi_set_failsafe_setpoint(void);
extern void oneloop_andi_attitude_run(struct Int32Quat quat_sp, struct FloatVect3 pos_des, bool in_flight);
extern void oneloop_andi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
//extern int32_t stabilization_cmd[COMMANDS_NB];
#endif  // ONELOOP_ANDI_H
