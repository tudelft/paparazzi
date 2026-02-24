/*
 * Copyright (C) 2026 Evangelos Ntouros <e.ntouros@tudelft.nl>
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

/** @file "modules/flatness/flatness_stabilization.c"
 * @author Evangelos Ntouros <e.ntouros@tudelft.nl>
 * A differential flatness-based INDI stabilization controller.
 */

#include "modules/flatness/flatness_stabilization.h"
#include "modules/actuators/actuators.h"

// #include "modules/datalink/telemetry.h"


// Model
#define PHI_COEFF_MULTIPLIER 10000000
#define MU_X 70 * PHI_COEFF_MULTIPLIER
#define MU_Y 150 * PHI_COEFF_MULTIPLIER
#define MU_Z 15 * PHI_COEFF_MULTIPLIER
#define C_Z  5 * PHI_COEFF_MULTIPLIER

#define Bx 0
#define By 1
#define Bz 2

struct Fl_stabilization fl_stabilization;
float uf[4]; 

void flatness_stabilization_run(bool in_flight, struct StabilizationSetpoint *att_sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
    for (int i = 0; i < ACTUATORS_NB; i++) {
        actuators_pprz[i] = (int16_t) thrust->sp.thrust_i[THRUST_AXIS_Z];
    }

    cmd[COMMAND_THRUST] = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3])/4;
    stabilization.cmd[COMMAND_THRUST] = cmd[COMMAND_THRUST]; // for autopilot_check_in_flight()

    // angaccel_cmd = att_control(quat_cmd, quat, 0, rate);

    // m_filt = forw_rot_flatness(u_filt);

    // // incremental law
    // m_cmd = (angaccel_cmd - angaccel_filt) + m_filt;

    // u_cmd = inv_rot_flatness(specific_thrust, m_cmd)

    struct FloatQuat quat_cmd = stab_sp_to_quat_f(att_sp);
    
    printf("%f\t%f\t%f\t%f\n", quat_cmd.qi, quat_cmd.qx, quat_cmd.qy, quat_cmd.qz);
}

// float forw_rot_flatness(float *u)
// {
//     float m[3];

//     m[Bx] = MU_X*(u[0]^2 - u[1]^2 - u[2]^2 + u[3]^2);
//     m[By] = MU_Y*(u[0]^2 + u[1]^2 - u[2]^2 - u[3]^2);
//     m[Bz] = MU_Z*(-u[0]^2 + u[1]^2 - u[2]^2 + u[3]^2);
    
//     return m;
// }

void att_control(void)
{

}

void inv_rot_flatness(void)
{

}