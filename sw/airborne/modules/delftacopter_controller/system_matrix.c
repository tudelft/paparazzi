/*
 * Copyright (C) 2018 Joost Meulenbeld
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file system_matrix.c
 * @brief Here the Delftacopter observer and controller matrices are defined
 * This is mainly done to have a single place for choosing which matrix to use
 * i.e. for the DC3 different matrices are required.
 */

#include "system_matrix.h"

#ifdef observer_DC3_6S6P_2600RPM_dccmsg_m50
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 0;
// -10.00, -10.00, -11.00, -11.00
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.967043,   0.008501,   0.000000,   0.027498},
	{ -0.055998,   0.967043,   0.222399,   0.000000},
	{ -0.006375,   0.001424,   0.992360,   0.006924},
	{  0.011517,   0.007826,  -0.068751,   0.992360}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.000149,   0.000026,   0.000000,   0.032930,  -0.008501},
	{ -0.001725,  -0.000199,   0.000000,   0.055998,   0.032738},
	{ -0.015307,  -0.001763,   0.000000,   0.006368,  -0.003370},
	{  0.011288,   0.001935,   0.000000,  -0.013463,  -0.007759}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000}
};
#endif

#ifdef observer_DC3_6S6P_2600RPM_dccmsg_m50
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 1;
// -50.00, -50.00, -51.00, -51.00
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.823582,   0.007862,   0.000000,   0.025432},
	{ -0.051790,   0.823582,   0.205685,  -0.000000},
	{ -0.041543,  -0.031103,   0.988570,   0.006404},
	{ -0.251551,   0.051004,  -0.063584,   0.988570}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.000142,   0.000025,   0.000000,   0.176393,  -0.007862},
	{ -0.001638,  -0.000189,   0.000000,   0.051790,   0.176211},
	{ -0.015289,  -0.001761,   0.000000,   0.041536,   0.029159},
	{  0.011247,   0.001929,   0.000000,   0.249608,  -0.050939}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000}
};
#endif

#ifdef observer_DC3_6S6P_2600RPM_dccmsg_m50
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 2;
// -100.00, -100.00, -101.00, -101.00
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.666705,   0.007130,   0.000000,   0.023065},
	{ -0.046972,   0.666705,   0.186548,   0.000000},
	{ -0.078091,  -0.127105,   0.976845,   0.005808},
	{ -1.027999,   0.095875,  -0.057669,   0.976845}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.000133,   0.000023,   0.000000,   0.333271,  -0.007130},
	{ -0.001536,  -0.000177,   0.000000,   0.046972,   0.333101},
	{ -0.015229,  -0.001754,   0.000000,   0.078085,   0.125170},
	{  0.011172,   0.001918,   0.000000,   1.026064,  -0.095815}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 0;
// 500.00, 0.00, 0.00, 500.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.015396,   0.000355,   0.094967,  -0.013197},
	{ -0.002059,  -0.000396,   0.010409,  -0.003090},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.338920,  -0.375492},
	{  3.075288,   2.154409},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 1;
// 200.00, 0.00, 0.00, 200.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.033556,   0.002790,   0.216757,  -0.027346},
	{ -0.004491,  -0.000658,   0.023662,  -0.006708},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.320760,  -0.377927},
	{  3.077720,   2.154671},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 2;
// 100.00, 0.00, 0.00, 100.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.056811,   0.008797,   0.383778,  -0.044479},
	{ -0.007631,  -0.000743,   0.041641,  -0.011557},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.297505,  -0.383934},
	{  3.080860,   2.154756},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 3;
// 50.00, 0.00, 0.00, 50.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.090842,   0.022482,   0.639320,  -0.070163},
	{ -0.012329,  -0.000553,   0.068630,  -0.019634},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.263474,  -0.397619},
	{  3.085558,   2.154566},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_20
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 4;
// 20.00, 0.00, 0.00, 20.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.157350,   0.060722,   1.130371,  -0.129508},
	{ -0.022050,  -0.000018,   0.118014,  -0.040458},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.196966,  -0.435859},
	{  3.095279,   2.154030},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 5;
// 10.00, 0.00, 0.00, 10.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.230263,   0.112027,   1.612166,  -0.209229},
	{ -0.033679,  -0.000423,   0.161411,  -0.071892},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.124053,  -0.487164},
	{  3.106908,   2.154436},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 6;
// 5.00, 0.00, 0.00, 5.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.331485,   0.190057,   2.175581,  -0.334920},
	{ -0.051570,  -0.004057,   0.201322,  -0.129851},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.022831,  -0.565194},
	{  3.124798,   2.158070},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_2
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 7;
// 2.00, 0.00, 0.00, 2.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.529041,   0.350749,   3.029498,  -0.596595},
	{ -0.092344,  -0.023437,   0.221894,  -0.287298},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.174725,  -0.725886},
	{  3.165573,   2.177450},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_1
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 8;
// 1.00, 0.00, 0.00, 1.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.747598,   0.534642,   3.754837,  -0.887007},
	{ -0.146751,  -0.064738,   0.174067,  -0.526423},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.393282,  -0.909779},
	{  3.219980,   2.218751},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 9;
// 500.00, 0.00, 0.00, 500.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.006050,   0.002606},
	{ -0.003207,  -0.001713},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.360366,  -0.377743},
	{  3.076436,   2.155726},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 10;
// 200.00, 0.00, 0.00, 200.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.015265,   0.006088},
	{ -0.004334,  -0.001173},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.369581,  -0.381225},
	{  3.077563,   2.155185},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 11;
// 100.00, 0.00, 0.00, 100.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.031402,   0.011423},
	{ -0.014169,  -0.000984},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.385718,  -0.386559},
	{  3.087398,   2.154997},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 12;
// 50.00, 0.00, 0.00, 50.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.238981,   0.017211},
	{ -1.390556,  -0.025285},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.593297,  -0.392348},
	{  4.463785,   2.179297},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 14;
// 10.00, 0.00, 0.00, 10.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.009283,   0.064748},
	{ -0.037289,  -0.000412},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.345033,  -0.439884},
	{  3.110518,   2.154425},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 15;
// 5.00, 0.00, 0.00, 5.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.017384,   0.124133},
	{ -0.062132,  -0.006437},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.336932,  -0.499270},
	{  3.135361,   2.160450},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_2
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 16;
// 2.00, 0.00, 0.00, 2.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.021914,   0.244130},
	{ -0.107181,  -0.034660},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.332402,  -0.619266},
	{  3.180410,   2.188673},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2600RPM_dccmsg_lqr_output_1
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 17;
// 1.00, 0.00, 0.00, 1.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.002237,   0.418964},
	{ -0.216538,  -0.112848},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{ -0.356553,  -0.794101},
	{  3.289767,   2.266861},
	{  0.000000,   0.000000}
};
#endif

#ifndef OBSERVER_MATRIX_PICKED
#error "Observer matrix was not selected"
#endif

#ifndef CONTROLLER_MATRIX_PICKED
#error "Controller matrix was not selected"
#endif
