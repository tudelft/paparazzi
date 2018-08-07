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

#ifdef observer_DC2_6S1P_cmsg_m50
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 0;
// -50.00, -50.00, -51.00, -51.00
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.831053,  -0.006039,   0.000000,   0.189114},
	{  0.053578,   0.831053,   0.801209,   0.000000},
	{  0.010034,  -0.006620,   0.981099,  -0.012646},
	{ -0.028047,  -0.004791,   0.025583,   0.981099}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.000683,  -0.000055,   0.000000,   0.168756,   0.006039},
	{ -0.000786,   0.001738,   0.000000,  -0.053578,   0.168139},
	{ -0.001928,   0.004163,   0.000000,  -0.010021,   0.004685},
	{  0.006898,  -0.000498,   0.000000,   0.026112,   0.004765}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000}
};
#endif

#ifdef observer_DC2_6S1P_m50
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 1;
// -50.00, -50.00, -51.00, -51.00
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.834891,  -0.005481,  -0.000000,   0.253820},
	{  0.117308,   0.834891,   1.232365,  -0.000000},
	{  0.013552,  -0.003589,   0.977260,  -0.024161},
	{ -0.017428,  -0.003074,   0.026611,   0.977260}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.000944,  -0.000092,   0.000000,   0.164853,   0.005481},
	{ -0.000629,   0.003834,   0.000000,  -0.117308,   0.163865},
	{ -0.001066,   0.005962,   0.000000,  -0.013528,   0.001658},
	{  0.007101,  -0.000612,   0.000000,   0.015496,   0.003047}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 0;
// 500.00, 0.00, 0.00, 500.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.003080,   0.002349,   0.067637,  -0.079211},
	{ -0.000623,  -0.002166,  -0.088741,   0.023550},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.293140,   0.036240},
	{  0.131853,   0.484953},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 1;
// 200.00, 0.00, 0.00, 200.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.007639,   0.005774,   0.163736,  -0.188496},
	{ -0.001562,  -0.005367,  -0.217029,   0.056125},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.297699,   0.032815},
	{  0.132792,   0.488155},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 2;
// 100.00, 0.00, 0.00, 100.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.015072,   0.011269,   0.311500,  -0.350258},
	{ -0.003152,  -0.010590,  -0.419555,   0.104227},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.305132,   0.027319},
	{  0.134382,   0.493377},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 3;
// 50.00, 0.00, 0.00, 50.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.029348,   0.021641,   0.569791,  -0.619038},
	{ -0.006463,  -0.020662,  -0.790096,   0.182561},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.319408,   0.016947},
	{  0.137693,   0.503449},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_20
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 4;
// 20.00, 0.00, 0.00, 20.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.068248,   0.049268,   1.151776,  -1.186491},
	{ -0.017247,  -0.048523,  -1.715120,   0.331013},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.358308,  -0.010680},
	{  0.148477,   0.531310},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 5;
// 10.00, 0.00, 0.00, 10.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.123898,   0.087510,   1.786300,  -1.787245},
	{ -0.036368,  -0.089509,  -2.902952,   0.442218},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.413958,  -0.048922},
	{  0.167598,   0.572296},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 6;
// 5.00, 0.00, 0.00, 5.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.215134,   0.146855,   2.541696,  -2.532940},
	{ -0.073734,  -0.159485,  -4.642659,   0.488519},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.505194,  -0.108266},
	{  0.204964,   0.642272},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_2
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 7;
// 2.00, 0.00, 0.00, 2.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.416867,   0.265281,   3.615447,  -3.754682},
	{ -0.167367,  -0.324776,  -7.936346,   0.345989},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.706927,  -0.226693},
	{  0.298597,   0.807563},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_1
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 8;
// 1.00, 0.00, 0.00, 1.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.658413,   0.391007,   4.433542,  -4.890072},
	{ -0.282856,  -0.535657, -11.232757,   0.043097},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.948473,  -0.352418},
	{  0.414085,   1.018445},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_lqr_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 9;
// 500.00, 0.00, 0.00, 500.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.003347,   0.001777,   0.075266,  -0.127314},
	{ -0.000842,  -0.003052,  -0.142141,   0.058234},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.279271,   0.030409},
	{  0.046087,   0.332804},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_lqr_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 10;
// 200.00, 0.00, 0.00, 200.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.008303,   0.004340,   0.180115,  -0.297148},
	{ -0.002074,  -0.007550,  -0.344284,   0.137981},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.284227,   0.027846},
	{  0.047319,   0.337302},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_lqr_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 11;
// 100.00, 0.00, 0.00, 100.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.016390,   0.008407,   0.336902,  -0.537979},
	{ -0.004087,  -0.014852,  -0.656424,   0.254455},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.292314,   0.023779},
	{  0.049332,   0.344605},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_lqr_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 12;
// 50.00, 0.00, 0.00, 50.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.031933,   0.015976,   0.599357,  -0.915798},
	{ -0.008058,  -0.028821,  -1.209366,   0.442499},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.307857,   0.016210},
	{  0.053303,   0.358573},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_lqr_20
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 13;
// 20.00, 0.00, 0.00, 20.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.074304,   0.035577,   1.145178,  -1.649699},
	{ -0.019697,  -0.066772,  -2.514764,   0.806646},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.350228,  -0.003392},
	{  0.064941,   0.396524},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_lqr_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 14;
// 10.00, 0.00, 0.00, 10.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.134867,   0.061547,   1.681768,  -2.370297},
	{ -0.037664,  -0.121121,  -4.082428,   1.125912},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.410791,  -0.029361},
	{  0.082908,   0.450873},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_lqr_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 15;
// 5.00, 0.00, 0.00, 5.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.233944,   0.099872,   2.254732,  -3.227472},
	{ -0.068146,  -0.210794,  -6.231245,   1.412453},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.509868,  -0.067687},
	{  0.113391,   0.540546},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 16;
// 500.00, 0.00, 0.00, 500.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.000144,  -0.000266},
	{ -0.001313,  -0.002266},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.290204,   0.038855},
	{  0.132542,   0.485054},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 17;
// 200.00, 0.00, 0.00, 200.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.000374,  -0.000926},
	{ -0.003406,  -0.005676},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.290434,   0.039514},
	{  0.134635,   0.488464},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 18;
// 100.00, 0.00, 0.00, 100.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.000707,  -0.002373},
	{ -0.005960,  -0.011301},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.290768,   0.040961},
	{  0.137189,   0.494089},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 19;
// 50.00, 0.00, 0.00, 50.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.002012,  -0.007249},
	{ -0.013688,  -0.022779},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.292072,   0.045837},
	{  0.144917,   0.505566},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_20
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 20;
// 20.00, 0.00, 0.00, 20.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.008743,  -0.026086},
	{ -0.039029,  -0.055142},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.298804,   0.064675},
	{  0.170259,   0.537929},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 21;
// 10.00, 0.00, 0.00, 10.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.025835,  -0.057960},
	{ -0.076729,  -0.100331},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.315896,   0.096548},
	{  0.207959,   0.583118},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 22;
// 5.00, 0.00, 0.00, 5.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.075855,  -0.119723},
	{ -0.160497,  -0.171157},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.365916,   0.158311},
	{  0.291727,   0.653945},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_2
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 23;
// 2.00, 0.00, 0.00, 2.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.192864,  -0.240672},
	{ -0.250519,  -0.317813},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.482924,   0.279261},
	{  0.381748,   0.800601},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_output_1
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 24;
// 1.00, 0.00, 0.00, 1.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.439584,  -0.416935},
	{ -0.470936,  -0.472553},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.729644,   0.455524},
	{  0.602166,   0.955340},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 25;
// 500.00, 0.00, 0.00, 500.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.001285,   0.010065},
	{ -0.002641,  -0.001806},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.302052,   0.071216},
	{  0.222665,   0.964224},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 26;
// 200.00, 0.00, 0.00, 200.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.003362,   0.024038},
	{ -0.006340,  -0.004247},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.304130,   0.057243},
	{  0.226364,   0.966664},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 27;
// 100.00, 0.00, 0.00, 100.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.007094,   0.045004},
	{ -0.011990,  -0.007769},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.307861,   0.036278},
	{  0.232014,   0.970186},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 28;
// 50.00, 0.00, 0.00, 50.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.015084,   0.080883},
	{ -0.022063,  -0.013434},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.315851,   0.000398},
	{  0.242087,   0.975851},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_20
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 29;
// 20.00, 0.00, 0.00, 20.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.039725,   0.162359},
	{ -0.047795,  -0.024853},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.340492,  -0.081078},
	{  0.267818,   0.987271},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 30;
// 10.00, 0.00, 0.00, 10.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.077678,   0.258653},
	{ -0.085245,  -0.036449},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.378446,  -0.177371},
	{  0.305269,   0.998866},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 31;
// 5.00, 0.00, 0.00, 5.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.140187,   0.393247},
	{ -0.152997,  -0.050776},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.440954,  -0.311965},
	{  0.373021,   1.013194},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_2
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 32;
// 2.00, 0.00, 0.00, 2.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.265948,   0.648028},
	{ -0.329851,  -0.077143},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.566715,  -0.566746},
	{  0.549875,   1.039561},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC2_6S1P_cmsg_lqr_cyl_1
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 33;
// 1.00, 0.00, 0.00, 1.00
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.389175,   0.920334},
	{ -0.572562,  -0.108983},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.689942,  -0.839052},
	{  0.792586,   1.071401},
	{  0.000000,   0.000000}
};
#endif

#ifndef OBSERVER_MATRIX_PICKED
#error "Observer matrix was not selected"
#endif

#ifndef CONTROLLER_MATRIX_PICKED
#error "Controller matrix was not selected"
#endif
