/*
 * Copyright (C) mavlab
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
/**
 * @file "modules/qpoas/qpoas.c"
 * @author mavlab
 * opti
 */

#include "modules/qpoas/qpoas.h"

// Eigen headers
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wshadow"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <math.h>
#include <unistd.h>
#include "state.h"
#pragma GCC diagnostic pop

#include "qpOASES.hpp"

using namespace Eigen;
USING_NAMESPACE_QPOASES



/** A structure for keeping internal timer data. */
typedef struct acado_timer_ {
	struct timespec tic;
	struct timespec toc;
} acado_timer;

struct ctrl_module_demo_struct {
    // RC Inputs
    struct Int32Eulers rc_sp;
    // Output command
    struct Int32Eulers cmd;
} ctrl;


/* read current time */
void profile_tic( acado_timer* t )
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}


/* return time passed since last call to tic on this timer */
real_t profile_toc( acado_timer* t )
{
	struct timespec temp;
    
	clock_gettime(CLOCK_MONOTONIC, &t->toc);	
    
	if ((t->toc.tv_nsec - t->tic.tv_nsec) < 0)
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
		temp.tv_nsec = 1000000000+t->toc.tv_nsec - t->tic.tv_nsec;
	}
	else
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_nsec = t->toc.tv_nsec - t->tic.tv_nsec;
	}
	
	return (real_t)temp.tv_sec + (real_t)temp.tv_nsec / 1e9;
}

float theta_cmd[100];
float phi_cmd[100];

void qp_init(void) {
	// theta_cmd = {0};
	// phi_cmd = {0};
}


void replan(void) {

}

acado_timer t2;
unsigned int N = 0;
bool lock_optimal = 0;
void optimal_enter(void) {

	acado_timer t;
	profile_tic(&t);

	float pos0[2] = {-2.7, 2.15};
	float posf[2] = {0.0, -3.2};
	
	float vel0[2] = {0.0, 0.0};
	float velf[2] = {0.0, -2.0};
	float dt = 0.1;
	float T = 2.5;
	N = round(T/dt);
	
	Eigen::Matrix<double, 4, 4> A;
	A << 0.9512, 0, 0, 0,
		0.09754, 1, 0, 0,
		0, 0, 0.9512, 0,
		0, 0, 0.09754, 1;

	Eigen::Matrix<double, 4, 2> B;
	B << 0.9569, 0,
		0.04824, 0,
		0, 0.9569,
		0, 0.04824;

	Eigen::Matrix<double, 4, 4> P;
	P << 1,0,0,0,
		0,10,0,0,
		0,0,1,0,
		0,0,0,10;
	
	// Eigen::Matrix<double, 4, Dynamic> R;
	Eigen::MatrixXd oldR(4, 2 * MAX_N);
	oldR.resize(4, 2*N);
	
	oldR.block(0, 2*N-2, 4, 2) =  B;
	Eigen::Matrix<double, 4, 4> AN = A;

	for(int i=1; i<N; i++) {
		oldR.block(0, 2*N-2*(i+1), 4, 2) =  A * oldR.block(0, 2*N-2*i, 4, 2);
		AN = A * AN; 
	}
	Eigen::MatrixXd R = oldR.block(0,0,4,2*N);

	Eigen::MatrixXd old_H = 2 * (R.transpose() * P * R);
	
	Eigen::MatrixXd eye(2* MAX_N, 2 * MAX_N); 
	eye.resize(2*N, 2*N);
	eye.setIdentity();
	Eigen::MatrixXd H = 0.5 * (old_H + old_H.transpose() + eye);

	Eigen::Matrix<double, 4, 1> x0; Eigen::Matrix<double, 4, 1> xd;
	x0 << vel0[0], pos0[0], vel0[1], pos0[1];
	xd << velf[0], posf[0], velf[1], posf[1];
	
	Eigen::MatrixXd f;
	f = (2 * ((AN * x0)- xd)).transpose() * P * R;
	
	float maxbank = 25.0 * 3.142 / 180.0;

	int sizes = 2 * N;

	real_t ub[sizes];
	real_t lb[sizes];
	
	for (int i=0; i<sizes; i++) {
		ub[i] = maxbank;
		lb[i] = -maxbank;
	}
	
	real_t newH[sizes * sizes];
	real_t newf[sizes];

  	Eigen::Map<MatrixXd>(newH, sizes, sizes) =   H.transpose();
	Eigen::Map<MatrixXd>(newf, 1, sizes) = f;
	
	// /* Setting up QProblemB object. */
	QProblemB mpctry( 2*N );  
	// our class of problem, we don't have any constraints on position or velocity, just the inputs

	Options optionsmpc;
	//options.enableFlippingBounds = BT_FALSE;
	optionsmpc.printLevel = PL_LOW;
	optionsmpc.initialStatusBounds = ST_INACTIVE;
	optionsmpc.numRefinementSteps = 1;
	// optionsmpc.enableCholeskyRefactorisation = 1;
	mpctry.setOptions( optionsmpc );

	/* Solve first QP. */
	int nWSR = 100;
	mpctry.init(newH, newf, lb, ub, nWSR, 0);

	/* Get and print solution of first QP. */
	real_t xOpt[sizes];

	if (mpctry.getPrimalSolution(xOpt) == SUCCESSFUL_RETURN) {
		for (int i=0; i<N; i++) {
			phi_cmd[i] = (float) xOpt[2*i];
			theta_cmd[i] = (float) xOpt[2*i + 1];
			printf("theta: %f \t phi: %f\n", 180/3.142 * (theta_cmd[i]), 180/3.142 * phi_cmd[i]);
			lock_optimal = 1;
			profile_tic(&t2);
		}
		printf("\nfval = %e \n\n", mpctry.getObjVal());
	}
	
	/* Read the elapsed time. */
	real_t te = profile_toc( &t );
	printf("qp solved in: %.3g us\n\n", 1e6 * te);

}

float arbiter_roll = 0.0;
float arbiter_pitch = 0.0;
void periodic_10Hz_demo(void) {
	if (lock_optimal == 1) {
		static int i = 0;
		if (i < N) {
			arbiter_roll  = phi_cmd[i];
			arbiter_pitch = theta_cmd[i];
			i++;
		}
		if (i >= N) {
			i = 0;
			arbiter_roll = 0;
			arbiter_pitch = 0;
			real_t te = profile_toc(&t2);
			printf("-------------------Finished maneuver in %f seconds-----------\n", te);
			lock_optimal = 0;
		}
	}
}

void dronerace_get_cmd(float* roll, float* pitch) {
	*roll = arbiter_roll;
	*pitch = arbiter_pitch;
}

