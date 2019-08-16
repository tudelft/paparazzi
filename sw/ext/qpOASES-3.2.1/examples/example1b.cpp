/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/example1b.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Very simple example for testing qpOASES using the QProblemB class.
 */


#include <qpOASES.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <math.h>
#include <iostream>

using namespace std;
using namespace Eigen;

#define MAX_N 100

/** Example for qpOASES main function using the QProblemB class. */
int main( )
{
	USING_NAMESPACE_QPOASES
	#if 0
	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };

	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };


	/* Setting up QProblemB object. */
	QProblemB example( 2 );  // our class of problem, we don't have any constraints on position or velocity, just the inputs
	// constructor can be initialized by Hessian type, so it can stop checking for positive definiteness

	Options options;
	//options.enableFlippingBounds = BT_FALSE;
	options.printLevel = PL_DEBUG_ITER;
	options.initialStatusBounds = ST_INACTIVE;
	options.numRefinementSteps = 1;
	options.enableCholeskyRefactorisation = 1;
	example.setOptions( options );


	/* Solve first QP. */
	int_t nWSR = 10;
	example.init( H,g,lb,ub, nWSR,0 );

	/* Get and print solution of first QP. */
	real_t xOpt[2];
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );
	

	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new, nWSR,0 );
// 	printf( "\nnWSR = %d\n\n", nWSR );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );
#endif


	float pos0[2] = {10, 2};
	float posf[2] = {0 ,0};
	
	float vel0[2] = {0, 0};
	float velf[2] = {0, 0};
	float dt = 0.1;
	float T = sqrt(pow((pos0[0] - posf[0]),2) + pow((pos0[1] - posf[1]),2)) / 4.0;
	unsigned int N = round(T/dt);
	  
	
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

	// Eigen::Matrix4d D;
	// A << 0, 0, 2, 3,
	// 	0, 0, 4, 5,
	// 	0, 0, 6, 7,
	// 	0, 0, 8, 9;
	// N = 4; //2*N-2
	
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
	
	// to compare with matlab
	cout << "size of H: " << H.rows() << " x " << H.cols() << endl; 
	cout << "H \n" << H << endl;
	cout << "size of f: " << f.rows() << " x " << f.cols() << endl; 
	cout << "f \n" << f << endl;

	float maxbank = 45.0 * 3.142 / 180.0;

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
	/* to check row or column major 
	for (int i=0; i<sizes*sizes; i++) {
		cout << newH[i] << ",";
	}
	*/
	//to check row or column major 
	for (int i=0; i<sizes; i++) {
		cout << newf[i] << ",";
	}
	

	/* Setting up QProblemB object. */
	QProblemB mpctry( 2*N );  // our class of problem, we don't have any constraints on position or velocity, just the inputs
	// constructor can be initialized by Hessian type, so it can stop checking for positive definiteness

	Options optionsmpc;
	//options.enableFlippingBounds = BT_FALSE;
	optionsmpc.printLevel = PL_DEBUG_ITER;
	optionsmpc.initialStatusBounds = ST_INACTIVE;
	optionsmpc.numRefinementSteps = 1;
	// optionsmpc.enableCholeskyRefactorisation = 1;
	mpctry.setOptions( optionsmpc );

	/* Solve first QP. */
	int_t nWSR = 100;
	mpctry.init(newH,newf, lb,ub, nWSR,0);

	/* Get and print solution of first QP. */
	real_t xOpt[sizes];
	mpctry.getPrimalSolution( xOpt );
	cout << "U: " << endl;
	//to check row or column major 
	for (int i=0; i<sizes; i++) {
		cout << xOpt[i] << ",";
	}
	
	printf( "\nfval = %e\n\n", xOpt[0],xOpt[1],mpctry.getObjVal() );

	


	return 0;
}


/*
 *	end of file
 */
