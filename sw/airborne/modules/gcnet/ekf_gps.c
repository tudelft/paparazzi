#include "ekf_gps.h"

float ekf_X[EKF_NUM_STATES];
float ekf_P[EKF_NUM_STATES][EKF_NUM_STATES];
float ekf_Q[EKF_NUM_INPUTS][EKF_NUM_INPUTS];
float ekf_R[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];

float ekf_H[EKF_NUM_OUTPUTS][EKF_NUM_STATES] = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0}};


void ekf_set_diag(float **a, float *b, int n)
{
	int i, j;
	for (i = 0 ; i < n; i++) {
	  for (j = 0 ; j < n; j++) {
	    if (i == j) {
		a[i][j] = b[i];
	    } else {
		a[i][j] = 0.0;
	    }
	  }
	}
}

void ekf_init(void)
{
	float X0[EKF_NUM_STATES] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	float Pdiag[EKF_NUM_STATES] = {1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.};
	float Qdiag[EKF_NUM_INPUTS] = {5., 5., 5., 0.1, 0.1, 0.1};
	float Rdiag[EKF_NUM_OUTPUTS] = {0.0001, 0.0001, 0.0001, 0.0001};

	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_INPUTS);
	MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);

	ekf_set_diag(ekf_P_, Pdiag, EKF_NUM_STATES);
	ekf_set_diag(ekf_Q_, Qdiag, EKF_NUM_INPUTS);
	ekf_set_diag(ekf_R_, Rdiag, EKF_NUM_OUTPUTS);
	float_vect_copy(ekf_X, X0, EKF_NUM_STATES);
}

void ekf_f(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES])
{
	float x0=cos(X[8]);
	float x1=U[0] - X[9];
	float x2=cos(X[7]);
	float x3=x1*x2;
	float x4=U[2] - X[11];
	float x5=sin(X[6]);
	float x6=sin(X[8]);
	float x7=x5*x6;
	float x8=sin(X[7]);
	float x9=cos(X[6]);
	float x10=x0*x9;
	float x11=U[1] - X[10];
	float x12=x6*x9;
	float x13=x0*x5;
	float x14=tan(X[7]);
	float x15=U[4] - X[13];
	float x16=x15*x5;
	float x17=U[5] - X[14];
	float x18=x17*x9;
	float x19=1.0/x2;
	out[0]=X[3];
	out[1]=X[4];
	out[2]=X[5];
	out[3]=x0*x3 + x11*(-x12 + x13*x8) + x4*(x10*x8 + x7);
	out[4]=x11*(x10 + x7*x8) + x3*x6 + x4*(x12*x8 - x13);
	out[5]=-x1*x8 + x11*x2*x5 + x2*x4*x9 + 9.8100000000000005;
	out[6]=U[3] - X[12] + x14*x16 + x14*x18;
	out[7]=x15*x9 - x17*x5;
	out[8]=x16*x19 + x18*x19;
	out[9]=0;
	out[10]=0;
	out[11]=0;
	out[12]=0;
	out[13]=0;
	out[14]=0;
}

void ekf_F(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_STATES])
{
	float x0=U[1] - X[10];
	float x1=sin(X[6]);
	float x2=sin(X[8]);
	float x3=x1*x2;
	float x4=sin(X[7]);
	float x5=cos(X[6]);
	float x6=cos(X[8]);
	float x7=x5*x6;
	float x8=x4*x7;
	float x9=x3 + x8;
	float x10=U[2] - X[11];
	float x11=x2*x5;
	float x12=x1*x6;
	float x13=x12*x4;
	float x14=x11 - x13;
	float x15=U[0] - X[9];
	float x16=x15*x4;
	float x17=cos(X[7]);
	float x18=x0*x17;
	float x19=x10*x17;
	float x20=x17*x2;
	float x21=x11*x4;
	float x22=x12 - x21;
	float x23=-x3*x4 - x7;
	float x24=x17*x6;
	float x25=x17*x5;
	float x26=x1*x17;
	float x27=x4*x5;
	float x28=U[4] - X[13];
	float x29=tan(X[7]);
	float x30=x29*x5;
	float x31=U[5] - X[14];
	float x32=x1*x29;
	float x33=pow(x29, 2) + 1;
	float x34=x1*x28;
	float x35=1.0/x17;
	float x36=x35*x5;
	float x37=x1*x35;
	float x38=pow(x17, -2);
	out[0][0]=0;
	out[0][1]=0;
	out[0][2]=0;
	out[0][3]=1;
	out[0][4]=0;
	out[0][5]=0;
	out[0][6]=0;
	out[0][7]=0;
	out[0][8]=0;
	out[0][9]=0;
	out[0][10]=0;
	out[0][11]=0;
	out[0][12]=0;
	out[0][13]=0;
	out[0][14]=0;
	out[1][0]=0;
	out[1][1]=0;
	out[1][2]=0;
	out[1][3]=0;
	out[1][4]=1;
	out[1][5]=0;
	out[1][6]=0;
	out[1][7]=0;
	out[1][8]=0;
	out[1][9]=0;
	out[1][10]=0;
	out[1][11]=0;
	out[1][12]=0;
	out[1][13]=0;
	out[1][14]=0;
	out[2][0]=0;
	out[2][1]=0;
	out[2][2]=0;
	out[2][3]=0;
	out[2][4]=0;
	out[2][5]=1;
	out[2][6]=0;
	out[2][7]=0;
	out[2][8]=0;
	out[2][9]=0;
	out[2][10]=0;
	out[2][11]=0;
	out[2][12]=0;
	out[2][13]=0;
	out[2][14]=0;
	out[3][0]=0;
	out[3][1]=0;
	out[3][2]=0;
	out[3][3]=0;
	out[3][4]=0;
	out[3][5]=0;
	out[3][6]=x0*x9 + x10*x14;
	out[3][7]=x12*x18 - x16*x6 + x19*x7;
	out[3][8]=x0*x23 + x10*x22 - x15*x20;
	out[3][9]=-x24;
	out[3][10]=x14;
	out[3][11]=-x3 - x8;
	out[3][12]=0;
	out[3][13]=0;
	out[3][14]=0;
	out[4][0]=0;
	out[4][1]=0;
	out[4][2]=0;
	out[4][3]=0;
	out[4][4]=0;
	out[4][5]=0;
	out[4][6]=x0*(-x12 + x21) + x10*x23;
	out[4][7]=x11*x19 - x16*x2 + x18*x3;
	out[4][8]=x0*(-x11 + x13) + x10*x9 + x15*x24;
	out[4][9]=-x20;
	out[4][10]=x23;
	out[4][11]=x22;
	out[4][12]=0;
	out[4][13]=0;
	out[4][14]=0;
	out[5][0]=0;
	out[5][1]=0;
	out[5][2]=0;
	out[5][3]=0;
	out[5][4]=0;
	out[5][5]=0;
	out[5][6]=x0*x25 - x10*x26;
	out[5][7]=-x0*x1*x4 - x10*x27 + x17*(-U[0] + X[9]);
	out[5][8]=0;
	out[5][9]=x4;
	out[5][10]=-x26;
	out[5][11]=-x25;
	out[5][12]=0;
	out[5][13]=0;
	out[5][14]=0;
	out[6][0]=0;
	out[6][1]=0;
	out[6][2]=0;
	out[6][3]=0;
	out[6][4]=0;
	out[6][5]=0;
	out[6][6]=x28*x30 - x31*x32;
	out[6][7]=x31*x33*x5 + x33*x34;
	out[6][8]=0;
	out[6][9]=0;
	out[6][10]=0;
	out[6][11]=0;
	out[6][12]=-1;
	out[6][13]=-x32;
	out[6][14]=-x30;
	out[7][0]=0;
	out[7][1]=0;
	out[7][2]=0;
	out[7][3]=0;
	out[7][4]=0;
	out[7][5]=0;
	out[7][6]=-x34 + x5*(-U[5] + X[14]);
	out[7][7]=0;
	out[7][8]=0;
	out[7][9]=0;
	out[7][10]=0;
	out[7][11]=0;
	out[7][12]=0;
	out[7][13]=-x5;
	out[7][14]=x1;
	out[8][0]=0;
	out[8][1]=0;
	out[8][2]=0;
	out[8][3]=0;
	out[8][4]=0;
	out[8][5]=0;
	out[8][6]=x28*x36 - x31*x37;
	out[8][7]=x27*x31*x38 + x34*x38*x4;
	out[8][8]=0;
	out[8][9]=0;
	out[8][10]=0;
	out[8][11]=0;
	out[8][12]=0;
	out[8][13]=-x37;
	out[8][14]=-x36;
	out[9][0]=0;
	out[9][1]=0;
	out[9][2]=0;
	out[9][3]=0;
	out[9][4]=0;
	out[9][5]=0;
	out[9][6]=0;
	out[9][7]=0;
	out[9][8]=0;
	out[9][9]=0;
	out[9][10]=0;
	out[9][11]=0;
	out[9][12]=0;
	out[9][13]=0;
	out[9][14]=0;
	out[10][0]=0;
	out[10][1]=0;
	out[10][2]=0;
	out[10][3]=0;
	out[10][4]=0;
	out[10][5]=0;
	out[10][6]=0;
	out[10][7]=0;
	out[10][8]=0;
	out[10][9]=0;
	out[10][10]=0;
	out[10][11]=0;
	out[10][12]=0;
	out[10][13]=0;
	out[10][14]=0;
	out[11][0]=0;
	out[11][1]=0;
	out[11][2]=0;
	out[11][3]=0;
	out[11][4]=0;
	out[11][5]=0;
	out[11][6]=0;
	out[11][7]=0;
	out[11][8]=0;
	out[11][9]=0;
	out[11][10]=0;
	out[11][11]=0;
	out[11][12]=0;
	out[11][13]=0;
	out[11][14]=0;
	out[12][0]=0;
	out[12][1]=0;
	out[12][2]=0;
	out[12][3]=0;
	out[12][4]=0;
	out[12][5]=0;
	out[12][6]=0;
	out[12][7]=0;
	out[12][8]=0;
	out[12][9]=0;
	out[12][10]=0;
	out[12][11]=0;
	out[12][12]=0;
	out[12][13]=0;
	out[12][14]=0;
	out[13][0]=0;
	out[13][1]=0;
	out[13][2]=0;
	out[13][3]=0;
	out[13][4]=0;
	out[13][5]=0;
	out[13][6]=0;
	out[13][7]=0;
	out[13][8]=0;
	out[13][9]=0;
	out[13][10]=0;
	out[13][11]=0;
	out[13][12]=0;
	out[13][13]=0;
	out[13][14]=0;
	out[14][0]=0;
	out[14][1]=0;
	out[14][2]=0;
	out[14][3]=0;
	out[14][4]=0;
	out[14][5]=0;
	out[14][6]=0;
	out[14][7]=0;
	out[14][8]=0;
	out[14][9]=0;
	out[14][10]=0;
	out[14][11]=0;
	out[14][12]=0;
	out[14][13]=0;
	out[14][14]=0;
}

void ekf_L(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_INPUTS])
{
	float x0=cos(X[7]);
	float x1=cos(X[8]);
	float x2=sin(X[8]);
	float x3=cos(X[6]);
	float x4=x2*x3;
	float x5=sin(X[7]);
	float x6=sin(X[6]);
	float x7=x1*x6;
	float x8=x2*x6;
	float x9=x1*x3;
	float x10=tan(X[7]);
	float x11=1.0/x0;
	out[0][0]=0;
	out[0][1]=0;
	out[0][2]=0;
	out[0][3]=0;
	out[0][4]=0;
	out[0][5]=0;
	out[1][0]=0;
	out[1][1]=0;
	out[1][2]=0;
	out[1][3]=0;
	out[1][4]=0;
	out[1][5]=0;
	out[2][0]=0;
	out[2][1]=0;
	out[2][2]=0;
	out[2][3]=0;
	out[2][4]=0;
	out[2][5]=0;
	out[3][0]=-x0*x1;
	out[3][1]=x4 - x5*x7;
	out[3][2]=-x5*x9 - x8;
	out[3][3]=0;
	out[3][4]=0;
	out[3][5]=0;
	out[4][0]=-x0*x2;
	out[4][1]=-x5*x8 - x9;
	out[4][2]=-x4*x5 + x7;
	out[4][3]=0;
	out[4][4]=0;
	out[4][5]=0;
	out[5][0]=x5;
	out[5][1]=-x0*x6;
	out[5][2]=-x0*x3;
	out[5][3]=0;
	out[5][4]=0;
	out[5][5]=0;
	out[6][0]=0;
	out[6][1]=0;
	out[6][2]=0;
	out[6][3]=-1;
	out[6][4]=-x10*x6;
	out[6][5]=-x10*x3;
	out[7][0]=0;
	out[7][1]=0;
	out[7][2]=0;
	out[7][3]=0;
	out[7][4]=-x3;
	out[7][5]=x6;
	out[8][0]=0;
	out[8][1]=0;
	out[8][2]=0;
	out[8][3]=0;
	out[8][4]=-x11*x6;
	out[8][5]=-x11*x3;
	out[9][0]=0;
	out[9][1]=0;
	out[9][2]=0;
	out[9][3]=0;
	out[9][4]=0;
	out[9][5]=0;
	out[10][0]=0;
	out[10][1]=0;
	out[10][2]=0;
	out[10][3]=0;
	out[10][4]=0;
	out[10][5]=0;
	out[11][0]=0;
	out[11][1]=0;
	out[11][2]=0;
	out[11][3]=0;
	out[11][4]=0;
	out[11][5]=0;
	out[12][0]=0;
	out[12][1]=0;
	out[12][2]=0;
	out[12][3]=0;
	out[12][4]=0;
	out[12][5]=0;
	out[13][0]=0;
	out[13][1]=0;
	out[13][2]=0;
	out[13][3]=0;
	out[13][4]=0;
	out[13][5]=0;
	out[14][0]=0;
	out[14][1]=0;
	out[14][2]=0;
	out[14][3]=0;
	out[14][4]=0;
	out[14][5]=0;
}



void ekf_f_rk4(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], const float dt, float out[EKF_NUM_STATES])
{
	float k1[EKF_NUM_STATES];
	float k2[EKF_NUM_STATES];
	float k3[EKF_NUM_STATES];
	float k4[EKF_NUM_STATES];

	float Xtmp[EKF_NUM_STATES];

	// k1   = f(X,U)
	ekf_f(X,U,k1);
	
	// Xtmp = X+dt*k1/2
	float_vect_smul(Xtmp, k1, dt/2, EKF_NUM_STATES);
	float_vect_add(Xtmp, X, EKF_NUM_STATES);
	
	// k2   = f(Xtmp,U)
	ekf_f(Xtmp,U,k2);
	
	// Xtmp = X+dt*k2/2
	float_vect_smul(Xtmp, k2, dt/2, EKF_NUM_STATES);
	float_vect_add(Xtmp, X, EKF_NUM_STATES);

	// k3   = f(Xtmp,U)
	ekf_f(Xtmp,U,k3);

	// Xtmp = X+dt*k3
	float_vect_smul(Xtmp, k3, dt, EKF_NUM_STATES);
	float_vect_add(Xtmp, X, EKF_NUM_STATES);

	// k4   = f(Xtmp,U)
	ekf_f(Xtmp,U,k4);	
	
	// out = k2+k3
	float_vect_sum(out, k2, k3, EKF_NUM_STATES);
	// out *= 2
	float_vect_scale(out, 2, EKF_NUM_STATES);
	// out += k1
	float_vect_add(out, k1, EKF_NUM_STATES);
	// out += k4
	float_vect_add(out, k4, EKF_NUM_STATES);
	// out *= dt/6
	float_vect_scale(out, dt/6, EKF_NUM_STATES);
	// out += X
	float_vect_add(out, X, EKF_NUM_STATES);	
}


void ekf_step(const float U[EKF_NUM_INPUTS], const float Z[EKF_NUM_OUTPUTS], const float dt)
{
	// [1] Predicted (a priori) state estimate:
	float Xkk_1[EKF_NUM_STATES];
	ekf_f_rk4(ekf_X, U, dt, Xkk_1);


	// [2] Get matrices
	float F[EKF_NUM_STATES][EKF_NUM_STATES];
	float L[EKF_NUM_STATES][EKF_NUM_INPUTS];
	ekf_F(ekf_X, U, F);
	ekf_L(ekf_X, U, L);


	// [3] Continuous to discrete
	// Fd = eye(N) + F*dt + F*F*dt**2/2 = I + [I+F*dt/2]*F*dt
	// Ld = L*dt+F*L*dt**2/2            = [I+F*dt/2]*L*dt
	float Fd[EKF_NUM_STATES][EKF_NUM_STATES];
	float Ld[EKF_NUM_STATES][EKF_NUM_INPUTS];
	float tmp[EKF_NUM_STATES][EKF_NUM_STATES];
	
	MAKE_MATRIX_PTR(F_, F, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(L_, L, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Fd_, Fd, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Ld_, Ld, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);
	
	// tmp = I+F*dt/2
	float_mat_diagonal_scal(tmp_, 1, EKF_NUM_STATES);
	float_mat_sum_scaled(tmp_, F_, dt/2, EKF_NUM_STATES, EKF_NUM_STATES);

	// Ld = tmp*L*dt
	float_mat_mul(Ld_, tmp_, L_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_INPUTS);
	float_mat_scale(Ld_, dt, EKF_NUM_STATES, EKF_NUM_INPUTS);
	
	// Fd = tmp*F*dt
	float_mat_mul(Fd_, tmp_, F_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
	float_mat_scale(Fd_, dt, EKF_NUM_STATES, EKF_NUM_STATES);

	// Fd += I
	int i;
	for (i = 0; i < EKF_NUM_STATES; i++) {
		Fd[i][i] += 1;
	}
	
	
	// [4] Predicted covariance estimate:
	// Pkk_1 = Fd*P*Fd.T + Ld*Q*Ld.T
	float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
	float LdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
	float QLdT[EKF_NUM_INPUTS][EKF_NUM_STATES];

	MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(LdT_, LdT, EKF_NUM_INPUTS);
	MAKE_MATRIX_PTR(QLdT_, QLdT, EKF_NUM_INPUTS);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// tmp = P*Fd
	float_mat_mul(tmp_, ekf_P_, Fd_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// Pkk_1 = Fd*tmp
	float_mat_mul(Pkk_1_, Fd_, tmp_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// LdT = Ld.T
	float_mat_transpose(LdT_, Ld_, EKF_NUM_STATES, EKF_NUM_INPUTS);
	
	// QLdT = Q*LdT
	float_mat_mul(QLdT_, ekf_Q_, LdT_, EKF_NUM_INPUTS, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// tmp = Ld*QLdT
	float_mat_mul(tmp_, Ld_, QLdT_, EKF_NUM_STATES, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// Pkk_1 += tmp
	float_mat_sum_scaled(Pkk_1_, tmp_, 1, EKF_NUM_STATES, EKF_NUM_STATES);


	// [5] Measurement residual:
	// yk = Z - H*Xkk_1
	float yk[EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(ekf_H_, ekf_H, EKF_NUM_OUTPUTS);

	float_mat_vect_mul(yk, ekf_H_, Xkk_1, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_vect_scale(yk, -1, EKF_NUM_OUTPUTS);
	float_vect_add(yk, Z, EKF_NUM_OUTPUTS);
	

	// [6] Residual covariance:
	// Sk = H*Pkk_1*H.T + R
	float Sk[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float PHT[EKF_NUM_STATES][EKF_NUM_OUTPUTS];
	
	MAKE_MATRIX_PTR(Sk_, Sk, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(PHT_, PHT, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);
	
	// PHT = Pkk_1*H.T
	float_mat_transpose(PHT_, ekf_H_, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_mat_mul_copy(PHT_, Pkk_1_, PHT_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

	// Sk = H*PHT
	float_mat_mul(Sk_, ekf_H_, PHT_, EKF_NUM_OUTPUTS, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	
	// Sk += R
	float_mat_sum_scaled(Sk_, ekf_R_, 1, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [7] Near-optimal Kalman gain:
	// K = Pkk_1*H.T*inv(Sk)
	float Sk_inv[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float K[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(Sk_inv_, Sk_inv, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(K_, K, EKF_NUM_STATES);
	
	// Sk_inv = inv(Sk)
	float_mat_invert(Sk_inv_, Sk_, EKF_NUM_OUTPUTS);

	// K = PHT*Sk_inv
	float_mat_mul(K_, PHT_, Sk_inv_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [8] Updated state estimate
	// Xkk = Xkk_1 + K*yk
	float_mat_vect_mul(ekf_X, K_, yk, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	float_vect_add(ekf_X, Xkk_1, EKF_NUM_STATES);

	
	// [9] Updated covariance estimate:
    	// Pkk = (I - K*H)*Pkk_1

	// tmp = K*H
	float_mat_mul(tmp_, K_, ekf_H_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_STATES);

	// tmp *= -1
	float_mat_scale(tmp_, -1, EKF_NUM_STATES, EKF_NUM_STATES);

	// tmp += I
	for (i = 0; i < EKF_NUM_STATES; i++) {
		tmp_[i][i] += 1;
	}
	// P = tmp*Pkk_1
	float_mat_mul(ekf_P_, tmp_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
}

void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt) {
	// [1] Predicted (a priori) state estimate:
	float Xkk_1[EKF_NUM_STATES];
	ekf_f_rk4(ekf_X, U, dt, Xkk_1);


	// [2] Get matrices
	float F[EKF_NUM_STATES][EKF_NUM_STATES];
	float L[EKF_NUM_STATES][EKF_NUM_INPUTS];
	ekf_F(ekf_X, U, F);
	ekf_L(ekf_X, U, L);


	// [3] Continuous to discrete
	// Fd = eye(N) + F*dt + F*F*dt**2/2 = I + [I+F*dt/2]*F*dt
	// Ld = L*dt+F*L*dt**2/2            = [I+F*dt/2]*L*dt
	float Fd[EKF_NUM_STATES][EKF_NUM_STATES];
	float Ld[EKF_NUM_STATES][EKF_NUM_INPUTS];
	float tmp[EKF_NUM_STATES][EKF_NUM_STATES];
	
	MAKE_MATRIX_PTR(F_, F, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(L_, L, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Fd_, Fd, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(Ld_, Ld, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);
	
	// tmp = I+F*dt/2
	float_mat_diagonal_scal(tmp_, 1, EKF_NUM_STATES);
	float_mat_sum_scaled(tmp_, F_, dt/2, EKF_NUM_STATES, EKF_NUM_STATES);

	// Ld = tmp*L*dt
	float_mat_mul(Ld_, tmp_, L_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_INPUTS);
	float_mat_scale(Ld_, dt, EKF_NUM_STATES, EKF_NUM_INPUTS);
	
	// Fd = tmp*F*dt
	float_mat_mul(Fd_, tmp_, F_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
	float_mat_scale(Fd_, dt, EKF_NUM_STATES, EKF_NUM_STATES);

	// Fd += I
	int i;
	for (i = 0; i < EKF_NUM_STATES; i++) {
		Fd[i][i] += 1;
	}
	
	
	// [4] Predicted covariance estimate:
	// Pkk_1 = Fd*P*Fd.T + Ld*Q*Ld.T
	float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
	float LdT[EKF_NUM_INPUTS][EKF_NUM_STATES];
	float QLdT[EKF_NUM_INPUTS][EKF_NUM_STATES];

	MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_Q_, ekf_Q, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(LdT_, LdT, EKF_NUM_INPUTS);
	MAKE_MATRIX_PTR(QLdT_, QLdT, EKF_NUM_INPUTS);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// tmp = P*Fd
	float_mat_mul(tmp_, ekf_P_, Fd_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// Fd = Fd.T
	float_mat_transpose_square(Fd_, EKF_NUM_STATES);

	// Pkk_1 = Fd*tmp
	float_mat_mul(Pkk_1_, Fd_, tmp_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);

	// LdT = Ld.T
	float_mat_transpose(LdT_, Ld_, EKF_NUM_STATES, EKF_NUM_INPUTS);
	
	// QLdT = Q*LdT
	float_mat_mul(QLdT_, ekf_Q_, LdT_, EKF_NUM_INPUTS, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// tmp = Ld*QLdT
	float_mat_mul(tmp_, Ld_, QLdT_, EKF_NUM_STATES, EKF_NUM_INPUTS, EKF_NUM_STATES);

	// Pkk_1 += tmp
	float_mat_sum_scaled(Pkk_1_, tmp_, 1, EKF_NUM_STATES, EKF_NUM_STATES);

	// X = Xkk_1
	float_vect_copy(ekf_X, Xkk_1, EKF_NUM_STATES);

	// P = Pkk_1
	float_mat_copy(ekf_P_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES);	
}

void ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS], const float dt) {
	// Xkk_1 = X
	float Xkk_1[EKF_NUM_STATES];
	float_vect_copy(Xkk_1, ekf_X, EKF_NUM_STATES);

	// Pkk_1 = P
	float Pkk_1[EKF_NUM_STATES][EKF_NUM_STATES];
	MAKE_MATRIX_PTR(Pkk_1_, Pkk_1, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_P_, ekf_P, EKF_NUM_STATES);
	float_mat_copy(Pkk_1_, ekf_P_, EKF_NUM_STATES, EKF_NUM_STATES);	

	// [5] Measurement residual:
	// yk = Z - H*Xkk_1
	float yk[EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(ekf_H_, ekf_H, EKF_NUM_OUTPUTS);

	float_mat_vect_mul(yk, ekf_H_, Xkk_1, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_vect_scale(yk, -1, EKF_NUM_OUTPUTS);
	float_vect_add(yk, Z, EKF_NUM_OUTPUTS);
	

	// [6] Residual covariance:
	// Sk = H*Pkk_1*H.T + R
	float Sk[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float PHT[EKF_NUM_STATES][EKF_NUM_OUTPUTS];
	
	MAKE_MATRIX_PTR(Sk_, Sk, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(PHT_, PHT, EKF_NUM_STATES);
	MAKE_MATRIX_PTR(ekf_R_, ekf_R, EKF_NUM_OUTPUTS);
	
	// PHT = Pkk_1*H.T
	float_mat_transpose(PHT_, ekf_H_, EKF_NUM_OUTPUTS, EKF_NUM_STATES);
	float_mat_mul_copy(PHT_, Pkk_1_, PHT_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_OUTPUTS);

	// Sk = H*PHT
	float_mat_mul(Sk_, ekf_H_, PHT_, EKF_NUM_OUTPUTS, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	
	// Sk += R
	float_mat_sum_scaled(Sk_, ekf_R_, 1, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [7] Near-optimal Kalman gain:
	// K = Pkk_1*H.T*inv(Sk)
	float Sk_inv[EKF_NUM_OUTPUTS][EKF_NUM_OUTPUTS];
	float K[EKF_NUM_STATES][EKF_NUM_OUTPUTS];

	MAKE_MATRIX_PTR(Sk_inv_, Sk_inv, EKF_NUM_OUTPUTS);
	MAKE_MATRIX_PTR(K_, K, EKF_NUM_STATES);
	
	// Sk_inv = inv(Sk)
	float_mat_invert(Sk_inv_, Sk_, EKF_NUM_OUTPUTS);

	// K = PHT*Sk_inv
	float_mat_mul(K_, PHT_, Sk_inv_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_OUTPUTS);


	// [8] Updated state estimate
	// Xkk = Xkk_1 + K*yk
	float_mat_vect_mul(ekf_X, K_, yk, EKF_NUM_STATES, EKF_NUM_OUTPUTS);
	float_vect_add(ekf_X, Xkk_1, EKF_NUM_STATES);

	
	// [9] Updated covariance estimate:
    	// Pkk = (I - K*H)*Pkk_1
	float tmp[EKF_NUM_STATES][EKF_NUM_STATES];
	MAKE_MATRIX_PTR(tmp_, tmp, EKF_NUM_STATES);

	// tmp = K*H
	float_mat_mul(tmp_, K_, ekf_H_, EKF_NUM_STATES, EKF_NUM_OUTPUTS, EKF_NUM_STATES);

	// tmp *= -1
	float_mat_scale(tmp_, -1, EKF_NUM_STATES, EKF_NUM_STATES);

	// tmp += I
	int i;
	for (i = 0; i < EKF_NUM_STATES; i++) {
		tmp_[i][i] += 1;
	}
	// P = tmp*Pkk_1
	float_mat_mul(ekf_P_, tmp_, Pkk_1_, EKF_NUM_STATES, EKF_NUM_STATES, EKF_NUM_STATES);
}




