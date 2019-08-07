#define EIGEN_NO_MALLOC 1
#define EIGEN_NO_DEBUG 1
#define EIGEN_NO_STATIC_ASSERT 1
#define eigen_assert(_c) { }
#define assert(ignore) ((void)0)

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#define MAX_N 100

int main()
{
  Matrix3f m = Matrix3f::Random(3,3);
  Vector3f v = Vector3f::Zero();
  Quaternionf q = Quaternionf::Identity();

  while(1) {
    v(0) += m(1,2);
    if (v(0) < -1000 || v(0) > 1000) { v(0) = 0.; }
    m(0,0) = v(0);
    q.normalize();

    Matrix3f m2 = Matrix3f::Random(3,3);
    Matrix3f m3 = m * m2;
    volatile Matrix3f m4 = m3.inverse();
  }

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


}

