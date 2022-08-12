#include "size_defines.h"
#include "wls_alloc.h"

#define N 5

void
main(int argc, char** argv) {
	num_t JG[CA_N_V*CA_N_U] = {-17.5000000000000000F,22.7454999999999998F,1.8263000000000000F,-0.8750000000000000F,-17.5000000000000000F,22.6764000000000010F,1.8262000000000000F,0.8750000000000000F,-17.5000000000000000F,15.6416000000000004F,1.7858000000000001F,-0.8750000000000000F,-17.5000000000000000F,-15.8587000000000007F,1.7967000000000000F,0.8750000000000000F,-17.5000000000000000F,-22.8399000000000001F,1.7736000000000001F,-0.8750000000000000F,-17.5000000000000000F,-22.8994000000000000F,1.8874000000000000F,0.8750000000000000F,-17.5000000000000000F,22.7361000000000004F,-1.6019000000000001F,-0.8750000000000000F,-17.5000000000000000F,22.6751000000000005F,-1.7385999999999999F,0.8750000000000000F,-17.5000000000000000F,15.6456000000000000F,-1.7277000000000000F,-0.8750000000000000F,-17.5000000000000000F,-15.7886000000000006F,-1.7313000000000001F,0.8750000000000000F,-17.5000000000000000F,-22.7852999999999994F,-1.6680999999999999F,-0.8750000000000000F,-17.5000000000000000F,-22.8384000000000000F,-1.6655000000000000F,0.8750000000000000F};
	num_t Wv[CA_N_V] = {1, 1, 1, 1};
	num_t Wu[CA_N_U] = {1, 1, 1,   1, 1, 1,   1, 1, 1,   1, 1, 1};
	num_t dv[CA_N_V] = {-100, 1, 1, 1};
	num_t up[CA_N_U] = {0, 0, 0,   0, 0, 0,   0, 0, 0,   0, 0, 0};
	num_t A[CA_N_C*CA_N_U];
	num_t b[CA_N_C];
	num_t theta = sqrt(1e-5);
	num_t cond_bound = 1e4;
	int n_v = 4;
	int n_u = 12;
	int n_c = n_v+n_u;

	setup_wls(4, 12, JG, Wv, Wu, up, dv, theta, cond_bound, A, b);

	for (int i=0; i<n_c; i++) {
		for (int j=0; j<n_u; j++)
			printf("%f, ", A[i+n_c*j]);
		printf("\n");
	}
	printf("\n\n");

	for (int i=0; i<n_c; i++) {
		printf("%f, ", b[i]);
	}
	printf("\n\n");

	num_t min_diag2 = 1e100;
	num_t A2[CA_N_V][CA_N_V];
	num_t * A2_ptr[CA_N_V];
	for (int i=0; i<n_v; i++)
		A2_ptr[i] = A2[i];

	for (int i=0; i<n_v; i++) {
		for (int j=i; j<n_v; j++) {
		A2_ptr[i][j] = 0;
		for (int k=0; k<n_u; k++) {
			A2_ptr[i][j] += A[i+n_c*k]*A[j+n_c*k];
		}
		if (i != j)
			A2_ptr[j][i] = A2_ptr[i][j];
		}
	}
	for (int i=0; i<n_u; i++)
		min_diag2 = (min_diag2 > A[i+n_v+n_c*i]*A[i+n_v+n_c*i]) ? A[i+n_v+n_v*i]*A[i+n_v+n_v*i] : min_diag2;

	num_t cond_est;
	num_t max_sig;
	cond_estimator(n_v, A2_ptr, min_diag2, &cond_est, &max_sig);

	printf("%f\n", cond_est);


	num_t umin[CA_N_U] = {0, 0, 0,   0, 0, 0,   0, 0, 0,   0, 0, 0};
	num_t umax[CA_N_U] = {1, 1, 1,   1, 1, 1,   1, 1, 1,   1, 1, 1};
	num_t u_guess[CA_N_U] = {.5, .5, .5,   .5, .5, .5,   .5, .5, .5,   .5, .5, .5};
	num_t us[CA_N_U];
	num_t Ws[CA_N_U]; memset(Ws, 0.F, sizeof(num_t)*CA_N_U);
	num_t placeholder = 0;
	num_t fl = 0;
	solveActiveSet(A, b, umin, umax, u_guess, true, us, Ws, n_u, n_v, &placeholder, &fl);

	for (int i=0; i<n_u; i++) {
		printf("%f, ", us[i]);
	}
	printf("\n");
}