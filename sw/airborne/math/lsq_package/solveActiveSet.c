
#include "math/lsq_package/common/size_defines.h"
#include "math/lsq_package/common/solveActiveSet.h"
#include <stdbool.h>

void solveActiveSet(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
                    const num_t umin[CA_N_U], const num_t umax[CA_N_U],
                    num_t u_guess[CA_N_U], bool updating, int imax,
                    num_t xs[CA_N_U], int Ws[CA_N_U], const int n_u,
                    const int n_v, int *iterations, int *n_free, activeSetAlgoChoice choice)
{
	num_t u_guess_[CA_N_U];
	if (!u_guess) {
		for (int i=0; i<n_u; i++) {
			u_guess_[i] = 0.5*(umin[i]+umax[i]);
		}
	} else {
		for (int i=0; i<n_u; i++) {
			u_guess_[i] = u_guess[i];
		}
	}
	
	int Ws_[CA_N_U];
	if (!Ws) {
		for (int i=0; i<n_u; i++) {
			Ws_[i] = 0;
		}
	} else {
		for (int i=0; i<n_u; i++) {
			Ws_[i] = Ws[i];
		}
	}

	switch (choice) {
		case QR: 
			//printf("Solving QR\n");
			solveActiveSet_qr(
				 A_col, b, umin, umax, u_guess_, updating, imax, xs, Ws_, n_u, n_v, iterations, n_free);
			break;
		case CHOL: 
			//printf("Solving CHOL\n");
			solveActiveSet_chol(
				 A_col, b, umin, umax, u_guess_, updating, imax, xs, Ws_, n_u, n_v, iterations, n_free);
			break;
		default:
			//printf("Solving PPRZ\n");
			solveActiveSet_pprz(
				 A_col, b, umin, umax, u_guess_, updating, imax, xs, Ws_, n_u, n_v, iterations, n_free);
	}
}