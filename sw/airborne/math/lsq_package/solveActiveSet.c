
#include "math/lsq_package/common/size_defines.h"
#include "math/lsq_package/common/solveActiveSet.h"
#include <stdbool.h>

void solveActiveSet(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
                    const num_t umin[CA_N_U], const num_t umax[CA_N_U],
                    const num_t u_guess[CA_N_U], bool updating,
                    num_t xs[CA_N_U], num_t Ws[CA_N_U], const int n_u,
                    const int n_v, num_t *placeholder, num_t *fl, activeSetAlgoChoice choice)
{
	switch (choice) {
		case QR: 
			printf("Solving QR\n");
			solveActiveSet_qr(
				 A_col, b, umin, umax, u_guess, updating, xs, Ws, n_u, n_v, placeholder, fl);
			break;
		case CHOL: 
			printf("Solving CHOL\n");
			solveActiveSet_chol(
				 A_col, b, umin, umax, u_guess, updating, xs, Ws, n_u, n_v, placeholder, fl);
			break;
		default:
			printf("Solving PPRZ\n");
			solveActiveSet_pprz(
				 A_col, b, umin, umax, u_guess, updating, xs, Ws, n_u, n_v, placeholder, fl);
	}
}