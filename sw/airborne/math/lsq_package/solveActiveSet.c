
#include "math/lsq_package/common/size_defines.h"
#include "math/lsq_package/common/solveActiveSet.h"
#include <stdbool.h>
#include <stdint.h>

void solveActiveSet(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
  const num_t umin[CA_N_U], const num_t umax[CA_N_U], num_t us[CA_N_U],
  int8_t Ws[CA_N_U], bool updating, int imax, const int n_u, const int n_v,
  int *iter, int *n_free, activeSetAlgoChoice choice)
{

	switch (choice) {
		case QR: 
			//printf("Solving QR\n");
			solveActiveSet_qr(
				 A_col, b, umin, umax, us, Ws, updating, imax, n_u, n_v, iter, n_free);
			break;
		case CHOL: 
			//printf("Solving CHOL\n");
			solveActiveSet_chol(
				 A_col, b, umin, umax, us, Ws, updating, imax, n_u, n_v, iter, n_free);
			break;
		default:
			//printf("Solving PPRZ\n");
			solveActiveSet_pprz(
				 A_col, b, umin, umax, us, Ws, updating, imax, n_u, n_v, iter, n_free);
	}
}