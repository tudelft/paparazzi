
#include "size_defines.h"
#include "solveActiveSet.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

int8_t solveActiveSet(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
  const num_t umin[CA_N_U], const num_t umax[CA_N_U], num_t us[CA_N_U],
  int8_t Ws[CA_N_U], bool updating, int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[RECORD_COST_N], activeSetAlgoChoice choice)
{
	int8_t res;

	switch (choice) {
		case QR:
			// printf("Solving QR\n");
			res = solveActiveSet_qr(
				 A_col, b, umin, umax, us, Ws, updating, imax, n_u, n_v, iter, n_free, costs);
			break;
		case CHOL:
			// printf("Solving CHOL\n");
			res = solveActiveSet_chol(
				 A_col, b, umin, umax, us, Ws, updating, imax, n_u, n_v, iter, n_free, costs);
			break;
		default:
			// printf("Solving PPRZ\n");
			res = solveActiveSet_pprz(
				 A_col, b, umin, umax, us, Ws, updating, imax, n_u, n_v, iter, n_free, costs);
	}

	return res;
}

#if defined(RECORD_COST) || defined(TRUNCATE_COST)
num_t calc_cost(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C], num_t us[CA_N_U],
  const int n_u, const int n_v)
{
	// checking cost in n_v*n_u+n_u time
	num_t cost = 0.;
	for (int i=0; i<(n_u+n_v); i++) {
		num_t i_cost = -b[i];
		if (i < n_v) {
			// dense part
			for (int j=0; j<n_u; j++)
				i_cost += A_col[i + j*(n_u+n_v)]*us[j];
		} else {
			// sparse part
			i_cost += A_col[i + (i-n_v)*(n_u+n_v)]*us[i-n_v]; 
		}
		cost += i_cost*i_cost;
	}
	return cost;
}
#endif