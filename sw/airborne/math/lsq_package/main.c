
#include "size_defines.h"
#include "solveActiveSet.h"
#include <stdbool.h>

void main (int argc, char** argv)
{
	num_t A_col[CA_N_C*CA_N_U];
	num_t b[CA_N_C];
    num_t umin[CA_N_U];
	num_t umax[CA_N_U];
    num_t u_guess[CA_N_U];
	bool updating = true;
	num_t xs[CA_N_U];
	num_t Ws[CA_N_U];
	int n_u = 6;
    int n_v = 4;
	num_t *placeholder;
	num_t *fl;
	activeSetAlgoChoice choice = (activeSetAlgoChoice) 0;

	solveActiveSet(
		A_col, b, umin, umax, u_guess, updating, xs, Ws, n_u, n_v, placeholder, fl, choice);

};