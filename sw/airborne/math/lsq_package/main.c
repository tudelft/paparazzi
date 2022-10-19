
#include "size_defines.h"
#include "solveActiveSet.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int main (int argc, char** argv)
{
	num_t A_col[CA_N_C*CA_N_U];
	num_t b[CA_N_C];
    num_t umin[CA_N_U];
	num_t umax[CA_N_U];
	bool updating = true;
	num_t xs[CA_N_U];
	int8_t Ws[CA_N_U];
	int n_u = 6;
    int n_v = 4;
	int iter;
	int n_free;
	activeSetAlgoChoice choice;
	choice = (argc == 2) ? (activeSetAlgoChoice) atoi(argv[1]) : 0;


	solveActiveSet(
		A_col, b, umin, umax, xs, Ws, updating, 100, n_u, n_v, &iter, &n_free, choice);

	return 0;

};