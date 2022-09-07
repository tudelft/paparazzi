
#ifndef SPARSE_MATH_H
#define SPARSE_MATH_H

#include "math/lsq_package/common/size_defines.h"
#include "generated/airframe.h"

#include <stdbool.h>

void backward_tri_solve(int n, num_t** A, const num_t* b, num_t* x);
void tri_mult(int n, int m, num_t** A, const num_t* x, num_t* b);
void block_diag_self_mult(int n, int m, num_t** A, num_t** H, int s_dense, int* pos);
void block_diag_mult(int n, int m, int p, num_t** A, num_t** B,
    num_t** Y, int s_dense, bool vertical, int* pos);
int check_limits_tol(int n, num_t tol, num_t* x, const num_t* xmin, const num_t* xmax, int* output, int* perm);
#endif