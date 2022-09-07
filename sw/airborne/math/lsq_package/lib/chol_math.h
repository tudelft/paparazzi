#ifndef CHOL_MATH_H
#define CHOL_MATH_H

#include "math/lsq_package/common/size_defines.h"
#include "generated/airframe.h"

#include <stdbool.h>

int dorgqr ( int m, int n, int k, const num_t A[], num_t Q[], int lda, num_t tau[], num_t work[], int lwork );

/** Cholesky decomposition
 *
 * http://rosettacode.org/wiki/Cholesky_decomposition#C
 *
 * @param out pointer to the output array [n x n]
 * @param in pointer to the input array [n x n]
 * @param n dimension of the matrix
 */
void pprz_cholesky_float(num_t **out, num_t **in, num_t* inv_diag, int n);
void cholesky_solve(num_t **L, num_t* inv_diag, int n, num_t *b, num_t *x);
void cholup(num_t** L, int i1, int i2);
void choldel(num_t** L, num_t* inv_diag, int f, int n);
void choladd(num_t** L, int nf, num_t* inv_diag, num_t** H, int* perm, int f);

#endif