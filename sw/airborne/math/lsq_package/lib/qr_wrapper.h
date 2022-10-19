
#ifndef QR_WRAPPER_H
#define QR_WRAPPER_H

#include "size_defines.h"
// #include "generated/airframe.h"


void qr_wrapper(int m, int n, int perm[], num_t** A, num_t** Q, num_t** R);
int dorgqr ( int m, int n, const num_t A[], num_t Q[], num_t tau[]);

#endif