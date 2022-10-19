
#ifndef QR_UPDATES_H
#define QR_UPDATES_H

#include "size_defines.h"
// #include "generated/airframe.h"


void qr_shift ( int m, int n, num_t** Q_ptr, num_t** R_ptr, int i, int j);
void givens(num_t a, num_t b, num_t G[4]);
void givens_left_apply(int p, num_t** A, num_t* G, int row1, int row2);
void givens_right_apply(int n, num_t** A, num_t* G, int col1, int col2);

#endif