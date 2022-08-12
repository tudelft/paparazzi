#ifndef SOLVEACTIVESET_H
#define SOLVEACTIVESET_H

#include "math/lsq_package/common/size_defines.h"
#include <stdbool.h>

typedef enum {
    PPRZ_NATIVE = 0,
    QR = 1,
    CHOL = 2
} activeSetAlgoChoice;

extern void solveActiveSet(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
                    const num_t umin[CA_N_U], const num_t umax[CA_N_U],
                    const num_t u_guess[CA_N_U], bool updating,
                    num_t xs[CA_N_U], num_t Ws[CA_N_U], const int n_u,
                    const int n_v, num_t *placeholder, num_t *fl, activeSetAlgoChoice choice);
extern void solveActiveSet_pprz(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
                    const num_t umin[CA_N_U], const num_t umax[CA_N_U],
                    const num_t u_guess[CA_N_U], bool updating,
                    num_t xs[CA_N_U], num_t Ws[CA_N_U], const int n_u,
                    const int n_v, num_t *placeholder, num_t *fl);
extern void solveActiveSet_qr(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
                    const num_t umin[CA_N_U], const num_t umax[CA_N_U],
                    const num_t u_guess[CA_N_U], bool updating,
                    num_t xs[CA_N_U], num_t Ws[CA_N_U], const int n_u,
                    const int n_v, num_t *placeholder, num_t *fl);
extern void solveActiveSet_chol(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
                    const num_t umin[CA_N_U], const num_t umax[CA_N_U],
                    const num_t u_guess[CA_N_U], bool updating,
                    num_t xs[CA_N_U], num_t Ws[CA_N_U], const int n_u,
                    const int n_v, num_t *placeholder, num_t *fl);
void print_debug(num_t** A_ptr, num_t** Q_ptr, num_t** R_ptr, const int* n_u, const int* n_c);
void print_final_values(int n_u, int n_v, num_t* u, num_t** B, num_t* v, num_t* umin, num_t* umax);
void print_in_and_outputs(int n_c, int n_free, num_t** A_free_ptr, num_t* d, num_t* p_free);

#endif

