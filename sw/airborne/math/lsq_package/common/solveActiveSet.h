#ifndef SOLVEACTIVESET_H
#define SOLVEACTIVESET_H

#include "size_defines.h"
//#include "generated/airframe.h"

#include <stdbool.h>
#include <stdint.h>

#define RECORD_COST
//#define RECORD_LG_RATIOS
#define RECORD_COST_N 15

typedef enum {
    PPRZ_NATIVE = 0,
    QR = 1,
    CHOL = 2
} activeSetAlgoChoice;

extern int8_t solveActiveSet(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
  const num_t umin[CA_N_U], const num_t umax[CA_N_U], num_t us[CA_N_U],
  int8_t Ws[CA_N_U], bool updating, int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[RECORD_COST_N], activeSetAlgoChoice choice);
int8_t solveActiveSet_pprz(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
  const num_t umin[CA_N_U], const num_t umax[CA_N_U], num_t us[CA_N_U],
  int8_t Ws[CA_N_U], bool updating, int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[RECORD_COST_N]);
int8_t solveActiveSet_qr(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
  const num_t umin[CA_N_U], const num_t umax[CA_N_U], num_t us[CA_N_U],
  int8_t Ws[CA_N_U], bool updating, int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[RECORD_COST_N]);
int8_t solveActiveSet_chol(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
  const num_t umin[CA_N_U], const num_t umax[CA_N_U], num_t us[CA_N_U],
  int8_t Ws[CA_N_U], bool updating, int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[RECORD_COST_N]);
void print_debug(num_t** A_ptr, num_t** Q_ptr, num_t** R_ptr, const int* n_u, const int* n_c);
void print_final_values(int n_u, int n_v, num_t* u, num_t** B, num_t* v, num_t* umin, num_t* umax);
void print_in_and_outputs(int n_c, int *n_free, num_t** A_free_ptr, num_t* d, num_t* p_free);


#if defined(RECORD_COST) || defined(TRUNCATE_COST)
num_t calc_cost(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C], num_t us[CA_N_U],
  const int n_u, const int n_v);
#endif

#endif

