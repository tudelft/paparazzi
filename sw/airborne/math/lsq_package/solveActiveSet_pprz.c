/*
 * Copyright (C) Anton Naruta && Daniel Hoppener
 * MAVLab Delft University of Technology
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file wls_alloc.c
 * @brief This is an active set algorithm for WLS control allocation
 *
 * This algorithm will find the optimal inputs to produce the least error wrt
 * the control objective, taking into account the weighting matrices on the
 * control objective and the control effort.
 *
 * The algorithm is described in:
 * Prioritized Control Allocation for Quadrotors Subject to Saturation -
 * E.J.J. Smeur, D.C. Höppener, C. de Wagter. In IMAV 2017
 *
 * written by Anton Naruta && Daniel Hoppener 2016
 * MAVLab Delft University of Technology
 */

#include "math/lsq_package/common/solveActiveSet.h"
#include "math/lsq_package/common/size_defines.h"
#include <stdio.h>
/*#include "std.h"*/
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>

#include <string.h>
#include <math.h>
#include <float.h>
#include "math/lsq_package/lib/qr_solve/qr_solve.h"
#include "math/lsq_package/lib/qr_solve/r8lib_min.h"
#include "math/lsq_package/lib/sparse_math.h"

// provide loop feedback
#define WLS_VERBOSE FALSE

// Problem size needs to be predefined to avoid having to use VLAs
/*
#ifndef CA_N_V
#error CA_N_V needs to be defined!
#endif

#ifndef CA_N_U
#error CA_N_U needs to be defined!
#endif
*/


/**
 * @brief Wrapper for qr solve
 *
 * Possible to use a different solver if needed.
 * Solves a system of the form Ax = b for x.
 *
 * @param m number of rows
 * @param n number of columns
 */
void qr_solve_wrapper_pprz(int m, int n, num_t** A, num_t* b, num_t* x);
void qr_solve_wrapper_pprz(int m, int n, num_t** A, num_t* b, num_t* x) {
  num_t in[m * n];
  // convert A to 1d array
  int k = 0;
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++) {
      in[k++] = A[i][j];
    }
  }
  // use solver
  qr_solve(m, n, in, b, x);
}

/**
 * @brief active set algorithm for control allocation
 *
 * Takes the control objective and max and min inputs from pprz and calculates
 * the inputs that will satisfy most of the control objective, subject to the
 * weighting matrices Wv and Wu
 *
 * @param u The control output vector
 * @param v The control objective
 * @param umin The minimum u vector
 * @param umax The maximum u vector
 * @param B The control effectiveness matrix
 * @param n_u Length of u
 * @param n_v Lenght of v
 * @param u_guess Initial value for u
 * @param W_init Initial working set, if known
 * @param Wv Weighting on different control objectives
 * @param Wu Weighting on different controls
 * @param up Preferred control vector
 * @param gamma_sq Preference of satisfying control objective over desired
 * control vector (sqare root of gamma)
 * @param imax Max number of *iterations
 *
 * @return Number of *iterations, -1 upon failure
 */
/* int wls_alloc(num_t* u, num_t* v, num_t* umin, num_t* umax, num_t** B,
    num_t* u_guess, num_t* W_init, num_t* Wv, num_t* Wu, num_t* up,;
    num_t gamma_sq, int imax) {
      */
// #define TOL 1.5e-6 // now defined in size_defines.h
void solveActiveSet_pprz(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C], const num_t umin[CA_N_U],
                    const num_t umax[CA_N_U], const num_t u_guess[CA_N_U], bool updating, int imax,
                    num_t xs[CA_N_U], int Ws[CA_N_U], const int n_u, const int n_v, int *iter, int *n_free)
{
  (void)(updating);

  if(!imax) imax = 100;

  int n_c = n_u+n_v;

  num_t A[CA_N_C][CA_N_U];
  num_t A_free[CA_N_C][CA_N_U];

  // Create a pointer array to the rows of A_free
  // such that we can pass it to a function
  num_t * A_free_ptr[CA_N_C];
  for(int i = 0; i < n_c; i++)
    A_free_ptr[i] = A_free[i];

  // convert col major input to 2d array
  for(int i = 0; i < n_c; i++) {
    for(int j = 0; j < n_u; j++) {
      A[i][j] = A_col[i + n_c * j];
    }
  }

  // num_t b[CA_N_C];
  num_t d[CA_N_C];

  int free_index[CA_N_U];
  int free_index_lookup[CA_N_U];
  int free_chk = -1;

  num_t p_free[CA_N_U];
  num_t p[CA_N_U];
  num_t u_opt[CA_N_U];
  int infeasible_index[CA_N_U]; /*UNUSED; whatever this means */
  (void)(infeasible_index);
  int n_infeasible = 0;
  num_t lambda[CA_N_U];
  num_t W[CA_N_U];

  // Initialize u and the working set, if provided from input
  num_t u[CA_N_U];
  if (!u_guess) {
    for (int i = 0; i < n_u; i++) {
      u[i] = (umax[i] + umin[i]) * 0.5;
    }
  } else {
    for (int i = 0; i < n_u; i++) {
      u[i] = u_guess[i];
    }
  }

  if (Ws) {
    memcpy(W, Ws, n_u * sizeof(int));
  } else {
    for (int i=0; i<n_u; i++) {
      W[i] = 0.;
    }
  }

  memset(free_index_lookup, -1, n_u * sizeof(int));


  // find free indices
  *n_free = 0;
  for (int i = 0; i < n_u; i++) {
    if (W[i] == 0 || true) { // todo: fix this properly. W should be int
      free_index_lookup[i] = *n_free;
      free_index[(*n_free)++] = i;
    }
  }

  // still need to calculate d = b - A*u
  for (int i = 0; i < n_c; i++) {
    d[i] = b[i];
    for (int j = 0; j < n_u; j++) {
      d[i] -= A[i][j] * u[j];
    }
  }

  // -------------- Start loop ------------
  *iter = 0;
  while ((*iter)++ < imax) {
    // clear p, copy u to u_opt
    memset(p, 0, n_u * sizeof(num_t));
    memcpy(u_opt, u, n_u * sizeof(num_t));

    // Construct a matrix with the free columns of A
    if (free_chk != *n_free) {
      for (int i = 0; i < n_c; i++) {
        for (int j = 0; j < *n_free; j++) {
          A_free[i][j] = A[i][free_index[j]];
        }
      }
      free_chk = *n_free;
    }


    if (*n_free) {
      // Still free variables left, calculate corresponding solution

      // use a solver to find the solution to A_free*p_free = d
      //printf("%d", *n_free);
      qr_solve_wrapper_pprz(n_c, *n_free, A_free_ptr, d, p_free);

      //print results current step
#if WLS_VERBOSE
      print_in_and_outputs(n_c, n_free, A_free_ptr, d, p_free);
#endif

    }

    // Set the nonzero values of p and add to u_opt
    for (int i = 0; i < *n_free; i++) {
      p[free_index[i]] = p_free[i];
      u_opt[free_index[i]] += p_free[i];
    }
    // check limits
    n_infeasible = 0;
    int limits_viol[CA_N_U];
    check_limits_tol(n_u, TOL, u_opt, umin, umax, limits_viol, 0);
    for (int i = 0; i < n_u; i++) {
      if (limits_viol[i] != 0)
        infeasible_index[n_infeasible++] = i;
    }

    // Check feasibility of the solution
    if (n_infeasible == 0) {
      // all variables are within limits
      memcpy(u, u_opt, n_u * sizeof(num_t));
      memset(lambda, 0, n_u * sizeof(num_t));

      // d = d + A_free*p_free; lambda = A*d; --> wrong
      // d = d - A_free*p_free; grad = A^T*d; lambda = W .* grad --> accurate
      for (int i = 0; i < n_c; i++) {
        for (int k = 0; k < *n_free; k++) {
          d[i] -= A_free[i][k] * p_free[k];
        }
        for (int k = 0; k < n_u; k++) {
          lambda[k] += A[i][k] * d[i];
        }
      }
      bool break_flag = true;

      // lambda = lambda x W;
      for (int i = 0; i < n_u; i++) {
        lambda[i] *= W[i];
        // if any lambdas are negative, keep looking for solution
        if (lambda[i] < -FLT_EPSILON) {
          break_flag = false;
          W[i] = 0;
          // add a free index
          if (free_index_lookup[i] < 0) {
            free_index_lookup[i] = *n_free;
            free_index[(*n_free)++] = i;
          }
        }
      }
      if (break_flag) {

#if WLS_VERBOSE
        print_final_values(1, n_u, n_v, u, B, v, umin, umax);
#endif

        // if solution is found, return number of *iterations
        memcpy(xs, u, n_u * sizeof(num_t));
        return;// *iter;
      }
    } else {
      num_t alpha = INFINITY;
      num_t alpha_tmp;
      int id_alpha = 0;

      // find the lowest distance from the limit among the free variables
      for (int i = 0; i < *n_free; i++) {
        int id = free_index[i];
        if(fabs(p[id]) > FLT_EPSILON) {
          alpha_tmp = (p[id] < 0) ? (umin[id] - u[id]) / p[id]
            : (umax[id] - u[id]) / p[id];
        } else {
          alpha_tmp = INFINITY;
        }
        if (alpha_tmp < alpha) {
          alpha = alpha_tmp;
          id_alpha = id;
        }
      }

      // update input u = u + alpha*p
      for (int i = 0; i < n_u; i++) {
        // TODO: implement in other solvers as well!!
        num_t incr = alpha * p[i];
        if (i == id_alpha) {
          u[i] = (incr > 0) ? umax[i] : umin[i];
        } else {
          u[i] += incr;
        }
      }
      // update d = d-alpha*A*p_free
      for (int i = 0; i < n_c; i++) {
        for (int k = 0; k < *n_free; k++) {
          d[i] -= A_free[i][k] * alpha * p_free[k];
        }
      }
      // get rid of a free index
      W[id_alpha] = (p[id_alpha] > 0) ? 1.0 : -1.0;

      free_index[free_index_lookup[id_alpha]] = free_index[--(*n_free)];
      free_index_lookup[free_index[free_index_lookup[id_alpha]]] =
        free_index_lookup[id_alpha];
      free_index_lookup[id_alpha] = -1;
    }
  }
  // solution failed, return negative one to indicate failure
  
  memcpy(xs, u, n_u * sizeof(num_t));
  return; // -1;
}

#if WLS_VERBOSE
void print_in_and_outputs(int n_c, int *n_free, num_t** A_free_ptr, num_t* d, num_t* p_free) {

  printf("n_c = %d *n_free = %d\n", n_c, *n_free);

  printf("A_free =\n");
  for(int i = 0; i < n_c; i++) {
    for (int j = 0; j < *n_free; j++) {
      printf("%f ", A_free_ptr[i][j]);
    }
    printf("\n");
  }

  printf("d = ");
  for (int j = 0; j < n_c; j++) {
    printf("%f ", d[j]);
  }

  printf("\noutput = ");
  for (int j = 0; j < *n_free; j++) {
    printf("%f ", p_free[j]);
  }
  printf("\n\n");
}

void print_final_values(int n_u, int n_v, num_t* u, num_t** B, num_t* v, num_t* umin, num_t* umax) {
  printf("n_u = %d n_v = %d\n", n_u, n_v);

  printf("B =\n");
  for(int i = 0; i < n_v; i++) {
    for (int j = 0; j < n_u; j++) {
      printf("%f ", B[i][j]);
    }
    printf("\n");
  }

  printf("v = ");
  for (int j = 0; j < n_v; j++) {
    printf("%f ", v[j]);
  }

  printf("\nu = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", u[j]);
  }
  printf("\n");

  printf("\numin = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", umin[j]);
  }
  printf("\n");

  printf("\numax = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", umax[j]);
  }
  printf("\n\n");

}
#endif
