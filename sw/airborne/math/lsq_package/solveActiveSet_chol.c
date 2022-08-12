#include "math/lsq_package/common/solveActiveSet.h"
#include "math/lsq_package/lib/chol_math.h"
#include "math/lsq_package/common/size_defines.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

//#define PRINT_COND_EST

void solveActiveSet_chol(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C], const num_t umin[CA_N_U],
                    const num_t umax[CA_N_U], const num_t u_guess[CA_N_U], bool updating,
                    num_t xs[CA_N_U], num_t Ws[CA_N_U], const int n_u, const int n_v, num_t *placeholder, num_t *fl)
  {

  // allocate variables, use defaults where parameters are set to 0
  // if(!gamma_sq) gamma_sq = 100000;
  int imax = 0;
  if(!imax) imax = 100;

  int n_c = n_u + n_v;

  //num_t xs[CA_N_U]; memcpy(xs, u_guess, sizeof(num_t)*n_u);
  memcpy(xs, u_guess, sizeof(num_t)*n_u);
  int iter = 0;
  int n_free = 0;

  num_t A[CA_N_C][CA_N_U];
  num_t A_free[CA_N_C][CA_N_U];
  num_t H[CA_N_U][CA_N_U];
  num_t L[CA_N_U][CA_N_U];

  // Create a pointer array to the rows of A
  // such that we can pass it to a function
  num_t * A_ptr[CA_N_C];
  num_t * H_ptr[CA_N_U];
  num_t * L_ptr[CA_N_U];
  for(int i = 0; i < n_c; i++) {
    A_ptr[i] = A[i];
    if (i < n_u) { H_ptr[i] = H[i]; L_ptr[i] = L[i]; }
  }

  int gamma[CA_N_U]; memset(gamma, 0, sizeof(int)*n_u);
  for (int i = 0; i < n_u; i++) {
    if (xs[i] <= umin[i]+TOL) {
      gamma[i] = +1;
    } else if (xs[i] >= umax[i]-TOL) {
      gamma[i] = -1;
    }
  }

  int free_index_lookup[CA_N_U]; memset(free_index_lookup, -1, sizeof(int)*n_u);
  int permutation[CA_N_U]; memset(permutation, 0, sizeof(int)*n_u);
  for (int i = 0; i < n_u; i++) {
    if (gamma[i] == 0) {
      free_index_lookup[i] = n_free;
      permutation[n_free] = i;
      n_free++;
    }
  }

  // convert col major input to 2d array, using the permutaiton just found
  for(int i = 0; i < n_c; i++) {
    for(int j = 0; j < n_u; j++) {
      A[i][j] = A_col[i + n_c * permutation[j]];
    }
  }

  num_t min_diag2 = 1e100;
  num_t A2[CA_N_V][CA_N_V];
  num_t * A2_ptr[CA_N_V];
  for (int i=0; i<n_v; i++)
    A2_ptr[i] = A2[i];

  for (int i=0; i<n_v; i++) {
    for (int j=i; j<n_v; j++) {
      A2_ptr[i][j] = 0;
      for (int k=0; k<n_u; k++) {
        A2_ptr[i][j] += A_ptr[i][k]*A_ptr[j][k];
      }
      if (i != j)
        A2_ptr[j][i] = A2_ptr[i][j];
    }
  }

  int ip;
  for (int i=0; i<n_u; i++) {
    ip = permutation[i];
    min_diag2 = (min_diag2 > A_ptr[ip+n_v][ip]*A_ptr[ip+n_v][ip])
      ? A_ptr[ip+n_v][ip]*A_ptr[ip+n_v][ip] : min_diag2;
  }

  num_t cond_est;
  num_t max_sig;
  cond_estimator(n_v, A2_ptr, min_diag2, &cond_est, &max_sig);

  #ifdef PRINT_COND_EST
  printf("cond(A^T*A) <= %f\n", cond_est);
  #endif

  // initial factorisation
  block_diag_self_mult(n_c, n_u, A_ptr, H_ptr, n_v, permutation);
  // debug output
  #ifdef DEBUG
  printf("H:\n");
  for (int i=0; i<n_u; i++) {
    for (int j=0; j<n_u; j++) {
      printf("%f ", H_ptr[i][j]);
    }
    printf("\n");
  }
  printf("\n");
  #endif

  num_t inv_diag[CA_N_U];// memset(inv_diag, 0.0, sizeof(num_t)*n_u);
  pprz_cholesky_float(L_ptr, H_ptr, inv_diag, n_u);



  int free_chk = 0;
  num_t q[CA_N_U];
  num_t z[CA_N_U];

  // -------------- Start loop ------------
  while (iter++ < imax) {
    num_t beta[CA_N_U];
    for (int i=0; i<n_free; i++) {
      beta[i] = 0;
      for (int j=n_free; j<n_u; j++) {
        beta[i] -= H[permutation[i]][permutation[j]] * ((num_t) xs[permutation[j]]);
      }
      for (int j=0; j<n_c; j++) {
        beta[i] += ((num_t)A[j][permutation[i]]) * ((num_t)b[j]); // TODO optimise with block_diag_mult?
      }
      #ifdef DEBUG
      printf("%f\n", beta[i]);
      #endif
    }

    #ifdef DEBUG
    printf("L:\n");
    for (int i=0; i<n_u; i++) {
      for (int j=0; j<n_u; j++) {
        printf("%f ", L_ptr[i][j]);
      }
      printf("\n");
    }
    printf("\n");
    #endif
    cholesky_solve(L_ptr, inv_diag, n_free, beta, q);

    for (int i = 0; i < n_free; i++) {
      z[permutation[i]] = q[i];
    }
    for (int i = n_free; i < n_u; i++) {
      z[permutation[i]] = xs[permutation[i]];
    }

    /*
    num_t u_bound_perm[CA_N_U];
    for (int k=0; k < n_u - n_free; k++) {
      if (gamma[permutation[k+n_free]] < 0)
        u_bound_perm[k] = umax[permutation[k+n_free]];
      else if (gamma[permutation[k+n_free]] > 0) 
        u_bound_perm[k] = umin[permutation[k+n_free]];
      else
        printf("Gamma out of bounds for bounded variables");
    }
    */

    int n_violated = 0;
    for (int i = 0; i < n_free; i++) {
      if ((umin[permutation[i]] - TOL > z[permutation[i]]) ||
        (z[permutation[i]] > TOL + umax[permutation[i]])) {
          n_violated++;
        }
    }

    if (!n_violated) {
      // is this the most efficient location TODO
      for (int i = 0; i < n_free; i++) {
        xs[permutation[i]] = z[permutation[i]];
      }

      if (n_free == n_u) {
        // no active constraints, we are optinal and feasible
        break;
      } else {
        // active constraints, check for optimality

        num_t lambda_perm[CA_N_U];
        int f_free = 0;
        num_t maxlam = -1e10; // TODO

        num_t r[CA_N_C];
        for (int i=0; i<n_c; i++) { // TODO: optimise with block_diag_mult
          r[i] = -b[i];
          for (int j=0; j<n_u; j++) {
            r[i] += A[i][j]*z[j];
          }
        }

        for (int i=n_free; i<n_u; i++) {
          lambda_perm[i] = 0;
          for (int j=0; j < n_c; j++) {
            lambda_perm[i] -= A[j][permutation[i]]*r[j]; // also optimise this
          }
          lambda_perm[i] *= gamma[permutation[i]];
          if (lambda_perm[i] > maxlam) {
            maxlam = lambda_perm[i];
            f_free = i-n_free;
          }
        }

        if (maxlam <= 0) {
          break;
        }

        // update chol
        choladd(L_ptr, n_free, inv_diag, H_ptr, permutation, n_free+f_free);

        gamma[permutation[n_free+f_free]] = 0;
        int last_val = permutation[n_free+f_free];
        for (int i = f_free-1; i >= 0; i--) {
          permutation[n_free+i+1] = permutation[n_free+i];
        }
        permutation[n_free] = last_val;

        n_free++;

      }
    } else {

      num_t a = 1e10;
      int i_a = 0;
      int f_bound = 0;
      int i_s = 0;
      num_t temp;
      int temp_s;
      for (int f=0; f < n_free; f++) {
        int i = permutation[f];
        if (z[i] < umin[i]-TOL) {
          temp = (xs[i] - umin[i]) / (xs[i] - z[i]);
          temp_s = +1;
        } else if (z[i] > umax[i]+TOL) {
          temp = (umax[i] - xs[i]) / (z[i] - xs[i]);
          temp_s = -1;
        } else {
          continue;
        }
        if (temp < a) {
          a = temp;
          i_a = i;
          f_bound = f;
          i_s = temp_s;
        }
      }

      // update xs
      for (int i=0; i<n_u; i++) {
        xs[i] += a * (z[i] - xs[i]);
      }

      // update chol
      choldel(L_ptr, inv_diag, f_bound, n_u);

      gamma[i_a] = i_s;
      int first_val = permutation[f_bound];
      for (int i = 0; i < n_free-f_bound-1; i++) {
        permutation[f_bound+i] = permutation[f_bound+i+1];
      }
      permutation[n_free-1] = first_val;

      n_free--;

    }

  }
  return; // -1;
}

#if WLS_VERBOSE
void print_in_and_outputs(int n_c, int n_free, num_t** A_free_ptr, num_t* d, num_t* p_free) {

  printf("n_c = %d n_free = %d\n", n_c, n_free);

  printf("A_free =\n");
  for(int i = 0; i < n_c; i++) {
    for (int j = 0; j < n_free; j++) {
      printf("%f ", A_free_ptr[i][j]);
    }
    printf("\n");
  }

  printf("d = ");
  for (int j = 0; j < n_c; j++) {
    printf("%f ", d[j]);
  }

  printf("\noutput = ");
  for (int j = 0; j < n_free; j++) {
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

#ifdef debug_qr
void print_debug(num_t** A_ptr, num_t** Q_ptr, num_t** R_ptr, const int* n_u, const int* n_c) {
  printf("A_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_u; j++) {
      printf("%f ", A_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("Q_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_c; j++) {
      printf("%f ", Q_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("R_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_u; j++) {
      printf("%f ", R_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");
}
#endif

#endif