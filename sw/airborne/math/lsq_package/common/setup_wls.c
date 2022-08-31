
#include "math/lsq_package/common/setup_wls.h"
#include <math.h>

#include <stdio.h>

void setup_wls(
    int n_v, int n_u, num_t JG[CA_N_V*CA_N_U], num_t Wv[CA_N_V],
    num_t Wu[CA_N_U], num_t up[CA_N_U], num_t dv[CA_N_V], num_t theta,
    num_t cond_bound, num_t A[CA_N_C*CA_N_U], num_t b[CA_N_C]) {

    int n_c = n_v + n_u;
    num_t gamma;

    if (cond_bound <= 0) {
      gamma = theta;
    } else {
      // do condition number limiting

      num_t min_diag = 1e100;
      for (int i=0; i<n_u; i++)
          min_diag = (min_diag > Wu[i]) ? Wu[i] : min_diag;

      min_diag = (min_diag > 1e-6) ? min_diag : 1e-6;
      num_t min_diag_inv = 1/min_diag;

      for (int i=0; i<n_u; i++)
          Wu[i] *= min_diag_inv;

      num_t A2[CA_N_V][CA_N_V];
      num_t * A2_ptr[CA_N_V];
      for (int i=0; i<n_v; i++)
          A2_ptr[i] = A2[i];

      for (int i=0; i<n_v; i++) {
          for (int j=0; j<n_v; j++) {
              A2_ptr[i][j] = 0;
              for (int k=0; k<n_u; k++)
                  A2_ptr[i][j] += JG[i+k*n_v]*JG[j+k*n_v];

              A2_ptr[i][j] *= Wv[i]*Wv[i];
              if (i != j)
                  A2_ptr[j][i] = A2_ptr[i][j];
          }
      }

      num_t max_sig;
      gamma_estimator(n_v, A2_ptr, cond_bound, &gamma, &max_sig);
      #ifdef DOUBLE
      gamma = (gamma > sqrt(max_sig)*theta) ? gamma : sqrt(max_sig)*theta;
      #else
      gamma = (gamma > sqrtf(max_sig)*theta) ? gamma : sqrtf(max_sig)*theta;
      #endif
      //printf("%f | %f\n", gamma, max_sig);
    }

    for (int i=0; i<n_c; i++) {
        if (i < n_v) {
            for (int j=0; j<n_u; j++)
                A[i+j*n_c] = Wv[i]*JG[i+j*n_v];
        } else {
            for (int j=0; j<n_u; j++) {
                if ( (i-n_v) == j ) {
                    A[i+j*n_c] = gamma*Wu[i-n_v];
                } else {
                    A[i+j*n_c] = 0;
                }
            }
        }
    }

    for (int i=0; i<n_c; i++) {
        if (i < n_v) {
            b[i] = Wv[i]*dv[i];
        } else {
            b[i] = gamma*Wu[i-n_v]*up[i-n_v];
        }
    }
}

void gamma_estimator(int n, num_t** A2, num_t cond_target, num_t* gamma, num_t* max_sig){
    /*
    % returns gamma to meet upper bound on condition number. Should run in
    % O(n*d^2) time when optimised. Also return upper bound on maximum
    % eigenvalue
    */
    *max_sig = 0;

    num_t R;
    for (int i=0; i<n; i++) {
        R = 0;
        for (int j=0; j<n; j++) {
            if (j != i)
#ifdef DOUBLE
                R += fabs(A2[i][j]);
#else
                R += fabsf(A2[i][j]);
#endif
        }
        if (*max_sig < (A2[i][i]+R))
            *max_sig = A2[i][i]+R;
    }

#ifdef DOUBLE
    *gamma = sqrt(*max_sig / cond_target);
#else
    *gamma = sqrtf(*max_sig / cond_target);
#endif

}

void cond_estimator(int n, num_t** A2, num_t* min_diag2, num_t* cond_est, num_t* max_sig){
    /*
    % returns upper bound on condition number of A matrix. Should run in
    % O(n*d^2) time when optimised
    */
    *max_sig = 0;

    num_t R;
    for (int i=0; i<n; i++) {
        R = 0;
        for (int j=0; j<n; j++) {
            if (j != i)
#ifdef DOUBLE
                R += fabs(A2[i][j]);
#else
                R += fabsf(A2[i][j]);
#endif
        }
        *max_sig = (*max_sig < A2[i][i]+R) ? A2[i][i]+R : *max_sig;
    }

    *cond_est = *max_sig / ((*min_diag2 > 1e-10) ? *min_diag2 : 1e-10);

}