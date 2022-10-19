#include "qr_wrapper.h"
#include "qr_solve/qr_solve.h"
#include "size_defines.h"
#include <math.h>
#include <string.h>


void qr_wrapper(int m, int n, int perm[], num_t** A, num_t** Q, num_t** R) {
  num_t in[CA_N_C*CA_N_U]; // changed from VLA...
  int k = 0;
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++) {
      in[k++] = A[i][perm[j]];
    }
  }
  int jpvt[CA_N_U];
  //int kr;
  num_t tau[CA_N_U];
  num_t work[CA_N_U];
  int job = 0;

  dqrdc ( in, m, m, n, tau, jpvt, work, job );
  num_t Qout[CA_N_C*CA_N_C];

  dorgqr ( m, n, in, Qout, tau);

  #ifdef debug_qr
  printf("in = [");
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      printf("%f ", in[i + j*m]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("tau = [");
  for (int i = 0; i < m; i++) {
    printf("%f ", tau[i]);
    printf(";\n");
  }
  printf("];\n\n");
  #endif

  for (int i = 0; i < m*n; i++) {
    if (i%m > i/m) {
      R[i%m][i/m] = 0;
    } else {
      R[i%m][i/m] = in[i];
    }
  }

  for (int i = 0; i < m*m; i++)
    Q[i%m][i/m] = Qout[i];

}

int dorgqr ( int m, int n, const num_t a[], num_t q[], num_t tau[])
{
  /*
  The matrix Q is represented as a product of elementary reflectors

     Q = H(0) H(1) . . . H(k), where k = min(m,n)-1.

  Each H(i) has the form

     H(i) = I - tau * v * v**T

  where tau is a real scalar, and v is a real vector with
  v(0:i-1) = 0 and v(i) = 1; v(i+1:m-1) = A(i+1:m-1,i), and tau = TAU(i).
  */
  memset(q, 0.0f, sizeof(num_t)*m*m);
  for (int k=0; k<m; k++) {
    // start with identity
    q[k + k*m] = 1.0F;
  }
  num_t sqtau, isqtau, tsum;
  for (int i=0; i<n; i++) {
    sqtau = sqrtf(tau[i]);
    isqtau = 1/sqtau;

    for (int k=0; k<m; k++) {
      // tsum = Q(k, :)*v
      tsum = q[k + i*m] * sqtau; // *v(i), but v(i)=1
      for (int l=i+1; l<m; l++) {
        tsum += q[k + l*m] * a[l + i*m] * isqtau;
      }

      // Q(k, j) = Q(k, j) - tau(i)*(Q(k, :)*v)*v(j)
      q[k + i*m] -= tsum * sqtau; // *v(j), but v(j==i)=1
      for (int j=i+1; j<m; j++) {
        q[k + j*m] -= tsum*a[j + i*m] * isqtau;
      }
    }
  }
  // i'll eat my hat if this works first try
  return 0;
}
