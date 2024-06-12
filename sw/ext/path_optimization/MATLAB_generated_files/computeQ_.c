/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeQ_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "computeQ_.h"
#include "path_optimizer_fcn_rtwutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include "xgerc.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : g_struct_T *obj
 *                int nrows
 * Return Type  : void
 */
void computeQ_(g_struct_T *obj, int nrows)
{
  double work_data[311];
  int b_i;
  int i;
  int iQR0;
  int ia;
  int k;
  int lastc;
  int lastv;
  int m;
  int n;
  lastv = obj->minRowCol;
  for (lastc = 0; lastc < lastv; lastc++) {
    iQR0 = 311 * lastc + lastc;
    n = obj->mrows - lastc;
    for (k = 0; k <= n - 2; k++) {
      i = (iQR0 + k) + 1;
      obj->Q->data[i] = obj->QR->data[i];
    }
  }
  m = obj->mrows;
  if (nrows >= 1) {
    int itau;
    i = nrows - 1;
    for (iQR0 = lastv; iQR0 <= i; iQR0++) {
      ia = iQR0 * 311;
      n = m - 1;
      for (b_i = 0; b_i <= n; b_i++) {
        obj->Q->data[ia + b_i] = 0.0;
      }
      obj->Q->data[ia + iQR0] = 1.0;
    }
    itau = obj->minRowCol - 1;
    memset(&work_data[0], 0, 311U * sizeof(double));
    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = (b_i + (b_i - 1) * 311) + 311;
      if (b_i < nrows) {
        obj->Q->data[iaii - 312] = 1.0;
        iQR0 = (m - b_i) - 312;
        if (obj->tau.data[itau] != 0.0) {
          bool exitg2;
          lastv = iQR0 + 313;
          iQR0 += iaii;
          while ((lastv > 0) && (obj->Q->data[iQR0] == 0.0)) {
            lastv--;
            iQR0--;
          }
          lastc = nrows - b_i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            int exitg1;
            iQR0 = iaii + (lastc - 1) * 311;
            ia = iQR0;
            do {
              exitg1 = 0;
              if (ia <= (iQR0 + lastv) - 1) {
                if (obj->Q->data[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }
        if (lastv > 0) {
          if (lastc != 0) {
            if (lastc - 1 >= 0) {
              memset(&work_data[0], 0, (unsigned int)lastc * sizeof(double));
            }
            i = iaii + 311 * (lastc - 1);
            for (k = iaii; k <= i; k += 311) {
              double c;
              c = 0.0;
              n = (k + lastv) - 1;
              for (ia = k; ia <= n; ia++) {
                c += obj->Q->data[ia - 1] *
                     obj->Q->data[((iaii + ia) - k) - 312];
              }
              iQR0 = div_nde_s32_floor(k - iaii, 311);
              work_data[iQR0] += c;
            }
          }
          xgerc(lastv, lastc, -obj->tau.data[itau], iaii - 311, work_data,
                obj->Q, iaii);
        }
      }
      if (b_i < m) {
        iQR0 = iaii - 310;
        i = (iaii + m) - b_i;
        for (lastv = iQR0; lastv <= i - 311; lastv++) {
          obj->Q->data[lastv - 1] *= -obj->tau.data[itau];
        }
      }
      obj->Q->data[iaii - 312] = 1.0 - obj->tau.data[itau];
      i = (unsigned char)(b_i - 1);
      for (iQR0 = 0; iQR0 < i; iQR0++) {
        obj->Q->data[(iaii - iQR0) - 313] = 0.0;
      }
      itau--;
    }
  }
}

/*
 * File trailer for computeQ_.c
 *
 * [EOF]
 */
