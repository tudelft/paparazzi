/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGradLag.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "computeGradLag.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double workspace_data[]
 *                int nVar
 *                const double grad_data[]
 *                const emxArray_real_T *AineqTrans
 *                const double AeqTrans_data[]
 *                const int finiteLB_data[]
 *                int mLB
 *                const double lambda_data[]
 * Return Type  : void
 */
void b_computeGradLag(double workspace_data[], int nVar,
                      const double grad_data[],
                      const emxArray_real_T *AineqTrans,
                      const double AeqTrans_data[], const int finiteLB_data[],
                      int mLB, const double lambda_data[])
{
  const double *AineqTrans_data;
  int i;
  int i1;
  int ia;
  int iac;
  int ix;
  AineqTrans_data = AineqTrans->data;
  i = (unsigned char)nVar;
  memcpy(&workspace_data[0], &grad_data[0], (unsigned int)i * sizeof(double));
  ix = 0;
  for (iac = 0; iac <= 3111; iac += 183) {
    i = iac + nVar;
    for (ia = iac + 1; ia <= i; ia++) {
      i1 = (ia - iac) - 1;
      workspace_data[i1] += AeqTrans_data[ia - 1] * lambda_data[ix];
    }
    ix++;
  }
  ix = 18;
  for (iac = 0; iac <= 23241; iac += 183) {
    i = iac + nVar;
    for (ia = iac + 1; ia <= i; ia++) {
      i1 = (ia - iac) - 1;
      workspace_data[i1] += AineqTrans_data[ia - 1] * lambda_data[ix];
    }
    ix++;
  }
  i = (unsigned char)mLB;
  for (ix = 0; ix < i; ix++) {
    workspace_data[finiteLB_data[ix] - 1] -= lambda_data[ix + 146];
  }
}

/*
 * Arguments    : emxArray_real_T *workspace
 *                int nVar
 *                const double grad_data[]
 *                const emxArray_real_T *AineqTrans
 *                const double AeqTrans_data[]
 *                const int finiteLB_data[]
 *                int mLB
 *                const double lambda_data[]
 * Return Type  : void
 */
void computeGradLag(emxArray_real_T *workspace, int nVar,
                    const double grad_data[], const emxArray_real_T *AineqTrans,
                    const double AeqTrans_data[], const int finiteLB_data[],
                    int mLB, const double lambda_data[])
{
  const double *AineqTrans_data;
  double *workspace_data;
  int i;
  int i1;
  int ia;
  int iac;
  int ix;
  AineqTrans_data = AineqTrans->data;
  workspace_data = workspace->data;
  i = (unsigned char)nVar;
  for (ix = 0; ix < i; ix++) {
    workspace_data[ix] = grad_data[ix];
  }
  ix = 0;
  for (iac = 0; iac <= 3111; iac += 183) {
    i = iac + nVar;
    for (ia = iac + 1; ia <= i; ia++) {
      i1 = (ia - iac) - 1;
      workspace_data[i1] += AeqTrans_data[ia - 1] * lambda_data[ix];
    }
    ix++;
  }
  ix = 18;
  for (iac = 0; iac <= 23241; iac += 183) {
    i = iac + nVar;
    for (ia = iac + 1; ia <= i; ia++) {
      i1 = (ia - iac) - 1;
      workspace_data[i1] += AineqTrans_data[ia - 1] * lambda_data[ix];
    }
    ix++;
  }
  i = (unsigned char)mLB;
  for (ix = 0; ix < i; ix++) {
    workspace_data[finiteLB_data[ix] - 1] -= lambda_data[ix + 146];
  }
}

/*
 * File trailer for computeGradLag.c
 *
 * [EOF]
 */
