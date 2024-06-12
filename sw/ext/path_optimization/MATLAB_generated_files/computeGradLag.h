/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeGradLag.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_computeGradLag(double workspace_data[], int nVar,
                      const double grad_data[],
                      const emxArray_real_T *AineqTrans,
                      const double AeqTrans_data[], const int finiteLB_data[],
                      int mLB, const double lambda_data[]);

void computeGradLag(emxArray_real_T *workspace, int nVar,
                    const double grad_data[], const emxArray_real_T *AineqTrans,
                    const double AeqTrans_data[], const int finiteLB_data[],
                    int mLB, const double lambda_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for computeGradLag.h
 *
 * [EOF]
 */
