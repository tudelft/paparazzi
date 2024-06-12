/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortLambdaQP.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef SORTLAMBDAQP_H
#define SORTLAMBDAQP_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void sortLambdaQP(double lambda_data[], int *lambda_size,
                  int WorkingSet_nActiveConstr, const int WorkingSet_sizes[5],
                  const int WorkingSet_isActiveIdx[6],
                  const int WorkingSet_Wid_data[],
                  const int WorkingSet_Wlocalidx_data[],
                  emxArray_real_T *workspace);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sortLambdaQP.h
 *
 * [EOF]
 */
