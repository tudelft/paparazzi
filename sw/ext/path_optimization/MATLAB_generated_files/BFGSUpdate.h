/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: BFGSUpdate.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef BFGSUPDATE_H
#define BFGSUPDATE_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
bool BFGSUpdate(int nvar, double Bk[324], const double sk_data[],
                double yk_data[], emxArray_real_T *workspace);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for BFGSUpdate.h
 *
 * [EOF]
 */
