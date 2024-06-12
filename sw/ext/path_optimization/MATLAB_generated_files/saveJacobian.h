/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: saveJacobian.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef SAVEJACOBIAN_H
#define SAVEJACOBIAN_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void saveJacobian(c_struct_T *obj, int nVar,
                  const emxArray_real_T *JacCineqTrans,
                  const double JacCeqTrans_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for saveJacobian.h
 *
 * [EOF]
 */
