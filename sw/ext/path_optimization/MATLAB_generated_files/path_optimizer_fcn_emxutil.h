/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: path_optimizer_fcn_emxutil.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef PATH_OPTIMIZER_FCN_EMXUTIL_H
#define PATH_OPTIMIZER_FCN_EMXUTIL_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxFreeStruct_struct_T(c_struct_T *pStruct);

extern void emxFreeStruct_struct_T1(e_struct_T *pStruct);

extern void emxFreeStruct_struct_T2(f_struct_T *pStruct);

extern void emxFreeStruct_struct_T3(g_struct_T *pStruct);

extern void emxFreeStruct_struct_T4(h_struct_T *pStruct);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxInitStruct_struct_T(c_struct_T *pStruct);

extern void emxInitStruct_struct_T1(e_struct_T *pStruct);

extern void emxInitStruct_struct_T2(f_struct_T *pStruct);

extern void emxInitStruct_struct_T3(g_struct_T *pStruct);

extern void emxInitStruct_struct_T4(h_struct_T *pStruct);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for path_optimizer_fcn_emxutil.h
 *
 * [EOF]
 */
