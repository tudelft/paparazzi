/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fullColLDL2_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "fullColLDL2_.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : h_struct_T *obj
 *                int LD_offset
 *                int NColsRemain
 * Return Type  : void
 */
void fullColLDL2_(h_struct_T *obj, int LD_offset, int NColsRemain)
{
  int ijA;
  int j;
  int jA;
  int k;
  for (k = 0; k < NColsRemain; k++) {
    double alpha1;
    double y;
    int LD_diagOffset;
    int i;
    int offset1;
    int subMatrixDim;
    LD_diagOffset = LD_offset + 312 * k;
    alpha1 = -1.0 / obj->FMat->data[LD_diagOffset - 1];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 1;
    y = obj->workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj->FMat->data[LD_diagOffset + jA];
    }
    obj->workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 312;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 312; ijA++) {
            obj->FMat->data[ijA - 1] += y * temp;
          }
        }
        jA += 311;
      }
    }
    alpha1 = 1.0 / obj->FMat->data[LD_diagOffset - 1];
    i = (LD_diagOffset + subMatrixDim) + 1;
    for (jA = offset1; jA <= i; jA++) {
      obj->FMat->data[jA - 1] *= alpha1;
    }
  }
}

/*
 * File trailer for fullColLDL2_.c
 *
 * [EOF]
 */
