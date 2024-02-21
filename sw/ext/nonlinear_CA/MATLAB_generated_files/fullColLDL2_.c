/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fullColLDL2_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Feb-2024 21:29:34
 */

/* Include Files */
#include "fullColLDL2_.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : j_struct_T *obj
 *                int NColsRemain
 * Return Type  : void
 */
void b_fullColLDL2_(j_struct_T *obj, int NColsRemain)
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
    LD_diagOffset = 28 * k;
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 2;
    y = obj->workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj->FMat[(LD_diagOffset + jA) + 1];
    }
    obj->workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 29;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 29; ijA++) {
            obj->FMat[ijA - 1] += y * temp;
          }
        }
        jA += 27;
      }
    }
    alpha1 = 1.0 / obj->FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i + 2; jA++) {
      obj->FMat[jA - 1] *= alpha1;
    }
  }
}

/*
 * Arguments    : g_struct_T *obj
 *                int NColsRemain
 * Return Type  : void
 */
void fullColLDL2_(g_struct_T *obj, int NColsRemain)
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
    LD_diagOffset = k << 5;
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 2;
    y = obj->workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj->FMat[(LD_diagOffset + jA) + 1];
    }
    obj->workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 33;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 33; ijA++) {
            obj->FMat[ijA - 1] += y * temp;
          }
        }
        jA += 31;
      }
    }
    alpha1 = 1.0 / obj->FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i + 2; jA++) {
      obj->FMat[jA - 1] *= alpha1;
    }
  }
}

/*
 * File trailer for fullColLDL2_.c
 *
 * [EOF]
 */
