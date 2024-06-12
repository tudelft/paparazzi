/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: partialColLDL3_.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "partialColLDL3_.h"
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
void partialColLDL3_(h_struct_T *obj, int LD_offset, int NColsRemain)
{
  double d;
  int LD_diagOffset;
  int i;
  int i1;
  int i2;
  int ia;
  int idx;
  int ix;
  int j;
  int k;
  int subRows;
  int u1_tmp;
  i = NColsRemain - 1;
  for (k = 0; k < 48; k++) {
    double y;
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset = (LD_offset + 312 * k) - 1;
    for (idx = 0; idx <= subRows; idx++) {
      obj->workspace_ = obj->FMat->data[LD_diagOffset + idx];
    }
    for (idx = 0; idx <= i; idx++) {
      obj->workspace2_ = obj->workspace_;
    }
    y = obj->workspace2_;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      i1 = 311 * (k - 1) + 1;
      for (idx = 1; idx <= i1; idx += 311) {
        i2 = (idx + NColsRemain) - 1;
        for (ia = idx; ia <= i2; ia++) {
          y += obj->workspace_ * -obj->FMat->data[ix - 1];
        }
        ix += 311;
      }
    }
    obj->workspace2_ = y;
    for (idx = 0; idx <= i; idx++) {
      obj->workspace_ = y;
    }
    for (idx = 0; idx <= subRows; idx++) {
      obj->FMat->data[LD_diagOffset + idx] = obj->workspace_;
    }
    for (idx = 0; idx < subRows; idx++) {
      i1 = (LD_diagOffset + idx) + 1;
      obj->FMat->data[i1] /= obj->FMat->data[LD_diagOffset];
    }
  }
  for (j = 48; j <= i; j += 48) {
    int i3;
    int iy0;
    int m;
    int subBlockSize;
    u1_tmp = NColsRemain - j;
    if (u1_tmp >= 48) {
      subBlockSize = 48;
    } else {
      subBlockSize = u1_tmp;
    }
    i1 = j + subBlockSize;
    i2 = i1 - 1;
    for (k = j; k <= i2; k++) {
      m = i1 - k;
      iy0 = (LD_offset + 312 * k) - 1;
      for (idx = 0; idx < 48; idx++) {
        d = obj->FMat->data[((LD_offset + k) + idx * 311) - 1];
      }
      obj->workspace2_ = d;
      ix = k + 1;
      if (m != 0) {
        i3 = k + 14618;
        for (idx = ix; idx <= i3; idx += 311) {
          subRows = (idx + m) - 1;
          for (ia = idx; ia <= subRows; ia++) {
            LD_diagOffset = (iy0 + ia) - idx;
            obj->FMat->data[LD_diagOffset] +=
                obj->workspace_ * -obj->workspace2_;
          }
        }
      }
    }
    if (i1 < NColsRemain) {
      m = u1_tmp - subBlockSize;
      iy0 = ((LD_offset + subBlockSize) + 312 * j) - 1;
      i1 = subBlockSize - 1;
      for (idx = 0; idx < 48; idx++) {
        ix = (LD_offset + j) + idx * 311;
        for (subRows = 0; subRows <= i1; subRows++) {
          obj->workspace2_ = obj->FMat->data[(ix + subRows) - 1];
        }
      }
      if ((m != 0) && (subBlockSize != 0)) {
        ix = iy0 + 311 * (subBlockSize - 1);
        subRows = 0;
        for (LD_diagOffset = iy0; LD_diagOffset <= ix; LD_diagOffset += 311) {
          subRows++;
          i1 = subRows + 14617;
          for (idx = subRows; idx <= i1; idx += 311) {
            i2 = LD_diagOffset + 1;
            i3 = LD_diagOffset + m;
            for (u1_tmp = i2; u1_tmp <= i3; u1_tmp++) {
              obj->FMat->data[u1_tmp - 1] +=
                  -obj->workspace2_ * obj->workspace_;
            }
          }
        }
      }
    }
  }
}

/*
 * File trailer for partialColLDL3_.c
 *
 * [EOF]
 */
