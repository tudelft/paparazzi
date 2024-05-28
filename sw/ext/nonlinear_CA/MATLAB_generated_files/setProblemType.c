/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: setProblemType.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 28-May-2024 17:44:56
 */

/* Include Files */
#include "setProblemType.h"
#include "Nonlinear_controller_w_ail_new_aero_sl_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : i_struct_T *obj
 *                int PROBLEM_TYPE
 * Return Type  : void
 */
void setProblemType(i_struct_T *obj, int PROBLEM_TYPE)
{
  int i;
  int idx;
  int idx_col;
  switch (PROBLEM_TYPE) {
  case 3:
    obj->nVar = 15;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = (unsigned char)obj->sizesNormal[4];
      for (idx = 0; idx < i; idx++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + idx) - 1] =
            obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;
  case 1: {
    int idxStartIneq;
    obj->nVar = 16;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      obj->ATwset[(idx << 4) + 15] = 0.0;
    }
    obj->indexLB[obj->sizes[3] - 1] = 16;
    obj->lb[15] = 1.0E-5;
    idxStartIneq = obj->isActiveIdx[2];
    i = obj->nActiveConstr;
    for (idx = idxStartIneq; idx <= i; idx++) {
      obj->ATwset[((idx - 1) << 4) + 15] = -1.0;
    }
    if (obj->nWConstr[4] > 0) {
      i = (unsigned char)(obj->sizesNormal[4] + 1);
      for (idx = 0; idx < i; idx++) {
        obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
      }
    }
    obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
  } break;
  case 2: {
    obj->nVar = 15;
    obj->mConstr = 30;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      int i1;
      int idxStartIneq;
      int idx_lb;
      idx_lb = 15;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (idx = i; idx <= i1; idx++) {
        idx_lb++;
        obj->indexLB[idx - 1] = idx_lb;
      }
      if (obj->nWConstr[4] > 0) {
        i = (unsigned char)obj->sizesRegularized[4];
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr[obj->isActiveIdxRegularized[4] + idx] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
        }
      }
      i = obj->isActiveIdx[4];
      i1 = obj->isActiveIdxRegularized[4] - 1;
      if (i <= i1) {
        memset(&obj->isActiveConstr[i + -1], 0,
               (unsigned int)((i1 - i) + 1) * sizeof(bool));
      }
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = idxStartIneq; idx_col <= i; idx_col++) {
        idx_lb = ((idx_col - 1) << 4) - 1;
        if (obj->Wid[idx_col - 1] == 3) {
          i1 = obj->Wlocalidx[idx_col - 1];
          idx = i1 + 14;
          if (idx >= 16) {
            memset(&obj->ATwset[idx_lb + 16], 0,
                   (unsigned int)(((idx + idx_lb) - idx_lb) - 15) *
                       sizeof(double));
          }
          obj->ATwset[(i1 + idx_lb) + 15] = -1.0;
          i1 += 16;
          if (i1 <= 15) {
            memset(&obj->ATwset[i1 + idx_lb], 0,
                   (unsigned int)(((idx_lb - i1) - idx_lb) + 16) *
                       sizeof(double));
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
  } break;
  default: {
    int idxStartIneq;
    obj->nVar = 16;
    obj->mConstr = 31;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      obj->ATwset[(idx << 4) + 15] = 0.0;
    }
    obj->indexLB[obj->sizes[3] - 1] = 16;
    obj->lb[15] = 1.0E-5;
    idxStartIneq = obj->isActiveIdx[2];
    i = obj->nActiveConstr;
    for (idx = idxStartIneq; idx <= i; idx++) {
      obj->ATwset[((idx - 1) << 4) + 15] = -1.0;
    }
    if (obj->nWConstr[4] > 0) {
      i = (unsigned char)(obj->sizesNormal[4] + 1);
      for (idx = 0; idx < i; idx++) {
        obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
      }
    }
    obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
  } break;
  }
  obj->probType = PROBLEM_TYPE;
}

/*
 * File trailer for setProblemType.c
 *
 * [EOF]
 */
