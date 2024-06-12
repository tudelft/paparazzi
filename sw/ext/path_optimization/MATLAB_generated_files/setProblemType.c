/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: setProblemType.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "setProblemType.h"
#include "modifyOverheadPhaseOne_.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 *                int PROBLEM_TYPE
 * Return Type  : void
 */
void setProblemType(f_struct_T *obj, int PROBLEM_TYPE)
{
  int b_idx_col;
  int i;
  int idx_col;
  int idx_row;
  switch (PROBLEM_TYPE) {
  case 3:
    obj->nVar = 18;
    obj->mConstr = 146;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;
  case 1:
    obj->nVar = 19;
    obj->mConstr = 147;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    break;
  case 2: {
    obj->nVar = 182;
    obj->mConstr = 310;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      int colOffsetATw;
      int colOffsetAineq;
      int i1;
      for (idx_col = 0; idx_col < 128; idx_col++) {
        colOffsetAineq = 183 * idx_col - 1;
        i = idx_col + 18;
        for (idx_row = 19; idx_row <= i; idx_row++) {
          obj->Aineq->data[idx_row + colOffsetAineq] = 0.0;
        }
        obj->Aineq->data[(idx_col + colOffsetAineq) + 19] = -1.0;
        i = idx_col + 20;
        for (idx_row = i; idx_row < 183; idx_row++) {
          obj->Aineq->data[idx_row + colOffsetAineq] = 0.0;
        }
      }
      for (idx_col = 0; idx_col < 18; idx_col++) {
        b_idx_col = idx_col + 146;
        colOffsetAineq = 183 * idx_col - 1;
        colOffsetATw = colOffsetAineq + 183 * (obj->isActiveIdx[1] - 1);
        for (idx_row = 0; idx_row < 128; idx_row++) {
          obj->Aeq.data[(idx_row + colOffsetAineq) + 19] = 0.0;
          obj->ATwset->data[(idx_row + colOffsetATw) + 19] = 0.0;
        }
        for (idx_row = 147; idx_row <= b_idx_col; idx_row++) {
          obj->Aeq.data[idx_row + colOffsetAineq] = 0.0;
          obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
        }
        i = idx_col + colOffsetAineq;
        obj->Aeq.data[i + 147] = -1.0;
        b_idx_col = idx_col + colOffsetATw;
        obj->ATwset->data[b_idx_col + 147] = -1.0;
        i1 = idx_col + 148;
        for (idx_row = i1; idx_row < 165; idx_row++) {
          obj->Aeq.data[idx_row + colOffsetAineq] = 0.0;
          obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
        }
        i1 = idx_col + 164;
        for (idx_row = 165; idx_row <= i1; idx_row++) {
          obj->Aeq.data[idx_row + colOffsetAineq] = 0.0;
          obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
        }
        obj->Aeq.data[i + 165] = 1.0;
        obj->ATwset->data[b_idx_col + 165] = 1.0;
        i = idx_col + 166;
        for (idx_row = i; idx_row < 183; idx_row++) {
          obj->Aeq.data[idx_row + colOffsetAineq] = 0.0;
          obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
        }
      }
      colOffsetAineq = 18;
      for (b_idx_col = 0; b_idx_col < 164; b_idx_col++) {
        colOffsetAineq++;
        obj->indexLB.data[b_idx_col] = colOffsetAineq;
      }
      i = obj->isActiveIdx[4];
      b_idx_col = obj->isActiveIdxRegularized[4] - 1;
      if (i <= b_idx_col) {
        memset(&obj->isActiveConstr.data[i + -1], 0,
               (unsigned int)((b_idx_col - i) + 1) * sizeof(bool));
      }
      memset(&obj->lb.data[18], 0, 164U * sizeof(double));
      colOffsetAineq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = colOffsetAineq; idx_col <= i; idx_col++) {
        colOffsetATw = 183 * (idx_col - 1) - 1;
        if (obj->Wid.data[idx_col - 1] == 3) {
          b_idx_col = obj->Wlocalidx.data[idx_col - 1];
          i1 = b_idx_col + 17;
          for (idx_row = 19; idx_row <= i1; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }
          obj->ATwset->data[(b_idx_col + colOffsetATw) + 18] = -1.0;
          b_idx_col += 19;
          for (idx_row = b_idx_col; idx_row < 183; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }
        } else {
          for (idx_row = 0; idx_row < 164; idx_row++) {
            obj->ATwset->data[(idx_row + colOffsetATw) + 19] = 0.0;
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
  } break;
  default:
    obj->nVar = 183;
    obj->mConstr = 311;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    break;
  }
  obj->probType = PROBLEM_TYPE;
}

/*
 * File trailer for setProblemType.c
 *
 * [EOF]
 */
