/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: loadProblem.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "loadProblem.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : f_struct_T *obj
 * Return Type  : void
 */
void loadProblem(f_struct_T *obj)
{
  static const unsigned char uv[6] = {1U, 0U, 18U, 128U, 0U, 0U};
  static const unsigned char uv4[6] = {1U, 0U, 18U, 128U, 1U, 0U};
  static const unsigned char uv5[6] = {1U, 0U, 18U, 128U, 164U, 0U};
  static const unsigned char uv6[6] = {1U, 0U, 18U, 128U, 165U, 0U};
  static const unsigned char obj_tmp[5] = {0U, 18U, 128U, 0U, 0U};
  static const unsigned char uv1[5] = {0U, 18U, 128U, 1U, 0U};
  static const unsigned char uv2[5] = {0U, 18U, 128U, 164U, 0U};
  static const unsigned char uv3[5] = {0U, 18U, 128U, 165U, 0U};
  int i;
  int k;
  obj->mConstr = 146;
  obj->mConstrOrig = 146;
  obj->mConstrMax = 311;
  for (i = 0; i < 5; i++) {
    unsigned char u;
    u = obj_tmp[i];
    obj->sizes[i] = u;
    obj->sizesNormal[i] = u;
    obj->sizesPhaseOne[i] = uv1[i];
    obj->sizesRegularized[i] = uv2[i];
    obj->sizesRegPhaseOne[i] = uv3[i];
  }
  for (i = 0; i < 6; i++) {
    obj->isActiveIdxRegPhaseOne[i] = uv[i];
  }
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (i = 0; i < 6; i++) {
    k = obj->isActiveIdxRegPhaseOne[i];
    obj->isActiveIdx[i] = k;
    obj->isActiveIdxNormal[i] = k;
    obj->isActiveIdxRegPhaseOne[i] = uv4[i];
  }
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (i = 0; i < 6; i++) {
    obj->isActiveIdxPhaseOne[i] = obj->isActiveIdxRegPhaseOne[i];
    obj->isActiveIdxRegPhaseOne[i] = uv5[i];
  }
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (i = 0; i < 6; i++) {
    obj->isActiveIdxRegularized[i] = obj->isActiveIdxRegPhaseOne[i];
    obj->isActiveIdxRegPhaseOne[i] = uv6[i];
  }
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
}

/*
 * File trailer for loadProblem.c
 *
 * [EOF]
 */
