/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: factoryConstruct.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

/* Include Files */
#include "factoryConstruct.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : e_struct_T *objfun_workspace
 *                const double lb[13]
 *                const double ub[13]
 *                m_struct_T *obj
 * Return Type  : void
 */
void factoryConstruct(e_struct_T *objfun_workspace, const double lb[13],
                      const double ub[13], m_struct_T *obj)
{
  int i;
  bool bv[13];
  bool b;
  obj->objfun.workspace = *objfun_workspace;
  obj->f_1 = 0.0;
  obj->f_2 = 0.0;
  obj->nVar = 13;
  obj->mIneq = 0;
  obj->mEq = 0;
  obj->numEvals = 0;
  obj->SpecifyObjectiveGradient = true;
  obj->SpecifyConstraintGradient = false;
  obj->isEmptyNonlcon = true;
  obj->FiniteDifferenceType = 0;
  for (i = 0; i < 13; i++) {
    bv[i] = obj->hasUB[i];
  }
  b = false;
  i = 0;
  while ((!b) && (i + 1 <= 13)) {
    obj->hasLB[i] = ((!rtIsInf(lb[i])) && (!rtIsNaN(lb[i])));
    bv[i] = ((!rtIsInf(ub[i])) && (!rtIsNaN(ub[i])));
    if (obj->hasLB[i] || bv[i]) {
      b = true;
    }
    i++;
  }
  while (i + 1 <= 13) {
    obj->hasLB[i] = ((!rtIsInf(lb[i])) && (!rtIsNaN(lb[i])));
    bv[i] = ((!rtIsInf(ub[i])) && (!rtIsNaN(ub[i])));
    i++;
  }
  for (i = 0; i < 13; i++) {
    obj->hasUB[i] = bv[i];
  }
  obj->hasBounds = b;
}

/*
 * File trailer for factoryConstruct.c
 *
 * [EOF]
 */
