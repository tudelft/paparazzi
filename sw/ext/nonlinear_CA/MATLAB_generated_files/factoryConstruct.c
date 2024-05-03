/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: factoryConstruct.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

/* Include Files */
#include "factoryConstruct.h"
#include "Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double fval
 *                b_struct_T *obj
 * Return Type  : void
 */
void factoryConstruct(double fval, b_struct_T *obj)
{
  obj->penaltyParam = 1.0;
  obj->threshold = 0.0001;
  obj->nPenaltyDecreases = 0;
  obj->linearizedConstrViol = 0.0;
  obj->initFval = fval;
  obj->initConstrViolationEq = 0.0;
  obj->initConstrViolationIneq = 0.0;
  obj->phi = 0.0;
  obj->phiPrimePlus = 0.0;
  obj->phiFullStep = 0.0;
  obj->feasRelativeFactor = 0.0;
  obj->nlpPrimalFeasError = 0.0;
  obj->nlpDualFeasError = 0.0;
  obj->nlpComplError = 0.0;
  obj->firstOrderOpt = 0.0;
  obj->hasObjective = true;
}

/*
 * File trailer for factoryConstruct.c
 *
 * [EOF]
 */
