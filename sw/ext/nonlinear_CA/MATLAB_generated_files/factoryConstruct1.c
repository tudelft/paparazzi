/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: factoryConstruct1.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 20-Feb-2024 13:21:00
 */

/* Include Files */
#include "factoryConstruct1.h"
#include "Cascaded_nonlinear_TestFlight_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double fval
 *                b_struct_T *obj
 * Return Type  : void
 */
void b_factoryConstruct(double fval, b_struct_T *obj)
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
 * File trailer for factoryConstruct1.c
 *
 * [EOF]
 */
