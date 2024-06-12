/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: phaseone.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "phaseone.h"
#include "computeFval.h"
#include "iterate.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double H[324]
 *                const double f_data[]
 *                c_struct_T *solution
 *                e_struct_T *memspace
 *                f_struct_T *workingset
 *                g_struct_T *qrmanager
 *                h_struct_T *cholmanager
 *                d_struct_T *objective
 *                j_struct_T *options
 *                const j_struct_T *runTimeOptions
 * Return Type  : void
 */
void phaseone(const double H[324], const double f_data[], c_struct_T *solution,
              e_struct_T *memspace, f_struct_T *workingset,
              g_struct_T *qrmanager, h_struct_T *cholmanager,
              d_struct_T *objective, j_struct_T *options,
              const j_struct_T *runTimeOptions)
{
  int PROBTYPE_ORIG;
  int idxEndIneq;
  int idxStartIneq;
  int idx_global;
  int mConstr;
  int nVar_tmp;
  PROBTYPE_ORIG = workingset->probType;
  nVar_tmp = workingset->nVar;
  solution->xstar.data[workingset->nVar] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    mConstr = 1;
  } else {
    mConstr = 4;
  }
  setProblemType(workingset, mConstr);
  mConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxStartIneq = mConstr + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
    workingset->isActiveConstr.data
        [(workingset->isActiveIdx[workingset->Wid.data[idx_global - 1] - 1] +
          workingset->Wlocalidx.data[idx_global - 1]) -
         2] = false;
  }
  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = mConstr;
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVar_tmp + 1;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 0.001;
  options->StepTolerance = 1.4901161193847657E-10;
  solution->fstar = computeFval(objective, memspace->workspace_double, H,
                                f_data, solution->xstar.data);
  solution->state = 5;
  iterate(H, f_data, solution, memspace, workingset, qrmanager, cholmanager,
          objective, options->SolverName, options->StepTolerance,
          options->ObjectiveLimit, runTimeOptions->MaxIterations);
  if (workingset->isActiveConstr
          .data[(workingset->isActiveIdx[3] + workingset->sizes[3]) - 2]) {
    bool exitg1;
    mConstr = 19;
    exitg1 = false;
    while ((!exitg1) && (mConstr <= workingset->nActiveConstr)) {
      if ((workingset->Wid.data[mConstr - 1] == 4) &&
          (workingset->Wlocalidx.data[mConstr - 1] == workingset->sizes[3])) {
        removeConstr(workingset, mConstr);
        exitg1 = true;
      } else {
        mConstr++;
      }
    }
  }
  mConstr = workingset->nActiveConstr;
  while ((mConstr > 18) && (mConstr > nVar_tmp)) {
    removeConstr(workingset, mConstr);
    mConstr--;
  }
  solution->maxConstr = solution->xstar.data[nVar_tmp];
  setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = rtMinusInf;
  options->StepTolerance = 1.0E-6;
}

/*
 * File trailer for phaseone.c
 *
 * [EOF]
 */
