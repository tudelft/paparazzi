/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: checkStoppingAndUpdateFval.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "checkStoppingAndUpdateFval.h"
#include "computeFval_ReuseHx.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int *activeSetChangeID
 *                const double f_data[]
 *                c_struct_T *solution
 *                e_struct_T *memspace
 *                const d_struct_T *objective
 *                f_struct_T *workingset
 *                g_struct_T *qrmanager
 *                double options_ObjectiveLimit
 *                int runTimeOptions_MaxIterations
 *                bool updateFval
 * Return Type  : void
 */
void checkStoppingAndUpdateFval(int *activeSetChangeID, const double f_data[],
                                c_struct_T *solution, e_struct_T *memspace,
                                const d_struct_T *objective,
                                f_struct_T *workingset, g_struct_T *qrmanager,
                                double options_ObjectiveLimit,
                                int runTimeOptions_MaxIterations,
                                bool updateFval)
{
  double y_data[311];
  solution->iterations++;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }
  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    double tempMaxConstr;
    solution->maxConstr =
        b_maxConstraintViolation(workingset, solution->xstar.data);
    tempMaxConstr = solution->maxConstr;
    if (objective->objtype == 5) {
      tempMaxConstr =
          solution->maxConstr - solution->xstar.data[objective->nvar - 1];
    }
    if (tempMaxConstr > 0.001) {
      int loop_ub;
      int y_size;
      bool nonDegenerateWset;
      y_size = solution->searchDir.size[0];
      loop_ub = solution->searchDir.size[0];
      if (loop_ub - 1 >= 0) {
        memcpy(&y_data[0], &solution->searchDir.data[0],
               (unsigned int)loop_ub * sizeof(double));
      }
      loop_ub = (unsigned short)objective->nvar;
      if (loop_ub - 1 >= 0) {
        memcpy(&y_data[0], &solution->xstar.data[0],
               (unsigned int)loop_ub * sizeof(double));
      }
      if (y_size - 1 >= 0) {
        memcpy(&solution->searchDir.data[0], &y_data[0],
               (unsigned int)y_size * sizeof(double));
      }
      nonDegenerateWset = feasibleX0ForWorkingSet(memspace->workspace_double,
                                                  solution->searchDir.data,
                                                  workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }
      *activeSetChangeID = 0;
      tempMaxConstr =
          b_maxConstraintViolation(workingset, solution->searchDir.data);
      if (tempMaxConstr < solution->maxConstr) {
        loop_ub = (unsigned char)objective->nvar;
        if (loop_ub - 1 >= 0) {
          memcpy(&solution->xstar.data[0], &solution->searchDir.data[0],
                 (unsigned int)loop_ub * sizeof(double));
        }
        solution->maxConstr = tempMaxConstr;
      }
    }
  }
  if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
    solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double,
                                          f_data, solution->xstar.data);
    if ((solution->fstar < options_ObjectiveLimit) &&
        ((solution->state != 0) || (objective->objtype != 5))) {
      solution->state = 2;
    }
  }
}

/*
 * File trailer for checkStoppingAndUpdateFval.c
 *
 * [EOF]
 */
