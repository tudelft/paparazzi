/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: linesearch.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "linesearch.h"
#include "evalObjAndConstr.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : bool *evalWellDefined
 *                int WorkingSet_nVar
 *                c_struct_T *TrialState
 *                double MeritFunction_penaltyParam
 *                double MeritFunction_phi
 *                double MeritFunction_phiPrimePlus
 *                double MeritFunction_phiFullStep
 *                const b_struct_T *c_FcnEvaluator_next_next_next_n
 *                bool socTaken
 *                int *exitflag
 * Return Type  : double
 */
double linesearch(bool *evalWellDefined, int WorkingSet_nVar,
                  c_struct_T *TrialState, double MeritFunction_penaltyParam,
                  double MeritFunction_phi, double MeritFunction_phiPrimePlus,
                  double MeritFunction_phiFullStep,
                  const b_struct_T *c_FcnEvaluator_next_next_next_n,
                  bool socTaken, int *exitflag)
{
  double y_data[311];
  double alpha;
  double phi_alpha;
  int i;
  int ixlast;
  int y_size;
  alpha = 1.0;
  *exitflag = 1;
  phi_alpha = MeritFunction_phiFullStep;
  y_size = TrialState->searchDir.size[0];
  ixlast = TrialState->searchDir.size[0];
  if (ixlast - 1 >= 0) {
    memcpy(&y_data[0], &TrialState->searchDir.data[0],
           (unsigned int)ixlast * sizeof(double));
  }
  i = (unsigned short)WorkingSet_nVar;
  if (i - 1 >= 0) {
    memcpy(&y_data[0], &TrialState->delta_x.data[0],
           (unsigned int)i * sizeof(double));
  }
  if (y_size - 1 >= 0) {
    memcpy(&TrialState->searchDir.data[0], &y_data[0],
           (unsigned int)y_size * sizeof(double));
  }
  int exitg1;
  do {
    exitg1 = 0;
    if (TrialState->FunctionEvaluations < 1800) {
      if ((*evalWellDefined) &&
          (phi_alpha <=
           MeritFunction_phi + alpha * 0.0001 * MeritFunction_phiPrimePlus)) {
        exitg1 = 1;
      } else {
        bool exitg2;
        bool tooSmallX;
        alpha *= 0.7;
        i = (unsigned char)WorkingSet_nVar;
        for (y_size = 0; y_size < i; y_size++) {
          TrialState->delta_x.data[y_size] =
              alpha * TrialState->xstar.data[y_size];
        }
        if (socTaken) {
          phi_alpha = alpha * alpha;
          if ((WorkingSet_nVar >= 1) && (!(phi_alpha == 0.0))) {
            ixlast = WorkingSet_nVar - 1;
            for (y_size = 0; y_size <= ixlast; y_size++) {
              TrialState->delta_x.data[y_size] +=
                  phi_alpha * TrialState->socDirection.data[y_size];
            }
          }
        }
        tooSmallX = true;
        y_size = 0;
        exitg2 = false;
        while ((!exitg2) && (y_size <= (unsigned char)WorkingSet_nVar - 1)) {
          if (1.0E-6 * fmax(1.0, fabs(TrialState->xstarsqp[y_size])) <=
              fabs(TrialState->delta_x.data[y_size])) {
            tooSmallX = false;
            exitg2 = true;
          } else {
            y_size++;
          }
        }
        if (tooSmallX) {
          *exitflag = -2;
          exitg1 = 1;
        } else {
          for (y_size = 0; y_size < i; y_size++) {
            TrialState->xstarsqp[y_size] = TrialState->xstarsqp_old[y_size] +
                                           TrialState->delta_x.data[y_size];
          }
          TrialState->sqpFval = evalObjAndConstr(
              c_FcnEvaluator_next_next_next_n->P0,
              c_FcnEvaluator_next_next_next_n->V0,
              c_FcnEvaluator_next_next_next_n->A0,
              c_FcnEvaluator_next_next_next_n->v_max,
              c_FcnEvaluator_next_next_next_n->v_min,
              c_FcnEvaluator_next_next_next_n->a_max,
              c_FcnEvaluator_next_next_next_n->a_min,
              c_FcnEvaluator_next_next_next_n->landing_time,
              c_FcnEvaluator_next_next_next_n->num_points,
              c_FcnEvaluator_next_next_next_n->coeffs_ship_prediction,
              c_FcnEvaluator_next_next_next_n->pos_gain,
              c_FcnEvaluator_next_next_next_n->speed_gain,
              c_FcnEvaluator_next_next_next_n->acc_gain, TrialState->xstarsqp,
              TrialState->cIneq.data, TrialState->cEq.data, &ixlast);
          TrialState->FunctionEvaluations++;
          *evalWellDefined = (ixlast == 1);
          if (*evalWellDefined) {
            double constrViolationIneq;
            phi_alpha = 0.0;
            for (y_size = 0; y_size < 18; y_size++) {
              phi_alpha += fabs(TrialState->cEq.data[y_size]);
            }
            constrViolationIneq = 0.0;
            for (y_size = 0; y_size < 128; y_size++) {
              double d;
              d = TrialState->cIneq.data[y_size];
              if (d > 0.0) {
                constrViolationIneq += d;
              }
            }
            phi_alpha =
                TrialState->sqpFval +
                MeritFunction_penaltyParam * (phi_alpha + constrViolationIneq);
          } else {
            phi_alpha = rtInf;
          }
        }
      }
    } else {
      *exitflag = 0;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return alpha;
}

/*
 * File trailer for linesearch.c
 *
 * [EOF]
 */
