/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: step.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "step.h"
#include "normal.h"
#include "path_optimizer_fcn_internal_types.h"
#include "path_optimizer_fcn_types.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "soc.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int *STEP_TYPE
 *                double Hessian[324]
 *                c_struct_T *TrialState
 *                struct_T *MeritFunction
 *                e_struct_T *memspace
 *                f_struct_T *WorkingSet
 *                g_struct_T *QRManager
 *                h_struct_T *CholManager
 *                d_struct_T *QPObjective
 *                j_struct_T *qpoptions
 * Return Type  : bool
 */
bool step(int *STEP_TYPE, double Hessian[324], c_struct_T *TrialState,
          struct_T *MeritFunction, e_struct_T *memspace, f_struct_T *WorkingSet,
          g_struct_T *QRManager, h_struct_T *CholManager,
          d_struct_T *QPObjective, j_struct_T *qpoptions)
{
  double y_data[311];
  int idxEndIneq;
  int idxStartIneq;
  int idx_global;
  int nVar;
  int y_size;
  bool stepSuccess;
  stepSuccess = true;
  nVar = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    idxStartIneq = (unsigned char)WorkingSet->nVar;
    memcpy(&TrialState->xstar.data[0], &TrialState->xstarsqp[0],
           (unsigned int)idxStartIneq * sizeof(double));
  } else {
    y_size = TrialState->searchDir.size[0];
    idxStartIneq = TrialState->searchDir.size[0];
    if (idxStartIneq - 1 >= 0) {
      memcpy(&y_data[0], &TrialState->searchDir.data[0],
             (unsigned int)idxStartIneq * sizeof(double));
    }
    idxStartIneq = (unsigned short)WorkingSet->nVar;
    if (idxStartIneq - 1 >= 0) {
      memcpy(&y_data[0], &TrialState->xstar.data[0],
             (unsigned int)idxStartIneq * sizeof(double));
    }
    if (y_size - 1 >= 0) {
      memcpy(&TrialState->searchDir.data[0], &y_data[0],
             (unsigned int)y_size * sizeof(double));
    }
  }
  int exitg1;
  bool guard1;
  do {
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
    case 1:
      normal(Hessian, TrialState->grad.data, TrialState, MeritFunction,
             memspace, WorkingSet, QRManager, CholManager, QPObjective,
             qpoptions);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        y_size = TrialState->delta_x.size[0];
        idxStartIneq = TrialState->delta_x.size[0];
        if (idxStartIneq - 1 >= 0) {
          memcpy(&y_data[0], &TrialState->delta_x.data[0],
                 (unsigned int)idxStartIneq * sizeof(double));
        }
        idxStartIneq = (unsigned short)nVar;
        if (idxStartIneq - 1 >= 0) {
          memcpy(&y_data[0], &TrialState->xstar.data[0],
                 (unsigned int)idxStartIneq * sizeof(double));
        }
        if (y_size - 1 >= 0) {
          memcpy(&TrialState->delta_x.data[0], &y_data[0],
                 (unsigned int)y_size * sizeof(double));
        }
        guard1 = true;
      }
      break;
    case 2:
      y_size = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idxStartIneq = y_size + 1;
      idxEndIneq = WorkingSet->nActiveConstr;
      for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
        WorkingSet->isActiveConstr
            .data[(WorkingSet
                       ->isActiveIdx[WorkingSet->Wid.data[idx_global - 1] - 1] +
                   WorkingSet->Wlocalidx.data[idx_global - 1]) -
                  2] = false;
      }
      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = y_size;
      relaxed(Hessian, TrialState->grad.data, TrialState, MeritFunction,
              memspace, WorkingSet, QRManager, CholManager, QPObjective,
              qpoptions);
      y_size = TrialState->delta_x.size[0];
      idxStartIneq = TrialState->delta_x.size[0];
      if (idxStartIneq - 1 >= 0) {
        memcpy(&y_data[0], &TrialState->delta_x.data[0],
               (unsigned int)idxStartIneq * sizeof(double));
      }
      idxStartIneq = (unsigned short)nVar;
      if (idxStartIneq - 1 >= 0) {
        memcpy(&y_data[0], &TrialState->xstar.data[0],
               (unsigned int)idxStartIneq * sizeof(double));
      }
      if (y_size - 1 >= 0) {
        memcpy(&TrialState->delta_x.data[0], &y_data[0],
               (unsigned int)y_size * sizeof(double));
      }
      guard1 = true;
      break;
    default:
      stepSuccess =
          soc(Hessian, TrialState->grad.data, TrialState, memspace, WorkingSet,
              QRManager, CholManager, QPObjective, qpoptions);
      if (stepSuccess && (TrialState->state != -6)) {
        idxStartIneq = (unsigned char)nVar;
        for (y_size = 0; y_size < idxStartIneq; y_size++) {
          TrialState->delta_x.data[y_size] =
              TrialState->xstar.data[y_size] +
              TrialState->socDirection.data[y_size];
        }
      }
      guard1 = true;
      break;
    }
    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        double nrmDirInf;
        double nrmGradInf;
        nrmGradInf = 0.0;
        nrmDirInf = 1.0;
        for (y_size = 0; y_size < 18; y_size++) {
          nrmGradInf = fmax(nrmGradInf, fabs(TrialState->grad.data[y_size]));
          nrmDirInf = fmax(nrmDirInf, fabs(TrialState->xstar.data[y_size]));
        }
        nrmGradInf = fmax(2.2204460492503131E-16, nrmGradInf / nrmDirInf);
        for (idxEndIneq = 0; idxEndIneq < 18; idxEndIneq++) {
          idxStartIneq = 18 * idxEndIneq;
          for (y_size = 0; y_size < idxEndIneq; y_size++) {
            Hessian[idxStartIneq + y_size] = 0.0;
          }
          y_size = idxEndIneq + 18 * idxEndIneq;
          Hessian[y_size] = nrmGradInf;
          idxStartIneq = 16 - idxEndIneq;
          if (idxStartIneq >= 0) {
            memset(&Hessian[y_size + 1], 0,
                   (unsigned int)(((idxStartIneq + y_size) - y_size) + 1) *
                       sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);
  return stepSuccess;
}

/*
 * File trailer for step.c
 *
 * [EOF]
 */
