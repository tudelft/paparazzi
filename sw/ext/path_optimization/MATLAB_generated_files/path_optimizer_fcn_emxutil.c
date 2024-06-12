/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: path_optimizer_fcn_emxutil.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

/* Include Files */
#include "path_optimizer_fcn_emxutil.h"
#include "path_optimizer_fcn_types.h"
#include "rt_nonfinite.h"
#include <stdlib.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = malloc((unsigned int)i * sizeof(double));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(double) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (double *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : c_struct_T *pStruct
 * Return Type  : void
 */
void emxFreeStruct_struct_T(c_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->JacCineqTrans_old);
  emxFree_real_T(&pStruct->JacCeqTrans_old);
}

/*
 * Arguments    : e_struct_T *pStruct
 * Return Type  : void
 */
void emxFreeStruct_struct_T1(e_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->workspace_double);
}

/*
 * Arguments    : f_struct_T *pStruct
 * Return Type  : void
 */
void emxFreeStruct_struct_T2(f_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->Aineq);
  emxFree_real_T(&pStruct->ATwset);
}

/*
 * Arguments    : g_struct_T *pStruct
 * Return Type  : void
 */
void emxFreeStruct_struct_T3(g_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->QR);
  emxFree_real_T(&pStruct->Q);
}

/*
 * Arguments    : h_struct_T *pStruct
 * Return Type  : void
 */
void emxFreeStruct_struct_T4(h_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->FMat);
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/*
 * Arguments    : c_struct_T *pStruct
 * Return Type  : void
 */
void emxInitStruct_struct_T(c_struct_T *pStruct)
{
  pStruct->cIneq.size[0] = 0;
  pStruct->cIneq_old.size[0] = 0;
  pStruct->cEq.size[0] = 0;
  pStruct->cEq_old.size[0] = 0;
  pStruct->grad.size[0] = 0;
  pStruct->grad_old.size[0] = 0;
  pStruct->lambdasqp.size[0] = 0;
  pStruct->lambdaStopTest.size[0] = 0;
  pStruct->lambdaStopTestPrev.size[0] = 0;
  pStruct->delta_x.size[0] = 0;
  pStruct->socDirection.size[0] = 0;
  pStruct->workingset_old.size[0] = 0;
  emxInit_real_T(&pStruct->JacCineqTrans_old, 2);
  emxInit_real_T(&pStruct->JacCeqTrans_old, 2);
  pStruct->gradLag.size[0] = 0;
  pStruct->delta_gradLag.size[0] = 0;
  pStruct->xstar.size[0] = 0;
  pStruct->lambda.size[0] = 0;
  pStruct->searchDir.size[0] = 0;
}

/*
 * Arguments    : e_struct_T *pStruct
 * Return Type  : void
 */
void emxInitStruct_struct_T1(e_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->workspace_double, 2);
  pStruct->workspace_int.size[0] = 0;
  pStruct->workspace_sort.size[0] = 0;
}

/*
 * Arguments    : f_struct_T *pStruct
 * Return Type  : void
 */
void emxInitStruct_struct_T2(f_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->Aineq, 1);
  pStruct->bineq.size[0] = 0;
  pStruct->Aeq.size[0] = 0;
  pStruct->beq.size[0] = 0;
  pStruct->lb.size[0] = 0;
  pStruct->ub.size[0] = 0;
  pStruct->indexLB.size[0] = 0;
  pStruct->indexUB.size[0] = 0;
  pStruct->indexFixed.size[0] = 0;
  pStruct->indexEqRemoved.size[0] = 0;
  emxInit_real_T(&pStruct->ATwset, 1);
  pStruct->bwset.size[0] = 0;
  pStruct->maxConstrWorkspace.size[0] = 0;
  pStruct->isActiveConstr.size[0] = 0;
  pStruct->Wid.size[0] = 0;
  pStruct->Wlocalidx.size[0] = 0;
}

/*
 * Arguments    : g_struct_T *pStruct
 * Return Type  : void
 */
void emxInitStruct_struct_T3(g_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->QR, 2);
  emxInit_real_T(&pStruct->Q, 2);
  pStruct->jpvt.size[0] = 0;
  pStruct->tau.size[0] = 0;
}

/*
 * Arguments    : h_struct_T *pStruct
 * Return Type  : void
 */
void emxInitStruct_struct_T4(h_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->FMat, 2);
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * File trailer for path_optimizer_fcn_emxutil.c
 *
 * [EOF]
 */
