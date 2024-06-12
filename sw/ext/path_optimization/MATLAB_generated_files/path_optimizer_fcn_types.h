/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: path_optimizer_fcn_types.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef PATH_OPTIMIZER_FCN_TYPES_H
#define PATH_OPTIMIZER_FCN_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T_183
#define struct_emxArray_real_T_183
struct emxArray_real_T_183 {
  double data[183];
  int size[1];
};
#endif /* struct_emxArray_real_T_183 */
#ifndef typedef_emxArray_real_T_183
#define typedef_emxArray_real_T_183
typedef struct emxArray_real_T_183 emxArray_real_T_183;
#endif /* typedef_emxArray_real_T_183 */

#ifndef struct_emxArray_real_T_18
#define struct_emxArray_real_T_18
struct emxArray_real_T_18 {
  double data[18];
  int size[1];
};
#endif /* struct_emxArray_real_T_18 */
#ifndef typedef_emxArray_real_T_18
#define typedef_emxArray_real_T_18
typedef struct emxArray_real_T_18 emxArray_real_T_18;
#endif /* typedef_emxArray_real_T_18 */

#ifndef struct_emxArray_real_T_311
#define struct_emxArray_real_T_311
struct emxArray_real_T_311 {
  double data[311];
  int size[1];
};
#endif /* struct_emxArray_real_T_311 */
#ifndef typedef_emxArray_real_T_311
#define typedef_emxArray_real_T_311
typedef struct emxArray_real_T_311 emxArray_real_T_311;
#endif /* typedef_emxArray_real_T_311 */

#ifndef struct_emxArray_int32_T_311
#define struct_emxArray_int32_T_311
struct emxArray_int32_T_311 {
  int data[311];
  int size[1];
};
#endif /* struct_emxArray_int32_T_311 */
#ifndef typedef_emxArray_int32_T_311
#define typedef_emxArray_int32_T_311
typedef struct emxArray_int32_T_311 emxArray_int32_T_311;
#endif /* typedef_emxArray_int32_T_311 */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  double xstarsqp[18];
  double xstarsqp_old[18];
  emxArray_real_T_183 cIneq;
  emxArray_real_T_183 cIneq_old;
  emxArray_real_T_18 cEq;
  emxArray_real_T_18 cEq_old;
  emxArray_real_T_183 grad;
  emxArray_real_T_183 grad_old;
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  emxArray_real_T_311 lambdasqp;
  emxArray_real_T_311 lambdaStopTest;
  emxArray_real_T_311 lambdaStopTestPrev;
  double steplength;
  emxArray_real_T_183 delta_x;
  emxArray_real_T_183 socDirection;
  emxArray_int32_T_311 workingset_old;
  emxArray_real_T *JacCineqTrans_old;
  emxArray_real_T *JacCeqTrans_old;
  emxArray_real_T_183 gradLag;
  emxArray_real_T_183 delta_gradLag;
  emxArray_real_T_183 xstar;
  double fstar;
  double firstorderopt;
  emxArray_real_T_311 lambda;
  int state;
  double maxConstr;
  int iterations;
  emxArray_real_T_183 searchDir;
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T
typedef struct {
  emxArray_real_T *workspace_double;
  emxArray_int32_T_311 workspace_int;
  emxArray_int32_T_311 workspace_sort;
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef struct_emxArray_real_T_128
#define struct_emxArray_real_T_128
struct emxArray_real_T_128 {
  double data[128];
  int size[1];
};
#endif /* struct_emxArray_real_T_128 */
#ifndef typedef_emxArray_real_T_128
#define typedef_emxArray_real_T_128
typedef struct emxArray_real_T_128 emxArray_real_T_128;
#endif /* typedef_emxArray_real_T_128 */

#ifndef struct_emxArray_real_T_3294
#define struct_emxArray_real_T_3294
struct emxArray_real_T_3294 {
  double data[3294];
  int size[1];
};
#endif /* struct_emxArray_real_T_3294 */
#ifndef typedef_emxArray_real_T_3294
#define typedef_emxArray_real_T_3294
typedef struct emxArray_real_T_3294 emxArray_real_T_3294;
#endif /* typedef_emxArray_real_T_3294 */

#ifndef struct_emxArray_int32_T_183
#define struct_emxArray_int32_T_183
struct emxArray_int32_T_183 {
  int data[183];
  int size[1];
};
#endif /* struct_emxArray_int32_T_183 */
#ifndef typedef_emxArray_int32_T_183
#define typedef_emxArray_int32_T_183
typedef struct emxArray_int32_T_183 emxArray_int32_T_183;
#endif /* typedef_emxArray_int32_T_183 */

#ifndef struct_emxArray_int32_T_18
#define struct_emxArray_int32_T_18
struct emxArray_int32_T_18 {
  int data[18];
  int size[1];
};
#endif /* struct_emxArray_int32_T_18 */
#ifndef typedef_emxArray_int32_T_18
#define typedef_emxArray_int32_T_18
typedef struct emxArray_int32_T_18 emxArray_int32_T_18;
#endif /* typedef_emxArray_int32_T_18 */

#ifndef struct_emxArray_boolean_T_311
#define struct_emxArray_boolean_T_311
struct emxArray_boolean_T_311 {
  bool data[311];
  int size[1];
};
#endif /* struct_emxArray_boolean_T_311 */
#ifndef typedef_emxArray_boolean_T_311
#define typedef_emxArray_boolean_T_311
typedef struct emxArray_boolean_T_311 emxArray_boolean_T_311;
#endif /* typedef_emxArray_boolean_T_311 */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T
typedef struct {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  emxArray_real_T *Aineq;
  emxArray_real_T_128 bineq;
  emxArray_real_T_3294 Aeq;
  emxArray_real_T_18 beq;
  emxArray_real_T_183 lb;
  emxArray_real_T_183 ub;
  emxArray_int32_T_183 indexLB;
  emxArray_int32_T_183 indexUB;
  emxArray_int32_T_183 indexFixed;
  int mEqRemoved;
  emxArray_int32_T_18 indexEqRemoved;
  emxArray_real_T *ATwset;
  emxArray_real_T_311 bwset;
  int nActiveConstr;
  emxArray_real_T_311 maxConstrWorkspace;
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  emxArray_boolean_T_311 isActiveConstr;
  emxArray_int32_T_311 Wid;
  emxArray_int32_T_311 Wlocalidx;
  int nWConstr[5];
  int probType;
  double SLACK0;
} f_struct_T;
#endif /* typedef_f_struct_T */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  int ldq;
  emxArray_real_T *QR;
  emxArray_real_T *Q;
  emxArray_int32_T_311 jpvt;
  int mrows;
  int ncols;
  emxArray_real_T_311 tau;
  int minRowCol;
  bool usedPivoting;
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T
typedef struct {
  emxArray_real_T *FMat;
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  bool ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
} h_struct_T;
#endif /* typedef_h_struct_T */

#endif
/*
 * File trailer for path_optimizer_fcn_types.h
 *
 * [EOF]
 */
