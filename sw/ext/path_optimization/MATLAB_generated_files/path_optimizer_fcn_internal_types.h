/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: path_optimizer_fcn_internal_types.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Jun-2024 14:47:14
 */

#ifndef PATH_OPTIMIZER_FCN_INTERNAL_TYPES_H
#define PATH_OPTIMIZER_FCN_INTERNAL_TYPES_H

/* Include Files */
#include "path_optimizer_fcn_types.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationEq;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpPrimalFeasError;
  double nlpDualFeasError;
  double nlpComplError;
  double firstOrderOpt;
  bool hasObjective;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  double P0[3];
  double V0[3];
  double A0[3];
  double v_max[3];
  double v_min[3];
  double a_max[3];
  double a_min[3];
  double landing_time;
  double num_points;
  double coeffs_ship_prediction[18];
  double pos_gain;
  double speed_gain;
  double acc_gain;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef typedef_anonymous_function
#define typedef_anonymous_function
typedef struct {
  b_struct_T workspace;
} anonymous_function;
#endif /* typedef_anonymous_function */

#ifndef typedef_coder_internal_stickyStruct
#define typedef_coder_internal_stickyStruct
typedef struct {
  anonymous_function value;
} coder_internal_stickyStruct;
#endif /* typedef_coder_internal_stickyStruct */

#ifndef typedef_b_coder_internal_stickyStruct
#define typedef_b_coder_internal_stickyStruct
typedef struct {
  coder_internal_stickyStruct next;
} b_coder_internal_stickyStruct;
#endif /* typedef_b_coder_internal_stickyStruct */

#ifndef typedef_c_coder_internal_stickyStruct
#define typedef_c_coder_internal_stickyStruct
typedef struct {
  int value;
  b_coder_internal_stickyStruct next;
} c_coder_internal_stickyStruct;
#endif /* typedef_c_coder_internal_stickyStruct */

#ifndef typedef_d_coder_internal_stickyStruct
#define typedef_d_coder_internal_stickyStruct
typedef struct {
  int value;
  c_coder_internal_stickyStruct next;
} d_coder_internal_stickyStruct;
#endif /* typedef_d_coder_internal_stickyStruct */

#ifndef typedef_e_coder_internal_stickyStruct
#define typedef_e_coder_internal_stickyStruct
typedef struct {
  d_coder_internal_stickyStruct next;
} e_coder_internal_stickyStruct;
#endif /* typedef_e_coder_internal_stickyStruct */

#ifndef typedef_f_coder_internal_stickyStruct
#define typedef_f_coder_internal_stickyStruct
typedef struct {
  e_coder_internal_stickyStruct next;
} f_coder_internal_stickyStruct;
#endif /* typedef_f_coder_internal_stickyStruct */

#ifndef typedef_g_coder_internal_stickyStruct
#define typedef_g_coder_internal_stickyStruct
typedef struct {
  f_coder_internal_stickyStruct next;
} g_coder_internal_stickyStruct;
#endif /* typedef_g_coder_internal_stickyStruct */

#ifndef typedef_h_coder_internal_stickyStruct
#define typedef_h_coder_internal_stickyStruct
typedef struct {
  g_coder_internal_stickyStruct next;
} h_coder_internal_stickyStruct;
#endif /* typedef_h_coder_internal_stickyStruct */

#ifndef struct_emxArray_real_T_182
#define struct_emxArray_real_T_182
struct emxArray_real_T_182 {
  double data[182];
  int size[1];
};
#endif /* struct_emxArray_real_T_182 */
#ifndef typedef_emxArray_real_T_182
#define typedef_emxArray_real_T_182
typedef struct emxArray_real_T_182 emxArray_real_T_182;
#endif /* typedef_emxArray_real_T_182 */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  emxArray_real_T_183 grad;
  emxArray_real_T_182 Hx;
  bool hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  bool prev_hasLinear;
  double gammaScalar;
} d_struct_T;
#endif /* typedef_d_struct_T */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T
typedef struct {
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
} i_struct_T;
#endif /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T
typedef struct {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
} j_struct_T;
#endif /* typedef_j_struct_T */

#endif
/*
 * File trailer for path_optimizer_fcn_internal_types.h
 *
 * [EOF]
 */
