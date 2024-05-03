/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-May-2024 02:28:05
 */

#ifndef CASCADED_NONLINEAR_CONTROLLER_W_AIL_NEW_AERO_INTERNAL_TYPES_H
#define CASCADED_NONLINEAR_CONTROLLER_W_AIL_NEW_AERO_INTERNAL_TYPES_H

/* Include Files */
#include "Cascaded_nonlinear_controller_w_ail_new_aero_types.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  double grad[16];
  double Hx[15];
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
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
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
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  double grad[14];
  double Hx[13];
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
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_captured_var
#define typedef_captured_var
typedef struct {
  double contents;
} captured_var;
#endif /* typedef_captured_var */

#ifndef typedef_b_captured_var
#define typedef_b_captured_var
typedef struct {
  double contents[6];
} b_captured_var;
#endif /* typedef_b_captured_var */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  b_captured_var *dv_global;
  captured_var *Beta;
  captured_var *CL_aileron;
  captured_var *Cd_zero;
  captured_var *Cl_alpha;
  captured_var *Cm_zero;
  captured_var *Cm_alpha;
  captured_var *I_xx;
  captured_var *I_yy;
  captured_var *I_zz;
  captured_var *K_Cd;
  captured_var *S;
  captured_var *V;
  captured_var *V_scaled;
  captured_var *W_act_phi;
  captured_var *W_act_theta;
  captured_var *W_act_motor;
  captured_var *W_dv_1;
  captured_var *W_dv_2;
  captured_var *W_dv_3;
  captured_var *W_dv_4;
  captured_var *W_dv_5;
  captured_var *W_dv_6;
  captured_var *W_act_tilt_el;
  captured_var *W_act_tilt_az;
  captured_var *W_act_ailerons;
  captured_var *desired_el_value;
  captured_var *desired_az_value;
  captured_var *desired_phi_value;
  captured_var *desired_theta_value;
  captured_var *desired_motor_value;
  captured_var *desired_ailerons_value;
  captured_var *flight_path_angle;
  captured_var *gain_el;
  captured_var *gain_az;
  captured_var *gain_phi;
  captured_var *gain_theta;
  captured_var *gain_motor;
  captured_var *gain_ailerons;
  captured_var *gamma_quadratic_du;
  captured_var *l_1;
  captured_var *l_2;
  captured_var *l_3;
  captured_var *l_4;
  captured_var *l_z;
  captured_var *m;
  captured_var *p;
  captured_var *power_Cd_0;
  captured_var *power_Cd_a;
  captured_var *prop_R;
  captured_var *prop_Cd_0;
  captured_var *prop_Cl_0;
  captured_var *prop_Cd_a;
  captured_var *prop_Cl_a;
  captured_var *prop_delta;
  captured_var *prop_sigma;
  captured_var *prop_theta;
  captured_var *q;
  captured_var *r;
  captured_var *rho;
  captured_var *wing_span;
  captured_var *wing_chord;
} d_struct_T;
#endif /* typedef_d_struct_T */

#ifndef typedef_nested_function
#define typedef_nested_function
typedef struct {
  d_struct_T workspace;
} nested_function;
#endif /* typedef_nested_function */

#ifndef typedef_coder_internal_stickyStruct
#define typedef_coder_internal_stickyStruct
typedef struct {
  nested_function value;
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
  b_coder_internal_stickyStruct next;
} c_coder_internal_stickyStruct;
#endif /* typedef_c_coder_internal_stickyStruct */

#ifndef typedef_d_coder_internal_stickyStruct
#define typedef_d_coder_internal_stickyStruct
typedef struct {
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

#ifndef typedef_i_coder_internal_stickyStruct
#define typedef_i_coder_internal_stickyStruct
typedef struct {
  h_coder_internal_stickyStruct next;
} i_coder_internal_stickyStruct;
#endif /* typedef_i_coder_internal_stickyStruct */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T
typedef struct {
  b_captured_var *dv_global;
  captured_var *Beta;
  captured_var *CL_aileron;
  captured_var *Cd_zero;
  captured_var *Cl_alpha;
  captured_var *Cm_zero;
  captured_var *Cm_alpha;
  captured_var *I_xx;
  captured_var *I_yy;
  captured_var *I_zz;
  captured_var *K_Cd;
  captured_var *Phi;
  captured_var *S;
  captured_var *Theta;
  captured_var *V;
  captured_var *V_scaled;
  captured_var *W_act_motor;
  captured_var *W_dv_1;
  captured_var *W_dv_2;
  captured_var *W_dv_3;
  captured_var *W_dv_4;
  captured_var *W_dv_5;
  captured_var *W_dv_6;
  captured_var *W_act_tilt_el;
  captured_var *W_act_tilt_az;
  captured_var *W_act_ailerons;
  captured_var *desired_el_value;
  captured_var *desired_az_value;
  captured_var *desired_motor_value;
  captured_var *desired_ailerons_value;
  captured_var *flight_path_angle;
  captured_var *gain_el;
  captured_var *gain_az;
  captured_var *gain_motor;
  captured_var *gain_ailerons;
  captured_var *gamma_quadratic_du;
  captured_var *l_1;
  captured_var *l_2;
  captured_var *l_3;
  captured_var *l_4;
  captured_var *l_z;
  captured_var *m;
  captured_var *p;
  captured_var *power_Cd_0;
  captured_var *power_Cd_a;
  captured_var *prop_R;
  captured_var *prop_Cd_0;
  captured_var *prop_Cl_0;
  captured_var *prop_Cd_a;
  captured_var *prop_Cl_a;
  captured_var *prop_delta;
  captured_var *prop_sigma;
  captured_var *prop_theta;
  captured_var *q;
  captured_var *r;
  captured_var *rho;
  captured_var *wing_span;
  captured_var *wing_chord;
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef typedef_b_nested_function
#define typedef_b_nested_function
typedef struct {
  e_struct_T workspace;
} b_nested_function;
#endif /* typedef_b_nested_function */

#ifndef typedef_j_coder_internal_stickyStruct
#define typedef_j_coder_internal_stickyStruct
typedef struct {
  b_nested_function value;
} j_coder_internal_stickyStruct;
#endif /* typedef_j_coder_internal_stickyStruct */

#ifndef typedef_k_coder_internal_stickyStruct
#define typedef_k_coder_internal_stickyStruct
typedef struct {
  j_coder_internal_stickyStruct next;
} k_coder_internal_stickyStruct;
#endif /* typedef_k_coder_internal_stickyStruct */

#ifndef typedef_l_coder_internal_stickyStruct
#define typedef_l_coder_internal_stickyStruct
typedef struct {
  k_coder_internal_stickyStruct next;
} l_coder_internal_stickyStruct;
#endif /* typedef_l_coder_internal_stickyStruct */

#ifndef typedef_m_coder_internal_stickyStruct
#define typedef_m_coder_internal_stickyStruct
typedef struct {
  l_coder_internal_stickyStruct next;
} m_coder_internal_stickyStruct;
#endif /* typedef_m_coder_internal_stickyStruct */

#ifndef typedef_n_coder_internal_stickyStruct
#define typedef_n_coder_internal_stickyStruct
typedef struct {
  m_coder_internal_stickyStruct next;
} n_coder_internal_stickyStruct;
#endif /* typedef_n_coder_internal_stickyStruct */

#ifndef typedef_o_coder_internal_stickyStruct
#define typedef_o_coder_internal_stickyStruct
typedef struct {
  n_coder_internal_stickyStruct next;
} o_coder_internal_stickyStruct;
#endif /* typedef_o_coder_internal_stickyStruct */

#ifndef typedef_p_coder_internal_stickyStruct
#define typedef_p_coder_internal_stickyStruct
typedef struct {
  o_coder_internal_stickyStruct next;
} p_coder_internal_stickyStruct;
#endif /* typedef_p_coder_internal_stickyStruct */

#ifndef typedef_q_coder_internal_stickyStruct
#define typedef_q_coder_internal_stickyStruct
typedef struct {
  p_coder_internal_stickyStruct next;
} q_coder_internal_stickyStruct;
#endif /* typedef_q_coder_internal_stickyStruct */

#ifndef typedef_r_coder_internal_stickyStruct
#define typedef_r_coder_internal_stickyStruct
typedef struct {
  q_coder_internal_stickyStruct next;
} r_coder_internal_stickyStruct;
#endif /* typedef_r_coder_internal_stickyStruct */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T
typedef struct {
  int ldq;
  double QR[961];
  double Q[961];
  int jpvt[31];
  int mrows;
  int ncols;
  double tau[31];
  int minRowCol;
  bool usedPivoting;
} f_struct_T;
#endif /* typedef_f_struct_T */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  double FMat[961];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  bool ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T
typedef struct {
  double workspace_double[496];
  int workspace_int[31];
  int workspace_sort[31];
} h_struct_T;
#endif /* typedef_h_struct_T */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T
typedef struct {
  int ldq;
  double QR[729];
  double Q[729];
  int jpvt[27];
  int mrows;
  int ncols;
  double tau[27];
  int minRowCol;
  bool usedPivoting;
} i_struct_T;
#endif /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T
typedef struct {
  double FMat[729];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  bool ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
} j_struct_T;
#endif /* typedef_j_struct_T */

#ifndef typedef_k_struct_T
#define typedef_k_struct_T
typedef struct {
  double workspace_double[378];
  int workspace_int[27];
  int workspace_sort[27];
} k_struct_T;
#endif /* typedef_k_struct_T */

#ifndef typedef_l_struct_T
#define typedef_l_struct_T
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
  double xstarsqp[15];
  double xstarsqp_old[15];
  double grad[16];
  double grad_old[16];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[31];
  double lambdaStopTest[31];
  double lambdaStopTestPrev[31];
  double steplength;
  double delta_x[16];
  double socDirection[16];
  int workingset_old[31];
  double gradLag[16];
  double delta_gradLag[16];
  double xstar[16];
  double fstar;
  double firstorderopt;
  double lambda[31];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[16];
} l_struct_T;
#endif /* typedef_l_struct_T */

#ifndef typedef_m_struct_T
#define typedef_m_struct_T
typedef struct {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double lb[16];
  double ub[16];
  int indexLB[16];
  int indexUB[16];
  int indexFixed[16];
  int mEqRemoved;
  double ATwset[496];
  double bwset[31];
  int nActiveConstr;
  double maxConstrWorkspace[31];
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
  bool isActiveConstr[31];
  int Wid[31];
  int Wlocalidx[31];
  int nWConstr[5];
  int probType;
  double SLACK0;
} m_struct_T;
#endif /* typedef_m_struct_T */

#ifndef typedef_n_struct_T
#define typedef_n_struct_T
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
  double xstarsqp[13];
  double xstarsqp_old[13];
  double grad[14];
  double grad_old[14];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[27];
  double lambdaStopTest[27];
  double lambdaStopTestPrev[27];
  double steplength;
  double delta_x[14];
  double socDirection[14];
  int workingset_old[27];
  double gradLag[14];
  double delta_gradLag[14];
  double xstar[14];
  double fstar;
  double firstorderopt;
  double lambda[27];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[14];
} n_struct_T;
#endif /* typedef_n_struct_T */

#ifndef typedef_o_struct_T
#define typedef_o_struct_T
typedef struct {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double lb[14];
  double ub[14];
  int indexLB[14];
  int indexUB[14];
  int indexFixed[14];
  int mEqRemoved;
  double ATwset[378];
  double bwset[27];
  int nActiveConstr;
  double maxConstrWorkspace[27];
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
  bool isActiveConstr[27];
  int Wid[27];
  int Wlocalidx[27];
  int nWConstr[5];
  int probType;
  double SLACK0;
} o_struct_T;
#endif /* typedef_o_struct_T */

#ifndef typedef_p_struct_T
#define typedef_p_struct_T
typedef struct {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
} p_struct_T;
#endif /* typedef_p_struct_T */

#ifndef typedef_q_struct_T
#define typedef_q_struct_T
typedef struct {
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
} q_struct_T;
#endif /* typedef_q_struct_T */

#endif
/*
 * File trailer for
 * Cascaded_nonlinear_controller_w_ail_new_aero_internal_types.h
 *
 * [EOF]
 */
