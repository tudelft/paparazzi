/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Cascaded_nonlinear_controller_control_rf_w_ailerons.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Dec-2023 22:29:53
 */

/* Include Files */
#include "Cascaded_nonlinear_controller_control_rf_w_ailerons.h"
#include "coder_posix_time.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

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

#endif                                 /* typedef_struct_T */

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

#endif                                 /* typedef_b_struct_T */

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

#endif                                 /* typedef_c_struct_T */

#ifndef typedef_captured_var
#define typedef_captured_var

typedef struct {
  double contents;
} captured_var;

#endif                                 /* typedef_captured_var */

#ifndef typedef_b_captured_var
#define typedef_b_captured_var

typedef struct {
  double contents[6];
} b_captured_var;

#endif                                 /* typedef_b_captured_var */

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
  captured_var *K_p_M;
  captured_var *K_p_T;
  captured_var *S;
  captured_var *V;
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
  captured_var *q;
  captured_var *r;
  captured_var *rho;
  captured_var *wing_chord;
} d_struct_T;

#endif                                 /* typedef_d_struct_T */

#ifndef typedef_nested_function
#define typedef_nested_function

typedef struct {
  d_struct_T workspace;
} nested_function;

#endif                                 /* typedef_nested_function */

#ifndef typedef_coder_internal_stickyStruct
#define typedef_coder_internal_stickyStruct

typedef struct {
  nested_function value;
} coder_internal_stickyStruct;

#endif                                 /* typedef_coder_internal_stickyStruct */

#ifndef typedef_b_coder_internal_stickyStruct
#define typedef_b_coder_internal_stickyStruct

typedef struct {
  coder_internal_stickyStruct next;
} b_coder_internal_stickyStruct;

#endif                                 /* typedef_b_coder_internal_stickyStruct */

#ifndef typedef_c_coder_internal_stickyStruct
#define typedef_c_coder_internal_stickyStruct

typedef struct {
  b_coder_internal_stickyStruct next;
} c_coder_internal_stickyStruct;

#endif                                 /* typedef_c_coder_internal_stickyStruct */

#ifndef typedef_d_coder_internal_stickyStruct
#define typedef_d_coder_internal_stickyStruct

typedef struct {
  c_coder_internal_stickyStruct next;
} d_coder_internal_stickyStruct;

#endif                                 /* typedef_d_coder_internal_stickyStruct */

#ifndef typedef_e_coder_internal_stickyStruct
#define typedef_e_coder_internal_stickyStruct

typedef struct {
  d_coder_internal_stickyStruct next;
} e_coder_internal_stickyStruct;

#endif                                 /* typedef_e_coder_internal_stickyStruct */

#ifndef typedef_f_coder_internal_stickyStruct
#define typedef_f_coder_internal_stickyStruct

typedef struct {
  e_coder_internal_stickyStruct next;
} f_coder_internal_stickyStruct;

#endif                                 /* typedef_f_coder_internal_stickyStruct */

#ifndef typedef_g_coder_internal_stickyStruct
#define typedef_g_coder_internal_stickyStruct

typedef struct {
  f_coder_internal_stickyStruct next;
} g_coder_internal_stickyStruct;

#endif                                 /* typedef_g_coder_internal_stickyStruct */

#ifndef typedef_h_coder_internal_stickyStruct
#define typedef_h_coder_internal_stickyStruct

typedef struct {
  g_coder_internal_stickyStruct next;
} h_coder_internal_stickyStruct;

#endif                                 /* typedef_h_coder_internal_stickyStruct */

#ifndef typedef_i_coder_internal_stickyStruct
#define typedef_i_coder_internal_stickyStruct

typedef struct {
  h_coder_internal_stickyStruct next;
} i_coder_internal_stickyStruct;

#endif                                 /* typedef_i_coder_internal_stickyStruct */

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
  captured_var *K_p_M;
  captured_var *K_p_T;
  captured_var *Phi;
  captured_var *S;
  captured_var *Theta;
  captured_var *V;
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
  captured_var *q;
  captured_var *r;
  captured_var *rho;
  captured_var *wing_chord;
} e_struct_T;

#endif                                 /* typedef_e_struct_T */

#ifndef typedef_b_nested_function
#define typedef_b_nested_function

typedef struct {
  e_struct_T workspace;
} b_nested_function;

#endif                                 /* typedef_b_nested_function */

#ifndef typedef_j_coder_internal_stickyStruct
#define typedef_j_coder_internal_stickyStruct

typedef struct {
  b_nested_function value;
} j_coder_internal_stickyStruct;

#endif                                 /* typedef_j_coder_internal_stickyStruct */

#ifndef typedef_k_coder_internal_stickyStruct
#define typedef_k_coder_internal_stickyStruct

typedef struct {
  j_coder_internal_stickyStruct next;
} k_coder_internal_stickyStruct;

#endif                                 /* typedef_k_coder_internal_stickyStruct */

#ifndef typedef_l_coder_internal_stickyStruct
#define typedef_l_coder_internal_stickyStruct

typedef struct {
  k_coder_internal_stickyStruct next;
} l_coder_internal_stickyStruct;

#endif                                 /* typedef_l_coder_internal_stickyStruct */

#ifndef typedef_m_coder_internal_stickyStruct
#define typedef_m_coder_internal_stickyStruct

typedef struct {
  l_coder_internal_stickyStruct next;
} m_coder_internal_stickyStruct;

#endif                                 /* typedef_m_coder_internal_stickyStruct */

#ifndef typedef_n_coder_internal_stickyStruct
#define typedef_n_coder_internal_stickyStruct

typedef struct {
  m_coder_internal_stickyStruct next;
} n_coder_internal_stickyStruct;

#endif                                 /* typedef_n_coder_internal_stickyStruct */

#ifndef typedef_o_coder_internal_stickyStruct
#define typedef_o_coder_internal_stickyStruct

typedef struct {
  n_coder_internal_stickyStruct next;
} o_coder_internal_stickyStruct;

#endif                                 /* typedef_o_coder_internal_stickyStruct */

#ifndef typedef_p_coder_internal_stickyStruct
#define typedef_p_coder_internal_stickyStruct

typedef struct {
  o_coder_internal_stickyStruct next;
} p_coder_internal_stickyStruct;

#endif                                 /* typedef_p_coder_internal_stickyStruct */

#ifndef typedef_q_coder_internal_stickyStruct
#define typedef_q_coder_internal_stickyStruct

typedef struct {
  p_coder_internal_stickyStruct next;
} q_coder_internal_stickyStruct;

#endif                                 /* typedef_q_coder_internal_stickyStruct */

#ifndef typedef_r_coder_internal_stickyStruct
#define typedef_r_coder_internal_stickyStruct

typedef struct {
  q_coder_internal_stickyStruct next;
} r_coder_internal_stickyStruct;

#endif                                 /* typedef_r_coder_internal_stickyStruct */

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

#endif                                 /* typedef_f_struct_T */

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

#endif                                 /* typedef_g_struct_T */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

typedef struct {
  double workspace_double[496];
  int workspace_int[31];
  int workspace_sort[31];
} h_struct_T;

#endif                                 /* typedef_h_struct_T */

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

#endif                                 /* typedef_i_struct_T */

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

#endif                                 /* typedef_j_struct_T */

#ifndef typedef_k_struct_T
#define typedef_k_struct_T

typedef struct {
  double workspace_double[378];
  int workspace_int[27];
  int workspace_sort[27];
} k_struct_T;

#endif                                 /* typedef_k_struct_T */

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

#endif                                 /* typedef_l_struct_T */

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

#endif                                 /* typedef_m_struct_T */

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

#endif                                 /* typedef_n_struct_T */

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

#endif                                 /* typedef_o_struct_T */

#ifndef typedef_p_struct_T
#define typedef_p_struct_T

typedef struct {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
} p_struct_T;

#endif                                 /* typedef_p_struct_T */

#ifndef typedef_q_struct_T
#define typedef_q_struct_T

typedef struct {
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
} q_struct_T;

#endif                                 /* typedef_q_struct_T */

/* Variable Definitions */
static double freq;
static bool freq_not_empty;
static coderTimespec savedTime;
static bool savedTime_not_empty;
static bool isInitialized_Cascaded_nonlinear_controller_control_rf_w_ailerons =
  false;

/* Function Declarations */
static bool BFGSUpdate(int nvar, double Bk[225], const double sk[16], double yk
  [16], double workspace[496]);
static void PresolveWorkingSet(l_struct_T *solution, h_struct_T *memspace,
  m_struct_T *workingset, f_struct_T *qrmanager);
static void RemoveDependentIneq_(m_struct_T *workingset, f_struct_T *qrmanager,
  h_struct_T *memspace, double tolfactor);
static void addBoundToActiveSetMatrix_(m_struct_T *obj, int TYPE, int idx_local);
static bool b_BFGSUpdate(int nvar, double Bk[169], const double sk[14], double
  yk[14], double workspace[378]);
static void b_PresolveWorkingSet(n_struct_T *solution, k_struct_T *memspace,
  o_struct_T *workingset, i_struct_T *qrmanager);
static void b_RemoveDependentIneq_(o_struct_T *workingset, i_struct_T *qrmanager,
  k_struct_T *memspace, double tolfactor);
static void b_addBoundToActiveSetMatrix_(o_struct_T *obj, int TYPE, int
  idx_local);
static double b_computeComplError(const double xCurrent[13], const int finiteLB
  [14], int mLB, const double lb[13], const int finiteUB[14], int mUB, const
  double ub[13], const double lambda[27], int iL0);
static double b_computeFval(const c_struct_T *obj, double workspace[378], const
  double H[169], const double f[14], const double x[14]);
static double b_computeFval_ReuseHx(const c_struct_T *obj, double workspace[378],
  const double f[14], const double x[14]);
static void b_computeGradLag(double workspace[496], int nVar, const double grad
  [16], const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB,
  const int finiteUB[16], int mUB, const double lambda[31]);
static void b_computeGrad_StoreHx(c_struct_T *obj, const double H[169], const
  double f[14], const double x[14]);
static void b_computeQ_(i_struct_T *obj, int nrows);
static void b_compute_deltax(const double H[169], n_struct_T *solution,
  k_struct_T *memspace, const i_struct_T *qrmanager, j_struct_T *cholmanager,
  const c_struct_T *objective, bool alwaysPositiveDef);
static void b_countsort(int x[27], int xLen, int workspace[27], int xMin, int
  xMax);
static void b_deleteColMoveEnd(i_struct_T *obj, int idx);
static void b_driver(const double H[169], const double f[14], n_struct_T
                     *solution, k_struct_T *memspace, o_struct_T *workingset,
                     i_struct_T *qrmanager, j_struct_T *cholmanager, c_struct_T *
                     objective, p_struct_T *options, int
                     runTimeOptions_MaxIterations);
static void b_factorQR(i_struct_T *obj, const double A[378], int mrows, int
  ncols);
static bool b_feasibleX0ForWorkingSet(double workspace[378], double xCurrent[14],
  const o_struct_T *workingset, i_struct_T *qrmanager);
static double b_feasibleratiotest(const double solution_xstar[14], const double
  solution_searchDir[14], int workingset_nVar, const double workingset_lb[14],
  const double workingset_ub[14], const int workingset_indexLB[14], const int
  workingset_indexUB[14], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[27], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx);
static double b_fmincon(e_struct_T *fun_workspace, double x0[13], const double
  lb[13], const double ub[13], double *exitflag, double *output_iterations,
  double *output_funcCount, char output_algorithm[3], double
  *output_constrviolation, double *output_stepsize, double *output_lssteplength,
  double *output_firstorderopt);
static void b_fullColLDL2_(j_struct_T *obj, int NColsRemain);
static void b_iterate(const double H[169], const double f[14], n_struct_T
                      *solution, k_struct_T *memspace, o_struct_T *workingset,
                      i_struct_T *qrmanager, j_struct_T *cholmanager, c_struct_T
                      *objective, const char options_SolverName[7], double
                      options_StepTolerance, double options_ObjectiveLimit, int
                      runTimeOptions_MaxIterations);
static void b_linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[378],
  const double H[169], const double f[14], const double x[14]);
static double b_maxConstraintViolation(const o_struct_T *obj, const double x[378],
  int ix0);
static double b_norm(const double x[6]);
static void b_qrf(double A[729], int m, int n, int nfxd, double tau[27]);
static void b_removeConstr(o_struct_T *obj, int idx_global);
static void b_setProblemType(o_struct_T *obj, int PROBLEM_TYPE);
static void b_solve(const j_struct_T *obj, double rhs[14]);
static void b_sortLambdaQP(double lambda[27], int WorkingSet_nActiveConstr,
  const int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[27], const int WorkingSet_Wlocalidx[27], double workspace[378]);
static bool b_step(int *STEP_TYPE, double Hessian[169], const double lb[13],
                   const double ub[13], n_struct_T *TrialState, b_struct_T
                   *MeritFunction, k_struct_T *memspace, o_struct_T *WorkingSet,
                   i_struct_T *QRManager, j_struct_T *CholManager, c_struct_T
                   *QPObjective, p_struct_T *qpoptions);
static void b_test_exit(q_struct_T *Flags, h_struct_T *memspace, b_struct_T
  *MeritFunction, const m_struct_T *WorkingSet, l_struct_T *TrialState,
  f_struct_T *QRManager, const double lb[15], const double ub[15]);
static double b_timeKeeper(double *outTime_tv_nsec);
static void b_xgemm(int m, int n, int k, const double A[961], int ia0, const
                    double B[496], double C[961]);
static void b_xgemv(int m, int n, const double A[169], int lda, const double x
                    [14], double y[13]);
static void b_xgeqp3(double A[729], int m, int n, int jpvt[27], double tau[27]);
static double b_xnrm2(int n, const double x[16]);
static int b_xpotrf(int n, double A[729]);
static void b_xzlarf(int m, int n, int iv0, double tau, double C[729], int ic0,
                     double work[27]);
static double b_xzlarfg(int n, double *alpha1, double x[729], int ix0);
static void c_CoderTimeAPI_callCoderClockGe(void);
static void c_computeGradLag(double workspace[14], int nVar, const double grad
  [14], const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB,
  const int finiteUB[14], int mUB, const double lambda[27]);
static void c_compute_acc_cascaded_nonlinea(const double u_in[15], double p,
  double q, double r, double K_p_T, double K_p_M, double m, double I_xx, double
  I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4, double l_z,
  double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero,
  double CL_aileron, double rho, double V, double S, double wing_chord, double
  flight_path_angle, double Beta, double accelerations_array[6]);
static double c_compute_cost_and_gradient_fir(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *S, const captured_var *V, const
  captured_var *W_act_phi, const captured_var *W_act_theta, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_phi_value, const captured_var *desired_theta_value,
  const captured_var *desired_motor_value, const captured_var
  *desired_ailerons_value, const captured_var *flight_path_angle, const
  captured_var *gain_el, const captured_var *gain_az, const captured_var
  *gain_phi, const captured_var *gain_theta, const captured_var *gain_motor,
  const captured_var *gain_ailerons, const captured_var *gamma_quadratic_du,
  const captured_var *l_1, const captured_var *l_2, const captured_var *l_3,
  const captured_var *l_4, const captured_var *l_z, const captured_var *m, const
  captured_var *p, const captured_var *q, const captured_var *r, const
  captured_var *rho, const captured_var *wing_chord, const double u_in[15],
  double gradient[15]);
static double c_compute_cost_and_gradient_sec(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *Phi, const captured_var *S,
  const captured_var *Theta, const captured_var *V, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_motor_value, const captured_var *desired_ailerons_value,
  const captured_var *flight_path_angle, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *gain_motor, const captured_var
  *gain_ailerons, const captured_var *gamma_quadratic_du, const captured_var
  *l_1, const captured_var *l_2, const captured_var *l_3, const captured_var
  *l_4, const captured_var *l_z, const captured_var *m, const captured_var *p,
  const captured_var *q, const captured_var *r, const captured_var *rho, const
  captured_var *wing_chord, const double u_in[13], double gradient[13]);
static void c_driver(const double lb[15], const double ub[15], l_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, h_struct_T
                     *memspace, m_struct_T *WorkingSet, double Hessian[225],
                     f_struct_T *QRManager, g_struct_T *CholManager, struct_T
                     *QPObjective);
static double c_maxConstraintViolation(const m_struct_T *obj, const double x[16]);
static bool c_test_exit(b_struct_T *MeritFunction, const o_struct_T *WorkingSet,
  n_struct_T *TrialState, const double lb[13], const double ub[13], bool
  *Flags_fevalOK, bool *Flags_done, bool *Flags_stepAccepted, bool
  *Flags_failedLineSearch, int *Flags_stepType);
static void c_xgemm(int m, int n, int k, const double A[169], int lda, const
                    double B[729], int ib0, double C[378]);
static void c_xgemv(int m, int n, const double A[961], const double x[16],
                    double y[496]);
static double c_xnrm2(int n, const double x[729], int ix0);
static double computeComplError(const double xCurrent[15], const int finiteLB[16],
  int mLB, const double lb[15], const int finiteUB[16], int mUB, const double
  ub[15], const double lambda[31], int iL0);
static double computeFval(const struct_T *obj, double workspace[496], const
  double H[225], const double f[16], const double x[16]);
static double computeFval_ReuseHx(const struct_T *obj, double workspace[496],
  const double f[16], const double x[16]);
static void computeGradLag(double workspace[16], int nVar, const double grad[16],
  const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB, const
  int finiteUB[16], int mUB, const double lambda[31]);
static void computeGrad_StoreHx(struct_T *obj, const double H[225], const double
  f[16], const double x[16]);
static void computeQ_(f_struct_T *obj, int nrows);
static void compute_deltax(const double H[225], l_struct_T *solution, h_struct_T
  *memspace, const f_struct_T *qrmanager, g_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef);
static void countsort(int x[31], int xLen, int workspace[31], int xMin, int xMax);
static void d_computeGradLag(double workspace[378], int nVar, const double grad
  [14], const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB,
  const int finiteUB[14], int mUB, const double lambda[27]);
static double d_compute_cost_and_gradient_fir(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *S, const captured_var *V, const
  captured_var *W_act_phi, const captured_var *W_act_theta, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_phi_value, const captured_var *desired_theta_value,
  const captured_var *desired_motor_value, const captured_var
  *desired_ailerons_value, const captured_var *flight_path_angle, const
  captured_var *gain_el, const captured_var *gain_az, const captured_var
  *gain_phi, const captured_var *gain_theta, const captured_var *gain_motor,
  const captured_var *gain_ailerons, const captured_var *gamma_quadratic_du,
  const captured_var *l_1, const captured_var *l_2, const captured_var *l_3,
  const captured_var *l_4, const captured_var *l_z, const captured_var *m, const
  captured_var *p, const captured_var *q, const captured_var *r, const
  captured_var *rho, const captured_var *wing_chord, const double u_in[15]);
static double d_compute_cost_and_gradient_sec(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *Phi, const captured_var *S,
  const captured_var *Theta, const captured_var *V, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_motor_value, const captured_var *desired_ailerons_value,
  const captured_var *flight_path_angle, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *gain_motor, const captured_var
  *gain_ailerons, const captured_var *gamma_quadratic_du, const captured_var
  *l_1, const captured_var *l_2, const captured_var *l_3, const captured_var
  *l_4, const captured_var *l_z, const captured_var *m, const captured_var *p,
  const captured_var *q, const captured_var *r, const captured_var *rho, const
  captured_var *wing_chord, const double u_in[13]);
static void d_driver(const double lb[13], const double ub[13], n_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     r_coder_internal_stickyStruct *FcnEvaluator, k_struct_T
                     *memspace, o_struct_T *WorkingSet, double Hessian[169],
                     i_struct_T *QRManager, j_struct_T *CholManager, c_struct_T *
                     QPObjective);
static double d_maxConstraintViolation(const o_struct_T *obj, const double x[14]);
static void d_test_exit(q_struct_T *Flags, k_struct_T *memspace, b_struct_T
  *MeritFunction, const o_struct_T *WorkingSet, n_struct_T *TrialState,
  i_struct_T *QRManager, const double lb[13], const double ub[13]);
static void d_xgemm(int m, int n, int k, const double A[729], int ia0, const
                    double B[378], double C[729]);
static void d_xgemv(int m, int n, const double A[729], const double x[14],
                    double y[378]);
static double d_xnrm2(int n, const double x[14]);
static void deleteColMoveEnd(f_struct_T *obj, int idx);
static int div_nde_s32_floor(int numerator, int denominator);
static void driver(const double H[225], const double f[16], l_struct_T *solution,
                   h_struct_T *memspace, m_struct_T *workingset, f_struct_T
                   *qrmanager, g_struct_T *cholmanager, struct_T *objective,
                   p_struct_T *options, int runTimeOptions_MaxIterations);
static void factorQR(f_struct_T *obj, const double A[496], int mrows, int ncols);
static void factoryConstruct(double fval, b_struct_T *obj);
static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const m_struct_T *workingset, f_struct_T *qrmanager);
static double feasibleratiotest(const double solution_xstar[16], const double
  solution_searchDir[16], int workingset_nVar, const double workingset_lb[16],
  const double workingset_ub[16], const int workingset_indexLB[16], const int
  workingset_indexUB[16], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[31], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx);
static double fmincon(d_struct_T *fun_workspace, double x0[15], const double lb
                      [15], const double ub[15], double *exitflag, double
                      *output_iterations, double *output_funcCount, char
                      output_algorithm[3], double *output_constrviolation,
                      double *output_stepsize, double *output_lssteplength,
                      double *output_firstorderopt);
static void fullColLDL2_(g_struct_T *obj, int NColsRemain);
static void iterate(const double H[225], const double f[16], l_struct_T
                    *solution, h_struct_T *memspace, m_struct_T *workingset,
                    f_struct_T *qrmanager, g_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations);
static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[496],
  const double H[225], const double f[16], const double x[16]);
static double maxConstraintViolation(const m_struct_T *obj, const double x[496],
  int ix0);
static double maximum(const double x[2]);
static double mean(const double x[4]);
static double minimum(const double x[2]);
static void qrf(double A[961], int m, int n, int nfxd, double tau[31]);
static void removeConstr(m_struct_T *obj, int idx_global);
static double rt_hypotd(double u0, double u1);
static void setProblemType(m_struct_T *obj, int PROBLEM_TYPE);
static void solve(const g_struct_T *obj, double rhs[16]);
static void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[31], const int WorkingSet_Wlocalidx[31], double workspace[496]);
static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], l_struct_T *TrialState, b_struct_T
                 *MeritFunction, h_struct_T *memspace, m_struct_T *WorkingSet,
                 f_struct_T *QRManager, g_struct_T *CholManager, struct_T
                 *QPObjective, p_struct_T *qpoptions);
static bool test_exit(b_struct_T *MeritFunction, const m_struct_T *WorkingSet,
                      l_struct_T *TrialState, const double lb[15], const double
                      ub[15], bool *Flags_fevalOK, bool *Flags_done, bool
                      *Flags_stepAccepted, bool *Flags_failedLineSearch, int
                      *Flags_stepType);
static void tic(void);
static void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec);
static void timeKeeper_init(void);
static double toc(void);
static void xgemm(int m, int n, int k, const double A[225], int lda, const
                  double B[961], int ib0, double C[496]);
static void xgemv(int m, int n, const double A[225], int lda, const double x[16],
                  double y[15]);
static void xgeqp3(double A[961], int m, int n, int jpvt[31], double tau[31]);
static double xnrm2(int n, const double x[961], int ix0);
static int xpotrf(int n, double A[961]);
static double xrotg(double *a, double *b, double *s);
static void xzlarf(int m, int n, int iv0, double tau, double C[961], int ic0,
                   double work[31]);
static double xzlarfg(int n, double *alpha1, double x[961], int ix0);

/* Function Definitions */
/*
 * Arguments    : int nvar
 *                double Bk[225]
 *                const double sk[16]
 *                double yk[16]
 *                double workspace[496]
 * Return Type  : bool
 */
static bool BFGSUpdate(int nvar, double Bk[225], const double sk[16], double yk
  [16], double workspace[496])
{
  double curvatureS;
  double dotSY;
  double theta;
  int i;
  int i1;
  int i2;
  int ia;
  int iac;
  int ix;
  int k;
  bool success;
  dotSY = 0.0;
  i = (unsigned char)nvar;
  for (k = 0; k < i; k++) {
    dotSY += sk[k] * yk[k];
    workspace[k] = 0.0;
  }

  ix = 0;
  i1 = 15 * (nvar - 1) + 1;
  for (iac = 1; iac <= i1; iac += 15) {
    i2 = (iac + nvar) - 1;
    for (ia = iac; ia <= i2; ia++) {
      k = ia - iac;
      workspace[k] += Bk[ia - 1] * sk[ix];
    }

    ix++;
  }

  curvatureS = 0.0;
  if (nvar >= 1) {
    for (k = 0; k < i; k++) {
      curvatureS += sk[k] * workspace[k];
    }
  }

  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    for (k = 0; k < i; k++) {
      yk[k] *= theta;
    }

    if (1.0 - theta != 0.0) {
      ix = nvar - 1;
      for (k = 0; k <= ix; k++) {
        yk[k] += (1.0 - theta) * workspace[k];
      }
    }

    dotSY = 0.0;
    for (k = 0; k < i; k++) {
      dotSY += sk[k] * yk[k];
    }
  }

  if ((curvatureS > 2.2204460492503131E-16) && (dotSY > 2.2204460492503131E-16))
  {
    success = true;
  } else {
    success = false;
  }

  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (curvatureS != 0.0) {
      ix = 0;
      for (k = 0; k < i; k++) {
        theta = workspace[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += workspace[(iac - ix) - 1] * theta;
          }
        }

        ix += 15;
      }
    }

    curvatureS = 1.0 / dotSY;
    if (curvatureS != 0.0) {
      ix = 0;
      for (k = 0; k < i; k++) {
        theta = yk[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += yk[(iac - ix) - 1] * theta;
          }
        }

        ix += 15;
      }
    }
  }

  return success;
}

/*
 * Arguments    : l_struct_T *solution
 *                h_struct_T *memspace
 *                m_struct_T *workingset
 *                f_struct_T *qrmanager
 * Return Type  : void
 */
static void PresolveWorkingSet(l_struct_T *solution, h_struct_T *memspace,
  m_struct_T *workingset, f_struct_T *qrmanager)
{
  double tol;
  int idxDiag;
  int idx_col;
  int ix;
  int ix0;
  int k;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  solution->state = 82;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    int i;
    int i1;
    int u0;
    i = (unsigned char)workingset->nVar;
    for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
      for (idx_col = 0; idx_col < i; idx_col++) {
        qrmanager->QR[ix + 31 * idx_col] = workingset->ATwset[idx_col + (ix << 4)];
      }
    }

    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }

    memset(&qrmanager->jpvt[0], 0, (unsigned int)i * sizeof(int));
    i1 = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i1 == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      idxDiag = workingset->nVar;
      if (mTotalWorkingEq_tmp_tmp <= idxDiag) {
        idxDiag = mTotalWorkingEq_tmp_tmp;
      }

      qrmanager->minRowCol = idxDiag;
      xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp, workingset->nVar,
             qrmanager->jpvt, qrmanager->tau);
    }

    tol = 100.0 * (double)workingset->nVar * 2.2204460492503131E-16;
    u0 = workingset->nVar;
    if (u0 > mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }

    idxDiag = u0 + 31 * (u0 - 1);
    while ((idxDiag > 0) && (fabs(qrmanager->QR[idxDiag - 1]) < tol)) {
      idxDiag -= 32;
      nDepInd++;
    }

    if (nDepInd > 0) {
      bool exitg1;
      computeQ_(qrmanager, qrmanager->mrows);
      idxDiag = 0;
      exitg1 = false;
      while ((!exitg1) && (idxDiag <= nDepInd - 1)) {
        double qtb;
        ix = 31 * ((mTotalWorkingEq_tmp_tmp - idxDiag) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[ix + k] * workingset->bwset[k];
        }

        if (fabs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idxDiag++;
        }
      }
    }

    if (nDepInd > 0) {
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = 31 * idx_col;
        ix0 = idx_col << 4;
        for (k = 0; k < i; k++) {
          qrmanager->QR[idxDiag + k] = workingset->ATwset[ix0 + k];
        }
      }

      for (idxDiag = 0; idxDiag < mWorkingFixed; idxDiag++) {
        qrmanager->jpvt[idxDiag] = 1;
      }

      i = workingset->nWConstr[0] + 1;
      if (i <= mTotalWorkingEq_tmp_tmp) {
        memset(&qrmanager->jpvt[i + -1], 0, (unsigned int)
               ((mTotalWorkingEq_tmp_tmp - i) + 1) * sizeof(int));
      }

      if (i1 == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = u0;
        xgeqp3(qrmanager->QR, workingset->nVar, mTotalWorkingEq_tmp_tmp,
               qrmanager->jpvt, qrmanager->tau);
      }

      for (idxDiag = 0; idxDiag < nDepInd; idxDiag++) {
        memspace->workspace_int[idxDiag] = qrmanager->jpvt
          [(mTotalWorkingEq_tmp_tmp - nDepInd) + idxDiag];
      }

      countsort(memspace->workspace_int, nDepInd, memspace->workspace_sort, 1,
                mTotalWorkingEq_tmp_tmp);
      for (idxDiag = nDepInd; idxDiag >= 1; idxDiag--) {
        i = memspace->workspace_int[idxDiag - 1];
        if (i <= mTotalWorkingEq_tmp_tmp) {
          if ((workingset->nActiveConstr == mTotalWorkingEq_tmp_tmp) || (i ==
               mTotalWorkingEq_tmp_tmp)) {
            workingset->mEqRemoved++;

            /* A check that is always false is detected at compile-time. Eliminating code that follows. */
          } else {
            workingset->mEqRemoved++;

            /* A check that is always false is detected at compile-time. Eliminating code that follows. */
          }
        }
      }
    }
  }

  if ((nDepInd != -1) && (workingset->nActiveConstr <= 31)) {
    bool guard1;
    bool okWorkingSet;
    RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
        solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = c_maxConstraintViolation(workingset, solution->xstar);
      if (tol > 1.0E-6) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    idxDiag = mTotalWorkingEq_tmp_tmp + 1;
    ix = workingset->nActiveConstr;
    for (ix0 = idxDiag; ix0 <= ix; ix0++) {
      workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[ix0 -
        1] - 1] + workingset->Wlocalidx[ix0 - 1]) - 2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = mTotalWorkingEq_tmp_tmp;
  }
}

/*
 * Arguments    : m_struct_T *workingset
 *                f_struct_T *qrmanager
 *                h_struct_T *memspace
 *                double tolfactor
 * Return Type  : void
 */
static void RemoveDependentIneq_(m_struct_T *workingset, f_struct_T *qrmanager,
  h_struct_T *memspace, double tolfactor)
{
  int idx;
  int idx_col;
  int k;
  int nActiveConstr_tmp;
  int nFixedConstr;
  int nVar;
  nActiveConstr_tmp = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    double tol;
    int idxDiag;
    int nDepIneq;
    tol = tolfactor * (double)workingset->nVar * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt[idx] = 1;
    }

    idxDiag = nFixedConstr + 1;
    if (idxDiag <= nActiveConstr_tmp) {
      memset(&qrmanager->jpvt[idxDiag + -1], 0, (unsigned int)
             ((nActiveConstr_tmp - idxDiag) + 1) * sizeof(int));
    }

    for (idx_col = 0; idx_col < nActiveConstr_tmp; idx_col++) {
      nDepIneq = 31 * idx_col;
      idx = idx_col << 4;
      idxDiag = (unsigned char)nVar;
      for (k = 0; k < idxDiag; k++) {
        qrmanager->QR[nDepIneq + k] = workingset->ATwset[idx + k];
      }
    }

    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      idxDiag = workingset->nVar;
      nDepIneq = workingset->nActiveConstr;
      if (idxDiag <= nDepIneq) {
        nDepIneq = idxDiag;
      }

      qrmanager->minRowCol = nDepIneq;
      xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
             qrmanager->jpvt, qrmanager->tau);
    }

    nDepIneq = 0;
    for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar; idx--) {
      nDepIneq++;
      memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
    }

    if (idx + 1 <= workingset->nVar) {
      idxDiag = idx + 31 * idx;
      while ((idx + 1 > nFixedConstr) && (fabs(qrmanager->QR[idxDiag]) < tol)) {
        nDepIneq++;
        memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
        idx--;
        idxDiag -= 32;
      }
    }

    countsort(memspace->workspace_int, nDepIneq, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      removeConstr(workingset, memspace->workspace_int[idx - 1]);
    }
  }
}

/*
 * Arguments    : m_struct_T *obj
 *                int TYPE
 *                int idx_local
 * Return Type  : void
 */
static void addBoundToActiveSetMatrix_(m_struct_T *obj, int TYPE, int idx_local)
{
  int colOffset;
  int i;
  int idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  i = obj->nActiveConstr - 1;
  obj->Wid[i] = TYPE;
  obj->Wlocalidx[i] = idx_local;
  colOffset = (i << 4) - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB[idx_local - 1];
    obj->bwset[i] = obj->ub[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[i] = obj->lb[idx_bnd_local - 1];
  }

  if (idx_bnd_local - 2 >= 0) {
    memset(&obj->ATwset[colOffset + 1], 0, (unsigned int)(((idx_bnd_local +
              colOffset) - colOffset) - 1) * sizeof(double));
  }

  obj->ATwset[idx_bnd_local + colOffset] = 2.0 * (double)(TYPE == 5) - 1.0;
  i = idx_bnd_local + 1;
  idx_bnd_local = obj->nVar;
  if (i <= idx_bnd_local) {
    memset(&obj->ATwset[i + colOffset], 0, (unsigned int)((((idx_bnd_local +
               colOffset) - i) - colOffset) + 1) * sizeof(double));
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/*
 * Arguments    : int nvar
 *                double Bk[169]
 *                const double sk[14]
 *                double yk[14]
 *                double workspace[378]
 * Return Type  : bool
 */
static bool b_BFGSUpdate(int nvar, double Bk[169], const double sk[14], double
  yk[14], double workspace[378])
{
  double curvatureS;
  double dotSY;
  double theta;
  int i;
  int i1;
  int i2;
  int ia;
  int iac;
  int ix;
  int k;
  bool success;
  dotSY = 0.0;
  i = (unsigned short)nvar;
  for (k = 0; k < i; k++) {
    dotSY += sk[k] * yk[k];
    workspace[k] = 0.0;
  }

  ix = 0;
  i1 = 13 * (nvar - 1) + 1;
  for (iac = 1; iac <= i1; iac += 13) {
    i2 = (iac + nvar) - 1;
    for (ia = iac; ia <= i2; ia++) {
      k = ia - iac;
      workspace[k] += Bk[ia - 1] * sk[ix];
    }

    ix++;
  }

  curvatureS = 0.0;
  if (nvar >= 1) {
    i1 = (unsigned char)nvar;
    for (k = 0; k < i1; k++) {
      curvatureS += sk[k] * workspace[k];
    }
  }

  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    for (k = 0; k < i; k++) {
      yk[k] *= theta;
    }

    if (1.0 - theta != 0.0) {
      ix = nvar - 1;
      for (k = 0; k <= ix; k++) {
        yk[k] += (1.0 - theta) * workspace[k];
      }
    }

    dotSY = 0.0;
    for (k = 0; k < i; k++) {
      dotSY += sk[k] * yk[k];
    }
  }

  if ((curvatureS > 2.2204460492503131E-16) && (dotSY > 2.2204460492503131E-16))
  {
    success = true;
  } else {
    success = false;
  }

  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (curvatureS != 0.0) {
      ix = 0;
      i = (unsigned char)nvar;
      for (k = 0; k < i; k++) {
        theta = workspace[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += workspace[(iac - ix) - 1] * theta;
          }
        }

        ix += 13;
      }
    }

    curvatureS = 1.0 / dotSY;
    if (curvatureS != 0.0) {
      ix = 0;
      i = (unsigned char)nvar;
      for (k = 0; k < i; k++) {
        theta = yk[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          for (iac = i1; iac <= i2; iac++) {
            Bk[iac - 1] += yk[(iac - ix) - 1] * theta;
          }
        }

        ix += 13;
      }
    }
  }

  return success;
}

/*
 * Arguments    : n_struct_T *solution
 *                k_struct_T *memspace
 *                o_struct_T *workingset
 *                i_struct_T *qrmanager
 * Return Type  : void
 */
static void b_PresolveWorkingSet(n_struct_T *solution, k_struct_T *memspace,
  o_struct_T *workingset, i_struct_T *qrmanager)
{
  double tol;
  int idxDiag;
  int idx_col;
  int ix;
  int ix0;
  int k;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  solution->state = 82;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    int i;
    int i1;
    int u0;
    i = (unsigned char)workingset->nVar;
    for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
      for (idx_col = 0; idx_col < i; idx_col++) {
        qrmanager->QR[ix + 27 * idx_col] = workingset->ATwset[idx_col + 14 * ix];
      }
    }

    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }

    memset(&qrmanager->jpvt[0], 0, (unsigned int)i * sizeof(int));
    i1 = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i1 == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      idxDiag = workingset->nVar;
      if (mTotalWorkingEq_tmp_tmp <= idxDiag) {
        idxDiag = mTotalWorkingEq_tmp_tmp;
      }

      qrmanager->minRowCol = idxDiag;
      b_xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp, workingset->nVar,
               qrmanager->jpvt, qrmanager->tau);
    }

    tol = 100.0 * (double)workingset->nVar * 2.2204460492503131E-16;
    u0 = workingset->nVar;
    if (u0 > mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }

    idxDiag = u0 + 27 * (u0 - 1);
    while ((idxDiag > 0) && (fabs(qrmanager->QR[idxDiag - 1]) < tol)) {
      idxDiag -= 28;
      nDepInd++;
    }

    if (nDepInd > 0) {
      bool exitg1;
      b_computeQ_(qrmanager, qrmanager->mrows);
      idxDiag = 0;
      exitg1 = false;
      while ((!exitg1) && (idxDiag <= nDepInd - 1)) {
        double qtb;
        ix = 27 * ((mTotalWorkingEq_tmp_tmp - idxDiag) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[ix + k] * workingset->bwset[k];
        }

        if (fabs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idxDiag++;
        }
      }
    }

    if (nDepInd > 0) {
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = 27 * idx_col;
        ix0 = 14 * idx_col;
        for (k = 0; k < i; k++) {
          qrmanager->QR[idxDiag + k] = workingset->ATwset[ix0 + k];
        }
      }

      for (idxDiag = 0; idxDiag < mWorkingFixed; idxDiag++) {
        qrmanager->jpvt[idxDiag] = 1;
      }

      i = workingset->nWConstr[0] + 1;
      if (i <= mTotalWorkingEq_tmp_tmp) {
        memset(&qrmanager->jpvt[i + -1], 0, (unsigned int)
               ((mTotalWorkingEq_tmp_tmp - i) + 1) * sizeof(int));
      }

      if (i1 == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = u0;
        b_xgeqp3(qrmanager->QR, workingset->nVar, mTotalWorkingEq_tmp_tmp,
                 qrmanager->jpvt, qrmanager->tau);
      }

      for (idxDiag = 0; idxDiag < nDepInd; idxDiag++) {
        memspace->workspace_int[idxDiag] = qrmanager->jpvt
          [(mTotalWorkingEq_tmp_tmp - nDepInd) + idxDiag];
      }

      b_countsort(memspace->workspace_int, nDepInd, memspace->workspace_sort, 1,
                  mTotalWorkingEq_tmp_tmp);
      for (idxDiag = nDepInd; idxDiag >= 1; idxDiag--) {
        i = memspace->workspace_int[idxDiag - 1];
        if (i <= mTotalWorkingEq_tmp_tmp) {
          if ((workingset->nActiveConstr == mTotalWorkingEq_tmp_tmp) || (i ==
               mTotalWorkingEq_tmp_tmp)) {
            workingset->mEqRemoved++;

            /* A check that is always false is detected at compile-time. Eliminating code that follows. */
          } else {
            workingset->mEqRemoved++;

            /* A check that is always false is detected at compile-time. Eliminating code that follows. */
          }
        }
      }
    }
  }

  if ((nDepInd != -1) && (workingset->nActiveConstr <= 27)) {
    bool guard1;
    bool okWorkingSet;
    b_RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    okWorkingSet = b_feasibleX0ForWorkingSet(memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      b_RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = b_feasibleX0ForWorkingSet(memspace->workspace_double,
        solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = d_maxConstraintViolation(workingset, solution->xstar);
      if (tol > 1.0E-6) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    idxDiag = mTotalWorkingEq_tmp_tmp + 1;
    ix = workingset->nActiveConstr;
    for (ix0 = idxDiag; ix0 <= ix; ix0++) {
      workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[ix0 -
        1] - 1] + workingset->Wlocalidx[ix0 - 1]) - 2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = mTotalWorkingEq_tmp_tmp;
  }
}

/*
 * Arguments    : o_struct_T *workingset
 *                i_struct_T *qrmanager
 *                k_struct_T *memspace
 *                double tolfactor
 * Return Type  : void
 */
static void b_RemoveDependentIneq_(o_struct_T *workingset, i_struct_T *qrmanager,
  k_struct_T *memspace, double tolfactor)
{
  int idx;
  int idx_col;
  int k;
  int nActiveConstr_tmp;
  int nFixedConstr;
  int nVar;
  nActiveConstr_tmp = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    double tol;
    int idxDiag;
    int nDepIneq;
    tol = tolfactor * (double)workingset->nVar * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt[idx] = 1;
    }

    idxDiag = nFixedConstr + 1;
    if (idxDiag <= nActiveConstr_tmp) {
      memset(&qrmanager->jpvt[idxDiag + -1], 0, (unsigned int)
             ((nActiveConstr_tmp - idxDiag) + 1) * sizeof(int));
    }

    for (idx_col = 0; idx_col < nActiveConstr_tmp; idx_col++) {
      nDepIneq = 27 * idx_col;
      idx = 14 * idx_col;
      idxDiag = (unsigned char)nVar;
      for (k = 0; k < idxDiag; k++) {
        qrmanager->QR[nDepIneq + k] = workingset->ATwset[idx + k];
      }
    }

    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      idxDiag = workingset->nVar;
      nDepIneq = workingset->nActiveConstr;
      if (idxDiag <= nDepIneq) {
        nDepIneq = idxDiag;
      }

      qrmanager->minRowCol = nDepIneq;
      b_xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
               qrmanager->jpvt, qrmanager->tau);
    }

    nDepIneq = 0;
    for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar; idx--) {
      nDepIneq++;
      memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
    }

    if (idx + 1 <= workingset->nVar) {
      idxDiag = idx + 27 * idx;
      while ((idx + 1 > nFixedConstr) && (fabs(qrmanager->QR[idxDiag]) < tol)) {
        nDepIneq++;
        memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
        idx--;
        idxDiag -= 28;
      }
    }

    b_countsort(memspace->workspace_int, nDepIneq, memspace->workspace_sort,
                nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      b_removeConstr(workingset, memspace->workspace_int[idx - 1]);
    }
  }
}

/*
 * Arguments    : o_struct_T *obj
 *                int TYPE
 *                int idx_local
 * Return Type  : void
 */
static void b_addBoundToActiveSetMatrix_(o_struct_T *obj, int TYPE, int
  idx_local)
{
  int colOffset;
  int i;
  int idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  i = obj->nActiveConstr - 1;
  obj->Wid[i] = TYPE;
  obj->Wlocalidx[i] = idx_local;
  colOffset = 14 * i - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB[idx_local - 1];
    obj->bwset[i] = obj->ub[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[i] = obj->lb[idx_bnd_local - 1];
  }

  if (idx_bnd_local - 2 >= 0) {
    memset(&obj->ATwset[colOffset + 1], 0, (unsigned int)(((idx_bnd_local +
              colOffset) - colOffset) - 1) * sizeof(double));
  }

  obj->ATwset[idx_bnd_local + colOffset] = 2.0 * (double)(TYPE == 5) - 1.0;
  i = idx_bnd_local + 1;
  idx_bnd_local = obj->nVar;
  if (i <= idx_bnd_local) {
    memset(&obj->ATwset[i + colOffset], 0, (unsigned int)((((idx_bnd_local +
               colOffset) - i) - colOffset) + 1) * sizeof(double));
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/*
 * Arguments    : const double xCurrent[13]
 *                const int finiteLB[14]
 *                int mLB
 *                const double lb[13]
 *                const int finiteUB[14]
 *                int mUB
 *                const double ub[13]
 *                const double lambda[27]
 *                int iL0
 * Return Type  : double
 */
static double b_computeComplError(const double xCurrent[13], const int finiteLB
  [14], int mLB, const double lb[13], const int finiteUB[14], int mUB, const
  double ub[13], const double lambda[27], int iL0)
{
  double nlpComplError;
  int idx;
  nlpComplError = 0.0;
  if (mLB + mUB > 0) {
    double lbDelta;
    double lbLambda;
    int i;
    int i1;
    int ubOffset;
    ubOffset = (iL0 + mLB) - 1;
    i = (unsigned char)mLB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteLB[idx];
      lbDelta = xCurrent[i1 - 1] - lb[i1 - 1];
      lbLambda = lambda[(iL0 + idx) - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }

    i = (unsigned char)mUB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteUB[idx];
      lbDelta = ub[i1 - 1] - xCurrent[i1 - 1];
      lbLambda = lambda[ubOffset + idx];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }
  }

  return nlpComplError;
}

/*
 * Arguments    : const c_struct_T *obj
 *                double workspace[378]
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : double
 */
static double b_computeFval(const c_struct_T *obj, double workspace[378], const
  double H[169], const double f[14], const double x[14])
{
  double val;
  int idx;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      b_linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
      val = 0.0;
      if (obj->nvar >= 1) {
        int i;
        i = (unsigned char)obj->nvar;
        for (idx = 0; idx < i; idx++) {
          val += x[idx] * workspace[idx];
        }
      }
    }
    break;

   default:
    {
      int i;
      b_linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
      i = obj->nvar + 1;
      for (idx = i; idx < 14; idx++) {
        workspace[idx - 1] = 0.5 * obj->beta * x[idx - 1] + obj->rho;
      }

      val = 0.0;
      for (idx = 0; idx < 13; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
    break;
  }

  return val;
}

/*
 * Arguments    : const c_struct_T *obj
 *                double workspace[378]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : double
 */
static double b_computeFval_ReuseHx(const c_struct_T *obj, double workspace[378],
  const double f[14], const double x[14])
{
  double val;
  int k;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int i;
        i = (unsigned char)obj->nvar;
        for (k = 0; k < i; k++) {
          workspace[k] = 0.5 * obj->Hx[k] + f[k];
        }

        val = 0.0;
        if (obj->nvar >= 1) {
          for (k = 0; k < i; k++) {
            val += x[k] * workspace[k];
          }
        }
      } else {
        val = 0.0;
        if (obj->nvar >= 1) {
          int i;
          i = (unsigned short)obj->nvar;
          for (k = 0; k < i; k++) {
            val += x[k] * obj->Hx[k];
          }
        }

        val *= 0.5;
      }
    }
    break;

   default:
    {
      if (obj->hasLinear) {
        int i;
        i = (unsigned char)obj->nvar;
        if (i - 1 >= 0) {
          memcpy(&workspace[0], &f[0], (unsigned int)i * sizeof(double));
        }

        i = 12 - obj->nvar;
        for (k = 0; k <= i; k++) {
          workspace[obj->nvar + k] = obj->rho;
        }

        val = 0.0;
        for (k = 0; k < 13; k++) {
          double d;
          d = workspace[k] + 0.5 * obj->Hx[k];
          workspace[k] = d;
          val += x[k] * d;
        }
      } else {
        int i;
        val = 0.0;
        for (k = 0; k < 13; k++) {
          val += x[k] * obj->Hx[k];
        }

        val *= 0.5;
        i = obj->nvar + 1;
        for (k = i; k < 14; k++) {
          val += x[k - 1] * obj->rho;
        }
      }
    }
    break;
  }

  return val;
}

/*
 * Arguments    : double workspace[496]
 *                int nVar
 *                const double grad[16]
 *                const int finiteFixed[16]
 *                int mFixed
 *                const int finiteLB[16]
 *                int mLB
 *                const int finiteUB[16]
 *                int mUB
 *                const double lambda[31]
 * Return Type  : void
 */
static void b_computeGradLag(double workspace[496], int nVar, const double grad
  [16], const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB,
  const int finiteUB[16], int mUB, const double lambda[31])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }

  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }

  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }

  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : c_struct_T *obj
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : void
 */
static void b_computeGrad_StoreHx(c_struct_T *obj, const double H[169], const
  double f[14], const double x[14])
{
  int ixlast;
  int k;
  switch (obj->objtype) {
   case 5:
    {
      int i;
      i = obj->nvar;
      if (i - 2 >= 0) {
        memset(&obj->grad[0], 0, (unsigned int)(i - 1) * sizeof(double));
      }

      obj->grad[obj->nvar - 1] = obj->gammaScalar;
    }
    break;

   case 3:
    {
      int i;
      b_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      i = (unsigned char)obj->nvar;
      if (i - 1 >= 0) {
        memcpy(&obj->grad[0], &obj->Hx[0], (unsigned int)i * sizeof(double));
      }

      if (obj->hasLinear && (obj->nvar >= 1)) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          obj->grad[k] += f[k];
        }
      }
    }
    break;

   default:
    {
      int i;
      b_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      i = obj->nvar + 1;
      for (ixlast = i; ixlast < 14; ixlast++) {
        obj->Hx[ixlast - 1] = obj->beta * x[ixlast - 1];
      }

      memcpy(&obj->grad[0], &obj->Hx[0], 13U * sizeof(double));
      if (obj->hasLinear && (obj->nvar >= 1)) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          obj->grad[k] += f[k];
        }
      }

      if (13 - obj->nvar >= 1) {
        ixlast = obj->nvar;
        i = 12 - obj->nvar;
        for (k = 0; k <= i; k++) {
          int i1;
          i1 = ixlast + k;
          obj->grad[i1] += obj->rho;
        }
      }
    }
    break;
  }
}

/*
 * Arguments    : i_struct_T *obj
 *                int nrows
 * Return Type  : void
 */
static void b_computeQ_(i_struct_T *obj, int nrows)
{
  double work[27];
  int b_i;
  int iQR0;
  int ia;
  int idx;
  int lastc;
  int m;
  int n;
  lastc = obj->minRowCol;
  for (idx = 0; idx < lastc; idx++) {
    iQR0 = 27 * idx + idx;
    n = obj->mrows - idx;
    if (n - 2 >= 0) {
      memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1], (unsigned int)(((n + iQR0) -
               iQR0) - 1) * sizeof(double));
    }
  }

  m = obj->mrows;
  if (nrows >= 1) {
    int i;
    int i1;
    int itau;
    i = nrows - 1;
    for (idx = lastc; idx <= i; idx++) {
      ia = idx * 27;
      i1 = m - 1;
      memset(&obj->Q[ia], 0, (unsigned int)(((i1 + ia) - ia) + 1) * sizeof
             (double));
      obj->Q[ia + idx] = 1.0;
    }

    itau = obj->minRowCol - 1;
    memset(&work[0], 0, 27U * sizeof(double));
    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * 27;
      if (b_i < nrows) {
        int lastv;
        obj->Q[iaii - 1] = 1.0;
        idx = iaii + 27;
        if (obj->tau[itau] != 0.0) {
          bool exitg2;
          lastv = m - b_i;
          iQR0 = (iaii + m) - b_i;
          while ((lastv + 1 > 0) && (obj->Q[iQR0 - 1] == 0.0)) {
            lastv--;
            iQR0--;
          }

          lastc = (nrows - b_i) - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int exitg1;
            iQR0 = (iaii + lastc * 27) + 27;
            ia = iQR0;
            do {
              exitg1 = 0;
              if (ia <= iQR0 + lastv) {
                if (obj->Q[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = -1;
        }

        if (lastv + 1 > 0) {
          double c;
          if (lastc + 1 != 0) {
            if (lastc >= 0) {
              memset(&work[0], 0, (unsigned int)(lastc + 1) * sizeof(double));
            }

            i = (iaii + 27 * lastc) + 27;
            for (n = idx; n <= i; n += 27) {
              c = 0.0;
              i1 = n + lastv;
              for (ia = n; ia <= i1; ia++) {
                c += obj->Q[ia - 1] * obj->Q[((iaii + ia) - n) - 1];
              }

              iQR0 = div_nde_s32_floor((n - iaii) - 27, 27);
              work[iQR0] += c;
            }
          }

          if (-obj->tau[itau] != 0.0) {
            iQR0 = iaii;
            for (idx = 0; idx <= lastc; idx++) {
              c = work[idx];
              if (c != 0.0) {
                c *= -obj->tau[itau];
                i = iQR0 + 27;
                i1 = lastv + iQR0;
                for (n = i; n <= i1 + 27; n++) {
                  obj->Q[n - 1] += obj->Q[((iaii + n) - iQR0) - 28] * c;
                }
              }

              iQR0 += 27;
            }
          }
        }
      }

      if (b_i < m) {
        iQR0 = iaii + 1;
        i = (iaii + m) - b_i;
        for (lastc = iQR0; lastc <= i; lastc++) {
          obj->Q[lastc - 1] *= -obj->tau[itau];
        }
      }

      obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
      i = (unsigned char)(b_i - 1);
      for (idx = 0; idx < i; idx++) {
        obj->Q[(iaii - idx) - 2] = 0.0;
      }

      itau--;
    }
  }
}

/*
 * Arguments    : const double H[169]
 *                n_struct_T *solution
 *                k_struct_T *memspace
 *                const i_struct_T *qrmanager
 *                j_struct_T *cholmanager
 *                const c_struct_T *objective
 *                bool alwaysPositiveDef
 * Return Type  : void
 */
static void b_compute_deltax(const double H[169], n_struct_T *solution,
  k_struct_T *memspace, const i_struct_T *qrmanager, j_struct_T *cholmanager,
  const c_struct_T *objective, bool alwaysPositiveDef)
{
  int b_i;
  int idx;
  int ix;
  int jA;
  int jjA;
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) * sizeof
             (double));
    }
  } else {
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = -objective->grad[idx];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        {
          double smax;
          int nVars;
          if (alwaysPositiveDef) {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              jjA = (nVar_tmp + 1) * idx;
              jA = 27 * idx;
              for (ix = 0; ix <= nVar_tmp; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            cholmanager->info = b_xpotrf(qrmanager->mrows, cholmanager->FMat);
          } else {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              jjA = qrmanager->mrows * idx;
              jA = 27 * idx;
              for (ix = 0; ix <= nVar_tmp; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            if (qrmanager->mrows < 1) {
              nVars = -1;
            } else {
              nVars = 0;
              if (qrmanager->mrows > 1) {
                smax = fabs(cholmanager->FMat[0]);
                for (ix = 2; ix <= nVar_tmp + 1; ix++) {
                  double s;
                  s = fabs(cholmanager->FMat[(ix - 1) * 28]);
                  if (s > smax) {
                    nVars = ix - 1;
                    smax = s;
                  }
                }
              }
            }

            cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 27 *
              nVars]) * 2.2204460492503131E-16, 0.0);
            b_fullColLDL2_(cholmanager, qrmanager->mrows);
            if (cholmanager->ConvexCheck) {
              idx = 0;
              int exitg1;
              do {
                exitg1 = 0;
                if (idx <= nVar_tmp) {
                  if (cholmanager->FMat[idx + 27 * idx] <= 0.0) {
                    cholmanager->info = -idx - 1;
                    exitg1 = 1;
                  } else {
                    idx++;
                  }
                } else {
                  cholmanager->ConvexCheck = false;
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
            }
          }

          if (cholmanager->info != 0) {
            solution->state = -6;
          } else if (alwaysPositiveDef) {
            b_solve(cholmanager, solution->searchDir);
          } else {
            int i;
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 27;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  solution->searchDir[ix] -= solution->searchDir[idx] *
                    cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }

            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              solution->searchDir[idx] /= cholmanager->FMat[idx + 27 * idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 27;
                smax = solution->searchDir[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                    solution->searchDir[b_i - 1];
                }

                solution->searchDir[idx - 1] = smax;
              }
            }
          }
        }
        break;

       default:
        {
          if (alwaysPositiveDef) {
            int nVars;
            nVars = objective->nvar;
            cholmanager->ndims = objective->nvar;
            for (idx = 0; idx < nVars; idx++) {
              jjA = nVars * idx;
              jA = 27 * idx;
              for (ix = 0; ix < nVars; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            cholmanager->info = b_xpotrf(objective->nvar, cholmanager->FMat);
            if (cholmanager->info != 0) {
              solution->state = -6;
            } else {
              double smax;
              int i;
              b_solve(cholmanager, solution->searchDir);
              smax = 1.0 / objective->beta;
              jjA = objective->nvar + 1;
              i = qrmanager->mrows;
              for (ix = jjA; ix <= i; ix++) {
                solution->searchDir[ix - 1] *= smax;
              }
            }
          }
        }
        break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = 27 * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] = -qrmanager->Q[nVar_tmp + 27 *
            (qrmanager->ncols + idx)];
        }

        if (qrmanager->mrows != 0) {
          int i;
          memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) *
                 sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
          for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
            int nVars;
            nVars = jjA + nVar_tmp;
            for (idx = jjA; idx <= nVars; idx++) {
              jA = idx - jjA;
              solution->searchDir[jA] += qrmanager->Q[idx - 1] *
                memspace->workspace_double[ix];
            }

            ix++;
          }
        }
      } else {
        double smax;
        int i;
        int nVars;
        if (objective->objtype == 3) {
          c_xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                  qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp,
                  memspace->workspace_double);
          d_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager->mrows;
          c_xgemm(objective->nvar, mNull_tmp, objective->nvar, H,
                  objective->nvar, qrmanager->Q, nullStartIdx_tmp,
                  memspace->workspace_double);
          for (jA = 0; jA < mNull_tmp; jA++) {
            i = objective->nvar + 1;
            for (jjA = i; jjA <= nVars; jjA++) {
              memspace->workspace_double[(jjA + 27 * jA) - 1] = objective->beta *
                qrmanager->Q[(jjA + 27 * (jA + qrmanager->ncols)) - 1];
            }
          }

          d_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        }

        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          cholmanager->info = b_xpotrf(mNull_tmp, cholmanager->FMat);
        } else {
          cholmanager->ndims = mNull_tmp;
          nVars = 0;
          if (mNull_tmp > 1) {
            smax = fabs(cholmanager->FMat[0]);
            for (ix = 2; ix <= mNull_tmp; ix++) {
              double s;
              s = fabs(cholmanager->FMat[(ix - 1) * 28]);
              if (s > smax) {
                nVars = ix - 1;
                smax = s;
              }
            }
          }

          cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 27 * nVars])
            * 2.2204460492503131E-16, 0.0);
          b_fullColLDL2_(cholmanager, mNull_tmp);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager->FMat[idx + 27 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            memset(&memspace->workspace_double[0], 0, (unsigned int)mNull_tmp *
                   sizeof(double));
            i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
              smax = 0.0;
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                smax += qrmanager->Q[idx - 1] * objective->grad[idx - jjA];
              }

              nVars = div_nde_s32_floor(jjA - nullStartIdx_tmp, 27);
              memspace->workspace_double[nVars] -= smax;
            }
          }

          if (alwaysPositiveDef) {
            nVars = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx < nVars; idx++) {
                jA = idx * 27;
                smax = memspace->workspace_double[idx];
                for (b_i = 0; b_i < idx; b_i++) {
                  smax -= cholmanager->FMat[jA + b_i] *
                    memspace->workspace_double[b_i];
                }

                memspace->workspace_double[idx] = smax / cholmanager->FMat[jA +
                  idx];
              }
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jjA = (idx + (idx - 1) * 27) - 1;
                memspace->workspace_double[idx - 1] /= cholmanager->FMat[jjA];
                for (b_i = 0; b_i <= idx - 2; b_i++) {
                  ix = (idx - b_i) - 2;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx - 1] * cholmanager->FMat[(jjA - b_i) -
                    1];
                }
              }
            }
          } else {
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 27;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx] * cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }

            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              memspace->workspace_double[idx] /= cholmanager->FMat[idx + 27 *
                idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 27;
                smax = memspace->workspace_double[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                    memspace->workspace_double[b_i - 1];
                }

                memspace->workspace_double[idx - 1] = smax;
              }
            }
          }

          if (qrmanager->mrows != 0) {
            memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) *
                   sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + 27 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 27) {
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                jA = idx - jjA;
                solution->searchDir[jA] += qrmanager->Q[idx - 1] *
                  memspace->workspace_double[ix];
              }

              ix++;
            }
          }
        }
      }
    }
  }
}

/*
 * Arguments    : int x[27]
 *                int xLen
 *                int workspace[27]
 *                int xMin
 *                int xMax
 * Return Type  : void
 */
static void b_countsort(int x[27], int xLen, int workspace[27], int xMin, int
  xMax)
{
  int idx;
  int idxFill;
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    if (idxStart >= 0) {
      memset(&workspace[0], 0, (unsigned int)(idxStart + 1) * sizeof(int));
    }

    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart]++;
    }

    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] += workspace[idx - 2];
    }

    idxStart = 1;
    idxEnd = workspace[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }

      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }

    for (idx = idxStart; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

/*
 * Arguments    : i_struct_T *obj
 *                int idx
 * Return Type  : void
 */
static void b_deleteColMoveEnd(i_struct_T *obj, int idx)
{
  double s;
  double temp_tmp;
  int b_k;
  int i;
  int k;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    int b_i;
    int u0;
    b_i = obj->ncols - 1;
    obj->jpvt[idx - 1] = obj->jpvt[b_i];
    i = obj->minRowCol;
    for (k = 0; k < i; k++) {
      obj->QR[k + 27 * (idx - 1)] = obj->QR[k + 27 * b_i];
    }

    obj->ncols = b_i;
    u0 = obj->mrows;
    i = obj->ncols;
    if (u0 <= i) {
      i = u0;
    }

    obj->minRowCol = i;
    if (idx < obj->mrows) {
      double c;
      double temp;
      int QRk0;
      int b_temp_tmp;
      int endIdx;
      int n;
      u0 = obj->mrows - 1;
      endIdx = obj->ncols;
      if (u0 <= endIdx) {
        endIdx = u0;
      }

      k = endIdx;
      i = 27 * (idx - 1);
      while (k >= idx) {
        b_i = k + i;
        temp_tmp = obj->QR[b_i];
        c = xrotg(&obj->QR[b_i - 1], &temp_tmp, &s);
        obj->QR[b_i] = temp_tmp;
        b_i = 27 * (k - 1);
        obj->QR[k + b_i] = 0.0;
        QRk0 = k + 27 * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 27;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = b_i + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 27];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 27] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }

        k--;
      }

      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        u0 = 27 * (k - 1);
        i = k + u0;
        temp_tmp = obj->QR[i];
        c = xrotg(&obj->QR[i - 1], &temp_tmp, &s);
        obj->QR[i] = temp_tmp;
        QRk0 = k * 28;
        n = obj->ncols - k;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 27;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = u0 + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 27];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 27] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }
      }
    }
  }
}

/*
 * Arguments    : const double H[169]
 *                const double f[14]
 *                n_struct_T *solution
 *                k_struct_T *memspace
 *                o_struct_T *workingset
 *                i_struct_T *qrmanager
 *                j_struct_T *cholmanager
 *                c_struct_T *objective
 *                p_struct_T *options
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void b_driver(const double H[169], const double f[14], n_struct_T
                     *solution, k_struct_T *memspace, o_struct_T *workingset,
                     i_struct_T *qrmanager, j_struct_T *cholmanager, c_struct_T *
                     objective, p_struct_T *options, int
                     runTimeOptions_MaxIterations)
{
  int idxStartIneq;
  int idx_global;
  int mConstr;
  int nVar_tmp;
  bool guard1;
  solution->iterations = 0;
  nVar_tmp = workingset->nVar;
  guard1 = false;
  if (workingset->probType == 3) {
    mConstr = (unsigned char)workingset->sizes[0];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      solution->xstar[workingset->indexFixed[idxStartIneq] - 1] = workingset->
        ub[workingset->indexFixed[idxStartIneq] - 1];
    }

    mConstr = (unsigned char)workingset->sizes[3];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + idxStartIneq)
          - 1]) {
        solution->xstar[workingset->indexLB[idxStartIneq] - 1] = -workingset->
          lb[workingset->indexLB[idxStartIneq] - 1];
      }
    }

    mConstr = (unsigned char)workingset->sizes[4];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[4] + idxStartIneq)
          - 1]) {
        solution->xstar[workingset->indexUB[idxStartIneq] - 1] = workingset->
          ub[workingset->indexUB[idxStartIneq] - 1];
      }
    }

    b_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution->state >= 0) {
      guard1 = true;
    }
  } else {
    solution->state = 82;
    guard1 = true;
  }

  if (guard1) {
    solution->iterations = 0;
    solution->maxConstr = d_maxConstraintViolation(workingset, solution->xstar);
    if (solution->maxConstr > 1.0E-6) {
      int PROBTYPE_ORIG;
      int idxEndIneq_tmp_tmp;
      PROBTYPE_ORIG = workingset->probType;
      solution->xstar[13] = solution->maxConstr + 1.0;
      if (workingset->probType == 3) {
        mConstr = 1;
      } else {
        mConstr = 4;
      }

      b_setProblemType(workingset, mConstr);
      mConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
      idxStartIneq = mConstr + 1;
      idxEndIneq_tmp_tmp = workingset->nActiveConstr;
      for (idx_global = idxStartIneq; idx_global <= idxEndIneq_tmp_tmp;
           idx_global++) {
        workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
          Wid[idx_global - 1] - 1] + workingset->Wlocalidx[idx_global - 1]) - 2]
          = false;
      }

      workingset->nWConstr[2] = 0;
      workingset->nWConstr[3] = 0;
      workingset->nWConstr[4] = 0;
      workingset->nActiveConstr = mConstr;
      objective->prev_objtype = objective->objtype;
      objective->prev_nvar = objective->nvar;
      objective->prev_hasLinear = objective->hasLinear;
      objective->objtype = 5;
      objective->nvar = 14;
      objective->gammaScalar = 1.0;
      objective->hasLinear = true;
      solution->fstar = b_computeFval(objective, memspace->workspace_double, H,
        f, solution->xstar);
      solution->state = 5;
      b_iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                objective, options->SolverName, 1.4901161193847657E-10, 1.0E-6,
                runTimeOptions_MaxIterations);
      if (workingset->isActiveConstr[(workingset->isActiveIdx[3] +
           workingset->sizes[3]) - 2]) {
        bool exitg1;
        idxStartIneq = workingset->sizes[0];
        exitg1 = false;
        while ((!exitg1) && (idxStartIneq + 1 <= workingset->nActiveConstr)) {
          if ((workingset->Wid[idxStartIneq] == 4) && (workingset->
               Wlocalidx[idxStartIneq] == workingset->sizes[3])) {
            b_removeConstr(workingset, idxStartIneq + 1);
            exitg1 = true;
          } else {
            idxStartIneq++;
          }
        }
      }

      mConstr = workingset->nActiveConstr;
      idxStartIneq = workingset->sizes[0];
      while ((mConstr > idxStartIneq) && (mConstr > nVar_tmp)) {
        b_removeConstr(workingset, mConstr);
        mConstr--;
      }

      solution->maxConstr = solution->xstar[13];
      b_setProblemType(workingset, PROBTYPE_ORIG);
      objective->objtype = objective->prev_objtype;
      objective->nvar = objective->prev_nvar;
      objective->hasLinear = objective->prev_hasLinear;
      options->ObjectiveLimit = -1.7976931348623157E+308;
      options->StepTolerance = 1.0E-6;
      if (solution->state != 0) {
        solution->maxConstr = d_maxConstraintViolation(workingset,
          solution->xstar);
        if (solution->maxConstr > 1.0E-6) {
          memset(&solution->lambda[0], 0, 27U * sizeof(double));
          solution->fstar = b_computeFval(objective, memspace->workspace_double,
            H, f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            double maxConstr_new;
            mConstr = (unsigned char)nVar_tmp;
            if (mConstr - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (unsigned int)
                     mConstr * sizeof(double));
            }

            b_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = d_maxConstraintViolation(workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (mConstr - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (unsigned
                        int)mConstr * sizeof(double));
              }
            }
          }

          b_iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                    objective, options->SolverName, options->StepTolerance,
                    options->ObjectiveLimit, runTimeOptions_MaxIterations);
        }
      }
    } else {
      b_iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                objective, options->SolverName, options->StepTolerance,
                options->ObjectiveLimit, runTimeOptions_MaxIterations);
    }
  }
}

/*
 * Arguments    : i_struct_T *obj
 *                const double A[378]
 *                int mrows
 *                int ncols
 * Return Type  : void
 */
static void b_factorQR(i_struct_T *obj, const double A[378], int mrows, int
  ncols)
{
  int i;
  int idx;
  int k;
  bool guard1;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    for (idx = 0; idx < ncols; idx++) {
      int ix0;
      int iy0;
      ix0 = 14 * idx;
      iy0 = 27 * idx;
      i = (unsigned char)mrows;
      for (k = 0; k < i; k++) {
        obj->QR[iy0 + k] = A[ix0 + k];
      }
    }

    guard1 = true;
  } else if (i == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt[idx] = idx + 1;
    }

    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }

    obj->minRowCol = i;
    memset(&obj->tau[0], 0, 27U * sizeof(double));
    if (i >= 1) {
      b_qrf(obj->QR, mrows, ncols, i, obj->tau);
    }
  }
}

/*
 * Arguments    : double workspace[378]
 *                double xCurrent[14]
 *                const o_struct_T *workingset
 *                i_struct_T *qrmanager
 * Return Type  : bool
 */
static bool b_feasibleX0ForWorkingSet(double workspace[378], double xCurrent[14],
  const o_struct_T *workingset, i_struct_T *qrmanager)
{
  double B[378];
  int b_i;
  int br;
  int iAcol;
  int j;
  int jBcol;
  int k;
  int mWConstr;
  int nVar;
  bool nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int i;
    int i1;
    for (iAcol = 0; iAcol < mWConstr; iAcol++) {
      c = workingset->bwset[iAcol];
      workspace[iAcol] = c;
      workspace[iAcol + 27] = c;
    }

    if (mWConstr != 0) {
      i = 14 * (mWConstr - 1) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 14) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (br = iAcol; br <= i1; br++) {
          c += workingset->ATwset[br - 1] * xCurrent[br - iAcol];
        }

        i1 = div_nde_s32_floor(iAcol - 1, 14);
        workspace[i1] -= c;
      }
    }

    if (mWConstr >= nVar) {
      i = (unsigned char)nVar;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (br = 0; br < i; br++) {
        iAcol = 27 * br;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] = workingset->ATwset[br + 14 * jBcol];
        }

        qrmanager->jpvt[br] = br + 1;
      }

      if (mWConstr <= nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }

      qrmanager->minRowCol = i;
      memset(&qrmanager->tau[0], 0, 27U * sizeof(double));
      if (i >= 1) {
        b_qrf(qrmanager->QR, mWConstr, nVar, i, qrmanager->tau);
      }

      b_computeQ_(qrmanager, mWConstr);
      memcpy(&B[0], &workspace[0], 378U * sizeof(double));
      for (b_i = 0; b_i <= 27; b_i += 27) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = -1;
      for (b_i = 0; b_i <= 27; b_i += 27) {
        jBcol = -1;
        i = b_i + 1;
        i1 = b_i + nVar;
        for (k = i; k <= i1; k++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q[(iAcol + jBcol) + 1] * B[(iAcol + br) + 1];
          }

          workspace[k - 1] += c;
          jBcol += 27;
        }

        br += 27;
      }

      for (j = 0; j < 2; j++) {
        jBcol = 27 * j - 1;
        for (k = nVar; k >= 1; k--) {
          iAcol = 27 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager->QR[k + iAcol];
            i1 = (unsigned char)(k - 1);
            for (b_i = 0; b_i < i1; b_i++) {
              int i2;
              i2 = (b_i + jBcol) + 1;
              workspace[i2] -= workspace[i] * qrmanager->QR[(b_i + iAcol) + 1];
            }
          }
        }
      }
    } else {
      b_factorQR(qrmanager, workingset->ATwset, nVar, mWConstr);
      b_computeQ_(qrmanager, qrmanager->minRowCol);
      for (j = 0; j < 2; j++) {
        jBcol = 27 * j;
        for (b_i = 0; b_i < mWConstr; b_i++) {
          iAcol = 27 * b_i;
          br = b_i + jBcol;
          c = workspace[br];
          i = (unsigned char)b_i;
          for (k = 0; k < i; k++) {
            c -= qrmanager->QR[k + iAcol] * workspace[k + jBcol];
          }

          workspace[br] = c / qrmanager->QR[b_i + iAcol];
        }
      }

      memcpy(&B[0], &workspace[0], 378U * sizeof(double));
      for (b_i = 0; b_i <= 27; b_i += 27) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = 0;
      for (b_i = 0; b_i <= 27; b_i += 27) {
        jBcol = -1;
        i = br + 1;
        i1 = br + mWConstr;
        for (j = i; j <= i1; j++) {
          int i2;
          i2 = b_i + 1;
          iAcol = b_i + nVar;
          for (k = i2; k <= iAcol; k++) {
            workspace[k - 1] += B[j - 1] * qrmanager->Q[(jBcol + k) - b_i];
          }

          jBcol += 27;
        }

        br += 27;
      }
    }

    iAcol = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iAcol <= (unsigned char)nVar - 1) {
        if ((fabs(workspace[iAcol]) >= 1.7976931348623157E+308) || (fabs
             (workspace[iAcol + 27]) >= 1.7976931348623157E+308)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          iAcol++;
        }
      } else {
        double constrViolation_basicX;
        iAcol = nVar - 1;
        for (k = 0; k <= iAcol; k++) {
          workspace[k] += xCurrent[k];
        }

        c = b_maxConstraintViolation(workingset, workspace, 1);
        constrViolation_basicX = b_maxConstraintViolation(workingset, workspace,
          28);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[0], (unsigned int)i * sizeof(double));
        } else {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[27], (unsigned int)i * sizeof(double));
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

/*
 * Arguments    : const double solution_xstar[14]
 *                const double solution_searchDir[14]
 *                int workingset_nVar
 *                const double workingset_lb[14]
 *                const double workingset_ub[14]
 *                const int workingset_indexLB[14]
 *                const int workingset_indexUB[14]
 *                const int workingset_sizes[5]
 *                const int workingset_isActiveIdx[6]
 *                const bool workingset_isActiveConstr[27]
 *                const int workingset_nWConstr[5]
 *                bool isPhaseOne
 *                bool *newBlocking
 *                int *constrType
 *                int *constrIdx
 * Return Type  : double
 */
static double b_feasibleratiotest(const double solution_xstar[14], const double
  solution_searchDir[14], int workingset_nVar, const double workingset_lb[14],
  const double workingset_ub[14], const int workingset_indexLB[14], const int
  workingset_indexUB[14], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[27], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx)
{
  double alpha;
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  int i;
  int i1;
  int idx;
  alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * d_xnrm2(workingset_nVar,
    solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = (double)isPhaseOne * solution_xstar[workingset_nVar -
      1];
    phaseOneCorrectionP = (double)isPhaseOne *
      solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      i1 = workingset_indexLB[idx];
      pk_corrected = -solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[i1 - 1] - workingset_lb[i1 - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }

    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    pk_corrected = -solution_searchDir[i];
    if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[i] - workingset_lb[i];
      pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
      if (pk_corrected < alpha) {
        alpha = pk_corrected;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = (double)isPhaseOne * solution_xstar[workingset_nVar -
      1];
    phaseOneCorrectionP = (double)isPhaseOne *
      solution_searchDir[workingset_nVar - 1];
    i = (unsigned char)workingset_sizes[4];
    for (idx = 0; idx < i; idx++) {
      i1 = workingset_indexUB[idx];
      pk_corrected = solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[i1 - 1] - workingset_ub[i1 - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (!isPhaseOne) {
    if ((*newBlocking) && (alpha > 1.0)) {
      *newBlocking = false;
    }

    alpha = fmin(alpha, 1.0);
  }

  return alpha;
}

/*
 * Arguments    : e_struct_T *fun_workspace
 *                double x0[13]
 *                const double lb[13]
 *                const double ub[13]
 *                double *exitflag
 *                double *output_iterations
 *                double *output_funcCount
 *                char output_algorithm[3]
 *                double *output_constrviolation
 *                double *output_stepsize
 *                double *output_lssteplength
 *                double *output_firstorderopt
 * Return Type  : double
 */
static double b_fmincon(e_struct_T *fun_workspace, double x0[13], const double
  lb[13], const double ub[13], double *exitflag, double *output_iterations,
  double *output_funcCount, char output_algorithm[3], double
  *output_constrviolation, double *output_stepsize, double *output_lssteplength,
  double *output_firstorderopt)
{
  b_struct_T MeritFunction;
  c_struct_T QPObjective;
  i_struct_T QRManager;
  j_struct_T CholManager;
  k_struct_T memspace;
  n_struct_T TrialState;
  o_struct_T WorkingSet;
  r_coder_internal_stickyStruct r;
  double varargout_2[13];
  double fval;
  int b_i;
  int i;
  int idx;
  int mFixed;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  bool exitg1;
  *exitflag = 1.7976931348623157E+308;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 13)) {
    if (lb[i] > ub[i]) {
      *exitflag = -2.0;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (*exitflag == -2.0) {
    fval = 1.7976931348623157E+308;
    *output_iterations = 0.0;
    *output_funcCount = 0.0;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = 1.7976931348623157E+308;
    *output_stepsize = 1.7976931348623157E+308;
    *output_lssteplength = 1.7976931348623157E+308;
    *output_firstorderopt = 1.7976931348623157E+308;
  } else {
    double scale;
    double y;
    int mLB;
    int mUB;
    TrialState.nVarMax = 14;
    TrialState.mNonlinIneq = 0;
    TrialState.mNonlinEq = 0;
    TrialState.mIneq = 0;
    TrialState.mEq = 0;
    TrialState.iNonIneq0 = 1;
    TrialState.iNonEq0 = 1;
    TrialState.sqpFval_old = 0.0;
    TrialState.sqpIterations = 0;
    TrialState.sqpExitFlag = 0;
    memset(&TrialState.lambdasqp[0], 0, 27U * sizeof(double));
    TrialState.steplength = 1.0;
    memset(&TrialState.delta_x[0], 0, 14U * sizeof(double));
    TrialState.fstar = 0.0;
    TrialState.firstorderopt = 0.0;
    memset(&TrialState.lambda[0], 0, 27U * sizeof(double));
    TrialState.state = 0;
    TrialState.maxConstr = 0.0;
    TrialState.iterations = 0;
    memcpy(&TrialState.xstarsqp[0], &x0[0], 13U * sizeof(double));
    WorkingSet.nVar = 13;
    WorkingSet.nVarOrig = 13;
    WorkingSet.nVarMax = 14;
    WorkingSet.ldA = 14;
    memset(&WorkingSet.lb[0], 0, 14U * sizeof(double));
    memset(&WorkingSet.ub[0], 0, 14U * sizeof(double));
    WorkingSet.mEqRemoved = 0;
    memset(&WorkingSet.ATwset[0], 0, 378U * sizeof(double));
    WorkingSet.nActiveConstr = 0;
    memset(&WorkingSet.bwset[0], 0, 27U * sizeof(double));
    memset(&WorkingSet.maxConstrWorkspace[0], 0, 27U * sizeof(double));
    memset(&WorkingSet.Wid[0], 0, 27U * sizeof(int));
    memset(&WorkingSet.Wlocalidx[0], 0, 27U * sizeof(int));
    for (i = 0; i < 27; i++) {
      WorkingSet.isActiveConstr[i] = false;
    }

    for (i = 0; i < 5; i++) {
      WorkingSet.nWConstr[i] = 0;
    }

    WorkingSet.probType = 3;
    WorkingSet.SLACK0 = 1.0E-5;
    for (i = 0; i < 14; i++) {
      WorkingSet.indexLB[i] = 0;
      WorkingSet.indexUB[i] = 0;
      WorkingSet.indexFixed[i] = 0;
    }

    mLB = 0;
    mUB = 0;
    mFixed = 0;
    for (idx = 0; idx < 13; idx++) {
      bool guard1;
      y = lb[idx];
      guard1 = false;
      if (y > -1.7976931348623157E+308) {
        if (fabs(y - ub[idx]) < 1.0E-6) {
          mFixed++;
          WorkingSet.indexFixed[mFixed - 1] = idx + 1;
        } else {
          mLB++;
          WorkingSet.indexLB[mLB - 1] = idx + 1;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1 && (ub[idx] < 1.7976931348623157E+308)) {
        mUB++;
        WorkingSet.indexUB[mUB - 1] = idx + 1;
      }
    }

    i = (mLB + mUB) + mFixed;
    WorkingSet.mConstr = i;
    WorkingSet.mConstrOrig = i;
    WorkingSet.mConstrMax = 27;
    obj_tmp[0] = (signed char)mFixed;
    obj_tmp[1] = 0;
    obj_tmp[2] = 0;
    obj_tmp[3] = (signed char)mLB;
    obj_tmp[4] = (signed char)mUB;
    b_obj_tmp[0] = (signed char)mFixed;
    b_obj_tmp[1] = 0;
    b_obj_tmp[2] = 0;
    b_obj_tmp[3] = (signed char)(mLB + 1);
    b_obj_tmp[4] = (signed char)mUB;
    WorkingSet.isActiveIdx[0] = 1;
    WorkingSet.isActiveIdx[1] = mFixed;
    WorkingSet.isActiveIdx[2] = 0;
    WorkingSet.isActiveIdx[3] = 0;
    WorkingSet.isActiveIdx[4] = mLB;
    WorkingSet.isActiveIdx[5] = mUB;
    for (i = 0; i < 5; i++) {
      signed char i1;
      signed char i2;
      i1 = obj_tmp[i];
      WorkingSet.sizes[i] = i1;
      WorkingSet.sizesNormal[i] = i1;
      i2 = b_obj_tmp[i];
      WorkingSet.sizesPhaseOne[i] = i2;
      WorkingSet.sizesRegularized[i] = i1;
      WorkingSet.sizesRegPhaseOne[i] = i2;
      WorkingSet.isActiveIdx[i + 1] += WorkingSet.isActiveIdx[i];
    }

    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxNormal[b_i] = WorkingSet.isActiveIdx[b_i];
    }

    WorkingSet.isActiveIdxPhaseOne[0] = 1;
    WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxPhaseOne[2] = 0;
    WorkingSet.isActiveIdxPhaseOne[3] = 0;
    WorkingSet.isActiveIdxPhaseOne[4] = mLB + 1;
    WorkingSet.isActiveIdxPhaseOne[5] = mUB;
    for (i = 0; i < 5; i++) {
      WorkingSet.isActiveIdxPhaseOne[i + 1] += WorkingSet.isActiveIdxPhaseOne[i];
    }

    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
      WorkingSet.isActiveIdxRegPhaseOne[b_i] =
        WorkingSet.isActiveIdxPhaseOne[b_i];
    }

    for (idx = 0; idx < mLB; idx++) {
      b_i = WorkingSet.indexLB[idx];
      TrialState.xstarsqp[b_i - 1] = fmax(TrialState.xstarsqp[b_i - 1], lb[b_i -
        1]);
    }

    for (idx = 0; idx < mUB; idx++) {
      b_i = WorkingSet.indexUB[idx];
      TrialState.xstarsqp[b_i - 1] = fmin(TrialState.xstarsqp[b_i - 1], ub[b_i -
        1]);
    }

    for (idx = 0; idx < mFixed; idx++) {
      b_i = WorkingSet.indexFixed[idx];
      TrialState.xstarsqp[b_i - 1] = ub[b_i - 1];
    }

    TrialState.sqpFval = c_compute_cost_and_gradient_sec
      (fun_workspace->dv_global, fun_workspace->Beta, fun_workspace->CL_aileron,
       fun_workspace->Cd_zero, fun_workspace->Cl_alpha, fun_workspace->Cm_zero,
       fun_workspace->Cm_alpha, fun_workspace->I_xx, fun_workspace->I_yy,
       fun_workspace->I_zz, fun_workspace->K_Cd, fun_workspace->K_p_M,
       fun_workspace->K_p_T, fun_workspace->Phi, fun_workspace->S,
       fun_workspace->Theta, fun_workspace->V, fun_workspace->W_act_motor,
       fun_workspace->W_dv_1, fun_workspace->W_dv_2, fun_workspace->W_dv_3,
       fun_workspace->W_dv_4, fun_workspace->W_dv_5, fun_workspace->W_dv_6,
       fun_workspace->W_act_tilt_el, fun_workspace->W_act_tilt_az,
       fun_workspace->W_act_ailerons, fun_workspace->desired_el_value,
       fun_workspace->desired_az_value, fun_workspace->desired_motor_value,
       fun_workspace->desired_ailerons_value, fun_workspace->flight_path_angle,
       fun_workspace->gain_el, fun_workspace->gain_az, fun_workspace->gain_motor,
       fun_workspace->gain_ailerons, fun_workspace->gamma_quadratic_du,
       fun_workspace->l_1, fun_workspace->l_2, fun_workspace->l_3,
       fun_workspace->l_4, fun_workspace->l_z, fun_workspace->m,
       fun_workspace->p, fun_workspace->q, fun_workspace->r, fun_workspace->rho,
       fun_workspace->wing_chord, TrialState.xstarsqp, varargout_2);
    memcpy(&TrialState.grad[0], &varargout_2[0], 13U * sizeof(double));
    TrialState.FunctionEvaluations = 1;
    for (idx = 0; idx < mLB; idx++) {
      WorkingSet.lb[WorkingSet.indexLB[idx] - 1] = -lb[WorkingSet.indexLB[idx] -
        1] + x0[WorkingSet.indexLB[idx] - 1];
    }

    for (idx = 0; idx < mUB; idx++) {
      WorkingSet.ub[WorkingSet.indexUB[idx] - 1] = ub[WorkingSet.indexUB[idx] -
        1] - x0[WorkingSet.indexUB[idx] - 1];
    }

    for (idx = 0; idx < mFixed; idx++) {
      y = ub[WorkingSet.indexFixed[idx] - 1] - x0[WorkingSet.indexFixed[idx] - 1];
      WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = y;
      WorkingSet.bwset[idx] = y;
    }

    b_setProblemType(&WorkingSet, 3);
    i = WorkingSet.isActiveIdx[2];
    for (idx = i; idx < 28; idx++) {
      WorkingSet.isActiveConstr[idx - 1] = false;
    }

    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    b_i = (unsigned char)WorkingSet.sizes[0];
    for (mFixed = 0; mFixed < b_i; mFixed++) {
      WorkingSet.Wid[mFixed] = 1;
      WorkingSet.Wlocalidx[mFixed] = mFixed + 1;
      WorkingSet.isActiveConstr[mFixed] = true;
      i = 14 * mFixed;
      mLB = WorkingSet.indexFixed[mFixed];
      if (mLB - 2 >= 0) {
        memset(&WorkingSet.ATwset[i], 0, (unsigned int)(((mLB + i) - i) - 1) *
               sizeof(double));
      }

      WorkingSet.ATwset[(WorkingSet.indexFixed[mFixed] + i) - 1] = 1.0;
      mLB = WorkingSet.indexFixed[mFixed] + 1;
      mUB = WorkingSet.nVar;
      if (mLB <= mUB) {
        memset(&WorkingSet.ATwset[(mLB + i) + -1], 0, (unsigned int)((((mUB + i)
                  - mLB) - i) + 1) * sizeof(double));
      }

      WorkingSet.bwset[mFixed] = WorkingSet.ub[WorkingSet.indexFixed[mFixed] - 1];
    }

    double Hessian[169];
    factoryConstruct(TrialState.sqpFval, &MeritFunction);
    r.next.next.next.next.next.next.next.next.value.workspace = *fun_workspace;
    d_driver(lb, ub, &TrialState, &MeritFunction, &r, &memspace, &WorkingSet,
             Hessian, &QRManager, &CholManager, &QPObjective);
    fval = TrialState.sqpFval;
    *exitflag = TrialState.sqpExitFlag;
    *output_iterations = TrialState.sqpIterations;
    *output_funcCount = TrialState.FunctionEvaluations;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = MeritFunction.nlpPrimalFeasError;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (i = 0; i < 13; i++) {
      double absxk;
      x0[i] = TrialState.xstarsqp[i];
      absxk = fabs(TrialState.delta_x[i]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        y += t * t;
      }
    }

    *output_stepsize = scale * sqrt(y);
    *output_lssteplength = TrialState.steplength;
    *output_firstorderopt = MeritFunction.firstOrderOpt;
  }

  return fval;
}

/*
 * Arguments    : j_struct_T *obj
 *                int NColsRemain
 * Return Type  : void
 */
static void b_fullColLDL2_(j_struct_T *obj, int NColsRemain)
{
  int ijA;
  int j;
  int jA;
  int k;
  for (k = 0; k < NColsRemain; k++) {
    double alpha1;
    double y;
    int LD_diagOffset;
    int i;
    int offset1;
    int subMatrixDim;
    LD_diagOffset = 28 * k;
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 2;
    y = obj->workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj->FMat[(LD_diagOffset + jA) + 1];
    }

    obj->workspace_ = y;
    if (alpha1 != 0.0) {
      jA = LD_diagOffset;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 29;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 29; ijA++) {
            obj->FMat[ijA - 1] += y * temp;
          }
        }

        jA += 27;
      }
    }

    alpha1 = 1.0 / obj->FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i + 2; jA++) {
      obj->FMat[jA - 1] *= alpha1;
    }
  }
}

/*
 * Arguments    : const double H[169]
 *                const double f[14]
 *                n_struct_T *solution
 *                k_struct_T *memspace
 *                o_struct_T *workingset
 *                i_struct_T *qrmanager
 *                j_struct_T *cholmanager
 *                c_struct_T *objective
 *                const char options_SolverName[7]
 *                double options_StepTolerance
 *                double options_ObjectiveLimit
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void b_iterate(const double H[169], const double f[14], n_struct_T
                      *solution, k_struct_T *memspace, o_struct_T *workingset,
                      i_struct_T *qrmanager, j_struct_T *cholmanager, c_struct_T
                      *objective, const char options_SolverName[7], double
                      options_StepTolerance, double options_ObjectiveLimit, int
                      runTimeOptions_MaxIterations)
{
  static const char b[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  double s;
  double temp;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int i;
  int idx;
  int iyend;
  int nActiveConstr;
  int nVar;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  b_computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = b_computeFval_ReuseHx(objective, memspace->workspace_double,
    f, solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  memset(&solution->lambda[0], 0, 27U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      double minLambda;
      int idxQR;
      int ix0;
      bool guard1;
      bool guard2;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          {
            double c;
            ix0 = 14 * (workingset->nActiveConstr - 1);
            iyend = qrmanager->mrows;
            idxQR = qrmanager->ncols + 1;
            if (iyend <= idxQR) {
              idxQR = iyend;
            }

            qrmanager->minRowCol = idxQR;
            idxQR = 27 * qrmanager->ncols;
            if (qrmanager->mrows != 0) {
              iyend = idxQR + qrmanager->mrows;
              if (idxQR + 1 <= iyend) {
                memset(&qrmanager->QR[idxQR], 0, (unsigned int)(iyend - idxQR) *
                       sizeof(double));
              }

              i = 27 * (qrmanager->mrows - 1) + 1;
              for (iyend = 1; iyend <= i; iyend += 27) {
                c = 0.0;
                nActiveConstr = (iyend + qrmanager->mrows) - 1;
                for (idx = iyend; idx <= nActiveConstr; idx++) {
                  c += qrmanager->Q[idx - 1] * workingset->ATwset[(ix0 + idx) -
                    iyend];
                }

                nActiveConstr = idxQR + div_nde_s32_floor(iyend - 1, 27);
                qrmanager->QR[nActiveConstr] += c;
              }
            }

            qrmanager->ncols++;
            i = qrmanager->ncols - 1;
            qrmanager->jpvt[i] = qrmanager->ncols;
            for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--)
            {
              nActiveConstr = idx + 27 * i;
              temp = qrmanager->QR[nActiveConstr + 1];
              c = xrotg(&qrmanager->QR[nActiveConstr], &temp, &s);
              qrmanager->QR[nActiveConstr + 1] = temp;
              iyend = 27 * idx;
              ix0 = qrmanager->mrows;
              if (qrmanager->mrows >= 1) {
                for (nActiveConstr = 0; nActiveConstr < ix0; nActiveConstr++) {
                  idxQR = iyend + nActiveConstr;
                  minLambda = qrmanager->Q[idxQR + 27];
                  temp = c * qrmanager->Q[idxQR] + s * minLambda;
                  qrmanager->Q[idxQR + 27] = c * minLambda - s * qrmanager->
                    Q[idxQR];
                  qrmanager->Q[idxQR] = temp;
                }
              }
            }
          }
          break;

         case -1:
          b_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          b_factorQR(qrmanager, workingset->ATwset, nVar,
                     workingset->nActiveConstr);
          b_computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        iyend = memcmp(&options_SolverName[0], &b[0], 7);
        b_compute_deltax(H, solution, memspace, qrmanager, cholmanager,
                         objective, iyend == 0);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((d_xnrm2(nVar, solution->searchDir) < options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar)) {
          guard2 = true;
        } else {
          minLambda = b_feasibleratiotest(solution->xstar, solution->searchDir,
            workingset->nVar, workingset->lb, workingset->ub,
            workingset->indexLB, workingset->indexUB, workingset->sizes,
            workingset->isActiveIdx, workingset->isActiveConstr,
            workingset->nWConstr, TYPE == 5, &updateFval, &i, &iyend);
          if (updateFval) {
            switch (i) {
             case 3:
              workingset->nWConstr[2]++;
              workingset->isActiveConstr[(workingset->isActiveIdx[2] + iyend) -
                2] = true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] = iyend;

              /* A check that is always false is detected at compile-time. Eliminating code that follows. */
              break;

             case 4:
              b_addBoundToActiveSetMatrix_(workingset, 4, iyend);
              break;

             default:
              b_addBoundToActiveSetMatrix_(workingset, 5, iyend);
              break;
            }

            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (d_xnrm2(objective->nvar, solution->searchDir) > 100.0 *
                  (double)objective->nvar * 1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }

            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (minLambda != 0.0)) {
            iyend = nVar - 1;
            for (nActiveConstr = 0; nActiveConstr <= iyend; nActiveConstr++) {
              solution->xstar[nActiveConstr] += minLambda * solution->
                searchDir[nActiveConstr];
            }
          }

          b_computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
          guard1 = true;
        }
      } else {
        i = (unsigned char)nVar;
        memset(&solution->searchDir[0], 0, (unsigned int)i * sizeof(double));
        guard2 = true;
      }

      if (guard2) {
        nActiveConstr = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          bool b_guard1;
          b_guard1 = false;
          if (objective->objtype != 4) {
            minLambda = 100.0 * (double)qrmanager->mrows *
              2.2204460492503131E-16;
            if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }

            if (updateFval) {
              bool b_guard2;
              idx = qrmanager->ncols;
              b_guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                idxQR = qrmanager->mrows + 27 * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) && (fabs(qrmanager->QR[idxQR - 1])
                        >= minLambda)) {
                  idx--;
                  idxQR -= 27;
                }

                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }

              if (b_guard2) {
                idxQR = idx + 27 * (idx - 1);
                while ((idx >= 1) && (fabs(qrmanager->QR[idxQR - 1]) >=
                                      minLambda)) {
                  idx--;
                  idxQR -= 28;
                }

                updateFval = (idx == 0);
              }
            }

            if (!updateFval) {
              solution->state = -7;
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }

          if (b_guard1) {
            ix0 = qrmanager->ncols;
            d_xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q,
                    objective->grad, memspace->workspace_double);
            if (qrmanager->ncols != 0) {
              for (idx = ix0; idx >= 1; idx--) {
                iyend = (idx + (idx - 1) * 27) - 1;
                memspace->workspace_double[idx - 1] /= qrmanager->QR[iyend];
                for (i = 0; i <= idx - 2; i++) {
                  idxQR = (idx - i) - 2;
                  memspace->workspace_double[idxQR] -=
                    memspace->workspace_double[idx - 1] * qrmanager->QR[(iyend -
                    i) - 1];
                }
              }
            }

            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }

        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          iyend = 0;
          minLambda = 0.0;
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          nActiveConstr = workingset->nActiveConstr;
          for (idx = i; idx <= nActiveConstr; idx++) {
            temp = solution->lambda[idx - 1];
            if (temp < minLambda) {
              minLambda = temp;
              iyend = idx;
            }
          }

          if (iyend == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iyend;
            subProblemChanged = true;
            b_removeConstr(workingset, iyend);
            solution->lambda[iyend - 1] = 0.0;
          }
        } else {
          iyend = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          b_removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda[iyend - 1] = 0.0;
        }

        updateFval = false;
        guard1 = true;
      }

      if (guard1) {
        solution->iterations++;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = d_maxConstraintViolation(workingset,
            solution->xstar);
          minLambda = solution->maxConstr;
          if (objective->objtype == 5) {
            minLambda = solution->maxConstr - solution->xstar[objective->nvar -
              1];
          }

          if (minLambda > 1.0E-6) {
            bool nonDegenerateWset;
            i = (unsigned char)objective->nvar;
            if (i - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (unsigned int)
                     i * sizeof(double));
            }

            nonDegenerateWset = b_feasibleX0ForWorkingSet
              (memspace->workspace_double, solution->searchDir, workingset,
               qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }

            activeSetChangeID = 0;
            minLambda = d_maxConstraintViolation(workingset, solution->searchDir);
            if (minLambda < solution->maxConstr) {
              if (i - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (unsigned
                        int)i * sizeof(double));
              }

              solution->maxConstr = minLambda;
            }
          }
        }

        if (updateFval) {
          solution->fstar = b_computeFval_ReuseHx(objective,
            memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) && ((solution->state !=
                0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = b_computeFval_ReuseHx(objective,
          memspace->workspace_double, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/*
 * Arguments    : bool obj_hasLinear
 *                int obj_nvar
 *                double workspace[378]
 *                const double H[169]
 *                const double f[14]
 *                const double x[14]
 * Return Type  : void
 */
static void b_linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[378],
  const double H[169], const double f[14], const double x[14])
{
  int beta1;
  int ia;
  int iac;
  beta1 = 0;
  if (obj_hasLinear) {
    beta1 = (unsigned char)obj_nvar;
    if (beta1 - 1 >= 0) {
      memcpy(&workspace[0], &f[0], (unsigned int)beta1 * sizeof(double));
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int ix;
    if (beta1 != 1) {
      beta1 = (unsigned char)obj_nvar;
      if (beta1 - 1 >= 0) {
        memset(&workspace[0], 0, (unsigned int)beta1 * sizeof(double));
      }
    }

    ix = 0;
    beta1 = obj_nvar * (obj_nvar - 1) + 1;
    for (iac = 1; obj_nvar < 0 ? iac >= beta1 : iac <= beta1; iac += obj_nvar) {
      double c;
      int i;
      c = 0.5 * x[ix];
      i = (iac + obj_nvar) - 1;
      for (ia = iac; ia <= i; ia++) {
        int i1;
        i1 = ia - iac;
        workspace[i1] += H[ia - 1] * c;
      }

      ix++;
    }
  }
}

/*
 * Arguments    : const o_struct_T *obj
 *                const double x[378]
 *                int ix0
 * Return Type  : double
 */
static double b_maxConstraintViolation(const o_struct_T *obj, const double x[378],
  int ix0)
{
  double v;
  int i;
  int idx;
  int idxLB;
  v = 0.0;
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[(ix0 + idxLB) - 1] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    i = (unsigned char)obj->sizes[4];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[(ix0 + idxLB) - 1] - obj->ub[idxLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      v = fmax(v, fabs(x[(ix0 + obj->indexFixed[idx]) - 2] - obj->ub
                       [obj->indexFixed[idx] - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : const double x[6]
 * Return Type  : double
 */
static double b_norm(const double x[6])
{
  double scale;
  double y;
  int k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 6; k++) {
    double absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/*
 * Arguments    : double A[729]
 *                int m
 *                int n
 *                int nfxd
 *                double tau[27]
 * Return Type  : void
 */
static void b_qrf(double A[729], int m, int n, int nfxd, double tau[27])
{
  double work[27];
  double atmp;
  int b_i;
  int i;
  memset(&tau[0], 0, 27U * sizeof(double));
  memset(&work[0], 0, 27U * sizeof(double));
  i = (unsigned char)nfxd;
  for (b_i = 0; b_i < i; b_i++) {
    double d;
    int ii;
    int mmi;
    ii = b_i * 27 + b_i;
    mmi = m - b_i;
    if (b_i + 1 < m) {
      atmp = A[ii];
      d = b_xzlarfg(mmi, &atmp, A, ii + 2);
      tau[b_i] = d;
      A[ii] = atmp;
    } else {
      d = 0.0;
      tau[b_i] = 0.0;
    }

    if (b_i + 1 < n) {
      atmp = A[ii];
      A[ii] = 1.0;
      b_xzlarf(mmi, (n - b_i) - 1, ii + 1, d, A, ii + 28, work);
      A[ii] = atmp;
    }
  }
}

/*
 * Arguments    : o_struct_T *obj
 *                int idx_global
 * Return Type  : void
 */
static void b_removeConstr(o_struct_T *obj, int idx_global)
{
  int TYPE_tmp;
  int i;
  int i1;
  int idx;
  TYPE_tmp = obj->Wid[idx_global - 1] - 1;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx[idx_global -
                       1]) - 2] = false;
  i = obj->nActiveConstr - 1;
  obj->Wid[idx_global - 1] = obj->Wid[i];
  obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[i];
  i1 = (unsigned char)obj->nVar;
  for (idx = 0; idx < i1; idx++) {
    obj->ATwset[idx + 14 * (idx_global - 1)] = obj->ATwset[idx + 14 * i];
  }

  obj->bwset[idx_global - 1] = obj->bwset[i];
  obj->nActiveConstr = i;
  obj->nWConstr[TYPE_tmp]--;
}

/*
 * Arguments    : o_struct_T *obj
 *                int PROBLEM_TYPE
 * Return Type  : void
 */
static void b_setProblemType(o_struct_T *obj, int PROBLEM_TYPE)
{
  int i;
  int idx;
  int idx_col;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 13;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = (unsigned char)obj->sizesNormal[4];
      for (idx = 0; idx < i; idx++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + idx) - 1] =
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
      }
    }

    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }

    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;

   case 1:
    {
      int idxStartIneq;
      obj->nVar = 14;
      obj->mConstr = obj->mConstrOrig + 1;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[14 * idx + 13] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 14;
      obj->lb[13] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[14 * (idx - 1) + 13] = -1.0;
      }

      if (obj->nWConstr[4] > 0) {
        i = (unsigned char)(obj->sizesNormal[4] + 1);
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
        }
      }

      obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
    }
    break;

   case 2:
    {
      obj->nVar = 13;
      obj->mConstr = 26;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegularized[i];
      }

      if (obj->probType != 4) {
        int i1;
        int idxStartIneq;
        int idx_lb;
        idx_lb = 13;
        i = obj->sizesNormal[3] + 1;
        i1 = obj->sizesRegularized[3];
        for (idx = i; idx <= i1; idx++) {
          idx_lb++;
          obj->indexLB[idx - 1] = idx_lb;
        }

        if (obj->nWConstr[4] > 0) {
          i = (unsigned char)obj->sizesRegularized[4];
          for (idx = 0; idx < i; idx++) {
            obj->isActiveConstr[obj->isActiveIdxRegularized[4] + idx] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
          }
        }

        i = obj->isActiveIdx[4];
        i1 = obj->isActiveIdxRegularized[4] - 1;
        if (i <= i1) {
          memset(&obj->isActiveConstr[i + -1], 0, (unsigned int)((i1 - i) + 1) *
                 sizeof(bool));
        }

        idxStartIneq = obj->isActiveIdx[2];
        i = obj->nActiveConstr;
        for (idx_col = idxStartIneq; idx_col <= i; idx_col++) {
          idx_lb = 14 * (idx_col - 1) - 1;
          if (obj->Wid[idx_col - 1] == 3) {
            i1 = obj->Wlocalidx[idx_col - 1];
            idx = i1 + 12;
            if (idx >= 14) {
              memset(&obj->ATwset[idx_lb + 14], 0, (unsigned int)(((idx + idx_lb)
                       - idx_lb) - 13) * sizeof(double));
            }

            obj->ATwset[(i1 + idx_lb) + 13] = -1.0;
            i1 += 14;
            if (i1 <= 13) {
              memset(&obj->ATwset[i1 + idx_lb], 0, (unsigned int)(((idx_lb - i1)
                       - idx_lb) + 14) * sizeof(double));
            }
          }
        }
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
      }
    }
    break;

   default:
    {
      int idxStartIneq;
      obj->nVar = 14;
      obj->mConstr = 27;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[14 * idx + 13] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 14;
      obj->lb[13] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[14 * (idx - 1) + 13] = -1.0;
      }

      if (obj->nWConstr[4] > 0) {
        i = (unsigned char)(obj->sizesNormal[4] + 1);
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
        }
      }

      obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

/*
 * Arguments    : const j_struct_T *obj
 *                double rhs[14]
 * Return Type  : void
 */
static void b_solve(const j_struct_T *obj, double rhs[14])
{
  int i;
  int j;
  int jA;
  int n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    for (j = 0; j < n_tmp; j++) {
      double temp;
      jA = j * 27;
      temp = rhs[j];
      for (i = 0; i < j; i++) {
        temp -= obj->FMat[jA + i] * rhs[i];
      }

      rhs[j] = temp / obj->FMat[jA + j];
    }
  }

  if (obj->ndims != 0) {
    for (j = n_tmp; j >= 1; j--) {
      jA = (j + (j - 1) * 27) - 1;
      rhs[j - 1] /= obj->FMat[jA];
      for (i = 0; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        rhs[ix] -= rhs[j - 1] * obj->FMat[(jA - i) - 1];
      }
    }
  }
}

/*
 * Arguments    : double lambda[27]
 *                int WorkingSet_nActiveConstr
 *                const int WorkingSet_sizes[5]
 *                const int WorkingSet_isActiveIdx[6]
 *                const int WorkingSet_Wid[27]
 *                const int WorkingSet_Wlocalidx[27]
 *                double workspace[378]
 * Return Type  : void
 */
static void b_sortLambdaQP(double lambda[27], int WorkingSet_nActiveConstr,
  const int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[27], const int WorkingSet_Wlocalidx[27], double workspace[378])
{
  if (WorkingSet_nActiveConstr != 0) {
    int idx;
    int idxOffset;
    int mAll;
    mAll = (WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4];
    idx = (unsigned char)mAll;
    if (idx - 1 >= 0) {
      memcpy(&workspace[0], &lambda[0], (unsigned int)idx * sizeof(double));
    }

    if (mAll - 1 >= 0) {
      memset(&lambda[0], 0, (unsigned int)mAll * sizeof(double));
    }

    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) && (WorkingSet_Wid[idx] <= 2))
    {
      if (WorkingSet_Wid[idx] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }

      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }

    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid[idx]) {
       case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }

      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }
  }
}

/*
 * Arguments    : int *STEP_TYPE
 *                double Hessian[169]
 *                const double lb[13]
 *                const double ub[13]
 *                n_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                k_struct_T *memspace
 *                o_struct_T *WorkingSet
 *                i_struct_T *QRManager
 *                j_struct_T *CholManager
 *                c_struct_T *QPObjective
 *                p_struct_T *qpoptions
 * Return Type  : bool
 */
static bool b_step(int *STEP_TYPE, double Hessian[169], const double lb[13],
                   const double ub[13], n_struct_T *TrialState, b_struct_T
                   *MeritFunction, k_struct_T *memspace, o_struct_T *WorkingSet,
                   i_struct_T *QRManager, j_struct_T *CholManager, c_struct_T
                   *QPObjective, p_struct_T *qpoptions)
{
  p_struct_T b_qpoptions;
  double dv[14];
  double oldDirIdx;
  double s;
  int idxEndIneq_tmp_tmp;
  int idxStartIneq;
  int k;
  int nVar_tmp_tmp;
  bool checkBoundViolation;
  bool stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (unsigned int)
           idxEndIneq_tmp_tmp * sizeof(double));
  } else {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    if (idxEndIneq_tmp_tmp - 1 >= 0) {
      memcpy(&TrialState->searchDir[0], &TrialState->xstar[0], (unsigned int)
             idxEndIneq_tmp_tmp * sizeof(double));
    }
  }

  int exitg1;
  bool guard1;
  do {
    int temp;
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
     case 1:
      b_qpoptions = *qpoptions;
      b_driver(Hessian, TrialState->grad, TrialState, memspace, WorkingSet,
               QRManager, CholManager, QPObjective, &b_qpoptions,
               qpoptions->MaxIterations);
      if (TrialState->state > 0) {
        MeritFunction->phi = TrialState->sqpFval;
        MeritFunction->linearizedConstrViol = 0.0;
        MeritFunction->penaltyParam = 1.0;
        MeritFunction->phiPrimePlus = fmin(TrialState->fstar, 0.0);
      }

      b_sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                     WorkingSet->Wlocalidx, memspace->workspace_double);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (unsigned int)
                 idxEndIneq_tmp_tmp * sizeof(double));
        }

        guard1 = true;
      }
      break;

     case 2:
      {
        double beta;
        temp = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
        idxStartIneq = temp + 1;
        idxEndIneq_tmp_tmp = WorkingSet->nActiveConstr;
        for (k = idxStartIneq; k <= idxEndIneq_tmp_tmp; k++) {
          WorkingSet->isActiveConstr[(WorkingSet->isActiveIdx[WorkingSet->Wid[k
            - 1] - 1] + WorkingSet->Wlocalidx[k - 1]) - 2] = false;
        }

        WorkingSet->nWConstr[2] = 0;
        WorkingSet->nWConstr[3] = 0;
        WorkingSet->nWConstr[4] = 0;
        WorkingSet->nActiveConstr = temp;
        memcpy(&dv[0], &TrialState->xstar[0], 14U * sizeof(double));
        idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          s = WorkingSet->lb[WorkingSet->indexLB[k] - 1];
          if (-dv[WorkingSet->indexLB[k] - 1] > s) {
            dv[WorkingSet->indexLB[k] - 1] = (WorkingSet->ub[WorkingSet->
              indexLB[k] - 1] - s) / 2.0;
          }
        }

        idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          s = WorkingSet->ub[WorkingSet->indexUB[k] - 1];
          if (dv[WorkingSet->indexUB[k] - 1] > s) {
            dv[WorkingSet->indexUB[k] - 1] = (s - WorkingSet->lb
              [WorkingSet->indexUB[k] - 1]) / 2.0;
          }
        }

        memcpy(&TrialState->xstar[0], &dv[0], 14U * sizeof(double));
        idxEndIneq_tmp_tmp = WorkingSet->nVar;
        beta = 0.0;
        idxStartIneq = (unsigned char)WorkingSet->nVar;
        for (k = 0; k < idxStartIneq; k++) {
          beta += Hessian[k + 13 * k];
        }

        if (TrialState->sqpIterations <= 1) {
          temp = QPObjective->nvar;
          if (QPObjective->nvar < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (QPObjective->nvar > 1) {
              oldDirIdx = fabs(TrialState->grad[0]);
              for (k = 2; k <= temp; k++) {
                s = fabs(TrialState->grad[k - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = k;
                  oldDirIdx = s;
                }
              }
            }
          }

          s = 100.0 * fmax(1.0, fabs(TrialState->grad[idxStartIneq - 1]));
        } else {
          temp = WorkingSet->mConstr;
          if (WorkingSet->mConstr < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (WorkingSet->mConstr > 1) {
              oldDirIdx = fabs(TrialState->lambdasqp[0]);
              for (k = 2; k <= temp; k++) {
                s = fabs(TrialState->lambdasqp[k - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = k;
                  oldDirIdx = s;
                }
              }
            }
          }

          s = fabs(TrialState->lambdasqp[idxStartIneq - 1]);
        }

        QPObjective->nvar = WorkingSet->nVar;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 4;
        QPObjective->beta = beta / (double)WorkingSet->nVar;
        QPObjective->rho = s;
        b_setProblemType(WorkingSet, 2);
        temp = qpoptions->MaxIterations;
        qpoptions->MaxIterations = (qpoptions->MaxIterations + WorkingSet->nVar)
          - idxEndIneq_tmp_tmp;
        memcpy(&dv[0], &TrialState->grad[0], 14U * sizeof(double));
        b_qpoptions = *qpoptions;
        b_driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
                 CholManager, QPObjective, &b_qpoptions,
                 qpoptions->MaxIterations);
        qpoptions->MaxIterations = temp;
        if (TrialState->state != -6) {
          MeritFunction->phi = TrialState->sqpFval;
          MeritFunction->linearizedConstrViol = 0.0;
          MeritFunction->penaltyParam = 1.0;
          MeritFunction->phiPrimePlus = fmin(TrialState->fstar, 0.0);
          temp = WorkingSet->isActiveIdx[2];
          idxStartIneq = WorkingSet->nActiveConstr;
          for (k = temp; k <= idxStartIneq; k++) {
            if (WorkingSet->Wid[k - 1] == 3) {
              TrialState->lambda[k - 1] *= (double)memspace->
                workspace_int[WorkingSet->Wlocalidx[k - 1] - 1];
            }
          }
        }

        QPObjective->nvar = idxEndIneq_tmp_tmp;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 3;
        b_setProblemType(WorkingSet, 3);
        b_sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                       WorkingSet->sizes, WorkingSet->isActiveIdx,
                       WorkingSet->Wid, WorkingSet->Wlocalidx,
                       memspace->workspace_double);
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (unsigned int)
                 idxEndIneq_tmp_tmp * sizeof(double));
        }

        guard1 = true;
      }
      break;

     default:
      idxEndIneq_tmp_tmp = WorkingSet->nVar;
      idxStartIneq = (unsigned char)WorkingSet->nVar;
      for (k = 0; k < idxStartIneq; k++) {
        TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k];
        TrialState->socDirection[k] = TrialState->xstar[k];
      }

      memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0], 27U *
             sizeof(double));
      memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (unsigned int)
             idxStartIneq * sizeof(double));
      memcpy(&dv[0], &TrialState->grad[0], 14U * sizeof(double));
      b_qpoptions = *qpoptions;
      b_driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
               CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
      idxStartIneq = (unsigned char)idxEndIneq_tmp_tmp;
      for (k = 0; k < idxStartIneq; k++) {
        s = TrialState->socDirection[k];
        oldDirIdx = s;
        s = TrialState->xstar[k] - s;
        TrialState->socDirection[k] = s;
        TrialState->xstar[k] = oldDirIdx;
      }

      stepSuccess = (d_xnrm2(idxEndIneq_tmp_tmp, TrialState->socDirection) <=
                     2.0 * d_xnrm2(idxEndIneq_tmp_tmp, TrialState->xstar));
      if (!stepSuccess) {
        memcpy(&TrialState->lambda[0], &TrialState->lambdaStopTest[0], 27U *
               sizeof(double));
      } else {
        b_sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                       WorkingSet->sizes, WorkingSet->isActiveIdx,
                       WorkingSet->Wid, WorkingSet->Wlocalidx,
                       memspace->workspace_double);
      }

      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          TrialState->delta_x[k] = TrialState->xstar[k] +
            TrialState->socDirection[k];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        s = 0.0;
        oldDirIdx = 1.0;
        for (k = 0; k < 13; k++) {
          s = fmax(s, fabs(TrialState->grad[k]));
          oldDirIdx = fmax(oldDirIdx, fabs(TrialState->xstar[k]));
        }

        s = fmax(2.2204460492503131E-16, s / oldDirIdx);
        for (idxStartIneq = 0; idxStartIneq < 13; idxStartIneq++) {
          temp = 13 * idxStartIneq;
          for (k = 0; k < idxStartIneq; k++) {
            Hessian[temp + k] = 0.0;
          }

          temp = idxStartIneq + 13 * idxStartIneq;
          Hessian[temp] = s;
          idxEndIneq_tmp_tmp = 11 - idxStartIneq;
          if (idxEndIneq_tmp_tmp >= 0) {
            memset(&Hessian[temp + 1], 0, (unsigned int)(((idxEndIneq_tmp_tmp +
                      temp) - temp) + 1) * sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    memcpy(&dv[0], &TrialState->delta_x[0], 14U * sizeof(double));
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexLB[k] - 1];
      s = (TrialState->xstarsqp[WorkingSet->indexLB[k] - 1] + oldDirIdx) -
        lb[WorkingSet->indexLB[k] - 1];
      if (s < 0.0) {
        dv[WorkingSet->indexLB[k] - 1] = oldDirIdx - s;
        TrialState->xstar[WorkingSet->indexLB[k] - 1] -= s;
      }
    }

    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexUB[k] - 1];
      s = (ub[WorkingSet->indexUB[k] - 1] - TrialState->xstarsqp
           [WorkingSet->indexUB[k] - 1]) - oldDirIdx;
      if (s < 0.0) {
        dv[WorkingSet->indexUB[k] - 1] = oldDirIdx + s;
        TrialState->xstar[WorkingSet->indexUB[k] - 1] += s;
      }
    }

    memcpy(&TrialState->delta_x[0], &dv[0], 14U * sizeof(double));
  }

  return stepSuccess;
}

/*
 * Arguments    : q_struct_T *Flags
 *                h_struct_T *memspace
 *                b_struct_T *MeritFunction
 *                const m_struct_T *WorkingSet
 *                l_struct_T *TrialState
 *                f_struct_T *QRManager
 *                const double lb[15]
 *                const double ub[15]
 * Return Type  : void
 */
static void b_test_exit(q_struct_T *Flags, h_struct_T *memspace, b_struct_T
  *MeritFunction, const m_struct_T *WorkingSet, l_struct_T *TrialState,
  f_struct_T *QRManager, const double lb[15], const double ub[15])
{
  double d;
  double optimRelativeFactor;
  double s;
  double smax;
  int b_i;
  int b_k;
  int i;
  int i1;
  int idx_max;
  int k;
  int mFixed;
  int nVar;
  bool dxTooSmall;
  bool exitg1;
  bool isFeasible;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  i = (unsigned char)((WorkingSet->sizes[0] + WorkingSet->sizes[3]) +
                      WorkingSet->sizes[4]);
  if (i - 1 >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (unsigned
            int)i * sizeof(double));
  }

  computeGradLag(TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  optimRelativeFactor = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (optimRelativeFactor >= 1.7976931348623157E+308) {
    optimRelativeFactor = 1.0;
  }

  smax = 0.0;
  i1 = (unsigned char)WorkingSet->sizes[3];
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet->indexLB[idx_max] - 1;
    smax = fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }

  i1 = (unsigned char)WorkingSet->sizes[4];
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet->indexUB[idx_max] - 1;
    smax = fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  }

  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  dxTooSmall = true;
  smax = 0.0;
  i1 = (unsigned char)WorkingSet->nVar;
  idx_max = 0;
  exitg1 = false;
  while ((!exitg1) && (idx_max <= i1 - 1)) {
    d = fabs(TrialState->gradLag[idx_max]);
    dxTooSmall = (d < 1.7976931348623157E+308);
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = fmax(smax, d);
      idx_max++;
    }
  }

  MeritFunction->nlpDualFeasError = smax;
  if (!dxTooSmall) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = computeComplError(TrialState->xstarsqp,
      WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
      WorkingSet->sizes[4], ub, TrialState->lambdaStopTest, WorkingSet->sizes[0]
      + 1);
    d = fmax(smax, MeritFunction->nlpComplError);
    MeritFunction->firstOrderOpt = d;
    if (TrialState->sqpIterations > 1) {
      double nlpComplErrorTmp;
      b_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                       TrialState->grad, WorkingSet->indexFixed,
                       WorkingSet->sizes[0], WorkingSet->indexLB,
                       WorkingSet->sizes[3], WorkingSet->indexUB,
                       WorkingSet->sizes[4], TrialState->lambdaStopTestPrev);
      smax = 0.0;
      idx_max = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_max <= i1 - 1)) {
        s = fabs(memspace->workspace_double[idx_max]);
        if (s >= 1.7976931348623157E+308) {
          exitg1 = true;
        } else {
          smax = fmax(smax, s);
          idx_max++;
        }
      }

      nlpComplErrorTmp = computeComplError(TrialState->xstarsqp,
        WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
        WorkingSet->sizes[4], ub, TrialState->lambdaStopTestPrev,
        WorkingSet->sizes[0] + 1);
      s = fmax(smax, nlpComplErrorTmp);
      if (s < d) {
        MeritFunction->nlpDualFeasError = smax;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = s;
        if (i - 1 >= 0) {
          memcpy(&TrialState->lambdaStopTest[0], &TrialState->
                 lambdaStopTestPrev[0], (unsigned int)i * sizeof(double));
        }
      } else if (i - 1 >= 0) {
        memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
               (unsigned int)i * sizeof(double));
      }
    } else if (i - 1 >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (unsigned int)i * sizeof(double));
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-9 *
                       optimRelativeFactor) && (MeritFunction->nlpComplError <=
         1.0E-9 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        bool guard1;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          idx_max = 0;
          exitg1 = false;
          while ((!exitg1) && (idx_max <= i1 - 1)) {
            if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[idx_max])) <= fabs
                (TrialState->delta_x[idx_max])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx_max++;
            }
          }

          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              nVar = WorkingSet->nActiveConstr - 1;
              if (WorkingSet->nActiveConstr > 0) {
                int rankR;
                for (k = 0; k <= nVar; k++) {
                  TrialState->lambda[k] = 0.0;
                  idx_max = k << 4;
                  rankR = 31 * k;
                  for (b_k = 0; b_k < i1; b_k++) {
                    QRManager->QR[rankR + b_k] = WorkingSet->ATwset[idx_max +
                      b_k];
                  }
                }

                QRManager->usedPivoting = true;
                QRManager->mrows = WorkingSet->nVar;
                QRManager->ncols = WorkingSet->nActiveConstr;
                nVar = WorkingSet->nVar;
                b_k = WorkingSet->nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }

                QRManager->minRowCol = b_k;
                xgeqp3(QRManager->QR, WorkingSet->nVar,
                       WorkingSet->nActiveConstr, QRManager->jpvt,
                       QRManager->tau);
                computeQ_(QRManager, WorkingSet->nVar);
                nVar = WorkingSet->nVar;
                idx_max = WorkingSet->nActiveConstr;
                if (nVar >= idx_max) {
                  idx_max = nVar;
                }

                smax = fabs(QRManager->QR[0]) * fmin(1.4901161193847656E-8,
                  (double)idx_max * 2.2204460492503131E-16);
                rankR = 0;
                nVar = 0;
                while ((rankR < b_k) && (fabs(QRManager->QR[nVar]) > smax)) {
                  rankR++;
                  nVar += 32;
                }

                c_xgemv(WorkingSet->nVar, WorkingSet->nVar, QRManager->Q,
                        TrialState->grad, memspace->workspace_double);
                if (rankR != 0) {
                  for (k = rankR; k >= 1; k--) {
                    nVar = (k + (k - 1) * 31) - 1;
                    memspace->workspace_double[k - 1] /= QRManager->QR[nVar];
                    for (b_i = 0; b_i <= k - 2; b_i++) {
                      idx_max = (k - b_i) - 2;
                      memspace->workspace_double[idx_max] -=
                        memspace->workspace_double[k - 1] * QRManager->QR[(nVar
                        - b_i) - 1];
                    }
                  }
                }

                nVar = WorkingSet->nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }

                nVar = (unsigned char)b_k;
                for (idx_max = 0; idx_max < nVar; idx_max++) {
                  TrialState->lambda[QRManager->jpvt[idx_max] - 1] =
                    memspace->workspace_double[idx_max];
                }

                nVar = WorkingSet->sizes[0] + 1;
                for (idx_max = nVar; idx_max <= mFixed; idx_max++) {
                  TrialState->lambda[idx_max - 1] = -TrialState->lambda[idx_max
                    - 1];
                }

                sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                             WorkingSet->sizes, WorkingSet->isActiveIdx,
                             WorkingSet->Wid, WorkingSet->Wlocalidx,
                             memspace->workspace_double);
                b_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                                 TrialState->grad, WorkingSet->indexFixed,
                                 WorkingSet->sizes[0], WorkingSet->indexLB,
                                 WorkingSet->sizes[3], WorkingSet->indexUB,
                                 WorkingSet->sizes[4], TrialState->lambda);
                s = 0.0;
                idx_max = 0;
                exitg1 = false;
                while ((!exitg1) && (idx_max <= i1 - 1)) {
                  d = fabs(memspace->workspace_double[idx_max]);
                  if (d >= 1.7976931348623157E+308) {
                    exitg1 = true;
                  } else {
                    s = fmax(s, d);
                    idx_max++;
                  }
                }

                smax = computeComplError(TrialState->xstarsqp,
                  WorkingSet->indexLB, WorkingSet->sizes[3], lb,
                  WorkingSet->indexUB, WorkingSet->sizes[4], ub,
                  TrialState->lambda, WorkingSet->sizes[0] + 1);
                if ((s <= 1.0E-9 * optimRelativeFactor) && (smax <= 1.0E-9 *
                     optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = s;
                  MeritFunction->nlpComplError = smax;
                  MeritFunction->firstOrderOpt = fmax(s, smax);
                  if (i - 1 >= 0) {
                    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0],
                           (unsigned int)i * sizeof(double));
                  }

                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TrialState->sqpIterations >= 800 || toc() >= first_opt_iter_max_time) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 4000 || toc() >= first_opt_iter_max_time) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

/*
 * Arguments    : double *outTime_tv_nsec
 * Return Type  : double
 */
static double b_timeKeeper(double *outTime_tv_nsec)
{
  double outTime_tv_sec;
  outTime_tv_sec = savedTime.tv_sec;
  *outTime_tv_nsec = savedTime.tv_nsec;
  return outTime_tv_sec;
}

/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const double A[961]
 *                int ia0
 *                const double B[496]
 *                double C[961]
 * Return Type  : void
 */
static void b_xgemm(int m, int n, int k, const double A[961], int ia0, const
                    double B[496], double C[961])
{
  int cr;
  int ic;
  int w;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = 31 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 31) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    br = -1;
    for (cr = 0; cr <= lastColC; cr += 31) {
      int ar;
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (w = 0; w < k; w++) {
          temp += A[(w + ar) - 1] * B[(w + br) + 1];
        }

        C[ic - 1] += temp;
        ar += 31;
      }

      br += 31;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[169]
 *                int lda
 *                const double x[14]
 *                double y[13]
 * Return Type  : void
 */
static void b_xgemv(int m, int n, const double A[169], int lda, const double x
                    [14], double y[13])
{
  int ia;
  int iac;
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = (unsigned char)m;
    if (i - 1 >= 0) {
      memset(&y[0], 0, (unsigned int)i * sizeof(double));
    }

    ix = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y[i2] += A[ia - 1] * x[ix];
      }

      ix++;
    }
  }
}

/*
 * Arguments    : double A[729]
 *                int m
 *                int n
 *                int jpvt[27]
 *                double tau[27]
 * Return Type  : void
 */
static void b_xgeqp3(double A[729], int m, int n, int jpvt[27], double tau[27])
{
  double vn1[27];
  double vn2[27];
  double work[27];
  double temp;
  int b_i;
  int k;
  int minmn_tmp;
  int pvt;
  if (m <= n) {
    minmn_tmp = m;
  } else {
    minmn_tmp = n;
  }

  memset(&tau[0], 0, 27U * sizeof(double));
  if (minmn_tmp < 1) {
    for (pvt = 0; pvt < n; pvt++) {
      jpvt[pvt] = pvt + 1;
    }
  } else {
    int i;
    int ix;
    int iy;
    int nfxd;
    int temp_tmp;
    nfxd = 0;
    for (pvt = 0; pvt < n; pvt++) {
      if (jpvt[pvt] != 0) {
        nfxd++;
        if (pvt + 1 != nfxd) {
          ix = pvt * 27;
          iy = (nfxd - 1) * 27;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            i = iy + k;
            A[temp_tmp] = A[i];
            A[i] = temp;
          }

          jpvt[pvt] = jpvt[nfxd - 1];
          jpvt[nfxd - 1] = pvt + 1;
        } else {
          jpvt[pvt] = pvt + 1;
        }
      } else {
        jpvt[pvt] = pvt + 1;
      }
    }

    if (nfxd > minmn_tmp) {
      nfxd = minmn_tmp;
    }

    b_qrf(A, m, n, nfxd, tau);
    if (nfxd < minmn_tmp) {
      double d;
      memset(&work[0], 0, 27U * sizeof(double));
      memset(&vn1[0], 0, 27U * sizeof(double));
      memset(&vn2[0], 0, 27U * sizeof(double));
      i = nfxd + 1;
      for (pvt = i; pvt <= n; pvt++) {
        d = c_xnrm2(m - nfxd, A, (nfxd + (pvt - 1) * 27) + 1);
        vn1[pvt - 1] = d;
        vn2[pvt - 1] = d;
      }

      for (b_i = i; b_i <= minmn_tmp; b_i++) {
        double s;
        int ii;
        int ip1;
        int mmi;
        int nmi;
        ip1 = b_i + 1;
        iy = (b_i - 1) * 27;
        ii = (iy + b_i) - 1;
        nmi = (n - b_i) + 1;
        mmi = m - b_i;
        if (nmi < 1) {
          nfxd = -2;
        } else {
          nfxd = -1;
          if (nmi > 1) {
            temp = fabs(vn1[b_i - 1]);
            for (k = 2; k <= nmi; k++) {
              s = fabs(vn1[(b_i + k) - 2]);
              if (s > temp) {
                nfxd = k - 2;
                temp = s;
              }
            }
          }
        }

        pvt = b_i + nfxd;
        if (pvt + 1 != b_i) {
          ix = pvt * 27;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            nfxd = iy + k;
            A[temp_tmp] = A[nfxd];
            A[nfxd] = temp;
          }

          nfxd = jpvt[pvt];
          jpvt[pvt] = jpvt[b_i - 1];
          jpvt[b_i - 1] = nfxd;
          vn1[pvt] = vn1[b_i - 1];
          vn2[pvt] = vn2[b_i - 1];
        }

        if (b_i < m) {
          temp = A[ii];
          d = b_xzlarfg(mmi + 1, &temp, A, ii + 2);
          tau[b_i - 1] = d;
          A[ii] = temp;
        } else {
          d = 0.0;
          tau[b_i - 1] = 0.0;
        }

        if (b_i < n) {
          temp = A[ii];
          A[ii] = 1.0;
          b_xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, ii + 28, work);
          A[ii] = temp;
        }

        for (pvt = ip1; pvt <= n; pvt++) {
          nfxd = b_i + (pvt - 1) * 27;
          d = vn1[pvt - 1];
          if (d != 0.0) {
            temp = fabs(A[nfxd - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }

            s = d / vn2[pvt - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < m) {
                d = c_xnrm2(mmi, A, nfxd + 1);
                vn1[pvt - 1] = d;
                vn2[pvt - 1] = d;
              } else {
                vn1[pvt - 1] = 0.0;
                vn2[pvt - 1] = 0.0;
              }
            } else {
              vn1[pvt - 1] = d * sqrt(temp);
            }
          }
        }
      }
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[16]
 * Return Type  : double
 */
static double b_xnrm2(int n, const double x[16])
{
  double y;
  int k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[0]);
    } else {
      double scale;
      int i;
      scale = 3.3121686421112381E-170;
      i = (unsigned char)n;
      for (k = 0; k < i; k++) {
        double absxk;
        absxk = fabs(x[k]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/*
 * Arguments    : int n
 *                double A[729]
 * Return Type  : int
 */
static int b_xpotrf(int n, double A[729])
{
  int ia;
  int iac;
  int info;
  int j;
  int nmj;
  bool exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= n - 1)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    idxA1j = j * 27;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (nmj = 0; nmj < j; nmj++) {
        c = A[idxA1j + nmj];
        ssq += c * c;
      }
    }

    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < n) {
        int i;
        int ia0;
        int idxAjjp1;
        nmj = (n - j) - 2;
        ia0 = idxA1j + 28;
        idxAjjp1 = idxAjj + 28;
        if ((j != 0) && (nmj + 1 != 0)) {
          i = (idxA1j + 27 * nmj) + 28;
          for (iac = ia0; iac <= i; iac += 27) {
            int i1;
            c = 0.0;
            i1 = (iac + j) - 1;
            for (ia = iac; ia <= i1; ia++) {
              c += A[ia - 1] * A[(idxA1j + ia) - iac];
            }

            i1 = (idxAjj + div_nde_s32_floor((iac - idxA1j) - 28, 27) * 27) + 27;
            A[i1] -= c;
          }
        }

        ssq = 1.0 / ssq;
        i = (idxAjj + 27 * nmj) + 28;
        for (nmj = idxAjjp1; nmj <= i; nmj += 27) {
          A[nmj - 1] *= ssq;
        }
      }

      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/*
 * Arguments    : int m
 *                int n
 *                int iv0
 *                double tau
 *                double C[729]
 *                int ic0
 *                double work[27]
 * Return Type  : void
 */
static void b_xzlarf(int m, int n, int iv0, double tau, double C[729], int ic0,
                     double work[27])
{
  int i;
  int ia;
  int iac;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    bool exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      int exitg1;
      i = ic0 + lastc * 27;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    double c;
    int b_i;
    if (lastc + 1 != 0) {
      if (lastc >= 0) {
        memset(&work[0], 0, (unsigned int)(lastc + 1) * sizeof(double));
      }

      b_i = ic0 + 27 * lastc;
      for (iac = ic0; iac <= b_i; iac += 27) {
        c = 0.0;
        i = (iac + lastv) - 1;
        for (ia = iac; ia <= i; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }

        i = div_nde_s32_floor(iac - ic0, 27);
        work[i] += c;
      }
    }

    if (-tau != 0.0) {
      i = ic0;
      for (iac = 0; iac <= lastc; iac++) {
        c = work[iac];
        if (c != 0.0) {
          c *= -tau;
          b_i = lastv + i;
          for (ia = i; ia < b_i; ia++) {
            C[ia - 1] += C[((iv0 + ia) - i) - 1] * c;
          }
        }

        i += 27;
      }
    }
  }
}

/*
 * Arguments    : int n
 *                double *alpha1
 *                double x[729]
 *                int ix0
 * Return Type  : double
 */
static double b_xzlarfg(int n, double *alpha1, double x[729], int ix0)
{
  double tau;
  int k;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = c_xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      double beta1;
      beta1 = rt_hypotd(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        beta1 = -beta1;
      }

      if (fabs(beta1) < 1.0020841800044864E-292) {
        int i;
        int knt;
        knt = 0;
        i = (ix0 + n) - 2;
        do {
          knt++;
          for (k = ix0; k <= i; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

        beta1 = rt_hypotd(*alpha1, c_xnrm2(n - 1, x, ix0));
        if (*alpha1 >= 0.0) {
          beta1 = -beta1;
        }

        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }

        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }

        *alpha1 = beta1;
      } else {
        int i;
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        i = (ix0 + n) - 2;
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }

        *alpha1 = beta1;
      }
    }
  }

  return tau;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void c_CoderTimeAPI_callCoderClockGe(void)
{
  freq_not_empty = false;
}

/*
 * Arguments    : double workspace[14]
 *                int nVar
 *                const double grad[14]
 *                const int finiteFixed[14]
 *                int mFixed
 *                const int finiteLB[14]
 *                int mLB
 *                const int finiteUB[14]
 *                int mUB
 *                const double lambda[27]
 * Return Type  : void
 */
static void c_computeGradLag(double workspace[14], int nVar, const double grad
  [14], const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB,
  const int finiteUB[14], int mUB, const double lambda[27])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }

  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }

  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }

  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : const double u_in[15]
 *                double p
 *                double q
 *                double r
 *                double K_p_T
 *                double K_p_M
 *                double m
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double Cm_zero
 *                double CL_aileron
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double Beta
 *                double accelerations_array[6]
 * Return Type  : void
 */
static void c_compute_acc_cascaded_nonlinea(const double u_in[15], double p,
  double q, double r, double K_p_T, double K_p_M, double m, double I_xx, double
  I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4, double l_z,
  double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero,
  double CL_aileron, double rho, double V, double S, double wing_chord, double
  flight_path_angle, double Beta, double accelerations_array[6])
{
  double accelerations_array_tmp;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t23;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t3;
  double t31;
  double t32;
  double t33;
  double t34;
  double t35;
  double t36;
  double t37;
  double t38;
  double t4;
  double t40;
  double t41;
  double t42;
  double t42_tmp;
  double t43;
  double t43_tmp;
  double t44;
  double t45;
  double t46;
  double t47;
  double t49;
  double t5;
  double t53;
  double t54_tmp_tmp;
  double t56;
  double t57;
  double t58;
  double t6;
  double t7;
  double t8;
  double t9;

  /* COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_V2 */
  /*     ACCELERATIONS_ARRAY = COMPUTE_ACC_NONLINEAR_CONTROL_RF_W_AILERONS_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1,Omega_2,Omega_3,Omega_4,Phi,S,Theta,V,B_1,B_2,B_3,B_4,DELTA_AILERONS,FLIGHT_PATH_ANGLE,G_1,G_2,G_3,G_4,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     30-Nov-2023 18:08:53 */
  t3 = cos(u_in[13]);
  t4 = sin(Beta);
  t5 = cos(u_in[12]);
  t6 = sin(u_in[13]);
  t7 = cos(u_in[4]);
  t8 = cos(u_in[5]);
  t9 = cos(u_in[6]);
  t10 = cos(u_in[7]);
  t11 = sin(u_in[12]);
  t12 = cos(u_in[8]);
  t13 = cos(u_in[9]);
  t14 = cos(u_in[10]);
  t15 = cos(u_in[11]);
  t16 = sin(u_in[4]);
  t17 = sin(u_in[5]);
  t18 = sin(u_in[6]);
  t19 = sin(u_in[7]);
  t20 = sin(u_in[8]);
  t21 = sin(u_in[9]);
  t22 = sin(u_in[10]);
  t23 = sin(u_in[11]);
  t25 = u_in[0] * u_in[0];
  t26 = u_in[1] * u_in[1];
  t27 = u_in[2] * u_in[2];
  t28 = u_in[3] * u_in[3];
  t29 = V * V;
  t31 = 1.0 / m;
  t32 = u_in[12] - flight_path_angle;
  t33 = K_p_T * t16 * t25;
  t34 = K_p_T * t17 * t26;
  t35 = K_p_T * t18 * t27;
  t36 = K_p_T * t19 * t28;
  t37 = K_p_T * t7;
  t40 = t37 * t12 * t25;
  t38 = K_p_T * t8;
  t41 = t38 * t13 * t26;
  t42_tmp = K_p_T * t9;
  t42 = t42_tmp * t14 * t27;
  t43_tmp = K_p_T * t10;
  t43 = t43_tmp * t15 * t28;
  t44 = t37 * t20 * t25;
  t45 = t38 * t21 * t26;
  t46 = t42_tmp * t22 * t27;
  t47 = t43_tmp * t23 * t28;
  t37 = cos(t32);
  t38 = sin(t32);
  t53 = ((t33 + t34) + t35) + t36;
  t56 = ((t40 + t41) + t42) + t43;
  t57 = ((t44 + t45) + t46) + t47;
  t42_tmp = Cl_alpha * S * rho * t29 * t32;
  t49 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t32 * t32);
  t54_tmp_tmp = S * rho;
  t43_tmp = t54_tmp_tmp * cos(Beta) * t29;
  t58 = t42_tmp * t37 / 2.0 + t43_tmp * t38 * t49 / 2.0;
  accelerations_array_tmp = t3 * t11;
  t42_tmp = t42_tmp * t38 / 2.0 - t43_tmp * t37 * t49 / 2.0;
  t43_tmp = t54_tmp_tmp * t4;
  accelerations_array[0] = -t31 * (((((t5 * t53 - t5 * t42_tmp) +
    accelerations_array_tmp * t56) + accelerations_array_tmp * t58) - t6 * t11 *
    t57) + t43_tmp * t6 * t11 * t29 * t49 / 2.0);
  accelerations_array[1] = t31 * (((t3 * t57 + t6 * t56) + t6 * t58) -
    t54_tmp_tmp * t3 * t4 * t29 * t49 / 2.0);
  accelerations_array_tmp = t3 * t5;
  accelerations_array[2] = -t31 * (((((-t11 * t53 + t11 * t42_tmp) +
    accelerations_array_tmp * t56) + accelerations_array_tmp * t58) - t5 * t6 *
    t57) + t43_tmp * t5 * t6 * t29 * t49 / 2.0) + 9.81;
  accelerations_array[3] = ((((((((((((((l_1 * t40 - l_1 * t41) - l_2 * t42) +
    l_2 * t43) + l_z * t44) + l_z * t45) + l_z * t46) + l_z * t47) + I_yy * q *
    r) - I_zz * q * r) + K_p_M * t16 * t25) - K_p_M * t17 * t26) + K_p_M * t18 *
    t27) - K_p_M * t19 * t28) + CL_aileron * S * u_in[14] * rho * t29 / 2.0) /
    I_xx;
  accelerations_array_tmp = I_xx * p;
  t42_tmp = K_p_M * t7;
  t43_tmp = K_p_M * t8;
  t38 = K_p_M * t9;
  t37 = K_p_M * t10;
  accelerations_array[4] = ((((((((((((((l_4 * t40 - l_3 * t42) + l_4 * t41) -
    l_3 * t43) + l_z * t33) + l_z * t34) + l_z * t35) + l_z * t36) -
    accelerations_array_tmp * r) + I_zz * p * r) - t42_tmp * t20 * t25) +
    t43_tmp * t21 * t26) - t38 * t22 * t27) + t37 * t23 * t28) + t54_tmp_tmp *
    t29 * wing_chord * (Cm_zero + Cm_alpha * t32) / 2.0) / I_yy;
  accelerations_array[5] = -(((((((((((((l_1 * t33 - l_1 * t34) - l_2 * t35) +
    l_2 * t36) - l_4 * t44) + l_3 * t46) - l_4 * t45) + l_3 * t47) -
    accelerations_array_tmp * q) + I_yy * p * q) - t42_tmp * t12 * t25) +
    t43_tmp * t13 * t26) - t38 * t14 * t27) + t37 * t15 * t28) / I_zz;
}

/*
 * Arguments    : const b_captured_var *dv_global
 *                const captured_var *Beta
 *                const captured_var *CL_aileron
 *                const captured_var *Cd_zero
 *                const captured_var *Cl_alpha
 *                const captured_var *Cm_zero
 *                const captured_var *Cm_alpha
 *                const captured_var *I_xx
 *                const captured_var *I_yy
 *                const captured_var *I_zz
 *                const captured_var *K_Cd
 *                const captured_var *K_p_M
 *                const captured_var *K_p_T
 *                const captured_var *S
 *                const captured_var *V
 *                const captured_var *W_act_phi
 *                const captured_var *W_act_theta
 *                const captured_var *W_act_motor
 *                const captured_var *W_dv_1
 *                const captured_var *W_dv_2
 *                const captured_var *W_dv_3
 *                const captured_var *W_dv_4
 *                const captured_var *W_dv_5
 *                const captured_var *W_dv_6
 *                const captured_var *W_act_tilt_el
 *                const captured_var *W_act_tilt_az
 *                const captured_var *W_act_ailerons
 *                const captured_var *desired_el_value
 *                const captured_var *desired_az_value
 *                const captured_var *desired_phi_value
 *                const captured_var *desired_theta_value
 *                const captured_var *desired_motor_value
 *                const captured_var *desired_ailerons_value
 *                const captured_var *flight_path_angle
 *                const captured_var *gain_el
 *                const captured_var *gain_az
 *                const captured_var *gain_phi
 *                const captured_var *gain_theta
 *                const captured_var *gain_motor
 *                const captured_var *gain_ailerons
 *                const captured_var *gamma_quadratic_du
 *                const captured_var *l_1
 *                const captured_var *l_2
 *                const captured_var *l_3
 *                const captured_var *l_4
 *                const captured_var *l_z
 *                const captured_var *m
 *                const captured_var *p
 *                const captured_var *q
 *                const captured_var *r
 *                const captured_var *rho
 *                const captured_var *wing_chord
 *                const double u_in[15]
 *                double gradient[15]
 * Return Type  : double
 */
static double c_compute_cost_and_gradient_fir(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *S, const captured_var *V, const
  captured_var *W_act_phi, const captured_var *W_act_theta, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_phi_value, const captured_var *desired_theta_value,
  const captured_var *desired_motor_value, const captured_var
  *desired_ailerons_value, const captured_var *flight_path_angle, const
  captured_var *gain_el, const captured_var *gain_az, const captured_var
  *gain_phi, const captured_var *gain_theta, const captured_var *gain_motor,
  const captured_var *gain_ailerons, const captured_var *gamma_quadratic_du,
  const captured_var *l_1, const captured_var *l_2, const captured_var *l_3,
  const captured_var *l_4, const captured_var *l_z, const captured_var *m, const
  captured_var *p, const captured_var *q, const captured_var *r, const
  captured_var *rho, const captured_var *wing_chord, const double u_in[15],
  double gradient[15])
{
  double b_CL_aileron;
  double b_Cl_alpha;
  double b_Cm_alpha;
  double b_Cm_zero;
  double b_I_yy;
  double b_I_zz;
  double b_K_p_M;
  double b_K_p_T;
  double b_S;
  double b_W_act_ailerons;
  double b_W_act_phi;
  double b_W_act_tilt_az;
  double b_W_act_tilt_el;
  double b_W_dv_2;
  double b_W_dv_6;
  double b_desired_ailerons_value;
  double b_desired_motor_value;
  double b_desired_phi_value;
  double b_desired_theta_value;
  double b_flight_path_angle;
  double b_gain_ailerons;
  double b_gain_az;
  double b_gain_el;
  double b_gain_motor;
  double b_gain_phi;
  double b_gain_theta;
  double b_gamma_quadratic_du;
  double b_l_1;
  double b_l_2;
  double b_l_3;
  double b_l_4;
  double b_l_z;
  double b_m;
  double b_p;
  double b_q;
  double b_r;
  double b_rho;
  double b_wing_chord;
  double cost;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double t10;
  double t102;
  double t103;
  double t104;
  double t105;
  double t106;
  double t107;
  double t108;
  double t109;
  double t11;
  double t115;
  double t116;
  double t117;
  double t118;
  double t119;
  double t12;
  double t120;
  double t121;
  double t122;
  double t123;
  double t124;
  double t125;
  double t126;
  double t127;
  double t128;
  double t129;
  double t13;
  double t130;
  double t131;
  double t133;
  double t135;
  double t137;
  double t14;
  double t140;
  double t141;
  double t143;
  double t15;
  double t155;
  double t156;
  double t158;
  double t16;
  double t169;
  double t17;
  double t171;
  double t172;
  double t173;
  double t174;
  double t179;
  double t18;
  double t180;
  double t181;
  double t182;
  double t183;
  double t184;
  double t185;
  double t186;
  double t188;
  double t19;
  double t192;
  double t194;
  double t195;
  double t197;
  double t199;
  double t199_tmp;
  double t2;
  double t20;
  double t200;
  double t200_tmp;
  double t203;
  double t203_tmp_tmp;
  double t204;
  double t209;
  double t21;
  double t216;
  double t22;
  double t220;
  double t226;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t32;
  double t33;
  double t34;
  double t37;
  double t4;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t53;
  double t54;
  double t55;
  double t56;
  double t57;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t69;
  double t7;
  double t8;
  double t80;
  double t81;
  double t82;
  double t84;
  double t86;
  double t88;
  double t9;
  double t92;
  double t93;
  double t94;
  double t95;
  double t96;
  double t97;
  double t98;
  double t99;
  dv_global_1 = dv_global->contents[0];
  dv_global_2 = dv_global->contents[1];
  dv_global_3 = dv_global->contents[2];
  dv_global_4 = dv_global->contents[3];
  dv_global_5 = dv_global->contents[4];
  dv_global_6 = dv_global->contents[5];
  t4 = Beta->contents;
  b_CL_aileron = CL_aileron->contents;
  t216 = Cd_zero->contents;
  b_Cl_alpha = Cl_alpha->contents;
  b_Cm_zero = Cm_zero->contents;
  b_Cm_alpha = Cm_alpha->contents;
  t188 = I_xx->contents;
  b_I_yy = I_yy->contents;
  b_I_zz = I_zz->contents;
  t209 = K_Cd->contents;
  b_K_p_M = K_p_M->contents;
  b_K_p_T = K_p_T->contents;
  b_S = S->contents;
  t179 = V->contents;
  b_W_act_phi = W_act_phi->contents;
  t174 = W_act_theta->contents;
  t183 = W_act_motor->contents;
  t171 = W_dv_1->contents;
  b_W_dv_2 = W_dv_2->contents;
  t158 = W_dv_3->contents;
  t81 = W_dv_4->contents;
  t82 = W_dv_5->contents;
  b_W_dv_6 = W_dv_6->contents;
  b_W_act_tilt_el = W_act_tilt_el->contents;
  b_W_act_tilt_az = W_act_tilt_az->contents;
  b_W_act_ailerons = W_act_ailerons->contents;
  t220 = desired_el_value->contents;
  t226 = desired_az_value->contents;
  b_desired_phi_value = desired_phi_value->contents;
  b_desired_theta_value = desired_theta_value->contents;
  b_desired_motor_value = desired_motor_value->contents;
  b_desired_ailerons_value = desired_ailerons_value->contents;
  b_flight_path_angle = flight_path_angle->contents;
  b_gain_el = gain_el->contents;
  b_gain_az = gain_az->contents;
  b_gain_phi = gain_phi->contents;
  b_gain_theta = gain_theta->contents;
  b_gain_motor = gain_motor->contents;
  b_gain_ailerons = gain_ailerons->contents;
  b_gamma_quadratic_du = gamma_quadratic_du->contents;
  b_l_1 = l_1->contents;
  b_l_2 = l_2->contents;
  b_l_3 = l_3->contents;
  b_l_4 = l_4->contents;
  b_l_z = l_z->contents;
  b_m = m->contents;
  b_p = p->contents;
  b_q = q->contents;
  b_r = r->contents;
  b_rho = rho->contents;
  b_wing_chord = wing_chord->contents;

  /* COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2 */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     30-Nov-2023 18:08:48 */
  t2 = cos(t4);
  t3 = sin(t4);
  t4 = u_in[13] * b_gain_phi;
  t5 = u_in[12] * b_gain_theta;
  t6 = u_in[4] * b_gain_el;
  t7 = u_in[5] * b_gain_el;
  t8 = u_in[6] * b_gain_el;
  t9 = u_in[7] * b_gain_el;
  t10 = u_in[8] * b_gain_az;
  t11 = u_in[9] * b_gain_az;
  t12 = u_in[10] * b_gain_az;
  t13 = u_in[11] * b_gain_az;
  t14 = b_Cl_alpha * b_Cl_alpha;
  t15 = u_in[0] * u_in[0];
  t16 = u_in[1] * u_in[1];
  t17 = u_in[2] * u_in[2];
  t18 = u_in[3] * u_in[3];
  t19 = t179 * t179;
  t20 = b_W_act_phi * b_W_act_phi;
  t21 = t174 * t174;
  t22 = t183 * t183;
  t23 = t171 * t171;
  t24 = b_W_dv_2 * b_W_dv_2;
  t25 = t158 * t158;
  t26 = t81 * t81;
  t27 = t82 * t82;
  t28 = b_W_dv_6 * b_W_dv_6;
  t29 = b_W_act_tilt_el * b_W_act_tilt_el;
  t30 = b_W_act_tilt_az * b_W_act_tilt_az;
  t31 = b_W_act_ailerons * b_W_act_ailerons;
  t32 = b_gain_motor * b_gain_motor;
  t59 = 1.0 / t188;
  t60 = 1.0 / b_I_yy;
  t61 = 1.0 / b_I_zz;
  t69 = 1.0 / b_m;
  t33 = cos(t4);
  t34 = cos(t5);
  t37 = sin(t4);
  t42 = cos(t6);
  t43 = cos(t7);
  t44 = cos(t8);
  t45 = cos(t9);
  t46 = sin(t5);
  t47 = cos(t10);
  t48 = cos(t11);
  t49 = cos(t12);
  t50 = cos(t13);
  t51 = sin(t6);
  t52 = sin(t7);
  t53 = sin(t8);
  t54 = sin(t9);
  t55 = sin(t10);
  t56 = sin(t11);
  t57 = sin(t12);
  t58 = sin(t13);
  t8 = t220 * (1.0 / b_gain_el);
  t9 = t226 * (1.0 / b_gain_az);
  t7 = b_desired_motor_value * (1.0 / b_gain_motor);
  t80 = b_flight_path_angle - t5;
  t179 = b_K_p_M * t15 * t32;
  t92 = t179 * t51;
  b_W_act_phi = b_K_p_M * t16 * t32;
  t93 = b_W_act_phi * t52;
  t174 = b_K_p_M * t17 * t32;
  t94 = t174 * t53;
  t171 = b_K_p_M * t18 * t32;
  t95 = t171 * t54;
  t183 = b_K_p_T * t15 * t32;
  t96 = t183 * t51;
  b_W_dv_2 = b_K_p_T * t16 * t32;
  t97 = b_W_dv_2 * t52;
  t158 = b_K_p_T * t17 * t32;
  t98 = t158 * t53;
  t81 = b_K_p_T * t18 * t32;
  t99 = t81 * t54;
  t179 *= t42;
  t116 = t179 * t47;
  b_W_act_phi *= t43;
  t117 = b_W_act_phi * t48;
  t174 *= t44;
  t118 = t174 * t49;
  t183 *= t42;
  t119 = t183 * t47;
  t171 *= t45;
  t120 = t171 * t50;
  b_W_dv_2 *= t43;
  t121 = b_W_dv_2 * t48;
  t158 *= t44;
  t122 = t158 * t49;
  t81 *= t45;
  t123 = t81 * t50;
  t124 = t179 * t55;
  t125 = b_W_act_phi * t56;
  t126 = t174 * t57;
  t127 = t183 * t55;
  t128 = t171 * t58;
  t129 = b_W_dv_2 * t56;
  t130 = t158 * t57;
  t131 = t81 * t58;
  t81 = cos(t80);
  t82 = sin(t80);
  t84 = -(t8 * 2.0);
  t86 = -(t9 * 2.0);
  t88 = -(t7 * 2.0);
  t102 = b_l_1 * t96;
  t103 = b_l_1 * t97;
  t104 = b_l_2 * t98;
  t105 = b_l_2 * t99;
  t106 = b_l_z * t96;
  t107 = b_l_z * t97;
  t108 = b_l_z * t98;
  t109 = b_l_z * t99;
  t133 = b_l_4 * t119;
  t135 = b_l_4 * t121;
  t137 = b_l_3 * t122;
  t140 = b_l_4 * t127;
  t141 = b_l_4 * t129;
  t143 = b_l_3 * t131;
  t179 = ((t96 + t97) + t98) + t99;
  t184 = ((t119 + t121) + t122) + t123;
  t185 = ((t127 + t129) + t130) + t131;
  t155 = -(b_l_3 * t123);
  t156 = -(b_l_3 * t130);
  t183 = b_Cl_alpha * b_S;
  b_W_dv_2 = t183 * b_gain_theta * b_rho * t19;
  t158 = b_W_dv_2 * t81 / 2.0;
  t180 = t34 * t179;
  t181 = t46 * t179;
  t194 = t37 * t184;
  t195 = t33 * t185;
  t197 = t34 * t37 * t185;
  t199_tmp = t33 * t34;
  t199 = t199_tmp * t184;
  t200_tmp = t33 * t46;
  t200 = t200_tmp * t184;
  t115 = t216 + t209 * t14 * (t80 * t80);
  b_W_dv_6 = b_S * b_rho;
  b_W_act_phi = b_W_dv_6 * t3 * t19;
  t169 = b_W_act_phi * t33 * t115 / 2.0;
  t174 = b_W_dv_6 * t2 * t19;
  t171 = t174 * t82 * t115 / 2.0;
  t172 = b_W_act_phi * t34 * t37 * t115 / 2.0;
  t173 = b_W_act_phi * t37 * t46 * t115 / 2.0;
  t174 = t174 * t81 * t115 / 2.0;
  b_W_act_phi = t183 * b_rho * t19 * t80;
  t182 = b_W_act_phi * t81 / 2.0 + t171;
  t183 = b_W_act_phi * t82 / 2.0 - t174;
  t186 = t37 * t182;
  t192 = t199_tmp * t182;
  t203_tmp_tmp = t209 * b_S * b_gain_theta * b_rho;
  b_W_act_phi = t203_tmp_tmp * t2 * t14 * t19 * t80;
  t203 = ((t158 + b_W_dv_2 * t80 * t82 * -0.5) + b_W_act_phi * t82) +
    b_gain_theta * t174;
  t204 = ((b_W_dv_2 * t82 / 2.0 + t80 * t158) - b_W_act_phi * t81) +
    b_gain_theta * t171;
  b_W_act_phi = t188 * b_p;
  t209 = dv_global_6 - t61 * (((((((((((((b_W_act_phi * b_q - b_I_yy * b_p * b_q)
    + t103) + t104) - t102) - t105) + t116) + t118) + t140) + t141) - t117) -
    t120) + t156) - t143);
  dv_global_6 = b_CL_aileron * b_S;
  t2 = dv_global_4 - t59 * ((((((((((((((b_I_yy * b_q * b_r - b_I_zz * b_q * b_r)
    + t92) + t94) + dv_global_6 * u_in[14] * b_gain_ailerons * b_rho * t19 / 2.0)
    - t93) - t95) + b_l_1 * t119) + b_l_2 * t123) + b_l_z * t127) + b_l_z * t129)
    + b_l_z * t130) + b_l_z * t131) - b_l_1 * t121) - b_l_2 * t122);
  t188 = t46 * t183;
  t216 = dv_global_5 - t60 * ((((((((((((((b_I_zz * b_p * b_r - b_W_act_phi *
    b_r) + t106) + t107) + t108) + t109) + t125) + t128) + t133) + t135) - t124)
    - t126) - t137) + t155) + b_W_dv_6 * t19 * (b_Cm_zero - b_Cm_alpha * t80) *
    b_wing_chord / 2.0);
  t220 = dv_global_2 + t69 * (((t169 + t186) - t194) - t195);
  t226 = (-dv_global_3 + -t69 * (((((t172 - t181) + t188) - t192) - t197) + t199))
    + 9.81;
  b_flight_path_angle = t34 * t183;
  t179 = t200_tmp * t182;
  b_Cl_alpha = t37 * t46 * t185;
  t13 = dv_global_1 + t69 * (((((t173 + t180) - b_flight_path_angle) - t179) +
    t200) - b_Cl_alpha);
  t5 = b_desired_phi_value * (1.0 / b_gain_phi);
  t6 = u_in[13] - t5;
  b_gain_motor = b_desired_theta_value * (1.0 / b_gain_theta);
  t4 = u_in[12] - b_gain_motor;
  b_desired_motor_value = b_desired_ailerons_value * (1.0 / b_gain_ailerons);
  b_W_act_tilt_el = u_in[14] - b_desired_motor_value;
  b_W_act_tilt_az = u_in[0] - t7;
  b_W_act_ailerons = u_in[1] - t7;
  b_m = u_in[2] - t7;
  t158 = u_in[3] - t7;
  t81 = u_in[4] - t8;
  t82 = u_in[5] - t8;
  b_W_dv_6 = u_in[6] - t8;
  t174 = u_in[7] - t8;
  t183 = u_in[8] - t9;
  t171 = u_in[9] - t9;
  b_W_dv_2 = u_in[10] - t9;
  b_W_act_phi = u_in[11] - t9;
  cost = (((((b_gamma_quadratic_du * ((((((((((((((t20 * (t6 * t6) + t21 * (t4 *
    t4)) + t31 * (b_W_act_tilt_el * b_W_act_tilt_el)) + t22 * (b_W_act_tilt_az *
    b_W_act_tilt_az)) + t22 * (b_W_act_ailerons * b_W_act_ailerons)) + t22 *
    (b_m * b_m)) + t22 * (t158 * t158)) + t29 * (t81 * t81)) + t29 * (t82 * t82))
    + t29 * (b_W_dv_6 * b_W_dv_6)) + t29 * (t174 * t174)) + t30 * (t183 * t183))
    + t30 * (t171 * t171)) + t30 * (b_W_dv_2 * b_W_dv_2)) + t30 * (b_W_act_phi *
    b_W_act_phi)) + t28 * (t209 * t209)) + t26 * (t2 * t2)) + t27 * (t216 * t216))
           + t24 * (t220 * t220)) + t23 * (t13 * t13)) + t25 * (t226 * t226);
  b_W_act_ailerons = b_K_p_T * u_in[0];
  b_m = b_K_p_M * u_in[0] * t32;
  t4 = b_W_act_ailerons * b_l_1 * t32;
  t6 = b_W_act_ailerons * b_l_z * t32;
  t7 = b_m * t42;
  t8 = b_W_act_ailerons * b_l_4 * t32 * t42;
  b_W_act_ailerons *= t32;
  t9 = b_W_act_ailerons * t33;
  t10 = b_W_act_ailerons * t34;
  t11 = t9 * t42;
  t12 = b_W_act_ailerons * t37 * t42;
  b_W_act_tilt_az = b_gamma_quadratic_du * t22;
  t82 = t28 * t61 * t209;
  b_W_dv_6 = t26 * t59 * t2;
  b_W_act_tilt_el = t27 * t60 * t216;
  t81 = t24 * t69 * t220;
  t158 = t25 * t69 * t226;
  b_W_dv_2 = t23 * t69 * t13;
  gradient[0] = (((((b_W_act_tilt_az * (u_in[0] * 2.0 + t88) - t82 * ((t4 * t51 *
    -2.0 + t7 * t47 * 2.0) + t8 * t55 * 2.0) * 2.0) - b_W_dv_6 * ((b_m * t51 *
    2.0 + t4 * t42 * t47 * 2.0) + t6 * t42 * t55 * 2.0) * 2.0) - b_W_act_tilt_el
                   * ((t6 * t51 * 2.0 - t7 * t55 * 2.0) + t8 * t47 * 2.0) * 2.0)
                  - t81 * (t12 * t47 * 2.0 + t11 * t55 * 2.0) * 2.0) + t158 *
                 ((b_W_act_ailerons * t46 * t51 * 2.0 - t9 * t34 * t42 * t47 *
                   2.0) + t10 * t37 * t42 * t55 * 2.0) * 2.0) + b_W_dv_2 * ((t10
    * t51 * 2.0 + t11 * t46 * t47 * 2.0) - t12 * t46 * t55 * 2.0) * 2.0;
  b_W_act_ailerons = b_K_p_T * u_in[1];
  b_m = b_K_p_M * u_in[1] * t32;
  t4 = b_W_act_ailerons * b_l_1 * t32;
  t6 = b_W_act_ailerons * b_l_z * t32;
  t7 = b_m * t43;
  t8 = b_W_act_ailerons * b_l_4 * t32 * t43;
  b_W_act_ailerons *= t32;
  t9 = b_W_act_ailerons * t33;
  t10 = b_W_act_ailerons * t34;
  t11 = t9 * t43;
  t12 = b_W_act_ailerons * t37 * t43;
  gradient[1] = (((((b_W_act_tilt_az * (u_in[1] * 2.0 + t88) - t82 * ((t4 * t52 *
    2.0 - t7 * t48 * 2.0) + t8 * t56 * 2.0) * 2.0) + b_W_dv_6 * ((b_m * t52 *
    2.0 + t4 * t43 * t48 * 2.0) - t6 * t43 * t56 * 2.0) * 2.0) - b_W_act_tilt_el
                   * ((t6 * t52 * 2.0 + t7 * t56 * 2.0) + t8 * t48 * 2.0) * 2.0)
                  - t81 * (t12 * t48 * 2.0 + t11 * t56 * 2.0) * 2.0) + t158 *
                 ((b_W_act_ailerons * t46 * t52 * 2.0 - t9 * t34 * t43 * t48 *
                   2.0) + t10 * t37 * t43 * t56 * 2.0) * 2.0) + b_W_dv_2 * ((t10
    * t52 * 2.0 + t11 * t46 * t48 * 2.0) - t12 * t46 * t56 * 2.0) * 2.0;
  b_W_act_ailerons = b_K_p_T * u_in[2];
  b_m = b_K_p_M * u_in[2] * t32;
  t4 = b_W_act_ailerons * b_l_2 * t32;
  t6 = b_W_act_ailerons * b_l_z * t32;
  t7 = b_m * t44;
  t8 = b_W_act_ailerons * b_l_3 * t32 * t44;
  b_W_act_ailerons *= t32;
  t9 = b_W_act_ailerons * t33;
  t10 = b_W_act_ailerons * t34;
  t11 = t9 * t44;
  t12 = b_W_act_ailerons * t37 * t44;
  gradient[2] = (((((b_W_act_tilt_az * (u_in[2] * 2.0 + t88) - t82 * ((t4 * t53 *
    2.0 + t7 * t49 * 2.0) - t8 * t57 * 2.0) * 2.0) - b_W_dv_6 * ((b_m * t53 *
    2.0 - t4 * t44 * t49 * 2.0) + t6 * t44 * t57 * 2.0) * 2.0) + b_W_act_tilt_el
                   * ((t6 * t53 * -2.0 + t7 * t57 * 2.0) + t8 * t49 * 2.0) * 2.0)
                  - t81 * (t12 * t49 * 2.0 + t11 * t57 * 2.0) * 2.0) + t158 *
                 ((b_W_act_ailerons * t46 * t53 * 2.0 - t9 * t34 * t44 * t49 *
                   2.0) + t10 * t37 * t44 * t57 * 2.0) * 2.0) + b_W_dv_2 * ((t10
    * t53 * 2.0 + t11 * t46 * t49 * 2.0) - t12 * t46 * t57 * 2.0) * 2.0;
  b_W_act_ailerons = b_K_p_T * u_in[3];
  b_m = b_K_p_M * u_in[3] * t32;
  t4 = b_W_act_ailerons * b_l_2 * t32;
  t6 = b_W_act_ailerons * b_l_z * t32;
  t7 = b_m * t45;
  t8 = b_W_act_ailerons * b_l_3 * t32 * t45;
  b_W_act_ailerons *= t32;
  t9 = b_W_act_ailerons * t33;
  t10 = b_W_act_ailerons * t34;
  t11 = t9 * t45;
  t12 = b_W_act_ailerons * t37 * t45;
  gradient[3] = (((((b_W_act_tilt_az * (u_in[3] * 2.0 + t88) + t82 * ((t4 * t54 *
    2.0 + t7 * t50 * 2.0) + t8 * t58 * 2.0) * 2.0) - b_W_dv_6 * ((b_m * t54 *
    -2.0 + t4 * t45 * t50 * 2.0) + t6 * t45 * t58 * 2.0) * 2.0) -
                   b_W_act_tilt_el * ((t6 * t54 * 2.0 + t7 * t58 * 2.0) - t8 *
    t50 * 2.0) * 2.0) - t81 * (t12 * t50 * 2.0 + t11 * t58 * 2.0) * 2.0) + t158 *
                 ((b_W_act_ailerons * t46 * t54 * 2.0 - t9 * t34 * t45 * t50 *
                   2.0) + t10 * t37 * t45 * t58 * 2.0) * 2.0) + b_W_dv_2 * ((t10
    * t54 * 2.0 + t11 * t46 * t50 * 2.0) - t12 * t46 * t58 * 2.0) * 2.0;
  t171 = b_K_p_T * b_gain_el;
  b_W_act_ailerons = t171 * t15 * t32;
  b_m = b_gain_el * t37;
  t4 = b_gain_el * t33;
  t6 = b_gain_el * b_l_4;
  t7 = b_gain_el * t47;
  t8 = b_gain_el * t55;
  t9 = b_gamma_quadratic_du * t29;
  t10 = t4 * t34;
  t11 = b_gain_el * t34 * t37;
  t12 = -b_gain_el * t33 * t46;
  b_W_act_tilt_az = b_m * t46;
  t174 = t171 * b_l_1;
  t183 = t171 * b_l_z;
  b_W_act_phi = b_K_p_M * b_gain_el;
  gradient[4] = (((((t9 * (u_in[4] * 2.0 + t84) + t158 * ((t10 * t47 * t96 - t11
    * t55 * t96) + b_W_act_ailerons * t42 * t46) * 2.0) + b_W_dv_2 * ((t12 * t47
    * t96 + b_W_act_tilt_az * t55 * t96) + b_W_act_ailerons * t34 * t42) * 2.0)
                   + t81 * (b_m * t47 * t96 + t4 * t55 * t96) * 2.0) + t82 *
                  ((t7 * t92 + t6 * t55 * t96) + t174 * t15 * t32 * t42) * 2.0)
                 - b_W_act_tilt_el * ((t8 * t92 - t6 * t47 * t96) + t183 * t15 *
    t32 * t42) * 2.0) + b_W_dv_6 * ((t7 * t102 + t8 * t106) - b_W_act_phi * t15 *
    t32 * t42) * 2.0;
  b_W_act_ailerons = t171 * t16 * t32;
  t7 = b_gain_el * t56;
  gradient[5] = (((((t9 * (u_in[5] * 2.0 + t84) + t158 * ((t10 * t48 * t97 - t11
    * t56 * t97) + b_W_act_ailerons * t43 * t46) * 2.0) + b_W_dv_2 * ((t12 * t48
    * t97 + b_W_act_tilt_az * t56 * t97) + b_W_act_ailerons * t34 * t43) * 2.0)
                   + t81 * (b_m * t48 * t97 + t4 * t56 * t97) * 2.0) - t82 *
                  ((b_gain_el * t48 * t93 - t6 * t56 * t97) + t174 * t16 * t32 *
                   t43) * 2.0) + b_W_act_tilt_el * ((t7 * t93 + t6 * t48 * t97)
    - t183 * t16 * t32 * t43) * 2.0) + b_W_dv_6 * ((-b_gain_el * t48 * t103 + t7
    * t107) + b_W_act_phi * t16 * t32 * t43) * 2.0;
  b_W_act_ailerons = t171 * t17 * t32;
  t6 = b_gain_el * b_l_3;
  t7 = b_gain_el * t57;
  t8 = t171 * b_l_2;
  gradient[6] = (((((t9 * (u_in[6] * 2.0 + t84) + t158 * ((t10 * t49 * t98 - t11
    * t57 * t98) + b_W_act_ailerons * t44 * t46) * 2.0) + b_W_dv_2 * ((t12 * t49
    * t98 + b_W_act_tilt_az * t57 * t98) + b_W_act_ailerons * t34 * t44) * 2.0)
                   + t81 * (b_m * t49 * t98 + t4 * t57 * t98) * 2.0) - t82 *
                  ((-b_gain_el * t49 * t94 + t6 * t57 * t98) + t8 * t17 * t32 *
                   t44) * 2.0) - b_W_act_tilt_el * ((t7 * t94 + t6 * t49 * t98)
    + t183 * t17 * t32 * t44) * 2.0) - b_W_dv_6 * ((b_gain_el * t49 * t104 - t7 *
    t108) + b_W_act_phi * t17 * t32 * t44) * 2.0;
  b_W_act_ailerons = t171 * t18 * t32;
  t7 = b_gain_el * t50;
  t174 = b_gain_el * t58;
  gradient[7] = (((((t9 * (u_in[7] * 2.0 + t84) + t158 * ((t10 * t50 * t99 - t11
    * t58 * t99) + b_W_act_ailerons * t45 * t46) * 2.0) + b_W_dv_2 * ((t12 * t50
    * t99 + b_W_act_tilt_az * t58 * t99) + b_W_act_ailerons * t34 * t45) * 2.0)
                   + t81 * (b_m * t50 * t99 + t4 * t58 * t99) * 2.0) - t82 *
                  ((t7 * t95 + t6 * t58 * t99) - t8 * t18 * t32 * t45) * 2.0) -
                 b_W_act_tilt_el * ((t174 * -t95 + t6 * t50 * t99) + t183 * t18 *
    t32 * t45) * 2.0) + b_W_dv_6 * ((t7 * t105 + t174 * t109) + b_W_act_phi *
    t18 * t32 * t45) * 2.0;
  b_W_act_ailerons = b_gain_az * t33;
  b_m = b_gain_az * t37;
  t4 = b_gamma_quadratic_du * t30;
  t6 = b_gain_az * t34 * t37;
  t7 = b_W_act_ailerons * t34;
  t8 = b_m * t46;
  t9 = b_W_act_ailerons * t46;
  t10 = b_gain_az * b_l_1;
  t11 = b_gain_az * b_l_z;
  gradient[8] = (((((t4 * (u_in[8] * 2.0 + t86) + t158 * (t6 * t119 + t7 * t127)
                     * 2.0) - b_W_dv_2 * (t8 * t119 + t9 * t127) * 2.0) +
                   b_W_dv_6 * (t10 * t127 - t11 * t119) * 2.0) - t81 *
                  (b_W_act_ailerons * t119 - b_m * t127) * 2.0) + t82 *
                 (b_gain_az * t124 - b_gain_az * t133) * 2.0) + b_W_act_tilt_el *
    (b_gain_az * t116 + b_gain_az * t140) * 2.0;
  gradient[9] = (((((t4 * (u_in[9] * 2.0 + t86) + t158 * (t6 * t121 + t7 * t129)
                     * 2.0) - b_W_dv_2 * (t8 * t121 + t9 * t129) * 2.0) -
                   b_W_dv_6 * (t10 * t129 + t11 * t121) * 2.0) - t81 *
                  (b_W_act_ailerons * t121 - b_m * t129) * 2.0) - t82 *
                 (b_gain_az * t125 + b_gain_az * t135) * 2.0) - b_W_act_tilt_el *
    (b_gain_az * t117 - b_gain_az * t141) * 2.0;
  t10 = b_gain_az * b_l_2;
  gradient[10] = (((((t4 * (u_in[10] * 2.0 + t86) + t158 * (t6 * t122 + t7 *
    t130) * 2.0) - b_W_dv_2 * (t8 * t122 + t9 * t130) * 2.0) - b_W_dv_6 * (t10 *
    t130 + t11 * t122) * 2.0) - t81 * (b_W_act_ailerons * t122 - b_m * t130) *
                   2.0) + t82 * (b_gain_az * t126 + b_gain_az * t137) * 2.0) +
    b_W_act_tilt_el * (b_gain_az * t118 + b_gain_az * t156) * 2.0;
  gradient[11] = (((((t4 * (u_in[11] * 2.0 + t86) + t158 * (t6 * t123 + t7 *
    t131) * 2.0) - b_W_dv_2 * (t8 * t123 + t9 * t131) * 2.0) + b_W_dv_6 * (t10 *
    t131 - t11 * t123) * 2.0) - t81 * (b_W_act_ailerons * t123 - b_m * t131) *
                   2.0) - b_W_act_tilt_el * (b_gain_az * t120 + b_gain_az * t143)
                  * 2.0) - t82 * (b_gain_az * t128 + b_gain_az * t155) * 2.0;
  b_W_act_ailerons = t203_tmp_tmp * t3 * t14 * t19;
  gradient[12] = (((b_gamma_quadratic_du * t21 * (u_in[12] * 2.0 - b_gain_motor *
    2.0) + t158 * ((((((((b_gain_theta * t173 + b_gain_theta * t180) +
    b_gain_theta * -b_flight_path_angle) + b_gain_theta * -t179) + b_gain_theta *
                       t200) + b_gain_theta * -b_Cl_alpha) + t46 * t204) -
                    t199_tmp * t203) + b_W_act_ailerons * t34 * t37 * t80) * 2.0)
                   + b_W_dv_2 * ((((((((b_gain_theta * t172 - b_gain_theta *
    t181) + b_gain_theta * t188) - b_gain_theta * t192) - b_gain_theta * t197) +
    b_gain_theta * t199) + t34 * t204) + t200_tmp * t203) - b_W_act_ailerons *
    t37 * t46 * t80) * 2.0) - t81 * (t37 * t203 + b_W_act_ailerons * t33 * t80) *
                  2.0) - b_Cm_alpha * b_S * b_gain_theta * b_rho * t19 * t27 *
    t60 * t216 * b_wing_chord;
  b_W_act_ailerons = b_gain_phi * t33;
  b_m = b_gain_phi * t34;
  t4 = b_gain_phi * t46;
  gradient[13] = ((b_gamma_quadratic_du * t20 * (u_in[13] * 2.0 - t5 * 2.0) +
                   t81 * (((b_W_act_ailerons * t182 - b_W_act_ailerons * t184) +
    b_gain_phi * t37 * t185) - b_S * b_gain_phi * b_rho * t3 * t19 * t37 * t115 /
    2.0) * 2.0) - t158 * (((b_m * t169 + b_m * t186) - b_m * t194) - b_m * t195)
                  * 2.0) + b_W_dv_2 * (((t4 * t169 + t4 * t186) - t4 * t194) -
    t4 * t195) * 2.0;
  gradient[14] = b_gamma_quadratic_du * t31 * (u_in[14] * 2.0 -
    b_desired_motor_value * 2.0) - dv_global_6 * b_gain_ailerons * b_rho * t19 *
    t26 * t59 * t2;
  return cost;
}

/*
 * Arguments    : const b_captured_var *dv_global
 *                const captured_var *Beta
 *                const captured_var *CL_aileron
 *                const captured_var *Cd_zero
 *                const captured_var *Cl_alpha
 *                const captured_var *Cm_zero
 *                const captured_var *Cm_alpha
 *                const captured_var *I_xx
 *                const captured_var *I_yy
 *                const captured_var *I_zz
 *                const captured_var *K_Cd
 *                const captured_var *K_p_M
 *                const captured_var *K_p_T
 *                const captured_var *Phi
 *                const captured_var *S
 *                const captured_var *Theta
 *                const captured_var *V
 *                const captured_var *W_act_motor
 *                const captured_var *W_dv_1
 *                const captured_var *W_dv_2
 *                const captured_var *W_dv_3
 *                const captured_var *W_dv_4
 *                const captured_var *W_dv_5
 *                const captured_var *W_dv_6
 *                const captured_var *W_act_tilt_el
 *                const captured_var *W_act_tilt_az
 *                const captured_var *W_act_ailerons
 *                const captured_var *desired_el_value
 *                const captured_var *desired_az_value
 *                const captured_var *desired_motor_value
 *                const captured_var *desired_ailerons_value
 *                const captured_var *flight_path_angle
 *                const captured_var *gain_el
 *                const captured_var *gain_az
 *                const captured_var *gain_motor
 *                const captured_var *gain_ailerons
 *                const captured_var *gamma_quadratic_du
 *                const captured_var *l_1
 *                const captured_var *l_2
 *                const captured_var *l_3
 *                const captured_var *l_4
 *                const captured_var *l_z
 *                const captured_var *m
 *                const captured_var *p
 *                const captured_var *q
 *                const captured_var *r
 *                const captured_var *rho
 *                const captured_var *wing_chord
 *                const double u_in[13]
 *                double gradient[13]
 * Return Type  : double
 */
static double c_compute_cost_and_gradient_sec(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *Phi, const captured_var *S,
  const captured_var *Theta, const captured_var *V, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_motor_value, const captured_var *desired_ailerons_value,
  const captured_var *flight_path_angle, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *gain_motor, const captured_var
  *gain_ailerons, const captured_var *gamma_quadratic_du, const captured_var
  *l_1, const captured_var *l_2, const captured_var *l_3, const captured_var
  *l_4, const captured_var *l_z, const captured_var *m, const captured_var *p,
  const captured_var *q, const captured_var *r, const captured_var *rho, const
  captured_var *wing_chord, const double u_in[13], double gradient[13])
{
  double b_Beta;
  double b_CL_aileron;
  double b_Cd_zero;
  double b_Cm_alpha;
  double b_Cm_zero;
  double b_I_xx;
  double b_I_yy;
  double b_I_zz;
  double b_K_p_M;
  double b_K_p_T;
  double b_S;
  double b_V;
  double b_W_act_motor;
  double b_W_dv_1;
  double b_W_dv_2;
  double b_W_dv_3;
  double b_W_dv_4;
  double b_W_dv_5;
  double b_W_dv_6;
  double b_desired_ailerons_value;
  double b_gain_ailerons;
  double b_gain_az;
  double b_gain_el;
  double b_gamma_quadratic_du;
  double b_l_1;
  double b_l_2;
  double b_l_3;
  double b_l_4;
  double b_l_z;
  double b_m;
  double b_p;
  double b_q;
  double b_r;
  double b_rho;
  double b_wing_chord;
  double cost;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double t10;
  double t100;
  double t101;
  double t102;
  double t103;
  double t109;
  double t11;
  double t110;
  double t111;
  double t112;
  double t113;
  double t114;
  double t115;
  double t116;
  double t117;
  double t118;
  double t119;
  double t12;
  double t120;
  double t121;
  double t122;
  double t123;
  double t124;
  double t126;
  double t128;
  double t13;
  double t130;
  double t133;
  double t134;
  double t136;
  double t14;
  double t148;
  double t149;
  double t15;
  double t17;
  double t174;
  double t18;
  double t187;
  double t19;
  double t191;
  double t196;
  double t198;
  double t20;
  double t202;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t32;
  double t39;
  double t4;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t53;
  double t54;
  double t55;
  double t56;
  double t57;
  double t6;
  double t64;
  double t65;
  double t67;
  double t68;
  double t69;
  double t7;
  double t74;
  double t75;
  double t76;
  double t79;
  double t8;
  double t81;
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t90;
  double t91;
  double t92;
  double t93;
  double t96;
  double t97;
  double t98;
  double t99;
  dv_global_1 = dv_global->contents[0];
  dv_global_2 = dv_global->contents[1];
  dv_global_3 = dv_global->contents[2];
  dv_global_4 = dv_global->contents[3];
  dv_global_5 = dv_global->contents[4];
  dv_global_6 = dv_global->contents[5];
  b_Beta = Beta->contents;
  b_CL_aileron = CL_aileron->contents;
  b_Cd_zero = Cd_zero->contents;
  t187 = Cl_alpha->contents;
  b_Cm_zero = Cm_zero->contents;
  b_Cm_alpha = Cm_alpha->contents;
  b_I_xx = I_xx->contents;
  b_I_yy = I_yy->contents;
  b_I_zz = I_zz->contents;
  t191 = K_Cd->contents;
  b_K_p_M = K_p_M->contents;
  b_K_p_T = K_p_T->contents;
  t8 = Phi->contents;
  b_S = S->contents;
  t174 = Theta->contents;
  b_V = V->contents;
  b_W_act_motor = W_act_motor->contents;
  b_W_dv_1 = W_dv_1->contents;
  b_W_dv_2 = W_dv_2->contents;
  b_W_dv_3 = W_dv_3->contents;
  b_W_dv_4 = W_dv_4->contents;
  b_W_dv_5 = W_dv_5->contents;
  b_W_dv_6 = W_dv_6->contents;
  t75 = W_act_tilt_el->contents;
  t76 = W_act_tilt_az->contents;
  t65 = W_act_ailerons->contents;
  t67 = desired_el_value->contents;
  t198 = desired_az_value->contents;
  t202 = desired_motor_value->contents;
  b_desired_ailerons_value = desired_ailerons_value->contents;
  t69 = flight_path_angle->contents;
  b_gain_el = gain_el->contents;
  b_gain_az = gain_az->contents;
  t196 = gain_motor->contents;
  b_gain_ailerons = gain_ailerons->contents;
  b_gamma_quadratic_du = gamma_quadratic_du->contents;
  b_l_1 = l_1->contents;
  b_l_2 = l_2->contents;
  b_l_3 = l_3->contents;
  b_l_4 = l_4->contents;
  b_l_z = l_z->contents;
  b_m = m->contents;
  b_p = p->contents;
  b_q = q->contents;
  b_r = r->contents;
  b_rho = rho->contents;
  b_wing_chord = wing_chord->contents;

  /* COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2 */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi,S,Theta,V,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     30-Nov-2023 18:08:51 */
  t3 = cos(t8);
  t4 = sin(b_Beta);
  t5 = cos(t174);
  t6 = sin(t8);
  t7 = sin(t174);
  t8 = u_in[4] * b_gain_el;
  t9 = u_in[5] * b_gain_el;
  t10 = u_in[6] * b_gain_el;
  t11 = u_in[7] * b_gain_el;
  t12 = u_in[8] * b_gain_az;
  t13 = u_in[9] * b_gain_az;
  t14 = u_in[10] * b_gain_az;
  t15 = u_in[11] * b_gain_az;
  t17 = u_in[0] * u_in[0];
  t18 = u_in[1] * u_in[1];
  t19 = u_in[2] * u_in[2];
  t20 = u_in[3] * u_in[3];
  t21 = b_V * b_V;
  t22 = b_W_act_motor * b_W_act_motor;
  t23 = b_W_dv_1 * b_W_dv_1;
  t24 = b_W_dv_2 * b_W_dv_2;
  t25 = b_W_dv_3 * b_W_dv_3;
  t26 = b_W_dv_4 * b_W_dv_4;
  t27 = b_W_dv_5 * b_W_dv_5;
  t28 = b_W_dv_6 * b_W_dv_6;
  t29 = t75 * t75;
  t30 = t76 * t76;
  t31 = t65 * t65;
  t32 = t196 * t196;
  t55 = 1.0 / b_I_xx;
  t56 = 1.0 / b_I_yy;
  t57 = 1.0 / b_I_zz;
  t64 = 1.0 / b_m;
  t39 = cos(t8);
  t40 = cos(t9);
  t41 = cos(t10);
  t42 = cos(t11);
  t43 = cos(t12);
  t44 = cos(t13);
  t45 = cos(t14);
  t46 = cos(t15);
  t47 = sin(t8);
  t48 = sin(t9);
  t49 = sin(t10);
  t50 = sin(t11);
  t51 = sin(t12);
  t52 = sin(t13);
  t53 = sin(t14);
  t54 = sin(t15);
  t65 = t174 - t69;
  t67 *= 1.0 / b_gain_el;
  t68 = t198 * (1.0 / b_gain_az);
  t69 = t202 * (1.0 / t196);
  t74 = t69 * 2.0;
  t75 = cos(t65);
  t76 = sin(t65);
  b_V = b_K_p_M * t17 * t32;
  t86 = b_V * t47;
  b_W_act_motor = b_K_p_M * t18 * t32;
  t87 = b_W_act_motor * t48;
  b_W_dv_1 = b_K_p_M * t19 * t32;
  t88 = b_W_dv_1 * t49;
  b_W_dv_3 = b_K_p_M * t20 * t32;
  t89 = b_W_dv_3 * t50;
  b_W_dv_2 = b_K_p_T * t17 * t32;
  t90 = b_W_dv_2 * t47;
  b_W_dv_4 = b_K_p_T * t18 * t32;
  t91 = b_W_dv_4 * t48;
  b_W_dv_5 = b_K_p_T * t19 * t32;
  t92 = b_W_dv_5 * t49;
  b_W_dv_6 = b_K_p_T * t20 * t32;
  t93 = b_W_dv_6 * t50;
  b_V *= t39;
  t109 = b_V * t43;
  b_W_act_motor *= t40;
  t110 = b_W_act_motor * t44;
  b_W_dv_1 *= t41;
  t111 = b_W_dv_1 * t45;
  b_W_dv_2 *= t39;
  t112 = b_W_dv_2 * t43;
  b_W_dv_3 *= t42;
  t113 = b_W_dv_3 * t46;
  b_W_dv_4 *= t40;
  t114 = b_W_dv_4 * t44;
  b_W_dv_5 *= t41;
  t115 = b_W_dv_5 * t45;
  b_W_dv_6 *= t42;
  t116 = b_W_dv_6 * t46;
  t117 = b_V * t51;
  t118 = b_W_act_motor * t52;
  t119 = b_W_dv_1 * t53;
  t120 = b_W_dv_2 * t51;
  t121 = b_W_dv_3 * t54;
  t122 = b_W_dv_4 * t52;
  t123 = b_W_dv_5 * t53;
  t124 = b_W_dv_6 * t54;
  t79 = -(t67 * 2.0);
  t81 = -(t68 * 2.0);
  t96 = b_l_1 * t90;
  t97 = b_l_1 * t91;
  t98 = b_l_2 * t92;
  t99 = b_l_2 * t93;
  t100 = b_l_z * t90;
  t101 = b_l_z * t91;
  t102 = b_l_z * t92;
  t103 = b_l_z * t93;
  t126 = b_l_4 * t112;
  t128 = b_l_4 * t114;
  t130 = b_l_3 * t115;
  t133 = b_l_4 * t120;
  t134 = b_l_4 * t122;
  t136 = b_l_3 * t124;
  b_W_dv_2 = t187 * b_S * b_rho * t21 * t65;
  t14 = ((t90 + t91) + t92) + t93;
  t15 = ((t112 + t114) + t115) + t116;
  t174 = ((t120 + t122) + t123) + t124;
  t11 = b_Cd_zero + t191 * (t187 * t187) * (t65 * t65);
  t148 = -(b_l_3 * t116);
  t149 = -(b_l_3 * t123);
  t12 = t3 * t5;
  b_W_dv_1 = b_S * b_rho;
  t13 = b_W_dv_1 * t4;
  b_W_act_motor = b_W_dv_1 * cos(b_Beta) * t21;
  t10 = b_W_dv_2 * t75 / 2.0 + b_W_act_motor * t76 * t11 / 2.0;
  b_V = b_I_xx * b_p;
  t187 = dv_global_6 - t57 * (((((((((((((b_V * b_q - b_I_yy * b_p * b_q) + t97)
    + t98) - t96) - t99) + t109) + t111) + t133) + t134) - t110) - t113) + t149)
    - t136);
  b_Beta = b_CL_aileron * b_S;
  t191 = dv_global_4 - t55 * ((((((((((((((b_I_yy * b_q * b_r - b_I_zz * b_q *
    b_r) + t86) + t88) + b_Beta * u_in[12] * b_gain_ailerons * b_rho * t21 / 2.0)
    - t87) - t89) + b_l_1 * t112) + b_l_2 * t116) + b_l_z * t120) + b_l_z * t122)
    + b_l_z * t123) + b_l_z * t124) - b_l_1 * t114) - b_l_2 * t115);
  t196 = dv_global_5 - t56 * ((((((((((((((b_I_zz * b_p * b_r - b_V * b_r) +
    t100) + t101) + t102) + t103) + t118) + t121) + t126) + t128) - t117) - t119)
    - t130) + t148) + b_W_dv_1 * t21 * (b_Cm_zero + b_Cm_alpha * t65) *
    b_wing_chord / 2.0);
  b_V = t3 * t7;
  t9 = b_W_dv_2 * t76 / 2.0 - b_W_act_motor * t75 * t11 / 2.0;
  t202 = dv_global_1 + t64 * (((((t13 * t6 * t7 * t21 * t11 / 2.0 + t5 * t14) +
    -t5 * t9) + b_V * t10) + b_V * t15) - t6 * t7 * t174);
  t198 = dv_global_2 - t64 * (((-(b_W_dv_1 * t3 * t4 * t21 * t11 / 2.0) + t6 *
    t10) + t6 * t15) + t3 * t174);
  b_Cd_zero = b_desired_ailerons_value * (1.0 / b_gain_ailerons);
  t76 = u_in[12] - b_Cd_zero;
  t65 = u_in[0] - t69;
  b_m = u_in[1] - t69;
  t8 = u_in[2] - t69;
  b_W_dv_4 = u_in[3] - t69;
  b_W_dv_5 = u_in[4] - t67;
  b_W_dv_6 = u_in[5] - t67;
  t75 = u_in[6] - t67;
  b_W_act_motor = u_in[7] - t67;
  b_W_dv_1 = u_in[8] - t68;
  b_W_dv_2 = u_in[9] - t68;
  b_W_dv_3 = u_in[10] - t68;
  b_V = u_in[11] - t68;
  t67 = (dv_global_3 + t64 * (((((t13 * t5 * t6 * t21 * t11 / 2.0 - t7 * t14) +
             t12 * t10) + t12 * t15) - t5 * t6 * t174) + t7 * t9)) - 9.81;
  cost = (((((b_gamma_quadratic_du * ((((((((((((t31 * (t76 * t76) + t22 * (t65 *
    t65)) + t22 * (b_m * b_m)) + t22 * (t8 * t8)) + t22 * (b_W_dv_4 * b_W_dv_4))
    + t29 * (b_W_dv_5 * b_W_dv_5)) + t29 * (b_W_dv_6 * b_W_dv_6)) + t29 * (t75 *
    t75)) + t29 * (b_W_act_motor * b_W_act_motor)) + t30 * (b_W_dv_1 * b_W_dv_1))
    + t30 * (b_W_dv_2 * b_W_dv_2)) + t30 * (b_W_dv_3 * b_W_dv_3)) + t30 * (b_V *
    b_V)) + t25 * (t67 * t67)) + t28 * (t187 * t187)) + t26 * (t191 * t191)) +
           t24 * (t198 * t198)) + t27 * (t196 * t196)) + t23 * (t202 * t202);
  t9 = b_K_p_T * u_in[0];
  t10 = b_K_p_M * u_in[0] * t32;
  t11 = t9 * b_l_1 * t32;
  t12 = t9 * b_l_z * t32;
  t13 = t10 * t39;
  t14 = t9 * b_l_4 * t32 * t39;
  t15 = t9 * t3;
  t174 = t9 * t5;
  t69 = t9 * t6;
  t8 = b_gamma_quadratic_du * t22;
  t65 = t28 * t57 * t187;
  b_m = t26 * t55 * t191;
  t76 = t27 * t56 * t196;
  t75 = t24 * t64 * t198;
  b_W_dv_6 = t25 * t64;
  b_W_dv_5 = t23 * t64 * t202;
  gradient[0] = (((((t8 * (u_in[0] * 2.0 - t74) - t65 * ((t11 * t47 * -2.0 + t13
    * t43 * 2.0) + t14 * t51 * 2.0) * 2.0) - b_m * ((t10 * t47 * 2.0 + t11 * t39
    * t43 * 2.0) + t12 * t39 * t51 * 2.0) * 2.0) - t76 * ((t12 * t47 * 2.0 - t13
    * t51 * 2.0) + t14 * t43 * 2.0) * 2.0) - t75 * (t69 * t32 * t39 * t43 * 2.0
    + t15 * t32 * t39 * t51 * 2.0) * 2.0) - b_W_dv_6 * ((t9 * t7 * t32 * t47 *
    2.0 - t15 * t5 * t32 * t39 * t43 * 2.0) + t174 * t6 * t32 * t39 * t51 * 2.0)
                 * t67 * 2.0) + b_W_dv_5 * ((t174 * t32 * t47 * 2.0 + t15 * t7 *
    t32 * t39 * t43 * 2.0) - t69 * t7 * t32 * t39 * t51 * 2.0) * 2.0;
  t9 = b_K_p_T * u_in[1];
  t10 = b_K_p_M * u_in[1] * t32;
  t11 = t9 * b_l_1 * t32;
  t12 = t9 * b_l_z * t32;
  t13 = t10 * t40;
  t14 = t9 * b_l_4 * t32 * t40;
  t15 = t9 * t3;
  t174 = t9 * t5;
  t69 = t9 * t6;
  gradient[1] = (((((t8 * (u_in[1] * 2.0 - t74) - t65 * ((t11 * t48 * 2.0 - t13 *
    t44 * 2.0) + t14 * t52 * 2.0) * 2.0) + b_m * ((t10 * t48 * 2.0 + t11 * t40 *
    t44 * 2.0) - t12 * t40 * t52 * 2.0) * 2.0) - t76 * ((t12 * t48 * 2.0 + t13 *
    t52 * 2.0) + t14 * t44 * 2.0) * 2.0) - t75 * (t69 * t32 * t40 * t44 * 2.0 +
    t15 * t32 * t40 * t52 * 2.0) * 2.0) - b_W_dv_6 * ((t9 * t7 * t32 * t48 * 2.0
    - t15 * t5 * t32 * t40 * t44 * 2.0) + t174 * t6 * t32 * t40 * t52 * 2.0) *
                 t67 * 2.0) + b_W_dv_5 * ((t174 * t32 * t48 * 2.0 + t15 * t7 *
    t32 * t40 * t44 * 2.0) - t69 * t7 * t32 * t40 * t52 * 2.0) * 2.0;
  t9 = b_K_p_T * u_in[2];
  t10 = b_K_p_M * u_in[2] * t32;
  t11 = t9 * b_l_2 * t32;
  t12 = t9 * b_l_z * t32;
  t13 = t10 * t41;
  t14 = t9 * b_l_3 * t32 * t41;
  t15 = t9 * t3;
  t174 = t9 * t5;
  t69 = t9 * t6;
  gradient[2] = (((((t8 * (u_in[2] * 2.0 - t74) - t65 * ((t11 * t49 * 2.0 + t13 *
    t45 * 2.0) - t14 * t53 * 2.0) * 2.0) - b_m * ((t10 * t49 * 2.0 - t11 * t41 *
    t45 * 2.0) + t12 * t41 * t53 * 2.0) * 2.0) + t76 * ((t12 * t49 * -2.0 + t13 *
    t53 * 2.0) + t14 * t45 * 2.0) * 2.0) - t75 * (t69 * t32 * t41 * t45 * 2.0 +
    t15 * t32 * t41 * t53 * 2.0) * 2.0) - b_W_dv_6 * ((t9 * t7 * t32 * t49 * 2.0
    - t15 * t5 * t32 * t41 * t45 * 2.0) + t174 * t6 * t32 * t41 * t53 * 2.0) *
                 t67 * 2.0) + b_W_dv_5 * ((t174 * t32 * t49 * 2.0 + t15 * t7 *
    t32 * t41 * t45 * 2.0) - t69 * t7 * t32 * t41 * t53 * 2.0) * 2.0;
  t9 = b_K_p_T * u_in[3];
  t10 = b_K_p_M * u_in[3] * t32;
  t11 = t9 * b_l_2 * t32;
  t12 = t9 * b_l_z * t32;
  t13 = t10 * t42;
  t14 = t9 * b_l_3 * t32 * t42;
  t15 = t9 * t3;
  t174 = t9 * t5;
  t69 = t9 * t6;
  gradient[3] = (((((t8 * (u_in[3] * 2.0 - t74) + t65 * ((t11 * t50 * 2.0 + t13 *
    t46 * 2.0) + t14 * t54 * 2.0) * 2.0) - b_m * ((t10 * t50 * -2.0 + t11 * t42 *
    t46 * 2.0) + t12 * t42 * t54 * 2.0) * 2.0) - t76 * ((t12 * t50 * 2.0 + t13 *
    t54 * 2.0) - t14 * t46 * 2.0) * 2.0) - t75 * (t69 * t32 * t42 * t46 * 2.0 +
    t15 * t32 * t42 * t54 * 2.0) * 2.0) - b_W_dv_6 * ((t9 * t7 * t32 * t50 * 2.0
    - t15 * t5 * t32 * t42 * t46 * 2.0) + t174 * t6 * t32 * t42 * t54 * 2.0) *
                 t67 * 2.0) + b_W_dv_5 * ((t174 * t32 * t50 * 2.0 + t15 * t7 *
    t32 * t42 * t46 * 2.0) - t69 * t7 * t32 * t42 * t54 * 2.0) * 2.0;
  t9 = b_K_p_T * b_gain_el;
  t10 = b_gain_el * t6;
  t11 = b_gain_el * t3;
  t12 = b_gain_el * b_l_4;
  t13 = b_gain_el * t43;
  t14 = b_gain_el * t51;
  t15 = b_gamma_quadratic_du * t29;
  t174 = t11 * t5;
  t69 = b_gain_el * t5 * t6;
  t8 = t9 * t7;
  b_W_act_motor = -b_gain_el * t3 * t7;
  b_W_dv_1 = t10 * t7;
  b_W_dv_2 = t9 * t5;
  b_W_dv_3 = t9 * b_l_1;
  b_W_dv_4 = t9 * b_l_z;
  b_V = b_K_p_M * b_gain_el;
  gradient[4] = (((((t15 * (u_in[4] * 2.0 + t79) - b_W_dv_6 * ((t174 * t43 * t90
    - t69 * t51 * t90) + t8 * t17 * t32 * t39) * t67 * 2.0) + b_W_dv_5 *
                    ((b_W_act_motor * t43 * t90 + b_W_dv_1 * t51 * t90) +
                     b_W_dv_2 * t17 * t32 * t39) * 2.0) + t75 * (t10 * t43 * t90
    + t11 * t51 * t90) * 2.0) + t65 * ((t13 * t86 + t12 * t51 * t90) + b_W_dv_3 *
    t17 * t32 * t39) * 2.0) - t76 * ((t14 * t86 - t12 * t43 * t90) + b_W_dv_4 *
    t17 * t32 * t39) * 2.0) + b_m * ((t13 * t96 + t14 * t100) - b_V * t17 * t32 *
    t39) * 2.0;
  t13 = b_gain_el * t52;
  gradient[5] = (((((t15 * (u_in[5] * 2.0 + t79) - b_W_dv_6 * ((t174 * t44 * t91
    - t69 * t52 * t91) + t8 * t18 * t32 * t40) * t67 * 2.0) + b_W_dv_5 *
                    ((b_W_act_motor * t44 * t91 + b_W_dv_1 * t52 * t91) +
                     b_W_dv_2 * t18 * t32 * t40) * 2.0) + t75 * (t10 * t44 * t91
    + t11 * t52 * t91) * 2.0) - t65 * ((b_gain_el * t44 * t87 - t12 * t52 * t91)
    + b_W_dv_3 * t18 * t32 * t40) * 2.0) + t76 * ((t13 * t87 + t12 * t44 * t91)
    - b_W_dv_4 * t18 * t32 * t40) * 2.0) + b_m * ((-b_gain_el * t44 * t97 + t13 *
    t101) + b_V * t18 * t32 * t40) * 2.0;
  t12 = b_gain_el * b_l_3;
  t13 = b_gain_el * t53;
  t9 *= b_l_2;
  gradient[6] = (((((t15 * (u_in[6] * 2.0 + t79) - b_W_dv_6 * ((t174 * t45 * t92
    - t69 * t53 * t92) + t8 * t19 * t32 * t41) * t67 * 2.0) + b_W_dv_5 *
                    ((b_W_act_motor * t45 * t92 + b_W_dv_1 * t53 * t92) +
                     b_W_dv_2 * t19 * t32 * t41) * 2.0) + t75 * (t10 * t45 * t92
    + t11 * t53 * t92) * 2.0) - t65 * ((-b_gain_el * t45 * t88 + t12 * t53 * t92)
    + t9 * t19 * t32 * t41) * 2.0) - t76 * ((t13 * t88 + t12 * t45 * t92) +
    b_W_dv_4 * t19 * t32 * t41) * 2.0) - b_m * ((b_gain_el * t45 * t98 - t13 *
    t102) + b_V * t19 * t32 * t41) * 2.0;
  t13 = b_gain_el * t46;
  t14 = b_gain_el * t54;
  gradient[7] = (((((t15 * (u_in[7] * 2.0 + t79) - b_W_dv_6 * ((t174 * t46 * t93
    - t69 * t54 * t93) + t8 * t20 * t32 * t42) * t67 * 2.0) + b_W_dv_5 *
                    ((b_W_act_motor * t46 * t93 + b_W_dv_1 * t54 * t93) +
                     b_W_dv_2 * t20 * t32 * t42) * 2.0) + t75 * (t10 * t46 * t93
    + t11 * t54 * t93) * 2.0) - t65 * ((t13 * t89 + t12 * t54 * t93) - t9 * t20 *
    t32 * t42) * 2.0) - t76 * ((t14 * -t89 + t12 * t46 * t93) + b_W_dv_4 * t20 *
    t32 * t42) * 2.0) + b_m * ((t13 * t99 + t14 * t103) + b_V * t20 * t32 * t42)
    * 2.0;
  t9 = b_gain_az * t3;
  t10 = b_gain_az * t6;
  t11 = b_gamma_quadratic_du * t30;
  t12 = b_gain_az * t5 * t6;
  t13 = t9 * t5;
  t14 = t10 * t7;
  t15 = t9 * t7;
  t174 = b_gain_az * b_l_1;
  t69 = b_gain_az * b_l_z;
  gradient[8] = (((((t11 * (u_in[8] * 2.0 + t81) - b_W_dv_6 * (t12 * t112 + t13 *
    t120) * t67 * 2.0) - b_W_dv_5 * (t14 * t112 + t15 * t120) * 2.0) + b_m *
                   (t174 * t120 - t69 * t112) * 2.0) - t75 * (t9 * t112 - t10 *
    t120) * 2.0) + t65 * (b_gain_az * t117 - b_gain_az * t126) * 2.0) + t76 *
    (b_gain_az * t109 + b_gain_az * t133) * 2.0;
  gradient[9] = (((((t11 * (u_in[9] * 2.0 + t81) - b_W_dv_6 * (t12 * t114 + t13 *
    t122) * t67 * 2.0) - b_W_dv_5 * (t14 * t114 + t15 * t122) * 2.0) - b_m *
                   (t174 * t122 + t69 * t114) * 2.0) - t75 * (t9 * t114 - t10 *
    t122) * 2.0) - t65 * (b_gain_az * t118 + b_gain_az * t128) * 2.0) - t76 *
    (b_gain_az * t110 - b_gain_az * t134) * 2.0;
  t174 = b_gain_az * b_l_2;
  gradient[10] = (((((t11 * (u_in[10] * 2.0 + t81) - b_W_dv_6 * (t12 * t115 +
    t13 * t123) * t67 * 2.0) - b_W_dv_5 * (t14 * t115 + t15 * t123) * 2.0) - b_m
                    * (t174 * t123 + t69 * t115) * 2.0) - t75 * (t9 * t115 - t10
    * t123) * 2.0) + t65 * (b_gain_az * t119 + b_gain_az * t130) * 2.0) + t76 *
    (b_gain_az * t111 + b_gain_az * t149) * 2.0;
  gradient[11] = (((((t11 * (u_in[11] * 2.0 + t81) - b_W_dv_6 * (t12 * t116 +
    t13 * t124) * t67 * 2.0) - b_W_dv_5 * (t14 * t116 + t15 * t124) * 2.0) + b_m
                    * (t174 * t124 - t69 * t116) * 2.0) - t75 * (t9 * t116 - t10
    * t124) * 2.0) - t76 * (b_gain_az * t113 + b_gain_az * t136) * 2.0) - t65 *
    (b_gain_az * t121 + b_gain_az * t148) * 2.0;
  gradient[12] = b_gamma_quadratic_du * t31 * (u_in[12] * 2.0 - b_Cd_zero * 2.0)
    - b_Beta * b_gain_ailerons * b_rho * t21 * t26 * t55 * t191;
  return cost;
}

/*
 * Arguments    : const double lb[15]
 *                const double ub[15]
 *                l_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                const i_coder_internal_stickyStruct *FcnEvaluator
 *                h_struct_T *memspace
 *                m_struct_T *WorkingSet
 *                double Hessian[225]
 *                f_struct_T *QRManager
 *                g_struct_T *CholManager
 *                struct_T *QPObjective
 * Return Type  : void
 */
static void c_driver(const double lb[15], const double ub[15], l_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, h_struct_T
                     *memspace, m_struct_T *WorkingSet, double Hessian[225],
                     f_struct_T *QRManager, g_struct_T *CholManager, struct_T
                     *QPObjective)
{
  static const signed char iv[225] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1 };

  static const char qpoptions_SolverName[7] = { 'f', 'm', 'i', 'n', 'c', 'o',
    'n' };

  p_struct_T b_expl_temp;
  p_struct_T expl_temp;
  q_struct_T Flags;
  double varargout_2[15];
  int i;
  int ineqStart;
  int ixlast;
  int k;
  int mConstr;
  int mFixed;
  int mLB;
  int mUB;
  int nVar_tmp_tmp;
  int qpoptions_MaxIterations;
  memset(&QPObjective->grad[0], 0, 16U * sizeof(double));
  memset(&QPObjective->Hx[0], 0, 15U * sizeof(double));
  QPObjective->hasLinear = true;
  QPObjective->nvar = 15;
  QPObjective->maxVar = 16;
  QPObjective->beta = 0.0;
  QPObjective->rho = 0.0;
  QPObjective->objtype = 3;
  QPObjective->prev_objtype = 3;
  QPObjective->prev_nvar = 0;
  QPObjective->prev_hasLinear = false;
  QPObjective->gammaScalar = 0.0;
  CholManager->ldm = 31;
  CholManager->ndims = 0;
  CholManager->info = 0;
  CholManager->scaleFactor = 0.0;
  CholManager->ConvexCheck = true;
  CholManager->regTol_ = 1.7976931348623157E+308;
  CholManager->workspace_ = 1.7976931348623157E+308;
  CholManager->workspace2_ = 1.7976931348623157E+308;
  QRManager->ldq = 31;
  memset(&CholManager->FMat[0], 0, 961U * sizeof(double));
  memset(&QRManager->QR[0], 0, 961U * sizeof(double));
  memset(&QRManager->Q[0], 0, 961U * sizeof(double));
  QRManager->mrows = 0;
  QRManager->ncols = 0;
  memset(&QRManager->jpvt[0], 0, 31U * sizeof(int));
  memset(&QRManager->tau[0], 0, 31U * sizeof(double));
  QRManager->minRowCol = 0;
  QRManager->usedPivoting = false;
  for (i = 0; i < 225; i++) {
    Hessian[i] = iv[i];
  }

  nVar_tmp_tmp = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  ineqStart = WorkingSet->nVar;
  ixlast = (WorkingSet->sizes[3] + WorkingSet->sizes[4]) + (WorkingSet->sizes[0]
    << 1);
  if (ineqStart >= ixlast) {
    ixlast = ineqStart;
  }

  qpoptions_MaxIterations = 10 * ixlast;
  TrialState->steplength = 1.0;
  test_exit(MeritFunction, WorkingSet, TrialState, lb, ub, &Flags.fevalOK,
            &Flags.done, &Flags.stepAccepted, &Flags.failedLineSearch,
            &Flags.stepType);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (k = 0; k < 15; k++) {
    TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
    TrialState->grad_old[k] = TrialState->grad[k];
  }

  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    double phi_alpha;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        i = (unsigned char)mLB;
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet->lb[WorkingSet->indexLB[ixlast] - 1] = -lb
            [WorkingSet->indexLB[ixlast] - 1] + TrialState->xstarsqp
            [WorkingSet->indexLB[ixlast] - 1];
        }

        i = (unsigned char)mUB;
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet->ub[WorkingSet->indexUB[ixlast] - 1] = ub
            [WorkingSet->indexUB[ixlast] - 1] - TrialState->xstarsqp
            [WorkingSet->indexUB[ixlast] - 1];
        }

        i = (unsigned char)mFixed;
        for (ixlast = 0; ixlast < i; ixlast++) {
          phi_alpha = ub[WorkingSet->indexFixed[ixlast] - 1] -
            TrialState->xstarsqp[WorkingSet->indexFixed[ixlast] - 1];
          WorkingSet->ub[WorkingSet->indexFixed[ixlast] - 1] = phi_alpha;
          WorkingSet->bwset[ixlast] = phi_alpha;
        }

        if (WorkingSet->nActiveConstr > mFixed) {
          ineqStart = mFixed + 1;
          if (ineqStart < 1) {
            ineqStart = 1;
          }

          i = WorkingSet->nActiveConstr;
          for (ixlast = ineqStart; ixlast <= i; ixlast++) {
            switch (WorkingSet->Wid[ixlast - 1]) {
             case 4:
              WorkingSet->bwset[ixlast - 1] = WorkingSet->lb[WorkingSet->
                indexLB[WorkingSet->Wlocalidx[ixlast - 1] - 1] - 1];
              break;

             case 5:
              WorkingSet->bwset[ixlast - 1] = WorkingSet->ub[WorkingSet->
                indexUB[WorkingSet->Wlocalidx[ixlast - 1] - 1] - 1];
              break;

             default:
              /* A check that is always false is detected at compile-time. Eliminating code that follows. */
              break;
            }
          }
        }
      }

      expl_temp.ObjectiveLimit = -1.7976931348623157E+308;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }

      b_expl_temp = expl_temp;
      Flags.stepAccepted = step(&Flags.stepType, Hessian, lb, ub, TrialState,
        MeritFunction, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        &b_expl_temp);
      if (Flags.stepAccepted) {
        i = (unsigned char)nVar_tmp_tmp;
        for (ineqStart = 0; ineqStart < i; ineqStart++) {
          TrialState->xstarsqp[ineqStart] += TrialState->delta_x[ineqStart];
        }

        TrialState->sqpFval = d_compute_cost_and_gradient_fir
          (FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.dv_global,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Beta,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.CL_aileron,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cd_zero,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cl_alpha,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_zero,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_alpha,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_xx,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_yy,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_zz,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_Cd,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_M,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_T,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.S,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.V,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_phi,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_theta,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_motor,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_1,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_2,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_3,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_4,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_5,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_6,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_el,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_az,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_ailerons,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_el_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_az_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_phi_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_theta_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_motor_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_ailerons_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.flight_path_angle,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_el,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_az,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_phi,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_theta,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_motor,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_ailerons,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gamma_quadratic_du,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_1,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_2,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_3,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_4,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_z,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.m,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.p,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.q,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.r,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.rho,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.wing_chord,
           TrialState->xstarsqp);
        Flags.fevalOK = true;
        TrialState->FunctionEvaluations++;
        MeritFunction->phiFullStep = TrialState->sqpFval;
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && (MeritFunction->phi <
           MeritFunction->phiFullStep) && (TrialState->sqpFval <
           TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        double alpha;
        int i1;
        bool socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }

        i = WorkingSet->nVar;
        alpha = 1.0;
        ineqStart = 1;
        phi_alpha = MeritFunction->phiFullStep;
        i1 = (unsigned char)WorkingSet->nVar;
        if (i1 - 1 >= 0) {
          memcpy(&TrialState->searchDir[0], &TrialState->delta_x[0], (unsigned
                  int)i1 * sizeof(double));
        }

        int exitg1;
        do {
          exitg1 = 0;
          if (TrialState->FunctionEvaluations < 4000 && toc() < first_opt_iter_max_time) {
            if (phi_alpha <= MeritFunction->phi + alpha * 0.0001 *
                MeritFunction->phiPrimePlus) {
              exitg1 = 1;
            } else {
              bool exitg2;
              bool tooSmallX;
              alpha *= 0.7;
              i1 = (unsigned char)i;
              for (ixlast = 0; ixlast < i1; ixlast++) {
                TrialState->delta_x[ixlast] = alpha * TrialState->xstar[ixlast];
              }

              if (socTaken) {
                phi_alpha = alpha * alpha;
                if ((i >= 1) && (phi_alpha != 0.0)) {
                  ixlast = i - 1;
                  for (k = 0; k <= ixlast; k++) {
                    TrialState->delta_x[k] += phi_alpha *
                      TrialState->socDirection[k];
                  }
                }
              }

              tooSmallX = true;
              ixlast = 0;
              exitg2 = false;
              while ((!exitg2) && (ixlast <= (unsigned char)i - 1)) {
                if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[ixlast])) <=
                    fabs(TrialState->delta_x[ixlast])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  ixlast++;
                }
              }

              if (tooSmallX) {
                ineqStart = -2;
                exitg1 = 1;
              } else {
                for (ixlast = 0; ixlast < i1; ixlast++) {
                  TrialState->xstarsqp[ixlast] = TrialState->xstarsqp_old[ixlast]
                    + TrialState->delta_x[ixlast];
                }

                TrialState->sqpFval = d_compute_cost_and_gradient_fir
                  (FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.dv_global,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Beta,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.CL_aileron,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cd_zero,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cl_alpha,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_zero,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_alpha,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_xx,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_yy,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_zz,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_Cd,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_M,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_T,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.S,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.V,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_phi,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_theta,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_motor,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_1,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_2,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_3,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_4,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_5,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_6,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_el,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_az,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_ailerons,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_el_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_az_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_phi_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_theta_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_motor_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_ailerons_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.flight_path_angle,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_el,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_az,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_phi,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_theta,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_motor,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_ailerons,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gamma_quadratic_du,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_1,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_2,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_3,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_4,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_z,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.m,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.p,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.q,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.r,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.rho,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.wing_chord,
                   TrialState->xstarsqp);
                TrialState->FunctionEvaluations++;
                phi_alpha = TrialState->sqpFval;
              }
            }
          } else {
            ineqStart = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);

        Flags.fevalOK = true;
        TrialState->steplength = alpha;
        if (ineqStart > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      i = (unsigned char)nVar_tmp_tmp;
      for (ixlast = 0; ixlast < i; ixlast++) {
        TrialState->xstarsqp[ixlast] = TrialState->xstarsqp_old[ixlast] +
          TrialState->delta_x[ixlast];
      }

      i = (unsigned char)mConstr;
      for (ixlast = 0; ixlast < i; ixlast++) {
        phi_alpha = TrialState->lambdasqp[ixlast];
        phi_alpha += TrialState->steplength * (TrialState->lambda[ixlast] -
          phi_alpha);
        TrialState->lambdasqp[ixlast] = phi_alpha;
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      for (k = 0; k < 15; k++) {
        TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
        TrialState->grad_old[k] = TrialState->grad[k];
      }

      TrialState->sqpFval = c_compute_cost_and_gradient_fir
        (FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.dv_global,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Beta,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.CL_aileron,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cd_zero,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cl_alpha,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_zero,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_alpha,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_xx,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_yy,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_zz,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_Cd,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_M,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_T,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.S,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.V,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_phi,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_theta,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_motor,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_1,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_2,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_3,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_4,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_5,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_6,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_el,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_az,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_ailerons,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_el_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_az_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_phi_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_theta_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_motor_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_ailerons_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.flight_path_angle,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_el,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_az,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_phi,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_theta,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_motor,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_ailerons,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gamma_quadratic_du,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_1,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_2,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_3,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_4,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_z,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.m,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.p,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.q,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.r,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.rho,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.wing_chord,
         TrialState->xstarsqp, varargout_2);
      memcpy(&TrialState->grad[0], &varargout_2[0], 15U * sizeof(double));
      TrialState->FunctionEvaluations++;
      Flags.fevalOK = true;
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0], 15U *
             sizeof(double));
    }

    b_test_exit(&Flags, memspace, MeritFunction, WorkingSet, TrialState,
                QRManager, lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      i = (unsigned char)nVar_tmp_tmp;
      memcpy(&TrialState->delta_gradLag[0], &TrialState->grad[0], (unsigned int)
             i * sizeof(double));
      if (nVar_tmp_tmp >= 1) {
        ixlast = nVar_tmp_tmp - 1;
        for (k = 0; k <= ixlast; k++) {
          TrialState->delta_gradLag[k] -= TrialState->grad_old[k];
        }
      }

      BFGSUpdate(nVar_tmp_tmp, Hessian, TrialState->delta_x,
                 TrialState->delta_gradLag, memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/*
 * Arguments    : const m_struct_T *obj
 *                const double x[16]
 * Return Type  : double
 */
static double c_maxConstraintViolation(const m_struct_T *obj, const double x[16])
{
  double v;
  int i;
  int idx;
  int idxLB;
  v = 0.0;
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    i = (unsigned char)obj->sizes[4];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[idxLB] - obj->ub[idxLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      v = fmax(v, fabs(x[obj->indexFixed[idx] - 1] - obj->ub[obj->indexFixed[idx]
                       - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : b_struct_T *MeritFunction
 *                const o_struct_T *WorkingSet
 *                n_struct_T *TrialState
 *                const double lb[13]
 *                const double ub[13]
 *                bool *Flags_fevalOK
 *                bool *Flags_done
 *                bool *Flags_stepAccepted
 *                bool *Flags_failedLineSearch
 *                int *Flags_stepType
 * Return Type  : bool
 */
static bool c_test_exit(b_struct_T *MeritFunction, const o_struct_T *WorkingSet,
  n_struct_T *TrialState, const double lb[13], const double ub[13], bool
  *Flags_fevalOK, bool *Flags_done, bool *Flags_stepAccepted, bool
  *Flags_failedLineSearch, int *Flags_stepType)
{
  double optimRelativeFactor;
  double s;
  double smax;
  int i;
  int idx_max;
  int k;
  int nVar;
  bool Flags_gradOK;
  bool exitg1;
  bool isFeasible;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  i = (unsigned char)((WorkingSet->sizes[0] + WorkingSet->sizes[3]) +
                      WorkingSet->sizes[4]);
  if (i - 1 >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (unsigned
            int)i * sizeof(double));
  }

  c_computeGradLag(TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
                   WorkingSet->indexFixed, WorkingSet->sizes[0],
                   WorkingSet->indexLB, WorkingSet->sizes[3],
                   WorkingSet->indexUB, WorkingSet->sizes[4],
                   TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  optimRelativeFactor = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (optimRelativeFactor >= 1.7976931348623157E+308) {
    optimRelativeFactor = 1.0;
  }

  smax = 0.0;
  idx_max = (unsigned char)WorkingSet->sizes[3];
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet->indexLB[k] - 1;
    smax = fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }

  idx_max = (unsigned char)WorkingSet->sizes[4];
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet->indexUB[k] - 1;
    smax = fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  Flags_gradOK = true;
  smax = 0.0;
  idx_max = (unsigned char)WorkingSet->nVar;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= idx_max - 1)) {
    s = fabs(TrialState->gradLag[k]);
    Flags_gradOK = (s < 1.7976931348623157E+308);
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = fmax(smax, s);
      k++;
    }
  }

  MeritFunction->nlpDualFeasError = smax;
  if (!Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = smax;
    if (i - 1 >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (unsigned int)i * sizeof(double));
    }

    if (isFeasible && (smax <= 1.0E-9 * optimRelativeFactor)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }

  return Flags_gradOK;
}

/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const double A[169]
 *                int lda
 *                const double B[729]
 *                int ib0
 *                double C[378]
 * Return Type  : void
 */
static void c_xgemm(int m, int n, int k, const double A[169], int lda, const
                    double B[729], int ib0, double C[378])
{
  int cr;
  int ib;
  int ic;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    br = ib0;
    lastColC = 27 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 27) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    for (cr = 0; cr <= lastColC; cr += 27) {
      int ar;
      ar = -1;
      i = br + k;
      for (ib = br; ib < i; ib++) {
        int i2;
        i1 = cr + 1;
        i2 = cr + m;
        for (ic = i1; ic <= i2; ic++) {
          C[ic - 1] += B[ib - 1] * A[(ar + ic) - cr];
        }

        ar += lda;
      }

      br += 27;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[961]
 *                const double x[16]
 *                double y[496]
 * Return Type  : void
 */
static void c_xgemv(int m, int n, const double A[961], const double x[16],
                    double y[496])
{
  int ia;
  int iac;
  if (m != 0) {
    int i;
    memset(&y[0], 0, (unsigned int)n * sizeof(double));
    i = 31 * (n - 1) + 1;
    for (iac = 1; iac <= i; iac += 31) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }

      i1 = div_nde_s32_floor(iac - 1, 31);
      y[i1] += c;
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[729]
 *                int ix0
 * Return Type  : double
 */
static double c_xnrm2(int n, const double x[729], int ix0)
{
  double y;
  int k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        double absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/*
 * Arguments    : const double xCurrent[15]
 *                const int finiteLB[16]
 *                int mLB
 *                const double lb[15]
 *                const int finiteUB[16]
 *                int mUB
 *                const double ub[15]
 *                const double lambda[31]
 *                int iL0
 * Return Type  : double
 */
static double computeComplError(const double xCurrent[15], const int finiteLB[16],
  int mLB, const double lb[15], const int finiteUB[16], int mUB, const double
  ub[15], const double lambda[31], int iL0)
{
  double nlpComplError;
  int idx;
  nlpComplError = 0.0;
  if (mLB + mUB > 0) {
    double lbDelta;
    double lbLambda;
    int i;
    int i1;
    int ubOffset;
    ubOffset = (iL0 + mLB) - 1;
    i = (unsigned char)mLB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteLB[idx];
      lbDelta = xCurrent[i1 - 1] - lb[i1 - 1];
      lbLambda = lambda[(iL0 + idx) - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }

    i = (unsigned char)mUB;
    for (idx = 0; idx < i; idx++) {
      i1 = finiteUB[idx];
      lbDelta = ub[i1 - 1] - xCurrent[i1 - 1];
      lbLambda = lambda[ubOffset + idx];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }
  }

  return nlpComplError;
}

/*
 * Arguments    : const struct_T *obj
 *                double workspace[496]
 *                const double H[225]
 *                const double f[16]
 *                const double x[16]
 * Return Type  : double
 */
static double computeFval(const struct_T *obj, double workspace[496], const
  double H[225], const double f[16], const double x[16])
{
  double val;
  int idx;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
      val = 0.0;
      if (obj->nvar >= 1) {
        int i;
        i = (unsigned char)obj->nvar;
        for (idx = 0; idx < i; idx++) {
          val += x[idx] * workspace[idx];
        }
      }
    }
    break;

   default:
    {
      int i;
      linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
      i = obj->nvar + 1;
      for (idx = i; idx < 16; idx++) {
        workspace[idx - 1] = 0.5 * obj->beta * x[idx - 1] + obj->rho;
      }

      val = 0.0;
      for (idx = 0; idx < 15; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
    break;
  }

  return val;
}

/*
 * Arguments    : const struct_T *obj
 *                double workspace[496]
 *                const double f[16]
 *                const double x[16]
 * Return Type  : double
 */
static double computeFval_ReuseHx(const struct_T *obj, double workspace[496],
  const double f[16], const double x[16])
{
  double val;
  int k;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int i;
        i = (unsigned char)obj->nvar;
        for (k = 0; k < i; k++) {
          workspace[k] = 0.5 * obj->Hx[k] + f[k];
        }

        val = 0.0;
        if (obj->nvar >= 1) {
          for (k = 0; k < i; k++) {
            val += x[k] * workspace[k];
          }
        }
      } else {
        val = 0.0;
        if (obj->nvar >= 1) {
          int i;
          i = (unsigned char)obj->nvar;
          for (k = 0; k < i; k++) {
            val += x[k] * obj->Hx[k];
          }
        }

        val *= 0.5;
      }
    }
    break;

   default:
    {
      if (obj->hasLinear) {
        int i;
        i = (unsigned char)obj->nvar;
        if (i - 1 >= 0) {
          memcpy(&workspace[0], &f[0], (unsigned int)i * sizeof(double));
        }

        i = 14 - obj->nvar;
        for (k = 0; k <= i; k++) {
          workspace[obj->nvar + k] = obj->rho;
        }

        val = 0.0;
        for (k = 0; k < 15; k++) {
          double d;
          d = workspace[k] + 0.5 * obj->Hx[k];
          workspace[k] = d;
          val += x[k] * d;
        }
      } else {
        int i;
        val = 0.0;
        for (k = 0; k < 15; k++) {
          val += x[k] * obj->Hx[k];
        }

        val *= 0.5;
        i = obj->nvar + 1;
        for (k = i; k < 16; k++) {
          val += x[k - 1] * obj->rho;
        }
      }
    }
    break;
  }

  return val;
}

/*
 * Arguments    : double workspace[16]
 *                int nVar
 *                const double grad[16]
 *                const int finiteFixed[16]
 *                int mFixed
 *                const int finiteLB[16]
 *                int mLB
 *                const int finiteUB[16]
 *                int mUB
 *                const double lambda[31]
 * Return Type  : void
 */
static void computeGradLag(double workspace[16], int nVar, const double grad[16],
  const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB, const
  int finiteUB[16], int mUB, const double lambda[31])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }

  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }

  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }

  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : struct_T *obj
 *                const double H[225]
 *                const double f[16]
 *                const double x[16]
 * Return Type  : void
 */
static void computeGrad_StoreHx(struct_T *obj, const double H[225], const double
  f[16], const double x[16])
{
  int ixlast;
  int k;
  switch (obj->objtype) {
   case 5:
    {
      int i;
      i = obj->nvar;
      if (i - 2 >= 0) {
        memset(&obj->grad[0], 0, (unsigned int)(i - 1) * sizeof(double));
      }

      obj->grad[obj->nvar - 1] = obj->gammaScalar;
    }
    break;

   case 3:
    {
      int i;
      xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      i = (unsigned char)obj->nvar;
      if (i - 1 >= 0) {
        memcpy(&obj->grad[0], &obj->Hx[0], (unsigned int)i * sizeof(double));
      }

      if (obj->hasLinear && (obj->nvar >= 1)) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          obj->grad[k] += f[k];
        }
      }
    }
    break;

   default:
    {
      int i;
      xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      i = obj->nvar + 1;
      for (ixlast = i; ixlast < 16; ixlast++) {
        obj->Hx[ixlast - 1] = obj->beta * x[ixlast - 1];
      }

      memcpy(&obj->grad[0], &obj->Hx[0], 15U * sizeof(double));
      if (obj->hasLinear && (obj->nvar >= 1)) {
        ixlast = obj->nvar - 1;
        for (k = 0; k <= ixlast; k++) {
          obj->grad[k] += f[k];
        }
      }

      if (15 - obj->nvar >= 1) {
        ixlast = obj->nvar;
        i = 14 - obj->nvar;
        for (k = 0; k <= i; k++) {
          int i1;
          i1 = ixlast + k;
          obj->grad[i1] += obj->rho;
        }
      }
    }
    break;
  }
}

/*
 * Arguments    : f_struct_T *obj
 *                int nrows
 * Return Type  : void
 */
static void computeQ_(f_struct_T *obj, int nrows)
{
  double work[31];
  int b_i;
  int iQR0;
  int ia;
  int idx;
  int lastc;
  int m;
  int n;
  lastc = obj->minRowCol;
  for (idx = 0; idx < lastc; idx++) {
    iQR0 = 31 * idx + idx;
    n = obj->mrows - idx;
    if (n - 2 >= 0) {
      memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1], (unsigned int)(((n + iQR0) -
               iQR0) - 1) * sizeof(double));
    }
  }

  m = obj->mrows;
  if (nrows >= 1) {
    int i;
    int i1;
    int itau;
    i = nrows - 1;
    for (idx = lastc; idx <= i; idx++) {
      ia = idx * 31;
      i1 = m - 1;
      memset(&obj->Q[ia], 0, (unsigned int)(((i1 + ia) - ia) + 1) * sizeof
             (double));
      obj->Q[ia + idx] = 1.0;
    }

    itau = obj->minRowCol - 1;
    memset(&work[0], 0, 31U * sizeof(double));
    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * 31;
      if (b_i < nrows) {
        int lastv;
        obj->Q[iaii - 1] = 1.0;
        idx = iaii + 31;
        if (obj->tau[itau] != 0.0) {
          bool exitg2;
          lastv = m - b_i;
          iQR0 = (iaii + m) - b_i;
          while ((lastv + 1 > 0) && (obj->Q[iQR0 - 1] == 0.0)) {
            lastv--;
            iQR0--;
          }

          lastc = (nrows - b_i) - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int exitg1;
            iQR0 = (iaii + lastc * 31) + 31;
            ia = iQR0;
            do {
              exitg1 = 0;
              if (ia <= iQR0 + lastv) {
                if (obj->Q[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = -1;
        }

        if (lastv + 1 > 0) {
          double c;
          if (lastc + 1 != 0) {
            if (lastc >= 0) {
              memset(&work[0], 0, (unsigned int)(lastc + 1) * sizeof(double));
            }

            i = (iaii + 31 * lastc) + 31;
            for (n = idx; n <= i; n += 31) {
              c = 0.0;
              i1 = n + lastv;
              for (ia = n; ia <= i1; ia++) {
                c += obj->Q[ia - 1] * obj->Q[((iaii + ia) - n) - 1];
              }

              iQR0 = div_nde_s32_floor((n - iaii) - 31, 31);
              work[iQR0] += c;
            }
          }

          if (-obj->tau[itau] != 0.0) {
            iQR0 = iaii;
            for (idx = 0; idx <= lastc; idx++) {
              c = work[idx];
              if (c != 0.0) {
                c *= -obj->tau[itau];
                i = iQR0 + 31;
                i1 = lastv + iQR0;
                for (n = i; n <= i1 + 31; n++) {
                  obj->Q[n - 1] += obj->Q[((iaii + n) - iQR0) - 32] * c;
                }
              }

              iQR0 += 31;
            }
          }
        }
      }

      if (b_i < m) {
        iQR0 = iaii + 1;
        i = (iaii + m) - b_i;
        for (lastc = iQR0; lastc <= i; lastc++) {
          obj->Q[lastc - 1] *= -obj->tau[itau];
        }
      }

      obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
      i = (unsigned char)(b_i - 1);
      for (idx = 0; idx < i; idx++) {
        obj->Q[(iaii - idx) - 2] = 0.0;
      }

      itau--;
    }
  }
}

/*
 * Arguments    : const double H[225]
 *                l_struct_T *solution
 *                h_struct_T *memspace
 *                const f_struct_T *qrmanager
 *                g_struct_T *cholmanager
 *                const struct_T *objective
 *                bool alwaysPositiveDef
 * Return Type  : void
 */
static void compute_deltax(const double H[225], l_struct_T *solution, h_struct_T
  *memspace, const f_struct_T *qrmanager, g_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef)
{
  int b_i;
  int idx;
  int ix;
  int jA;
  int jjA;
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) * sizeof
             (double));
    }
  } else {
    for (idx = 0; idx <= nVar_tmp; idx++) {
      solution->searchDir[idx] = -objective->grad[idx];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        {
          double smax;
          int nVars;
          if (alwaysPositiveDef) {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              jjA = (nVar_tmp + 1) * idx;
              jA = 31 * idx;
              for (ix = 0; ix <= nVar_tmp; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            cholmanager->info = xpotrf(qrmanager->mrows, cholmanager->FMat);
          } else {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              jjA = qrmanager->mrows * idx;
              jA = 31 * idx;
              for (ix = 0; ix <= nVar_tmp; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            if (qrmanager->mrows < 1) {
              nVars = -1;
            } else {
              nVars = 0;
              if (qrmanager->mrows > 1) {
                smax = fabs(cholmanager->FMat[0]);
                for (ix = 2; ix <= nVar_tmp + 1; ix++) {
                  double s;
                  s = fabs(cholmanager->FMat[(ix - 1) << 5]);
                  if (s > smax) {
                    nVars = ix - 1;
                    smax = s;
                  }
                }
              }
            }

            cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 31 *
              nVars]) * 2.2204460492503131E-16, 0.0);
            fullColLDL2_(cholmanager, qrmanager->mrows);
            if (cholmanager->ConvexCheck) {
              idx = 0;
              int exitg1;
              do {
                exitg1 = 0;
                if (idx <= nVar_tmp) {
                  if (cholmanager->FMat[idx + 31 * idx] <= 0.0) {
                    cholmanager->info = -idx - 1;
                    exitg1 = 1;
                  } else {
                    idx++;
                  }
                } else {
                  cholmanager->ConvexCheck = false;
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
            }
          }

          if (cholmanager->info != 0) {
            solution->state = -6;
          } else if (alwaysPositiveDef) {
            solve(cholmanager, solution->searchDir);
          } else {
            int i;
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 31;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  solution->searchDir[ix] -= solution->searchDir[idx] *
                    cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }

            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              solution->searchDir[idx] /= cholmanager->FMat[idx + 31 * idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 31;
                smax = solution->searchDir[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                    solution->searchDir[b_i - 1];
                }

                solution->searchDir[idx - 1] = smax;
              }
            }
          }
        }
        break;

       default:
        {
          if (alwaysPositiveDef) {
            int nVars;
            nVars = objective->nvar;
            cholmanager->ndims = objective->nvar;
            for (idx = 0; idx < nVars; idx++) {
              jjA = nVars * idx;
              jA = 31 * idx;
              for (ix = 0; ix < nVars; ix++) {
                cholmanager->FMat[jA + ix] = H[jjA + ix];
              }
            }

            cholmanager->info = xpotrf(objective->nvar, cholmanager->FMat);
            if (cholmanager->info != 0) {
              solution->state = -6;
            } else {
              double smax;
              int i;
              solve(cholmanager, solution->searchDir);
              smax = 1.0 / objective->beta;
              jjA = objective->nvar + 1;
              i = qrmanager->mrows;
              for (ix = jjA; ix <= i; ix++) {
                solution->searchDir[ix - 1] *= smax;
              }
            }
          }
        }
        break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = 31 * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace->workspace_double[idx] = -qrmanager->Q[nVar_tmp + 31 *
            (qrmanager->ncols + idx)];
        }

        if (qrmanager->mrows != 0) {
          int i;
          memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) *
                 sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
          for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
            int nVars;
            nVars = jjA + nVar_tmp;
            for (idx = jjA; idx <= nVars; idx++) {
              jA = idx - jjA;
              solution->searchDir[jA] += qrmanager->Q[idx - 1] *
                memspace->workspace_double[ix];
            }

            ix++;
          }
        }
      } else {
        double smax;
        int i;
        int nVars;
        if (objective->objtype == 3) {
          xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp,
                memspace->workspace_double);
          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager->mrows;
          xgemm(objective->nvar, mNull_tmp, objective->nvar, H, objective->nvar,
                qrmanager->Q, nullStartIdx_tmp, memspace->workspace_double);
          for (jA = 0; jA < mNull_tmp; jA++) {
            i = objective->nvar + 1;
            for (jjA = i; jjA <= nVars; jjA++) {
              memspace->workspace_double[(jjA + 31 * jA) - 1] = objective->beta *
                qrmanager->Q[(jjA + 31 * (jA + qrmanager->ncols)) - 1];
            }
          }

          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp, memspace->workspace_double,
                  cholmanager->FMat);
        }

        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          cholmanager->info = xpotrf(mNull_tmp, cholmanager->FMat);
        } else {
          cholmanager->ndims = mNull_tmp;
          nVars = 0;
          if (mNull_tmp > 1) {
            smax = fabs(cholmanager->FMat[0]);
            for (ix = 2; ix <= mNull_tmp; ix++) {
              double s;
              s = fabs(cholmanager->FMat[(ix - 1) << 5]);
              if (s > smax) {
                nVars = ix - 1;
                smax = s;
              }
            }
          }

          cholmanager->regTol_ = fmax(fabs(cholmanager->FMat[nVars + 31 * nVars])
            * 2.2204460492503131E-16, 0.0);
          fullColLDL2_(cholmanager, mNull_tmp);
          if (cholmanager->ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager->FMat[idx + 31 * idx] <= 0.0) {
                  cholmanager->info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            memset(&memspace->workspace_double[0], 0, (unsigned int)mNull_tmp *
                   sizeof(double));
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
              smax = 0.0;
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                smax += qrmanager->Q[idx - 1] * objective->grad[idx - jjA];
              }

              nVars = div_nde_s32_floor(jjA - nullStartIdx_tmp, 31);
              memspace->workspace_double[nVars] -= smax;
            }
          }

          if (alwaysPositiveDef) {
            nVars = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx < nVars; idx++) {
                jA = idx * 31;
                smax = memspace->workspace_double[idx];
                for (b_i = 0; b_i < idx; b_i++) {
                  smax -= cholmanager->FMat[jA + b_i] *
                    memspace->workspace_double[b_i];
                }

                memspace->workspace_double[idx] = smax / cholmanager->FMat[jA +
                  idx];
              }
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jjA = (idx + (idx - 1) * 31) - 1;
                memspace->workspace_double[idx - 1] /= cholmanager->FMat[jjA];
                for (b_i = 0; b_i <= idx - 2; b_i++) {
                  ix = (idx - b_i) - 2;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx - 1] * cholmanager->FMat[(jjA - b_i) -
                    1];
                }
              }
            }
          } else {
            nVars = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= nVars + 1; idx++) {
                jjA = idx + idx * 31;
                i = nVars - idx;
                for (b_i = 0; b_i <= i; b_i++) {
                  ix = (idx + b_i) + 1;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx] * cholmanager->FMat[(jjA + b_i) + 1];
                }
              }
            }

            nVars = cholmanager->ndims;
            for (idx = 0; idx < nVars; idx++) {
              memspace->workspace_double[idx] /= cholmanager->FMat[idx + 31 *
                idx];
            }

            if (cholmanager->ndims != 0) {
              for (idx = nVars; idx >= 1; idx--) {
                jA = (idx - 1) * 31;
                smax = memspace->workspace_double[idx - 1];
                i = idx + 1;
                for (b_i = nVars; b_i >= i; b_i--) {
                  smax -= cholmanager->FMat[(jA + b_i) - 1] *
                    memspace->workspace_double[b_i - 1];
                }

                memspace->workspace_double[idx - 1] = smax;
              }
            }
          }

          if (qrmanager->mrows != 0) {
            memset(&solution->searchDir[0], 0, (unsigned int)(nVar_tmp + 1) *
                   sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (jjA = nullStartIdx_tmp; jjA <= i; jjA += 31) {
              nVars = jjA + nVar_tmp;
              for (idx = jjA; idx <= nVars; idx++) {
                jA = idx - jjA;
                solution->searchDir[jA] += qrmanager->Q[idx - 1] *
                  memspace->workspace_double[ix];
              }

              ix++;
            }
          }
        }
      }
    }
  }
}

/*
 * Arguments    : int x[31]
 *                int xLen
 *                int workspace[31]
 *                int xMin
 *                int xMax
 * Return Type  : void
 */
static void countsort(int x[31], int xLen, int workspace[31], int xMin, int xMax)
{
  int idx;
  int idxFill;
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    if (idxStart >= 0) {
      memset(&workspace[0], 0, (unsigned int)(idxStart + 1) * sizeof(int));
    }

    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart]++;
    }

    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] += workspace[idx - 2];
    }

    idxStart = 1;
    idxEnd = workspace[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }

      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }

    for (idx = idxStart; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

/*
 * Arguments    : double workspace[378]
 *                int nVar
 *                const double grad[14]
 *                const int finiteFixed[14]
 *                int mFixed
 *                const int finiteLB[14]
 *                int mLB
 *                const int finiteUB[14]
 *                int mUB
 *                const double lambda[27]
 * Return Type  : void
 */
static void d_computeGradLag(double workspace[378], int nVar, const double grad
  [14], const int finiteFixed[14], int mFixed, const int finiteLB[14], int mLB,
  const int finiteUB[14], int mUB, const double lambda[27])
{
  int i;
  int i1;
  int iL0;
  int idx;
  i = (unsigned char)nVar;
  memcpy(&workspace[0], &grad[0], (unsigned int)i * sizeof(double));
  i = (unsigned char)mFixed;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }

  i = (unsigned char)mLB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }

  if ((unsigned char)mLB - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + (unsigned char)mLB;
  }

  i = (unsigned char)mUB;
  for (idx = 0; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : const b_captured_var *dv_global
 *                const captured_var *Beta
 *                const captured_var *CL_aileron
 *                const captured_var *Cd_zero
 *                const captured_var *Cl_alpha
 *                const captured_var *Cm_zero
 *                const captured_var *Cm_alpha
 *                const captured_var *I_xx
 *                const captured_var *I_yy
 *                const captured_var *I_zz
 *                const captured_var *K_Cd
 *                const captured_var *K_p_M
 *                const captured_var *K_p_T
 *                const captured_var *S
 *                const captured_var *V
 *                const captured_var *W_act_phi
 *                const captured_var *W_act_theta
 *                const captured_var *W_act_motor
 *                const captured_var *W_dv_1
 *                const captured_var *W_dv_2
 *                const captured_var *W_dv_3
 *                const captured_var *W_dv_4
 *                const captured_var *W_dv_5
 *                const captured_var *W_dv_6
 *                const captured_var *W_act_tilt_el
 *                const captured_var *W_act_tilt_az
 *                const captured_var *W_act_ailerons
 *                const captured_var *desired_el_value
 *                const captured_var *desired_az_value
 *                const captured_var *desired_phi_value
 *                const captured_var *desired_theta_value
 *                const captured_var *desired_motor_value
 *                const captured_var *desired_ailerons_value
 *                const captured_var *flight_path_angle
 *                const captured_var *gain_el
 *                const captured_var *gain_az
 *                const captured_var *gain_phi
 *                const captured_var *gain_theta
 *                const captured_var *gain_motor
 *                const captured_var *gain_ailerons
 *                const captured_var *gamma_quadratic_du
 *                const captured_var *l_1
 *                const captured_var *l_2
 *                const captured_var *l_3
 *                const captured_var *l_4
 *                const captured_var *l_z
 *                const captured_var *m
 *                const captured_var *p
 *                const captured_var *q
 *                const captured_var *r
 *                const captured_var *rho
 *                const captured_var *wing_chord
 *                const double u_in[15]
 * Return Type  : double
 */
static double d_compute_cost_and_gradient_fir(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *S, const captured_var *V, const
  captured_var *W_act_phi, const captured_var *W_act_theta, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_phi_value, const captured_var *desired_theta_value,
  const captured_var *desired_motor_value, const captured_var
  *desired_ailerons_value, const captured_var *flight_path_angle, const
  captured_var *gain_el, const captured_var *gain_az, const captured_var
  *gain_phi, const captured_var *gain_theta, const captured_var *gain_motor,
  const captured_var *gain_ailerons, const captured_var *gamma_quadratic_du,
  const captured_var *l_1, const captured_var *l_2, const captured_var *l_3,
  const captured_var *l_4, const captured_var *l_z, const captured_var *m, const
  captured_var *p, const captured_var *q, const captured_var *r, const
  captured_var *rho, const captured_var *wing_chord, const double u_in[15])
{
  double b_Beta;
  double b_CL_aileron;
  double b_Cd_zero;
  double b_Cl_alpha;
  double b_Cm_alpha;
  double b_Cm_zero;
  double b_I_xx;
  double b_I_yy;
  double b_I_zz;
  double b_K_Cd;
  double b_K_p_M;
  double b_S;
  double b_V;
  double b_W_act_ailerons;
  double b_W_act_motor;
  double b_W_act_phi;
  double b_W_act_theta;
  double b_W_dv_1;
  double b_W_dv_2;
  double b_W_dv_3;
  double b_W_dv_4;
  double b_W_dv_5;
  double b_W_dv_6;
  double b_desired_ailerons_value;
  double b_desired_motor_value;
  double b_desired_phi_value;
  double b_desired_theta_value;
  double b_flight_path_angle;
  double b_gain_ailerons;
  double b_gain_az;
  double b_gain_motor;
  double b_gain_phi;
  double b_gain_theta;
  double b_gamma_quadratic_du;
  double b_l_1;
  double b_l_2;
  double b_l_3;
  double b_l_4;
  double b_l_z;
  double b_p;
  double b_q;
  double b_r;
  double b_rho;
  double b_wing_chord;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double t10;
  double t11;
  double t115;
  double t119;
  double t12;
  double t121;
  double t122;
  double t123;
  double t127;
  double t129;
  double t13;
  double t15;
  double t16;
  double t17;
  double t18;
  double t184;
  double t185;
  double t19;
  double t22;
  double t29;
  double t30;
  double t32;
  double t33;
  double t34;
  double t37;
  double t4;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t53;
  double t54;
  double t55;
  double t56;
  double t57;
  double t58;
  double t6;
  double t69;
  double t7;
  double t72;
  double t73;
  double t74;
  double t8;
  double t80;
  double t81;
  double t82;
  double t9;
  double t96;
  double t97;
  double t98;
  double t99;
  dv_global_1 = dv_global->contents[0];
  dv_global_2 = dv_global->contents[1];
  dv_global_3 = dv_global->contents[2];
  dv_global_4 = dv_global->contents[3];
  dv_global_5 = dv_global->contents[4];
  dv_global_6 = dv_global->contents[5];
  b_Beta = Beta->contents;
  b_CL_aileron = CL_aileron->contents;
  b_Cd_zero = Cd_zero->contents;
  b_Cl_alpha = Cl_alpha->contents;
  b_Cm_zero = Cm_zero->contents;
  b_Cm_alpha = Cm_alpha->contents;
  b_I_xx = I_xx->contents;
  b_I_yy = I_yy->contents;
  b_I_zz = I_zz->contents;
  b_K_Cd = K_Cd->contents;
  b_K_p_M = K_p_M->contents;
  t184 = K_p_T->contents;
  b_S = S->contents;
  b_V = V->contents;
  b_W_act_phi = W_act_phi->contents;
  b_W_act_theta = W_act_theta->contents;
  b_W_act_motor = W_act_motor->contents;
  b_W_dv_1 = W_dv_1->contents;
  b_W_dv_2 = W_dv_2->contents;
  b_W_dv_3 = W_dv_3->contents;
  b_W_dv_4 = W_dv_4->contents;
  b_W_dv_5 = W_dv_5->contents;
  b_W_dv_6 = W_dv_6->contents;
  t81 = W_act_tilt_el->contents;
  t82 = W_act_tilt_az->contents;
  b_W_act_ailerons = W_act_ailerons->contents;
  t129 = desired_el_value->contents;
  t127 = desired_az_value->contents;
  b_desired_phi_value = desired_phi_value->contents;
  b_desired_theta_value = desired_theta_value->contents;
  b_desired_motor_value = desired_motor_value->contents;
  b_desired_ailerons_value = desired_ailerons_value->contents;
  b_flight_path_angle = flight_path_angle->contents;
  t115 = gain_el->contents;
  b_gain_az = gain_az->contents;
  b_gain_phi = gain_phi->contents;
  b_gain_theta = gain_theta->contents;
  b_gain_motor = gain_motor->contents;
  b_gain_ailerons = gain_ailerons->contents;
  b_gamma_quadratic_du = gamma_quadratic_du->contents;
  b_l_1 = l_1->contents;
  b_l_2 = l_2->contents;
  b_l_3 = l_3->contents;
  b_l_4 = l_4->contents;
  b_l_z = l_z->contents;
  t43 = m->contents;
  b_p = p->contents;
  b_q = q->contents;
  b_r = r->contents;
  b_rho = rho->contents;
  b_wing_chord = wing_chord->contents;

  /* COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2 */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_FIRST_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     30-Nov-2023 18:08:48 */
  t4 = u_in[13] * b_gain_phi;
  t5 = u_in[12] * b_gain_theta;
  t6 = u_in[4] * t115;
  t7 = u_in[5] * t115;
  t8 = u_in[6] * t115;
  t9 = u_in[7] * t115;
  t10 = u_in[8] * b_gain_az;
  t11 = u_in[9] * b_gain_az;
  t12 = u_in[10] * b_gain_az;
  t13 = u_in[11] * b_gain_az;
  t15 = u_in[0] * u_in[0];
  t16 = u_in[1] * u_in[1];
  t17 = u_in[2] * u_in[2];
  t18 = u_in[3] * u_in[3];
  t19 = b_V * b_V;
  t22 = b_W_act_motor * b_W_act_motor;
  t29 = t81 * t81;
  t30 = t82 * t82;
  t32 = b_gain_motor * b_gain_motor;
  t69 = 1.0 / t43;
  t33 = cos(t4);
  t34 = cos(t5);
  t37 = sin(t4);
  t42 = cos(t6);
  t43 = cos(t7);
  t44 = cos(t8);
  t45 = cos(t9);
  t46 = sin(t5);
  t47 = cos(t10);
  t48 = cos(t11);
  t49 = cos(t12);
  t50 = cos(t13);
  t51 = sin(t6);
  t52 = sin(t7);
  t53 = sin(t8);
  t54 = sin(t9);
  t55 = sin(t10);
  t56 = sin(t11);
  t57 = sin(t12);
  t58 = sin(t13);
  t72 = t129 * (1.0 / t115);
  t73 = t127 * (1.0 / b_gain_az);
  t74 = b_desired_motor_value * (1.0 / b_gain_motor);
  t80 = b_flight_path_angle - t5;
  t11 = t184 * t15 * t32;
  t96 = t11 * t51;
  t12 = t184 * t16 * t32;
  t97 = t12 * t52;
  t13 = t184 * t17 * t32;
  t98 = t13 * t53;
  t115 = t184 * t18 * t32;
  t99 = t115 * t54;
  t127 = t11 * t42;
  t119 = t127 * t47;
  b_gain_az = t12 * t43;
  t121 = b_gain_az * t48;
  b_desired_motor_value = t13 * t44;
  t122 = b_desired_motor_value * t49;
  b_gain_motor = t115 * t45;
  t123 = b_gain_motor * t50;
  t127 *= t55;
  t129 = b_gain_az * t56;
  t12 = b_desired_motor_value * t57;
  t11 = b_gain_motor * t58;
  t81 = cos(t80);
  t82 = sin(t80);
  t5 = ((t96 + t97) + t98) + t99;
  t184 = ((t119 + t121) + t122) + t123;
  t185 = ((t127 + t129) + t12) + t11;
  t115 = b_Cd_zero + b_K_Cd * (b_Cl_alpha * b_Cl_alpha) * (t80 * t80);
  b_V = b_Cl_alpha * b_S * b_rho * t19 * t80;
  t10 = b_S * b_rho;
  b_W_act_motor = t10 * cos(b_Beta) * t19;
  t13 = b_V * t81 / 2.0 + b_W_act_motor * t82 * t115 / 2.0;
  t9 = b_V * t82 / 2.0 - b_W_act_motor * t81 * t115 / 2.0;
  t8 = b_K_p_M * t15 * t32;
  t7 = b_K_p_M * t17 * t32;
  t6 = b_K_p_M * t16 * t32;
  t82 = b_K_p_M * t18 * t32;
  t4 = b_I_xx * b_p;
  t43 *= t6;
  t81 = t82 * t45;
  b_W_act_motor = t8 * t42;
  b_V = t7 * t44;
  b_flight_path_angle = dv_global_6 - 1.0 / b_I_zz * (((((((((((((t4 * b_q -
    b_I_yy * b_p * b_q) + b_l_1 * t97) + b_l_2 * t98) - b_l_1 * t96) - b_l_2 *
    t99) + b_W_act_motor * t47) + b_V * t49) + b_l_4 * t127) + b_l_4 * t129) -
    t43 * t48) - t81 * t50) - b_l_3 * t12) - b_l_3 * t11);
  t129 = dv_global_4 - 1.0 / b_I_xx * ((((((((((((((b_I_yy * b_q * b_r - b_I_zz *
    b_q * b_r) + t8 * t51) + t7 * t53) + b_CL_aileron * b_S * u_in[14] *
    b_gain_ailerons * b_rho * t19 / 2.0) - t6 * t52) - t82 * t54) + b_l_1 * t119)
    + b_l_2 * t123) + b_l_z * t127) + b_l_z * t129) + b_l_z * t12) + b_l_z * t11)
    - b_l_1 * t121) - b_l_2 * t122);
  b_gain_motor = dv_global_5 - 1.0 / b_I_yy * ((((((((((((((b_I_zz * b_p * b_r -
    t4 * b_r) + b_l_z * t96) + b_l_z * t97) + b_l_z * t98) + b_l_z * t99) + t43 *
    t56) + t81 * t58) + b_l_4 * t119) + b_l_4 * t121) - b_W_act_motor * t55) -
    b_V * t57) - b_l_3 * t122) - b_l_3 * t123) + t10 * t19 * (b_Cm_zero -
    b_Cm_alpha * t80) * b_wing_chord / 2.0);
  t8 = t10 * sin(b_Beta) * t19;
  b_desired_motor_value = dv_global_2 + t69 * (((t8 * t33 * t115 / 2.0 + t37 *
    t13) - t37 * t184) - t33 * t185);
  t7 = t33 * t34;
  b_gain_az = (-dv_global_3 + -t69 * (((((t8 * t34 * t37 * t115 / 2.0 - t46 * t5)
    + t46 * t9) - t7 * t13) - t34 * t37 * t185) + t7 * t184)) + 9.81;
  t7 = t33 * t46;
  t127 = dv_global_1 + t69 * (((((t8 * t37 * t46 * t115 / 2.0 + t34 * t5) - t34 *
    t9) - t7 * t13) + t7 * t184) - t37 * t46 * t185);
  t10 = u_in[13] - b_desired_phi_value * (1.0 / b_gain_phi);
  t115 = u_in[12] - b_desired_theta_value * (1.0 / b_gain_theta);
  t7 = u_in[14] - b_desired_ailerons_value * (1.0 / b_gain_ailerons);
  t8 = u_in[0] - t74;
  t13 = u_in[1] - t74;
  t9 = u_in[2] - t74;
  t4 = u_in[3] - t74;
  t11 = u_in[4] - t72;
  t6 = u_in[5] - t72;
  t12 = u_in[6] - t72;
  b_W_act_motor = u_in[7] - t72;
  t81 = u_in[8] - t73;
  t82 = u_in[9] - t73;
  t43 = u_in[10] - t73;
  b_V = u_in[11] - t73;
  return (((((b_gamma_quadratic_du * ((((((((((((((b_W_act_phi * b_W_act_phi *
    (t10 * t10) + b_W_act_theta * b_W_act_theta * (t115 * t115)) +
    b_W_act_ailerons * b_W_act_ailerons * (t7 * t7)) + t22 * (t8 * t8)) + t22 *
    (t13 * t13)) + t22 * (t9 * t9)) + t22 * (t4 * t4)) + t29 * (t11 * t11)) +
    t29 * (t6 * t6)) + t29 * (t12 * t12)) + t29 * (b_W_act_motor * b_W_act_motor))
    + t30 * (t81 * t81)) + t30 * (t82 * t82)) + t30 * (t43 * t43)) + t30 * (b_V *
    b_V)) + b_W_dv_6 * b_W_dv_6 * (b_flight_path_angle * b_flight_path_angle)) +
             b_W_dv_4 * b_W_dv_4 * (t129 * t129)) + b_W_dv_5 * b_W_dv_5 *
            (b_gain_motor * b_gain_motor)) + b_W_dv_2 * b_W_dv_2 *
           (b_desired_motor_value * b_desired_motor_value)) + b_W_dv_1 *
          b_W_dv_1 * (t127 * t127)) + b_W_dv_3 * b_W_dv_3 * (b_gain_az *
    b_gain_az);
}

/*
 * Arguments    : const b_captured_var *dv_global
 *                const captured_var *Beta
 *                const captured_var *CL_aileron
 *                const captured_var *Cd_zero
 *                const captured_var *Cl_alpha
 *                const captured_var *Cm_zero
 *                const captured_var *Cm_alpha
 *                const captured_var *I_xx
 *                const captured_var *I_yy
 *                const captured_var *I_zz
 *                const captured_var *K_Cd
 *                const captured_var *K_p_M
 *                const captured_var *K_p_T
 *                const captured_var *Phi
 *                const captured_var *S
 *                const captured_var *Theta
 *                const captured_var *V
 *                const captured_var *W_act_motor
 *                const captured_var *W_dv_1
 *                const captured_var *W_dv_2
 *                const captured_var *W_dv_3
 *                const captured_var *W_dv_4
 *                const captured_var *W_dv_5
 *                const captured_var *W_dv_6
 *                const captured_var *W_act_tilt_el
 *                const captured_var *W_act_tilt_az
 *                const captured_var *W_act_ailerons
 *                const captured_var *desired_el_value
 *                const captured_var *desired_az_value
 *                const captured_var *desired_motor_value
 *                const captured_var *desired_ailerons_value
 *                const captured_var *flight_path_angle
 *                const captured_var *gain_el
 *                const captured_var *gain_az
 *                const captured_var *gain_motor
 *                const captured_var *gain_ailerons
 *                const captured_var *gamma_quadratic_du
 *                const captured_var *l_1
 *                const captured_var *l_2
 *                const captured_var *l_3
 *                const captured_var *l_4
 *                const captured_var *l_z
 *                const captured_var *m
 *                const captured_var *p
 *                const captured_var *q
 *                const captured_var *r
 *                const captured_var *rho
 *                const captured_var *wing_chord
 *                const double u_in[13]
 * Return Type  : double
 */
static double d_compute_cost_and_gradient_sec(const b_captured_var *dv_global,
  const captured_var *Beta, const captured_var *CL_aileron, const captured_var
  *Cd_zero, const captured_var *Cl_alpha, const captured_var *Cm_zero, const
  captured_var *Cm_alpha, const captured_var *I_xx, const captured_var *I_yy,
  const captured_var *I_zz, const captured_var *K_Cd, const captured_var *K_p_M,
  const captured_var *K_p_T, const captured_var *Phi, const captured_var *S,
  const captured_var *Theta, const captured_var *V, const captured_var
  *W_act_motor, const captured_var *W_dv_1, const captured_var *W_dv_2, const
  captured_var *W_dv_3, const captured_var *W_dv_4, const captured_var *W_dv_5,
  const captured_var *W_dv_6, const captured_var *W_act_tilt_el, const
  captured_var *W_act_tilt_az, const captured_var *W_act_ailerons, const
  captured_var *desired_el_value, const captured_var *desired_az_value, const
  captured_var *desired_motor_value, const captured_var *desired_ailerons_value,
  const captured_var *flight_path_angle, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *gain_motor, const captured_var
  *gain_ailerons, const captured_var *gamma_quadratic_du, const captured_var
  *l_1, const captured_var *l_2, const captured_var *l_3, const captured_var
  *l_4, const captured_var *l_z, const captured_var *m, const captured_var *p,
  const captured_var *q, const captured_var *r, const captured_var *rho, const
  captured_var *wing_chord, const double u_in[13])
{
  double b_CL_aileron;
  double b_Cd_zero;
  double b_Cl_alpha;
  double b_Cm_alpha;
  double b_Cm_zero;
  double b_I_xx;
  double b_I_yy;
  double b_I_zz;
  double b_K_Cd;
  double b_K_p_M;
  double b_S;
  double b_V;
  double b_W_act_ailerons;
  double b_W_act_motor;
  double b_W_act_tilt_el;
  double b_W_dv_1;
  double b_W_dv_2;
  double b_W_dv_3;
  double b_W_dv_4;
  double b_W_dv_5;
  double b_W_dv_6;
  double b_desired_ailerons_value;
  double b_desired_az_value;
  double b_desired_el_value;
  double b_gain_ailerons;
  double b_gain_az;
  double b_gain_el;
  double b_gamma_quadratic_du;
  double b_l_1;
  double b_l_2;
  double b_l_3;
  double b_l_4;
  double b_l_z;
  double b_p;
  double b_q;
  double b_r;
  double b_rho;
  double b_wing_chord;
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double t10;
  double t108;
  double t11;
  double t112;
  double t114;
  double t115;
  double t116;
  double t12;
  double t13;
  double t14;
  double t15;
  double t162;
  double t165;
  double t17;
  double t173;
  double t174;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t29;
  double t3;
  double t30;
  double t32;
  double t39;
  double t4;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t53;
  double t54;
  double t6;
  double t64;
  double t65;
  double t67;
  double t68;
  double t69;
  double t7;
  double t75;
  double t76;
  double t8;
  double t9;
  double t90;
  double t91;
  double t92;
  double t93;
  dv_global_1 = dv_global->contents[0];
  dv_global_2 = dv_global->contents[1];
  dv_global_3 = dv_global->contents[2];
  dv_global_4 = dv_global->contents[3];
  dv_global_5 = dv_global->contents[4];
  dv_global_6 = dv_global->contents[5];
  t165 = Beta->contents;
  b_CL_aileron = CL_aileron->contents;
  b_Cd_zero = Cd_zero->contents;
  b_Cl_alpha = Cl_alpha->contents;
  b_Cm_zero = Cm_zero->contents;
  b_Cm_alpha = Cm_alpha->contents;
  b_I_xx = I_xx->contents;
  b_I_yy = I_yy->contents;
  b_I_zz = I_zz->contents;
  b_K_Cd = K_Cd->contents;
  b_K_p_M = K_p_M->contents;
  t93 = K_p_T->contents;
  t15 = Phi->contents;
  b_S = S->contents;
  t116 = Theta->contents;
  b_V = V->contents;
  b_W_act_motor = W_act_motor->contents;
  b_W_dv_1 = W_dv_1->contents;
  b_W_dv_2 = W_dv_2->contents;
  b_W_dv_3 = W_dv_3->contents;
  b_W_dv_4 = W_dv_4->contents;
  b_W_dv_5 = W_dv_5->contents;
  b_W_dv_6 = W_dv_6->contents;
  b_W_act_tilt_el = W_act_tilt_el->contents;
  t32 = W_act_tilt_az->contents;
  b_W_act_ailerons = W_act_ailerons->contents;
  b_desired_el_value = desired_el_value->contents;
  b_desired_az_value = desired_az_value->contents;
  t114 = desired_motor_value->contents;
  b_desired_ailerons_value = desired_ailerons_value->contents;
  t115 = flight_path_angle->contents;
  b_gain_el = gain_el->contents;
  b_gain_az = gain_az->contents;
  t112 = gain_motor->contents;
  b_gain_ailerons = gain_ailerons->contents;
  b_gamma_quadratic_du = gamma_quadratic_du->contents;
  b_l_1 = l_1->contents;
  b_l_2 = l_2->contents;
  b_l_3 = l_3->contents;
  b_l_4 = l_4->contents;
  b_l_z = l_z->contents;
  t40 = m->contents;
  b_p = p->contents;
  b_q = q->contents;
  b_r = r->contents;
  b_rho = rho->contents;
  b_wing_chord = wing_chord->contents;

  /* COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2 */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT_SECOND_ITERATION_V2(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,K_p_M,K_p_T,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi,S,Theta,V,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_MOTOR,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_2,L_3,L_4,L_Z,M,P,Q,R,RHO,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     30-Nov-2023 18:08:51 */
  t3 = cos(t15);
  t4 = sin(t165);
  t5 = cos(t116);
  t6 = sin(t15);
  t7 = sin(t116);
  t8 = u_in[4] * b_gain_el;
  t9 = u_in[5] * b_gain_el;
  t10 = u_in[6] * b_gain_el;
  t11 = u_in[7] * b_gain_el;
  t12 = u_in[8] * b_gain_az;
  t13 = u_in[9] * b_gain_az;
  t14 = u_in[10] * b_gain_az;
  t15 = u_in[11] * b_gain_az;
  t17 = u_in[0] * u_in[0];
  t18 = u_in[1] * u_in[1];
  t19 = u_in[2] * u_in[2];
  t20 = u_in[3] * u_in[3];
  t21 = b_V * b_V;
  t22 = b_W_act_motor * b_W_act_motor;
  t29 = b_W_act_tilt_el * b_W_act_tilt_el;
  t30 = t32 * t32;
  t32 = t112 * t112;
  t64 = 1.0 / t40;
  t39 = cos(t8);
  t40 = cos(t9);
  t41 = cos(t10);
  t42 = cos(t11);
  t43 = cos(t12);
  t44 = cos(t13);
  t45 = cos(t14);
  t46 = cos(t15);
  t47 = sin(t8);
  t48 = sin(t9);
  t49 = sin(t10);
  t50 = sin(t11);
  t51 = sin(t12);
  t52 = sin(t13);
  t53 = sin(t14);
  t54 = sin(t15);
  t65 = t116 - t115;
  t67 = b_desired_el_value * (1.0 / b_gain_el);
  t68 = b_desired_az_value * (1.0 / b_gain_az);
  t69 = t114 * (1.0 / t112);
  t75 = cos(t65);
  t76 = sin(t65);
  t116 = t93 * t17 * t32;
  t90 = t116 * t47;
  t115 = t93 * t18 * t32;
  t91 = t115 * t48;
  b_gain_el = t93 * t19 * t32;
  t92 = b_gain_el * t49;
  t15 = t93 * t20 * t32;
  t93 = t15 * t50;
  b_desired_az_value = t116 * t39;
  t112 = b_desired_az_value * t43;
  b_V = t115 * t40;
  t114 = b_V * t44;
  b_W_act_motor = b_gain_el * t41;
  t115 = b_W_act_motor * t45;
  b_W_act_tilt_el = t15 * t42;
  t116 = b_W_act_tilt_el * t46;
  t15 = b_desired_az_value * t51;
  t13 = b_V * t52;
  t12 = b_W_act_motor * t53;
  t10 = b_W_act_tilt_el * t54;
  t14 = b_Cl_alpha * b_S * b_rho * t21 * t65;
  t162 = ((t90 + t91) + t92) + t93;
  t173 = ((t112 + t114) + t115) + t116;
  t174 = ((t15 + t13) + t12) + t10;
  t108 = b_Cd_zero + b_K_Cd * (b_Cl_alpha * b_Cl_alpha) * (t65 * t65);
  t11 = b_S * b_rho;
  t9 = t11 * cos(t165) * t21;
  t165 = t14 * t75 / 2.0 + t9 * t76 * t108 / 2.0;
  b_K_Cd = b_K_p_M * t17 * t32;
  b_Cl_alpha = b_K_p_M * t19 * t32;
  b_Cd_zero = b_K_p_M * t18 * t32;
  t32 *= b_K_p_M * t20;
  t8 = b_I_xx * b_p;
  t40 *= b_Cd_zero;
  b_W_act_tilt_el = t32 * t42;
  b_W_act_motor = b_K_Cd * t39;
  b_V = b_Cl_alpha * t41;
  b_gain_az = dv_global_6 - 1.0 / b_I_zz * (((((((((((((t8 * b_q - b_I_yy * b_p *
    b_q) + b_l_1 * t91) + b_l_2 * t92) - b_l_1 * t90) - b_l_2 * t93) +
    b_W_act_motor * t43) + b_V * t45) + b_l_4 * t15) + b_l_4 * t13) - t40 * t44)
    - b_W_act_tilt_el * t46) - b_l_3 * t12) - b_l_3 * t10);
  b_desired_el_value = dv_global_4 - 1.0 / b_I_xx * ((((((((((((((b_I_yy * b_q *
    b_r - b_I_zz * b_q * b_r) + b_K_Cd * t47) + b_Cl_alpha * t49) + b_CL_aileron
    * b_S * u_in[12] * b_gain_ailerons * b_rho * t21 / 2.0) - b_Cd_zero * t48) -
    t32 * t50) + b_l_1 * t112) + b_l_2 * t116) + b_l_z * t15) + b_l_z * t13) +
    b_l_z * t12) + b_l_z * t10) - b_l_1 * t114) - b_l_2 * t115);
  b_desired_az_value = dv_global_5 - 1.0 / b_I_yy * ((((((((((((((b_I_zz * b_p *
    b_r - t8 * b_r) + b_l_z * t90) + b_l_z * t91) + b_l_z * t92) + b_l_z * t93)
    + t40 * t52) + b_W_act_tilt_el * t54) + b_l_4 * t112) + b_l_4 * t114) -
    b_W_act_motor * t51) - b_V * t53) - b_l_3 * t115) - b_l_3 * t116) + t11 *
    t21 * (b_Cm_zero + b_Cm_alpha * t65) * b_wing_chord / 2.0);
  b_K_Cd = t3 * t7;
  b_Cl_alpha = t11 * t4;
  b_Cd_zero = t14 * t76 / 2.0 - t9 * t75 * t108 / 2.0;
  t14 = dv_global_1 + t64 * (((((b_Cl_alpha * t6 * t7 * t21 * t108 / 2.0 + t5 *
    t162) + -t5 * b_Cd_zero) + b_K_Cd * t165) + b_K_Cd * t173) - t6 * t7 * t174);
  b_gain_el = dv_global_2 - t64 * (((-(t11 * t3 * t4 * t21 * t108 / 2.0) + t6 *
    t165) + t6 * t173) + t3 * t174);
  t11 = u_in[12] - b_desired_ailerons_value * (1.0 / b_gain_ailerons);
  t115 = u_in[0] - t69;
  t12 = u_in[1] - t69;
  t13 = u_in[2] - t69;
  t8 = u_in[3] - t69;
  t9 = u_in[4] - t67;
  t116 = u_in[5] - t67;
  t10 = u_in[6] - t67;
  b_W_act_motor = u_in[7] - t67;
  b_W_act_tilt_el = u_in[8] - t68;
  t32 = u_in[9] - t68;
  t40 = u_in[10] - t68;
  b_V = u_in[11] - t68;
  b_K_Cd = t3 * t5;
  t15 = (dv_global_3 + t64 * (((((b_Cl_alpha * t5 * t6 * t21 * t108 / 2.0 - t7 *
              t162) + b_K_Cd * t165) + b_K_Cd * t173) - t5 * t6 * t174) + t7 *
          b_Cd_zero)) - 9.81;
  return (((((b_gamma_quadratic_du * ((((((((((((b_W_act_ailerons *
    b_W_act_ailerons * (t11 * t11) + t22 * (t115 * t115)) + t22 * (t12 * t12)) +
    t22 * (t13 * t13)) + t22 * (t8 * t8)) + t29 * (t9 * t9)) + t29 * (t116 *
    t116)) + t29 * (t10 * t10)) + t29 * (b_W_act_motor * b_W_act_motor)) + t30 *
    (b_W_act_tilt_el * b_W_act_tilt_el)) + t30 * (t32 * t32)) + t30 * (t40 * t40))
    + t30 * (b_V * b_V)) + b_W_dv_3 * b_W_dv_3 * (t15 * t15)) + b_W_dv_6 *
             b_W_dv_6 * (b_gain_az * b_gain_az)) + b_W_dv_4 * b_W_dv_4 *
            (b_desired_el_value * b_desired_el_value)) + b_W_dv_2 * b_W_dv_2 *
           (b_gain_el * b_gain_el)) + b_W_dv_5 * b_W_dv_5 * (b_desired_az_value *
           b_desired_az_value)) + b_W_dv_1 * b_W_dv_1 * (t14 * t14);
}

/*
 * Arguments    : const double lb[13]
 *                const double ub[13]
 *                n_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                const r_coder_internal_stickyStruct *FcnEvaluator
 *                k_struct_T *memspace
 *                o_struct_T *WorkingSet
 *                double Hessian[169]
 *                i_struct_T *QRManager
 *                j_struct_T *CholManager
 *                c_struct_T *QPObjective
 * Return Type  : void
 */
static void d_driver(const double lb[13], const double ub[13], n_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     r_coder_internal_stickyStruct *FcnEvaluator, k_struct_T
                     *memspace, o_struct_T *WorkingSet, double Hessian[169],
                     i_struct_T *QRManager, j_struct_T *CholManager, c_struct_T *
                     QPObjective)
{
  static const signed char iv[169] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const char qpoptions_SolverName[7] = { 'f', 'm', 'i', 'n', 'c', 'o',
    'n' };

  p_struct_T b_expl_temp;
  p_struct_T expl_temp;
  q_struct_T Flags;
  double varargout_2[13];
  int i;
  int ineqStart;
  int ixlast;
  int k;
  int mConstr;
  int mFixed;
  int mLB;
  int mUB;
  int nVar_tmp_tmp;
  int qpoptions_MaxIterations;
  memset(&QPObjective->grad[0], 0, 14U * sizeof(double));
  memset(&QPObjective->Hx[0], 0, 13U * sizeof(double));
  QPObjective->hasLinear = true;
  QPObjective->nvar = 13;
  QPObjective->maxVar = 14;
  QPObjective->beta = 0.0;
  QPObjective->rho = 0.0;
  QPObjective->objtype = 3;
  QPObjective->prev_objtype = 3;
  QPObjective->prev_nvar = 0;
  QPObjective->prev_hasLinear = false;
  QPObjective->gammaScalar = 0.0;
  CholManager->ldm = 27;
  CholManager->ndims = 0;
  CholManager->info = 0;
  CholManager->scaleFactor = 0.0;
  CholManager->ConvexCheck = true;
  CholManager->regTol_ = 1.7976931348623157E+308;
  CholManager->workspace_ = 1.7976931348623157E+308;
  CholManager->workspace2_ = 1.7976931348623157E+308;
  QRManager->ldq = 27;
  memset(&CholManager->FMat[0], 0, 729U * sizeof(double));
  memset(&QRManager->QR[0], 0, 729U * sizeof(double));
  memset(&QRManager->Q[0], 0, 729U * sizeof(double));
  QRManager->mrows = 0;
  QRManager->ncols = 0;
  memset(&QRManager->jpvt[0], 0, 27U * sizeof(int));
  memset(&QRManager->tau[0], 0, 27U * sizeof(double));
  QRManager->minRowCol = 0;
  QRManager->usedPivoting = false;
  for (i = 0; i < 169; i++) {
    Hessian[i] = iv[i];
  }

  nVar_tmp_tmp = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  ineqStart = WorkingSet->nVar;
  ixlast = (WorkingSet->sizes[3] + WorkingSet->sizes[4]) + (WorkingSet->sizes[0]
    << 1);
  if (ineqStart >= ixlast) {
    ixlast = ineqStart;
  }

  qpoptions_MaxIterations = 10 * ixlast;
  TrialState->steplength = 1.0;
  c_test_exit(MeritFunction, WorkingSet, TrialState, lb, ub, &Flags.fevalOK,
              &Flags.done, &Flags.stepAccepted, &Flags.failedLineSearch,
              &Flags.stepType);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (k = 0; k < 13; k++) {
    TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
    TrialState->grad_old[k] = TrialState->grad[k];
  }

  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    double phi_alpha;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        i = (unsigned char)mLB;
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet->lb[WorkingSet->indexLB[ixlast] - 1] = -lb
            [WorkingSet->indexLB[ixlast] - 1] + TrialState->xstarsqp
            [WorkingSet->indexLB[ixlast] - 1];
        }

        i = (unsigned char)mUB;
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet->ub[WorkingSet->indexUB[ixlast] - 1] = ub
            [WorkingSet->indexUB[ixlast] - 1] - TrialState->xstarsqp
            [WorkingSet->indexUB[ixlast] - 1];
        }

        i = (unsigned char)mFixed;
        for (ixlast = 0; ixlast < i; ixlast++) {
          phi_alpha = ub[WorkingSet->indexFixed[ixlast] - 1] -
            TrialState->xstarsqp[WorkingSet->indexFixed[ixlast] - 1];
          WorkingSet->ub[WorkingSet->indexFixed[ixlast] - 1] = phi_alpha;
          WorkingSet->bwset[ixlast] = phi_alpha;
        }

        if (WorkingSet->nActiveConstr > mFixed) {
          ineqStart = mFixed + 1;
          if (ineqStart < 1) {
            ineqStart = 1;
          }

          i = WorkingSet->nActiveConstr;
          for (ixlast = ineqStart; ixlast <= i; ixlast++) {
            switch (WorkingSet->Wid[ixlast - 1]) {
             case 4:
              WorkingSet->bwset[ixlast - 1] = WorkingSet->lb[WorkingSet->
                indexLB[WorkingSet->Wlocalidx[ixlast - 1] - 1] - 1];
              break;

             case 5:
              WorkingSet->bwset[ixlast - 1] = WorkingSet->ub[WorkingSet->
                indexUB[WorkingSet->Wlocalidx[ixlast - 1] - 1] - 1];
              break;

             default:
              /* A check that is always false is detected at compile-time. Eliminating code that follows. */
              break;
            }
          }
        }
      }

      expl_temp.ObjectiveLimit = -1.7976931348623157E+308;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }

      b_expl_temp = expl_temp;
      Flags.stepAccepted = b_step(&Flags.stepType, Hessian, lb, ub, TrialState,
        MeritFunction, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        &b_expl_temp);
      if (Flags.stepAccepted) {
        i = (unsigned char)nVar_tmp_tmp;
        for (ineqStart = 0; ineqStart < i; ineqStart++) {
          TrialState->xstarsqp[ineqStart] += TrialState->delta_x[ineqStart];
        }

        TrialState->sqpFval = d_compute_cost_and_gradient_sec
          (FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.dv_global,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Beta,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.CL_aileron,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cd_zero,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cl_alpha,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_zero,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_alpha,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_xx,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_yy,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_zz,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_Cd,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_M,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_T,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Phi,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.S,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Theta,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.V,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_motor,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_1,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_2,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_3,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_4,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_5,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_6,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_el,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_az,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_ailerons,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_el_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_az_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_motor_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_ailerons_value,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.flight_path_angle,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_el,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_az,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_motor,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_ailerons,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gamma_quadratic_du,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_1,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_2,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_3,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_4,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_z,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.m,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.p,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.q,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.r,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.rho,
           FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.wing_chord,
           TrialState->xstarsqp);
        Flags.fevalOK = true;
        TrialState->FunctionEvaluations++;
        MeritFunction->phiFullStep = TrialState->sqpFval;
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && (MeritFunction->phi <
           MeritFunction->phiFullStep) && (TrialState->sqpFval <
           TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        double alpha;
        int i1;
        bool socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }

        i = WorkingSet->nVar;
        alpha = 1.0;
        ineqStart = 1;
        phi_alpha = MeritFunction->phiFullStep;
        i1 = (unsigned char)WorkingSet->nVar;
        if (i1 - 1 >= 0) {
          memcpy(&TrialState->searchDir[0], &TrialState->delta_x[0], (unsigned
                  int)i1 * sizeof(double));
        }

        int exitg1;
        do {
          exitg1 = 0;
          if (TrialState->FunctionEvaluations < 4000 && toc() < second_opt_iter_max_time) {
            if (phi_alpha <= MeritFunction->phi + alpha * 0.0001 *
                MeritFunction->phiPrimePlus) {
              exitg1 = 1;
            } else {
              bool exitg2;
              bool tooSmallX;
              alpha *= 0.7;
              i1 = (unsigned char)i;
              for (ixlast = 0; ixlast < i1; ixlast++) {
                TrialState->delta_x[ixlast] = alpha * TrialState->xstar[ixlast];
              }

              if (socTaken) {
                phi_alpha = alpha * alpha;
                if ((i >= 1) && (phi_alpha != 0.0)) {
                  ixlast = i - 1;
                  for (k = 0; k <= ixlast; k++) {
                    TrialState->delta_x[k] += phi_alpha *
                      TrialState->socDirection[k];
                  }
                }
              }

              tooSmallX = true;
              ixlast = 0;
              exitg2 = false;
              while ((!exitg2) && (ixlast <= (unsigned char)i - 1)) {
                if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[ixlast])) <=
                    fabs(TrialState->delta_x[ixlast])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  ixlast++;
                }
              }

              if (tooSmallX) {
                ineqStart = -2;
                exitg1 = 1;
              } else {
                for (ixlast = 0; ixlast < i1; ixlast++) {
                  TrialState->xstarsqp[ixlast] = TrialState->xstarsqp_old[ixlast]
                    + TrialState->delta_x[ixlast];
                }

                TrialState->sqpFval = d_compute_cost_and_gradient_sec
                  (FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.dv_global,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Beta,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.CL_aileron,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cd_zero,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cl_alpha,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_zero,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_alpha,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_xx,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_yy,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_zz,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_Cd,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_M,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_T,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Phi,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.S,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Theta,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.V,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_motor,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_1,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_2,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_3,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_4,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_5,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_6,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_el,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_az,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_ailerons,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_el_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_az_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_motor_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_ailerons_value,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.flight_path_angle,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_el,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_az,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_motor,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_ailerons,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gamma_quadratic_du,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_1,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_2,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_3,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_4,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_z,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.m,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.p,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.q,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.r,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.rho,
                   FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.wing_chord,
                   TrialState->xstarsqp);
                TrialState->FunctionEvaluations++;
                phi_alpha = TrialState->sqpFval;
              }
            }
          } else {
            ineqStart = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);

        Flags.fevalOK = true;
        TrialState->steplength = alpha;
        if (ineqStart > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      i = (unsigned char)nVar_tmp_tmp;
      for (ixlast = 0; ixlast < i; ixlast++) {
        TrialState->xstarsqp[ixlast] = TrialState->xstarsqp_old[ixlast] +
          TrialState->delta_x[ixlast];
      }

      i = (unsigned char)mConstr;
      for (ixlast = 0; ixlast < i; ixlast++) {
        phi_alpha = TrialState->lambdasqp[ixlast];
        phi_alpha += TrialState->steplength * (TrialState->lambda[ixlast] -
          phi_alpha);
        TrialState->lambdasqp[ixlast] = phi_alpha;
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      for (k = 0; k < 13; k++) {
        TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
        TrialState->grad_old[k] = TrialState->grad[k];
      }

      TrialState->sqpFval = c_compute_cost_and_gradient_sec
        (FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.dv_global,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Beta,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.CL_aileron,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cd_zero,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cl_alpha,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_zero,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Cm_alpha,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_xx,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_yy,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.I_zz,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_Cd,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_M,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.K_p_T,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Phi,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.S,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.Theta,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.V,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_motor,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_1,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_2,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_3,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_4,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_5,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_dv_6,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_el,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_tilt_az,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.W_act_ailerons,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_el_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_az_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_motor_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.desired_ailerons_value,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.flight_path_angle,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_el,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_az,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_motor,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gain_ailerons,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.gamma_quadratic_du,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_1,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_2,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_3,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_4,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.l_z,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.m,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.p,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.q,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.r,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.rho,
         FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace.wing_chord,
         TrialState->xstarsqp, varargout_2);
      memcpy(&TrialState->grad[0], &varargout_2[0], 13U * sizeof(double));
      TrialState->FunctionEvaluations++;
      Flags.fevalOK = true;
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0], 13U *
             sizeof(double));
    }

    d_test_exit(&Flags, memspace, MeritFunction, WorkingSet, TrialState,
                QRManager, lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      i = (unsigned char)nVar_tmp_tmp;
      memcpy(&TrialState->delta_gradLag[0], &TrialState->grad[0], (unsigned int)
             i * sizeof(double));
      if (nVar_tmp_tmp >= 1) {
        ixlast = nVar_tmp_tmp - 1;
        for (k = 0; k <= ixlast; k++) {
          TrialState->delta_gradLag[k] -= TrialState->grad_old[k];
        }
      }

      b_BFGSUpdate(nVar_tmp_tmp, Hessian, TrialState->delta_x,
                   TrialState->delta_gradLag, memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/*
 * Arguments    : const o_struct_T *obj
 *                const double x[14]
 * Return Type  : double
 */
static double d_maxConstraintViolation(const o_struct_T *obj, const double x[14])
{
  double v;
  int i;
  int idx;
  int idxLB;
  v = 0.0;
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    i = (unsigned char)obj->sizes[4];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[idxLB] - obj->ub[idxLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      v = fmax(v, fabs(x[obj->indexFixed[idx] - 1] - obj->ub[obj->indexFixed[idx]
                       - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : q_struct_T *Flags
 *                k_struct_T *memspace
 *                b_struct_T *MeritFunction
 *                const o_struct_T *WorkingSet
 *                n_struct_T *TrialState
 *                i_struct_T *QRManager
 *                const double lb[13]
 *                const double ub[13]
 * Return Type  : void
 */
static void d_test_exit(q_struct_T *Flags, k_struct_T *memspace, b_struct_T
  *MeritFunction, const o_struct_T *WorkingSet, n_struct_T *TrialState,
  i_struct_T *QRManager, const double lb[13], const double ub[13])
{
  double d;
  double optimRelativeFactor;
  double s;
  double smax;
  int b_i;
  int b_k;
  int i;
  int i1;
  int idx_max;
  int k;
  int mFixed;
  int nVar;
  bool dxTooSmall;
  bool exitg1;
  bool isFeasible;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  i = (unsigned char)((WorkingSet->sizes[0] + WorkingSet->sizes[3]) +
                      WorkingSet->sizes[4]);
  if (i - 1 >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (unsigned
            int)i * sizeof(double));
  }

  c_computeGradLag(TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
                   WorkingSet->indexFixed, WorkingSet->sizes[0],
                   WorkingSet->indexLB, WorkingSet->sizes[3],
                   WorkingSet->indexUB, WorkingSet->sizes[4],
                   TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  optimRelativeFactor = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (optimRelativeFactor >= 1.7976931348623157E+308) {
    optimRelativeFactor = 1.0;
  }

  smax = 0.0;
  i1 = (unsigned char)WorkingSet->sizes[3];
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet->indexLB[idx_max] - 1;
    smax = fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }

  i1 = (unsigned char)WorkingSet->sizes[4];
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet->indexUB[idx_max] - 1;
    smax = fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  }

  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  dxTooSmall = true;
  smax = 0.0;
  i1 = (unsigned char)WorkingSet->nVar;
  idx_max = 0;
  exitg1 = false;
  while ((!exitg1) && (idx_max <= i1 - 1)) {
    d = fabs(TrialState->gradLag[idx_max]);
    dxTooSmall = (d < 1.7976931348623157E+308);
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = fmax(smax, d);
      idx_max++;
    }
  }

  MeritFunction->nlpDualFeasError = smax;
  if (!dxTooSmall) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = b_computeComplError(TrialState->xstarsqp,
      WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
      WorkingSet->sizes[4], ub, TrialState->lambdaStopTest, WorkingSet->sizes[0]
      + 1);
    d = fmax(smax, MeritFunction->nlpComplError);
    MeritFunction->firstOrderOpt = d;
    if (TrialState->sqpIterations > 1) {
      double nlpComplErrorTmp;
      d_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                       TrialState->grad, WorkingSet->indexFixed,
                       WorkingSet->sizes[0], WorkingSet->indexLB,
                       WorkingSet->sizes[3], WorkingSet->indexUB,
                       WorkingSet->sizes[4], TrialState->lambdaStopTestPrev);
      smax = 0.0;
      idx_max = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_max <= i1 - 1)) {
        s = fabs(memspace->workspace_double[idx_max]);
        if (s >= 1.7976931348623157E+308) {
          exitg1 = true;
        } else {
          smax = fmax(smax, s);
          idx_max++;
        }
      }

      nlpComplErrorTmp = b_computeComplError(TrialState->xstarsqp,
        WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
        WorkingSet->sizes[4], ub, TrialState->lambdaStopTestPrev,
        WorkingSet->sizes[0] + 1);
      s = fmax(smax, nlpComplErrorTmp);
      if (s < d) {
        MeritFunction->nlpDualFeasError = smax;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = s;
        if (i - 1 >= 0) {
          memcpy(&TrialState->lambdaStopTest[0], &TrialState->
                 lambdaStopTestPrev[0], (unsigned int)i * sizeof(double));
        }
      } else if (i - 1 >= 0) {
        memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
               (unsigned int)i * sizeof(double));
      }
    } else if (i - 1 >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (unsigned int)i * sizeof(double));
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-9 *
                       optimRelativeFactor) && (MeritFunction->nlpComplError <=
         1.0E-9 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        bool guard1;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          idx_max = 0;
          exitg1 = false;
          while ((!exitg1) && (idx_max <= i1 - 1)) {
            if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[idx_max])) <= fabs
                (TrialState->delta_x[idx_max])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx_max++;
            }
          }

          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              nVar = WorkingSet->nActiveConstr - 1;
              if (WorkingSet->nActiveConstr > 0) {
                int rankR;
                for (k = 0; k <= nVar; k++) {
                  TrialState->lambda[k] = 0.0;
                  idx_max = 14 * k;
                  rankR = 27 * k;
                  for (b_k = 0; b_k < i1; b_k++) {
                    QRManager->QR[rankR + b_k] = WorkingSet->ATwset[idx_max +
                      b_k];
                  }
                }

                QRManager->usedPivoting = true;
                QRManager->mrows = WorkingSet->nVar;
                QRManager->ncols = WorkingSet->nActiveConstr;
                nVar = WorkingSet->nVar;
                b_k = WorkingSet->nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }

                QRManager->minRowCol = b_k;
                b_xgeqp3(QRManager->QR, WorkingSet->nVar,
                         WorkingSet->nActiveConstr, QRManager->jpvt,
                         QRManager->tau);
                b_computeQ_(QRManager, WorkingSet->nVar);
                nVar = WorkingSet->nVar;
                idx_max = WorkingSet->nActiveConstr;
                if (nVar >= idx_max) {
                  idx_max = nVar;
                }

                smax = fabs(QRManager->QR[0]) * fmin(1.4901161193847656E-8,
                  (double)idx_max * 2.2204460492503131E-16);
                rankR = 0;
                nVar = 0;
                while ((rankR < b_k) && (fabs(QRManager->QR[nVar]) > smax)) {
                  rankR++;
                  nVar += 28;
                }

                d_xgemv(WorkingSet->nVar, WorkingSet->nVar, QRManager->Q,
                        TrialState->grad, memspace->workspace_double);
                if (rankR != 0) {
                  for (k = rankR; k >= 1; k--) {
                    nVar = (k + (k - 1) * 27) - 1;
                    memspace->workspace_double[k - 1] /= QRManager->QR[nVar];
                    for (b_i = 0; b_i <= k - 2; b_i++) {
                      idx_max = (k - b_i) - 2;
                      memspace->workspace_double[idx_max] -=
                        memspace->workspace_double[k - 1] * QRManager->QR[(nVar
                        - b_i) - 1];
                    }
                  }
                }

                nVar = WorkingSet->nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }

                nVar = (unsigned char)b_k;
                for (idx_max = 0; idx_max < nVar; idx_max++) {
                  TrialState->lambda[QRManager->jpvt[idx_max] - 1] =
                    memspace->workspace_double[idx_max];
                }

                nVar = WorkingSet->sizes[0] + 1;
                for (idx_max = nVar; idx_max <= mFixed; idx_max++) {
                  TrialState->lambda[idx_max - 1] = -TrialState->lambda[idx_max
                    - 1];
                }

                b_sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                               WorkingSet->sizes, WorkingSet->isActiveIdx,
                               WorkingSet->Wid, WorkingSet->Wlocalidx,
                               memspace->workspace_double);
                d_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                                 TrialState->grad, WorkingSet->indexFixed,
                                 WorkingSet->sizes[0], WorkingSet->indexLB,
                                 WorkingSet->sizes[3], WorkingSet->indexUB,
                                 WorkingSet->sizes[4], TrialState->lambda);
                s = 0.0;
                idx_max = 0;
                exitg1 = false;
                while ((!exitg1) && (idx_max <= i1 - 1)) {
                  d = fabs(memspace->workspace_double[idx_max]);
                  if (d >= 1.7976931348623157E+308) {
                    exitg1 = true;
                  } else {
                    s = fmax(s, d);
                    idx_max++;
                  }
                }

                smax = b_computeComplError(TrialState->xstarsqp,
                  WorkingSet->indexLB, WorkingSet->sizes[3], lb,
                  WorkingSet->indexUB, WorkingSet->sizes[4], ub,
                  TrialState->lambda, WorkingSet->sizes[0] + 1);
                if ((s <= 1.0E-9 * optimRelativeFactor) && (smax <= 1.0E-9 *
                     optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = s;
                  MeritFunction->nlpComplError = smax;
                  MeritFunction->firstOrderOpt = fmax(s, smax);
                  if (i - 1 >= 0) {
                    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0],
                           (unsigned int)i * sizeof(double));
                  }

                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TrialState->sqpIterations >= 800 || toc() >= second_opt_iter_max_time) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 4000 || toc() >= second_opt_iter_max_time) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const double A[729]
 *                int ia0
 *                const double B[378]
 *                double C[729]
 * Return Type  : void
 */
static void d_xgemm(int m, int n, int k, const double A[729], int ia0, const
                    double B[378], double C[729])
{
  int cr;
  int ic;
  int w;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = 27 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 27) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    br = -1;
    for (cr = 0; cr <= lastColC; cr += 27) {
      int ar;
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (w = 0; w < k; w++) {
          temp += A[(w + ar) - 1] * B[(w + br) + 1];
        }

        C[ic - 1] += temp;
        ar += 27;
      }

      br += 27;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[729]
 *                const double x[14]
 *                double y[378]
 * Return Type  : void
 */
static void d_xgemv(int m, int n, const double A[729], const double x[14],
                    double y[378])
{
  int ia;
  int iac;
  if (m != 0) {
    int i;
    memset(&y[0], 0, (unsigned int)n * sizeof(double));
    i = 27 * (n - 1) + 1;
    for (iac = 1; iac <= i; iac += 27) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }

      i1 = div_nde_s32_floor(iac - 1, 27);
      y[i1] += c;
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[14]
 * Return Type  : double
 */
static double d_xnrm2(int n, const double x[14])
{
  double y;
  int k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[0]);
    } else {
      double scale;
      int i;
      scale = 3.3121686421112381E-170;
      i = (unsigned short)n;
      for (k = 0; k < i; k++) {
        double absxk;
        absxk = fabs(x[k]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/*
 * Arguments    : f_struct_T *obj
 *                int idx
 * Return Type  : void
 */
static void deleteColMoveEnd(f_struct_T *obj, int idx)
{
  double s;
  double temp_tmp;
  int b_k;
  int i;
  int k;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    int b_i;
    int u0;
    b_i = obj->ncols - 1;
    obj->jpvt[idx - 1] = obj->jpvt[b_i];
    i = obj->minRowCol;
    for (k = 0; k < i; k++) {
      obj->QR[k + 31 * (idx - 1)] = obj->QR[k + 31 * b_i];
    }

    obj->ncols = b_i;
    u0 = obj->mrows;
    i = obj->ncols;
    if (u0 <= i) {
      i = u0;
    }

    obj->minRowCol = i;
    if (idx < obj->mrows) {
      double c;
      double temp;
      int QRk0;
      int b_temp_tmp;
      int endIdx;
      int n;
      u0 = obj->mrows - 1;
      endIdx = obj->ncols;
      if (u0 <= endIdx) {
        endIdx = u0;
      }

      k = endIdx;
      i = 31 * (idx - 1);
      while (k >= idx) {
        b_i = k + i;
        temp_tmp = obj->QR[b_i];
        c = xrotg(&obj->QR[b_i - 1], &temp_tmp, &s);
        obj->QR[b_i] = temp_tmp;
        b_i = 31 * (k - 1);
        obj->QR[k + b_i] = 0.0;
        QRk0 = k + 31 * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 31;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = b_i + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 31];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 31] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }

        k--;
      }

      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        u0 = 31 * (k - 1);
        i = k + u0;
        temp_tmp = obj->QR[i];
        c = xrotg(&obj->QR[i - 1], &temp_tmp, &s);
        obj->QR[i] = temp_tmp;
        QRk0 = k << 5;
        n = obj->ncols - k;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 31;
            temp_tmp = obj->QR[b_temp_tmp - 1];
            temp = c * temp_tmp + s * obj->QR[b_temp_tmp];
            obj->QR[b_temp_tmp] = c * obj->QR[b_temp_tmp] - s * temp_tmp;
            obj->QR[b_temp_tmp - 1] = temp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = u0 + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 31];
          temp = c * obj->Q[b_temp_tmp] + s * temp_tmp;
          obj->Q[b_temp_tmp + 31] = c * temp_tmp - s * obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp] = temp;
        }
      }
    }
  }
}

/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
static int div_nde_s32_floor(int numerator, int denominator)
{
  int i;
  if (((numerator < 0) != (denominator < 0)) && (numerator % denominator != 0))
  {
    i = -1;
  } else {
    i = 0;
  }

  return numerator / denominator + i;
}

/*
 * Arguments    : const double H[225]
 *                const double f[16]
 *                l_struct_T *solution
 *                h_struct_T *memspace
 *                m_struct_T *workingset
 *                f_struct_T *qrmanager
 *                g_struct_T *cholmanager
 *                struct_T *objective
 *                p_struct_T *options
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void driver(const double H[225], const double f[16], l_struct_T *solution,
                   h_struct_T *memspace, m_struct_T *workingset, f_struct_T
                   *qrmanager, g_struct_T *cholmanager, struct_T *objective,
                   p_struct_T *options, int runTimeOptions_MaxIterations)
{
  int idxStartIneq;
  int idx_global;
  int mConstr;
  int nVar_tmp;
  bool guard1;
  solution->iterations = 0;
  nVar_tmp = workingset->nVar;
  guard1 = false;
  if (workingset->probType == 3) {
    mConstr = (unsigned char)workingset->sizes[0];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      solution->xstar[workingset->indexFixed[idxStartIneq] - 1] = workingset->
        ub[workingset->indexFixed[idxStartIneq] - 1];
    }

    mConstr = (unsigned char)workingset->sizes[3];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + idxStartIneq)
          - 1]) {
        solution->xstar[workingset->indexLB[idxStartIneq] - 1] = -workingset->
          lb[workingset->indexLB[idxStartIneq] - 1];
      }
    }

    mConstr = (unsigned char)workingset->sizes[4];
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[4] + idxStartIneq)
          - 1]) {
        solution->xstar[workingset->indexUB[idxStartIneq] - 1] = workingset->
          ub[workingset->indexUB[idxStartIneq] - 1];
      }
    }

    PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution->state >= 0) {
      guard1 = true;
    }
  } else {
    solution->state = 82;
    guard1 = true;
  }

  if (guard1) {
    solution->iterations = 0;
    solution->maxConstr = c_maxConstraintViolation(workingset, solution->xstar);
    if (solution->maxConstr > 1.0E-6) {
      int PROBTYPE_ORIG;
      int idxEndIneq_tmp_tmp;
      PROBTYPE_ORIG = workingset->probType;
      solution->xstar[15] = solution->maxConstr + 1.0;
      if (workingset->probType == 3) {
        mConstr = 1;
      } else {
        mConstr = 4;
      }

      setProblemType(workingset, mConstr);
      mConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
      idxStartIneq = mConstr + 1;
      idxEndIneq_tmp_tmp = workingset->nActiveConstr;
      for (idx_global = idxStartIneq; idx_global <= idxEndIneq_tmp_tmp;
           idx_global++) {
        workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
          Wid[idx_global - 1] - 1] + workingset->Wlocalidx[idx_global - 1]) - 2]
          = false;
      }

      workingset->nWConstr[2] = 0;
      workingset->nWConstr[3] = 0;
      workingset->nWConstr[4] = 0;
      workingset->nActiveConstr = mConstr;
      objective->prev_objtype = objective->objtype;
      objective->prev_nvar = objective->nvar;
      objective->prev_hasLinear = objective->hasLinear;
      objective->objtype = 5;
      objective->nvar = 16;
      objective->gammaScalar = 1.0;
      objective->hasLinear = true;
      solution->fstar = computeFval(objective, memspace->workspace_double, H, f,
        solution->xstar);
      solution->state = 5;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options->SolverName, 1.4901161193847657E-10, 1.0E-6,
              runTimeOptions_MaxIterations);
      if (workingset->isActiveConstr[(workingset->isActiveIdx[3] +
           workingset->sizes[3]) - 2]) {
        bool exitg1;
        idxStartIneq = workingset->sizes[0];
        exitg1 = false;
        while ((!exitg1) && (idxStartIneq + 1 <= workingset->nActiveConstr)) {
          if ((workingset->Wid[idxStartIneq] == 4) && (workingset->
               Wlocalidx[idxStartIneq] == workingset->sizes[3])) {
            removeConstr(workingset, idxStartIneq + 1);
            exitg1 = true;
          } else {
            idxStartIneq++;
          }
        }
      }

      mConstr = workingset->nActiveConstr;
      idxStartIneq = workingset->sizes[0];
      while ((mConstr > idxStartIneq) && (mConstr > nVar_tmp)) {
        removeConstr(workingset, mConstr);
        mConstr--;
      }

      solution->maxConstr = solution->xstar[15];
      setProblemType(workingset, PROBTYPE_ORIG);
      objective->objtype = objective->prev_objtype;
      objective->nvar = objective->prev_nvar;
      objective->hasLinear = objective->prev_hasLinear;
      options->ObjectiveLimit = -1.7976931348623157E+308;
      options->StepTolerance = 1.0E-6;
      if (solution->state != 0) {
        solution->maxConstr = c_maxConstraintViolation(workingset,
          solution->xstar);
        if (solution->maxConstr > 1.0E-6) {
          memset(&solution->lambda[0], 0, 31U * sizeof(double));
          solution->fstar = computeFval(objective, memspace->workspace_double, H,
            f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            double maxConstr_new;
            mConstr = (unsigned char)nVar_tmp;
            if (mConstr - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (unsigned int)
                     mConstr * sizeof(double));
            }

            PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = c_maxConstraintViolation(workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (mConstr - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (unsigned
                        int)mConstr * sizeof(double));
              }
            }
          }

          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options->SolverName, options->StepTolerance,
                  options->ObjectiveLimit, runTimeOptions_MaxIterations);
        }
      }
    } else {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options->SolverName, options->StepTolerance,
              options->ObjectiveLimit, runTimeOptions_MaxIterations);
    }
  }
}

/*
 * Arguments    : f_struct_T *obj
 *                const double A[496]
 *                int mrows
 *                int ncols
 * Return Type  : void
 */
static void factorQR(f_struct_T *obj, const double A[496], int mrows, int ncols)
{
  int i;
  int idx;
  int k;
  bool guard1;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    for (idx = 0; idx < ncols; idx++) {
      int ix0;
      int iy0;
      ix0 = idx << 4;
      iy0 = 31 * idx;
      i = (unsigned char)mrows;
      for (k = 0; k < i; k++) {
        obj->QR[iy0 + k] = A[ix0 + k];
      }
    }

    guard1 = true;
  } else if (i == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt[idx] = idx + 1;
    }

    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }

    obj->minRowCol = i;
    memset(&obj->tau[0], 0, 31U * sizeof(double));
    if (i >= 1) {
      qrf(obj->QR, mrows, ncols, i, obj->tau);
    }
  }
}

/*
 * Arguments    : double fval
 *                b_struct_T *obj
 * Return Type  : void
 */
static void factoryConstruct(double fval, b_struct_T *obj)
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
 * Arguments    : double workspace[496]
 *                double xCurrent[16]
 *                const m_struct_T *workingset
 *                f_struct_T *qrmanager
 * Return Type  : bool
 */
static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const m_struct_T *workingset, f_struct_T *qrmanager)
{
  double B[496];
  int b_i;
  int br;
  int iAcol;
  int j;
  int jBcol;
  int k;
  int mWConstr;
  int nVar;
  bool nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int i;
    int i1;
    for (iAcol = 0; iAcol < mWConstr; iAcol++) {
      c = workingset->bwset[iAcol];
      workspace[iAcol] = c;
      workspace[iAcol + 31] = c;
    }

    if (mWConstr != 0) {
      i = ((mWConstr - 1) << 4) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 16) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (br = iAcol; br <= i1; br++) {
          c += workingset->ATwset[br - 1] * xCurrent[br - iAcol];
        }

        i1 = (iAcol - 1) >> 4;
        workspace[i1] -= c;
      }
    }

    if (mWConstr >= nVar) {
      i = (unsigned char)nVar;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (br = 0; br < i; br++) {
        iAcol = 31 * br;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] = workingset->ATwset[br + (jBcol << 4)];
        }

        qrmanager->jpvt[br] = br + 1;
      }

      if (mWConstr <= nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }

      qrmanager->minRowCol = i;
      memset(&qrmanager->tau[0], 0, 31U * sizeof(double));
      if (i >= 1) {
        qrf(qrmanager->QR, mWConstr, nVar, i, qrmanager->tau);
      }

      computeQ_(qrmanager, mWConstr);
      memcpy(&B[0], &workspace[0], 496U * sizeof(double));
      for (b_i = 0; b_i <= 31; b_i += 31) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = -1;
      for (b_i = 0; b_i <= 31; b_i += 31) {
        jBcol = -1;
        i = b_i + 1;
        i1 = b_i + nVar;
        for (k = i; k <= i1; k++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q[(iAcol + jBcol) + 1] * B[(iAcol + br) + 1];
          }

          workspace[k - 1] += c;
          jBcol += 31;
        }

        br += 31;
      }

      for (j = 0; j < 2; j++) {
        jBcol = 31 * j - 1;
        for (k = nVar; k >= 1; k--) {
          iAcol = 31 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager->QR[k + iAcol];
            i1 = (unsigned char)(k - 1);
            for (b_i = 0; b_i < i1; b_i++) {
              int i2;
              i2 = (b_i + jBcol) + 1;
              workspace[i2] -= workspace[i] * qrmanager->QR[(b_i + iAcol) + 1];
            }
          }
        }
      }
    } else {
      factorQR(qrmanager, workingset->ATwset, nVar, mWConstr);
      computeQ_(qrmanager, qrmanager->minRowCol);
      for (j = 0; j < 2; j++) {
        jBcol = 31 * j;
        for (b_i = 0; b_i < mWConstr; b_i++) {
          iAcol = 31 * b_i;
          br = b_i + jBcol;
          c = workspace[br];
          i = (unsigned char)b_i;
          for (k = 0; k < i; k++) {
            c -= qrmanager->QR[k + iAcol] * workspace[k + jBcol];
          }

          workspace[br] = c / qrmanager->QR[b_i + iAcol];
        }
      }

      memcpy(&B[0], &workspace[0], 496U * sizeof(double));
      for (b_i = 0; b_i <= 31; b_i += 31) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof
                 (double));
        }
      }

      br = 0;
      for (b_i = 0; b_i <= 31; b_i += 31) {
        jBcol = -1;
        i = br + 1;
        i1 = br + mWConstr;
        for (j = i; j <= i1; j++) {
          int i2;
          i2 = b_i + 1;
          iAcol = b_i + nVar;
          for (k = i2; k <= iAcol; k++) {
            workspace[k - 1] += B[j - 1] * qrmanager->Q[(jBcol + k) - b_i];
          }

          jBcol += 31;
        }

        br += 31;
      }
    }

    iAcol = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iAcol <= (unsigned char)nVar - 1) {
        if ((fabs(workspace[iAcol]) >= 1.7976931348623157E+308) || (fabs
             (workspace[iAcol + 31]) >= 1.7976931348623157E+308)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          iAcol++;
        }
      } else {
        double constrViolation_basicX;
        iAcol = nVar - 1;
        for (k = 0; k <= iAcol; k++) {
          workspace[k] += xCurrent[k];
        }

        c = maxConstraintViolation(workingset, workspace, 1);
        constrViolation_basicX = maxConstraintViolation(workingset, workspace,
          32);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[0], (unsigned int)i * sizeof(double));
        } else {
          i = (unsigned char)nVar;
          memcpy(&xCurrent[0], &workspace[31], (unsigned int)i * sizeof(double));
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

/*
 * Arguments    : const double solution_xstar[16]
 *                const double solution_searchDir[16]
 *                int workingset_nVar
 *                const double workingset_lb[16]
 *                const double workingset_ub[16]
 *                const int workingset_indexLB[16]
 *                const int workingset_indexUB[16]
 *                const int workingset_sizes[5]
 *                const int workingset_isActiveIdx[6]
 *                const bool workingset_isActiveConstr[31]
 *                const int workingset_nWConstr[5]
 *                bool isPhaseOne
 *                bool *newBlocking
 *                int *constrType
 *                int *constrIdx
 * Return Type  : double
 */
static double feasibleratiotest(const double solution_xstar[16], const double
  solution_searchDir[16], int workingset_nVar, const double workingset_lb[16],
  const double workingset_ub[16], const int workingset_indexLB[16], const int
  workingset_indexUB[16], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[31], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx)
{
  double alpha;
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  int i;
  int i1;
  int idx;
  alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * b_xnrm2(workingset_nVar,
    solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = (double)isPhaseOne * solution_xstar[workingset_nVar -
      1];
    phaseOneCorrectionP = (double)isPhaseOne *
      solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      i1 = workingset_indexLB[idx];
      pk_corrected = -solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[i1 - 1] - workingset_lb[i1 - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }

    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    pk_corrected = -solution_searchDir[i];
    if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[i] - workingset_lb[i];
      pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
      if (pk_corrected < alpha) {
        alpha = pk_corrected;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = (double)isPhaseOne * solution_xstar[workingset_nVar -
      1];
    phaseOneCorrectionP = (double)isPhaseOne *
      solution_searchDir[workingset_nVar - 1];
    i = (unsigned char)workingset_sizes[4];
    for (idx = 0; idx < i; idx++) {
      i1 = workingset_indexUB[idx];
      pk_corrected = solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[i1 - 1] - workingset_ub[i1 - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (!isPhaseOne) {
    if ((*newBlocking) && (alpha > 1.0)) {
      *newBlocking = false;
    }

    alpha = fmin(alpha, 1.0);
  }

  return alpha;
}

/*
 * Arguments    : d_struct_T *fun_workspace
 *                double x0[15]
 *                const double lb[15]
 *                const double ub[15]
 *                double *exitflag
 *                double *output_iterations
 *                double *output_funcCount
 *                char output_algorithm[3]
 *                double *output_constrviolation
 *                double *output_stepsize
 *                double *output_lssteplength
 *                double *output_firstorderopt
 * Return Type  : double
 */
static double fmincon(d_struct_T *fun_workspace, double x0[15], const double lb
                      [15], const double ub[15], double *exitflag, double
                      *output_iterations, double *output_funcCount, char
                      output_algorithm[3], double *output_constrviolation,
                      double *output_stepsize, double *output_lssteplength,
                      double *output_firstorderopt)
{
  b_struct_T MeritFunction;
  f_struct_T QRManager;
  g_struct_T CholManager;
  h_struct_T memspace;
  i_coder_internal_stickyStruct r;
  l_struct_T TrialState;
  m_struct_T WorkingSet;
  struct_T QPObjective;
  double varargout_2[15];
  double fval;
  int b_i;
  int i;
  int idx;
  int mFixed;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  bool exitg1;
  *exitflag = 1.7976931348623157E+308;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 15)) {
    if (lb[i] > ub[i]) {
      *exitflag = -2.0;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (*exitflag == -2.0) {
    fval = 1.7976931348623157E+308;
    *output_iterations = 0.0;
    *output_funcCount = 0.0;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = 1.7976931348623157E+308;
    *output_stepsize = 1.7976931348623157E+308;
    *output_lssteplength = 1.7976931348623157E+308;
    *output_firstorderopt = 1.7976931348623157E+308;
  } else {
    double scale;
    double y;
    int mLB;
    int mUB;
    TrialState.nVarMax = 16;
    TrialState.mNonlinIneq = 0;
    TrialState.mNonlinEq = 0;
    TrialState.mIneq = 0;
    TrialState.mEq = 0;
    TrialState.iNonIneq0 = 1;
    TrialState.iNonEq0 = 1;
    TrialState.sqpFval_old = 0.0;
    TrialState.sqpIterations = 0;
    TrialState.sqpExitFlag = 0;
    memset(&TrialState.lambdasqp[0], 0, 31U * sizeof(double));
    TrialState.steplength = 1.0;
    memset(&TrialState.delta_x[0], 0, 16U * sizeof(double));
    TrialState.fstar = 0.0;
    TrialState.firstorderopt = 0.0;
    memset(&TrialState.lambda[0], 0, 31U * sizeof(double));
    TrialState.state = 0;
    TrialState.maxConstr = 0.0;
    TrialState.iterations = 0;
    memcpy(&TrialState.xstarsqp[0], &x0[0], 15U * sizeof(double));
    WorkingSet.nVar = 15;
    WorkingSet.nVarOrig = 15;
    WorkingSet.nVarMax = 16;
    WorkingSet.ldA = 16;
    memset(&WorkingSet.lb[0], 0, 16U * sizeof(double));
    memset(&WorkingSet.ub[0], 0, 16U * sizeof(double));
    WorkingSet.mEqRemoved = 0;
    memset(&WorkingSet.ATwset[0], 0, 496U * sizeof(double));
    WorkingSet.nActiveConstr = 0;
    memset(&WorkingSet.bwset[0], 0, 31U * sizeof(double));
    memset(&WorkingSet.maxConstrWorkspace[0], 0, 31U * sizeof(double));
    memset(&WorkingSet.Wid[0], 0, 31U * sizeof(int));
    memset(&WorkingSet.Wlocalidx[0], 0, 31U * sizeof(int));
    for (i = 0; i < 31; i++) {
      WorkingSet.isActiveConstr[i] = false;
    }

    for (i = 0; i < 5; i++) {
      WorkingSet.nWConstr[i] = 0;
    }

    WorkingSet.probType = 3;
    WorkingSet.SLACK0 = 1.0E-5;
    memset(&WorkingSet.indexLB[0], 0, 16U * sizeof(int));
    memset(&WorkingSet.indexUB[0], 0, 16U * sizeof(int));
    memset(&WorkingSet.indexFixed[0], 0, 16U * sizeof(int));
    mLB = 0;
    mUB = 0;
    mFixed = 0;
    for (idx = 0; idx < 15; idx++) {
      bool guard1;
      y = lb[idx];
      guard1 = false;
      if (y > -1.7976931348623157E+308) {
        if (fabs(y - ub[idx]) < 1.0E-6) {
          mFixed++;
          WorkingSet.indexFixed[mFixed - 1] = idx + 1;
        } else {
          mLB++;
          WorkingSet.indexLB[mLB - 1] = idx + 1;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1 && (ub[idx] < 1.7976931348623157E+308)) {
        mUB++;
        WorkingSet.indexUB[mUB - 1] = idx + 1;
      }
    }

    i = (mLB + mUB) + mFixed;
    WorkingSet.mConstr = i;
    WorkingSet.mConstrOrig = i;
    WorkingSet.mConstrMax = 31;
    obj_tmp[0] = (signed char)mFixed;
    obj_tmp[1] = 0;
    obj_tmp[2] = 0;
    obj_tmp[3] = (signed char)mLB;
    obj_tmp[4] = (signed char)mUB;
    b_obj_tmp[0] = (signed char)mFixed;
    b_obj_tmp[1] = 0;
    b_obj_tmp[2] = 0;
    b_obj_tmp[3] = (signed char)(mLB + 1);
    b_obj_tmp[4] = (signed char)mUB;
    WorkingSet.isActiveIdx[0] = 1;
    WorkingSet.isActiveIdx[1] = mFixed;
    WorkingSet.isActiveIdx[2] = 0;
    WorkingSet.isActiveIdx[3] = 0;
    WorkingSet.isActiveIdx[4] = mLB;
    WorkingSet.isActiveIdx[5] = mUB;
    for (i = 0; i < 5; i++) {
      signed char i1;
      signed char i2;
      i1 = obj_tmp[i];
      WorkingSet.sizes[i] = i1;
      WorkingSet.sizesNormal[i] = i1;
      i2 = b_obj_tmp[i];
      WorkingSet.sizesPhaseOne[i] = i2;
      WorkingSet.sizesRegularized[i] = i1;
      WorkingSet.sizesRegPhaseOne[i] = i2;
      WorkingSet.isActiveIdx[i + 1] += WorkingSet.isActiveIdx[i];
    }

    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxNormal[b_i] = WorkingSet.isActiveIdx[b_i];
    }

    WorkingSet.isActiveIdxPhaseOne[0] = 1;
    WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxPhaseOne[2] = 0;
    WorkingSet.isActiveIdxPhaseOne[3] = 0;
    WorkingSet.isActiveIdxPhaseOne[4] = mLB + 1;
    WorkingSet.isActiveIdxPhaseOne[5] = mUB;
    for (i = 0; i < 5; i++) {
      WorkingSet.isActiveIdxPhaseOne[i + 1] += WorkingSet.isActiveIdxPhaseOne[i];
    }

    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
      WorkingSet.isActiveIdxRegPhaseOne[b_i] =
        WorkingSet.isActiveIdxPhaseOne[b_i];
    }

    for (idx = 0; idx < mLB; idx++) {
      b_i = WorkingSet.indexLB[idx];
      TrialState.xstarsqp[b_i - 1] = fmax(TrialState.xstarsqp[b_i - 1], lb[b_i -
        1]);
    }

    for (idx = 0; idx < mUB; idx++) {
      b_i = WorkingSet.indexUB[idx];
      TrialState.xstarsqp[b_i - 1] = fmin(TrialState.xstarsqp[b_i - 1], ub[b_i -
        1]);
    }

    for (idx = 0; idx < mFixed; idx++) {
      b_i = WorkingSet.indexFixed[idx];
      TrialState.xstarsqp[b_i - 1] = ub[b_i - 1];
    }

    TrialState.sqpFval = c_compute_cost_and_gradient_fir
      (fun_workspace->dv_global, fun_workspace->Beta, fun_workspace->CL_aileron,
       fun_workspace->Cd_zero, fun_workspace->Cl_alpha, fun_workspace->Cm_zero,
       fun_workspace->Cm_alpha, fun_workspace->I_xx, fun_workspace->I_yy,
       fun_workspace->I_zz, fun_workspace->K_Cd, fun_workspace->K_p_M,
       fun_workspace->K_p_T, fun_workspace->S, fun_workspace->V,
       fun_workspace->W_act_phi, fun_workspace->W_act_theta,
       fun_workspace->W_act_motor, fun_workspace->W_dv_1, fun_workspace->W_dv_2,
       fun_workspace->W_dv_3, fun_workspace->W_dv_4, fun_workspace->W_dv_5,
       fun_workspace->W_dv_6, fun_workspace->W_act_tilt_el,
       fun_workspace->W_act_tilt_az, fun_workspace->W_act_ailerons,
       fun_workspace->desired_el_value, fun_workspace->desired_az_value,
       fun_workspace->desired_phi_value, fun_workspace->desired_theta_value,
       fun_workspace->desired_motor_value, fun_workspace->desired_ailerons_value,
       fun_workspace->flight_path_angle, fun_workspace->gain_el,
       fun_workspace->gain_az, fun_workspace->gain_phi,
       fun_workspace->gain_theta, fun_workspace->gain_motor,
       fun_workspace->gain_ailerons, fun_workspace->gamma_quadratic_du,
       fun_workspace->l_1, fun_workspace->l_2, fun_workspace->l_3,
       fun_workspace->l_4, fun_workspace->l_z, fun_workspace->m,
       fun_workspace->p, fun_workspace->q, fun_workspace->r, fun_workspace->rho,
       fun_workspace->wing_chord, TrialState.xstarsqp, varargout_2);
    memcpy(&TrialState.grad[0], &varargout_2[0], 15U * sizeof(double));
    TrialState.FunctionEvaluations = 1;
    for (idx = 0; idx < mLB; idx++) {
      WorkingSet.lb[WorkingSet.indexLB[idx] - 1] = -lb[WorkingSet.indexLB[idx] -
        1] + x0[WorkingSet.indexLB[idx] - 1];
    }

    for (idx = 0; idx < mUB; idx++) {
      WorkingSet.ub[WorkingSet.indexUB[idx] - 1] = ub[WorkingSet.indexUB[idx] -
        1] - x0[WorkingSet.indexUB[idx] - 1];
    }

    for (idx = 0; idx < mFixed; idx++) {
      y = ub[WorkingSet.indexFixed[idx] - 1] - x0[WorkingSet.indexFixed[idx] - 1];
      WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = y;
      WorkingSet.bwset[idx] = y;
    }

    setProblemType(&WorkingSet, 3);
    i = WorkingSet.isActiveIdx[2];
    for (idx = i; idx < 32; idx++) {
      WorkingSet.isActiveConstr[idx - 1] = false;
    }

    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    b_i = (unsigned char)WorkingSet.sizes[0];
    for (mFixed = 0; mFixed < b_i; mFixed++) {
      WorkingSet.Wid[mFixed] = 1;
      WorkingSet.Wlocalidx[mFixed] = mFixed + 1;
      WorkingSet.isActiveConstr[mFixed] = true;
      i = mFixed << 4;
      mLB = WorkingSet.indexFixed[mFixed];
      if (mLB - 2 >= 0) {
        memset(&WorkingSet.ATwset[i], 0, (unsigned int)(((mLB + i) - i) - 1) *
               sizeof(double));
      }

      WorkingSet.ATwset[(WorkingSet.indexFixed[mFixed] + i) - 1] = 1.0;
      mLB = WorkingSet.indexFixed[mFixed] + 1;
      mUB = WorkingSet.nVar;
      if (mLB <= mUB) {
        memset(&WorkingSet.ATwset[(mLB + i) + -1], 0, (unsigned int)((((mUB + i)
                  - mLB) - i) + 1) * sizeof(double));
      }

      WorkingSet.bwset[mFixed] = WorkingSet.ub[WorkingSet.indexFixed[mFixed] - 1];
    }

    double Hessian[225];
    factoryConstruct(TrialState.sqpFval, &MeritFunction);
    r.next.next.next.next.next.next.next.next.value.workspace = *fun_workspace;
    c_driver(lb, ub, &TrialState, &MeritFunction, &r, &memspace, &WorkingSet,
             Hessian, &QRManager, &CholManager, &QPObjective);
    fval = TrialState.sqpFval;
    *exitflag = TrialState.sqpExitFlag;
    *output_iterations = TrialState.sqpIterations;
    *output_funcCount = TrialState.FunctionEvaluations;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = MeritFunction.nlpPrimalFeasError;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (i = 0; i < 15; i++) {
      double absxk;
      x0[i] = TrialState.xstarsqp[i];
      absxk = fabs(TrialState.delta_x[i]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        y += t * t;
      }
    }

    *output_stepsize = scale * sqrt(y);
    *output_lssteplength = TrialState.steplength;
    *output_firstorderopt = MeritFunction.firstOrderOpt;
  }

  return fval;
}

/*
 * Arguments    : g_struct_T *obj
 *                int NColsRemain
 * Return Type  : void
 */
static void fullColLDL2_(g_struct_T *obj, int NColsRemain)
{
  int ijA;
  int j;
  int jA;
  int k;
  for (k = 0; k < NColsRemain; k++) {
    double alpha1;
    double y;
    int LD_diagOffset;
    int i;
    int offset1;
    int subMatrixDim;
    LD_diagOffset = k << 5;
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 2;
    y = obj->workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj->FMat[(LD_diagOffset + jA) + 1];
    }

    obj->workspace_ = y;
    if (alpha1 != 0.0) {
      jA = LD_diagOffset;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 33;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 33; ijA++) {
            obj->FMat[ijA - 1] += y * temp;
          }
        }

        jA += 31;
      }
    }

    alpha1 = 1.0 / obj->FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i + 2; jA++) {
      obj->FMat[jA - 1] *= alpha1;
    }
  }
}

/*
 * Arguments    : const double H[225]
 *                const double f[16]
 *                l_struct_T *solution
 *                h_struct_T *memspace
 *                m_struct_T *workingset
 *                f_struct_T *qrmanager
 *                g_struct_T *cholmanager
 *                struct_T *objective
 *                const char options_SolverName[7]
 *                double options_StepTolerance
 *                double options_ObjectiveLimit
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void iterate(const double H[225], const double f[16], l_struct_T
                    *solution, h_struct_T *memspace, m_struct_T *workingset,
                    f_struct_T *qrmanager, g_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations)
{
  static const char b[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  double s;
  double temp;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int i;
  int idx;
  int iyend;
  int nActiveConstr;
  int nVar;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double, f,
    solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  memset(&solution->lambda[0], 0, 31U * sizeof(double));
  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      double minLambda;
      int idxQR;
      int ix0;
      bool guard1;
      bool guard2;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          {
            double c;
            ix0 = (workingset->nActiveConstr - 1) << 4;
            iyend = qrmanager->mrows;
            idxQR = qrmanager->ncols + 1;
            if (iyend <= idxQR) {
              idxQR = iyend;
            }

            qrmanager->minRowCol = idxQR;
            idxQR = 31 * qrmanager->ncols;
            if (qrmanager->mrows != 0) {
              iyend = idxQR + qrmanager->mrows;
              if (idxQR + 1 <= iyend) {
                memset(&qrmanager->QR[idxQR], 0, (unsigned int)(iyend - idxQR) *
                       sizeof(double));
              }

              i = 31 * (qrmanager->mrows - 1) + 1;
              for (iyend = 1; iyend <= i; iyend += 31) {
                c = 0.0;
                nActiveConstr = (iyend + qrmanager->mrows) - 1;
                for (idx = iyend; idx <= nActiveConstr; idx++) {
                  c += qrmanager->Q[idx - 1] * workingset->ATwset[(ix0 + idx) -
                    iyend];
                }

                nActiveConstr = idxQR + div_nde_s32_floor(iyend - 1, 31);
                qrmanager->QR[nActiveConstr] += c;
              }
            }

            qrmanager->ncols++;
            i = qrmanager->ncols - 1;
            qrmanager->jpvt[i] = qrmanager->ncols;
            for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--)
            {
              nActiveConstr = idx + 31 * i;
              temp = qrmanager->QR[nActiveConstr + 1];
              c = xrotg(&qrmanager->QR[nActiveConstr], &temp, &s);
              qrmanager->QR[nActiveConstr + 1] = temp;
              iyend = 31 * idx;
              ix0 = qrmanager->mrows;
              if (qrmanager->mrows >= 1) {
                for (nActiveConstr = 0; nActiveConstr < ix0; nActiveConstr++) {
                  idxQR = iyend + nActiveConstr;
                  minLambda = qrmanager->Q[idxQR + 31];
                  temp = c * qrmanager->Q[idxQR] + s * minLambda;
                  qrmanager->Q[idxQR + 31] = c * minLambda - s * qrmanager->
                    Q[idxQR];
                  qrmanager->Q[idxQR] = temp;
                }
              }
            }
          }
          break;

         case -1:
          deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          factorQR(qrmanager, workingset->ATwset, nVar,
                   workingset->nActiveConstr);
          computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        iyend = memcmp(&options_SolverName[0], &b[0], 7);
        compute_deltax(H, solution, memspace, qrmanager, cholmanager, objective,
                       iyend == 0);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((b_xnrm2(nVar, solution->searchDir) < options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar)) {
          guard2 = true;
        } else {
          minLambda = feasibleratiotest(solution->xstar, solution->searchDir,
            workingset->nVar, workingset->lb, workingset->ub,
            workingset->indexLB, workingset->indexUB, workingset->sizes,
            workingset->isActiveIdx, workingset->isActiveConstr,
            workingset->nWConstr, TYPE == 5, &updateFval, &i, &iyend);
          if (updateFval) {
            switch (i) {
             case 3:
              workingset->nWConstr[2]++;
              workingset->isActiveConstr[(workingset->isActiveIdx[2] + iyend) -
                2] = true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] = iyend;

              /* A check that is always false is detected at compile-time. Eliminating code that follows. */
              break;

             case 4:
              addBoundToActiveSetMatrix_(workingset, 4, iyend);
              break;

             default:
              addBoundToActiveSetMatrix_(workingset, 5, iyend);
              break;
            }

            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (b_xnrm2(objective->nvar, solution->searchDir) > 100.0 *
                  (double)objective->nvar * 1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }

            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if ((nVar >= 1) && (minLambda != 0.0)) {
            iyend = nVar - 1;
            for (nActiveConstr = 0; nActiveConstr <= iyend; nActiveConstr++) {
              solution->xstar[nActiveConstr] += minLambda * solution->
                searchDir[nActiveConstr];
            }
          }

          computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
          guard1 = true;
        }
      } else {
        i = (unsigned char)nVar;
        memset(&solution->searchDir[0], 0, (unsigned int)i * sizeof(double));
        guard2 = true;
      }

      if (guard2) {
        nActiveConstr = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          bool b_guard1;
          b_guard1 = false;
          if (objective->objtype != 4) {
            minLambda = 100.0 * (double)qrmanager->mrows *
              2.2204460492503131E-16;
            if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }

            if (updateFval) {
              bool b_guard2;
              idx = qrmanager->ncols;
              b_guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                idxQR = qrmanager->mrows + 31 * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) && (fabs(qrmanager->QR[idxQR - 1])
                        >= minLambda)) {
                  idx--;
                  idxQR -= 31;
                }

                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }

              if (b_guard2) {
                idxQR = idx + 31 * (idx - 1);
                while ((idx >= 1) && (fabs(qrmanager->QR[idxQR - 1]) >=
                                      minLambda)) {
                  idx--;
                  idxQR -= 32;
                }

                updateFval = (idx == 0);
              }
            }

            if (!updateFval) {
              solution->state = -7;
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }

          if (b_guard1) {
            ix0 = qrmanager->ncols;
            c_xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q,
                    objective->grad, memspace->workspace_double);
            if (qrmanager->ncols != 0) {
              for (idx = ix0; idx >= 1; idx--) {
                iyend = (idx + (idx - 1) * 31) - 1;
                memspace->workspace_double[idx - 1] /= qrmanager->QR[iyend];
                for (i = 0; i <= idx - 2; i++) {
                  idxQR = (idx - i) - 2;
                  memspace->workspace_double[idxQR] -=
                    memspace->workspace_double[idx - 1] * qrmanager->QR[(iyend -
                    i) - 1];
                }
              }
            }

            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }

        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          iyend = 0;
          minLambda = 0.0;
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          nActiveConstr = workingset->nActiveConstr;
          for (idx = i; idx <= nActiveConstr; idx++) {
            temp = solution->lambda[idx - 1];
            if (temp < minLambda) {
              minLambda = temp;
              iyend = idx;
            }
          }

          if (iyend == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iyend;
            subProblemChanged = true;
            removeConstr(workingset, iyend);
            solution->lambda[iyend - 1] = 0.0;
          }
        } else {
          iyend = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda[iyend - 1] = 0.0;
        }

        updateFval = false;
        guard1 = true;
      }

      if (guard1) {
        solution->iterations++;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = c_maxConstraintViolation(workingset,
            solution->xstar);
          minLambda = solution->maxConstr;
          if (objective->objtype == 5) {
            minLambda = solution->maxConstr - solution->xstar[objective->nvar -
              1];
          }

          if (minLambda > 1.0E-6) {
            bool nonDegenerateWset;
            i = (unsigned char)objective->nvar;
            if (i - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (unsigned int)
                     i * sizeof(double));
            }

            nonDegenerateWset = feasibleX0ForWorkingSet
              (memspace->workspace_double, solution->searchDir, workingset,
               qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }

            activeSetChangeID = 0;
            minLambda = c_maxConstraintViolation(workingset, solution->searchDir);
            if (minLambda < solution->maxConstr) {
              if (i - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (unsigned
                        int)i * sizeof(double));
              }

              solution->maxConstr = minLambda;
            }
          }
        }

        if (updateFval) {
          solution->fstar = computeFval_ReuseHx(objective,
            memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) && ((solution->state !=
                0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = computeFval_ReuseHx(objective,
          memspace->workspace_double, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/*
 * Arguments    : bool obj_hasLinear
 *                int obj_nvar
 *                double workspace[496]
 *                const double H[225]
 *                const double f[16]
 *                const double x[16]
 * Return Type  : void
 */
static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[496],
  const double H[225], const double f[16], const double x[16])
{
  int beta1;
  int ia;
  int iac;
  beta1 = 0;
  if (obj_hasLinear) {
    beta1 = (unsigned char)obj_nvar;
    if (beta1 - 1 >= 0) {
      memcpy(&workspace[0], &f[0], (unsigned int)beta1 * sizeof(double));
    }

    beta1 = 1;
  }

  if (obj_nvar != 0) {
    int ix;
    if (beta1 != 1) {
      beta1 = (unsigned char)obj_nvar;
      memset(&workspace[0], 0, (unsigned int)beta1 * sizeof(double));
    }

    ix = 0;
    beta1 = obj_nvar * (obj_nvar - 1) + 1;
    for (iac = 1; obj_nvar < 0 ? iac >= beta1 : iac <= beta1; iac += obj_nvar) {
      double c;
      int i;
      c = 0.5 * x[ix];
      i = (iac + obj_nvar) - 1;
      for (ia = iac; ia <= i; ia++) {
        int i1;
        i1 = ia - iac;
        workspace[i1] += H[ia - 1] * c;
      }

      ix++;
    }
  }
}

/*
 * Arguments    : const m_struct_T *obj
 *                const double x[496]
 *                int ix0
 * Return Type  : double
 */
static double maxConstraintViolation(const m_struct_T *obj, const double x[496],
  int ix0)
{
  double v;
  int i;
  int idx;
  int idxLB;
  v = 0.0;
  if (obj->sizes[3] > 0) {
    i = (unsigned char)obj->sizes[3];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[(ix0 + idxLB) - 1] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    i = (unsigned char)obj->sizes[4];
    for (idx = 0; idx < i; idx++) {
      idxLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[(ix0 + idxLB) - 1] - obj->ub[idxLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    i = (unsigned char)obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      v = fmax(v, fabs(x[(ix0 + obj->indexFixed[idx]) - 2] - obj->ub
                       [obj->indexFixed[idx] - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
static double maximum(const double x[2])
{
  double ex;
  if (x[0] < x[1]) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

/*
 * Arguments    : const double x[4]
 * Return Type  : double
 */
static double mean(const double x[4])
{
  return (((x[0] + x[1]) + x[2]) + x[3]) / 4.0;
}

/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
static double minimum(const double x[2])
{
  double ex;
  if (x[0] > x[1]) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

/*
 * Arguments    : double A[961]
 *                int m
 *                int n
 *                int nfxd
 *                double tau[31]
 * Return Type  : void
 */
static void qrf(double A[961], int m, int n, int nfxd, double tau[31])
{
  double work[31];
  double atmp;
  int b_i;
  int i;
  memset(&tau[0], 0, 31U * sizeof(double));
  memset(&work[0], 0, 31U * sizeof(double));
  i = (unsigned char)nfxd;
  for (b_i = 0; b_i < i; b_i++) {
    double d;
    int ii;
    int mmi;
    ii = b_i * 31 + b_i;
    mmi = m - b_i;
    if (b_i + 1 < m) {
      atmp = A[ii];
      d = xzlarfg(mmi, &atmp, A, ii + 2);
      tau[b_i] = d;
      A[ii] = atmp;
    } else {
      d = 0.0;
      tau[b_i] = 0.0;
    }

    if (b_i + 1 < n) {
      atmp = A[ii];
      A[ii] = 1.0;
      xzlarf(mmi, (n - b_i) - 1, ii + 1, d, A, ii + 32, work);
      A[ii] = atmp;
    }
  }
}

/*
 * Arguments    : m_struct_T *obj
 *                int idx_global
 * Return Type  : void
 */
static void removeConstr(m_struct_T *obj, int idx_global)
{
  int TYPE_tmp;
  int i;
  int i1;
  int idx;
  TYPE_tmp = obj->Wid[idx_global - 1] - 1;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx[idx_global -
                       1]) - 2] = false;
  i = obj->nActiveConstr - 1;
  obj->Wid[idx_global - 1] = obj->Wid[i];
  obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[i];
  i1 = (unsigned char)obj->nVar;
  for (idx = 0; idx < i1; idx++) {
    obj->ATwset[idx + ((idx_global - 1) << 4)] = obj->ATwset[idx + (i << 4)];
  }

  obj->bwset[idx_global - 1] = obj->bwset[i];
  obj->nActiveConstr = i;
  obj->nWConstr[TYPE_tmp]--;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd(double u0, double u1)
{
  double a;
  double b;
  double y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Arguments    : m_struct_T *obj
 *                int PROBLEM_TYPE
 * Return Type  : void
 */
static void setProblemType(m_struct_T *obj, int PROBLEM_TYPE)
{
  int i;
  int idx;
  int idx_col;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 15;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = (unsigned char)obj->sizesNormal[4];
      for (idx = 0; idx < i; idx++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + idx) - 1] =
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
      }
    }

    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }

    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;

   case 1:
    {
      int idxStartIneq;
      obj->nVar = 16;
      obj->mConstr = obj->mConstrOrig + 1;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[(idx << 4) + 15] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 16;
      obj->lb[15] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[((idx - 1) << 4) + 15] = -1.0;
      }

      if (obj->nWConstr[4] > 0) {
        i = (unsigned char)(obj->sizesNormal[4] + 1);
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
        }
      }

      obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
    }
    break;

   case 2:
    {
      obj->nVar = 15;
      obj->mConstr = 30;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegularized[i];
      }

      if (obj->probType != 4) {
        int i1;
        int idxStartIneq;
        int idx_lb;
        idx_lb = 15;
        i = obj->sizesNormal[3] + 1;
        i1 = obj->sizesRegularized[3];
        for (idx = i; idx <= i1; idx++) {
          idx_lb++;
          obj->indexLB[idx - 1] = idx_lb;
        }

        if (obj->nWConstr[4] > 0) {
          i = (unsigned char)obj->sizesRegularized[4];
          for (idx = 0; idx < i; idx++) {
            obj->isActiveConstr[obj->isActiveIdxRegularized[4] + idx] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
          }
        }

        i = obj->isActiveIdx[4];
        i1 = obj->isActiveIdxRegularized[4] - 1;
        if (i <= i1) {
          memset(&obj->isActiveConstr[i + -1], 0, (unsigned int)((i1 - i) + 1) *
                 sizeof(bool));
        }

        idxStartIneq = obj->isActiveIdx[2];
        i = obj->nActiveConstr;
        for (idx_col = idxStartIneq; idx_col <= i; idx_col++) {
          idx_lb = ((idx_col - 1) << 4) - 1;
          if (obj->Wid[idx_col - 1] == 3) {
            i1 = obj->Wlocalidx[idx_col - 1];
            idx = i1 + 14;
            if (idx >= 16) {
              memset(&obj->ATwset[idx_lb + 16], 0, (unsigned int)(((idx + idx_lb)
                       - idx_lb) - 15) * sizeof(double));
            }

            obj->ATwset[(i1 + idx_lb) + 15] = -1.0;
            i1 += 16;
            if (i1 <= 15) {
              memset(&obj->ATwset[i1 + idx_lb], 0, (unsigned int)(((idx_lb - i1)
                       - idx_lb) + 16) * sizeof(double));
            }
          }
        }
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
      }
    }
    break;

   default:
    {
      int idxStartIneq;
      obj->nVar = 16;
      obj->mConstr = 31;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
      }

      i = (unsigned char)obj->sizes[0];
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[(idx << 4) + 15] = 0.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = 16;
      obj->lb[15] = 1.0E-5;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx = idxStartIneq; idx <= i; idx++) {
        obj->ATwset[((idx - 1) << 4) + 15] = -1.0;
      }

      if (obj->nWConstr[4] > 0) {
        i = (unsigned char)(obj->sizesNormal[4] + 1);
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
        }
      }

      obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

/*
 * Arguments    : const g_struct_T *obj
 *                double rhs[16]
 * Return Type  : void
 */
static void solve(const g_struct_T *obj, double rhs[16])
{
  int i;
  int j;
  int jA;
  int n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    for (j = 0; j < n_tmp; j++) {
      double temp;
      jA = j * 31;
      temp = rhs[j];
      for (i = 0; i < j; i++) {
        temp -= obj->FMat[jA + i] * rhs[i];
      }

      rhs[j] = temp / obj->FMat[jA + j];
    }
  }

  if (obj->ndims != 0) {
    for (j = n_tmp; j >= 1; j--) {
      jA = (j + (j - 1) * 31) - 1;
      rhs[j - 1] /= obj->FMat[jA];
      for (i = 0; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        rhs[ix] -= rhs[j - 1] * obj->FMat[(jA - i) - 1];
      }
    }
  }
}

/*
 * Arguments    : double lambda[31]
 *                int WorkingSet_nActiveConstr
 *                const int WorkingSet_sizes[5]
 *                const int WorkingSet_isActiveIdx[6]
 *                const int WorkingSet_Wid[31]
 *                const int WorkingSet_Wlocalidx[31]
 *                double workspace[496]
 * Return Type  : void
 */
static void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[31], const int WorkingSet_Wlocalidx[31], double workspace[496])
{
  if (WorkingSet_nActiveConstr != 0) {
    int idx;
    int idxOffset;
    int mAll;
    mAll = (WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4];
    idx = (unsigned char)mAll;
    if (idx - 1 >= 0) {
      memcpy(&workspace[0], &lambda[0], (unsigned int)idx * sizeof(double));
    }

    if (mAll - 1 >= 0) {
      memset(&lambda[0], 0, (unsigned int)mAll * sizeof(double));
    }

    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) && (WorkingSet_Wid[idx] <= 2))
    {
      if (WorkingSet_Wid[idx] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }

      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }

    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid[idx]) {
       case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }

      lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] = workspace[mAll];
      mAll++;
      idx++;
    }
  }
}

/*
 * Arguments    : int *STEP_TYPE
 *                double Hessian[225]
 *                const double lb[15]
 *                const double ub[15]
 *                l_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                h_struct_T *memspace
 *                m_struct_T *WorkingSet
 *                f_struct_T *QRManager
 *                g_struct_T *CholManager
 *                struct_T *QPObjective
 *                p_struct_T *qpoptions
 * Return Type  : bool
 */
static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], l_struct_T *TrialState, b_struct_T
                 *MeritFunction, h_struct_T *memspace, m_struct_T *WorkingSet,
                 f_struct_T *QRManager, g_struct_T *CholManager, struct_T
                 *QPObjective, p_struct_T *qpoptions)
{
  p_struct_T b_qpoptions;
  double dv[16];
  double oldDirIdx;
  double s;
  int idxEndIneq_tmp_tmp;
  int idxStartIneq;
  int k;
  int nVar_tmp_tmp;
  bool checkBoundViolation;
  bool stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (unsigned int)
           idxEndIneq_tmp_tmp * sizeof(double));
  } else {
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->nVar;
    if (idxEndIneq_tmp_tmp - 1 >= 0) {
      memcpy(&TrialState->searchDir[0], &TrialState->xstar[0], (unsigned int)
             idxEndIneq_tmp_tmp * sizeof(double));
    }
  }

  int exitg1;
  bool guard1;
  do {
    int temp;
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
     case 1:
      b_qpoptions = *qpoptions;
      driver(Hessian, TrialState->grad, TrialState, memspace, WorkingSet,
             QRManager, CholManager, QPObjective, &b_qpoptions,
             qpoptions->MaxIterations);
      if (TrialState->state > 0) {
        MeritFunction->phi = TrialState->sqpFval;
        MeritFunction->linearizedConstrViol = 0.0;
        MeritFunction->penaltyParam = 1.0;
        MeritFunction->phiPrimePlus = fmin(TrialState->fstar, 0.0);
      }

      sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                   WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                   WorkingSet->Wlocalidx, memspace->workspace_double);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (unsigned int)
                 idxEndIneq_tmp_tmp * sizeof(double));
        }

        guard1 = true;
      }
      break;

     case 2:
      {
        double beta;
        temp = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
        idxStartIneq = temp + 1;
        idxEndIneq_tmp_tmp = WorkingSet->nActiveConstr;
        for (k = idxStartIneq; k <= idxEndIneq_tmp_tmp; k++) {
          WorkingSet->isActiveConstr[(WorkingSet->isActiveIdx[WorkingSet->Wid[k
            - 1] - 1] + WorkingSet->Wlocalidx[k - 1]) - 2] = false;
        }

        WorkingSet->nWConstr[2] = 0;
        WorkingSet->nWConstr[3] = 0;
        WorkingSet->nWConstr[4] = 0;
        WorkingSet->nActiveConstr = temp;
        memcpy(&dv[0], &TrialState->xstar[0], 16U * sizeof(double));
        idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          s = WorkingSet->lb[WorkingSet->indexLB[k] - 1];
          if (-dv[WorkingSet->indexLB[k] - 1] > s) {
            dv[WorkingSet->indexLB[k] - 1] = (WorkingSet->ub[WorkingSet->
              indexLB[k] - 1] - s) / 2.0;
          }
        }

        idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          s = WorkingSet->ub[WorkingSet->indexUB[k] - 1];
          if (dv[WorkingSet->indexUB[k] - 1] > s) {
            dv[WorkingSet->indexUB[k] - 1] = (s - WorkingSet->lb
              [WorkingSet->indexUB[k] - 1]) / 2.0;
          }
        }

        memcpy(&TrialState->xstar[0], &dv[0], 16U * sizeof(double));
        idxEndIneq_tmp_tmp = WorkingSet->nVar;
        beta = 0.0;
        idxStartIneq = (unsigned char)WorkingSet->nVar;
        for (k = 0; k < idxStartIneq; k++) {
          beta += Hessian[k + 15 * k];
        }

        if (TrialState->sqpIterations <= 1) {
          temp = QPObjective->nvar;
          if (QPObjective->nvar < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (QPObjective->nvar > 1) {
              oldDirIdx = fabs(TrialState->grad[0]);
              for (k = 2; k <= temp; k++) {
                s = fabs(TrialState->grad[k - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = k;
                  oldDirIdx = s;
                }
              }
            }
          }

          s = 100.0 * fmax(1.0, fabs(TrialState->grad[idxStartIneq - 1]));
        } else {
          temp = WorkingSet->mConstr;
          if (WorkingSet->mConstr < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (WorkingSet->mConstr > 1) {
              oldDirIdx = fabs(TrialState->lambdasqp[0]);
              for (k = 2; k <= temp; k++) {
                s = fabs(TrialState->lambdasqp[k - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = k;
                  oldDirIdx = s;
                }
              }
            }
          }

          s = fabs(TrialState->lambdasqp[idxStartIneq - 1]);
        }

        QPObjective->nvar = WorkingSet->nVar;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 4;
        QPObjective->beta = beta / (double)WorkingSet->nVar;
        QPObjective->rho = s;
        setProblemType(WorkingSet, 2);
        temp = qpoptions->MaxIterations;
        qpoptions->MaxIterations = (qpoptions->MaxIterations + WorkingSet->nVar)
          - idxEndIneq_tmp_tmp;
        memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
        b_qpoptions = *qpoptions;
        driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
               CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
        qpoptions->MaxIterations = temp;
        if (TrialState->state != -6) {
          MeritFunction->phi = TrialState->sqpFval;
          MeritFunction->linearizedConstrViol = 0.0;
          MeritFunction->penaltyParam = 1.0;
          MeritFunction->phiPrimePlus = fmin(TrialState->fstar, 0.0);
          temp = WorkingSet->isActiveIdx[2];
          idxStartIneq = WorkingSet->nActiveConstr;
          for (k = temp; k <= idxStartIneq; k++) {
            if (WorkingSet->Wid[k - 1] == 3) {
              TrialState->lambda[k - 1] *= (double)memspace->
                workspace_int[WorkingSet->Wlocalidx[k - 1] - 1];
            }
          }
        }

        QPObjective->nvar = idxEndIneq_tmp_tmp;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 3;
        setProblemType(WorkingSet, 3);
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                     WorkingSet->Wlocalidx, memspace->workspace_double);
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (unsigned int)
                 idxEndIneq_tmp_tmp * sizeof(double));
        }

        guard1 = true;
      }
      break;

     default:
      idxEndIneq_tmp_tmp = WorkingSet->nVar;
      idxStartIneq = (unsigned char)WorkingSet->nVar;
      for (k = 0; k < idxStartIneq; k++) {
        TrialState->xstarsqp[k] = TrialState->xstarsqp_old[k];
        TrialState->socDirection[k] = TrialState->xstar[k];
      }

      memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0], 31U *
             sizeof(double));
      memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (unsigned int)
             idxStartIneq * sizeof(double));
      memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
      b_qpoptions = *qpoptions;
      driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
             CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
      idxStartIneq = (unsigned char)idxEndIneq_tmp_tmp;
      for (k = 0; k < idxStartIneq; k++) {
        s = TrialState->socDirection[k];
        oldDirIdx = s;
        s = TrialState->xstar[k] - s;
        TrialState->socDirection[k] = s;
        TrialState->xstar[k] = oldDirIdx;
      }

      stepSuccess = (b_xnrm2(idxEndIneq_tmp_tmp, TrialState->socDirection) <=
                     2.0 * b_xnrm2(idxEndIneq_tmp_tmp, TrialState->xstar));
      if (!stepSuccess) {
        memcpy(&TrialState->lambda[0], &TrialState->lambdaStopTest[0], 31U *
               sizeof(double));
      } else {
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                     WorkingSet->Wlocalidx, memspace->workspace_double);
      }

      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        idxEndIneq_tmp_tmp = (unsigned char)nVar_tmp_tmp;
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          TrialState->delta_x[k] = TrialState->xstar[k] +
            TrialState->socDirection[k];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        s = 0.0;
        oldDirIdx = 1.0;
        for (k = 0; k < 15; k++) {
          s = fmax(s, fabs(TrialState->grad[k]));
          oldDirIdx = fmax(oldDirIdx, fabs(TrialState->xstar[k]));
        }

        s = fmax(2.2204460492503131E-16, s / oldDirIdx);
        for (idxStartIneq = 0; idxStartIneq < 15; idxStartIneq++) {
          temp = 15 * idxStartIneq;
          for (k = 0; k < idxStartIneq; k++) {
            Hessian[temp + k] = 0.0;
          }

          temp = idxStartIneq + 15 * idxStartIneq;
          Hessian[temp] = s;
          idxEndIneq_tmp_tmp = 13 - idxStartIneq;
          if (idxEndIneq_tmp_tmp >= 0) {
            memset(&Hessian[temp + 1], 0, (unsigned int)(((idxEndIneq_tmp_tmp +
                      temp) - temp) + 1) * sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    memcpy(&dv[0], &TrialState->delta_x[0], 16U * sizeof(double));
    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[3];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexLB[k] - 1];
      s = (TrialState->xstarsqp[WorkingSet->indexLB[k] - 1] + oldDirIdx) -
        lb[WorkingSet->indexLB[k] - 1];
      if (s < 0.0) {
        dv[WorkingSet->indexLB[k] - 1] = oldDirIdx - s;
        TrialState->xstar[WorkingSet->indexLB[k] - 1] -= s;
      }
    }

    idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
    for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet->indexUB[k] - 1];
      s = (ub[WorkingSet->indexUB[k] - 1] - TrialState->xstarsqp
           [WorkingSet->indexUB[k] - 1]) - oldDirIdx;
      if (s < 0.0) {
        dv[WorkingSet->indexUB[k] - 1] = oldDirIdx + s;
        TrialState->xstar[WorkingSet->indexUB[k] - 1] += s;
      }
    }

    memcpy(&TrialState->delta_x[0], &dv[0], 16U * sizeof(double));
  }

  return stepSuccess;
}

/*
 * Arguments    : b_struct_T *MeritFunction
 *                const m_struct_T *WorkingSet
 *                l_struct_T *TrialState
 *                const double lb[15]
 *                const double ub[15]
 *                bool *Flags_fevalOK
 *                bool *Flags_done
 *                bool *Flags_stepAccepted
 *                bool *Flags_failedLineSearch
 *                int *Flags_stepType
 * Return Type  : bool
 */
static bool test_exit(b_struct_T *MeritFunction, const m_struct_T *WorkingSet,
                      l_struct_T *TrialState, const double lb[15], const double
                      ub[15], bool *Flags_fevalOK, bool *Flags_done, bool
                      *Flags_stepAccepted, bool *Flags_failedLineSearch, int
                      *Flags_stepType)
{
  double optimRelativeFactor;
  double s;
  double smax;
  int i;
  int idx_max;
  int k;
  int nVar;
  bool Flags_gradOK;
  bool exitg1;
  bool isFeasible;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  i = (unsigned char)((WorkingSet->sizes[0] + WorkingSet->sizes[3]) +
                      WorkingSet->sizes[4]);
  if (i - 1 >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (unsigned
            int)i * sizeof(double));
  }

  computeGradLag(TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = fabs(TrialState->grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = fabs(TrialState->grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }

  optimRelativeFactor = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (optimRelativeFactor >= 1.7976931348623157E+308) {
    optimRelativeFactor = 1.0;
  }

  smax = 0.0;
  idx_max = (unsigned char)WorkingSet->sizes[3];
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet->indexLB[k] - 1;
    smax = fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }

  idx_max = (unsigned char)WorkingSet->sizes[4];
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet->indexUB[k] - 1;
    smax = fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  Flags_gradOK = true;
  smax = 0.0;
  idx_max = (unsigned char)WorkingSet->nVar;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= idx_max - 1)) {
    s = fabs(TrialState->gradLag[k]);
    Flags_gradOK = (s < 1.7976931348623157E+308);
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = fmax(smax, s);
      k++;
    }
  }

  MeritFunction->nlpDualFeasError = smax;
  if (!Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = smax;
    if (i - 1 >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (unsigned int)i * sizeof(double));
    }

    if (isFeasible && (smax <= 1.0E-9 * optimRelativeFactor)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }

  return Flags_gradOK;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void tic(void)
{
  coderTimespec b_timespec;
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  timeKeeper(b_timespec.tv_sec, b_timespec.tv_nsec);
}

/*
 * Arguments    : double newTime_tv_sec
 *                double newTime_tv_nsec
 * Return Type  : void
 */
static void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec)
{
  if (!savedTime_not_empty) {
    coderTimespec b_timespec;
    if (!freq_not_empty) {
      freq_not_empty = true;
      coderInitTimeFunctions(&freq);
    }

    coderTimeClockGettimeMonotonic(&b_timespec, freq);
    savedTime_not_empty = true;
  }

  savedTime.tv_sec = newTime_tv_sec;
  savedTime.tv_nsec = newTime_tv_nsec;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void timeKeeper_init(void)
{
  savedTime_not_empty = false;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double toc(void)
{
  coderTimespec b_timespec;
  double tstart_tv_nsec;
  double tstart_tv_sec;
  tstart_tv_sec = b_timeKeeper(&tstart_tv_nsec);
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  return (b_timespec.tv_sec - tstart_tv_sec) + (b_timespec.tv_nsec -
    tstart_tv_nsec) / 1.0E+9;
}

/*
 * Arguments    : int m
 *                int n
 *                int k
 *                const double A[225]
 *                int lda
 *                const double B[961]
 *                int ib0
 *                double C[496]
 * Return Type  : void
 */
static void xgemm(int m, int n, int k, const double A[225], int lda, const
                  double B[961], int ib0, double C[496])
{
  int cr;
  int ib;
  int ic;
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    br = ib0;
    lastColC = 31 * (n - 1);
    for (cr = 0; cr <= lastColC; cr += 31) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        memset(&C[i + -1], 0, (unsigned int)((i1 - i) + 1) * sizeof(double));
      }
    }

    for (cr = 0; cr <= lastColC; cr += 31) {
      int ar;
      ar = -1;
      i = br + k;
      for (ib = br; ib < i; ib++) {
        int i2;
        i1 = cr + 1;
        i2 = cr + m;
        for (ic = i1; ic <= i2; ic++) {
          C[ic - 1] += B[ib - 1] * A[(ar + ic) - cr];
        }

        ar += lda;
      }

      br += 31;
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[225]
 *                int lda
 *                const double x[16]
 *                double y[15]
 * Return Type  : void
 */
static void xgemv(int m, int n, const double A[225], int lda, const double x[16],
                  double y[15])
{
  int ia;
  int iac;
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = (unsigned char)m;
    memset(&y[0], 0, (unsigned int)i * sizeof(double));
    ix = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y[i2] += A[ia - 1] * x[ix];
      }

      ix++;
    }
  }
}

/*
 * Arguments    : double A[961]
 *                int m
 *                int n
 *                int jpvt[31]
 *                double tau[31]
 * Return Type  : void
 */
static void xgeqp3(double A[961], int m, int n, int jpvt[31], double tau[31])
{
  double vn1[31];
  double vn2[31];
  double work[31];
  double temp;
  int b_i;
  int k;
  int minmn_tmp;
  int pvt;
  if (m <= n) {
    minmn_tmp = m;
  } else {
    minmn_tmp = n;
  }

  memset(&tau[0], 0, 31U * sizeof(double));
  if (minmn_tmp < 1) {
    for (pvt = 0; pvt < n; pvt++) {
      jpvt[pvt] = pvt + 1;
    }
  } else {
    int i;
    int ix;
    int iy;
    int nfxd;
    int temp_tmp;
    nfxd = 0;
    for (pvt = 0; pvt < n; pvt++) {
      if (jpvt[pvt] != 0) {
        nfxd++;
        if (pvt + 1 != nfxd) {
          ix = pvt * 31;
          iy = (nfxd - 1) * 31;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            i = iy + k;
            A[temp_tmp] = A[i];
            A[i] = temp;
          }

          jpvt[pvt] = jpvt[nfxd - 1];
          jpvt[nfxd - 1] = pvt + 1;
        } else {
          jpvt[pvt] = pvt + 1;
        }
      } else {
        jpvt[pvt] = pvt + 1;
      }
    }

    if (nfxd > minmn_tmp) {
      nfxd = minmn_tmp;
    }

    qrf(A, m, n, nfxd, tau);
    if (nfxd < minmn_tmp) {
      double d;
      memset(&work[0], 0, 31U * sizeof(double));
      memset(&vn1[0], 0, 31U * sizeof(double));
      memset(&vn2[0], 0, 31U * sizeof(double));
      i = nfxd + 1;
      for (pvt = i; pvt <= n; pvt++) {
        d = xnrm2(m - nfxd, A, (nfxd + (pvt - 1) * 31) + 1);
        vn1[pvt - 1] = d;
        vn2[pvt - 1] = d;
      }

      for (b_i = i; b_i <= minmn_tmp; b_i++) {
        double s;
        int ii;
        int ip1;
        int mmi;
        int nmi;
        ip1 = b_i + 1;
        iy = (b_i - 1) * 31;
        ii = (iy + b_i) - 1;
        nmi = (n - b_i) + 1;
        mmi = m - b_i;
        if (nmi < 1) {
          nfxd = -2;
        } else {
          nfxd = -1;
          if (nmi > 1) {
            temp = fabs(vn1[b_i - 1]);
            for (k = 2; k <= nmi; k++) {
              s = fabs(vn1[(b_i + k) - 2]);
              if (s > temp) {
                nfxd = k - 2;
                temp = s;
              }
            }
          }
        }

        pvt = b_i + nfxd;
        if (pvt + 1 != b_i) {
          ix = pvt * 31;
          for (k = 0; k < m; k++) {
            temp_tmp = ix + k;
            temp = A[temp_tmp];
            nfxd = iy + k;
            A[temp_tmp] = A[nfxd];
            A[nfxd] = temp;
          }

          nfxd = jpvt[pvt];
          jpvt[pvt] = jpvt[b_i - 1];
          jpvt[b_i - 1] = nfxd;
          vn1[pvt] = vn1[b_i - 1];
          vn2[pvt] = vn2[b_i - 1];
        }

        if (b_i < m) {
          temp = A[ii];
          d = xzlarfg(mmi + 1, &temp, A, ii + 2);
          tau[b_i - 1] = d;
          A[ii] = temp;
        } else {
          d = 0.0;
          tau[b_i - 1] = 0.0;
        }

        if (b_i < n) {
          temp = A[ii];
          A[ii] = 1.0;
          xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, ii + 32, work);
          A[ii] = temp;
        }

        for (pvt = ip1; pvt <= n; pvt++) {
          nfxd = b_i + (pvt - 1) * 31;
          d = vn1[pvt - 1];
          if (d != 0.0) {
            temp = fabs(A[nfxd - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }

            s = d / vn2[pvt - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < m) {
                d = xnrm2(mmi, A, nfxd + 1);
                vn1[pvt - 1] = d;
                vn2[pvt - 1] = d;
              } else {
                vn1[pvt - 1] = 0.0;
                vn2[pvt - 1] = 0.0;
              }
            } else {
              vn1[pvt - 1] = d * sqrt(temp);
            }
          }
        }
      }
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[961]
 *                int ix0
 * Return Type  : double
 */
static double xnrm2(int n, const double x[961], int ix0)
{
  double y;
  int k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        double absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/*
 * Arguments    : int n
 *                double A[961]
 * Return Type  : int
 */
static int xpotrf(int n, double A[961])
{
  int ia;
  int iac;
  int info;
  int j;
  int nmj;
  bool exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= n - 1)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    idxA1j = j * 31;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (nmj = 0; nmj < j; nmj++) {
        c = A[idxA1j + nmj];
        ssq += c * c;
      }
    }

    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < n) {
        int i;
        int ia0;
        int idxAjjp1;
        nmj = (n - j) - 2;
        ia0 = idxA1j + 32;
        idxAjjp1 = idxAjj + 32;
        if ((j != 0) && (nmj + 1 != 0)) {
          i = (idxA1j + 31 * nmj) + 32;
          for (iac = ia0; iac <= i; iac += 31) {
            int i1;
            c = 0.0;
            i1 = (iac + j) - 1;
            for (ia = iac; ia <= i1; ia++) {
              c += A[ia - 1] * A[(idxA1j + ia) - iac];
            }

            i1 = (idxAjj + div_nde_s32_floor((iac - idxA1j) - 32, 31) * 31) + 31;
            A[i1] -= c;
          }
        }

        ssq = 1.0 / ssq;
        i = (idxAjj + 31 * nmj) + 32;
        for (nmj = idxAjjp1; nmj <= i; nmj += 31) {
          A[nmj - 1] *= ssq;
        }
      }

      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/*
 * Arguments    : double *a
 *                double *b
 *                double *s
 * Return Type  : double
 */
static double xrotg(double *a, double *b, double *s)
{
  double absa;
  double absb;
  double c;
  double roe;
  double scale;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    double ads;
    double bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (c != 0.0) {
      *b = 1.0 / c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }

  return c;
}

/*
 * Arguments    : int m
 *                int n
 *                int iv0
 *                double tau
 *                double C[961]
 *                int ic0
 *                double work[31]
 * Return Type  : void
 */
static void xzlarf(int m, int n, int iv0, double tau, double C[961], int ic0,
                   double work[31])
{
  int i;
  int ia;
  int iac;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    bool exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      int exitg1;
      i = ic0 + lastc * 31;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    double c;
    int b_i;
    if (lastc + 1 != 0) {
      if (lastc >= 0) {
        memset(&work[0], 0, (unsigned int)(lastc + 1) * sizeof(double));
      }

      b_i = ic0 + 31 * lastc;
      for (iac = ic0; iac <= b_i; iac += 31) {
        c = 0.0;
        i = (iac + lastv) - 1;
        for (ia = iac; ia <= i; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }

        i = div_nde_s32_floor(iac - ic0, 31);
        work[i] += c;
      }
    }

    if (-tau != 0.0) {
      i = ic0;
      for (iac = 0; iac <= lastc; iac++) {
        c = work[iac];
        if (c != 0.0) {
          c *= -tau;
          b_i = lastv + i;
          for (ia = i; ia < b_i; ia++) {
            C[ia - 1] += C[((iv0 + ia) - i) - 1] * c;
          }
        }

        i += 31;
      }
    }
  }
}

/*
 * Arguments    : int n
 *                double *alpha1
 *                double x[961]
 *                int ix0
 * Return Type  : double
 */
static double xzlarfg(int n, double *alpha1, double x[961], int ix0)
{
  double tau;
  int k;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      double beta1;
      beta1 = rt_hypotd(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        beta1 = -beta1;
      }

      if (fabs(beta1) < 1.0020841800044864E-292) {
        int i;
        int knt;
        knt = 0;
        i = (ix0 + n) - 2;
        do {
          knt++;
          for (k = ix0; k <= i; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

        beta1 = rt_hypotd(*alpha1, xnrm2(n - 1, x, ix0));
        if (*alpha1 >= 0.0) {
          beta1 = -beta1;
        }

        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }

        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }

        *alpha1 = beta1;
      } else {
        int i;
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        i = (ix0 + n) - 2;
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }

        *alpha1 = beta1;
      }
    }
  }

  return tau;
}

/*
 * Create variables necessary for the optimization
 *
 * Arguments    : double K_p_T
 *                double K_p_M
 *                double m
 *                double I_xx
 *                double I_yy
 *                double I_zz
 *                double l_1
 *                double l_2
 *                double l_3
 *                double l_4
 *                double l_z
 *                double Phi
 *                double Theta
 *                double Omega_1
 *                double Omega_2
 *                double Omega_3
 *                double Omega_4
 *                double b_1
 *                double b_2
 *                double b_3
 *                double b_4
 *                double g_1
 *                double g_2
 *                double g_3
 *                double g_4
 *                double delta_ailerons
 *                double W_act_motor_const
 *                double W_act_motor_speed
 *                double W_act_tilt_el_const
 *                double W_act_tilt_el_speed
 *                double W_act_tilt_az_const
 *                double W_act_tilt_az_speed
 *                double W_act_theta_const
 *                double W_act_theta_speed
 *                double W_act_phi_const
 *                double W_act_phi_speed
 *                double W_act_ailerons_const
 *                double W_act_ailerons_speed
 *                double W_dv_1
 *                double W_dv_2
 *                double W_dv_3
 *                double W_dv_4
 *                double W_dv_5
 *                double W_dv_6
 *                double max_omega
 *                double min_omega
 *                double max_b
 *                double min_b
 *                double max_g
 *                double min_g
 *                double max_theta
 *                double min_theta
 *                double max_phi
 *                double max_delta_ailerons
 *                double min_delta_ailerons
 *                double dv[6]
 *                double p
 *                double q
 *                double r
 *                double Cm_zero
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double CL_aileron
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double max_alpha
 *                double min_alpha
 *                double Beta
 *                double gamma_quadratic_du
 *                double desired_motor_value
 *                double desired_el_value
 *                double desired_az_value
 *                double desired_theta_value
 *                double desired_phi_value
 *                double desired_ailerons_value
 *                double k_alt_tilt_constraint
 *                double min_alt_tilt_constraint
 *                double lidar_alt_corrected
 *                double approach_mode
 *                double verbose
 *                double aoa_protection_speed
 *                double transition_speed
 *                double p_body_current
 *                double q_body_current
 *                double r_body_current
 *                double p_dot_current
 *                double q_dot_current
 *                double r_dot_current
 *                double phi_current
 *                double theta_current
 *                double theta_gain
 *                double phi_gain
 *                double p_body_gain
 *                double q_body_gain
 *                double r_body_gain
 *                double des_psi_dot
 *                double u_out[15]
 *                double residuals[6]
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluations
 *                double *exitflag
 * Return Type  : void
 */
void Cascaded_nonlinear_controller_control_rf_w_ailerons(double K_p_T, double
  K_p_M, double m, double I_xx, double I_yy, double I_zz, double l_1, double l_2,
  double l_3, double l_4, double l_z, double Phi, double Theta, double Omega_1,
  double Omega_2, double Omega_3, double Omega_4, double b_1, double b_2, double
  b_3, double b_4, double g_1, double g_2, double g_3, double g_4, double
  delta_ailerons, double W_act_motor_const, double W_act_motor_speed, double
  W_act_tilt_el_const, double W_act_tilt_el_speed, double W_act_tilt_az_const,
  double W_act_tilt_az_speed, double W_act_theta_const, double W_act_theta_speed,
  double W_act_phi_const, double W_act_phi_speed, double W_act_ailerons_const,
  double W_act_ailerons_speed, double W_dv_1, double W_dv_2, double W_dv_3,
  double W_dv_4, double W_dv_5, double W_dv_6, double max_omega, double
  min_omega, double max_b, double min_b, double max_g, double min_g, double
  max_theta, double min_theta, double max_phi, double max_delta_ailerons, double
  min_delta_ailerons, double dv[6], double p, double q, double r, double Cm_zero,
  double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double
  CL_aileron, double rho, double V, double S, double wing_chord, double
  flight_path_angle, double max_alpha, double min_alpha, double Beta, double
  gamma_quadratic_du, double desired_motor_value, double desired_el_value,
  double desired_az_value, double desired_theta_value, double desired_phi_value,
  double desired_ailerons_value, double k_alt_tilt_constraint, double
  min_alt_tilt_constraint, double lidar_alt_corrected, double approach_mode,
  double verbose, double aoa_protection_speed, double transition_speed, double
  p_body_current, double q_body_current, double r_body_current, double
  p_dot_current, double q_dot_current, double r_dot_current, double phi_current,
  double theta_current, double theta_gain, double phi_gain, double p_body_gain,
  double q_body_gain, double r_body_gain, double des_psi_dot, double u_out[15],
  double residuals[6], double *elapsed_time, double *N_iterations, double
  *N_evaluations, double *exitflag)
{
  b_captured_var dv_global;
  captured_var W_act_ailerons;
  captured_var W_act_motor;
  captured_var W_act_phi;
  captured_var W_act_theta;
  captured_var W_act_tilt_az;
  captured_var W_act_tilt_el;
  captured_var b_Beta;
  captured_var b_CL_aileron;
  captured_var b_Cd_zero;
  captured_var b_Cl_alpha;
  captured_var b_Cm_alpha;
  captured_var b_Cm_zero;
  captured_var b_I_xx;
  captured_var b_I_yy;
  captured_var b_I_zz;
  captured_var b_K_Cd;
  captured_var b_K_p_M;
  captured_var b_K_p_T;
  captured_var b_Phi;
  captured_var b_S;
  captured_var b_Theta;
  captured_var b_V;
  captured_var b_W_dv_1;
  captured_var b_W_dv_2;
  captured_var b_W_dv_3;
  captured_var b_W_dv_4;
  captured_var b_W_dv_5;
  captured_var b_W_dv_6;
  captured_var b_desired_ailerons_value;
  captured_var b_desired_az_value;
  captured_var b_desired_el_value;
  captured_var b_desired_motor_value;
  captured_var b_desired_phi_value;
  captured_var b_desired_theta_value;
  captured_var b_flight_path_angle;
  captured_var b_gamma_quadratic_du;
  captured_var b_l_1;
  captured_var b_l_2;
  captured_var b_l_3;
  captured_var b_l_4;
  captured_var b_l_z;
  captured_var b_m;
  captured_var b_p;
  captured_var b_q;
  captured_var b_r;
  captured_var b_rho;
  captured_var b_wing_chord;
  captured_var gain_ailerons;
  captured_var gain_az;
  captured_var gain_el;
  captured_var gain_motor;
  captured_var gain_phi;
  captured_var gain_theta;
  d_struct_T b_expl_temp;
  d_struct_T expl_temp;
  e_struct_T d_expl_temp;
  e_struct_T e_expl_temp;
  double actual_u[15];
  double u_max[15];
  double u_max_scaled[15];
  double u_min[15];
  double u_min_scaled[15];
  double b_u_max_scaled[13];
  double b_u_min_scaled[13];
  double u_out_second[13];
  double b_dv[9];
  double c_accelerations_attitude_alloca[6];
  double current_accelerations[6];
  double des_body_rates[3];
  double b_max_approach;
  double b_min_approach;
  double d;
  double des_phi_first_iteration;
  double des_theta_first_iteration;
  double elapsed_time_first;
  double g_max_approach;
  double g_min_approach;
  double max_theta_protection;
  double max_tilt_value_approach;
  double min_theta_protection;
  double output_funcCount;
  double output_iterations;
  int i;
  char c_expl_temp[3];
  if (!isInitialized_Cascaded_nonlinear_controller_control_rf_w_ailerons) {
    Cascaded_nonlinear_controller_control_rf_w_ailerons_initialize();
  }

  b_K_p_T.contents = K_p_T;
  b_K_p_M.contents = K_p_M;
  b_m.contents = m;
  b_I_xx.contents = I_xx;
  b_I_yy.contents = I_yy;
  b_I_zz.contents = I_zz;
  b_l_1.contents = l_1;
  b_l_2.contents = l_2;
  b_l_3.contents = l_3;
  b_l_4.contents = l_4;
  b_l_z.contents = l_z;
  b_Phi.contents = Phi;
  b_Theta.contents = Theta;
  b_W_dv_1.contents = W_dv_1;
  b_W_dv_2.contents = W_dv_2;
  b_W_dv_3.contents = W_dv_3;
  b_W_dv_4.contents = W_dv_4;
  b_W_dv_5.contents = W_dv_5;
  b_W_dv_6.contents = W_dv_6;
  b_p.contents = p;
  b_q.contents = q;
  b_r.contents = r;
  b_Cm_zero.contents = Cm_zero;
  b_Cl_alpha.contents = Cl_alpha;
  b_Cd_zero.contents = Cd_zero;
  b_K_Cd.contents = K_Cd;
  b_Cm_alpha.contents = Cm_alpha;
  b_CL_aileron.contents = CL_aileron;
  b_rho.contents = rho;
  b_V.contents = V;
  b_S.contents = S;
  b_wing_chord.contents = wing_chord;
  b_flight_path_angle.contents = flight_path_angle;
  b_Beta.contents = Beta;
  b_gamma_quadratic_du.contents = gamma_quadratic_du;
  b_desired_motor_value.contents = desired_motor_value;
  b_desired_el_value.contents = desired_el_value;
  b_desired_az_value.contents = desired_az_value;
  b_desired_theta_value.contents = desired_theta_value;
  b_desired_phi_value.contents = desired_phi_value;
  b_desired_ailerons_value.contents = desired_ailerons_value;
  if (b_V.contents > aoa_protection_speed) {
    max_tilt_value_approach = (max_alpha + b_flight_path_angle.contents) * 180.0
      / 3.1415926535897931;
    max_theta_protection = fmin(max_theta, max_tilt_value_approach);
    max_tilt_value_approach = (min_alpha + b_flight_path_angle.contents) * 180.0
      / 3.1415926535897931;
    min_theta_protection = fmax(min_theta, max_tilt_value_approach);
  } else {
    max_theta_protection = max_theta;
    min_theta_protection = min_theta;
  }

  if (b_desired_motor_value.contents < min_omega) {
    double b_Omega_1[4];
    b_Omega_1[0] = Omega_1;
    b_Omega_1[1] = Omega_2;
    b_Omega_1[2] = Omega_3;
    b_Omega_1[3] = Omega_4;
    b_desired_motor_value.contents = mean(b_Omega_1);
  }

  gain_motor.contents = max_omega / 2.0;
  gain_el.contents = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az.contents = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
  gain_theta.contents = (max_theta_protection - min_theta_protection) *
    3.1415926535897931 / 180.0 / 2.0;
  gain_phi.contents = max_phi * 3.1415926535897931 / 180.0;
  gain_ailerons.contents = (max_delta_ailerons - min_delta_ailerons) *
    3.1415926535897931 / 180.0 / 2.0;
  actual_u[0] = Omega_1;
  actual_u[1] = Omega_2;
  actual_u[2] = Omega_3;
  actual_u[3] = Omega_4;
  actual_u[4] = b_1;
  actual_u[5] = b_2;
  actual_u[6] = b_3;
  actual_u[7] = b_4;
  actual_u[8] = g_1;
  actual_u[9] = g_2;
  actual_u[10] = g_3;
  actual_u[11] = g_4;
  actual_u[12] = b_Theta.contents;
  actual_u[13] = b_Phi.contents;
  actual_u[14] = delta_ailerons;

  /* Build the max and minimum actuator array: */
  u_max[0] = max_omega;
  u_max[1] = max_omega;
  u_max[2] = max_omega;
  u_max[3] = max_omega;
  u_max[4] = max_b;
  u_max[5] = max_b;
  u_max[6] = max_b;
  u_max[7] = max_b;
  u_max[8] = max_g;
  u_max[9] = max_g;
  u_max[10] = max_g;
  u_max[11] = max_g;
  u_max[12] = max_theta_protection;
  u_max[13] = max_phi;
  u_max[14] = max_delta_ailerons;
  u_min[0] = min_omega;
  u_min[1] = min_omega;
  u_min[2] = min_omega;
  u_min[3] = min_omega;
  u_min[4] = min_b;
  u_min[5] = min_b;
  u_min[6] = min_b;
  u_min[7] = min_b;
  u_min[8] = min_g;
  u_min[9] = min_g;
  u_min[10] = min_g;
  u_min[11] = min_g;
  u_min[12] = min_theta_protection;
  u_min[13] = -max_phi;
  u_min[14] = min_delta_ailerons;
  if (approach_mode != 0.0) {
    double b_max_tilt_value_approach[2];
    b_max_tilt_value_approach[0] = 0.0;
    b_max_tilt_value_approach[1] = k_alt_tilt_constraint * lidar_alt_corrected -
      min_alt_tilt_constraint * k_alt_tilt_constraint;
    max_tilt_value_approach = maximum(b_max_tilt_value_approach);

    /* Elevation angle */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_b;
    b_max_approach = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_b;
    b_min_approach = maximum(b_max_tilt_value_approach);

    /* Azimuth angle */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_g;
    g_max_approach = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_g;
    g_min_approach = maximum(b_max_tilt_value_approach);
    u_max[4] = b_max_approach;
    u_min[4] = b_min_approach;
    u_max[8] = g_max_approach;
    u_min[8] = g_min_approach;
    u_max[5] = b_max_approach;
    u_min[5] = b_min_approach;
    u_max[9] = g_max_approach;
    u_min[9] = g_min_approach;
    u_max[6] = b_max_approach;
    u_min[6] = b_min_approach;
    u_max[10] = g_max_approach;
    u_min[10] = g_min_approach;
    u_max[7] = b_max_approach;
    u_min[7] = b_min_approach;
    u_max[11] = g_max_approach;
    u_min[11] = g_min_approach;

    /* Pitch angle  */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_theta_protection;
    u_max[12] = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = min_theta_protection;
    u_min[12] = maximum(b_max_tilt_value_approach);

    /* Roll angle  */
    b_max_tilt_value_approach[0] = max_tilt_value_approach;
    b_max_tilt_value_approach[1] = max_phi;
    u_max[13] = minimum(b_max_tilt_value_approach);
    b_max_tilt_value_approach[0] = -max_tilt_value_approach;
    b_max_tilt_value_approach[1] = -max_phi;
    u_min[13] = maximum(b_max_tilt_value_approach);
  }

  for (i = 0; i < 11; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }

  memcpy(&u_max_scaled[0], &u_max[0], 15U * sizeof(double));
  memcpy(&u_min_scaled[0], &u_min[0], 15U * sizeof(double));
  u_max_scaled[0] = u_max[0] / gain_motor.contents;
  u_max_scaled[1] = u_max[1] / gain_motor.contents;
  u_max_scaled[2] = u_max[2] / gain_motor.contents;
  u_max_scaled[3] = u_max[3] / gain_motor.contents;
  u_min_scaled[0] = u_min[0] / gain_motor.contents;
  u_min_scaled[1] = u_min[1] / gain_motor.contents;
  u_min_scaled[2] = u_min[2] / gain_motor.contents;
  u_min_scaled[3] = u_min[3] / gain_motor.contents;
  u_max_scaled[4] /= gain_el.contents;
  u_max_scaled[5] /= gain_el.contents;
  u_max_scaled[6] /= gain_el.contents;
  u_max_scaled[7] /= gain_el.contents;
  u_min_scaled[4] /= gain_el.contents;
  u_min_scaled[5] /= gain_el.contents;
  u_min_scaled[6] /= gain_el.contents;
  u_min_scaled[7] /= gain_el.contents;
  u_max_scaled[8] /= gain_az.contents;
  u_max_scaled[9] /= gain_az.contents;
  u_max_scaled[10] /= gain_az.contents;
  u_max_scaled[11] /= gain_az.contents;
  u_min_scaled[8] /= gain_az.contents;
  u_min_scaled[9] /= gain_az.contents;
  u_min_scaled[10] /= gain_az.contents;
  u_min_scaled[11] /= gain_az.contents;
  u_max_scaled[12] /= gain_theta.contents;
  u_min_scaled[12] /= gain_theta.contents;
  u_max_scaled[13] /= gain_phi.contents;
  u_min_scaled[13] /= gain_phi.contents;
  u_max_scaled[14] /= gain_ailerons.contents;
  u_min_scaled[14] /= gain_ailerons.contents;
  memcpy(&u_max[0], &actual_u[0], 15U * sizeof(double));
  u_max[0] = Omega_1 / gain_motor.contents;
  u_max[1] = Omega_2 / gain_motor.contents;
  u_max[2] = Omega_3 / gain_motor.contents;
  u_max[3] = Omega_4 / gain_motor.contents;
  u_max[4] /= gain_el.contents;
  u_max[5] /= gain_el.contents;
  u_max[6] /= gain_el.contents;
  u_max[7] /= gain_el.contents;
  u_max[8] /= gain_az.contents;
  u_max[9] /= gain_az.contents;
  u_max[10] /= gain_az.contents;
  u_max[11] /= gain_az.contents;
  u_max[12] /= gain_theta.contents;
  u_max[13] /= gain_phi.contents;
  u_max[14] /= gain_ailerons.contents;

  /*  Apply Nonlinear optimization algorithm: */
  c_compute_acc_cascaded_nonlinea(actual_u, b_p.contents, b_q.contents,
    b_r.contents, b_K_p_T.contents, b_K_p_M.contents, b_m.contents,
    b_I_xx.contents, b_I_yy.contents, b_I_zz.contents, b_l_1.contents,
    b_l_2.contents, b_l_3.contents, b_l_4.contents, b_l_z.contents,
    b_Cl_alpha.contents, b_Cd_zero.contents, b_K_Cd.contents,
    b_Cm_alpha.contents, b_Cm_zero.contents, b_CL_aileron.contents,
    b_rho.contents, b_V.contents, b_S.contents, b_wing_chord.contents,
    b_flight_path_angle.contents, b_Beta.contents, current_accelerations);
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = dv[i] + current_accelerations[i];
  }

  /* Pseudo-control hedging:  */
  if (b_V.contents > transition_speed) {
    /*  CL_max =  Cl_alpha * max_alpha;   */
    b_max_approach = b_Cl_alpha.contents * min_alpha;

    /*  max_vert_force_fwd = CL_max * 0.5 * rho * S * V^2 * cos(max_theta_protection*pi/180); */
    max_tilt_value_approach = b_V.contents;
    b_max_approach = b_max_approach * 0.5 * b_rho.contents * b_S.contents *
      (max_tilt_value_approach * max_tilt_value_approach) * cos
      (min_theta_protection * 3.1415926535897931 / 180.0);
    b_max_approach = 9.81 - b_max_approach / b_m.contents;

    /*      min_vert_acc_fwd = 9.81 - max_vert_force_fwd / m;  */
    max_tilt_value_approach = 0.0;
    if (dv_global.contents[2] >= b_max_approach) {
      max_tilt_value_approach = dv_global.contents[2] - b_max_approach;

      /*      elseif(dv_global(3) <= min_vert_acc_fwd) */
      /*          vert_acc_cut = dv_global(3) - min_vert_acc_fwd;  */
    }

    dv_global.contents[2] -= max_tilt_value_approach;
  }

  /* Compute weights for actuators and make sure they are always positive */
  max_tilt_value_approach = W_act_motor_const + W_act_motor_speed * b_V.contents;
  W_act_motor.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_tilt_el_const + W_act_tilt_el_speed *
    b_V.contents;
  W_act_tilt_el.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_tilt_az_const + W_act_tilt_az_speed *
    b_V.contents;
  W_act_tilt_az.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_theta_const + W_act_theta_speed * b_V.contents;
  W_act_theta.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_phi_const + W_act_phi_speed * b_V.contents;
  W_act_phi.contents = fmax(0.0, max_tilt_value_approach);
  max_tilt_value_approach = W_act_ailerons_const + W_act_ailerons_speed *
    b_V.contents;
  W_act_ailerons.contents = fmax(0.0, max_tilt_value_approach);

  /* Default values for the optimizer: */
  tic();

  /* First optimization run to identify the pitch and roll angles: */
  memcpy(&u_min[0], &u_max[0], 15U * sizeof(double));
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.rho = &b_rho;
  expl_temp.r = &b_r;
  expl_temp.q = &b_q;
  expl_temp.p = &b_p;
  expl_temp.m = &b_m;
  expl_temp.l_z = &b_l_z;
  expl_temp.l_4 = &b_l_4;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_2 = &b_l_2;
  expl_temp.l_1 = &b_l_1;
  expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  expl_temp.gain_ailerons = &gain_ailerons;
  expl_temp.gain_motor = &gain_motor;
  expl_temp.gain_theta = &gain_theta;
  expl_temp.gain_phi = &gain_phi;
  expl_temp.gain_az = &gain_az;
  expl_temp.gain_el = &gain_el;
  expl_temp.flight_path_angle = &b_flight_path_angle;
  expl_temp.desired_ailerons_value = &b_desired_ailerons_value;
  expl_temp.desired_motor_value = &b_desired_motor_value;
  expl_temp.desired_theta_value = &b_desired_theta_value;
  expl_temp.desired_phi_value = &b_desired_phi_value;
  expl_temp.desired_az_value = &b_desired_az_value;
  expl_temp.desired_el_value = &b_desired_el_value;
  expl_temp.W_act_ailerons = &W_act_ailerons;
  expl_temp.W_act_tilt_az = &W_act_tilt_az;
  expl_temp.W_act_tilt_el = &W_act_tilt_el;
  expl_temp.W_dv_6 = &b_W_dv_6;
  expl_temp.W_dv_5 = &b_W_dv_5;
  expl_temp.W_dv_4 = &b_W_dv_4;
  expl_temp.W_dv_3 = &b_W_dv_3;
  expl_temp.W_dv_2 = &b_W_dv_2;
  expl_temp.W_dv_1 = &b_W_dv_1;
  expl_temp.W_act_motor = &W_act_motor;
  expl_temp.W_act_theta = &W_act_theta;
  expl_temp.W_act_phi = &W_act_phi;
  expl_temp.V = &b_V;
  expl_temp.S = &b_S;
  expl_temp.K_p_T = &b_K_p_T;
  expl_temp.K_p_M = &b_K_p_M;
  expl_temp.K_Cd = &b_K_Cd;
  expl_temp.I_zz = &b_I_zz;
  expl_temp.I_yy = &b_I_yy;
  expl_temp.I_xx = &b_I_xx;
  expl_temp.Cm_alpha = &b_Cm_alpha;
  expl_temp.Cm_zero = &b_Cm_zero;
  expl_temp.Cl_alpha = &b_Cl_alpha;
  expl_temp.Cd_zero = &b_Cd_zero;
  expl_temp.CL_aileron = &b_CL_aileron;
  expl_temp.Beta = &b_Beta;
  expl_temp.dv_global = &dv_global;
  b_expl_temp = expl_temp;
  fmincon(&b_expl_temp, u_min, u_min_scaled, u_max_scaled, &b_max_approach,
          &output_iterations, &output_funcCount, c_expl_temp, &b_min_approach,
          &g_max_approach, &g_min_approach, &max_theta_protection);
  elapsed_time_first = toc();
  des_theta_first_iteration = u_min[12] * gain_theta.contents;
  des_phi_first_iteration = u_min[13] * gain_phi.contents;
  actual_u[12] = des_theta_first_iteration;
  actual_u[13] = des_phi_first_iteration;
  c_compute_acc_cascaded_nonlinea(actual_u, b_p.contents, b_q.contents,
    b_r.contents, b_K_p_T.contents, b_K_p_M.contents, b_m.contents,
    b_I_xx.contents, b_I_yy.contents, b_I_zz.contents, b_l_1.contents,
    b_l_2.contents, b_l_3.contents, b_l_4.contents, b_l_z.contents,
    b_Cl_alpha.contents, b_Cd_zero.contents, b_K_Cd.contents,
    b_Cm_alpha.contents, b_Cm_zero.contents, b_CL_aileron.contents,
    b_rho.contents, b_V.contents, b_S.contents, b_wing_chord.contents,
    b_flight_path_angle.contents, b_Beta.contents,
    c_accelerations_attitude_alloca);
  max_tilt_value_approach = 1.0 / transition_speed * b_V.contents;
  max_tilt_value_approach = fmin(1.0, max_tilt_value_approach);
  for (i = 0; i < 6; i++) {
    c_accelerations_attitude_alloca[i] = dv_global.contents[i] +
      (current_accelerations[i] - c_accelerations_attitude_alloca[i]) *
      max_tilt_value_approach;
  }

  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = c_accelerations_attitude_alloca[i];
  }

  /* with the desired theta and phi, let's compute the associated angular */
  /* accelerations through the EC gains and feedbacks:  */
  max_tilt_value_approach = b_Theta.contents;
  max_tilt_value_approach = sin(max_tilt_value_approach);
  b_max_approach = b_Phi.contents;
  b_max_approach = cos(b_max_approach);
  b_min_approach = b_Phi.contents;
  b_min_approach = sin(b_min_approach);
  g_max_approach = b_Theta.contents;
  g_max_approach = cos(g_max_approach);
  g_min_approach = b_Phi.contents;
  g_min_approach = sin(g_min_approach);
  max_theta_protection = b_Phi.contents;
  max_theta_protection = cos(max_theta_protection);
  min_theta_protection = b_Theta.contents;
  min_theta_protection = cos(min_theta_protection);
  b_dv[0] = 1.0;
  b_dv[3] = 0.0;
  b_dv[6] = -max_tilt_value_approach;
  b_dv[1] = 0.0;
  b_dv[4] = b_max_approach;
  b_dv[7] = b_min_approach * g_max_approach;
  b_dv[2] = 0.0;
  b_dv[5] = -g_min_approach;
  b_dv[8] = max_theta_protection * min_theta_protection;
  max_tilt_value_approach = (des_phi_first_iteration - phi_current) * phi_gain;
  b_max_approach = (des_theta_first_iteration - theta_current) * theta_gain;
  for (i = 0; i < 3; i++) {
    des_body_rates[i] = (b_dv[i] * max_tilt_value_approach + b_dv[i + 3] *
                         b_max_approach) + b_dv[i + 6] * des_psi_dot;
  }

  max_tilt_value_approach = dv_global.contents[3];
  b_max_approach = dv_global.contents[4];
  b_min_approach = dv_global.contents[5];
  dv_global.contents[3] = max_tilt_value_approach + ((des_body_rates[0] -
    p_body_current) * p_body_gain - p_dot_current);
  dv_global.contents[4] = b_max_approach + ((des_body_rates[1] - q_body_current)
    * q_body_gain - q_dot_current);
  dv_global.contents[5] = b_min_approach + ((des_body_rates[2] - r_body_current)
    * r_body_gain - r_dot_current);

  /* before entering the second optimization run, let's remove the extra inputs: */
  /* Second optimization run to identify the actuator commands: */
  memcpy(&u_out_second[0], &u_max[0], 12U * sizeof(double));
  u_out_second[12] = u_max[14];
  d_expl_temp.wing_chord = &b_wing_chord;
  d_expl_temp.rho = &b_rho;
  d_expl_temp.r = &b_r;
  d_expl_temp.q = &b_q;
  d_expl_temp.p = &b_p;
  d_expl_temp.m = &b_m;
  d_expl_temp.l_z = &b_l_z;
  d_expl_temp.l_4 = &b_l_4;
  d_expl_temp.l_3 = &b_l_3;
  d_expl_temp.l_2 = &b_l_2;
  d_expl_temp.l_1 = &b_l_1;
  d_expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  d_expl_temp.gain_ailerons = &gain_ailerons;
  d_expl_temp.gain_motor = &gain_motor;
  d_expl_temp.gain_az = &gain_az;
  d_expl_temp.gain_el = &gain_el;
  d_expl_temp.flight_path_angle = &b_flight_path_angle;
  d_expl_temp.desired_ailerons_value = &b_desired_ailerons_value;
  d_expl_temp.desired_motor_value = &b_desired_motor_value;
  d_expl_temp.desired_az_value = &b_desired_az_value;
  d_expl_temp.desired_el_value = &b_desired_el_value;
  d_expl_temp.W_act_ailerons = &W_act_ailerons;
  d_expl_temp.W_act_tilt_az = &W_act_tilt_az;
  d_expl_temp.W_act_tilt_el = &W_act_tilt_el;
  d_expl_temp.W_dv_6 = &b_W_dv_6;
  d_expl_temp.W_dv_5 = &b_W_dv_5;
  d_expl_temp.W_dv_4 = &b_W_dv_4;
  d_expl_temp.W_dv_3 = &b_W_dv_3;
  d_expl_temp.W_dv_2 = &b_W_dv_2;
  d_expl_temp.W_dv_1 = &b_W_dv_1;
  d_expl_temp.W_act_motor = &W_act_motor;
  d_expl_temp.V = &b_V;
  d_expl_temp.Theta = &b_Theta;
  d_expl_temp.S = &b_S;
  d_expl_temp.Phi = &b_Phi;
  d_expl_temp.K_p_T = &b_K_p_T;
  d_expl_temp.K_p_M = &b_K_p_M;
  d_expl_temp.K_Cd = &b_K_Cd;
  d_expl_temp.I_zz = &b_I_zz;
  d_expl_temp.I_yy = &b_I_yy;
  d_expl_temp.I_xx = &b_I_xx;
  d_expl_temp.Cm_alpha = &b_Cm_alpha;
  d_expl_temp.Cm_zero = &b_Cm_zero;
  d_expl_temp.Cl_alpha = &b_Cl_alpha;
  d_expl_temp.Cd_zero = &b_Cd_zero;
  d_expl_temp.CL_aileron = &b_CL_aileron;
  d_expl_temp.Beta = &b_Beta;
  d_expl_temp.dv_global = &dv_global;
  b_u_min_scaled[12] = u_min_scaled[14];
  memcpy(&b_u_min_scaled[0], &u_min_scaled[0], 12U * sizeof(double));
  memcpy(&b_u_max_scaled[0], &u_max_scaled[0], 12U * sizeof(double));
  b_u_max_scaled[12] = u_max_scaled[14];
  e_expl_temp = d_expl_temp;
  b_fmincon(&e_expl_temp, u_out_second, b_u_min_scaled, b_u_max_scaled, &d,
            &b_max_approach, &min_theta_protection, c_expl_temp, &b_min_approach,
            &g_max_approach, &g_min_approach, &max_theta_protection);
  *exitflag = d;
  *elapsed_time = toc();
  *N_evaluations = output_funcCount + min_theta_protection;
  *N_iterations = output_iterations + b_max_approach;
  max_tilt_value_approach = gain_motor.contents;
  u_out[0] = u_out_second[0] * max_tilt_value_approach;
  u_out[1] = u_out_second[1] * max_tilt_value_approach;
  u_out[2] = u_out_second[2] * max_tilt_value_approach;
  u_out[3] = u_out_second[3] * max_tilt_value_approach;
  max_tilt_value_approach = gain_el.contents;
  u_out[4] = u_out_second[4] * max_tilt_value_approach;
  u_out[5] = u_out_second[5] * max_tilt_value_approach;
  u_out[6] = u_out_second[6] * max_tilt_value_approach;
  u_out[7] = u_out_second[7] * max_tilt_value_approach;
  max_tilt_value_approach = gain_az.contents;
  u_out[8] = u_out_second[8] * max_tilt_value_approach;
  u_out[9] = u_out_second[9] * max_tilt_value_approach;
  u_out[10] = u_out_second[10] * max_tilt_value_approach;
  u_out[11] = u_out_second[11] * max_tilt_value_approach;
  u_out[12] = b_Theta.contents;
  u_out[13] = b_Phi.contents;
  u_out[14] = u_out_second[12] * gain_ailerons.contents;
  c_compute_acc_cascaded_nonlinea(u_out, b_p.contents, b_q.contents,
    b_r.contents, b_K_p_T.contents, b_K_p_M.contents, b_m.contents,
    b_I_xx.contents, b_I_yy.contents, b_I_zz.contents, b_l_1.contents,
    b_l_2.contents, b_l_3.contents, b_l_4.contents, b_l_z.contents,
    b_Cl_alpha.contents, b_Cd_zero.contents, b_K_Cd.contents,
    b_Cm_alpha.contents, b_Cm_zero.contents, b_CL_aileron.contents,
    b_rho.contents, b_V.contents, b_S.contents, b_wing_chord.contents,
    b_flight_path_angle.contents, b_Beta.contents,
    c_accelerations_attitude_alloca);
  for (i = 0; i < 6; i++) {
    residuals[i] = dv_global.contents[i] - c_accelerations_attitude_alloca[i];
  }

  u_out[12] = des_theta_first_iteration;
  u_out[13] = des_phi_first_iteration;

  /*  Print infos */
  if (verbose != 0.0) {
    printf("\n Solution: \n");
    fflush(stdout);
    printf(" Motors [rad/s] =  ");
    fflush(stdout);
    printf(" %f ", u_out[0]);
    fflush(stdout);
    printf(" %f ", u_out[1]);
    fflush(stdout);
    printf(" %f ", u_out[2]);
    fflush(stdout);
    printf(" %f ", u_out[3]);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Elevator angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[4] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[5] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[6] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[7] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Azimuth angles [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[8] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[9] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[10] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf(" %f ", u_out[11] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Theta [deg] =  ");
    fflush(stdout);
    printf(" %f ", des_theta_first_iteration * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Phi [deg] =  ");
    fflush(stdout);
    printf(" %f ", des_phi_first_iteration * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Ailerons deflection [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[14] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time first / second = %f / %f \n", elapsed_time_first,
           *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations first / second = %f / %f", output_iterations,
           b_max_approach);
    fflush(stdout);
    printf("\n Number of evaluations first / second = %f / %f", output_funcCount,
           min_theta_protection);
    fflush(stdout);
    printf("\n Exit flag optimizer = %f \n", *exitflag);
    fflush(stdout);
    printf("\n Modeled accelerations =   ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", current_accelerations[i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n desired acc increment =    ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", dv[i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Requested accelerations =  ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      max_tilt_value_approach = dv_global.contents[i];
      printf(" %f ", max_tilt_value_approach);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Achieved accelerations =   ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", c_accelerations_attitude_alloca[i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Acc residuals / norm  =    ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", residuals[i]);
      fflush(stdout);
    }

    printf(" / ");
    fflush(stdout);
    printf(" %f \n", b_norm(residuals));
    fflush(stdout);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Cascaded_nonlinear_controller_control_rf_w_ailerons_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_Cascaded_nonlinear_controller_control_rf_w_ailerons = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Cascaded_nonlinear_controller_control_rf_w_ailerons_terminate(void)
{
  isInitialized_Cascaded_nonlinear_controller_control_rf_w_ailerons = false;
}

/*
 * File trailer for Cascaded_nonlinear_controller_control_rf_w_ailerons.c
 *
 * [EOF]
 */
