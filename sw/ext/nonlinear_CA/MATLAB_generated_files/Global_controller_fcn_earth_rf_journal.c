/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Global_controller_fcn_earth_rf_journal.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 20-Nov-2023 22:27:20
 */

/* Include Files */
#include "Global_controller_fcn_earth_rf_journal.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"
#include "rt_nonfinite.h"
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
  bool gradOK;
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
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
  captured_var *gain_theta;
  captured_var *Cl_alpha;
  captured_var *S;
  captured_var *V;
  captured_var *rho;
  captured_var *flight_path_angle;
  captured_var *Beta;
  captured_var *K_Cd;
  captured_var *Cd_zero;
  captured_var *gain_phi;
  captured_var *K_p_T;
  captured_var *gain_motor;
  captured_var *gain_el;
  captured_var *gain_az;
  captured_var *m;
  captured_var *I_zz;
  captured_var *p;
  captured_var *r;
  captured_var *I_xx;
  captured_var *I_yy;
  captured_var *l_z;
  captured_var *K_p_M;
  captured_var *Cm_zero;
  captured_var *wing_chord;
  captured_var *l_4;
  captured_var *l_3;
  captured_var *Cm_alpha;
  captured_var *q;
  captured_var *l_1;
  captured_var *l_2;
  captured_var *CL_aileron;
  captured_var *gain_ailerons;
  captured_var *W_dv_4;
  captured_var *W_dv_6;
  captured_var *W_act_motor;
  captured_var *gamma_quadratic_du;
  captured_var *desired_motor_value;
  captured_var *W_dv_5;
  captured_var *W_dv_3;
  captured_var *W_dv_1;
  captured_var *W_dv_2;
  captured_var *W_act_tilt_el;
  captured_var *desired_el_value;
  captured_var *W_act_tilt_az;
  captured_var *desired_az_value;
  captured_var *W_act_theta;
  captured_var *desired_theta_value;
  captured_var *W_act_phi;
  captured_var *desired_phi_value;
  captured_var *W_act_ailerons;
  captured_var *desired_ailerons_value;
} d_struct_T;

#endif                                 /* typedef_d_struct_T */

#ifndef typedef_nested_function
#define typedef_nested_function

typedef struct {
  d_struct_T workspace;
} nested_function;

#endif                                 /* typedef_nested_function */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T

typedef struct {
  double workspace_double[496];
  int workspace_int[31];
  int workspace_sort[31];
} e_struct_T;

#endif                                 /* typedef_e_struct_T */

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
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double OptimalityTolerance;
  double ConstraintTolerance;
  double ObjectiveLimit;
  double PricingTolerance;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  bool RemainFeasible;
  bool IterDisplayQP;
} h_struct_T;

#endif                                 /* typedef_h_struct_T */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T

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
} i_struct_T;

#endif                                 /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T

typedef struct {
  nested_function objfun;
  double f_1;
  double f_2;
  int nVar;
  int mIneq;
  int mEq;
  int numEvals;
  bool SpecifyObjectiveGradient;
  bool SpecifyConstraintGradient;
  bool isEmptyNonlcon;
  bool hasLB[15];
  bool hasUB[15];
  bool hasBounds;
  int FiniteDifferenceType;
} j_struct_T;

#endif                                 /* typedef_j_struct_T */

#ifndef typedef_k_struct_T
#define typedef_k_struct_T

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
} k_struct_T;

#endif                                 /* typedef_k_struct_T */

#ifndef typedef_l_struct_T
#define typedef_l_struct_T

typedef struct {
  nested_function objfun;
  int nVar;
  int mCineq;
  int mCeq;
  bool NonFiniteSupport;
  bool SpecifyObjectiveGradient;
  bool SpecifyConstraintGradient;
  bool ScaleProblem;
} l_struct_T;

#endif                                 /* typedef_l_struct_T */

/* Variable Definitions */
static double freq;
static bool freq_not_empty;
static coderTimespec savedTime;
static bool savedTime_not_empty;
static bool isInitialized_Global_controller_fcn_earth_rf_journal = false;

/* Function Declarations */
static bool BFGSUpdate(int nvar, double Bk[225], const double sk[16], double yk
  [16], double workspace[496]);
static void PresolveWorkingSet(i_struct_T *solution, e_struct_T *memspace,
  k_struct_T *workingset, f_struct_T *qrmanager);
static void RemoveDependentIneq_(k_struct_T *workingset, f_struct_T *qrmanager,
  e_struct_T *memspace, double tolfactor);
static void addBoundToActiveSetMatrix_(k_struct_T *obj, int TYPE, int idx_local);
static void b_computeGradLag(double workspace[496], int nVar, const double grad
  [16], const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB,
  const int finiteUB[16], int mUB, const double lambda[31]);
static void b_driver(const double lb[15], const double ub[15], i_struct_T
                     *TrialState, b_struct_T *MeritFunction, const l_struct_T
                     *FcnEvaluator, e_struct_T *memspace, k_struct_T *WorkingSet,
                     double Hessian[225], f_struct_T *QRManager, g_struct_T
                     *CholManager, struct_T *QPObjective);
static double b_maxConstraintViolation(const k_struct_T *obj, const double x[16]);
static double b_norm(const double x[15]);
static void b_test_exit(c_struct_T *Flags, e_struct_T *memspace, b_struct_T
  *MeritFunction, const k_struct_T *WorkingSet, i_struct_T *TrialState,
  f_struct_T *QRManager, const double lb[15], const double ub[15]);
static void b_timeKeeper(double *outTime_tv_sec, double *outTime_tv_nsec);
static void b_xaxpy(int n, double a, const double x[72], int ix0, double y[12],
                    int iy0);
static double b_xdotc(int n, const double x[36], int ix0, const double y[36],
                      int iy0);
static void b_xgemm(int m, int n, int k, const double A[961], int ia0, const
                    double B[496], double C[961]);
static double b_xnrm2(int n, const double x[16]);
static void b_xrot(double x[72], int ix0, int iy0, double c, double s);
static void b_xswap(double x[72], int ix0, int iy0);
static void c_Nonlinear_controller_fcn_cont(double K_p_T, double K_p_M, double m,
  double I_xx, double I_yy, double I_zz, double l_1, double l_2, double l_3,
  double l_4, double l_z, double Phi, double Theta, double Omega_1, double
  Omega_2, double Omega_3, double Omega_4, double b_1, double b_2, double b_3,
  double b_4, double g_1, double g_2, double g_3, double g_4, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_act_theta_const, double W_act_theta_speed,
  double W_act_phi_const, double W_act_phi_speed, double W_dv_1, double W_dv_2,
  double W_dv_3, double W_dv_4, double W_dv_5, double W_dv_6, double max_omega,
  double min_omega, double max_b, double min_b, double max_g, double min_g,
  double max_theta, double min_theta, double max_phi, const double dv[6], double
  p, double q, double r, double Cm_zero, double Cl_alpha, double Cd_zero, double
  K_Cd, double Cm_alpha, double rho, double V, double S, double wing_chord,
  double flight_path_angle, double max_alpha, double min_alpha, double Beta,
  double gamma_quadratic_du, double desired_motor_value, double desired_el_value,
  double desired_az_value, double desired_theta_value, double desired_phi_value,
  double verbose, double u_out[15], double residuals[6], double *elapsed_time,
  double *N_iterations, double *N_evaluation, double *exitflag);
static void c_WLS_controller_fcn_earth_rf_j(double K_p_T, double K_p_M, double m,
  double I_xx, double I_yy, double I_zz, double l_1, double l_2, double l_3,
  double l_4, double l_z, double Phi, double Theta, double Psi, double Omega_1,
  double Omega_2, double Omega_3, double Omega_4, double b_1, double b_2, double
  b_3, double b_4, double g_1, double g_2, double g_3, double g_4, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_dv_1, double W_dv_2, double W_dv_3, double
  W_dv_4, double W_dv_5, double W_dv_6, double max_omega, double min_omega,
  double max_b, double min_b, double max_g, double min_g, double dv[6], double p,
  double q, double r, double Cm_zero, double Cl_alpha, double Cd_zero, double
  K_Cd, double Cm_alpha, double rho, double V, double S, double wing_chord,
  double flight_path_angle, double max_alpha, double Beta, double gammasq,
  double desired_motor_value, double desired_el_value, double desired_az_value,
  double desired_theta_value, double desired_phi_value, double u_out[14], double
  residuals[6], double *elapsed_time, double *N_iterations, double *N_evaluation,
  double *exitflag);
static void c_compute_acc_nonlinear_control(const double u_in[15], double p,
  double q, double r, double K_p_T, double K_p_M, double m, double I_xx, double
  I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4, double l_z,
  double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero,
  double CL_aileron, double rho, double V, double S, double wing_chord, double
  flight_path_angle, double Beta, double computed_acc[6]);
static void c_compute_acc_nonlinear_earth_r(const double u_in[12], double Theta,
  double Phi, double Psi, double p, double q, double r, double K_p_T, double
  K_p_M, double m, double I_xx, double I_yy, double I_zz, double l_1, double l_2,
  double l_3, double l_4, double l_z, double Cl_alpha, double Cd_zero, double
  K_Cd, double Cm_alpha, double Cm_zero, double rho, double V, double S, double
  wing_chord, double flight_path_angle, double Beta, double computed_acc[6]);
static void c_xaxpy(int n, double a, const double x[12], int ix0, double y[72],
                    int iy0);
static double c_xnrm2(int n, const double x_data[], int ix0);
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
static void compute_cost_and_gradient_fcn(const b_captured_var *dv_global, const
  captured_var *gain_theta, const captured_var *Cl_alpha, const captured_var *S,
  const captured_var *V, const captured_var *rho, const captured_var
  *flight_path_angle, const captured_var *Beta, const captured_var *K_Cd, const
  captured_var *Cd_zero, const captured_var *gain_phi, const captured_var *K_p_T,
  const captured_var *gain_motor, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *m, const captured_var *I_zz, const
  captured_var *p, const captured_var *r, const captured_var *I_xx, const
  captured_var *I_yy, const captured_var *l_z, const captured_var *K_p_M, const
  captured_var *Cm_zero, const captured_var *wing_chord, const captured_var *l_4,
  const captured_var *l_3, const captured_var *Cm_alpha, const captured_var *q,
  const captured_var *l_1, const captured_var *l_2, const captured_var
  *CL_aileron, const captured_var *gain_ailerons, const captured_var *W_dv_4,
  const captured_var *W_dv_6, const captured_var *W_act_motor, const
  captured_var *gamma_quadratic_du, const captured_var *desired_motor_value,
  const captured_var *W_dv_5, const captured_var *W_dv_3, const captured_var
  *W_dv_1, const captured_var *W_dv_2, const captured_var *W_act_tilt_el, const
  captured_var *desired_el_value, const captured_var *W_act_tilt_az, const
  captured_var *desired_az_value, const captured_var *W_act_theta, const
  captured_var *desired_theta_value, const captured_var *W_act_phi, const
  captured_var *desired_phi_value, const captured_var *W_act_ailerons, const
  captured_var *desired_ailerons_value, const double u_in[15], double *cost,
  double gradient[15]);
static void compute_deltax(const double H[225], i_struct_T *solution, e_struct_T
  *memspace, const f_struct_T *qrmanager, g_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef);
static void countsort(int x[31], int xLen, int workspace[31], int xMin, int xMax);
static void d_xaxpy(int n, double a, int ix0, double y[36], int iy0);
static double d_xnrm2(int n, const double x[72], int ix0);
static void deleteColMoveEnd(f_struct_T *obj, int idx);
static int div_nde_s32_floor(int numerator, int denominator);
static void driver(const double H[225], const double f[16], i_struct_T *solution,
                   e_struct_T *memspace, k_struct_T *workingset, f_struct_T
                   *qrmanager, g_struct_T *cholmanager, struct_T *objective,
                   h_struct_T *options, int runTimeOptions_MaxIterations);
static double e_xnrm2(int n, const double x[6], int ix0);
static void evalObjAndConstr(const d_struct_T *obj_objfun_workspace, const
  double x[15], double *fval, int *status);
static void evalObjAndConstrAndDerivatives(const d_struct_T
  *obj_objfun_workspace, const double x[15], double grad_workspace[16], double
  *fval, int *status);
static void factorQR(f_struct_T *obj, const double A[496], int mrows, int ncols);
static void factoryConstruct(d_struct_T *objfun_workspace, const double lb[15],
  const double ub[15], j_struct_T *obj);
static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const k_struct_T *workingset, f_struct_T *qrmanager);
static void feasibleratiotest(const double solution_xstar[16], const double
  solution_searchDir[16], int workingset_nVar, const double workingset_lb[16],
  const double workingset_ub[16], const int workingset_indexLB[16], const int
  workingset_indexUB[16], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[31], const int
  workingset_nWConstr[5], bool isPhaseOne, double *alpha, bool *newBlocking, int
  *constrType, int *constrIdx);
static void fmincon(d_struct_T *fun_workspace, const double x0[15], const double
                    lb[15], const double ub[15], double x[15], double *fval,
                    double *exitflag, double *output_iterations, double
                    *output_funcCount, char output_algorithm[3], double
                    *output_constrviolation, double *output_stepsize, double
                    *output_lssteplength, double *output_firstorderopt);
static void fullColLDL2_(g_struct_T *obj, int NColsRemain);
static void iterate(const double H[225], const double f[16], i_struct_T
                    *solution, e_struct_T *memspace, k_struct_T *workingset,
                    f_struct_T *qrmanager, g_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations);
static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[496],
  const double H[225], const double f[16], const double x[16]);
static double maxConstraintViolation(const k_struct_T *obj, const double x[496],
  int ix0);
static void minimum(const double x[12], double *ex, int *idx);
static void mldivide(const double A_data[], const int A_size[2], const double B
                     [18], double Y_data[], int *Y_size);
static void qrf(double A[961], int m, int n, int nfxd, double tau[31]);
static double rt_hypotd_snf(double u0, double u1);
static void setProblemType(k_struct_T *obj, int PROBLEM_TYPE);
static void solve(const g_struct_T *obj, double rhs[16]);
static void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[31], const int WorkingSet_Wlocalidx[31], double workspace[496]);
static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], i_struct_T *TrialState, b_struct_T
                 *MeritFunction, e_struct_T *memspace, k_struct_T *WorkingSet,
                 f_struct_T *QRManager, g_struct_T *CholManager, struct_T
                 *QPObjective, h_struct_T *qpoptions);
static void svd(const double A[72], double U[72], double s[6], double V[36]);
static void test_exit(b_struct_T *MeritFunction, const k_struct_T *WorkingSet,
                      i_struct_T *TrialState, const double lb[15], const double
                      ub[15], bool *Flags_gradOK, bool *Flags_fevalOK, bool
                      *Flags_done, bool *Flags_stepAccepted, bool
                      *Flags_failedLineSearch, int *Flags_stepType);
static void tic(void);
static void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec);
static double toc(void);
static void xaxpy(int n, double a, int ix0, double y[72], int iy0);
static double xdotc(int n, const double x[72], int ix0, const double y[72], int
                    iy0);
static void xgemm(int m, int n, int k, const double A[225], int lda, const
                  double B[961], int ib0, double C[496]);
static void xgemv(int m, int n, const double A[961], const double x[16], double
                  y[496]);
static void xgeqp3(double A[961], int m, int n, int jpvt[31], double tau[31]);
static double xnrm2(int n, const double x[961], int ix0);
static int xpotrf(int n, double A[961]);
static void xrot(double x[36], int ix0, int iy0, double c, double s);
static void xrotg(double *a, double *b, double *c, double *s);
static void xswap(double x[36], int ix0, int iy0);
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
  int ia;
  int iac;
  int ix;
  int k;
  bool success;
  dotSY = 0.0;
  for (k = 0; k < nvar; k++) {
    dotSY += sk[k] * yk[k];
    workspace[k] = 0.0;
  }

  ix = 0;
  i = 15 * (nvar - 1) + 1;
  for (iac = 1; iac <= i; iac += 15) {
    i1 = (iac + nvar) - 1;
    for (ia = iac; ia <= i1; ia++) {
      k = ia - iac;
      workspace[k] += Bk[ia - 1] * sk[ix];
    }

    ix++;
  }

  curvatureS = 0.0;
  if (nvar >= 1) {
    for (k = 0; k < nvar; k++) {
      curvatureS += sk[k] * workspace[k];
    }
  }

  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    for (k = 0; k < nvar; k++) {
      yk[k] *= theta;
    }

    if (!(1.0 - theta == 0.0)) {
      ix = nvar - 1;
      for (k = 0; k <= ix; k++) {
        yk[k] += (1.0 - theta) * workspace[k];
      }
    }

    dotSY = 0.0;
    for (k = 0; k < nvar; k++) {
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
    theta = -1.0 / curvatureS;
    if (!(theta == 0.0)) {
      ix = 0;
      for (k = 0; k < nvar; k++) {
        if (workspace[k] != 0.0) {
          curvatureS = workspace[k] * theta;
          i = ix + 1;
          i1 = nvar + ix;
          for (iac = i; iac <= i1; iac++) {
            Bk[iac - 1] += workspace[(iac - ix) - 1] * curvatureS;
          }
        }

        ix += 15;
      }
    }

    theta = 1.0 / dotSY;
    if (!(theta == 0.0)) {
      ix = 0;
      for (k = 0; k < nvar; k++) {
        if (yk[k] != 0.0) {
          curvatureS = yk[k] * theta;
          i = ix + 1;
          i1 = nvar + ix;
          for (iac = i; iac <= i1; iac++) {
            Bk[iac - 1] += yk[(iac - ix) - 1] * curvatureS;
          }
        }

        ix += 15;
      }
    }
  }

  return success;
}

/*
 * Arguments    : i_struct_T *solution
 *                e_struct_T *memspace
 *                k_struct_T *workingset
 *                f_struct_T *qrmanager
 * Return Type  : void
 */
static void PresolveWorkingSet(i_struct_T *solution, e_struct_T *memspace,
  k_struct_T *workingset, f_struct_T *qrmanager)
{
  double tol;
  int idx;
  int idxDiag;
  int idx_col;
  int ix;
  int k;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  int nVar;
  solution->state = 82;
  nVar = workingset->nVar - 1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    int i;
    int u0;
    for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
      for (idx_col = 0; idx_col <= nVar; idx_col++) {
        qrmanager->QR[ix + 31 * idx_col] = workingset->ATwset[idx_col + (ix << 4)];
      }
    }

    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }

    memset(&qrmanager->jpvt[0], 0, (nVar + 1) * sizeof(int));
    i = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i == 0) {
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
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= nDepInd - 1)) {
        double qtb;
        ix = 31 * ((mTotalWorkingEq_tmp_tmp - idx) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[ix + k] * workingset->bwset[k];
        }

        if (fabs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx++;
        }
      }
    }

    if (nDepInd > 0) {
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = 31 * idx_col;
        idx = idx_col << 4;
        for (k = 0; k <= nVar; k++) {
          qrmanager->QR[idxDiag + k] = workingset->ATwset[idx + k];
        }
      }

      for (idx = 0; idx < mWorkingFixed; idx++) {
        qrmanager->jpvt[idx] = 1;
      }

      idxDiag = workingset->nWConstr[0] + 1;
      if (idxDiag <= mTotalWorkingEq_tmp_tmp) {
        memset(&qrmanager->jpvt[idxDiag + -1], 0, ((mTotalWorkingEq_tmp_tmp -
                 idxDiag) + 1) * sizeof(int));
      }

      if (i == 0) {
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

      for (idx = 0; idx < nDepInd; idx++) {
        memspace->workspace_int[idx] = qrmanager->jpvt[(mTotalWorkingEq_tmp_tmp
          - nDepInd) + idx];
      }

      countsort(memspace->workspace_int, nDepInd, memspace->workspace_sort, 1,
                mTotalWorkingEq_tmp_tmp);
      for (idx = nDepInd; idx >= 1; idx--) {
        i = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (i != 0) {
          idxDiag = memspace->workspace_int[idx - 1];
          if (idxDiag <= i) {
            if ((workingset->nActiveConstr == i) || (idxDiag == i)) {
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
  }

  if ((nDepInd != -1) && (workingset->nActiveConstr <= 31)) {
    bool guard1 = false;
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
      tol = b_maxConstraintViolation(workingset, solution->xstar);
      if (tol > 1.0E-6) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    idxDiag = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    ix = workingset->nActiveConstr;
    for (idx = idxDiag; idx <= ix; idx++) {
      workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[idx -
        1] - 1] + workingset->Wlocalidx[idx - 1]) - 2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

/*
 * Arguments    : k_struct_T *workingset
 *                f_struct_T *qrmanager
 *                e_struct_T *memspace
 *                double tolfactor
 * Return Type  : void
 */
static void RemoveDependentIneq_(k_struct_T *workingset, f_struct_T *qrmanager,
  e_struct_T *memspace, double tolfactor)
{
  int idx;
  int idx_col;
  int k;
  int nDepIneq;
  int nFixedConstr;
  int nVar_tmp_tmp;
  nDepIneq = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar_tmp_tmp = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    double tol;
    int idxDiag;
    int iy0;
    tol = tolfactor * (double)workingset->nVar * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt[idx] = 1;
    }

    idx_col = nFixedConstr + 1;
    if (idx_col <= nDepIneq) {
      memset(&qrmanager->jpvt[idx_col + -1], 0, ((nDepIneq - idx_col) + 1) *
             sizeof(int));
    }

    for (idx_col = 0; idx_col < nDepIneq; idx_col++) {
      iy0 = 31 * idx_col;
      idxDiag = idx_col << 4;
      for (k = 0; k < nVar_tmp_tmp; k++) {
        qrmanager->QR[iy0 + k] = workingset->ATwset[idxDiag + k];
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
      iy0 = workingset->nActiveConstr;
      if (idxDiag <= iy0) {
        iy0 = idxDiag;
      }

      qrmanager->minRowCol = iy0;
      xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
             qrmanager->jpvt, qrmanager->tau);
    }

    nDepIneq = 0;
    for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar_tmp_tmp; idx--) {
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
      iy0 = memspace->workspace_int[idx - 1] - 1;
      idxDiag = workingset->Wid[iy0] - 1;
      workingset->isActiveConstr[(workingset->isActiveIdx[idxDiag] +
        workingset->Wlocalidx[iy0]) - 2] = false;
      workingset->Wid[iy0] = workingset->Wid[workingset->nActiveConstr - 1];
      workingset->Wlocalidx[iy0] = workingset->Wlocalidx
        [workingset->nActiveConstr - 1];
      idx_col = workingset->nVar;
      for (k = 0; k < idx_col; k++) {
        workingset->ATwset[k + (iy0 << 4)] = workingset->ATwset[k +
          ((workingset->nActiveConstr - 1) << 4)];
      }

      workingset->bwset[iy0] = workingset->bwset[workingset->nActiveConstr - 1];
      workingset->nActiveConstr--;
      workingset->nWConstr[idxDiag]--;
    }
  }
}

/*
 * Arguments    : k_struct_T *obj
 *                int TYPE
 *                int idx_local
 * Return Type  : void
 */
static void addBoundToActiveSetMatrix_(k_struct_T *obj, int TYPE, int idx_local)
{
  int colOffset;
  int i;
  int idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  colOffset = ((obj->nActiveConstr - 1) << 4) - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = obj->ub[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = obj->lb[idx_bnd_local - 1];
  }

  if (idx_bnd_local - 2 >= 0) {
    memset(&obj->ATwset[colOffset + 1], 0, (((idx_bnd_local + colOffset) -
             colOffset) - 1) * sizeof(double));
  }

  obj->ATwset[idx_bnd_local + colOffset] = 2.0 * (double)(TYPE == 5) - 1.0;
  idx_bnd_local++;
  i = obj->nVar;
  if (idx_bnd_local <= i) {
    memset(&obj->ATwset[idx_bnd_local + colOffset], 0, ((((i + colOffset) -
              idx_bnd_local) - colOffset) + 1) * sizeof(double));
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
  int iL0;
  int idx;
  memcpy(&workspace[0], &grad[0], nVar * sizeof(double));
  for (idx = 0; idx < mFixed; idx++) {
    i = finiteFixed[idx];
    workspace[i - 1] += lambda[idx];
  }

  for (idx = 0; idx < mLB; idx++) {
    i = finiteLB[idx];
    workspace[i - 1] -= lambda[mFixed + idx];
  }

  iL0 = mFixed + mLB;
  for (idx = 0; idx < mUB; idx++) {
    i = finiteUB[idx];
    workspace[i - 1] += lambda[iL0 + idx];
  }
}

/*
 * Arguments    : const double lb[15]
 *                const double ub[15]
 *                i_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                const l_struct_T *FcnEvaluator
 *                e_struct_T *memspace
 *                k_struct_T *WorkingSet
 *                double Hessian[225]
 *                f_struct_T *QRManager
 *                g_struct_T *CholManager
 *                struct_T *QPObjective
 * Return Type  : void
 */
static void b_driver(const double lb[15], const double ub[15], i_struct_T
                     *TrialState, b_struct_T *MeritFunction, const l_struct_T
                     *FcnEvaluator, e_struct_T *memspace, k_struct_T *WorkingSet,
                     double Hessian[225], f_struct_T *QRManager, g_struct_T
                     *CholManager, struct_T *QPObjective)
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

  c_struct_T Flags;
  h_struct_T b_expl_temp;
  h_struct_T expl_temp;
  int i;
  int ineqStart;
  int mConstr;
  int mFixed;
  int mLB;
  int mUB;
  int nVar_tmp_tmp;
  int qpoptions_MaxIterations;
  int u1;
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
  CholManager->regTol_ = rtInf;
  CholManager->workspace_ = rtInf;
  CholManager->workspace2_ = rtInf;
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

  nVar_tmp_tmp = WorkingSet->nVar - 1;
  mFixed = WorkingSet->sizes[0];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  ineqStart = WorkingSet->nVar;
  u1 = (WorkingSet->sizes[3] + WorkingSet->sizes[4]) + (WorkingSet->sizes[0] <<
    1);
  if (ineqStart >= u1) {
    u1 = ineqStart;
  }

  qpoptions_MaxIterations = 10 * u1;
  TrialState->steplength = 1.0;
  test_exit(MeritFunction, WorkingSet, TrialState, lb, ub, &Flags.gradOK,
            &Flags.fevalOK, &Flags.done, &Flags.stepAccepted,
            &Flags.failedLineSearch, &Flags.stepType);
  TrialState->sqpFval_old = TrialState->sqpFval;
  for (u1 = 0; u1 < 15; u1++) {
    TrialState->xstarsqp_old[u1] = TrialState->xstarsqp[u1];
    TrialState->grad_old[u1] = TrialState->grad[u1];
  }

  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        for (u1 = 0; u1 < mLB; u1++) {
          WorkingSet->lb[WorkingSet->indexLB[u1] - 1] = -lb[WorkingSet->
            indexLB[u1] - 1] + TrialState->xstarsqp[WorkingSet->indexLB[u1] - 1];
        }

        for (u1 = 0; u1 < mUB; u1++) {
          WorkingSet->ub[WorkingSet->indexUB[u1] - 1] = ub[WorkingSet->
            indexUB[u1] - 1] - TrialState->xstarsqp[WorkingSet->indexUB[u1] - 1];
        }

        for (u1 = 0; u1 < mFixed; u1++) {
          WorkingSet->ub[WorkingSet->indexFixed[u1] - 1] = ub
            [WorkingSet->indexFixed[u1] - 1] - TrialState->xstarsqp
            [WorkingSet->indexFixed[u1] - 1];
          WorkingSet->bwset[u1] = ub[WorkingSet->indexFixed[u1] - 1] -
            TrialState->xstarsqp[WorkingSet->indexFixed[u1] - 1];
        }

        if (WorkingSet->nActiveConstr > mFixed) {
          ineqStart = mFixed + 1;
          if (ineqStart < 1) {
            ineqStart = 1;
          }

          i = WorkingSet->nActiveConstr;
          for (u1 = ineqStart; u1 <= i; u1++) {
            switch (WorkingSet->Wid[u1 - 1]) {
             case 4:
              WorkingSet->bwset[u1 - 1] = WorkingSet->lb[WorkingSet->
                indexLB[WorkingSet->Wlocalidx[u1 - 1] - 1] - 1];
              break;

             case 5:
              WorkingSet->bwset[u1 - 1] = WorkingSet->ub[WorkingSet->
                indexUB[WorkingSet->Wlocalidx[u1 - 1] - 1] - 1];
              break;

             default:
              /* A check that is always false is detected at compile-time. Eliminating code that follows. */
              break;
            }
          }
        }
      }

      expl_temp.IterDisplayQP = false;
      expl_temp.RemainFeasible = false;
      expl_temp.ProbRelTolFactor = 1.0;
      expl_temp.ConstrRelTolFactor = 1.0;
      expl_temp.PricingTolerance = 0.0;
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.ConstraintTolerance = 1.0E-6;
      expl_temp.OptimalityTolerance = 2.2204460492503131E-14;
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
        for (u1 = 0; u1 <= nVar_tmp_tmp; u1++) {
          TrialState->xstarsqp[u1] += TrialState->delta_x[u1];
        }

        evalObjAndConstr(&FcnEvaluator->objfun.workspace, TrialState->xstarsqp,
                         &TrialState->sqpFval, &ineqStart);
        Flags.fevalOK = (ineqStart == 1);
        TrialState->FunctionEvaluations++;
        if (Flags.fevalOK) {
          MeritFunction->phiFullStep = TrialState->sqpFval;
        } else {
          MeritFunction->phiFullStep = rtInf;
        }
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        double alpha;
        double phi_alpha;
        bool evalWellDefined;
        bool socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }

        evalWellDefined = Flags.fevalOK;
        i = WorkingSet->nVar - 1;
        alpha = 1.0;
        ineqStart = 1;
        phi_alpha = MeritFunction->phiFullStep;
        if (i >= 0) {
          memcpy(&TrialState->searchDir[0], &TrialState->delta_x[0], (i + 1) *
                 sizeof(double));
        }

        int exitg1;
        do {
          exitg1 = 0;
          if (TrialState->FunctionEvaluations < 200) {
            if (evalWellDefined && (phi_alpha <= MeritFunction->phi + alpha *
                                    0.0001 * MeritFunction->phiPrimePlus)) {
              exitg1 = 1;
            } else {
              bool exitg2;
              bool tooSmallX;
              alpha *= 0.7;
              for (u1 = 0; u1 <= i; u1++) {
                TrialState->delta_x[u1] = alpha * TrialState->xstar[u1];
              }

              if (socTaken) {
                phi_alpha = alpha * alpha;
                if ((i + 1 >= 1) && (!(phi_alpha == 0.0))) {
                  for (u1 = 0; u1 <= i; u1++) {
                    TrialState->delta_x[u1] += phi_alpha *
                      TrialState->socDirection[u1];
                  }
                }
              }

              tooSmallX = true;
              u1 = 0;
              exitg2 = false;
              while ((!exitg2) && (u1 <= i)) {
                if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[u1])) <= fabs
                    (TrialState->delta_x[u1])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  u1++;
                }
              }

              if (tooSmallX) {
                ineqStart = -2;
                exitg1 = 1;
              } else {
                for (u1 = 0; u1 <= i; u1++) {
                  TrialState->xstarsqp[u1] = TrialState->xstarsqp_old[u1] +
                    TrialState->delta_x[u1];
                }

                evalObjAndConstr(&FcnEvaluator->objfun.workspace,
                                 TrialState->xstarsqp, &TrialState->sqpFval, &u1);
                TrialState->FunctionEvaluations++;
                evalWellDefined = (u1 == 1);
                if (evalWellDefined) {
                  phi_alpha = TrialState->sqpFval;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            ineqStart = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);

        Flags.fevalOK = evalWellDefined;
        TrialState->steplength = alpha;
        if (ineqStart > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      for (u1 = 0; u1 <= nVar_tmp_tmp; u1++) {
        TrialState->xstarsqp[u1] = TrialState->xstarsqp_old[u1] +
          TrialState->delta_x[u1];
      }

      for (u1 = 0; u1 < mConstr; u1++) {
        TrialState->lambdasqp[u1] += TrialState->steplength *
          (TrialState->lambda[u1] - TrialState->lambdasqp[u1]);
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      for (u1 = 0; u1 < 15; u1++) {
        TrialState->xstarsqp_old[u1] = TrialState->xstarsqp[u1];
        TrialState->grad_old[u1] = TrialState->grad[u1];
      }

      Flags.gradOK = true;
      evalObjAndConstrAndDerivatives(&FcnEvaluator->objfun.workspace,
        TrialState->xstarsqp, TrialState->grad, &TrialState->sqpFval, &ineqStart);
      TrialState->FunctionEvaluations++;
      Flags.fevalOK = (ineqStart == 1);
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
      memcpy(&TrialState->delta_gradLag[0], &TrialState->grad[0], (nVar_tmp_tmp
              + 1) * sizeof(double));
      if (nVar_tmp_tmp + 1 >= 1) {
        for (u1 = 0; u1 <= nVar_tmp_tmp; u1++) {
          TrialState->delta_gradLag[u1] += -TrialState->grad_old[u1];
        }
      }

      BFGSUpdate(nVar_tmp_tmp + 1, Hessian, TrialState->delta_x,
                 TrialState->delta_gradLag, memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/*
 * Arguments    : const k_struct_T *obj
 *                const double x[16]
 * Return Type  : double
 */
static double b_maxConstraintViolation(const k_struct_T *obj, const double x[16])
{
  double v;
  int idx;
  int mFixed;
  int mLB;
  int mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  v = 0.0;
  if (obj->sizes[3] > 0) {
    for (idx = 0; idx < mLB; idx++) {
      int idxLB;
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    for (idx = 0; idx < mUB; idx++) {
      mLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[mLB] - obj->ub[mLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    for (idx = 0; idx < mFixed; idx++) {
      v = fmax(v, fabs(x[obj->indexFixed[idx] - 1] - obj->ub[obj->indexFixed[idx]
                       - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : const double x[15]
 * Return Type  : double
 */
static double b_norm(const double x[15])
{
  double scale;
  double y;
  int k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 15; k++) {
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
 * Arguments    : c_struct_T *Flags
 *                e_struct_T *memspace
 *                b_struct_T *MeritFunction
 *                const k_struct_T *WorkingSet
 *                i_struct_T *TrialState
 *                f_struct_T *QRManager
 *                const double lb[15]
 *                const double ub[15]
 * Return Type  : void
 */
static void b_test_exit(c_struct_T *Flags, e_struct_T *memspace, b_struct_T
  *MeritFunction, const k_struct_T *WorkingSet, i_struct_T *TrialState,
  f_struct_T *QRManager, const double lb[15], const double ub[15])
{
  double optimRelativeFactor;
  double s;
  double smax;
  int idxFiniteLB;
  int idx_max;
  int k;
  int mFixed;
  int mLB;
  int mLambda;
  int mUB;
  int nVar;
  bool dxTooSmall;
  bool exitg1;
  bool isFeasible;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda = ((WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4])
    - 1;
  if (mLambda >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (mLambda +
            1) * sizeof(double));
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
  if (rtIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }

  smax = 0.0;
  for (k = 0; k < mLB; k++) {
    idxFiniteLB = WorkingSet->indexLB[k] - 1;
    smax = fmax(smax, lb[idxFiniteLB] - TrialState->xstarsqp[idxFiniteLB]);
  }

  for (k = 0; k < mUB; k++) {
    idxFiniteLB = WorkingSet->indexUB[k] - 1;
    smax = fmax(smax, TrialState->xstarsqp[idxFiniteLB] - ub[idxFiniteLB]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  }

  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  dxTooSmall = true;
  smax = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= WorkingSet->nVar - 1)) {
    dxTooSmall = ((!rtIsInf(TrialState->gradLag[k])) && (!rtIsNaN
      (TrialState->gradLag[k])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag[k]));
      k++;
    }
  }

  Flags->gradOK = dxTooSmall;
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
    MeritFunction->firstOrderOpt = fmax(smax, MeritFunction->nlpComplError);
    if (TrialState->sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      b_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                       TrialState->grad, WorkingSet->indexFixed,
                       WorkingSet->sizes[0], WorkingSet->indexLB,
                       WorkingSet->sizes[3], WorkingSet->indexUB,
                       WorkingSet->sizes[4], TrialState->lambdaStopTestPrev);
      s = 0.0;
      k = 0;
      while ((k <= WorkingSet->nVar - 1) && ((!rtIsInf
               (memspace->workspace_double[k])) && (!rtIsNaN
               (memspace->workspace_double[k])))) {
        s = fmax(s, fabs(memspace->workspace_double[k]));
        k++;
      }

      nlpComplErrorTmp = computeComplError(TrialState->xstarsqp,
        WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
        WorkingSet->sizes[4], ub, TrialState->lambdaStopTestPrev,
        WorkingSet->sizes[0] + 1);
      d = fmax(s, nlpComplErrorTmp);
      if (d < fmax(smax, MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
        if (mLambda >= 0) {
          memcpy(&TrialState->lambdaStopTest[0], &TrialState->
                 lambdaStopTestPrev[0], (mLambda + 1) * sizeof(double));
        }
      } else if (mLambda >= 0) {
        memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
               (mLambda + 1) * sizeof(double));
      }
    } else if (mLambda >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (mLambda + 1) * sizeof(double));
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
        bool guard1 = false;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          k = 0;
          exitg1 = false;
          while ((!exitg1) && (k <= nVar - 1)) {
            if (1.0E-9 * fmax(1.0, fabs(TrialState->xstarsqp[k])) <= fabs
                (TrialState->delta_x[k])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              k++;
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
              idx_max = WorkingSet->nActiveConstr - 1;
              if (WorkingSet->nActiveConstr > 0) {
                for (k = 0; k <= idx_max; k++) {
                  TrialState->lambda[k] = 0.0;
                  idxFiniteLB = k << 4;
                  mLB = 31 * k;
                  for (mUB = 0; mUB < nVar; mUB++) {
                    QRManager->QR[mLB + mUB] = WorkingSet->ATwset[idxFiniteLB +
                      mUB];
                  }
                }

                QRManager->usedPivoting = true;
                QRManager->mrows = WorkingSet->nVar;
                QRManager->ncols = WorkingSet->nActiveConstr;
                idx_max = WorkingSet->nVar;
                mUB = WorkingSet->nActiveConstr;
                if (idx_max <= mUB) {
                  mUB = idx_max;
                }

                QRManager->minRowCol = mUB;
                xgeqp3(QRManager->QR, WorkingSet->nVar,
                       WorkingSet->nActiveConstr, QRManager->jpvt,
                       QRManager->tau);
                computeQ_(QRManager, WorkingSet->nVar);
                idx_max = WorkingSet->nVar;
                idxFiniteLB = WorkingSet->nActiveConstr;
                if (idx_max >= idxFiniteLB) {
                  idxFiniteLB = idx_max;
                }

                smax = fabs(QRManager->QR[0]) * fmin(1.4901161193847656E-8,
                  (double)idxFiniteLB * 2.2204460492503131E-16);
                mLB = 0;
                idx_max = 0;
                while ((mLB < mUB) && (fabs(QRManager->QR[idx_max]) > smax)) {
                  mLB++;
                  idx_max += 32;
                }

                xgemv(WorkingSet->nVar, WorkingSet->nVar, QRManager->Q,
                      TrialState->grad, memspace->workspace_double);
                if (mLB != 0) {
                  for (nVar = mLB; nVar >= 1; nVar--) {
                    idx_max = (nVar + (nVar - 1) * 31) - 1;
                    memspace->workspace_double[nVar - 1] /= QRManager->
                      QR[idx_max];
                    for (k = 0; k <= nVar - 2; k++) {
                      idxFiniteLB = (nVar - k) - 2;
                      memspace->workspace_double[idxFiniteLB] -=
                        memspace->workspace_double[nVar - 1] * QRManager->QR
                        [(idx_max - k) - 1];
                    }
                  }
                }

                idx_max = WorkingSet->nActiveConstr;
                if (idx_max <= mUB) {
                  mUB = idx_max;
                }

                for (k = 0; k < mUB; k++) {
                  TrialState->lambda[QRManager->jpvt[k] - 1] =
                    memspace->workspace_double[k];
                }

                idx_max = WorkingSet->sizes[0] + 1;
                for (k = idx_max; k <= mFixed; k++) {
                  TrialState->lambda[k - 1] = -TrialState->lambda[k - 1];
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
                smax = 0.0;
                k = 0;
                while ((k <= WorkingSet->nVar - 1) && ((!rtIsInf
                         (memspace->workspace_double[k])) && (!rtIsNaN
                         (memspace->workspace_double[k])))) {
                  smax = fmax(smax, fabs(memspace->workspace_double[k]));
                  k++;
                }

                s = computeComplError(TrialState->xstarsqp, WorkingSet->indexLB,
                                      WorkingSet->sizes[3], lb,
                                      WorkingSet->indexUB, WorkingSet->sizes[4],
                                      ub, TrialState->lambda, WorkingSet->sizes
                                      [0] + 1);
                if ((smax <= 1.0E-9 * optimRelativeFactor) && (s <= 1.0E-9 *
                     optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = smax;
                  MeritFunction->nlpComplError = s;
                  MeritFunction->firstOrderOpt = fmax(smax, s);
                  if (mLambda >= 0) {
                    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0],
                           (mLambda + 1) * sizeof(double));
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
          if (TrialState->sqpIterations >= 40) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 200) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

/*
 * Arguments    : double *outTime_tv_sec
 *                double *outTime_tv_nsec
 * Return Type  : void
 */
static void b_timeKeeper(double *outTime_tv_sec, double *outTime_tv_nsec)
{
  *outTime_tv_sec = savedTime.tv_sec;
  *outTime_tv_nsec = savedTime.tv_nsec;
}

/*
 * Arguments    : int n
 *                double a
 *                const double x[72]
 *                int ix0
 *                double y[12]
 *                int iy0
 * Return Type  : void
 */
static void b_xaxpy(int n, double a, const double x[72], int ix0, double y[12],
                    int iy0)
{
  int k;
  if (!(a == 0.0)) {
    int i;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[36]
 *                int ix0
 *                const double y[36]
 *                int iy0
 * Return Type  : double
 */
static double b_xdotc(int n, const double x[36], int ix0, const double y[36],
                      int iy0)
{
  double d;
  int k;
  d = 0.0;
  for (k = 0; k < n; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
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
        memset(&C[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
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
      scale = 3.3121686421112381E-170;
      for (k = 0; k < n; k++) {
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
 * Arguments    : double x[72]
 *                int ix0
 *                int iy0
 *                double c
 *                double s
 * Return Type  : void
 */
static void b_xrot(double x[72], int ix0, int iy0, double c, double s)
{
  int k;
  for (k = 0; k < 12; k++) {
    double b_temp_tmp;
    double d_temp_tmp;
    int c_temp_tmp;
    int temp_tmp;
    temp_tmp = (iy0 + k) - 1;
    b_temp_tmp = x[temp_tmp];
    c_temp_tmp = (ix0 + k) - 1;
    d_temp_tmp = x[c_temp_tmp];
    x[temp_tmp] = c * b_temp_tmp - s * d_temp_tmp;
    x[c_temp_tmp] = c * d_temp_tmp + s * b_temp_tmp;
  }
}

/*
 * Arguments    : double x[72]
 *                int ix0
 *                int iy0
 * Return Type  : void
 */
static void b_xswap(double x[72], int ix0, int iy0)
{
  int k;
  for (k = 0; k < 12; k++) {
    double temp;
    int i;
    int temp_tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    i = (iy0 + k) - 1;
    x[temp_tmp] = x[i];
    x[i] = temp;
  }
}

/*
 * Testing parameters:
 *  (0.5e-5, 2e-7, 2.4, 0.15, 0.15, 0.2, 0.25, 0.25, 0.35, 0.35, 0, ...
 *  0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...
 *  1, 0, 0, 0, ...
 *  0, 1.5, 100, -15, 100, -15, 0.5, 0, ...
 *  0.01, 0.01, 0.05, 0.1, 0.1, 0.1, ...
 *  1000, 100, 25, -130, 45, -45, 80, 15, 40, 25, -25, [0 0 -5 0 0 0]', 0, 0, 0, 0.1, 3, 0.3, 0.15,...
 *  -0.1, 0.1, 1.15, 0, 0.3, 0.3, 0, 15, -5, 0, 1e-5, ...
 *  100, 0, 0, 0, 0, 0, 1, 7)
 *  or
 *  (0.5e-5, 2e-7, 2.4, 0.15, 0.15, 0.2, 0.25, 0.25, 0.35, 0.35, 0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1.5, 100, -15, 100, -15, 0.5, 0, 0.01, 0.01, 0.05, 0.1, 0.1, 0.1, 1000, 100, 25, 130, 45, -45, 80, 15, 40, 25, -25, [0 0 -5 0 0 0]', 0, 0, 0, 0.1, 3, 0.3, 0.15, -0.1, 0.1, 1.15, 0, 0.3, 0.3, 0, 15, -5, 0, 1e-5, 100, 0, 0, 0, 0, 0, 1, 7)
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
 *                const double dv[6]
 *                double p
 *                double q
 *                double r
 *                double Cm_zero
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
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
 *                double verbose
 *                double u_out[15]
 *                double residuals[6]
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluation
 *                double *exitflag
 * Return Type  : void
 */
static void c_Nonlinear_controller_fcn_cont(double K_p_T, double K_p_M, double m,
  double I_xx, double I_yy, double I_zz, double l_1, double l_2, double l_3,
  double l_4, double l_z, double Phi, double Theta, double Omega_1, double
  Omega_2, double Omega_3, double Omega_4, double b_1, double b_2, double b_3,
  double b_4, double g_1, double g_2, double g_3, double g_4, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_act_theta_const, double W_act_theta_speed,
  double W_act_phi_const, double W_act_phi_speed, double W_dv_1, double W_dv_2,
  double W_dv_3, double W_dv_4, double W_dv_5, double W_dv_6, double max_omega,
  double min_omega, double max_b, double min_b, double max_g, double min_g,
  double max_theta, double min_theta, double max_phi, const double dv[6], double
  p, double q, double r, double Cm_zero, double Cl_alpha, double Cd_zero, double
  K_Cd, double Cm_alpha, double rho, double V, double S, double wing_chord,
  double flight_path_angle, double max_alpha, double min_alpha, double Beta,
  double gamma_quadratic_du, double desired_motor_value, double desired_el_value,
  double desired_az_value, double desired_theta_value, double desired_phi_value,
  double verbose, double u_out[15], double residuals[6], double *elapsed_time,
  double *N_iterations, double *N_evaluation, double *exitflag)
{
  b_captured_var dv_global;
  captured_var CL_aileron;
  captured_var W_act_ailerons;
  captured_var W_act_motor;
  captured_var W_act_phi;
  captured_var W_act_theta;
  captured_var W_act_tilt_az;
  captured_var W_act_tilt_el;
  captured_var b_Beta;
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
  captured_var b_S;
  captured_var b_V;
  captured_var b_W_dv_1;
  captured_var b_W_dv_2;
  captured_var b_W_dv_3;
  captured_var b_W_dv_4;
  captured_var b_W_dv_5;
  captured_var b_W_dv_6;
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
  captured_var desired_ailerons_value;
  captured_var gain_ailerons;
  captured_var gain_az;
  captured_var gain_el;
  captured_var gain_motor;
  captured_var gain_phi;
  captured_var gain_theta;
  d_struct_T expl_temp;
  double actual_u[15];
  double u_max[15];
  double u_max_scaled[15];
  double u_min[15];
  double c_expl_temp;
  double max_theta_protection;
  double min_theta_protection;
  double t;
  double y;
  int i;
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
  CL_aileron.contents = 0.1;
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
  desired_ailerons_value.contents = 0.0;

  /*  Create variables necessary for the optimization */
  if (b_V.contents > 20.0) {
    y = (max_alpha + b_flight_path_angle.contents) * 180.0 / 3.1415926535897931;
    max_theta_protection = fmin(max_theta, y);
    y = (min_alpha + b_flight_path_angle.contents) * 180.0 / 3.1415926535897931;
    min_theta_protection = fmax(min_theta, y);
  } else {
    max_theta_protection = max_theta;
    min_theta_protection = min_theta;
  }

  if (b_desired_motor_value.contents < min_omega) {
    b_desired_motor_value.contents = (((Omega_1 + Omega_2) + Omega_3) + Omega_4)
      / 4.0;
  }

  gain_motor.contents = max_omega / 2.0;
  gain_el.contents = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az.contents = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
  gain_theta.contents = (max_theta_protection - min_theta_protection) *
    3.1415926535897931 / 180.0 / 2.0;
  gain_phi.contents = max_phi * 3.1415926535897931 / 180.0;
  gain_ailerons.contents = 0.017453292519943295;
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
  actual_u[12] = Theta;
  actual_u[13] = Phi;
  actual_u[14] = 0.0;

  /* Build the max and minimum actuator array:  */
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
  u_max[14] = 1.0;
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
  u_min[14] = -1.0;
  for (i = 0; i < 11; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }

  memcpy(&u_max_scaled[0], &u_max[0], 15U * sizeof(double));
  u_max_scaled[0] = u_max[0] / gain_motor.contents;
  u_max_scaled[1] = u_max[1] / gain_motor.contents;
  u_max_scaled[2] = u_max[2] / gain_motor.contents;
  u_max_scaled[3] = u_max[3] / gain_motor.contents;
  min_theta_protection = u_min[0] / gain_motor.contents;
  max_theta_protection = u_min[1] / gain_motor.contents;
  t = u_min[2] / gain_motor.contents;
  y = u_min[3] / gain_motor.contents;
  u_min[0] = min_theta_protection;
  u_min[1] = max_theta_protection;
  u_min[2] = t;
  u_min[3] = y;
  min_theta_protection = u_max_scaled[4] / gain_el.contents;
  max_theta_protection = u_max_scaled[5] / gain_el.contents;
  t = u_max_scaled[6] / gain_el.contents;
  y = u_max_scaled[7] / gain_el.contents;
  u_max_scaled[4] = min_theta_protection;
  u_max_scaled[5] = max_theta_protection;
  u_max_scaled[6] = t;
  u_max_scaled[7] = y;
  min_theta_protection = u_min[4] / gain_el.contents;
  max_theta_protection = u_min[5] / gain_el.contents;
  t = u_min[6] / gain_el.contents;
  y = u_min[7] / gain_el.contents;
  u_min[4] = min_theta_protection;
  u_min[5] = max_theta_protection;
  u_min[6] = t;
  u_min[7] = y;
  min_theta_protection = u_max_scaled[8] / gain_az.contents;
  max_theta_protection = u_max_scaled[9] / gain_az.contents;
  t = u_max_scaled[10] / gain_az.contents;
  y = u_max_scaled[11] / gain_az.contents;
  u_max_scaled[8] = min_theta_protection;
  u_max_scaled[9] = max_theta_protection;
  u_max_scaled[10] = t;
  u_max_scaled[11] = y;
  min_theta_protection = u_min[8] / gain_az.contents;
  max_theta_protection = u_min[9] / gain_az.contents;
  t = u_min[10] / gain_az.contents;
  y = u_min[11] / gain_az.contents;
  u_min[8] = min_theta_protection;
  u_min[9] = max_theta_protection;
  u_min[10] = t;
  u_min[11] = y;
  u_max_scaled[12] /= gain_theta.contents;
  u_min[12] /= gain_theta.contents;
  u_max_scaled[13] /= gain_phi.contents;
  u_min[13] /= gain_phi.contents;
  u_max_scaled[14] /= gain_ailerons.contents;
  u_min[14] /= gain_ailerons.contents;
  memcpy(&u_max[0], &actual_u[0], 15U * sizeof(double));
  u_max[0] = Omega_1 / gain_motor.contents;
  u_max[1] = Omega_2 / gain_motor.contents;
  u_max[2] = Omega_3 / gain_motor.contents;
  u_max[3] = Omega_4 / gain_motor.contents;
  min_theta_protection = u_max[4] / gain_el.contents;
  max_theta_protection = u_max[5] / gain_el.contents;
  t = u_max[6] / gain_el.contents;
  y = u_max[7] / gain_el.contents;
  u_max[4] = min_theta_protection;
  u_max[5] = max_theta_protection;
  u_max[6] = t;
  u_max[7] = y;
  min_theta_protection = u_max[8] / gain_az.contents;
  max_theta_protection = u_max[9] / gain_az.contents;
  t = u_max[10] / gain_az.contents;
  y = u_max[11] / gain_az.contents;
  u_max[8] = min_theta_protection;
  u_max[9] = max_theta_protection;
  u_max[10] = t;
  u_max[11] = y;
  u_max[12] /= gain_theta.contents;
  u_max[13] /= gain_phi.contents;
  u_max[14] /= gain_ailerons.contents;

  /*  Apply Nonlinear optimization algorithm: */
  c_compute_acc_nonlinear_control(actual_u, b_p.contents, b_q.contents,
    b_r.contents, b_K_p_T.contents, b_K_p_M.contents, b_m.contents,
    b_I_xx.contents, b_I_yy.contents, b_I_zz.contents, b_l_1.contents,
    b_l_2.contents, b_l_3.contents, b_l_4.contents, b_l_z.contents,
    b_Cl_alpha.contents, b_Cd_zero.contents, b_K_Cd.contents,
    b_Cm_alpha.contents, b_Cm_zero.contents, CL_aileron.contents, b_rho.contents,
    b_V.contents, b_S.contents, b_wing_chord.contents,
    b_flight_path_angle.contents, b_Beta.contents, dv_global.contents);
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] += dv[i];
  }

  char b_expl_temp[3];

  /* Compute weights for actuators and make sure they are always positive */
  y = W_act_motor_const + W_act_motor_speed * b_V.contents;
  W_act_motor.contents = fmax(0.0, y);
  y = W_act_tilt_el_const + W_act_tilt_el_speed * b_V.contents;
  W_act_tilt_el.contents = fmax(0.0, y);
  y = W_act_tilt_az_const + W_act_tilt_az_speed * b_V.contents;
  W_act_tilt_az.contents = fmax(0.0, y);
  y = W_act_theta_const + W_act_theta_speed * b_V.contents;
  W_act_theta.contents = fmax(0.0, y);
  y = W_act_phi_const + W_act_phi_speed * b_V.contents;
  W_act_phi.contents = fmax(0.0, y);
  y = 0.0 * b_V.contents + 100.0;
  W_act_ailerons.contents = fmax(0.0, y);

  /* Default values for the optimizer: */
  tic();
  expl_temp.desired_ailerons_value = &desired_ailerons_value;
  expl_temp.W_act_ailerons = &W_act_ailerons;
  expl_temp.desired_phi_value = &b_desired_phi_value;
  expl_temp.W_act_phi = &W_act_phi;
  expl_temp.desired_theta_value = &b_desired_theta_value;
  expl_temp.W_act_theta = &W_act_theta;
  expl_temp.desired_az_value = &b_desired_az_value;
  expl_temp.W_act_tilt_az = &W_act_tilt_az;
  expl_temp.desired_el_value = &b_desired_el_value;
  expl_temp.W_act_tilt_el = &W_act_tilt_el;
  expl_temp.W_dv_2 = &b_W_dv_2;
  expl_temp.W_dv_1 = &b_W_dv_1;
  expl_temp.W_dv_3 = &b_W_dv_3;
  expl_temp.W_dv_5 = &b_W_dv_5;
  expl_temp.desired_motor_value = &b_desired_motor_value;
  expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  expl_temp.W_act_motor = &W_act_motor;
  expl_temp.W_dv_6 = &b_W_dv_6;
  expl_temp.W_dv_4 = &b_W_dv_4;
  expl_temp.gain_ailerons = &gain_ailerons;
  expl_temp.CL_aileron = &CL_aileron;
  expl_temp.l_2 = &b_l_2;
  expl_temp.l_1 = &b_l_1;
  expl_temp.q = &b_q;
  expl_temp.Cm_alpha = &b_Cm_alpha;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_4 = &b_l_4;
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.Cm_zero = &b_Cm_zero;
  expl_temp.K_p_M = &b_K_p_M;
  expl_temp.l_z = &b_l_z;
  expl_temp.I_yy = &b_I_yy;
  expl_temp.I_xx = &b_I_xx;
  expl_temp.r = &b_r;
  expl_temp.p = &b_p;
  expl_temp.I_zz = &b_I_zz;
  expl_temp.m = &b_m;
  expl_temp.gain_az = &gain_az;
  expl_temp.gain_el = &gain_el;
  expl_temp.gain_motor = &gain_motor;
  expl_temp.K_p_T = &b_K_p_T;
  expl_temp.gain_phi = &gain_phi;
  expl_temp.Cd_zero = &b_Cd_zero;
  expl_temp.K_Cd = &b_K_Cd;
  expl_temp.Beta = &b_Beta;
  expl_temp.flight_path_angle = &b_flight_path_angle;
  expl_temp.rho = &b_rho;
  expl_temp.V = &b_V;
  expl_temp.S = &b_S;
  expl_temp.Cl_alpha = &b_Cl_alpha;
  expl_temp.gain_theta = &gain_theta;
  expl_temp.dv_global = &dv_global;
  fmincon(&expl_temp, u_max, u_min, u_max_scaled, u_out, &min_theta_protection,
          exitflag, N_iterations, N_evaluation, b_expl_temp,
          &max_theta_protection, &t, &y, &c_expl_temp);
  *elapsed_time = toc();
  y = gain_motor.contents;
  u_out[0] *= y;
  u_out[1] *= y;
  u_out[2] *= y;
  u_out[3] *= y;
  y = gain_el.contents;
  u_out[4] *= y;
  u_out[5] *= y;
  u_out[6] *= y;
  u_out[7] *= y;
  y = gain_az.contents;
  u_out[8] *= y;
  u_out[9] *= y;
  u_out[10] *= y;
  u_out[11] *= y;
  u_out[12] *= gain_theta.contents;
  u_out[13] *= gain_phi.contents;
  u_out[14] *= gain_ailerons.contents;
  c_compute_acc_nonlinear_control(u_out, b_p.contents, b_q.contents,
    b_r.contents, b_K_p_T.contents, b_K_p_M.contents, b_m.contents,
    b_I_xx.contents, b_I_yy.contents, b_I_zz.contents, b_l_1.contents,
    b_l_2.contents, b_l_3.contents, b_l_4.contents, b_l_z.contents,
    b_Cl_alpha.contents, b_Cd_zero.contents, b_K_Cd.contents,
    b_Cm_alpha.contents, b_Cm_zero.contents, CL_aileron.contents, b_rho.contents,
    b_V.contents, b_S.contents, b_wing_chord.contents,
    b_flight_path_angle.contents, b_Beta.contents, residuals);
  for (i = 0; i < 6; i++) {
    residuals[i] = dv_global.contents[i] - residuals[i];
  }

  /*  Print infos */
  if (verbose != 0.0) {
    printf("\n Solution: \n");
    fflush(stdout);
    printf("Motors [rad/s] =  ");
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
    printf("Elevator angles [deg] =  ");
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
    printf("Azimuth angles [deg] =  ");
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
    printf("Theta [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[12] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("Phi [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[13] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("Ailerons deflection [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[14] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time = %f \n", *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations = %f \n", *N_iterations);
    fflush(stdout);
    printf("\n Number of evaluation = %f \n", *N_evaluation);
    fflush(stdout);
    printf("\n Residuals = ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", residuals[i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    y = 0.0;
    min_theta_protection = 3.3121686421112381E-170;
    for (i = 0; i < 6; i++) {
      max_theta_protection = fabs(residuals[i]);
      if (max_theta_protection > min_theta_protection) {
        t = min_theta_protection / max_theta_protection;
        y = y * t * t + 1.0;
        min_theta_protection = max_theta_protection;
      } else {
        t = max_theta_protection / min_theta_protection;
        y += t * t;
      }
    }

    y = min_theta_protection * sqrt(y);
    printf("\n Residual norm = %f \n", y);
    fflush(stdout);
    memcpy(&u_max[0], &u_out[0], 15U * sizeof(double));
    u_max[0] = u_out[0] / gain_motor.contents;
    u_max[1] = u_out[1] / gain_motor.contents;
    u_max[2] = u_out[2] / gain_motor.contents;
    u_max[3] = u_out[3] / gain_motor.contents;
    min_theta_protection = u_max[4] / gain_el.contents;
    max_theta_protection = u_max[5] / gain_el.contents;
    t = u_max[6] / gain_el.contents;
    y = u_max[7] / gain_el.contents;
    u_max[4] = min_theta_protection;
    u_max[5] = max_theta_protection;
    u_max[6] = t;
    u_max[7] = y;
    min_theta_protection = u_max[8] / gain_az.contents;
    max_theta_protection = u_max[9] / gain_az.contents;
    t = u_max[10] / gain_az.contents;
    y = u_max[11] / gain_az.contents;
    u_max[8] = min_theta_protection;
    u_max[9] = max_theta_protection;
    u_max[10] = t;
    u_max[11] = y;
    u_max[12] /= gain_theta.contents;
    u_max[13] /= gain_phi.contents;
    u_max[14] /= gain_ailerons.contents;
    printf("\n Solution scaled norm = %f \n", b_norm(u_max));
    fflush(stdout);
    printf("\n Exit flag optimizer = %f \n", *exitflag);
    fflush(stdout);
  }
}

/*
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
 *                double Psi
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
 *                double W_act_motor_const
 *                double W_act_motor_speed
 *                double W_act_tilt_el_const
 *                double W_act_tilt_el_speed
 *                double W_act_tilt_az_const
 *                double W_act_tilt_az_speed
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
 *                double dv[6]
 *                double p
 *                double q
 *                double r
 *                double Cm_zero
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double max_alpha
 *                double Beta
 *                double gammasq
 *                double desired_motor_value
 *                double desired_el_value
 *                double desired_az_value
 *                double desired_theta_value
 *                double desired_phi_value
 *                double u_out[14]
 *                double residuals[6]
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluation
 *                double *exitflag
 * Return Type  : void
 */
static void c_WLS_controller_fcn_earth_rf_j(double K_p_T, double K_p_M, double m,
  double I_xx, double I_yy, double I_zz, double l_1, double l_2, double l_3,
  double l_4, double l_z, double Phi, double Theta, double Psi, double Omega_1,
  double Omega_2, double Omega_3, double Omega_4, double b_1, double b_2, double
  b_3, double b_4, double g_1, double g_2, double g_3, double g_4, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_dv_1, double W_dv_2, double W_dv_3, double
  W_dv_4, double W_dv_5, double W_dv_6, double max_omega, double min_omega,
  double max_b, double min_b, double max_g, double min_g, double dv[6], double p,
  double q, double r, double Cm_zero, double Cl_alpha, double Cd_zero, double
  K_Cd, double Cm_alpha, double rho, double V, double S, double wing_chord,
  double flight_path_angle, double max_alpha, double Beta, double gammasq,
  double desired_motor_value, double desired_el_value, double desired_az_value,
  double desired_theta_value, double desired_phi_value, double u_out[14], double
  residuals[6], double *elapsed_time, double *N_iterations, double *N_evaluation,
  double *exitflag)
{
  double A[216];
  double Wu[144];
  double B_eval[72];
  double gam_sq[72];
  double Wv[36];
  double b_A[18];
  double d[18];
  double actual_u[12];
  double desired_u_scaled[12];
  double max_u_scaled[12];
  double min_u_scaled[12];
  double u_max[12];
  double u_max_constrain[12];
  double u_min[12];
  double u_min_constrain[12];
  double v[6];
  double B_eval_tmp;
  double B_eval_tmp_tmp;
  double W_act_az;
  double W_act_el;
  double W_act_motor;
  double ab_B_eval_tmp;
  double ac_B_eval_tmp;
  double b_B_eval_tmp;
  double bb_B_eval_tmp;
  double c_B_eval_tmp;
  double cb_B_eval_tmp;
  double d_B_eval_tmp;
  double db_B_eval_tmp;
  double e_B_eval_tmp;
  double eb_B_eval_tmp;
  double f_B_eval_tmp;
  double fb_B_eval_tmp;
  double g_B_eval_tmp;
  double gain_az;
  double gain_el;
  double gain_motor;
  double gb_B_eval_tmp;
  double h_B_eval_tmp;
  double hb_B_eval_tmp;
  double i_B_eval_tmp;
  double ib_B_eval_tmp;
  double j_B_eval_tmp;
  double jb_B_eval_tmp;
  double k_B_eval_tmp;
  double kb_B_eval_tmp;
  double l_B_eval_tmp;
  double lb_B_eval_tmp;
  double m_B_eval_tmp;
  double mb_B_eval_tmp;
  double n_B_eval_tmp;
  double nb_B_eval_tmp;
  double o_B_eval_tmp;
  double ob_B_eval_tmp;
  double p_B_eval_tmp;
  double pb_B_eval_tmp;
  double q_B_eval_tmp;
  double qb_B_eval_tmp;
  double r_B_eval_tmp;
  double rb_B_eval_tmp;
  double s_B_eval_tmp;
  double sb_B_eval_tmp;
  double t_B_eval_tmp;
  double tb_B_eval_tmp;
  double u_B_eval_tmp;
  double ub_B_eval_tmp;
  double v_B_eval_tmp;
  double vb_B_eval_tmp;
  double w_B_eval_tmp;
  double wb_B_eval_tmp;
  double x_B_eval_tmp;
  double xb_B_eval_tmp;
  double y_B_eval_tmp;
  double yb_B_eval_tmp;
  int A_free_size[2];
  int aoffset;
  int b_N_iterations;
  int b_i;
  int b_trueCount;
  int i;
  int iter;
  int k;
  signed char b_tmp_data[12];
  signed char c_tmp_data[12];
  signed char d_tmp_data[12];
  signed char tmp_data[12];
  bool i_free[12];
  bool exitg1;
  tic();

  /*  Assign geometrical and create variables */
  gain_motor = max_omega / 2.0;
  gain_el = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;

  /*  Identify and evaluate the effectiveness matrix [6,12] */
  B_eval_tmp = cos(Psi);
  b_B_eval_tmp = cos(Phi);
  c_B_eval_tmp = sin(Psi);
  d_B_eval_tmp = sin(Phi);
  e_B_eval_tmp = sin(Theta);
  f_B_eval_tmp = cos(Theta);
  g_B_eval_tmp = cos(g_1);
  h_B_eval_tmp = sin(b_1);
  i_B_eval_tmp = cos(b_1);
  j_B_eval_tmp = sin(g_1);
  k_B_eval_tmp = cos(g_2);
  l_B_eval_tmp = sin(b_2);
  m_B_eval_tmp = cos(b_2);
  n_B_eval_tmp = sin(g_2);
  o_B_eval_tmp = cos(g_3);
  p_B_eval_tmp = sin(b_3);
  q_B_eval_tmp = cos(b_3);
  r_B_eval_tmp = sin(g_3);
  s_B_eval_tmp = cos(g_4);
  t_B_eval_tmp = sin(b_4);
  u_B_eval_tmp = cos(b_4);
  v_B_eval_tmp = sin(g_4);
  w_B_eval_tmp = 2.0 * K_p_T * Omega_1;
  B_eval[0] = -((w_B_eval_tmp * B_eval_tmp * f_B_eval_tmp * h_B_eval_tmp +
                 w_B_eval_tmp * i_B_eval_tmp * g_B_eval_tmp * (d_B_eval_tmp *
    c_B_eval_tmp + b_B_eval_tmp * B_eval_tmp * e_B_eval_tmp)) + 2.0 * K_p_T
                * Omega_1 * cos(b_1) * j_B_eval_tmp * (b_B_eval_tmp *
    c_B_eval_tmp - B_eval_tmp * d_B_eval_tmp * e_B_eval_tmp)) / m;
  x_B_eval_tmp = 2.0 * K_p_T * Omega_2;
  y_B_eval_tmp = sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta);
  ab_B_eval_tmp = cos(Phi) * sin(Psi) - cos(Psi) * sin(Phi) * sin(Theta);
  B_eval[6] = -((x_B_eval_tmp * B_eval_tmp * f_B_eval_tmp * l_B_eval_tmp +
                 x_B_eval_tmp * m_B_eval_tmp * k_B_eval_tmp * y_B_eval_tmp) +
                2.0 * K_p_T * Omega_2 * cos(b_2) * n_B_eval_tmp * ab_B_eval_tmp)
    / m;
  bb_B_eval_tmp = 2.0 * K_p_T * Omega_3;
  B_eval[12] = -((bb_B_eval_tmp * B_eval_tmp * f_B_eval_tmp * p_B_eval_tmp +
                  bb_B_eval_tmp * q_B_eval_tmp * o_B_eval_tmp * y_B_eval_tmp) +
                 2.0 * K_p_T * Omega_3 * cos(b_3) * r_B_eval_tmp * ab_B_eval_tmp)
    / m;
  cb_B_eval_tmp = 2.0 * K_p_T * Omega_4;
  B_eval[18] = -((cb_B_eval_tmp * B_eval_tmp * f_B_eval_tmp * t_B_eval_tmp +
                  cb_B_eval_tmp * u_B_eval_tmp * s_B_eval_tmp * y_B_eval_tmp) +
                 2.0 * K_p_T * Omega_4 * cos(b_4) * v_B_eval_tmp * ab_B_eval_tmp)
    / m;
  W_act_motor = Omega_1 * Omega_1;
  db_B_eval_tmp = K_p_T * W_act_motor;
  eb_B_eval_tmp = db_B_eval_tmp * g_B_eval_tmp * h_B_eval_tmp;
  fb_B_eval_tmp = db_B_eval_tmp * h_B_eval_tmp * j_B_eval_tmp;
  B_eval[24] = ((eb_B_eval_tmp * y_B_eval_tmp - db_B_eval_tmp * B_eval_tmp *
                 f_B_eval_tmp * i_B_eval_tmp) + fb_B_eval_tmp * ab_B_eval_tmp) /
    m;
  W_act_el = Omega_2 * Omega_2;
  gb_B_eval_tmp = K_p_T * W_act_el;
  hb_B_eval_tmp = gb_B_eval_tmp * k_B_eval_tmp * l_B_eval_tmp;
  ib_B_eval_tmp = gb_B_eval_tmp * l_B_eval_tmp * n_B_eval_tmp;
  B_eval[30] = ((hb_B_eval_tmp * y_B_eval_tmp - gb_B_eval_tmp * B_eval_tmp *
                 f_B_eval_tmp * m_B_eval_tmp) + ib_B_eval_tmp * ab_B_eval_tmp) /
    m;
  W_act_az = Omega_3 * Omega_3;
  jb_B_eval_tmp = K_p_T * W_act_az;
  kb_B_eval_tmp = jb_B_eval_tmp * o_B_eval_tmp * p_B_eval_tmp;
  lb_B_eval_tmp = jb_B_eval_tmp * p_B_eval_tmp * r_B_eval_tmp;
  B_eval[36] = ((kb_B_eval_tmp * y_B_eval_tmp - jb_B_eval_tmp * B_eval_tmp *
                 f_B_eval_tmp * q_B_eval_tmp) + lb_B_eval_tmp * ab_B_eval_tmp) /
    m;
  B_eval_tmp_tmp = Omega_4 * Omega_4;
  mb_B_eval_tmp = K_p_T * B_eval_tmp_tmp;
  nb_B_eval_tmp = mb_B_eval_tmp * s_B_eval_tmp * t_B_eval_tmp;
  ob_B_eval_tmp = mb_B_eval_tmp * t_B_eval_tmp * v_B_eval_tmp;
  B_eval[42] = ((nb_B_eval_tmp * y_B_eval_tmp - mb_B_eval_tmp * B_eval_tmp *
                 f_B_eval_tmp * u_B_eval_tmp) + ob_B_eval_tmp * ab_B_eval_tmp) /
    m;
  B_eval_tmp = db_B_eval_tmp * i_B_eval_tmp;
  pb_B_eval_tmp = B_eval_tmp * g_B_eval_tmp;
  qb_B_eval_tmp = B_eval_tmp * j_B_eval_tmp;
  B_eval[48] = -(pb_B_eval_tmp * ab_B_eval_tmp - qb_B_eval_tmp * y_B_eval_tmp) /
    m;
  rb_B_eval_tmp = gb_B_eval_tmp * m_B_eval_tmp;
  sb_B_eval_tmp = rb_B_eval_tmp * k_B_eval_tmp;
  tb_B_eval_tmp = rb_B_eval_tmp * n_B_eval_tmp;
  B_eval[54] = -(sb_B_eval_tmp * ab_B_eval_tmp - tb_B_eval_tmp * y_B_eval_tmp) /
    m;
  ub_B_eval_tmp = jb_B_eval_tmp * q_B_eval_tmp;
  vb_B_eval_tmp = ub_B_eval_tmp * o_B_eval_tmp;
  wb_B_eval_tmp = ub_B_eval_tmp * r_B_eval_tmp;
  B_eval[60] = -(vb_B_eval_tmp * ab_B_eval_tmp - wb_B_eval_tmp * y_B_eval_tmp) /
    m;
  xb_B_eval_tmp = mb_B_eval_tmp * u_B_eval_tmp;
  yb_B_eval_tmp = xb_B_eval_tmp * s_B_eval_tmp;
  ac_B_eval_tmp = xb_B_eval_tmp * v_B_eval_tmp;
  B_eval[66] = -(yb_B_eval_tmp * ab_B_eval_tmp - ac_B_eval_tmp * y_B_eval_tmp) /
    m;
  B_eval[1] = ((2.0 * K_p_T * Omega_1 * cos(b_1) * cos(g_1) * (cos(Psi) * sin
    (Phi) - cos(Phi) * sin(Psi) * e_B_eval_tmp) - w_B_eval_tmp * f_B_eval_tmp *
                c_B_eval_tmp * h_B_eval_tmp) + 2.0 * K_p_T * Omega_1 * cos(b_1) *
               sin(g_1) * (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) *
    e_B_eval_tmp)) / m;
  e_B_eval_tmp = cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta);
  y_B_eval_tmp = cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta);
  B_eval[7] = ((2.0 * K_p_T * Omega_2 * cos(b_2) * cos(g_2) * e_B_eval_tmp -
                x_B_eval_tmp * f_B_eval_tmp * c_B_eval_tmp * l_B_eval_tmp) + 2.0
               * K_p_T * Omega_2 * cos(b_2) * sin(g_2) * y_B_eval_tmp) / m;
  B_eval[13] = ((2.0 * K_p_T * Omega_3 * cos(b_3) * cos(g_3) * e_B_eval_tmp -
                 bb_B_eval_tmp * f_B_eval_tmp * c_B_eval_tmp * p_B_eval_tmp) +
                2.0 * K_p_T * Omega_3 * cos(b_3) * sin(g_3) * y_B_eval_tmp) / m;
  B_eval[19] = ((2.0 * K_p_T * Omega_4 * cos(b_4) * cos(g_4) * e_B_eval_tmp -
                 cb_B_eval_tmp * f_B_eval_tmp * c_B_eval_tmp * t_B_eval_tmp) +
                2.0 * K_p_T * Omega_4 * cos(b_4) * sin(g_4) * y_B_eval_tmp) / m;
  ab_B_eval_tmp = db_B_eval_tmp * f_B_eval_tmp;
  B_eval[25] = -((ab_B_eval_tmp * c_B_eval_tmp * i_B_eval_tmp + eb_B_eval_tmp *
                  e_B_eval_tmp) + fb_B_eval_tmp * y_B_eval_tmp) / m;
  eb_B_eval_tmp = gb_B_eval_tmp * f_B_eval_tmp;
  B_eval[31] = -((eb_B_eval_tmp * c_B_eval_tmp * m_B_eval_tmp + hb_B_eval_tmp *
                  e_B_eval_tmp) + ib_B_eval_tmp * y_B_eval_tmp) / m;
  fb_B_eval_tmp = jb_B_eval_tmp * f_B_eval_tmp;
  B_eval[37] = -((fb_B_eval_tmp * c_B_eval_tmp * q_B_eval_tmp + kb_B_eval_tmp *
                  e_B_eval_tmp) + lb_B_eval_tmp * y_B_eval_tmp) / m;
  hb_B_eval_tmp = mb_B_eval_tmp * f_B_eval_tmp;
  B_eval[43] = -((hb_B_eval_tmp * c_B_eval_tmp * u_B_eval_tmp + nb_B_eval_tmp *
                  e_B_eval_tmp) + ob_B_eval_tmp * y_B_eval_tmp) / m;
  B_eval[49] = (pb_B_eval_tmp * y_B_eval_tmp - qb_B_eval_tmp * e_B_eval_tmp) / m;
  B_eval[55] = (sb_B_eval_tmp * y_B_eval_tmp - tb_B_eval_tmp * e_B_eval_tmp) / m;
  B_eval[61] = (vb_B_eval_tmp * y_B_eval_tmp - wb_B_eval_tmp * e_B_eval_tmp) / m;
  B_eval[67] = (yb_B_eval_tmp * y_B_eval_tmp - ac_B_eval_tmp * e_B_eval_tmp) / m;
  B_eval[2] = ((w_B_eval_tmp * h_B_eval_tmp * e_B_eval_tmp + 2.0 * K_p_T
                * Omega_1 * cos(Theta) * d_B_eval_tmp * i_B_eval_tmp *
                j_B_eval_tmp) - w_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp *
               i_B_eval_tmp * g_B_eval_tmp) / m;
  B_eval[8] = ((x_B_eval_tmp * l_B_eval_tmp * e_B_eval_tmp + 2.0 * K_p_T
                * Omega_2 * cos(Theta) * d_B_eval_tmp * m_B_eval_tmp *
                n_B_eval_tmp) - x_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp *
               m_B_eval_tmp * k_B_eval_tmp) / m;
  B_eval[14] = ((bb_B_eval_tmp * p_B_eval_tmp * e_B_eval_tmp + 2.0 * K_p_T
                 * Omega_3 * cos(Theta) * d_B_eval_tmp * q_B_eval_tmp *
                 r_B_eval_tmp) - bb_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp *
                q_B_eval_tmp * o_B_eval_tmp) / m;
  B_eval[20] = ((cb_B_eval_tmp * t_B_eval_tmp * e_B_eval_tmp + 2.0 * K_p_T
                 * Omega_4 * cos(Theta) * d_B_eval_tmp * u_B_eval_tmp *
                 v_B_eval_tmp) - cb_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp *
                u_B_eval_tmp * s_B_eval_tmp) / m;
  c_B_eval_tmp = db_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp;
  y_B_eval_tmp = ab_B_eval_tmp * d_B_eval_tmp;
  B_eval[26] = ((B_eval_tmp * e_B_eval_tmp + c_B_eval_tmp * g_B_eval_tmp *
                 h_B_eval_tmp) - y_B_eval_tmp * h_B_eval_tmp * j_B_eval_tmp) / m;
  B_eval_tmp = gb_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp;
  ab_B_eval_tmp = eb_B_eval_tmp * d_B_eval_tmp;
  B_eval[32] = ((rb_B_eval_tmp * e_B_eval_tmp + B_eval_tmp * k_B_eval_tmp *
                 l_B_eval_tmp) - ab_B_eval_tmp * l_B_eval_tmp * n_B_eval_tmp) /
    m;
  eb_B_eval_tmp = jb_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp;
  fb_B_eval_tmp *= d_B_eval_tmp;
  B_eval[38] = ((ub_B_eval_tmp * e_B_eval_tmp + eb_B_eval_tmp * o_B_eval_tmp *
                 p_B_eval_tmp) - fb_B_eval_tmp * p_B_eval_tmp * r_B_eval_tmp) /
    m;
  b_B_eval_tmp = mb_B_eval_tmp * b_B_eval_tmp * f_B_eval_tmp;
  d_B_eval_tmp *= hb_B_eval_tmp;
  B_eval[44] = ((xb_B_eval_tmp * e_B_eval_tmp + b_B_eval_tmp * s_B_eval_tmp *
                 t_B_eval_tmp) - d_B_eval_tmp * t_B_eval_tmp * v_B_eval_tmp) / m;
  B_eval[50] = (c_B_eval_tmp * i_B_eval_tmp * j_B_eval_tmp + y_B_eval_tmp *
                i_B_eval_tmp * g_B_eval_tmp) / m;
  B_eval[56] = (B_eval_tmp * m_B_eval_tmp * n_B_eval_tmp + ab_B_eval_tmp *
                m_B_eval_tmp * k_B_eval_tmp) / m;
  B_eval[62] = (eb_B_eval_tmp * q_B_eval_tmp * r_B_eval_tmp + fb_B_eval_tmp *
                q_B_eval_tmp * o_B_eval_tmp) / m;
  B_eval[68] = (b_B_eval_tmp * u_B_eval_tmp * v_B_eval_tmp + d_B_eval_tmp *
                u_B_eval_tmp * s_B_eval_tmp) / m;
  B_eval_tmp = w_B_eval_tmp * l_z;
  b_B_eval_tmp = 2.0 * K_p_M * Omega_1;
  c_B_eval_tmp = w_B_eval_tmp * l_1;
  B_eval[3] = ((b_B_eval_tmp * h_B_eval_tmp + B_eval_tmp * i_B_eval_tmp *
                j_B_eval_tmp) + c_B_eval_tmp * i_B_eval_tmp * g_B_eval_tmp) /
    I_xx;
  d_B_eval_tmp = x_B_eval_tmp * l_z;
  e_B_eval_tmp = 2.0 * K_p_M * Omega_2;
  f_B_eval_tmp = x_B_eval_tmp * l_1;
  B_eval[9] = -((e_B_eval_tmp * l_B_eval_tmp - d_B_eval_tmp * m_B_eval_tmp *
                 n_B_eval_tmp) + f_B_eval_tmp * m_B_eval_tmp * k_B_eval_tmp) /
    I_xx;
  y_B_eval_tmp = 2.0 * K_p_M * Omega_3;
  ab_B_eval_tmp = bb_B_eval_tmp * l_z;
  eb_B_eval_tmp = bb_B_eval_tmp * l_2;
  B_eval[15] = ((y_B_eval_tmp * p_B_eval_tmp + ab_B_eval_tmp * q_B_eval_tmp *
                 r_B_eval_tmp) - eb_B_eval_tmp * q_B_eval_tmp * o_B_eval_tmp) /
    I_xx;
  fb_B_eval_tmp = cb_B_eval_tmp * l_z;
  hb_B_eval_tmp = 2.0 * K_p_M * Omega_4;
  ib_B_eval_tmp = cb_B_eval_tmp * l_2;
  B_eval[21] = ((fb_B_eval_tmp * u_B_eval_tmp * v_B_eval_tmp - hb_B_eval_tmp *
                 t_B_eval_tmp) + ib_B_eval_tmp * u_B_eval_tmp * s_B_eval_tmp) /
    I_xx;
  kb_B_eval_tmp = db_B_eval_tmp * l_z;
  lb_B_eval_tmp = db_B_eval_tmp * l_1;
  nb_B_eval_tmp = K_p_M * W_act_motor;
  ob_B_eval_tmp = nb_B_eval_tmp * i_B_eval_tmp;
  B_eval[27] = -((lb_B_eval_tmp * g_B_eval_tmp * h_B_eval_tmp - ob_B_eval_tmp) +
                 kb_B_eval_tmp * h_B_eval_tmp * j_B_eval_tmp) / I_xx;
  pb_B_eval_tmp = gb_B_eval_tmp * l_z;
  qb_B_eval_tmp = gb_B_eval_tmp * l_1;
  rb_B_eval_tmp = K_p_M * W_act_el;
  sb_B_eval_tmp = rb_B_eval_tmp * m_B_eval_tmp;
  B_eval[33] = -((sb_B_eval_tmp - qb_B_eval_tmp * k_B_eval_tmp * l_B_eval_tmp) +
                 pb_B_eval_tmp * l_B_eval_tmp * n_B_eval_tmp) / I_xx;
  tb_B_eval_tmp = jb_B_eval_tmp * l_z;
  ub_B_eval_tmp = jb_B_eval_tmp * l_2;
  vb_B_eval_tmp = K_p_M * W_act_az;
  wb_B_eval_tmp = vb_B_eval_tmp * q_B_eval_tmp;
  B_eval[39] = ((wb_B_eval_tmp + ub_B_eval_tmp * o_B_eval_tmp * p_B_eval_tmp) -
                tb_B_eval_tmp * p_B_eval_tmp * r_B_eval_tmp) / I_xx;
  xb_B_eval_tmp = mb_B_eval_tmp * l_z;
  yb_B_eval_tmp = mb_B_eval_tmp * l_2;
  ac_B_eval_tmp = K_p_M * B_eval_tmp_tmp;
  W_act_motor = ac_B_eval_tmp * u_B_eval_tmp;
  B_eval[45] = -((W_act_motor + yb_B_eval_tmp * s_B_eval_tmp * t_B_eval_tmp) +
                 xb_B_eval_tmp * t_B_eval_tmp * v_B_eval_tmp) / I_xx;
  kb_B_eval_tmp *= i_B_eval_tmp;
  lb_B_eval_tmp *= i_B_eval_tmp;
  B_eval[51] = (kb_B_eval_tmp * g_B_eval_tmp - lb_B_eval_tmp * j_B_eval_tmp) /
    I_xx;
  pb_B_eval_tmp *= m_B_eval_tmp;
  qb_B_eval_tmp *= m_B_eval_tmp;
  B_eval[57] = (pb_B_eval_tmp * k_B_eval_tmp + qb_B_eval_tmp * n_B_eval_tmp) /
    I_xx;
  tb_B_eval_tmp *= q_B_eval_tmp;
  ub_B_eval_tmp *= q_B_eval_tmp;
  B_eval[63] = (tb_B_eval_tmp * o_B_eval_tmp + ub_B_eval_tmp * r_B_eval_tmp) /
    I_xx;
  xb_B_eval_tmp *= u_B_eval_tmp;
  yb_B_eval_tmp *= u_B_eval_tmp;
  B_eval[69] = (xb_B_eval_tmp * s_B_eval_tmp - yb_B_eval_tmp * v_B_eval_tmp) /
    I_xx;
  B_eval[4] = ((B_eval_tmp * h_B_eval_tmp - b_B_eval_tmp * i_B_eval_tmp *
                j_B_eval_tmp) + w_B_eval_tmp * l_4 * i_B_eval_tmp * g_B_eval_tmp)
    / I_yy;
  B_eval[10] = ((d_B_eval_tmp * l_B_eval_tmp + e_B_eval_tmp * m_B_eval_tmp *
                 n_B_eval_tmp) + x_B_eval_tmp * l_4 * m_B_eval_tmp *
                k_B_eval_tmp) / I_yy;
  B_eval[16] = -((y_B_eval_tmp * q_B_eval_tmp * r_B_eval_tmp - ab_B_eval_tmp *
                  p_B_eval_tmp) + bb_B_eval_tmp * l_3 * q_B_eval_tmp *
                 o_B_eval_tmp) / I_yy;
  B_eval[22] = ((fb_B_eval_tmp * t_B_eval_tmp + hb_B_eval_tmp * u_B_eval_tmp *
                 v_B_eval_tmp) - cb_B_eval_tmp * l_3 * u_B_eval_tmp *
                s_B_eval_tmp) / I_yy;
  B_eval_tmp = db_B_eval_tmp * l_4;
  B_eval[28] = ((nb_B_eval_tmp * h_B_eval_tmp * j_B_eval_tmp + kb_B_eval_tmp) -
                B_eval_tmp * g_B_eval_tmp * h_B_eval_tmp) / I_yy;
  b_B_eval_tmp = gb_B_eval_tmp * l_4;
  B_eval[34] = -((rb_B_eval_tmp * l_B_eval_tmp * n_B_eval_tmp - pb_B_eval_tmp) +
                 b_B_eval_tmp * k_B_eval_tmp * l_B_eval_tmp) / I_yy;
  d_B_eval_tmp = jb_B_eval_tmp * l_3;
  B_eval[40] = ((vb_B_eval_tmp * p_B_eval_tmp * r_B_eval_tmp + tb_B_eval_tmp) +
                d_B_eval_tmp * o_B_eval_tmp * p_B_eval_tmp) / I_yy;
  e_B_eval_tmp = mb_B_eval_tmp * l_3;
  B_eval[46] = ((xb_B_eval_tmp - ac_B_eval_tmp * t_B_eval_tmp * v_B_eval_tmp) +
                e_B_eval_tmp * s_B_eval_tmp * t_B_eval_tmp) / I_yy;
  i_B_eval_tmp *= B_eval_tmp;
  B_eval[52] = -(ob_B_eval_tmp * g_B_eval_tmp + i_B_eval_tmp * j_B_eval_tmp) /
    I_yy;
  m_B_eval_tmp *= b_B_eval_tmp;
  B_eval[58] = (sb_B_eval_tmp * k_B_eval_tmp - m_B_eval_tmp * n_B_eval_tmp) /
    I_yy;
  q_B_eval_tmp *= d_B_eval_tmp;
  B_eval[64] = -(wb_B_eval_tmp * o_B_eval_tmp - q_B_eval_tmp * r_B_eval_tmp) /
    I_yy;
  u_B_eval_tmp *= e_B_eval_tmp;
  B_eval[70] = (W_act_motor * s_B_eval_tmp + u_B_eval_tmp * v_B_eval_tmp) / I_yy;
  B_eval[5] = ((2.0 * K_p_M * Omega_1 * cos(b_1) * g_B_eval_tmp - c_B_eval_tmp *
                h_B_eval_tmp) + 2.0 * K_p_T * Omega_1 * l_4 * cos(b_1) *
               j_B_eval_tmp) / I_zz;
  B_eval[11] = ((f_B_eval_tmp * l_B_eval_tmp - 2.0 * K_p_M * Omega_2 * cos(b_2) *
                 k_B_eval_tmp) + 2.0 * K_p_T * Omega_2 * l_4 * cos(b_2) *
                n_B_eval_tmp) / I_zz;
  B_eval[17] = ((eb_B_eval_tmp * p_B_eval_tmp + 2.0 * K_p_M * Omega_3 * cos(b_3)
                 * o_B_eval_tmp) - 2.0 * K_p_T * Omega_3 * l_3 * cos(b_3) *
                r_B_eval_tmp) / I_zz;
  B_eval[23] = -((ib_B_eval_tmp * t_B_eval_tmp + 2.0 * K_p_M * Omega_4 * cos(b_4)
                  * s_B_eval_tmp) + 2.0 * K_p_T * Omega_4 * l_3 * cos(b_4) *
                 v_B_eval_tmp) / I_zz;
  B_eval[29] = -((lb_B_eval_tmp + nb_B_eval_tmp * g_B_eval_tmp * h_B_eval_tmp) +
                 B_eval_tmp * h_B_eval_tmp * j_B_eval_tmp) / I_zz;
  B_eval[35] = ((qb_B_eval_tmp + rb_B_eval_tmp * k_B_eval_tmp * l_B_eval_tmp) -
                b_B_eval_tmp * l_B_eval_tmp * n_B_eval_tmp) / I_zz;
  B_eval[41] = ((ub_B_eval_tmp - vb_B_eval_tmp * o_B_eval_tmp * p_B_eval_tmp) +
                d_B_eval_tmp * p_B_eval_tmp * r_B_eval_tmp) / I_zz;
  B_eval[47] = ((ac_B_eval_tmp * s_B_eval_tmp * t_B_eval_tmp - yb_B_eval_tmp) +
                e_B_eval_tmp * t_B_eval_tmp * v_B_eval_tmp) / I_zz;
  B_eval[53] = -(ob_B_eval_tmp * j_B_eval_tmp - i_B_eval_tmp * g_B_eval_tmp) /
    I_zz;
  B_eval[59] = (sb_B_eval_tmp * n_B_eval_tmp + m_B_eval_tmp * k_B_eval_tmp) /
    I_zz;
  B_eval[65] = -(wb_B_eval_tmp * r_B_eval_tmp + q_B_eval_tmp * o_B_eval_tmp) /
    I_zz;
  B_eval[71] = (W_act_motor * v_B_eval_tmp - u_B_eval_tmp * s_B_eval_tmp) / I_zz;
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

  /*  Apply gains for the normalization of the effectiveness matrix: */
  c_compute_acc_nonlinear_earth_r(actual_u, Theta, Phi, Psi, p, q, r, K_p_T,
    K_p_M, m, I_xx, I_yy, I_zz, l_1, l_2, l_3, l_4, l_z, Cl_alpha, Cd_zero, K_Cd,
    Cm_alpha, Cm_zero, rho, V, S, wing_chord, flight_path_angle, Beta, residuals);
  for (i = 0; i < 6; i++) {
    W_act_motor = dv[i];
    residuals[i] += W_act_motor;
    W_act_el = 0.0;
    for (k = 0; k < 12; k++) {
      W_act_el += B_eval[i + 6 * k] * actual_u[k];
    }

    W_act_motor += W_act_el;
    dv[i] = W_act_motor;
  }

  for (i = 0; i < 4; i++) {
    for (k = 0; k < 6; k++) {
      aoffset = k + 6 * i;
      B_eval[aoffset] *= gain_motor;
      aoffset = k + 6 * (i + 4);
      B_eval[aoffset] *= gain_el;
      aoffset = k + 6 * (i + 8);
      B_eval[aoffset] *= gain_az;
    }
  }

  /*  Apply WLS algorithm: */
  /* Build the max and minimum actuator array:  */
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
  for (i = 0; i < 8; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }

  /* Comput now the delta to take from the actual actuator position to the desired one:  */
  W_act_el = desired_motor_value / gain_motor;
  desired_u_scaled[0] = W_act_el;
  desired_u_scaled[1] = W_act_el;
  desired_u_scaled[2] = W_act_el;
  desired_u_scaled[3] = W_act_el;
  W_act_el = desired_el_value / gain_el;
  desired_u_scaled[4] = W_act_el;
  W_act_motor = desired_az_value / gain_az;
  desired_u_scaled[8] = W_act_motor;
  desired_u_scaled[5] = W_act_el;
  desired_u_scaled[9] = W_act_motor;
  desired_u_scaled[6] = W_act_el;
  desired_u_scaled[10] = W_act_motor;
  desired_u_scaled[7] = W_act_el;
  desired_u_scaled[11] = W_act_motor;

  /* Compute the maximum and minimum control input value for each channel:  */
  /*  Motor */
  memcpy(&u_max_constrain[0], &u_max[0], 12U * sizeof(double));
  memcpy(&u_min_constrain[0], &u_min[0], 12U * sizeof(double));
  if (max_alpha > 3.0) {
    W_act_el = max_alpha * 3.1415926535897931 / 180.0;
    for (i = 0; i < 8; i++) {
      W_act_motor = actual_u[i + 4];
      u_max_constrain[i + 4] = W_act_motor + W_act_el;
      u_min_constrain[i + 4] = W_act_motor - W_act_el;
    }
  }

  /*  Elevator */
  for (k = 0; k < 12; k++) {
    max_u_scaled[k] = fmin(u_max[k], u_max_constrain[k]);
    min_u_scaled[k] = fmax(u_min[k], u_min_constrain[k]);
  }

  max_u_scaled[0] /= gain_motor;
  min_u_scaled[0] /= gain_motor;
  max_u_scaled[4] /= gain_el;
  min_u_scaled[4] /= gain_el;
  max_u_scaled[8] /= gain_az;
  min_u_scaled[8] /= gain_az;
  max_u_scaled[1] /= gain_motor;
  min_u_scaled[1] /= gain_motor;
  max_u_scaled[5] /= gain_el;
  min_u_scaled[5] /= gain_el;
  max_u_scaled[9] /= gain_az;
  min_u_scaled[9] /= gain_az;
  max_u_scaled[2] /= gain_motor;
  min_u_scaled[2] /= gain_motor;
  max_u_scaled[6] /= gain_el;
  min_u_scaled[6] /= gain_el;
  max_u_scaled[10] /= gain_az;
  min_u_scaled[10] /= gain_az;
  max_u_scaled[3] /= gain_motor;
  min_u_scaled[3] /= gain_motor;
  max_u_scaled[7] /= gain_el;
  min_u_scaled[7] /= gain_el;
  max_u_scaled[11] /= gain_az;
  min_u_scaled[11] /= gain_az;

  /* Weighting matrix for the control objective and pseudo control: */
  W_act_motor = fmax(1.0, W_act_motor_const + W_act_motor_speed * V);
  W_act_el = fmax(0.0, W_act_tilt_el_const + W_act_tilt_el_speed * V);
  W_act_az = fmax(0.0, W_act_tilt_az_const + W_act_tilt_az_speed * V);
  v[0] = W_dv_1;
  v[1] = W_dv_2;
  v[2] = W_dv_3;
  v[3] = W_dv_4;
  v[4] = W_dv_5;
  v[5] = W_dv_6;
  memset(&Wv[0], 0, 36U * sizeof(double));
  for (aoffset = 0; aoffset < 6; aoffset++) {
    Wv[aoffset + 6 * aoffset] = v[aoffset];
  }

  actual_u[0] = W_act_motor;
  actual_u[1] = W_act_motor;
  actual_u[2] = W_act_motor;
  actual_u[3] = W_act_motor;
  actual_u[4] = W_act_el;
  actual_u[5] = W_act_el;
  actual_u[6] = W_act_el;
  actual_u[7] = W_act_el;
  actual_u[8] = W_act_az;
  actual_u[9] = W_act_az;
  actual_u[10] = W_act_az;
  actual_u[11] = W_act_az;
  memset(&Wu[0], 0, 144U * sizeof(double));
  for (aoffset = 0; aoffset < 12; aoffset++) {
    Wu[aoffset + 12 * aoffset] = actual_u[aoffset];
  }

  /*  WLS_ALLOC - Control allocation using weighted least squares. */
  /*  */
  /*   [u,W,iter] = wls_alloc(B,v,umin,umax,[Wv,Wu,ud,gamma,u0,W0,imax]) */
  /*  */
  /*  Solves the weighted, bounded least-squares problem */
  /*  */
  /*    min ||Wu(u-ud)||^2 + gamma ||Wv(Bu-v)||^2 */
  /*  */
  /*    subj. to  umin <= u <= umax */
  /*  */
  /*  using an active set method. */
  /*  */
  /*   Inputs: */
  /*   ------- */
  /*  B     control effectiveness matrix (k x m) */
  /*  v     commanded virtual control (k x 1) */
  /*  umin  lower position limits (m x 1) */
  /*  umax  upper position limits (m x 1) */
  /*  Wv    virtual control weighting matrix (k x k) [I] */
  /*  Wu    control weighting matrix (m x m) [I] */
  /*  ud    desired control (m x 1) [0] */
  /*  gamma weight (scalar) [1e6] */
  /*  u0    initial point (m x 1) */
  /*  W0    initial working set (m x 1) [empty] */
  /*  imax  max no. of iterations [100] */
  /*   */
  /*   Outputs: */
  /*   ------- */
  /*  u     optimal control */
  /*  W     optimal active set */
  /*  iter  no. of iterations (= no. of changes in the working set + 1) */
  /*  */
  /*                             0 if u_i not saturated */
  /*  Working set syntax: W_i = -1 if u_i = umin_i */
  /*                            +1 if u_i = umax_i */
  /*  */
  /*  See also: WLSC_ALLOC, IP_ALLOC, FXP_ALLOC, QP_SIM. */
  memset(&u_max_constrain[0], 0, 12U * sizeof(double));
  memset(&u_min[0], 0, 12U * sizeof(double));

  /*  Number of variables */
  /*  Set default values of optional arguments */
  W_act_el = sqrt(gammasq * gammasq);
  for (i = 0; i < 6; i++) {
    for (k = 0; k < 12; k++) {
      W_act_motor = 0.0;
      for (aoffset = 0; aoffset < 6; aoffset++) {
        W_act_motor += W_act_el * Wv[i + 6 * aoffset] * B_eval[aoffset + 6 * k];
      }

      gam_sq[i + 6 * k] = W_act_motor;
    }
  }

  for (i = 0; i < 12; i++) {
    for (k = 0; k < 6; k++) {
      A[k + 18 * i] = gam_sq[k + 6 * i];
    }

    memcpy(&A[i * 18 + 6], &Wu[i * 12], 12U * sizeof(double));
  }

  /*  Initial residual. */
  for (i = 0; i < 6; i++) {
    W_act_motor = 0.0;
    for (k = 0; k < 6; k++) {
      W_act_motor += W_act_el * Wv[i + 6 * k] * dv[k];
    }

    v[i] = W_act_motor;
  }

  for (i = 0; i < 12; i++) {
    W_act_motor = 0.0;
    for (k = 0; k < 12; k++) {
      W_act_motor += Wu[i + 12 * k] * desired_u_scaled[k];
    }

    u_min_constrain[i] = W_act_motor;
  }

  for (i = 0; i < 6; i++) {
    d[i] = v[i];
  }

  memcpy(&d[6], &u_min_constrain[0], 12U * sizeof(double));
  for (i = 0; i < 18; i++) {
    W_act_motor = 0.0;
    for (k = 0; k < 12; k++) {
      W_act_motor += A[i + 18 * k] * 0.0;
    }

    d[i] -= W_act_motor;
  }

  /*  Determine indeces of free variables. */
  for (b_i = 0; b_i < 12; b_i++) {
    i_free[b_i] = true;
  }

  /*  Iterate until optimum is found or maximum number of iterations */
  /*  is reached. */
  b_N_iterations = 1;
  iter = 0;
  exitg1 = false;
  while ((!exitg1) && (iter < 100)) {
    double A_free_data[216];
    int trueCount;
    bool x_data[12];
    bool exitg2;
    bool y;
    b_N_iterations = iter + 1;

    /*  ---------------------------------------- */
    /*   Compute optimal perturbation vector p. */
    /*  ---------------------------------------- */
    /*  Eliminate saturated variables. */
    trueCount = 0;
    aoffset = 0;
    for (b_i = 0; b_i < 12; b_i++) {
      if (i_free[b_i]) {
        trueCount++;
        tmp_data[aoffset] = (signed char)(b_i + 1);
        aoffset++;
      }
    }

    A_free_size[0] = 18;
    A_free_size[1] = trueCount;
    for (i = 0; i < trueCount; i++) {
      for (k = 0; k < 18; k++) {
        A_free_data[k + 18 * i] = A[k + 18 * (tmp_data[i] - 1)];
      }
    }

    /*  Solve the reduced optimization problem for free variables. */
    mldivide(A_free_data, A_free_size, d, u_min_constrain, &aoffset);

    /*  Zero all perturbations corresponding to active constraints. */
    /*  Insert perturbations from p_free into free the variables. */
    aoffset = 0;

    /*  ---------------------------- */
    /*   Is the new point feasible? */
    /*  ---------------------------- */
    b_trueCount = 0;
    k = 0;
    for (b_i = 0; b_i < 12; b_i++) {
      W_act_motor = 0.0;
      u_max[b_i] = 0.0;
      y = i_free[b_i];
      if (y) {
        W_act_motor = u_min_constrain[aoffset];
        u_max[b_i] = u_min_constrain[aoffset];
        aoffset++;
      }

      actual_u[b_i] = u_min[b_i] + W_act_motor;
      if (y) {
        b_trueCount++;
        b_tmp_data[k] = (signed char)(b_i + 1);
        k++;
      }
    }

    for (i = 0; i < b_trueCount; i++) {
      aoffset = b_tmp_data[i] - 1;
      W_act_el = actual_u[aoffset];
      x_data[i] = ((W_act_el < min_u_scaled[aoffset]) || (W_act_el >
        max_u_scaled[aoffset]));
    }

    y = false;
    aoffset = 1;
    exitg2 = false;
    while ((!exitg2) && (aoffset <= b_trueCount)) {
      if (x_data[aoffset - 1]) {
        y = true;
        exitg2 = true;
      } else {
        aoffset++;
      }
    }

    if (!y) {
      /*  ---------------------------- */
      /*   Yes, check for optimality. */
      /*  ---------------------------- */
      /*  Update point and residual. */
      memcpy(&u_min[0], &actual_u[0], 12U * sizeof(double));
      memset(&b_A[0], 0, 18U * sizeof(double));
      for (k = 0; k < trueCount; k++) {
        aoffset = k * 18;
        for (b_i = 0; b_i < 18; b_i++) {
          i = aoffset + b_i;
          b_A[b_i] += A[i % 18 + 18 * (tmp_data[i / 18] - 1)] *
            u_min_constrain[k];
        }
      }

      for (i = 0; i < 18; i++) {
        d[i] -= b_A[i];
      }

      /*  Compute Lagrangian multipliers. */
      for (i = 0; i < 12; i++) {
        W_act_motor = 0.0;
        for (k = 0; k < 18; k++) {
          W_act_motor += A[k + 18 * i] * d[k];
        }

        actual_u[i] = u_max_constrain[i] * W_act_motor;
      }

      /*  Are all lambda non-negative? */
      y = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 12)) {
        if (!(actual_u[k] >= -2.2204460492503131E-16)) {
          y = false;
          exitg2 = true;
        } else {
          k++;
        }
      }

      if (y) {
        /*  / ------------------------ \ */
        /*  | Optimum found, bail out. | */
        /*  \ ------------------------ / */
        exitg1 = true;
      } else {
        /*  -------------------------------------------------- */
        /*   Optimum not found, remove one active constraint. */
        /*  -------------------------------------------------- */
        /*  Remove constraint with most negative lambda from the */
        /*  working set. */
        minimum(actual_u, &W_act_el, &b_trueCount);
        u_max_constrain[b_trueCount - 1] = 0.0;
        i_free[b_trueCount - 1] = true;
        iter++;
      }
    } else {
      bool bv[12];

      /*  --------------------------------------- */
      /*   No, find primary bounding constraint. */
      /*  --------------------------------------- */
      /*  Compute distances to the different boundaries. Since alpha < 1 */
      /*  is the maximum step length, initiate with ones. */
      b_trueCount = 0;
      aoffset = 0;
      for (b_i = 0; b_i < 12; b_i++) {
        bool b;
        actual_u[b_i] = 1.0;
        W_act_motor = u_max[b_i];
        y = (W_act_motor < 0.0);
        x_data[b_i] = y;
        bv[b_i] = (W_act_motor > 0.0);
        b = i_free[b_i];
        if (b && y) {
          b_trueCount++;
          c_tmp_data[aoffset] = (signed char)(b_i + 1);
          aoffset++;
        }
      }

      for (i = 0; i < b_trueCount; i++) {
        k = c_tmp_data[i] - 1;
        desired_u_scaled[i] = (min_u_scaled[k] - u_min[k]) / u_max[k];
      }

      aoffset = 0;
      b_trueCount = 0;
      k = 0;
      for (b_i = 0; b_i < 12; b_i++) {
        y = i_free[b_i];
        if (y && x_data[b_i]) {
          actual_u[b_i] = desired_u_scaled[aoffset];
          aoffset++;
        }

        if (y && bv[b_i]) {
          b_trueCount++;
          d_tmp_data[k] = (signed char)(b_i + 1);
          k++;
        }
      }

      for (i = 0; i < b_trueCount; i++) {
        k = d_tmp_data[i] - 1;
        desired_u_scaled[i] = (max_u_scaled[k] - u_min[k]) / u_max[k];
      }

      aoffset = 0;
      for (b_i = 0; b_i < 12; b_i++) {
        if (i_free[b_i] && bv[b_i]) {
          actual_u[b_i] = desired_u_scaled[aoffset];
          aoffset++;
        }
      }

      /*  Proportion of p to travel */
      minimum(actual_u, &W_act_el, &b_trueCount);

      /*  Update point and residual. */
      for (i = 0; i < 12; i++) {
        u_min[i] += W_act_el * u_max[i];
      }

      aoffset = 18 * trueCount;
      for (i = 0; i < aoffset; i++) {
        A_free_data[i] *= W_act_el;
      }

      memset(&b_A[0], 0, 18U * sizeof(double));
      for (k = 0; k < trueCount; k++) {
        aoffset = k * 18;
        for (b_i = 0; b_i < 18; b_i++) {
          b_A[b_i] += A_free_data[aoffset + b_i] * u_min_constrain[k];
        }
      }

      for (i = 0; i < 18; i++) {
        d[i] -= b_A[i];
      }

      /*  Add corresponding constraint to working set. */
      W_act_motor = u_max[b_trueCount - 1];
      u_max_constrain[b_trueCount - 1] = W_act_motor;
      if (!rtIsNaN(W_act_motor)) {
        if (u_max[b_trueCount - 1] < 0.0) {
          u_max_constrain[b_trueCount - 1] = -1.0;
        } else {
          u_max_constrain[b_trueCount - 1] = (u_max[b_trueCount - 1] > 0.0);
        }
      }

      i_free[b_trueCount - 1] = false;
      iter++;
    }
  }

  *N_iterations = b_N_iterations;

  /* Scale back the increment to remove the normalization */
  u_min[0] *= gain_motor;
  u_min[4] *= gain_el;
  u_min[8] *= gain_az;
  u_min[1] *= gain_motor;
  u_min[5] *= gain_el;
  u_min[9] *= gain_az;
  u_min[2] *= gain_motor;
  u_min[6] *= gain_el;
  u_min[10] *= gain_az;
  u_min[3] *= gain_motor;
  u_min[7] *= gain_el;
  u_min[11] *= gain_az;
  c_compute_acc_nonlinear_earth_r(u_min, Theta, Phi, Psi, p, q, r, K_p_T, K_p_M,
    m, I_xx, I_yy, I_zz, l_1, l_2, l_3, l_4, l_z, Cl_alpha, Cd_zero, K_Cd,
    Cm_alpha, Cm_zero, rho, V, S, wing_chord, flight_path_angle, Beta, v);
  for (i = 0; i < 6; i++) {
    residuals[i] -= v[i];
  }

  memcpy(&u_out[0], &u_min[0], 12U * sizeof(double));
  u_out[12] = desired_theta_value * 3.1415926535897931 / 180.0;
  u_out[13] = desired_phi_value * 3.1415926535897931 / 180.0;
  *elapsed_time = toc();
  *N_evaluation = b_N_iterations;
  for (i = 0; i < 12; i++) {
    i_free[i] = rtIsNaN(u_min[i]);
  }

  aoffset = i_free[0];
  for (k = 0; k < 11; k++) {
    aoffset += i_free[k + 1];
  }

  if (aoffset > 0.5) {
    *exitflag = -1.0;
  } else {
    *exitflag = 1.0;
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
 *                double computed_acc[6]
 * Return Type  : void
 */
static void c_compute_acc_nonlinear_control(const double u_in[15], double p,
  double q, double r, double K_p_T, double K_p_M, double m, double I_xx, double
  I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4, double l_z,
  double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double Cm_zero,
  double CL_aileron, double rho, double V, double S, double wing_chord, double
  flight_path_angle, double Beta, double computed_acc[6])
{
  double ab_computed_acc_tmp;
  double b_computed_acc_tmp;
  double b_computed_acc_tmp_tmp;
  double bb_computed_acc_tmp;
  double c_computed_acc_tmp;
  double cb_computed_acc_tmp;
  double computed_acc_tmp;
  double computed_acc_tmp_tmp;
  double d_computed_acc_tmp;
  double db_computed_acc_tmp;
  double e_computed_acc_tmp;
  double eb_computed_acc_tmp;
  double f_computed_acc_tmp;
  double fb_computed_acc_tmp;
  double g_computed_acc_tmp;
  double gb_computed_acc_tmp;
  double h_computed_acc_tmp;
  double hb_computed_acc_tmp;
  double i_computed_acc_tmp;
  double j_computed_acc_tmp;
  double k_computed_acc_tmp;
  double l_computed_acc_tmp;
  double m_computed_acc_tmp;
  double n_computed_acc_tmp;
  double o_computed_acc_tmp;
  double p_computed_acc_tmp;
  double q_computed_acc_tmp;
  double r_computed_acc_tmp;
  double s_computed_acc_tmp;
  double t_computed_acc_tmp;
  double u_computed_acc_tmp;
  double v_computed_acc_tmp;
  double w_computed_acc_tmp;
  double x_computed_acc_tmp;
  double y_computed_acc_tmp;
  computed_acc_tmp = cos(u_in[12]);
  b_computed_acc_tmp = sin(u_in[12]);
  computed_acc_tmp_tmp = u_in[12] - flight_path_angle;
  c_computed_acc_tmp = sin(computed_acc_tmp_tmp);
  d_computed_acc_tmp = cos(computed_acc_tmp_tmp);
  e_computed_acc_tmp = sin(u_in[13]);
  f_computed_acc_tmp = cos(u_in[13]);
  g_computed_acc_tmp = sin(Beta);
  h_computed_acc_tmp = sin(u_in[4]);
  i_computed_acc_tmp = sin(u_in[5]);
  j_computed_acc_tmp = sin(u_in[6]);
  k_computed_acc_tmp = sin(u_in[7]);
  l_computed_acc_tmp = cos(u_in[4]);
  m_computed_acc_tmp = cos(u_in[8]);
  n_computed_acc_tmp = cos(u_in[5]);
  o_computed_acc_tmp = cos(u_in[9]);
  p_computed_acc_tmp = cos(u_in[6]);
  q_computed_acc_tmp = cos(u_in[10]);
  r_computed_acc_tmp = cos(u_in[7]);
  s_computed_acc_tmp = cos(u_in[11]);
  t_computed_acc_tmp = sin(u_in[8]);
  u_computed_acc_tmp = sin(u_in[9]);
  v_computed_acc_tmp = sin(u_in[10]);
  w_computed_acc_tmp = sin(u_in[11]);
  x_computed_acc_tmp = V * V;
  y_computed_acc_tmp = u_in[0] * u_in[0];
  ab_computed_acc_tmp = u_in[1] * u_in[1];
  bb_computed_acc_tmp = u_in[2] * u_in[2];
  cb_computed_acc_tmp = u_in[3] * u_in[3];
  b_computed_acc_tmp_tmp = S * x_computed_acc_tmp * rho;
  db_computed_acc_tmp = b_computed_acc_tmp_tmp * cos(Beta);
  eb_computed_acc_tmp = Cl_alpha * S * x_computed_acc_tmp * rho;
  fb_computed_acc_tmp = b_computed_acc_tmp_tmp * g_computed_acc_tmp;
  gb_computed_acc_tmp = Cd_zero + Cl_alpha * Cl_alpha * K_Cd *
    (computed_acc_tmp_tmp * computed_acc_tmp_tmp);
  hb_computed_acc_tmp = db_computed_acc_tmp * c_computed_acc_tmp *
    gb_computed_acc_tmp / 2.0 + eb_computed_acc_tmp * d_computed_acc_tmp *
    computed_acc_tmp_tmp / 2.0;
  c_computed_acc_tmp = db_computed_acc_tmp * d_computed_acc_tmp *
    gb_computed_acc_tmp / 2.0 - eb_computed_acc_tmp * c_computed_acc_tmp *
    computed_acc_tmp_tmp / 2.0;
  computed_acc[0] = -(((((computed_acc_tmp * c_computed_acc_tmp +
    computed_acc_tmp * (((K_p_T * h_computed_acc_tmp * y_computed_acc_tmp +
    K_p_T * i_computed_acc_tmp * ab_computed_acc_tmp) + K_p_T
    * j_computed_acc_tmp * bb_computed_acc_tmp) + K_p_T * k_computed_acc_tmp *
                        cb_computed_acc_tmp)) - e_computed_acc_tmp *
    b_computed_acc_tmp * (((K_p_T * l_computed_acc_tmp * t_computed_acc_tmp *
    y_computed_acc_tmp + K_p_T * n_computed_acc_tmp * u_computed_acc_tmp *
    ab_computed_acc_tmp) + K_p_T * p_computed_acc_tmp * v_computed_acc_tmp *
    bb_computed_acc_tmp) + K_p_T * r_computed_acc_tmp * w_computed_acc_tmp *
    cb_computed_acc_tmp)) + f_computed_acc_tmp * b_computed_acc_tmp * (((K_p_T *
    cos(u_in[4]) * m_computed_acc_tmp * y_computed_acc_tmp + K_p_T * cos(u_in[5])
    * o_computed_acc_tmp * ab_computed_acc_tmp) + K_p_T * cos(u_in[6]) *
    q_computed_acc_tmp * bb_computed_acc_tmp) + K_p_T * cos(u_in[7]) *
    s_computed_acc_tmp * cb_computed_acc_tmp)) + cos(u_in[13]) * sin(u_in[12]) *
                       hb_computed_acc_tmp) + fb_computed_acc_tmp *
                      e_computed_acc_tmp * b_computed_acc_tmp *
                      gb_computed_acc_tmp / 2.0) / m;
  d_computed_acc_tmp = ((K_p_T * cos(u_in[4]) * sin(u_in[8]) * (u_in[0] * u_in[0])
    + K_p_T * cos(u_in[5]) * sin(u_in[9]) * (u_in[1] * u_in[1])) + K_p_T * cos
                        (u_in[6]) * sin(u_in[10]) * (u_in[2] * u_in[2])) + K_p_T
    * cos(u_in[7]) * sin(u_in[11]) * (u_in[3] * u_in[3]);
  db_computed_acc_tmp = ((K_p_T * cos(u_in[4]) * cos(u_in[8]) * (u_in[0] * u_in
    [0]) + K_p_T * cos(u_in[5]) * cos(u_in[9]) * (u_in[1] * u_in[1])) + K_p_T
    * cos(u_in[6]) * cos(u_in[10]) * (u_in[2] * u_in[2])) + K_p_T * cos(u_in[7])
    * cos(u_in[11]) * (u_in[3] * u_in[3]);
  computed_acc[1] = (((e_computed_acc_tmp * db_computed_acc_tmp +
                       e_computed_acc_tmp * hb_computed_acc_tmp) +
                      f_computed_acc_tmp * d_computed_acc_tmp) -
                     b_computed_acc_tmp_tmp * f_computed_acc_tmp *
                     g_computed_acc_tmp * gb_computed_acc_tmp / 2.0) / m;
  computed_acc[2] = (((((b_computed_acc_tmp * c_computed_acc_tmp +
    b_computed_acc_tmp * (((K_p_T * sin(u_in[4]) * (u_in[0] * u_in[0]) + K_p_T *
    sin(u_in[5]) * (u_in[1] * u_in[1])) + K_p_T * sin(u_in[6]) * (u_in[2] *
    u_in[2])) + K_p_T * sin(u_in[7]) * (u_in[3] * u_in[3]))) + computed_acc_tmp *
                        e_computed_acc_tmp * d_computed_acc_tmp) -
                       f_computed_acc_tmp * computed_acc_tmp *
                       db_computed_acc_tmp) - cos(u_in[13]) * cos(u_in[12]) *
                      hb_computed_acc_tmp) - fb_computed_acc_tmp *
                     computed_acc_tmp * e_computed_acc_tmp * gb_computed_acc_tmp
                     / 2.0) / m + 9.81;
  computed_acc_tmp = K_p_T * y_computed_acc_tmp;
  b_computed_acc_tmp = K_p_T * ab_computed_acc_tmp;
  c_computed_acc_tmp = K_p_T * bb_computed_acc_tmp;
  d_computed_acc_tmp = K_p_T * cb_computed_acc_tmp;
  e_computed_acc_tmp = computed_acc_tmp * l_z;
  f_computed_acc_tmp = b_computed_acc_tmp * l_z;
  g_computed_acc_tmp = c_computed_acc_tmp * l_z;
  db_computed_acc_tmp = d_computed_acc_tmp * l_z;
  y_computed_acc_tmp *= K_p_M;
  ab_computed_acc_tmp *= K_p_M;
  bb_computed_acc_tmp *= K_p_M;
  cb_computed_acc_tmp *= K_p_M;
  eb_computed_acc_tmp = computed_acc_tmp * l_1;
  fb_computed_acc_tmp = b_computed_acc_tmp * l_1;
  gb_computed_acc_tmp = c_computed_acc_tmp * l_2;
  hb_computed_acc_tmp = d_computed_acc_tmp * l_2;
  computed_acc[3] = ((((((((((((((y_computed_acc_tmp * h_computed_acc_tmp -
    ab_computed_acc_tmp * i_computed_acc_tmp) + bb_computed_acc_tmp *
    j_computed_acc_tmp) - cb_computed_acc_tmp * k_computed_acc_tmp) + I_yy * q *
    r) - I_zz * q * r) + eb_computed_acc_tmp * l_computed_acc_tmp *
    m_computed_acc_tmp) - fb_computed_acc_tmp * n_computed_acc_tmp *
    o_computed_acc_tmp) - gb_computed_acc_tmp * p_computed_acc_tmp *
    q_computed_acc_tmp) + hb_computed_acc_tmp * r_computed_acc_tmp *
    s_computed_acc_tmp) + e_computed_acc_tmp * l_computed_acc_tmp *
    t_computed_acc_tmp) + f_computed_acc_tmp * n_computed_acc_tmp *
                        u_computed_acc_tmp) + g_computed_acc_tmp *
                       p_computed_acc_tmp * v_computed_acc_tmp) +
                      db_computed_acc_tmp * r_computed_acc_tmp *
                      w_computed_acc_tmp) + CL_aileron * S * x_computed_acc_tmp *
                     u_in[14] * rho / 2.0) / I_xx;
  x_computed_acc_tmp = I_xx * p;
  y_computed_acc_tmp *= l_computed_acc_tmp;
  ab_computed_acc_tmp *= n_computed_acc_tmp;
  bb_computed_acc_tmp *= p_computed_acc_tmp;
  cb_computed_acc_tmp *= r_computed_acc_tmp;
  computed_acc_tmp = computed_acc_tmp * l_4 * l_computed_acc_tmp;
  b_computed_acc_tmp = b_computed_acc_tmp * l_4 * n_computed_acc_tmp;
  c_computed_acc_tmp = c_computed_acc_tmp * l_3 * p_computed_acc_tmp;
  d_computed_acc_tmp = d_computed_acc_tmp * l_3 * r_computed_acc_tmp;
  computed_acc[4] = ((((((((((((((I_zz * p * r - x_computed_acc_tmp * r) +
    e_computed_acc_tmp * h_computed_acc_tmp) + f_computed_acc_tmp *
    i_computed_acc_tmp) + g_computed_acc_tmp * j_computed_acc_tmp) +
    db_computed_acc_tmp * k_computed_acc_tmp) - y_computed_acc_tmp *
    t_computed_acc_tmp) + ab_computed_acc_tmp * u_computed_acc_tmp) -
    bb_computed_acc_tmp * v_computed_acc_tmp) + cb_computed_acc_tmp *
    w_computed_acc_tmp) + b_computed_acc_tmp_tmp * wing_chord * (Cm_zero +
    Cm_alpha * computed_acc_tmp_tmp) / 2.0) + computed_acc_tmp *
                        m_computed_acc_tmp) + b_computed_acc_tmp *
                       o_computed_acc_tmp) - c_computed_acc_tmp *
                      q_computed_acc_tmp) - d_computed_acc_tmp *
                     s_computed_acc_tmp) / I_yy;
  computed_acc[5] = (((((((((((((x_computed_acc_tmp * q - I_yy * p * q) -
    eb_computed_acc_tmp * h_computed_acc_tmp) + fb_computed_acc_tmp *
    i_computed_acc_tmp) + gb_computed_acc_tmp * j_computed_acc_tmp) -
    hb_computed_acc_tmp * k_computed_acc_tmp) + y_computed_acc_tmp *
    m_computed_acc_tmp) - ab_computed_acc_tmp * o_computed_acc_tmp) +
    bb_computed_acc_tmp * q_computed_acc_tmp) - cb_computed_acc_tmp *
    s_computed_acc_tmp) + computed_acc_tmp * t_computed_acc_tmp) +
                       b_computed_acc_tmp * u_computed_acc_tmp) -
                      c_computed_acc_tmp * v_computed_acc_tmp) -
                     d_computed_acc_tmp * w_computed_acc_tmp) / I_zz;
}

/*
 * Arguments    : const double u_in[12]
 *                double Theta
 *                double Phi
 *                double Psi
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
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double Beta
 *                double computed_acc[6]
 * Return Type  : void
 */
static void c_compute_acc_nonlinear_earth_r(const double u_in[12], double Theta,
  double Phi, double Psi, double p, double q, double r, double K_p_T, double
  K_p_M, double m, double I_xx, double I_yy, double I_zz, double l_1, double l_2,
  double l_3, double l_4, double l_z, double Cl_alpha, double Cd_zero, double
  K_Cd, double Cm_alpha, double Cm_zero, double rho, double V, double S, double
  wing_chord, double flight_path_angle, double Beta, double computed_acc[6])
{
  double ab_computed_acc_tmp;
  double b_computed_acc_tmp;
  double b_computed_acc_tmp_tmp;
  double bb_computed_acc_tmp;
  double c_computed_acc_tmp;
  double c_computed_acc_tmp_tmp;
  double cb_computed_acc_tmp;
  double computed_acc_tmp;
  double computed_acc_tmp_tmp;
  double d_computed_acc_tmp;
  double d_computed_acc_tmp_tmp;
  double db_computed_acc_tmp;
  double e_computed_acc_tmp;
  double eb_computed_acc_tmp;
  double f_computed_acc_tmp;
  double fb_computed_acc_tmp;
  double g_computed_acc_tmp;
  double gb_computed_acc_tmp;
  double h_computed_acc_tmp;
  double hb_computed_acc_tmp;
  double i_computed_acc_tmp;
  double j_computed_acc_tmp;
  double k_computed_acc_tmp;
  double l_computed_acc_tmp;
  double m_computed_acc_tmp;
  double n_computed_acc_tmp;
  double o_computed_acc_tmp;
  double p_computed_acc_tmp;
  double q_computed_acc_tmp;
  double r_computed_acc_tmp;
  double s_computed_acc_tmp;
  double t_computed_acc_tmp;
  double u_computed_acc_tmp;
  double v_computed_acc_tmp;
  double w_computed_acc_tmp;
  double x_computed_acc_tmp;
  double y_computed_acc_tmp;

  /*      computed_acc(1) = -((sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1^2 + K_p_T*cos(b_2)*cos(g_2)*Omega_2^2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3^2 + K_p_T*cos(b_4)*cos(g_4)*Omega_4^2) + (cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1^2 + K_p_T*cos(b_2)*sin(g_2)*Omega_2^2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3^2 + K_p_T*cos(b_4)*sin(g_4)*Omega_4^2) + cos(Psi)*cos(Theta)*(K_p_T*sin(b_1)*Omega_1^2 + K_p_T*sin(b_2)*Omega_2^2 + K_p_T*sin(b_3)*Omega_3^2 + K_p_T*sin(b_4)*Omega_4^2) + (S*V^2*rho*(Cd_zero + Cl_alpha^2*K_Cd*(Theta - flight_path_angle)^2)*(cos(Beta)*sin(Theta - flight_path_angle)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - sin(Beta)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + cos(Beta)*cos(Psi)*cos(Theta)*cos(Theta - flight_path_angle)))/2 + (Cl_alpha*S*V^2*rho*(cos(Theta - flight_path_angle)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - cos(Psi)*cos(Theta)*sin(Theta - flight_path_angle))*(Theta - flight_path_angle))/2)/m; */
  /*      computed_acc(2) = ((cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1^2 + K_p_T*cos(b_2)*cos(g_2)*Omega_2^2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3^2 + K_p_T*cos(b_4)*cos(g_4)*Omega_4^2) + (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1^2 + K_p_T*cos(b_2)*sin(g_2)*Omega_2^2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3^2 + K_p_T*cos(b_4)*sin(g_4)*Omega_4^2) - cos(Theta)*sin(Psi)*(K_p_T*sin(b_1)*Omega_1^2 + K_p_T*sin(b_2)*Omega_2^2 + K_p_T*sin(b_3)*Omega_3^2 + K_p_T*sin(b_4)*Omega_4^2) - (S*V^2*rho*(Cd_zero + Cl_alpha^2*K_Cd*(Theta - flight_path_angle)^2)*(sin(Beta)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - cos(Beta)*sin(Theta - flight_path_angle)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + cos(Beta)*cos(Theta)*sin(Psi)*cos(Theta - flight_path_angle)))/2 + (Cl_alpha*S*V^2*rho*(cos(Theta - flight_path_angle)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + cos(Theta)*sin(Psi)*sin(Theta - flight_path_angle))*(Theta - flight_path_angle))/2)/m; */
  /*      computed_acc(3) =                                                 981/100 - (cos(Phi)*cos(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1^2 + K_p_T*cos(b_2)*cos(g_2)*Omega_2^2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3^2 + K_p_T*cos(b_4)*cos(g_4)*Omega_4^2) - cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1^2 + K_p_T*cos(b_2)*sin(g_2)*Omega_2^2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3^2 + K_p_T*cos(b_4)*sin(g_4)*Omega_4^2) - (cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))*(K_p_T*sin(b_1)*Omega_1^2 + K_p_T*sin(b_2)*Omega_2^2 + K_p_T*sin(b_3)*Omega_3^2 + K_p_T*sin(b_4)*Omega_4^2) + (S*V^2*rho*(Cd_zero + Cl_alpha^2*K_Cd*(Theta - flight_path_angle)^2)*(sin(Beta)*cos(Theta)*sin(Phi) - cos(Beta)*cos(Theta - flight_path_angle)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + cos(Beta)*cos(Phi)*cos(Theta)*sin(Theta - flight_path_angle)))/2 + (Cl_alpha*S*V^2*rho*(sin(Theta - flight_path_angle)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + cos(Phi)*cos(Theta)*cos(Theta - flight_path_angle))*(Theta - flight_path_angle))/2)/m; */
  computed_acc_tmp = cos(Phi);
  b_computed_acc_tmp = sin(Psi);
  c_computed_acc_tmp = cos(Psi);
  d_computed_acc_tmp = sin(Phi);
  computed_acc_tmp_tmp = sin(Theta);
  b_computed_acc_tmp_tmp = Theta - flight_path_angle;
  e_computed_acc_tmp = cos(b_computed_acc_tmp_tmp);
  f_computed_acc_tmp = sin(b_computed_acc_tmp_tmp);
  g_computed_acc_tmp = cos(Theta);
  h_computed_acc_tmp = V;
  if (!rtIsNaN(V)) {
    if (V < 0.0) {
      h_computed_acc_tmp = -1.0;
    } else {
      h_computed_acc_tmp = (V > 0.0);
    }
  }

  i_computed_acc_tmp = sin(u_in[4]);
  j_computed_acc_tmp = sin(u_in[5]);
  k_computed_acc_tmp = sin(u_in[6]);
  l_computed_acc_tmp = sin(u_in[7]);
  m_computed_acc_tmp = cos(u_in[4]);
  n_computed_acc_tmp = cos(u_in[8]);
  o_computed_acc_tmp = cos(u_in[5]);
  p_computed_acc_tmp = cos(u_in[9]);
  q_computed_acc_tmp = cos(u_in[6]);
  r_computed_acc_tmp = cos(u_in[10]);
  s_computed_acc_tmp = cos(u_in[7]);
  t_computed_acc_tmp = cos(u_in[11]);
  u_computed_acc_tmp = sin(u_in[8]);
  v_computed_acc_tmp = sin(u_in[9]);
  w_computed_acc_tmp = sin(u_in[10]);
  x_computed_acc_tmp = sin(u_in[11]);
  y_computed_acc_tmp = V * V;
  ab_computed_acc_tmp = u_in[0] * u_in[0];
  bb_computed_acc_tmp = u_in[1] * u_in[1];
  cb_computed_acc_tmp = u_in[2] * u_in[2];
  db_computed_acc_tmp = u_in[3] * u_in[3];
  c_computed_acc_tmp_tmp = S * y_computed_acc_tmp * rho;
  eb_computed_acc_tmp = c_computed_acc_tmp_tmp * cos(Beta);
  y_computed_acc_tmp = Cl_alpha * S * y_computed_acc_tmp * rho;
  fb_computed_acc_tmp = c_computed_acc_tmp_tmp * sin(Beta);
  gb_computed_acc_tmp = Cd_zero + Cl_alpha * Cl_alpha * K_Cd *
    (b_computed_acc_tmp_tmp * b_computed_acc_tmp_tmp);
  hb_computed_acc_tmp = eb_computed_acc_tmp * f_computed_acc_tmp *
    gb_computed_acc_tmp / 2.0 + y_computed_acc_tmp * e_computed_acc_tmp *
    b_computed_acc_tmp_tmp / 2.0;
  e_computed_acc_tmp = eb_computed_acc_tmp * e_computed_acc_tmp *
    gb_computed_acc_tmp / 2.0 - y_computed_acc_tmp * f_computed_acc_tmp *
    b_computed_acc_tmp_tmp / 2.0;
  f_computed_acc_tmp = fb_computed_acc_tmp * gb_computed_acc_tmp;
  computed_acc[0] = -((((((d_computed_acc_tmp * b_computed_acc_tmp +
    computed_acc_tmp * c_computed_acc_tmp * computed_acc_tmp_tmp) * (((K_p_T
    * m_computed_acc_tmp * n_computed_acc_tmp * ab_computed_acc_tmp + K_p_T
    * o_computed_acc_tmp * p_computed_acc_tmp * bb_computed_acc_tmp) + K_p_T
    * q_computed_acc_tmp * r_computed_acc_tmp * cb_computed_acc_tmp) + K_p_T
    * s_computed_acc_tmp * t_computed_acc_tmp * db_computed_acc_tmp) +
    hb_computed_acc_tmp * (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)))
    + (computed_acc_tmp * b_computed_acc_tmp - c_computed_acc_tmp *
       d_computed_acc_tmp * computed_acc_tmp_tmp) * (((K_p_T * cos(u_in[4]) *
    u_computed_acc_tmp * ab_computed_acc_tmp + K_p_T * cos(u_in[5]) *
    v_computed_acc_tmp * bb_computed_acc_tmp) + K_p_T * cos(u_in[6]) *
    w_computed_acc_tmp * cb_computed_acc_tmp) + K_p_T * cos(u_in[7]) *
    x_computed_acc_tmp * db_computed_acc_tmp)) + c_computed_acc_tmp *
                        g_computed_acc_tmp * (((K_p_T * i_computed_acc_tmp *
    ab_computed_acc_tmp + K_p_T * j_computed_acc_tmp * bb_computed_acc_tmp) +
    K_p_T * k_computed_acc_tmp * cb_computed_acc_tmp) + K_p_T
    * l_computed_acc_tmp * db_computed_acc_tmp)) + cos(Psi) * cos(Theta) *
                       h_computed_acc_tmp * e_computed_acc_tmp) -
                      f_computed_acc_tmp * (cos(Phi) * sin(Psi) - cos(Psi) * sin
    (Phi) * sin(Theta)) / 2.0) / m;
  d_computed_acc_tmp_tmp = cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) *
    computed_acc_tmp_tmp;
  c_computed_acc_tmp = ((K_p_T * sin(u_in[4]) * (u_in[0] * u_in[0]) + K_p_T
    * sin(u_in[5]) * (u_in[1] * u_in[1])) + K_p_T * sin(u_in[6]) * (u_in[2] *
    u_in[2])) + K_p_T * sin(u_in[7]) * (u_in[3] * u_in[3]);
  y_computed_acc_tmp = ((K_p_T * cos(u_in[4]) * sin(u_in[8]) * (u_in[0] * u_in[0])
    + K_p_T * cos(u_in[5]) * sin(u_in[9]) * (u_in[1] * u_in[1])) + K_p_T * cos
                        (u_in[6]) * sin(u_in[10]) * (u_in[2] * u_in[2])) + K_p_T
    * cos(u_in[7]) * sin(u_in[11]) * (u_in[3] * u_in[3]);
  eb_computed_acc_tmp = ((K_p_T * cos(u_in[4]) * cos(u_in[8]) * (u_in[0] * u_in
    [0]) + K_p_T * cos(u_in[5]) * cos(u_in[9]) * (u_in[1] * u_in[1])) + K_p_T
    * cos(u_in[6]) * cos(u_in[10]) * (u_in[2] * u_in[2])) + K_p_T * cos(u_in[7])
    * cos(u_in[11]) * (u_in[3] * u_in[3]);
  computed_acc[1] = (((((d_computed_acc_tmp_tmp * eb_computed_acc_tmp +
    hb_computed_acc_tmp * d_computed_acc_tmp_tmp) + (cos(Phi) * cos(Psi) + sin
    (Phi) * sin(Psi) * computed_acc_tmp_tmp) * y_computed_acc_tmp) -
                       g_computed_acc_tmp * b_computed_acc_tmp *
                       c_computed_acc_tmp) - cos(Theta) * sin(Psi) *
                      h_computed_acc_tmp * e_computed_acc_tmp) -
                     f_computed_acc_tmp * (cos(Phi) * cos(Psi) + sin(Phi) * sin
    (Psi) * sin(Theta)) / 2.0) / m;
  computed_acc[2] = (((((d_computed_acc_tmp_tmp * c_computed_acc_tmp +
    h_computed_acc_tmp * e_computed_acc_tmp * d_computed_acc_tmp_tmp) +
                        g_computed_acc_tmp * d_computed_acc_tmp *
                        y_computed_acc_tmp) - computed_acc_tmp *
                       g_computed_acc_tmp * eb_computed_acc_tmp) - cos(Phi) *
                      cos(Theta) * hb_computed_acc_tmp) - fb_computed_acc_tmp *
                     g_computed_acc_tmp * d_computed_acc_tmp *
                     gb_computed_acc_tmp / 2.0) / m + 9.81;

  /*      computed_acc(4) =                                                                            (K_p_M*Omega_1^2*sin(b_1) - K_p_M*Omega_2^2*sin(b_2) + K_p_M*Omega_3^2*sin(b_3) - K_p_M*Omega_4^2*sin(b_4) + I_yy*q*r - I_zz*q*r + K_p_T*Omega_1^2*l_1*cos(b_1)*cos(g_1) - K_p_T*Omega_2^2*l_1*cos(b_2)*cos(g_2) - K_p_T*Omega_3^2*l_2*cos(b_3)*cos(g_3) + K_p_T*Omega_4^2*l_2*cos(b_4)*cos(g_4) + K_p_T*Omega_1^2*l_z*cos(b_1)*sin(g_1) + K_p_T*Omega_2^2*l_z*cos(b_2)*sin(g_2) + K_p_T*Omega_3^2*l_z*cos(b_3)*sin(g_3) + K_p_T*Omega_4^2*l_z*cos(b_4)*sin(g_4))/I_xx; */
  /*      computed_acc(5) = (I_zz*p*r - I_xx*p*r + K_p_T*Omega_1^2*l_z*sin(b_1) + K_p_T*Omega_2^2*l_z*sin(b_2) + K_p_T*Omega_3^2*l_z*sin(b_3) + K_p_T*Omega_4^2*l_z*sin(b_4) - K_p_M*Omega_1^2*cos(b_1)*sin(g_1) + K_p_M*Omega_2^2*cos(b_2)*sin(g_2) - K_p_M*Omega_3^2*cos(b_3)*sin(g_3) + K_p_M*Omega_4^2*cos(b_4)*sin(g_4) + (S*V^2*rho*wing_chord*(Cm_zero + Cm_alpha*(Theta - flight_path_angle)))/2 + K_p_T*Omega_1^2*l_4*cos(b_1)*cos(g_1) + K_p_T*Omega_2^2*l_4*cos(b_2)*cos(g_2) - K_p_T*Omega_3^2*l_3*cos(b_3)*cos(g_3) - K_p_T*Omega_4^2*l_3*cos(b_4)*cos(g_4))/I_yy; */
  /*      computed_acc(6) =                                                                        (I_xx*p*q - I_yy*p*q - K_p_T*Omega_1^2*l_1*sin(b_1) + K_p_T*Omega_2^2*l_1*sin(b_2) + K_p_T*Omega_3^2*l_2*sin(b_3) - K_p_T*Omega_4^2*l_2*sin(b_4) + K_p_M*Omega_1^2*cos(b_1)*cos(g_1) - K_p_M*Omega_2^2*cos(b_2)*cos(g_2) + K_p_M*Omega_3^2*cos(b_3)*cos(g_3) - K_p_M*Omega_4^2*cos(b_4)*cos(g_4) + K_p_T*Omega_1^2*l_4*cos(b_1)*sin(g_1) + K_p_T*Omega_2^2*l_4*cos(b_2)*sin(g_2) - K_p_T*Omega_3^2*l_3*cos(b_3)*sin(g_3) - K_p_T*Omega_4^2*l_3*cos(b_4)*sin(g_4))/I_zz; */
  computed_acc_tmp = K_p_T * ab_computed_acc_tmp;
  b_computed_acc_tmp = K_p_T * bb_computed_acc_tmp;
  c_computed_acc_tmp = K_p_T * cb_computed_acc_tmp;
  d_computed_acc_tmp = K_p_T * db_computed_acc_tmp;
  e_computed_acc_tmp = computed_acc_tmp * l_z;
  f_computed_acc_tmp = b_computed_acc_tmp * l_z;
  g_computed_acc_tmp = c_computed_acc_tmp * l_z;
  h_computed_acc_tmp = d_computed_acc_tmp * l_z;
  y_computed_acc_tmp = K_p_M * ab_computed_acc_tmp;
  ab_computed_acc_tmp = K_p_M * bb_computed_acc_tmp;
  bb_computed_acc_tmp = K_p_M * cb_computed_acc_tmp;
  cb_computed_acc_tmp = K_p_M * db_computed_acc_tmp;
  db_computed_acc_tmp = computed_acc_tmp * l_1;
  eb_computed_acc_tmp = b_computed_acc_tmp * l_1;
  fb_computed_acc_tmp = c_computed_acc_tmp * l_2;
  gb_computed_acc_tmp = d_computed_acc_tmp * l_2;
  computed_acc[3] = (((((((((((((y_computed_acc_tmp * i_computed_acc_tmp -
    ab_computed_acc_tmp * j_computed_acc_tmp) + bb_computed_acc_tmp *
    k_computed_acc_tmp) - cb_computed_acc_tmp * l_computed_acc_tmp) + I_yy * q *
    r) - I_zz * q * r) + db_computed_acc_tmp * m_computed_acc_tmp *
    n_computed_acc_tmp) - eb_computed_acc_tmp * o_computed_acc_tmp *
    p_computed_acc_tmp) - fb_computed_acc_tmp * q_computed_acc_tmp *
    r_computed_acc_tmp) + gb_computed_acc_tmp * s_computed_acc_tmp *
    t_computed_acc_tmp) + e_computed_acc_tmp * m_computed_acc_tmp *
                        u_computed_acc_tmp) + f_computed_acc_tmp *
                       o_computed_acc_tmp * v_computed_acc_tmp) +
                      g_computed_acc_tmp * q_computed_acc_tmp *
                      w_computed_acc_tmp) + h_computed_acc_tmp *
                     s_computed_acc_tmp * x_computed_acc_tmp) / I_xx;
  hb_computed_acc_tmp = I_xx * p;
  y_computed_acc_tmp *= m_computed_acc_tmp;
  ab_computed_acc_tmp *= o_computed_acc_tmp;
  bb_computed_acc_tmp *= q_computed_acc_tmp;
  cb_computed_acc_tmp *= s_computed_acc_tmp;
  computed_acc_tmp = computed_acc_tmp * l_4 * m_computed_acc_tmp;
  b_computed_acc_tmp = b_computed_acc_tmp * l_4 * o_computed_acc_tmp;
  c_computed_acc_tmp = c_computed_acc_tmp * l_3 * q_computed_acc_tmp;
  d_computed_acc_tmp = d_computed_acc_tmp * l_3 * s_computed_acc_tmp;
  computed_acc[4] = ((((((((((((((I_zz * p * r - hb_computed_acc_tmp * r) +
    e_computed_acc_tmp * i_computed_acc_tmp) + f_computed_acc_tmp *
    j_computed_acc_tmp) + g_computed_acc_tmp * k_computed_acc_tmp) +
    h_computed_acc_tmp * l_computed_acc_tmp) - y_computed_acc_tmp *
    u_computed_acc_tmp) + ab_computed_acc_tmp * v_computed_acc_tmp) -
    bb_computed_acc_tmp * w_computed_acc_tmp) + cb_computed_acc_tmp *
    x_computed_acc_tmp) + c_computed_acc_tmp_tmp * wing_chord * (Cm_zero +
    Cm_alpha * b_computed_acc_tmp_tmp) / 2.0) + computed_acc_tmp *
                        n_computed_acc_tmp) + b_computed_acc_tmp *
                       p_computed_acc_tmp) - c_computed_acc_tmp *
                      r_computed_acc_tmp) - d_computed_acc_tmp *
                     t_computed_acc_tmp) / I_yy;
  computed_acc[5] = (((((((((((((hb_computed_acc_tmp * q - I_yy * p * q) -
    db_computed_acc_tmp * i_computed_acc_tmp) + eb_computed_acc_tmp *
    j_computed_acc_tmp) + fb_computed_acc_tmp * k_computed_acc_tmp) -
    gb_computed_acc_tmp * l_computed_acc_tmp) + y_computed_acc_tmp *
    n_computed_acc_tmp) - ab_computed_acc_tmp * p_computed_acc_tmp) +
    bb_computed_acc_tmp * r_computed_acc_tmp) - cb_computed_acc_tmp *
    t_computed_acc_tmp) + computed_acc_tmp * u_computed_acc_tmp) +
                       b_computed_acc_tmp * v_computed_acc_tmp) -
                      c_computed_acc_tmp * w_computed_acc_tmp) -
                     d_computed_acc_tmp * x_computed_acc_tmp) / I_zz;
}

/*
 * Arguments    : int n
 *                double a
 *                const double x[12]
 *                int ix0
 *                double y[72]
 *                int iy0
 * Return Type  : void
 */
static void c_xaxpy(int n, double a, const double x[12], int ix0, double y[72],
                    int iy0)
{
  int k;
  if (!(a == 0.0)) {
    int i;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * x[(ix0 + k) - 1];
    }
  }
}

/*
 * Arguments    : int n
 *                const double x_data[]
 *                int ix0
 * Return Type  : double
 */
static double c_xnrm2(int n, const double x_data[], int ix0)
{
  double scale;
  double y;
  int k;
  int kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    double absxk;
    absxk = fabs(x_data[k - 1]);
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
    int ubOffset;
    ubOffset = (iL0 + mLB) - 1;
    for (idx = 0; idx < mLB; idx++) {
      i = finiteLB[idx];
      lbDelta = xCurrent[i - 1] - lb[i - 1];
      lbLambda = lambda[(iL0 + idx) - 1];
      nlpComplError = fmax(nlpComplError, fmin(fabs(lbDelta * lbLambda), fmin
        (fabs(lbDelta), lbLambda)));
    }

    for (idx = 0; idx < mUB; idx++) {
      i = finiteUB[idx];
      lbDelta = ub[i - 1] - xCurrent[i - 1];
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
        int ixlast;
        ixlast = obj->nvar;
        for (idx = 0; idx < ixlast; idx++) {
          val += x[idx] * workspace[idx];
        }
      }
    }
    break;

   default:
    {
      int ixlast;
      linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
      ixlast = obj->nvar + 1;
      for (idx = ixlast; idx < 16; idx++) {
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
        int ixlast;
        ixlast = obj->nvar;
        for (k = 0; k < ixlast; k++) {
          workspace[k] = 0.5 * obj->Hx[k] + f[k];
        }

        val = 0.0;
        if (obj->nvar >= 1) {
          ixlast = obj->nvar;
          for (k = 0; k < ixlast; k++) {
            val += x[k] * workspace[k];
          }
        }
      } else {
        val = 0.0;
        if (obj->nvar >= 1) {
          int ixlast;
          ixlast = obj->nvar;
          for (k = 0; k < ixlast; k++) {
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
        int ixlast;
        ixlast = obj->nvar;
        if (ixlast - 1 >= 0) {
          memcpy(&workspace[0], &f[0], ixlast * sizeof(double));
        }

        ixlast = 14 - obj->nvar;
        for (k = 0; k <= ixlast; k++) {
          workspace[obj->nvar + k] = obj->rho;
        }

        val = 0.0;
        for (k = 0; k < 15; k++) {
          workspace[k] += 0.5 * obj->Hx[k];
          val += x[k] * workspace[k];
        }
      } else {
        int ixlast;
        val = 0.0;
        for (k = 0; k < 15; k++) {
          val += x[k] * obj->Hx[k];
        }

        val *= 0.5;
        ixlast = obj->nvar + 1;
        for (k = ixlast; k < 16; k++) {
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
  int iL0;
  int idx;
  memcpy(&workspace[0], &grad[0], nVar * sizeof(double));
  for (idx = 0; idx < mFixed; idx++) {
    i = finiteFixed[idx];
    workspace[i - 1] += lambda[idx];
  }

  for (idx = 0; idx < mLB; idx++) {
    i = finiteLB[idx];
    workspace[i - 1] -= lambda[mFixed + idx];
  }

  iL0 = mFixed + mLB;
  for (idx = 0; idx < mUB; idx++) {
    i = finiteUB[idx];
    workspace[i - 1] += lambda[iL0 + idx];
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
  int ia;
  int iac;
  int iy;
  int lda;
  switch (obj->objtype) {
   case 5:
    {
      int i;
      i = obj->nvar;
      if (i - 2 >= 0) {
        memset(&obj->grad[0], 0, (i - 1) * sizeof(double));
      }

      obj->grad[obj->nvar - 1] = obj->gammaScalar;
    }
    break;

   case 3:
    {
      int i;
      iy = obj->nvar - 1;
      lda = obj->nvar;
      if (obj->nvar != 0) {
        int ix;
        memset(&obj->Hx[0], 0, (iy + 1) * sizeof(double));
        ix = 0;
        i = obj->nvar * (obj->nvar - 1) + 1;
        for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
          int i1;
          i1 = iac + iy;
          for (ia = iac; ia <= i1; ia++) {
            int i2;
            i2 = ia - iac;
            obj->Hx[i2] += H[ia - 1] * x[ix];
          }

          ix++;
        }
      }

      i = obj->nvar;
      if (i - 1 >= 0) {
        memcpy(&obj->grad[0], &obj->Hx[0], i * sizeof(double));
      }

      if (obj->hasLinear && (obj->nvar >= 1)) {
        i = obj->nvar - 1;
        for (lda = 0; lda <= i; lda++) {
          obj->grad[lda] += f[lda];
        }
      }
    }
    break;

   default:
    {
      int i;
      int i1;
      iy = obj->nvar - 1;
      lda = obj->nvar;
      if (obj->nvar != 0) {
        int ix;
        memset(&obj->Hx[0], 0, (iy + 1) * sizeof(double));
        ix = 0;
        i = obj->nvar * (obj->nvar - 1) + 1;
        for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
          i1 = iac + iy;
          for (ia = iac; ia <= i1; ia++) {
            int i2;
            i2 = ia - iac;
            obj->Hx[i2] += H[ia - 1] * x[ix];
          }

          ix++;
        }
      }

      i = obj->nvar + 1;
      for (iy = i; iy < 16; iy++) {
        obj->Hx[iy - 1] = obj->beta * x[iy - 1];
      }

      memcpy(&obj->grad[0], &obj->Hx[0], 15U * sizeof(double));
      if (obj->hasLinear && (obj->nvar >= 1)) {
        i = obj->nvar - 1;
        for (lda = 0; lda <= i; lda++) {
          obj->grad[lda] += f[lda];
        }
      }

      if (15 - obj->nvar >= 1) {
        iy = obj->nvar;
        i = 14 - obj->nvar;
        for (lda = 0; lda <= i; lda++) {
          i1 = iy + lda;
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
  int i;
  int iQR0;
  int ia;
  int idx;
  int m;
  int n;
  i = obj->minRowCol;
  for (idx = 0; idx < i; idx++) {
    iQR0 = 31 * idx + idx;
    n = obj->mrows - idx;
    if (n - 2 >= 0) {
      memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1], (((n + iQR0) - iQR0) - 1) *
             sizeof(double));
    }
  }

  m = obj->mrows;
  n = obj->minRowCol;
  if (nrows >= 1) {
    int i1;
    int itau;
    i = nrows - 1;
    for (idx = n; idx <= i; idx++) {
      ia = idx * 31;
      i1 = m - 1;
      memset(&obj->Q[ia], 0, (((i1 + ia) - ia) + 1) * sizeof(double));
      obj->Q[ia + idx] = 1.0;
    }

    itau = obj->minRowCol - 1;
    memset(&work[0], 0, 31U * sizeof(double));
    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * 31;
      if (b_i < nrows) {
        int lastc;
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
              memset(&work[0], 0, (lastc + 1) * sizeof(double));
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

          if (!(-obj->tau[itau] == 0.0)) {
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
        for (n = iQR0; n <= i; n++) {
          obj->Q[n - 1] *= -obj->tau[itau];
        }
      }

      obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
      for (idx = 0; idx <= b_i - 2; idx++) {
        obj->Q[(iaii - idx) - 2] = 0.0;
      }

      itau--;
    }
  }
}

/*
 * Arguments    : const b_captured_var *dv_global
 *                const captured_var *gain_theta
 *                const captured_var *Cl_alpha
 *                const captured_var *S
 *                const captured_var *V
 *                const captured_var *rho
 *                const captured_var *flight_path_angle
 *                const captured_var *Beta
 *                const captured_var *K_Cd
 *                const captured_var *Cd_zero
 *                const captured_var *gain_phi
 *                const captured_var *K_p_T
 *                const captured_var *gain_motor
 *                const captured_var *gain_el
 *                const captured_var *gain_az
 *                const captured_var *m
 *                const captured_var *I_zz
 *                const captured_var *p
 *                const captured_var *r
 *                const captured_var *I_xx
 *                const captured_var *I_yy
 *                const captured_var *l_z
 *                const captured_var *K_p_M
 *                const captured_var *Cm_zero
 *                const captured_var *wing_chord
 *                const captured_var *l_4
 *                const captured_var *l_3
 *                const captured_var *Cm_alpha
 *                const captured_var *q
 *                const captured_var *l_1
 *                const captured_var *l_2
 *                const captured_var *CL_aileron
 *                const captured_var *gain_ailerons
 *                const captured_var *W_dv_4
 *                const captured_var *W_dv_6
 *                const captured_var *W_act_motor
 *                const captured_var *gamma_quadratic_du
 *                const captured_var *desired_motor_value
 *                const captured_var *W_dv_5
 *                const captured_var *W_dv_3
 *                const captured_var *W_dv_1
 *                const captured_var *W_dv_2
 *                const captured_var *W_act_tilt_el
 *                const captured_var *desired_el_value
 *                const captured_var *W_act_tilt_az
 *                const captured_var *desired_az_value
 *                const captured_var *W_act_theta
 *                const captured_var *desired_theta_value
 *                const captured_var *W_act_phi
 *                const captured_var *desired_phi_value
 *                const captured_var *W_act_ailerons
 *                const captured_var *desired_ailerons_value
 *                const double u_in[15]
 *                double *cost
 *                double gradient[15]
 * Return Type  : void
 */
static void compute_cost_and_gradient_fcn(const b_captured_var *dv_global, const
  captured_var *gain_theta, const captured_var *Cl_alpha, const captured_var *S,
  const captured_var *V, const captured_var *rho, const captured_var
  *flight_path_angle, const captured_var *Beta, const captured_var *K_Cd, const
  captured_var *Cd_zero, const captured_var *gain_phi, const captured_var *K_p_T,
  const captured_var *gain_motor, const captured_var *gain_el, const
  captured_var *gain_az, const captured_var *m, const captured_var *I_zz, const
  captured_var *p, const captured_var *r, const captured_var *I_xx, const
  captured_var *I_yy, const captured_var *l_z, const captured_var *K_p_M, const
  captured_var *Cm_zero, const captured_var *wing_chord, const captured_var *l_4,
  const captured_var *l_3, const captured_var *Cm_alpha, const captured_var *q,
  const captured_var *l_1, const captured_var *l_2, const captured_var
  *CL_aileron, const captured_var *gain_ailerons, const captured_var *W_dv_4,
  const captured_var *W_dv_6, const captured_var *W_act_motor, const
  captured_var *gamma_quadratic_du, const captured_var *desired_motor_value,
  const captured_var *W_dv_5, const captured_var *W_dv_3, const captured_var
  *W_dv_1, const captured_var *W_dv_2, const captured_var *W_act_tilt_el, const
  captured_var *desired_el_value, const captured_var *W_act_tilt_az, const
  captured_var *desired_az_value, const captured_var *W_act_theta, const
  captured_var *desired_theta_value, const captured_var *W_act_phi, const
  captured_var *desired_phi_value, const captured_var *W_act_ailerons, const
  captured_var *desired_ailerons_value, const double u_in[15], double *cost,
  double gradient[15])
{
  double gradient_fcn_15[16];
  double a;
  double a_tmp;
  double ab_a;
  double ab_sigma_1_tmp;
  double b_a;
  double b_a_tmp;
  double b_gradient_fcn_15_tmp;
  double b_gradient_fcn_15_tmp_tmp;
  double b_sigma_15_tmp;
  double b_sigma_1_tmp;
  double b_sigma_1_tmp_tmp;
  double b_sigma_2_tmp;
  double b_sigma_3_tmp;
  double b_sigma_5_tmp;
  double bb_a;
  double bb_sigma_1_tmp;
  double c_a;
  double c_a_tmp;
  double c_gradient_fcn_15_tmp;
  double c_gradient_fcn_15_tmp_tmp;
  double c_sigma_15_tmp;
  double c_sigma_1_tmp;
  double c_sigma_1_tmp_tmp;
  double c_sigma_3_tmp;
  double c_sigma_5_tmp;
  double cb_a;
  double cb_sigma_1_tmp;
  double d_a;
  double d_a_tmp;
  double d_gradient_fcn_15_tmp;
  double d_gradient_fcn_15_tmp_tmp;
  double d_sigma_1_tmp;
  double d_sigma_1_tmp_tmp;
  double d_sigma_3_tmp;
  double d_sigma_5_tmp;
  double db_a;
  double db_sigma_1_tmp;
  double e_a;
  double e_a_tmp;
  double e_gradient_fcn_15_tmp;
  double e_gradient_fcn_15_tmp_tmp;
  double e_sigma_1_tmp;
  double e_sigma_1_tmp_tmp;
  double e_sigma_3_tmp;
  double eb_a;
  double eb_sigma_1_tmp;
  double f_a;
  double f_a_tmp;
  double f_gradient_fcn_15_tmp;
  double f_gradient_fcn_15_tmp_tmp;
  double f_sigma_1_tmp;
  double f_sigma_1_tmp_tmp;
  double f_sigma_3_tmp;
  double fb_a;
  double fb_sigma_1_tmp;
  double g_a;
  double g_a_tmp;
  double g_gradient_fcn_15_tmp;
  double g_sigma_1_tmp;
  double g_sigma_1_tmp_tmp;
  double g_sigma_3_tmp;
  double gb_a;
  double gb_sigma_1_tmp;
  double gradient_fcn_15_tmp;
  double gradient_fcn_15_tmp_tmp;
  double h_a;
  double h_gradient_fcn_15_tmp;
  double h_sigma_1_tmp;
  double h_sigma_1_tmp_tmp;
  double h_sigma_3_tmp;
  double hb_a;
  double hb_sigma_1_tmp;
  double i_a;
  double i_gradient_fcn_15_tmp;
  double i_sigma_1_tmp;
  double i_sigma_1_tmp_tmp;
  double i_sigma_3_tmp;
  double ib_a;
  double ib_sigma_1_tmp;
  double j_a;
  double j_gradient_fcn_15_tmp;
  double j_sigma_1_tmp;
  double j_sigma_1_tmp_tmp;
  double j_sigma_3_tmp;
  double jb_a;
  double jb_sigma_1_tmp;
  double k_a;
  double k_gradient_fcn_15_tmp;
  double k_sigma_1_tmp;
  double k_sigma_1_tmp_tmp;
  double k_sigma_3_tmp;
  double kb_a;
  double kb_sigma_1_tmp;
  double l_a;
  double l_gradient_fcn_15_tmp;
  double l_sigma_1_tmp;
  double l_sigma_1_tmp_tmp;
  double l_sigma_3_tmp;
  double lb_a;
  double m_a;
  double m_gradient_fcn_15_tmp;
  double m_sigma_1_tmp;
  double m_sigma_1_tmp_tmp;
  double m_sigma_3_tmp;
  double mb_a;
  double n_a;
  double n_gradient_fcn_15_tmp;
  double n_sigma_1_tmp;
  double n_sigma_1_tmp_tmp;
  double n_sigma_3_tmp;
  double nb_a;
  double o_a;
  double o_gradient_fcn_15_tmp;
  double o_sigma_1_tmp;
  double o_sigma_1_tmp_tmp;
  double ob_a;
  double p_a;
  double p_gradient_fcn_15_tmp;
  double p_sigma_1_tmp;
  double p_sigma_1_tmp_tmp;
  double pb_a;
  double q_a;
  double q_sigma_1_tmp;
  double q_sigma_1_tmp_tmp;
  double qb_a;
  double r_a;
  double r_sigma_1_tmp;
  double r_sigma_1_tmp_tmp;
  double rb_a;
  double s_a;
  double s_sigma_1_tmp;
  double s_sigma_1_tmp_tmp;
  double sb_a;
  double sigma_1;
  double sigma_14;
  double sigma_15;
  double sigma_15_tmp;
  double sigma_1_tmp;
  double sigma_1_tmp_tmp;
  double sigma_1_tmp_tmp_tmp;
  double sigma_2;
  double sigma_2_tmp;
  double sigma_3;
  double sigma_3_tmp;
  double sigma_4;
  double sigma_4_tmp;
  double sigma_5;
  double sigma_5_tmp;
  double sigma_6;
  double sigma_7;
  double sigma_7_tmp;
  double t_a;
  double t_sigma_1_tmp;
  double t_sigma_1_tmp_tmp;
  double u_a;
  double u_sigma_1_tmp;
  double v_a;
  double v_sigma_1_tmp;
  double w_a;
  double w_sigma_1_tmp;
  double x_a;
  double x_sigma_1_tmp;
  double y_a;
  double y_sigma_1_tmp;
  sigma_1_tmp_tmp_tmp = u_in[12] * gain_theta->contents;
  sigma_1_tmp_tmp = flight_path_angle->contents - sigma_1_tmp_tmp_tmp;
  b_sigma_1_tmp_tmp = cos(sigma_1_tmp_tmp);
  c_sigma_1_tmp_tmp = sin(sigma_1_tmp_tmp);
  d_sigma_1_tmp_tmp = cos(Beta->contents);
  e_sigma_1_tmp_tmp = cos(sigma_1_tmp_tmp_tmp);
  f_sigma_1_tmp_tmp = u_in[13] * gain_phi->contents;
  g_sigma_1_tmp_tmp = cos(f_sigma_1_tmp_tmp);
  h_sigma_1_tmp_tmp = sin(sigma_1_tmp_tmp_tmp);
  i_sigma_1_tmp_tmp = u_in[4] * gain_el->contents;
  sigma_1_tmp = cos(i_sigma_1_tmp_tmp);
  j_sigma_1_tmp_tmp = u_in[5] * gain_el->contents;
  b_sigma_1_tmp = cos(j_sigma_1_tmp_tmp);
  k_sigma_1_tmp_tmp = u_in[6] * gain_el->contents;
  c_sigma_1_tmp = cos(k_sigma_1_tmp_tmp);
  l_sigma_1_tmp_tmp = u_in[7] * gain_el->contents;
  d_sigma_1_tmp = cos(l_sigma_1_tmp_tmp);
  m_sigma_1_tmp_tmp = sin(f_sigma_1_tmp_tmp);
  e_sigma_1_tmp = sin(Beta->contents);
  f_sigma_1_tmp = sin(i_sigma_1_tmp_tmp);
  g_sigma_1_tmp = sin(j_sigma_1_tmp_tmp);
  h_sigma_1_tmp = sin(k_sigma_1_tmp_tmp);
  i_sigma_1_tmp = sin(l_sigma_1_tmp_tmp);
  n_sigma_1_tmp_tmp = u_in[8] * gain_az->contents;
  j_sigma_1_tmp = sin(n_sigma_1_tmp_tmp);
  o_sigma_1_tmp_tmp = u_in[9] * gain_az->contents;
  k_sigma_1_tmp = sin(o_sigma_1_tmp_tmp);
  p_sigma_1_tmp_tmp = u_in[10] * gain_az->contents;
  l_sigma_1_tmp = sin(p_sigma_1_tmp_tmp);
  q_sigma_1_tmp_tmp = u_in[11] * gain_az->contents;
  m_sigma_1_tmp = sin(q_sigma_1_tmp_tmp);
  n_sigma_1_tmp = cos(n_sigma_1_tmp_tmp);
  o_sigma_1_tmp = cos(o_sigma_1_tmp_tmp);
  p_sigma_1_tmp = cos(p_sigma_1_tmp_tmp);
  q_sigma_1_tmp = cos(q_sigma_1_tmp_tmp);
  a_tmp = V->contents;
  b_a_tmp = Cl_alpha->contents;
  a = gain_theta->contents;
  b_a = flight_path_angle->contents;
  c_a = gain_theta->contents;
  d_a = flight_path_angle->contents;
  c_a_tmp = gain_motor->contents;
  e_a = gain_theta->contents;
  f_a = flight_path_angle->contents;
  r_sigma_1_tmp = u_in[0] * u_in[0];
  s_sigma_1_tmp = u_in[1] * u_in[1];
  t_sigma_1_tmp = u_in[2] * u_in[2];
  u_sigma_1_tmp = u_in[3] * u_in[3];
  v_sigma_1_tmp = ((f_sigma_1_tmp * r_sigma_1_tmp + g_sigma_1_tmp *
                    s_sigma_1_tmp) + h_sigma_1_tmp * t_sigma_1_tmp) +
    i_sigma_1_tmp * u_sigma_1_tmp;
  w_sigma_1_tmp = a_tmp * a_tmp;
  x_sigma_1_tmp = b_a_tmp * b_a_tmp;
  y_sigma_1_tmp = K_Cd->contents * x_sigma_1_tmp;
  r_sigma_1_tmp_tmp = Cl_alpha->contents * S->contents * w_sigma_1_tmp;
  ab_sigma_1_tmp = r_sigma_1_tmp_tmp * rho->contents;
  s_sigma_1_tmp_tmp = S->contents * w_sigma_1_tmp;
  bb_sigma_1_tmp = s_sigma_1_tmp_tmp * rho->contents;
  cb_sigma_1_tmp = y_sigma_1_tmp * (u_in[12] * u_in[12]);
  db_sigma_1_tmp = 2.0 * K_Cd->contents * x_sigma_1_tmp * u_in[12] *
    flight_path_angle->contents * gain_theta->contents;
  t_sigma_1_tmp_tmp = c_a_tmp * c_a_tmp;
  eb_sigma_1_tmp = K_p_T->contents * t_sigma_1_tmp_tmp;
  fb_sigma_1_tmp = bb_sigma_1_tmp * c_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp;
  gb_sigma_1_tmp = ab_sigma_1_tmp * b_sigma_1_tmp_tmp * sigma_1_tmp_tmp / 2.0;
  hb_sigma_1_tmp = eb_sigma_1_tmp * m_sigma_1_tmp_tmp;
  ib_sigma_1_tmp = eb_sigma_1_tmp * g_sigma_1_tmp_tmp;
  jb_sigma_1_tmp = ab_sigma_1_tmp * c_sigma_1_tmp_tmp * sigma_1_tmp_tmp / 2.0;
  kb_sigma_1_tmp = eb_sigma_1_tmp * e_sigma_1_tmp_tmp;
  sigma_1 = dv_global->contents[0] - (((((e_sigma_1_tmp_tmp * (jb_sigma_1_tmp -
    bb_sigma_1_tmp * b_sigma_1_tmp_tmp * d_sigma_1_tmp_tmp * (((cb_sigma_1_tmp *
    (a * a) - db_sigma_1_tmp) + y_sigma_1_tmp * (b_a * b_a)) + Cd_zero->contents)
    / 2.0) + g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp * (gb_sigma_1_tmp +
    fb_sigma_1_tmp * (((cb_sigma_1_tmp * (c_a * c_a) - db_sigma_1_tmp) +
                       y_sigma_1_tmp * (d_a * d_a)) + Cd_zero->contents) / 2.0))
    - kb_sigma_1_tmp * v_sigma_1_tmp) - ib_sigma_1_tmp * h_sigma_1_tmp_tmp *
    (((sigma_1_tmp * n_sigma_1_tmp * r_sigma_1_tmp + b_sigma_1_tmp *
       o_sigma_1_tmp * s_sigma_1_tmp) + c_sigma_1_tmp * p_sigma_1_tmp *
      t_sigma_1_tmp) + d_sigma_1_tmp * q_sigma_1_tmp * u_sigma_1_tmp)) +
    hb_sigma_1_tmp * h_sigma_1_tmp_tmp * (((sigma_1_tmp * j_sigma_1_tmp *
    r_sigma_1_tmp + b_sigma_1_tmp * k_sigma_1_tmp * s_sigma_1_tmp) +
    c_sigma_1_tmp * l_sigma_1_tmp * t_sigma_1_tmp) + d_sigma_1_tmp *
    m_sigma_1_tmp * u_sigma_1_tmp)) - bb_sigma_1_tmp * m_sigma_1_tmp_tmp *
    h_sigma_1_tmp_tmp * e_sigma_1_tmp * (((cb_sigma_1_tmp * (e_a * e_a) -
    db_sigma_1_tmp) + y_sigma_1_tmp * (f_a * f_a)) + Cd_zero->contents) / 2.0) /
    m->contents;
  a = gain_theta->contents;
  b_a = flight_path_angle->contents;
  c_a = gain_theta->contents;
  d_a = flight_path_angle->contents;
  sigma_2_tmp = ((cos(u_in[4] * gain_el->contents) * cos(u_in[8] *
    gain_az->contents) * (u_in[0] * u_in[0]) + cos(u_in[5] * gain_el->contents) *
                  cos(u_in[9] * gain_az->contents) * (u_in[1] * u_in[1])) + cos
                 (u_in[6] * gain_el->contents) * cos(u_in[10] *
    gain_az->contents) * (u_in[2] * u_in[2])) + cos(u_in[7] * gain_el->contents)
    * cos(u_in[11] * gain_az->contents) * (u_in[3] * u_in[3]);
  b_sigma_2_tmp = ((cos(u_in[4] * gain_el->contents) * sin(u_in[8] *
    gain_az->contents) * (u_in[0] * u_in[0]) + cos(u_in[5] * gain_el->contents) *
                    sin(u_in[9] * gain_az->contents) * (u_in[1] * u_in[1])) +
                   cos(u_in[6] * gain_el->contents) * sin(u_in[10] *
    gain_az->contents) * (u_in[2] * u_in[2])) + cos(u_in[7] * gain_el->contents)
    * sin(u_in[11] * gain_az->contents) * (u_in[3] * u_in[3]);
  sigma_2 = dv_global->contents[1] + (((m_sigma_1_tmp_tmp * (gb_sigma_1_tmp +
    fb_sigma_1_tmp * (((cb_sigma_1_tmp * (a * a) - db_sigma_1_tmp) +
                       y_sigma_1_tmp * (b_a * b_a)) + Cd_zero->contents) / 2.0)
    - hb_sigma_1_tmp * sigma_2_tmp) - ib_sigma_1_tmp * b_sigma_2_tmp) +
    bb_sigma_1_tmp * g_sigma_1_tmp_tmp * e_sigma_1_tmp * (((cb_sigma_1_tmp *
    (c_a * c_a) - db_sigma_1_tmp) + y_sigma_1_tmp * (d_a * d_a)) +
    Cd_zero->contents) / 2.0) / m->contents;
  sigma_3_tmp = 2.0 * K_p_T->contents;
  hb_sigma_1_tmp = 2.0 * K_p_M->contents;
  b_sigma_3_tmp = sigma_3_tmp * r_sigma_1_tmp;
  c_sigma_3_tmp = sigma_3_tmp * s_sigma_1_tmp;
  d_sigma_3_tmp = sigma_3_tmp * t_sigma_1_tmp;
  e_sigma_3_tmp = sigma_3_tmp * u_sigma_1_tmp;
  fb_sigma_1_tmp = Cm_alpha->contents * S->contents;
  b_a_tmp = 2.0 * I_yy->contents;
  a_tmp = 2.0 * I_xx->contents;
  f_sigma_3_tmp = 2.0 * I_zz->contents;
  g_sigma_3_tmp = b_sigma_3_tmp * t_sigma_1_tmp_tmp;
  h_sigma_3_tmp = c_sigma_3_tmp * t_sigma_1_tmp_tmp;
  c_a_tmp = d_sigma_3_tmp * t_sigma_1_tmp_tmp;
  i_sigma_3_tmp = e_sigma_3_tmp * t_sigma_1_tmp_tmp;
  j_sigma_3_tmp = hb_sigma_1_tmp * r_sigma_1_tmp * t_sigma_1_tmp_tmp;
  k_sigma_3_tmp = hb_sigma_1_tmp * s_sigma_1_tmp * t_sigma_1_tmp_tmp;
  l_sigma_3_tmp = hb_sigma_1_tmp * t_sigma_1_tmp * t_sigma_1_tmp_tmp;
  hb_sigma_1_tmp = hb_sigma_1_tmp * u_sigma_1_tmp * t_sigma_1_tmp_tmp;
  m_sigma_3_tmp = g_sigma_3_tmp * l_z->contents;
  a = h_sigma_3_tmp * l_z->contents;
  b_a = c_a_tmp * l_z->contents;
  c_a = i_sigma_3_tmp * l_z->contents;
  n_sigma_3_tmp = fb_sigma_1_tmp * w_sigma_1_tmp;
  sigma_3 = ((((((((((((((((f_sigma_3_tmp * p->contents * r->contents - a_tmp *
    p->contents * r->contents) - b_a_tmp * dv_global->contents[4]) +
    m_sigma_3_tmp * f_sigma_1_tmp) + a * g_sigma_1_tmp) + b_a * h_sigma_1_tmp) +
                       c_a * i_sigma_1_tmp) - j_sigma_3_tmp * sigma_1_tmp *
                      j_sigma_1_tmp) + k_sigma_3_tmp * b_sigma_1_tmp *
                     k_sigma_1_tmp) - l_sigma_3_tmp * c_sigma_1_tmp *
                    l_sigma_1_tmp) + hb_sigma_1_tmp * d_sigma_1_tmp *
                   m_sigma_1_tmp) + Cm_zero->contents * S->contents *
                  w_sigma_1_tmp * rho->contents * wing_chord->contents) +
                 g_sigma_3_tmp * l_4->contents * sigma_1_tmp * n_sigma_1_tmp) +
                h_sigma_3_tmp * l_4->contents * b_sigma_1_tmp * o_sigma_1_tmp) -
               c_a_tmp * l_3->contents * c_sigma_1_tmp * p_sigma_1_tmp) -
              i_sigma_3_tmp * l_3->contents * d_sigma_1_tmp * q_sigma_1_tmp) -
             n_sigma_3_tmp * flight_path_angle->contents * rho->contents *
             wing_chord->contents) + fb_sigma_1_tmp * u_in[12] * w_sigma_1_tmp *
    gain_theta->contents * rho->contents * wing_chord->contents;
  sigma_4_tmp = CL_aileron->contents * S->contents * w_sigma_1_tmp;
  sigma_4 = ((((((((((((((b_a_tmp * q->contents * r->contents - a_tmp *
    dv_global->contents[3]) - f_sigma_3_tmp * q->contents * r->contents) +
                        j_sigma_3_tmp * f_sigma_1_tmp) - k_sigma_3_tmp *
                       g_sigma_1_tmp) + l_sigma_3_tmp * h_sigma_1_tmp) -
                     hb_sigma_1_tmp * i_sigma_1_tmp) + g_sigma_3_tmp *
                    l_1->contents * sigma_1_tmp * n_sigma_1_tmp) - h_sigma_3_tmp
                   * l_1->contents * b_sigma_1_tmp * o_sigma_1_tmp) - c_a_tmp *
                  l_2->contents * c_sigma_1_tmp * p_sigma_1_tmp) + i_sigma_3_tmp
                 * l_2->contents * d_sigma_1_tmp * q_sigma_1_tmp) +
                m_sigma_3_tmp * sigma_1_tmp * j_sigma_1_tmp) + a * b_sigma_1_tmp
               * k_sigma_1_tmp) + b_a * c_sigma_1_tmp * l_sigma_1_tmp) + c_a *
             d_sigma_1_tmp * m_sigma_1_tmp) + sigma_4_tmp * u_in[14] *
    gain_ailerons->contents * rho->contents;
  sigma_5_tmp = K_p_T->contents * r_sigma_1_tmp;
  b_sigma_5_tmp = K_p_T->contents * s_sigma_1_tmp;
  c_sigma_5_tmp = K_p_T->contents * t_sigma_1_tmp;
  d_sigma_5_tmp = K_p_T->contents * u_sigma_1_tmp;
  fb_sigma_1_tmp = sigma_5_tmp * t_sigma_1_tmp_tmp;
  hb_sigma_1_tmp = b_sigma_5_tmp * t_sigma_1_tmp_tmp;
  b_a_tmp = c_sigma_5_tmp * t_sigma_1_tmp_tmp;
  a_tmp = d_sigma_5_tmp * t_sigma_1_tmp_tmp;
  sigma_5 = (((((((((((((I_zz->contents * dv_global->contents[5] -
    I_xx->contents * p->contents * q->contents) + I_yy->contents * p->contents *
                        q->contents) + fb_sigma_1_tmp * l_1->contents *
                       f_sigma_1_tmp) - hb_sigma_1_tmp * l_1->contents *
                      g_sigma_1_tmp) - b_a_tmp * l_2->contents * h_sigma_1_tmp)
                    + a_tmp * l_2->contents * i_sigma_1_tmp) - K_p_M->contents *
                   r_sigma_1_tmp * t_sigma_1_tmp_tmp * sigma_1_tmp *
                   n_sigma_1_tmp) + K_p_M->contents * s_sigma_1_tmp *
                  t_sigma_1_tmp_tmp * b_sigma_1_tmp * o_sigma_1_tmp) -
                 K_p_M->contents * t_sigma_1_tmp * t_sigma_1_tmp_tmp *
                 c_sigma_1_tmp * p_sigma_1_tmp) + K_p_M->contents *
                u_sigma_1_tmp * t_sigma_1_tmp_tmp * d_sigma_1_tmp *
                q_sigma_1_tmp) - fb_sigma_1_tmp * l_4->contents * sigma_1_tmp *
               j_sigma_1_tmp) - hb_sigma_1_tmp * l_4->contents * b_sigma_1_tmp *
              k_sigma_1_tmp) + b_a_tmp * l_3->contents * c_sigma_1_tmp *
             l_sigma_1_tmp) + a_tmp * l_3->contents * d_sigma_1_tmp *
    m_sigma_1_tmp;
  a = gain_theta->contents;
  b_a = flight_path_angle->contents;
  sigma_6 = ((cb_sigma_1_tmp * (a * a) - db_sigma_1_tmp) + y_sigma_1_tmp * (b_a *
              b_a)) + Cd_zero->contents;
  sigma_7_tmp = bb_sigma_1_tmp * sigma_6;
  sigma_7 = (100.0 * (((((h_sigma_1_tmp_tmp * (sigma_7_tmp * b_sigma_1_tmp_tmp *
    d_sigma_1_tmp_tmp / 2.0 - jb_sigma_1_tmp) + g_sigma_1_tmp_tmp *
    e_sigma_1_tmp_tmp * (gb_sigma_1_tmp + sigma_7_tmp * c_sigma_1_tmp_tmp *
    d_sigma_1_tmp_tmp / 2.0)) + eb_sigma_1_tmp * h_sigma_1_tmp_tmp *
    v_sigma_1_tmp) - ib_sigma_1_tmp * e_sigma_1_tmp_tmp * sigma_2_tmp) +
                       kb_sigma_1_tmp * m_sigma_1_tmp_tmp * b_sigma_2_tmp) -
                      sigma_7_tmp * e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp *
                      e_sigma_1_tmp / 2.0) / m->contents - 100.0 *
             dv_global->contents[2]) + 981.0;
  sigma_14 = 1.0 / m->contents;
  fb_sigma_1_tmp = r_sigma_1_tmp_tmp * gain_theta->contents * rho->contents;
  sigma_15_tmp = s_sigma_1_tmp_tmp * gain_theta->contents * rho->contents;
  b_sigma_15_tmp = sigma_15_tmp * sigma_6;
  c_sigma_15_tmp = y_sigma_1_tmp * S->contents * w_sigma_1_tmp *
    gain_theta->contents * rho->contents * sigma_1_tmp_tmp;
  sigma_15 = ((fb_sigma_1_tmp * b_sigma_1_tmp_tmp / 2.0 + b_sigma_15_tmp *
               d_sigma_1_tmp_tmp * b_sigma_1_tmp_tmp / 2.0) - fb_sigma_1_tmp *
              sigma_1_tmp_tmp * c_sigma_1_tmp_tmp / 2.0) + c_sigma_15_tmp *
    d_sigma_1_tmp_tmp * c_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp_tmp = f_sigma_1_tmp_tmp + n_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp = sin(gradient_fcn_15_tmp_tmp);
  b_gradient_fcn_15_tmp_tmp = f_sigma_1_tmp_tmp + o_sigma_1_tmp_tmp;
  b_gradient_fcn_15_tmp = sin(b_gradient_fcn_15_tmp_tmp);
  c_gradient_fcn_15_tmp_tmp = f_sigma_1_tmp_tmp + p_sigma_1_tmp_tmp;
  c_gradient_fcn_15_tmp = sin(c_gradient_fcn_15_tmp_tmp);
  d_gradient_fcn_15_tmp_tmp = f_sigma_1_tmp_tmp + q_sigma_1_tmp_tmp;
  d_gradient_fcn_15_tmp = sin(d_gradient_fcn_15_tmp_tmp);
  a_tmp = W_dv_4->contents;
  a = I_xx->contents;
  b_a_tmp = W_dv_6->contents;
  b_a = I_zz->contents;
  c_a_tmp = W_act_motor->contents;
  hb_sigma_1_tmp = W_dv_5->contents;
  c_a = I_yy->contents;
  f_sigma_3_tmp = W_dv_3->contents;
  g_sigma_3_tmp = W_dv_1->contents;
  h_sigma_3_tmp = W_dv_2->contents;
  d_a = I_yy->contents;
  e_a = I_zz->contents;
  f_a = I_xx->contents;
  i_sigma_3_tmp = I_xx->contents;
  j_sigma_3_tmp = I_zz->contents;
  k_sigma_3_tmp = I_yy->contents;
  l_sigma_3_tmp = I_zz->contents;
  m_sigma_3_tmp = I_xx->contents;
  cb_sigma_1_tmp = I_yy->contents;
  bb_sigma_1_tmp = I_zz->contents;
  db_sigma_1_tmp = W_act_tilt_el->contents;
  eb_sigma_1_tmp = I_xx->contents;
  gb_sigma_1_tmp = I_yy->contents;
  jb_sigma_1_tmp = I_zz->contents;
  kb_sigma_1_tmp = I_xx->contents;
  r_sigma_1_tmp_tmp = I_yy->contents;
  g_a = I_xx->contents;
  h_a = I_zz->contents;
  i_a = I_yy->contents;
  j_a = I_yy->contents;
  k_a = I_zz->contents;
  l_a = I_xx->contents;
  m_a = I_zz->contents;
  n_a = I_yy->contents;
  d_a_tmp = W_act_tilt_az->contents;
  o_a = I_xx->contents;
  p_a = I_yy->contents;
  q_a = I_zz->contents;
  r_a = I_xx->contents;
  s_a = I_zz->contents;
  t_a = I_yy->contents;
  u_a = I_xx->contents;
  v_a = I_yy->contents;
  w_a = I_zz->contents;
  x_a = I_xx->contents;
  e_a_tmp = W_act_theta->contents;
  y_a = I_yy->contents;
  f_a_tmp = W_act_phi->contents;
  g_a_tmp = W_act_ailerons->contents;
  ab_a = I_xx->contents;
  fb_sigma_1_tmp = desired_motor_value->contents / gain_motor->contents;
  bb_a = u_in[0] - fb_sigma_1_tmp;
  cb_a = u_in[1] - fb_sigma_1_tmp;
  db_a = u_in[2] - fb_sigma_1_tmp;
  eb_a = u_in[3] - fb_sigma_1_tmp;
  fb_a = u_in[13] - desired_phi_value->contents / gain_phi->contents;
  gb_a = u_in[12] - desired_theta_value->contents / gain_theta->contents;
  fb_sigma_1_tmp = desired_el_value->contents / gain_el->contents;
  hb_a = u_in[4] - fb_sigma_1_tmp;
  ib_a = u_in[5] - fb_sigma_1_tmp;
  jb_a = u_in[6] - fb_sigma_1_tmp;
  kb_a = u_in[7] - fb_sigma_1_tmp;
  lb_a = u_in[14] - desired_ailerons_value->contents / gain_ailerons->contents;
  fb_sigma_1_tmp = desired_az_value->contents / gain_az->contents;
  mb_a = u_in[8] - fb_sigma_1_tmp;
  nb_a = u_in[9] - fb_sigma_1_tmp;
  ob_a = u_in[10] - fb_sigma_1_tmp;
  pb_a = u_in[11] - fb_sigma_1_tmp;
  qb_a = I_xx->contents;
  rb_a = I_yy->contents;
  sb_a = I_zz->contents;
  e_gradient_fcn_15_tmp = K_p_T->contents * l_1->contents;
  f_gradient_fcn_15_tmp = K_p_T->contents * l_z->contents;
  g_gradient_fcn_15_tmp = cos(u_in[4] * gain_el->contents) * cos(u_in[8] *
    gain_az->contents);
  h_gradient_fcn_15_tmp = cos(u_in[4] * gain_el->contents) * sin(u_in[8] *
    gain_az->contents);
  e_gradient_fcn_15_tmp_tmp = 4.0 * K_p_T->contents;
  i_gradient_fcn_15_tmp = e_gradient_fcn_15_tmp_tmp * u_in[0];
  j_gradient_fcn_15_tmp = K_p_T->contents * l_4->contents;
  k_gradient_fcn_15_tmp = K_p_M->contents * cos(u_in[4] * gain_el->contents);
  l_gradient_fcn_15_tmp = hb_sigma_1_tmp * hb_sigma_1_tmp;
  m_gradient_fcn_15_tmp = b_a_tmp * b_a_tmp;
  n_gradient_fcn_15_tmp = a_tmp * a_tmp;
  f_gradient_fcn_15_tmp_tmp = c_a_tmp * c_a_tmp;
  o_gradient_fcn_15_tmp = 2.0 * f_gradient_fcn_15_tmp_tmp *
    gamma_quadratic_du->contents;
  p_gradient_fcn_15_tmp = f_sigma_3_tmp * f_sigma_3_tmp;
  y_sigma_1_tmp = g_sigma_3_tmp * g_sigma_3_tmp;
  s_sigma_1_tmp_tmp = h_sigma_3_tmp * h_sigma_3_tmp;
  gradient_fcn_15[0] = (((((2.0 * u_in[0] * n_gradient_fcn_15_tmp * sigma_4 *
    t_sigma_1_tmp_tmp * ((K_p_M->contents * f_sigma_1_tmp +
    e_gradient_fcn_15_tmp * sigma_1_tmp * n_sigma_1_tmp) + f_gradient_fcn_15_tmp
    * sigma_1_tmp * j_sigma_1_tmp) / (a * a) - 4.0 * u_in[0] *
    m_gradient_fcn_15_tmp * sigma_5 * t_sigma_1_tmp_tmp * ((K_p_M->contents *
    sigma_1_tmp * n_sigma_1_tmp - e_gradient_fcn_15_tmp * f_sigma_1_tmp) +
    j_gradient_fcn_15_tmp * sigma_1_tmp * j_sigma_1_tmp) / (b_a * b_a)) -
    o_gradient_fcn_15_tmp * (desired_motor_value->contents - u_in[0] *
    gain_motor->contents) / gain_motor->contents) + 2.0 * u_in[0] *
    l_gradient_fcn_15_tmp * sigma_3 * t_sigma_1_tmp_tmp *
    ((f_gradient_fcn_15_tmp * f_sigma_1_tmp - k_gradient_fcn_15_tmp *
      j_sigma_1_tmp) + K_p_T->contents * l_4->contents * cos(u_in[4] *
    gain_el->contents) * n_sigma_1_tmp) / (c_a * c_a)) + K_p_T->contents * u_in
    [0] * p_gradient_fcn_15_tmp * sigma_7 * t_sigma_1_tmp_tmp * sigma_14 *
    ((f_sigma_1_tmp * h_sigma_1_tmp_tmp - g_gradient_fcn_15_tmp *
      e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) + h_gradient_fcn_15_tmp *
     e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) / 25.0) + i_gradient_fcn_15_tmp *
                        y_sigma_1_tmp * sigma_1 * t_sigma_1_tmp_tmp * sigma_14 *
                        ((f_sigma_1_tmp * e_sigma_1_tmp_tmp +
    g_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) -
    h_gradient_fcn_15_tmp * h_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp)) -
    i_gradient_fcn_15_tmp * s_sigma_1_tmp_tmp * sigma_2 * t_sigma_1_tmp_tmp *
    sigma_14 * gradient_fcn_15_tmp * sigma_1_tmp;
  g_gradient_fcn_15_tmp = cos(u_in[5] * gain_el->contents) * cos(u_in[9] *
    gain_az->contents);
  h_gradient_fcn_15_tmp = cos(u_in[5] * gain_el->contents) * sin(u_in[9] *
    gain_az->contents);
  i_gradient_fcn_15_tmp = e_gradient_fcn_15_tmp_tmp * u_in[1];
  ib_sigma_1_tmp = K_p_M->contents * cos(u_in[5] * gain_el->contents);
  gradient_fcn_15[1] = (((((2.0 * u_in[1] * l_gradient_fcn_15_tmp * sigma_3 *
    t_sigma_1_tmp_tmp * ((f_gradient_fcn_15_tmp * g_sigma_1_tmp +
    K_p_M->contents * b_sigma_1_tmp * k_sigma_1_tmp) + j_gradient_fcn_15_tmp *
    b_sigma_1_tmp * o_sigma_1_tmp) / (d_a * d_a) - 4.0 * u_in[1] *
    m_gradient_fcn_15_tmp * sigma_5 * t_sigma_1_tmp_tmp *
    ((e_gradient_fcn_15_tmp * g_sigma_1_tmp - ib_sigma_1_tmp * o_sigma_1_tmp) +
     K_p_T->contents * l_4->contents * cos(u_in[5] * gain_el->contents) *
     k_sigma_1_tmp) / (e_a * e_a)) - 2.0 * u_in[1] * n_gradient_fcn_15_tmp *
    sigma_4 * t_sigma_1_tmp_tmp * ((K_p_M->contents * g_sigma_1_tmp +
    e_gradient_fcn_15_tmp * b_sigma_1_tmp * o_sigma_1_tmp) -
    f_gradient_fcn_15_tmp * b_sigma_1_tmp * k_sigma_1_tmp) / (f_a * f_a)) -
    o_gradient_fcn_15_tmp * (desired_motor_value->contents - u_in[1] *
    gain_motor->contents) / gain_motor->contents) + K_p_T->contents * u_in[1] *
    p_gradient_fcn_15_tmp * sigma_7 * t_sigma_1_tmp_tmp * sigma_14 *
    ((g_sigma_1_tmp * h_sigma_1_tmp_tmp - g_gradient_fcn_15_tmp *
      e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) + h_gradient_fcn_15_tmp *
     e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) / 25.0) + i_gradient_fcn_15_tmp *
                        y_sigma_1_tmp * sigma_1 * t_sigma_1_tmp_tmp * sigma_14 *
                        ((g_sigma_1_tmp * e_sigma_1_tmp_tmp +
    g_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) -
    h_gradient_fcn_15_tmp * h_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp)) -
    i_gradient_fcn_15_tmp * s_sigma_1_tmp_tmp * sigma_2 * t_sigma_1_tmp_tmp *
    sigma_14 * b_gradient_fcn_15_tmp * b_sigma_1_tmp;
  g_gradient_fcn_15_tmp = K_p_T->contents * l_2->contents;
  h_gradient_fcn_15_tmp = cos(u_in[6] * gain_el->contents) * cos(u_in[10] *
    gain_az->contents);
  i_gradient_fcn_15_tmp = cos(u_in[6] * gain_el->contents) * sin(u_in[10] *
    gain_az->contents);
  a = e_gradient_fcn_15_tmp_tmp * u_in[2];
  b_a = K_p_T->contents * l_3->contents;
  c_a = K_p_M->contents * cos(u_in[6] * gain_el->contents);
  gradient_fcn_15[2] = (((((2.0 * u_in[2] * n_gradient_fcn_15_tmp * sigma_4 *
    t_sigma_1_tmp_tmp * ((K_p_M->contents * h_sigma_1_tmp -
    g_gradient_fcn_15_tmp * c_sigma_1_tmp * p_sigma_1_tmp) +
    f_gradient_fcn_15_tmp * c_sigma_1_tmp * l_sigma_1_tmp) / (i_sigma_3_tmp *
    i_sigma_3_tmp) - 4.0 * u_in[2] * m_gradient_fcn_15_tmp * sigma_5 *
    t_sigma_1_tmp_tmp * ((g_gradient_fcn_15_tmp * h_sigma_1_tmp +
    K_p_M->contents * c_sigma_1_tmp * p_sigma_1_tmp) - b_a * c_sigma_1_tmp *
    l_sigma_1_tmp) / (j_sigma_3_tmp * j_sigma_3_tmp)) - o_gradient_fcn_15_tmp *
    (desired_motor_value->contents - u_in[2] * gain_motor->contents) /
    gain_motor->contents) - 2.0 * u_in[2] * l_gradient_fcn_15_tmp * sigma_3 *
    t_sigma_1_tmp_tmp * ((c_a * l_sigma_1_tmp - f_gradient_fcn_15_tmp *
    h_sigma_1_tmp) + K_p_T->contents * l_3->contents * cos(u_in[6] *
    gain_el->contents) * p_sigma_1_tmp) / (k_sigma_3_tmp * k_sigma_3_tmp)) +
    K_p_T->contents * u_in[2] * p_gradient_fcn_15_tmp * sigma_7 *
    t_sigma_1_tmp_tmp * sigma_14 * ((h_sigma_1_tmp * h_sigma_1_tmp_tmp -
    h_gradient_fcn_15_tmp * e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) +
    i_gradient_fcn_15_tmp * e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) / 25.0) + a *
                        y_sigma_1_tmp * sigma_1 * t_sigma_1_tmp_tmp * sigma_14 *
                        ((h_sigma_1_tmp * e_sigma_1_tmp_tmp +
    h_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) -
    i_gradient_fcn_15_tmp * h_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp)) - a *
    s_sigma_1_tmp_tmp * sigma_2 * t_sigma_1_tmp_tmp * sigma_14 *
    c_gradient_fcn_15_tmp * c_sigma_1_tmp;
  h_gradient_fcn_15_tmp = cos(u_in[7] * gain_el->contents) * cos(u_in[11] *
    gain_az->contents);
  i_gradient_fcn_15_tmp = cos(u_in[7] * gain_el->contents) * sin(u_in[11] *
    gain_az->contents);
  a = e_gradient_fcn_15_tmp_tmp * u_in[3];
  fb_sigma_1_tmp = K_p_M->contents * cos(u_in[7] * gain_el->contents);
  gradient_fcn_15[3] = (((((4.0 * u_in[3] * m_gradient_fcn_15_tmp * sigma_5 *
    t_sigma_1_tmp_tmp * ((g_gradient_fcn_15_tmp * i_sigma_1_tmp +
    K_p_M->contents * d_sigma_1_tmp * q_sigma_1_tmp) + b_a * d_sigma_1_tmp *
    m_sigma_1_tmp) / (l_sigma_3_tmp * l_sigma_3_tmp) - o_gradient_fcn_15_tmp *
    (desired_motor_value->contents - u_in[3] * gain_motor->contents) /
    gain_motor->contents) + 2.0 * u_in[3] * n_gradient_fcn_15_tmp * sigma_4 *
    t_sigma_1_tmp_tmp * ((g_gradient_fcn_15_tmp * d_sigma_1_tmp * q_sigma_1_tmp
    - K_p_M->contents * i_sigma_1_tmp) + f_gradient_fcn_15_tmp * d_sigma_1_tmp *
    m_sigma_1_tmp) / (m_sigma_3_tmp * m_sigma_3_tmp)) + 2.0 * u_in[3] *
    l_gradient_fcn_15_tmp * sigma_3 * t_sigma_1_tmp_tmp *
    ((f_gradient_fcn_15_tmp * i_sigma_1_tmp + fb_sigma_1_tmp * m_sigma_1_tmp) -
     K_p_T->contents * l_3->contents * cos(u_in[7] * gain_el->contents) *
     q_sigma_1_tmp) / (cb_sigma_1_tmp * cb_sigma_1_tmp)) + K_p_T->contents *
    u_in[3] * p_gradient_fcn_15_tmp * sigma_7 * t_sigma_1_tmp_tmp * sigma_14 *
    ((i_sigma_1_tmp * h_sigma_1_tmp_tmp - h_gradient_fcn_15_tmp *
      e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) + i_gradient_fcn_15_tmp *
     e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) / 25.0) + a * y_sigma_1_tmp *
                        sigma_1 * t_sigma_1_tmp_tmp * sigma_14 * ((i_sigma_1_tmp
    * e_sigma_1_tmp_tmp + h_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp *
    h_sigma_1_tmp_tmp) - i_gradient_fcn_15_tmp * h_sigma_1_tmp_tmp *
    m_sigma_1_tmp_tmp)) - a * s_sigma_1_tmp_tmp * sigma_2 * t_sigma_1_tmp_tmp *
    sigma_14 * d_gradient_fcn_15_tmp * d_sigma_1_tmp;
  e_gradient_fcn_15_tmp_tmp = db_sigma_1_tmp * db_sigma_1_tmp;
  f_gradient_fcn_15_tmp = 2.0 * e_gradient_fcn_15_tmp_tmp *
    gamma_quadratic_du->contents;
  h_gradient_fcn_15_tmp = 2.0 * r_sigma_1_tmp * m_gradient_fcn_15_tmp;
  i_gradient_fcn_15_tmp = r_sigma_1_tmp * l_gradient_fcn_15_tmp;
  o_gradient_fcn_15_tmp = b_sigma_3_tmp * s_sigma_1_tmp_tmp;
  a = sigma_5_tmp * p_gradient_fcn_15_tmp;
  m_sigma_3_tmp = b_sigma_3_tmp * y_sigma_1_tmp;
  gradient_fcn_15[4] = (((((h_gradient_fcn_15_tmp * gain_el->contents * sigma_5 *
    t_sigma_1_tmp_tmp * ((K_p_T->contents * l_1->contents * cos(u_in[4] *
    gain_el->contents) + K_p_M->contents * n_sigma_1_tmp * f_sigma_1_tmp) +
    j_gradient_fcn_15_tmp * f_sigma_1_tmp * j_sigma_1_tmp) / (bb_sigma_1_tmp *
    bb_sigma_1_tmp) - f_gradient_fcn_15_tmp * (desired_el_value->contents -
    i_sigma_1_tmp_tmp) / gain_el->contents) - r_sigma_1_tmp *
    n_gradient_fcn_15_tmp * gain_el->contents * sigma_4 * t_sigma_1_tmp_tmp *
    ((e_gradient_fcn_15_tmp * n_sigma_1_tmp * f_sigma_1_tmp -
      k_gradient_fcn_15_tmp) + K_p_T->contents * l_z->contents * sin(u_in[4] *
    gain_el->contents) * j_sigma_1_tmp) / (eb_sigma_1_tmp * eb_sigma_1_tmp)) +
    i_gradient_fcn_15_tmp * gain_el->contents * sigma_3 * t_sigma_1_tmp_tmp *
    ((K_p_T->contents * l_z->contents * cos(u_in[4] * gain_el->contents) +
      K_p_M->contents * sin(u_in[4] * gain_el->contents) * j_sigma_1_tmp) -
     j_gradient_fcn_15_tmp * n_sigma_1_tmp * f_sigma_1_tmp) / (gb_sigma_1_tmp *
    gb_sigma_1_tmp)) + a * gain_el->contents * sigma_7 * t_sigma_1_tmp_tmp *
    sigma_14 * ((sigma_1_tmp * h_sigma_1_tmp_tmp + n_sigma_1_tmp * f_sigma_1_tmp
                 * e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) - f_sigma_1_tmp *
                j_sigma_1_tmp * e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) / 50.0) +
                        m_sigma_3_tmp * gain_el->contents * sigma_1 *
                        t_sigma_1_tmp_tmp * sigma_14 * ((sigma_1_tmp *
    e_sigma_1_tmp_tmp - cos(u_in[8] * gain_az->contents) * sin(u_in[4] *
    gain_el->contents) * g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) + sin(u_in[4] *
    gain_el->contents) * sin(u_in[8] * gain_az->contents) * h_sigma_1_tmp_tmp *
    m_sigma_1_tmp_tmp)) + o_gradient_fcn_15_tmp * gain_el->contents * sigma_2 *
    t_sigma_1_tmp_tmp * sigma_14 * gradient_fcn_15_tmp * f_sigma_1_tmp;
  k_gradient_fcn_15_tmp = s_sigma_1_tmp * l_gradient_fcn_15_tmp;
  j_sigma_3_tmp = 2.0 * s_sigma_1_tmp * m_gradient_fcn_15_tmp;
  k_sigma_3_tmp = c_sigma_3_tmp * s_sigma_1_tmp_tmp;
  l_sigma_3_tmp = b_sigma_5_tmp * p_gradient_fcn_15_tmp;
  i_sigma_3_tmp = c_sigma_3_tmp * y_sigma_1_tmp;
  gradient_fcn_15[5] = (((((l_sigma_3_tmp * gain_el->contents * sigma_7 *
    t_sigma_1_tmp_tmp * sigma_14 * ((b_sigma_1_tmp * h_sigma_1_tmp_tmp +
    o_sigma_1_tmp * g_sigma_1_tmp * e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) -
    g_sigma_1_tmp * k_sigma_1_tmp * e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) /
    50.0 - j_sigma_3_tmp * gain_el->contents * sigma_5 * t_sigma_1_tmp_tmp *
    ((K_p_T->contents * l_1->contents * cos(u_in[5] * gain_el->contents) +
      K_p_M->contents * o_sigma_1_tmp * g_sigma_1_tmp) - j_gradient_fcn_15_tmp *
     g_sigma_1_tmp * k_sigma_1_tmp) / (jb_sigma_1_tmp * jb_sigma_1_tmp)) -
    s_sigma_1_tmp * n_gradient_fcn_15_tmp * gain_el->contents * sigma_4 *
    t_sigma_1_tmp_tmp * ((ib_sigma_1_tmp - e_gradient_fcn_15_tmp * o_sigma_1_tmp
    * g_sigma_1_tmp) + K_p_T->contents * l_z->contents * sin(u_in[5] *
    gain_el->contents) * k_sigma_1_tmp) / (kb_sigma_1_tmp * kb_sigma_1_tmp)) -
    k_gradient_fcn_15_tmp * gain_el->contents * sigma_3 * t_sigma_1_tmp_tmp *
    ((K_p_M->contents * sin(u_in[5] * gain_el->contents) * k_sigma_1_tmp -
      K_p_T->contents * l_z->contents * cos(u_in[5] * gain_el->contents)) +
     j_gradient_fcn_15_tmp * o_sigma_1_tmp * g_sigma_1_tmp) / (r_sigma_1_tmp_tmp
    * r_sigma_1_tmp_tmp)) - f_gradient_fcn_15_tmp * (desired_el_value->contents
    - j_sigma_1_tmp_tmp) / gain_el->contents) + i_sigma_3_tmp *
                        gain_el->contents * sigma_1 * t_sigma_1_tmp_tmp *
                        sigma_14 * ((b_sigma_1_tmp * e_sigma_1_tmp_tmp - cos
    (u_in[9] * gain_az->contents) * sin(u_in[5] * gain_el->contents) *
    g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) + sin(u_in[5] * gain_el->contents) *
    sin(u_in[9] * gain_az->contents) * h_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp)) +
    k_sigma_3_tmp * gain_el->contents * sigma_2 * t_sigma_1_tmp_tmp * sigma_14 *
    b_gradient_fcn_15_tmp * g_sigma_1_tmp;
  e_gradient_fcn_15_tmp = 2.0 * t_sigma_1_tmp * m_gradient_fcn_15_tmp;
  ib_sigma_1_tmp = t_sigma_1_tmp * l_gradient_fcn_15_tmp;
  h_sigma_3_tmp = d_sigma_3_tmp * s_sigma_1_tmp_tmp;
  c_a_tmp = c_sigma_5_tmp * p_gradient_fcn_15_tmp;
  g_sigma_3_tmp = d_sigma_3_tmp * y_sigma_1_tmp;
  gradient_fcn_15[6] = (((((t_sigma_1_tmp * n_gradient_fcn_15_tmp *
    gain_el->contents * sigma_4 * t_sigma_1_tmp_tmp * ((c_a +
    g_gradient_fcn_15_tmp * p_sigma_1_tmp * h_sigma_1_tmp) - K_p_T->contents *
    l_z->contents * sin(u_in[6] * gain_el->contents) * l_sigma_1_tmp) / (g_a *
    g_a) - e_gradient_fcn_15_tmp * gain_el->contents * sigma_5 *
    t_sigma_1_tmp_tmp * ((K_p_T->contents * l_2->contents * cos(u_in[6] *
    gain_el->contents) - K_p_M->contents * p_sigma_1_tmp * h_sigma_1_tmp) + b_a *
    h_sigma_1_tmp * l_sigma_1_tmp) / (h_a * h_a)) - f_gradient_fcn_15_tmp *
    (desired_el_value->contents - k_sigma_1_tmp_tmp) / gain_el->contents) +
    ib_sigma_1_tmp * gain_el->contents * sigma_3 * t_sigma_1_tmp_tmp *
    ((K_p_T->contents * l_z->contents * cos(u_in[6] * gain_el->contents) +
      K_p_M->contents * sin(u_in[6] * gain_el->contents) * l_sigma_1_tmp) + b_a *
     p_sigma_1_tmp * h_sigma_1_tmp) / (i_a * i_a)) + c_a_tmp * gain_el->contents
    * sigma_7 * t_sigma_1_tmp_tmp * sigma_14 * ((c_sigma_1_tmp *
    h_sigma_1_tmp_tmp + p_sigma_1_tmp * h_sigma_1_tmp * e_sigma_1_tmp_tmp *
    g_sigma_1_tmp_tmp) - h_sigma_1_tmp * l_sigma_1_tmp * e_sigma_1_tmp_tmp *
    m_sigma_1_tmp_tmp) / 50.0) + g_sigma_3_tmp * gain_el->contents * sigma_1 *
                        t_sigma_1_tmp_tmp * sigma_14 * ((c_sigma_1_tmp *
    e_sigma_1_tmp_tmp - cos(u_in[10] * gain_az->contents) * sin(u_in[6] *
    gain_el->contents) * g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) + sin(u_in[6] *
    gain_el->contents) * sin(u_in[10] * gain_az->contents) * h_sigma_1_tmp_tmp *
    m_sigma_1_tmp_tmp)) + h_sigma_3_tmp * gain_el->contents * sigma_2 *
    t_sigma_1_tmp_tmp * sigma_14 * c_gradient_fcn_15_tmp * h_sigma_1_tmp;
  c_a = u_sigma_1_tmp * l_gradient_fcn_15_tmp;
  b_a_tmp = 2.0 * u_sigma_1_tmp * m_gradient_fcn_15_tmp;
  a_tmp = e_sigma_3_tmp * s_sigma_1_tmp_tmp;
  f_sigma_3_tmp = d_sigma_5_tmp * p_gradient_fcn_15_tmp;
  hb_sigma_1_tmp = e_sigma_3_tmp * y_sigma_1_tmp;
  gradient_fcn_15[7] = (((((c_a * gain_el->contents * sigma_3 *
    t_sigma_1_tmp_tmp * ((K_p_T->contents * l_z->contents * cos(u_in[7] *
    gain_el->contents) - K_p_M->contents * sin(u_in[7] * gain_el->contents) *
    m_sigma_1_tmp) + b_a * q_sigma_1_tmp * i_sigma_1_tmp) / (j_a * j_a) -
    b_a_tmp * gain_el->contents * sigma_5 * t_sigma_1_tmp_tmp *
    ((K_p_M->contents * q_sigma_1_tmp * i_sigma_1_tmp - K_p_T->contents *
      l_2->contents * cos(u_in[7] * gain_el->contents)) + b_a * i_sigma_1_tmp *
     m_sigma_1_tmp) / (k_a * k_a)) - u_sigma_1_tmp * n_gradient_fcn_15_tmp *
    gain_el->contents * sigma_4 * t_sigma_1_tmp_tmp * ((fb_sigma_1_tmp +
    g_gradient_fcn_15_tmp * q_sigma_1_tmp * i_sigma_1_tmp) + K_p_T->contents *
    l_z->contents * sin(u_in[7] * gain_el->contents) * m_sigma_1_tmp) / (l_a *
    l_a)) - f_gradient_fcn_15_tmp * (desired_el_value->contents -
    l_sigma_1_tmp_tmp) / gain_el->contents) + f_sigma_3_tmp * gain_el->contents *
    sigma_7 * t_sigma_1_tmp_tmp * sigma_14 * ((d_sigma_1_tmp * h_sigma_1_tmp_tmp
    + q_sigma_1_tmp * i_sigma_1_tmp * e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) -
    i_sigma_1_tmp * m_sigma_1_tmp * e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) /
    50.0) + hb_sigma_1_tmp * gain_el->contents * sigma_1 * t_sigma_1_tmp_tmp *
                        sigma_14 * ((d_sigma_1_tmp * e_sigma_1_tmp_tmp - cos
    (u_in[11] * gain_az->contents) * sin(u_in[7] * gain_el->contents) *
    g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) + sin(u_in[7] * gain_el->contents) *
    sin(u_in[11] * gain_az->contents) * h_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp))
    + a_tmp * gain_el->contents * sigma_2 * t_sigma_1_tmp_tmp * sigma_14 *
    d_gradient_fcn_15_tmp * i_sigma_1_tmp;
  fb_sigma_1_tmp = d_a_tmp * d_a_tmp;
  f_gradient_fcn_15_tmp = 2.0 * fb_sigma_1_tmp * gamma_quadratic_du->contents;
  gradient_fcn_15[8] = (((((h_gradient_fcn_15_tmp * gain_az->contents * sigma_5 *
    t_sigma_1_tmp_tmp * sigma_1_tmp * (K_p_M->contents * j_sigma_1_tmp -
    K_p_T->contents * l_4->contents * cos(u_in[8] * gain_az->contents)) / (m_a *
    m_a) - i_gradient_fcn_15_tmp * gain_az->contents * sigma_3 *
    t_sigma_1_tmp_tmp * sigma_1_tmp * (K_p_M->contents * cos(u_in[8] *
    gain_az->contents) + j_gradient_fcn_15_tmp * j_sigma_1_tmp) / (n_a * n_a)) -
    f_gradient_fcn_15_tmp * (desired_az_value->contents - n_sigma_1_tmp_tmp) /
    gain_az->contents) + sigma_5_tmp * n_gradient_fcn_15_tmp * gain_az->contents
    * sigma_4 * t_sigma_1_tmp_tmp * sigma_1_tmp * (l_z->contents * n_sigma_1_tmp
    - l_1->contents * j_sigma_1_tmp) / (o_a * o_a)) - o_gradient_fcn_15_tmp *
    gain_az->contents * sigma_2 * t_sigma_1_tmp_tmp * sigma_14 * cos
    (gradient_fcn_15_tmp_tmp) * sigma_1_tmp) + a * gain_az->contents * sigma_7 *
                        t_sigma_1_tmp_tmp * sigma_14 * gradient_fcn_15_tmp *
                        sigma_1_tmp * e_sigma_1_tmp_tmp / 50.0) - m_sigma_3_tmp *
    gain_az->contents * sigma_1 * t_sigma_1_tmp_tmp * sigma_14 *
    gradient_fcn_15_tmp * sigma_1_tmp * h_sigma_1_tmp_tmp;
  gradient_fcn_15[9] = (((((k_gradient_fcn_15_tmp * gain_az->contents * sigma_3 *
    t_sigma_1_tmp_tmp * b_sigma_1_tmp * (K_p_M->contents * cos(u_in[9] *
    gain_az->contents) - j_gradient_fcn_15_tmp * k_sigma_1_tmp) / (p_a * p_a) -
    f_gradient_fcn_15_tmp * (desired_az_value->contents - o_sigma_1_tmp_tmp) /
    gain_az->contents) - j_sigma_3_tmp * gain_az->contents * sigma_5 *
    t_sigma_1_tmp_tmp * b_sigma_1_tmp * (K_p_M->contents * k_sigma_1_tmp +
    K_p_T->contents * l_4->contents * cos(u_in[9] * gain_az->contents)) / (q_a *
    q_a)) + b_sigma_5_tmp * n_gradient_fcn_15_tmp * gain_az->contents * sigma_4 *
    t_sigma_1_tmp_tmp * b_sigma_1_tmp * (l_z->contents * o_sigma_1_tmp +
    l_1->contents * k_sigma_1_tmp) / (r_a * r_a)) - k_sigma_3_tmp *
    gain_az->contents * sigma_2 * t_sigma_1_tmp_tmp * sigma_14 * cos
    (b_gradient_fcn_15_tmp_tmp) * b_sigma_1_tmp) + l_sigma_3_tmp *
                        gain_az->contents * sigma_7 * t_sigma_1_tmp_tmp *
                        sigma_14 * b_gradient_fcn_15_tmp * b_sigma_1_tmp *
                        e_sigma_1_tmp_tmp / 50.0) - i_sigma_3_tmp *
    gain_az->contents * sigma_1 * t_sigma_1_tmp_tmp * sigma_14 *
    b_gradient_fcn_15_tmp * b_sigma_1_tmp * h_sigma_1_tmp_tmp;
  gradient_fcn_15[10] = (((((e_gradient_fcn_15_tmp * gain_az->contents * sigma_5
    * t_sigma_1_tmp_tmp * c_sigma_1_tmp * (K_p_M->contents * l_sigma_1_tmp +
    K_p_T->contents * l_3->contents * cos(u_in[10] * gain_az->contents)) / (s_a *
    s_a) - ib_sigma_1_tmp * gain_az->contents * sigma_3 * t_sigma_1_tmp_tmp *
    c_sigma_1_tmp * (K_p_M->contents * cos(u_in[10] * gain_az->contents) - b_a *
                     l_sigma_1_tmp) / (t_a * t_a)) - f_gradient_fcn_15_tmp *
    (desired_az_value->contents - p_sigma_1_tmp_tmp) / gain_az->contents) +
    c_sigma_5_tmp * n_gradient_fcn_15_tmp * gain_az->contents * sigma_4 *
    t_sigma_1_tmp_tmp * c_sigma_1_tmp * (l_z->contents * p_sigma_1_tmp +
    l_2->contents * l_sigma_1_tmp) / (u_a * u_a)) - h_sigma_3_tmp *
    gain_az->contents * sigma_2 * t_sigma_1_tmp_tmp * sigma_14 * cos
    (c_gradient_fcn_15_tmp_tmp) * c_sigma_1_tmp) + c_a_tmp * gain_az->contents *
    sigma_7 * t_sigma_1_tmp_tmp * sigma_14 * c_gradient_fcn_15_tmp *
    c_sigma_1_tmp * e_sigma_1_tmp_tmp / 50.0) - g_sigma_3_tmp *
    gain_az->contents * sigma_1 * t_sigma_1_tmp_tmp * sigma_14 *
    c_gradient_fcn_15_tmp * c_sigma_1_tmp * h_sigma_1_tmp_tmp;
  gradient_fcn_15[11] = (((((c_a * gain_az->contents * sigma_3 *
    t_sigma_1_tmp_tmp * d_sigma_1_tmp * (K_p_M->contents * cos(u_in[11] *
    gain_az->contents) + b_a * m_sigma_1_tmp) / (v_a * v_a) -
    f_gradient_fcn_15_tmp * (desired_az_value->contents - q_sigma_1_tmp_tmp) /
    gain_az->contents) - b_a_tmp * gain_az->contents * sigma_5 *
    t_sigma_1_tmp_tmp * d_sigma_1_tmp * (K_p_M->contents * m_sigma_1_tmp -
    K_p_T->contents * l_3->contents * cos(u_in[11] * gain_az->contents)) / (w_a *
    w_a)) + d_sigma_5_tmp * n_gradient_fcn_15_tmp * gain_az->contents * sigma_4 *
    t_sigma_1_tmp_tmp * d_sigma_1_tmp * (l_z->contents * q_sigma_1_tmp -
    l_2->contents * m_sigma_1_tmp) / (x_a * x_a)) - a_tmp * gain_az->contents *
    sigma_2 * t_sigma_1_tmp_tmp * sigma_14 * cos(d_gradient_fcn_15_tmp_tmp) *
    d_sigma_1_tmp) + f_sigma_3_tmp * gain_az->contents * sigma_7 *
    t_sigma_1_tmp_tmp * sigma_14 * d_gradient_fcn_15_tmp * d_sigma_1_tmp *
    e_sigma_1_tmp_tmp / 50.0) - hb_sigma_1_tmp * gain_az->contents * sigma_1 *
    t_sigma_1_tmp_tmp * sigma_14 * d_gradient_fcn_15_tmp * d_sigma_1_tmp *
    h_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp = K_p_T->contents * gain_theta->contents;
  b_gradient_fcn_15_tmp = Cl_alpha->contents * sigma_1_tmp_tmp;
  gradient_fcn_15_tmp_tmp = sigma_6 * d_sigma_1_tmp_tmp;
  c_gradient_fcn_15_tmp = gradient_fcn_15_tmp * t_sigma_1_tmp_tmp;
  d_gradient_fcn_15_tmp = gradient_fcn_15_tmp_tmp * c_sigma_1_tmp_tmp;
  e_gradient_fcn_15_tmp = b_gradient_fcn_15_tmp * b_sigma_1_tmp_tmp;
  f_gradient_fcn_15_tmp = gradient_fcn_15_tmp * sigma_2_tmp * t_sigma_1_tmp_tmp;
  gradient_fcn_15_tmp = gradient_fcn_15_tmp * b_sigma_2_tmp * t_sigma_1_tmp_tmp;
  g_gradient_fcn_15_tmp = d_gradient_fcn_15_tmp + e_gradient_fcn_15_tmp;
  h_gradient_fcn_15_tmp = 2.0 * x_sigma_1_tmp * K_Cd->contents * sigma_1_tmp_tmp
    * d_sigma_1_tmp_tmp * b_sigma_1_tmp_tmp;
  i_gradient_fcn_15_tmp = sigma_15_tmp * e_sigma_1_tmp_tmp;
  j_gradient_fcn_15_tmp = sigma_15_tmp * h_sigma_1_tmp_tmp;
  k_gradient_fcn_15_tmp = b_sigma_15_tmp * e_sigma_1_tmp;
  o_gradient_fcn_15_tmp = c_sigma_15_tmp * e_sigma_1_tmp;
  ib_sigma_1_tmp = e_a_tmp * e_a_tmp;
  gradient_fcn_15[12] = (((p_gradient_fcn_15_tmp * sigma_7 * sigma_14 *
    ((((((((c_gradient_fcn_15_tmp * e_sigma_1_tmp_tmp * v_sigma_1_tmp - sigma_15
            * e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) + j_gradient_fcn_15_tmp *
           (((Cl_alpha->contents * c_sigma_1_tmp_tmp + gradient_fcn_15_tmp_tmp *
              c_sigma_1_tmp_tmp) + b_gradient_fcn_15_tmp * b_sigma_1_tmp_tmp) -
            h_gradient_fcn_15_tmp) / 2.0) + f_gradient_fcn_15_tmp *
          g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) + i_gradient_fcn_15_tmp *
         (gradient_fcn_15_tmp_tmp * b_sigma_1_tmp_tmp - b_gradient_fcn_15_tmp *
          c_sigma_1_tmp_tmp) / 2.0) - gradient_fcn_15_tmp * h_sigma_1_tmp_tmp *
        m_sigma_1_tmp_tmp) - sigma_15_tmp * g_sigma_1_tmp_tmp *
       h_sigma_1_tmp_tmp * g_gradient_fcn_15_tmp / 2.0) + k_gradient_fcn_15_tmp *
      h_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp / 2.0) + o_gradient_fcn_15_tmp *
     e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) / 50.0 - 2.0 * y_sigma_1_tmp *
    sigma_1 * sigma_14 * ((((((((c_gradient_fcn_15_tmp * h_sigma_1_tmp_tmp *
    v_sigma_1_tmp - sigma_15 * g_sigma_1_tmp_tmp * h_sigma_1_tmp_tmp) -
    i_gradient_fcn_15_tmp * (((Cl_alpha->contents * c_sigma_1_tmp_tmp +
    d_gradient_fcn_15_tmp) + e_gradient_fcn_15_tmp) - h_gradient_fcn_15_tmp) /
    2.0) - f_gradient_fcn_15_tmp * e_sigma_1_tmp_tmp * g_sigma_1_tmp_tmp) +
    gradient_fcn_15_tmp * e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp) +
    j_gradient_fcn_15_tmp * (gradient_fcn_15_tmp_tmp * b_sigma_1_tmp_tmp -
    b_gradient_fcn_15_tmp * c_sigma_1_tmp_tmp) / 2.0) + i_gradient_fcn_15_tmp *
    g_sigma_1_tmp_tmp * g_gradient_fcn_15_tmp / 2.0) - k_gradient_fcn_15_tmp *
    e_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp / 2.0) + o_gradient_fcn_15_tmp *
    h_sigma_1_tmp_tmp * m_sigma_1_tmp_tmp)) - 2.0 * s_sigma_1_tmp_tmp * sigma_2 *
    sigma_14 * (K_Cd->contents * S->contents * gain_theta->contents *
                rho->contents * sigma_1_tmp_tmp * e_sigma_1_tmp *
                g_sigma_1_tmp_tmp * x_sigma_1_tmp * w_sigma_1_tmp + sigma_15 *
                m_sigma_1_tmp_tmp)) - 2.0 * ib_sigma_1_tmp *
    gamma_quadratic_du->contents * (desired_theta_value->contents -
    sigma_1_tmp_tmp_tmp) / gain_theta->contents) + n_sigma_3_tmp *
    l_gradient_fcn_15_tmp * gain_theta->contents * rho->contents * sigma_3 *
    wing_chord->contents / (2.0 * (y_a * y_a));
  gradient_fcn_15_tmp = sigma_3_tmp * b_sigma_2_tmp * t_sigma_1_tmp_tmp;
  b_gradient_fcn_15_tmp = sigma_3_tmp * sigma_2_tmp * t_sigma_1_tmp_tmp;
  c_gradient_fcn_15_tmp = sigma_7_tmp * e_sigma_1_tmp;
  d_gradient_fcn_15_tmp = ab_sigma_1_tmp * sigma_1_tmp_tmp * b_sigma_1_tmp_tmp;
  e_gradient_fcn_15_tmp = sigma_7_tmp * d_sigma_1_tmp_tmp;
  f_gradient_fcn_15_tmp = c_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp;
  g_gradient_fcn_15_tmp = d_gradient_fcn_15_tmp * m_sigma_1_tmp_tmp;
  h_gradient_fcn_15_tmp = e_gradient_fcn_15_tmp * c_sigma_1_tmp_tmp *
    m_sigma_1_tmp_tmp;
  i_gradient_fcn_15_tmp = f_a_tmp * f_a_tmp;
  gradient_fcn_15[13] = ((gain_phi->contents * sigma_1 * sigma_14 *
    h_sigma_1_tmp_tmp * ((((f_gradient_fcn_15_tmp - b_gradient_fcn_15_tmp *
    m_sigma_1_tmp_tmp) - gradient_fcn_15_tmp * g_sigma_1_tmp_tmp) +
    g_gradient_fcn_15_tmp) + h_gradient_fcn_15_tmp) * y_sigma_1_tmp +
    gain_phi->contents * sigma_2 * sigma_14 * ((((gradient_fcn_15_tmp *
    m_sigma_1_tmp_tmp - b_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp) -
    c_gradient_fcn_15_tmp * m_sigma_1_tmp_tmp) + d_gradient_fcn_15_tmp *
    g_sigma_1_tmp_tmp) + e_gradient_fcn_15_tmp * g_sigma_1_tmp_tmp *
    c_sigma_1_tmp_tmp) * s_sigma_1_tmp_tmp) - 2.0 * i_gradient_fcn_15_tmp *
    gamma_quadratic_du->contents * (desired_phi_value->contents -
    f_sigma_1_tmp_tmp) / gain_phi->contents) - p_gradient_fcn_15_tmp *
    gain_phi->contents * sigma_7 * sigma_14 * e_sigma_1_tmp_tmp *
    ((((f_gradient_fcn_15_tmp - b_gradient_fcn_15_tmp * m_sigma_1_tmp_tmp) -
       gradient_fcn_15_tmp * g_sigma_1_tmp_tmp) + g_gradient_fcn_15_tmp) +
     h_gradient_fcn_15_tmp) / 100.0;
  gradient_fcn_15_tmp = g_a_tmp * g_a_tmp;
  gradient_fcn_15[14] = gradient_fcn_15_tmp * gamma_quadratic_du->contents *
    (2.0 * u_in[14] - 2.0 * desired_ailerons_value->contents /
     gain_ailerons->contents) + sigma_4_tmp * n_gradient_fcn_15_tmp *
    gain_ailerons->contents * rho->contents * sigma_4 / (2.0 * (ab_a * ab_a));
  gradient_fcn_15[15] = (((((gamma_quadratic_du->contents *
    ((((((((((((((f_gradient_fcn_15_tmp_tmp * (bb_a * bb_a) +
                  f_gradient_fcn_15_tmp_tmp * (cb_a * cb_a)) +
                 f_gradient_fcn_15_tmp_tmp * (db_a * db_a)) +
                f_gradient_fcn_15_tmp_tmp * (eb_a * eb_a)) +
               i_gradient_fcn_15_tmp * (fb_a * fb_a)) + ib_sigma_1_tmp * (gb_a *
    gb_a)) + e_gradient_fcn_15_tmp_tmp * (hb_a * hb_a)) +
            e_gradient_fcn_15_tmp_tmp * (ib_a * ib_a)) +
           e_gradient_fcn_15_tmp_tmp * (jb_a * jb_a)) +
          e_gradient_fcn_15_tmp_tmp * (kb_a * kb_a)) + gradient_fcn_15_tmp *
         (lb_a * lb_a)) + fb_sigma_1_tmp * (mb_a * mb_a)) + fb_sigma_1_tmp *
       (nb_a * nb_a)) + fb_sigma_1_tmp * (ob_a * ob_a)) + fb_sigma_1_tmp * (pb_a
    * pb_a)) + y_sigma_1_tmp * (sigma_1 * sigma_1)) + s_sigma_1_tmp_tmp *
    (sigma_2 * sigma_2)) + p_gradient_fcn_15_tmp * (sigma_7 * sigma_7) / 10000.0)
    + n_gradient_fcn_15_tmp * (sigma_4 * sigma_4) / (4.0 * (qb_a * qb_a))) +
    l_gradient_fcn_15_tmp * (sigma_3 * sigma_3) / (4.0 * (rb_a * rb_a))) +
    m_gradient_fcn_15_tmp * (sigma_5 * sigma_5) / (sb_a * sb_a);
  *cost = gradient_fcn_15[15];
  memcpy(&gradient[0], &gradient_fcn_15[0], 15U * sizeof(double));
}

/*
 * Arguments    : const double H[225]
 *                i_struct_T *solution
 *                e_struct_T *memspace
 *                const f_struct_T *qrmanager
 *                g_struct_T *cholmanager
 *                const struct_T *objective
 *                bool alwaysPositiveDef
 * Return Type  : void
 */
static void compute_deltax(const double H[225], i_struct_T *solution, e_struct_T
  *memspace, const f_struct_T *qrmanager, g_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef)
{
  int idx;
  int idx_row;
  int ix;
  int k;
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      memset(&solution->searchDir[0], 0, (nVar_tmp + 1) * sizeof(double));
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
              idx_row = (nVar_tmp + 1) * idx;
              ix = 31 * idx;
              for (k = 0; k <= nVar_tmp; k++) {
                cholmanager->FMat[ix + k] = H[idx_row + k];
              }
            }

            cholmanager->info = xpotrf(qrmanager->mrows, cholmanager->FMat);
          } else {
            cholmanager->ndims = qrmanager->mrows;
            for (idx = 0; idx <= nVar_tmp; idx++) {
              idx_row = qrmanager->mrows * idx;
              ix = 31 * idx;
              for (k = 0; k <= nVar_tmp; k++) {
                cholmanager->FMat[ix + k] = H[idx_row + k];
              }
            }

            if (qrmanager->mrows < 1) {
              nVars = -1;
            } else {
              nVars = 0;
              if (qrmanager->mrows > 1) {
                smax = fabs(cholmanager->FMat[0]);
                for (k = 2; k <= nVar_tmp + 1; k++) {
                  double s;
                  s = fabs(cholmanager->FMat[(k - 1) << 5]);
                  if (s > smax) {
                    nVars = k - 1;
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
            idx_row = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= idx_row + 1; idx++) {
                nVars = idx + idx * 31;
                i = idx_row - idx;
                for (k = 0; k <= i; k++) {
                  ix = (idx + k) + 1;
                  solution->searchDir[ix] -= solution->searchDir[idx] *
                    cholmanager->FMat[(nVars + k) + 1];
                }
              }
            }

            i = cholmanager->ndims;
            for (idx = 0; idx < i; idx++) {
              solution->searchDir[idx] /= cholmanager->FMat[idx + 31 * idx];
            }

            idx_row = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = idx_row; idx >= 1; idx--) {
                ix = (idx - 1) * 31;
                smax = solution->searchDir[idx - 1];
                i = idx + 1;
                for (k = idx_row; k >= i; k--) {
                  smax -= cholmanager->FMat[(ix + k) - 1] * solution->
                    searchDir[k - 1];
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
              idx_row = nVars * idx;
              ix = 31 * idx;
              for (k = 0; k < nVars; k++) {
                cholmanager->FMat[ix + k] = H[idx_row + k];
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
              idx_row = objective->nvar + 1;
              i = qrmanager->mrows;
              for (k = idx_row; k <= i; k++) {
                solution->searchDir[k - 1] *= smax;
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
          memset(&solution->searchDir[0], 0, (nVar_tmp + 1) * sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
          for (idx = nullStartIdx_tmp; idx <= i; idx += 31) {
            idx_row = idx + nVar_tmp;
            for (k = idx; k <= idx_row; k++) {
              int nVars;
              nVars = k - idx;
              solution->searchDir[nVars] += qrmanager->Q[k - 1] *
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
          i = objective->nvar + 1;
          for (ix = 0; ix < mNull_tmp; ix++) {
            for (idx_row = i; idx_row <= nVars; idx_row++) {
              memspace->workspace_double[(idx_row + 31 * ix) - 1] =
                objective->beta * qrmanager->Q[(idx_row + 31 * (ix +
                qrmanager->ncols)) - 1];
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
            for (k = 2; k <= mNull_tmp; k++) {
              double s;
              s = fabs(cholmanager->FMat[(k - 1) << 5]);
              if (s > smax) {
                nVars = k - 1;
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
            memset(&memspace->workspace_double[0], 0, mNull_tmp * sizeof(double));
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (idx = nullStartIdx_tmp; idx <= i; idx += 31) {
              smax = 0.0;
              idx_row = idx + nVar_tmp;
              for (k = idx; k <= idx_row; k++) {
                smax += qrmanager->Q[k - 1] * objective->grad[k - idx];
              }

              idx_row = div_nde_s32_floor(idx - nullStartIdx_tmp, 31);
              memspace->workspace_double[idx_row] += -smax;
            }
          }

          if (alwaysPositiveDef) {
            idx_row = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx < idx_row; idx++) {
                ix = idx * 31;
                smax = memspace->workspace_double[idx];
                for (k = 0; k < idx; k++) {
                  smax -= cholmanager->FMat[ix + k] * memspace->
                    workspace_double[k];
                }

                memspace->workspace_double[idx] = smax / cholmanager->FMat[ix +
                  idx];
              }
            }

            idx_row = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = idx_row; idx >= 1; idx--) {
                nVars = (idx + (idx - 1) * 31) - 1;
                memspace->workspace_double[idx - 1] /= cholmanager->FMat[nVars];
                for (k = 0; k <= idx - 2; k++) {
                  ix = (idx - k) - 2;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx - 1] * cholmanager->FMat[(nVars - k) -
                    1];
                }
              }
            }
          } else {
            idx_row = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx = 0; idx <= idx_row + 1; idx++) {
                nVars = idx + idx * 31;
                i = idx_row - idx;
                for (k = 0; k <= i; k++) {
                  ix = (idx + k) + 1;
                  memspace->workspace_double[ix] -= memspace->
                    workspace_double[idx] * cholmanager->FMat[(nVars + k) + 1];
                }
              }
            }

            i = cholmanager->ndims;
            for (idx = 0; idx < i; idx++) {
              memspace->workspace_double[idx] /= cholmanager->FMat[idx + 31 *
                idx];
            }

            idx_row = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx = idx_row; idx >= 1; idx--) {
                ix = (idx - 1) * 31;
                smax = memspace->workspace_double[idx - 1];
                i = idx + 1;
                for (k = idx_row; k >= i; k--) {
                  smax -= cholmanager->FMat[(ix + k) - 1] *
                    memspace->workspace_double[k - 1];
                }

                memspace->workspace_double[idx - 1] = smax;
              }
            }
          }

          if (qrmanager->mrows != 0) {
            memset(&solution->searchDir[0], 0, (nVar_tmp + 1) * sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + 31 * (mNull_tmp - 1);
            for (idx = nullStartIdx_tmp; idx <= i; idx += 31) {
              idx_row = idx + nVar_tmp;
              for (k = idx; k <= idx_row; k++) {
                nVars = k - idx;
                solution->searchDir[nVars] += qrmanager->Q[k - 1] *
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
      memset(&workspace[0], 0, (idxStart + 1) * sizeof(int));
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
 * Arguments    : int n
 *                double a
 *                int ix0
 *                double y[36]
 *                int iy0
 * Return Type  : void
 */
static void d_xaxpy(int n, double a, int ix0, double y[36], int iy0)
{
  int k;
  if (!(a == 0.0)) {
    int i;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[72]
 *                int ix0
 * Return Type  : double
 */
static double d_xnrm2(int n, const double x[72], int ix0)
{
  double scale;
  double y;
  int k;
  int kend;
  y = 0.0;
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

  return scale * sqrt(y);
}

/*
 * Arguments    : f_struct_T *obj
 *                int idx
 * Return Type  : void
 */
static void deleteColMoveEnd(f_struct_T *obj, int idx)
{
  double c;
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
    obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
    b_i = obj->minRowCol;
    for (k = 0; k < b_i; k++) {
      obj->QR[k + 31 * (idx - 1)] = obj->QR[k + 31 * (obj->ncols - 1)];
    }

    obj->ncols--;
    u0 = obj->mrows;
    i = obj->ncols;
    if (u0 <= i) {
      i = u0;
    }

    obj->minRowCol = i;
    if (idx < obj->mrows) {
      double c_temp_tmp;
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
        xrotg(&obj->QR[(k + i) - 1], &temp_tmp, &c, &s);
        obj->QR[b_i] = temp_tmp;
        b_i = 31 * (k - 1);
        obj->QR[k + b_i] = 0.0;
        QRk0 = k + 31 * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 31;
            temp_tmp = obj->QR[b_temp_tmp];
            c_temp_tmp = obj->QR[b_temp_tmp - 1];
            obj->QR[b_temp_tmp] = c * temp_tmp - s * c_temp_tmp;
            obj->QR[b_temp_tmp - 1] = c * c_temp_tmp + s * temp_tmp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = b_i + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 31];
          c_temp_tmp = obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp + 31] = c * temp_tmp - s * c_temp_tmp;
          obj->Q[b_temp_tmp] = c * c_temp_tmp + s * temp_tmp;
        }

        k--;
      }

      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        u0 = 31 * (k - 1);
        i = k + u0;
        temp_tmp = obj->QR[i];
        xrotg(&obj->QR[(k + u0) - 1], &temp_tmp, &c, &s);
        obj->QR[i] = temp_tmp;
        QRk0 = k << 5;
        n = obj->ncols - k;
        if (n >= 1) {
          for (b_k = 0; b_k < n; b_k++) {
            b_temp_tmp = QRk0 + b_k * 31;
            temp_tmp = obj->QR[b_temp_tmp];
            c_temp_tmp = obj->QR[b_temp_tmp - 1];
            obj->QR[b_temp_tmp] = c * temp_tmp - s * c_temp_tmp;
            obj->QR[b_temp_tmp - 1] = c * c_temp_tmp + s * temp_tmp;
          }
        }

        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          b_temp_tmp = u0 + b_k;
          temp_tmp = obj->Q[b_temp_tmp + 31];
          c_temp_tmp = obj->Q[b_temp_tmp];
          obj->Q[b_temp_tmp + 31] = c * temp_tmp - s * c_temp_tmp;
          obj->Q[b_temp_tmp] = c * c_temp_tmp + s * temp_tmp;
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
  int b_numerator;
  if (((numerator < 0) != (denominator < 0)) && (numerator % denominator != 0))
  {
    b_numerator = -1;
  } else {
    b_numerator = 0;
  }

  return numerator / denominator + b_numerator;
}

/*
 * Arguments    : const double H[225]
 *                const double f[16]
 *                i_struct_T *solution
 *                e_struct_T *memspace
 *                k_struct_T *workingset
 *                f_struct_T *qrmanager
 *                g_struct_T *cholmanager
 *                struct_T *objective
 *                h_struct_T *options
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void driver(const double H[225], const double f[16], i_struct_T *solution,
                   e_struct_T *memspace, k_struct_T *workingset, f_struct_T
                   *qrmanager, g_struct_T *cholmanager, struct_T *objective,
                   h_struct_T *options, int runTimeOptions_MaxIterations)
{
  int TYPE;
  int idx;
  int idxEndIneq;
  int idxStartIneq;
  int nVar;
  bool guard1 = false;
  solution->iterations = 0;
  nVar = workingset->nVar - 1;
  guard1 = false;
  if (workingset->probType == 3) {
    idxEndIneq = workingset->sizes[0];
    for (idx = 0; idx < idxEndIneq; idx++) {
      solution->xstar[workingset->indexFixed[idx] - 1] = workingset->
        ub[workingset->indexFixed[idx] - 1];
    }

    idxEndIneq = workingset->sizes[3];
    for (idx = 0; idx < idxEndIneq; idx++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + idx) - 1]) {
        solution->xstar[workingset->indexLB[idx] - 1] = -workingset->
          lb[workingset->indexLB[idx] - 1];
      }
    }

    idxEndIneq = workingset->sizes[4];
    for (idx = 0; idx < idxEndIneq; idx++) {
      if (workingset->isActiveConstr[(workingset->isActiveIdx[4] + idx) - 1]) {
        solution->xstar[workingset->indexUB[idx] - 1] = workingset->
          ub[workingset->indexUB[idx] - 1];
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
    solution->maxConstr = b_maxConstraintViolation(workingset, solution->xstar);
    if (solution->maxConstr > 1.0E-6) {
      int PROBTYPE_ORIG;
      int b_nVar;
      PROBTYPE_ORIG = workingset->probType;
      b_nVar = workingset->nVar;
      solution->xstar[15] = solution->maxConstr + 1.0;
      if (workingset->probType == 3) {
        idxEndIneq = 1;
      } else {
        idxEndIneq = 4;
      }

      setProblemType(workingset, idxEndIneq);
      idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
      idxEndIneq = workingset->nActiveConstr;
      for (TYPE = idxStartIneq; TYPE <= idxEndIneq; TYPE++) {
        workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[TYPE
          - 1] - 1] + workingset->Wlocalidx[TYPE - 1]) - 2] = false;
      }

      workingset->nWConstr[2] = 0;
      workingset->nWConstr[3] = 0;
      workingset->nWConstr[4] = 0;
      workingset->nActiveConstr = workingset->nWConstr[0] + workingset->
        nWConstr[1];
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
        idx = workingset->sizes[0];
        exitg1 = false;
        while ((!exitg1) && (idx + 1 <= workingset->nActiveConstr)) {
          if ((workingset->Wid[idx] == 4) && (workingset->Wlocalidx[idx] ==
               workingset->sizes[3])) {
            TYPE = workingset->Wid[idx] - 1;
            workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
              Wid[idx] - 1] + workingset->Wlocalidx[idx]) - 2] = false;
            workingset->Wid[idx] = workingset->Wid[workingset->nActiveConstr - 1];
            workingset->Wlocalidx[idx] = workingset->Wlocalidx
              [workingset->nActiveConstr - 1];
            idxEndIneq = workingset->nVar;
            for (idxStartIneq = 0; idxStartIneq < idxEndIneq; idxStartIneq++) {
              workingset->ATwset[idxStartIneq + (idx << 4)] = workingset->
                ATwset[idxStartIneq + ((workingset->nActiveConstr - 1) << 4)];
            }

            workingset->bwset[idx] = workingset->bwset[workingset->nActiveConstr
              - 1];
            workingset->nActiveConstr--;
            workingset->nWConstr[TYPE]--;
            exitg1 = true;
          } else {
            idx++;
          }
        }
      }

      idxStartIneq = workingset->nActiveConstr - 1;
      while ((idxStartIneq + 1 > workingset->sizes[0]) && (idxStartIneq + 1 >
              b_nVar)) {
        TYPE = workingset->Wid[idxStartIneq] - 1;
        workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
          Wid[idxStartIneq] - 1] + workingset->Wlocalidx[idxStartIneq]) - 2] =
          false;
        workingset->Wid[idxStartIneq] = workingset->Wid
          [workingset->nActiveConstr - 1];
        workingset->Wlocalidx[idxStartIneq] = workingset->Wlocalidx
          [workingset->nActiveConstr - 1];
        idxEndIneq = workingset->nVar;
        for (idx = 0; idx < idxEndIneq; idx++) {
          workingset->ATwset[idx + (idxStartIneq << 4)] = workingset->ATwset[idx
            + ((workingset->nActiveConstr - 1) << 4)];
        }

        workingset->bwset[idxStartIneq] = workingset->bwset
          [workingset->nActiveConstr - 1];
        workingset->nActiveConstr--;
        workingset->nWConstr[TYPE]--;
        idxStartIneq--;
      }

      solution->maxConstr = solution->xstar[15];
      setProblemType(workingset, PROBTYPE_ORIG);
      objective->objtype = objective->prev_objtype;
      objective->nvar = objective->prev_nvar;
      objective->hasLinear = objective->prev_hasLinear;
      options->ObjectiveLimit = rtMinusInf;
      options->StepTolerance = 1.0E-6;
      if (solution->state != 0) {
        solution->maxConstr = b_maxConstraintViolation(workingset,
          solution->xstar);
        if (solution->maxConstr > 1.0E-6) {
          memset(&solution->lambda[0], 0, 31U * sizeof(double));
          solution->fstar = computeFval(objective, memspace->workspace_double, H,
            f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            double maxConstr_new;
            if (nVar >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (nVar + 1) *
                     sizeof(double));
            }

            PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = b_maxConstraintViolation(workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (nVar >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (nVar + 1) *
                       sizeof(double));
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
 * Arguments    : int n
 *                const double x[6]
 *                int ix0
 * Return Type  : double
 */
static double e_xnrm2(int n, const double x[6], int ix0)
{
  double scale;
  double y;
  int k;
  int kend;
  y = 0.0;
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

  return scale * sqrt(y);
}

/*
 * Arguments    : const d_struct_T *obj_objfun_workspace
 *                const double x[15]
 *                double *fval
 *                int *status
 * Return Type  : void
 */
static void evalObjAndConstr(const d_struct_T *obj_objfun_workspace, const
  double x[15], double *fval, int *status)
{
  double a;
  double ab_sigma_1_tmp;
  double b_a;
  double b_sigma_1_tmp;
  double bb_sigma_1_tmp;
  double c_a;
  double c_sigma_1_tmp;
  double cb_sigma_1_tmp;
  double d_a;
  double d_sigma_1_tmp;
  double e_a;
  double e_sigma_1_tmp;
  double f_a;
  double f_sigma_1_tmp;
  double g_a;
  double g_sigma_1_tmp;
  double h_a;
  double h_sigma_1_tmp;
  double i_a;
  double i_sigma_1_tmp;
  double j_a;
  double j_sigma_1_tmp;
  double k_a;
  double k_sigma_1_tmp;
  double l_a;
  double l_sigma_1_tmp;
  double m_a;
  double m_sigma_1_tmp;
  double n_a;
  double n_sigma_1_tmp;
  double o_a;
  double o_sigma_1_tmp;
  double p_a;
  double p_sigma_1_tmp;
  double q_a;
  double q_sigma_1_tmp;
  double r_a;
  double r_sigma_1_tmp;
  double s_a;
  double s_sigma_1_tmp;
  double sigma_1;
  double sigma_1_tmp;
  double sigma_2;
  double sigma_3;
  double sigma_4;
  double sigma_5;
  double sigma_6;
  double t_a;
  double t_sigma_1_tmp;
  double u_a;
  double u_sigma_1_tmp;
  double v_a;
  double v_sigma_1_tmp;
  double w_a;
  double w_sigma_1_tmp;
  double x_sigma_1_tmp;
  double y_sigma_1_tmp;
  sigma_1_tmp = cos(obj_objfun_workspace->flight_path_angle->contents - x[12] *
                    obj_objfun_workspace->gain_theta->contents);
  b_sigma_1_tmp = sin(obj_objfun_workspace->flight_path_angle->contents - x[12] *
                      obj_objfun_workspace->gain_theta->contents);
  c_sigma_1_tmp = cos(obj_objfun_workspace->Beta->contents);
  d_sigma_1_tmp = cos(x[12] * obj_objfun_workspace->gain_theta->contents);
  e_sigma_1_tmp = cos(x[13] * obj_objfun_workspace->gain_phi->contents);
  f_sigma_1_tmp = sin(x[12] * obj_objfun_workspace->gain_theta->contents);
  sigma_6 = cos(x[4] * obj_objfun_workspace->gain_el->contents);
  g_sigma_1_tmp = cos(x[5] * obj_objfun_workspace->gain_el->contents);
  h_sigma_1_tmp = cos(x[6] * obj_objfun_workspace->gain_el->contents);
  i_sigma_1_tmp = cos(x[7] * obj_objfun_workspace->gain_el->contents);
  j_sigma_1_tmp = sin(x[13] * obj_objfun_workspace->gain_phi->contents);
  k_sigma_1_tmp = sin(obj_objfun_workspace->Beta->contents);
  l_sigma_1_tmp = sin(x[4] * obj_objfun_workspace->gain_el->contents);
  m_sigma_1_tmp = sin(x[5] * obj_objfun_workspace->gain_el->contents);
  n_sigma_1_tmp = sin(x[6] * obj_objfun_workspace->gain_el->contents);
  o_sigma_1_tmp = sin(x[7] * obj_objfun_workspace->gain_el->contents);
  p_sigma_1_tmp = sin(x[8] * obj_objfun_workspace->gain_az->contents);
  q_sigma_1_tmp = sin(x[9] * obj_objfun_workspace->gain_az->contents);
  r_sigma_1_tmp = sin(x[10] * obj_objfun_workspace->gain_az->contents);
  s_sigma_1_tmp = sin(x[11] * obj_objfun_workspace->gain_az->contents);
  t_sigma_1_tmp = cos(x[8] * obj_objfun_workspace->gain_az->contents);
  u_sigma_1_tmp = cos(x[9] * obj_objfun_workspace->gain_az->contents);
  v_sigma_1_tmp = cos(x[10] * obj_objfun_workspace->gain_az->contents);
  w_sigma_1_tmp = cos(x[11] * obj_objfun_workspace->gain_az->contents);
  a = obj_objfun_workspace->V->contents;
  b_a = obj_objfun_workspace->V->contents;
  c_a = obj_objfun_workspace->Cl_alpha->contents;
  d_a = obj_objfun_workspace->gain_theta->contents;
  e_a = obj_objfun_workspace->Cl_alpha->contents;
  f_a = obj_objfun_workspace->Cl_alpha->contents;
  g_a = obj_objfun_workspace->flight_path_angle->contents;
  h_a = obj_objfun_workspace->V->contents;
  i_a = obj_objfun_workspace->V->contents;
  j_a = obj_objfun_workspace->Cl_alpha->contents;
  k_a = obj_objfun_workspace->gain_theta->contents;
  l_a = obj_objfun_workspace->Cl_alpha->contents;
  m_a = obj_objfun_workspace->Cl_alpha->contents;
  n_a = obj_objfun_workspace->flight_path_angle->contents;
  o_a = obj_objfun_workspace->gain_motor->contents;
  p_a = obj_objfun_workspace->gain_motor->contents;
  q_a = obj_objfun_workspace->gain_motor->contents;
  r_a = obj_objfun_workspace->V->contents;
  s_a = obj_objfun_workspace->Cl_alpha->contents;
  t_a = obj_objfun_workspace->gain_theta->contents;
  u_a = obj_objfun_workspace->Cl_alpha->contents;
  v_a = obj_objfun_workspace->Cl_alpha->contents;
  w_a = obj_objfun_workspace->flight_path_angle->contents;
  x_sigma_1_tmp = x[12] * x[12];
  y_sigma_1_tmp = x[0] * x[0];
  sigma_5 = x[1] * x[1];
  ab_sigma_1_tmp = x[2] * x[2];
  bb_sigma_1_tmp = x[3] * x[3];
  cb_sigma_1_tmp = ((l_sigma_1_tmp * y_sigma_1_tmp + m_sigma_1_tmp * sigma_5) +
                    n_sigma_1_tmp * ab_sigma_1_tmp) + o_sigma_1_tmp *
    bb_sigma_1_tmp;
  sigma_1 = obj_objfun_workspace->dv_global->contents[0] - (((((d_sigma_1_tmp *
    (obj_objfun_workspace->Cl_alpha->contents * obj_objfun_workspace->
     S->contents * (a * a) * obj_objfun_workspace->rho->contents * b_sigma_1_tmp
     * (obj_objfun_workspace->flight_path_angle->contents - x[12] *
        obj_objfun_workspace->gain_theta->contents) / 2.0 -
     obj_objfun_workspace->S->contents * (b_a * b_a) * obj_objfun_workspace->
     rho->contents * sigma_1_tmp * c_sigma_1_tmp * (((obj_objfun_workspace->
    K_Cd->contents * (c_a * c_a) * x_sigma_1_tmp * (d_a * d_a) - 2.0 *
    obj_objfun_workspace->K_Cd->contents * (e_a * e_a) * x[12] *
    obj_objfun_workspace->flight_path_angle->contents *
    obj_objfun_workspace->gain_theta->contents) + obj_objfun_workspace->
    K_Cd->contents * (f_a * f_a) * (g_a * g_a)) + obj_objfun_workspace->
    Cd_zero->contents) / 2.0) + e_sigma_1_tmp * f_sigma_1_tmp *
    (obj_objfun_workspace->Cl_alpha->contents * obj_objfun_workspace->
     S->contents * (h_a * h_a) * obj_objfun_workspace->rho->contents *
     sigma_1_tmp * (obj_objfun_workspace->flight_path_angle->contents - x[12] *
                    obj_objfun_workspace->gain_theta->contents) / 2.0 +
     obj_objfun_workspace->S->contents * (i_a * i_a) * obj_objfun_workspace->
     rho->contents * b_sigma_1_tmp * c_sigma_1_tmp *
     (((obj_objfun_workspace->K_Cd->contents * (j_a * j_a) * x_sigma_1_tmp *
        (k_a * k_a) - 2.0 * obj_objfun_workspace->K_Cd->contents * (l_a * l_a) *
        x[12] * obj_objfun_workspace->flight_path_angle->contents *
        obj_objfun_workspace->gain_theta->contents) + obj_objfun_workspace->
       K_Cd->contents * (m_a * m_a) * (n_a * n_a)) +
      obj_objfun_workspace->Cd_zero->contents) / 2.0)) -
    obj_objfun_workspace->K_p_T->contents * (o_a * o_a) * d_sigma_1_tmp *
    cb_sigma_1_tmp) - obj_objfun_workspace->K_p_T->contents * (p_a * p_a) *
    e_sigma_1_tmp * f_sigma_1_tmp * (((sigma_6 * t_sigma_1_tmp * y_sigma_1_tmp +
    g_sigma_1_tmp * u_sigma_1_tmp * sigma_5) + h_sigma_1_tmp * v_sigma_1_tmp *
    ab_sigma_1_tmp) + i_sigma_1_tmp * w_sigma_1_tmp * bb_sigma_1_tmp)) +
    obj_objfun_workspace->K_p_T->contents * (q_a * q_a) * j_sigma_1_tmp *
    f_sigma_1_tmp * (((sigma_6 * p_sigma_1_tmp * y_sigma_1_tmp + g_sigma_1_tmp *
                       q_sigma_1_tmp * sigma_5) + h_sigma_1_tmp * r_sigma_1_tmp *
                      ab_sigma_1_tmp) + i_sigma_1_tmp * s_sigma_1_tmp *
                     bb_sigma_1_tmp)) - obj_objfun_workspace->S->contents * (r_a
    * r_a) * obj_objfun_workspace->rho->contents * j_sigma_1_tmp * f_sigma_1_tmp
    * k_sigma_1_tmp * (((obj_objfun_workspace->K_Cd->contents * (s_a * s_a) *
    x_sigma_1_tmp * (t_a * t_a) - 2.0 * obj_objfun_workspace->K_Cd->contents *
    (u_a * u_a) * x[12] * obj_objfun_workspace->flight_path_angle->contents *
    obj_objfun_workspace->gain_theta->contents) + obj_objfun_workspace->
                        K_Cd->contents * (v_a * v_a) * (w_a * w_a)) +
                       obj_objfun_workspace->Cd_zero->contents) / 2.0) /
    obj_objfun_workspace->m->contents;
  a = obj_objfun_workspace->V->contents;
  b_a = obj_objfun_workspace->V->contents;
  c_a = obj_objfun_workspace->Cl_alpha->contents;
  d_a = obj_objfun_workspace->gain_theta->contents;
  e_a = obj_objfun_workspace->Cl_alpha->contents;
  f_a = obj_objfun_workspace->Cl_alpha->contents;
  g_a = obj_objfun_workspace->flight_path_angle->contents;
  h_a = obj_objfun_workspace->gain_motor->contents;
  i_a = obj_objfun_workspace->gain_motor->contents;
  j_a = obj_objfun_workspace->V->contents;
  k_a = obj_objfun_workspace->Cl_alpha->contents;
  l_a = obj_objfun_workspace->gain_theta->contents;
  m_a = obj_objfun_workspace->Cl_alpha->contents;
  n_a = obj_objfun_workspace->Cl_alpha->contents;
  o_a = obj_objfun_workspace->flight_path_angle->contents;
  sigma_2 = obj_objfun_workspace->dv_global->contents[1] + (((j_sigma_1_tmp *
    (obj_objfun_workspace->Cl_alpha->contents * obj_objfun_workspace->
     S->contents * (a * a) * obj_objfun_workspace->rho->contents * sigma_1_tmp *
     (obj_objfun_workspace->flight_path_angle->contents - x[12] *
      obj_objfun_workspace->gain_theta->contents) / 2.0 +
     obj_objfun_workspace->S->contents * (b_a * b_a) * obj_objfun_workspace->
     rho->contents * b_sigma_1_tmp * c_sigma_1_tmp *
     (((obj_objfun_workspace->K_Cd->contents * (c_a * c_a) * x_sigma_1_tmp *
        (d_a * d_a) - 2.0 * obj_objfun_workspace->K_Cd->contents * (e_a * e_a) *
        x[12] * obj_objfun_workspace->flight_path_angle->contents *
        obj_objfun_workspace->gain_theta->contents) + obj_objfun_workspace->
       K_Cd->contents * (f_a * f_a) * (g_a * g_a)) +
      obj_objfun_workspace->Cd_zero->contents) / 2.0) -
    obj_objfun_workspace->K_p_T->contents * (h_a * h_a) * j_sigma_1_tmp * (((cos
    (x[4] * obj_objfun_workspace->gain_el->contents) * cos(x[8] *
    obj_objfun_workspace->gain_az->contents) * (x[0] * x[0]) + cos(x[5] *
    obj_objfun_workspace->gain_el->contents) * cos(x[9] *
    obj_objfun_workspace->gain_az->contents) * (x[1] * x[1])) + cos(x[6] *
    obj_objfun_workspace->gain_el->contents) * cos(x[10] *
    obj_objfun_workspace->gain_az->contents) * (x[2] * x[2])) + cos(x[7] *
    obj_objfun_workspace->gain_el->contents) * cos(x[11] *
    obj_objfun_workspace->gain_az->contents) * (x[3] * x[3]))) -
    obj_objfun_workspace->K_p_T->contents * (i_a * i_a) * e_sigma_1_tmp * (((cos
    (x[4] * obj_objfun_workspace->gain_el->contents) * sin(x[8] *
    obj_objfun_workspace->gain_az->contents) * (x[0] * x[0]) + cos(x[5] *
    obj_objfun_workspace->gain_el->contents) * sin(x[9] *
    obj_objfun_workspace->gain_az->contents) * (x[1] * x[1])) + cos(x[6] *
    obj_objfun_workspace->gain_el->contents) * sin(x[10] *
    obj_objfun_workspace->gain_az->contents) * (x[2] * x[2])) + cos(x[7] *
    obj_objfun_workspace->gain_el->contents) * sin(x[11] *
    obj_objfun_workspace->gain_az->contents) * (x[3] * x[3]))) +
    obj_objfun_workspace->S->contents * (j_a * j_a) * obj_objfun_workspace->
    rho->contents * e_sigma_1_tmp * k_sigma_1_tmp *
    (((obj_objfun_workspace->K_Cd->contents * (k_a * k_a) * x_sigma_1_tmp * (l_a
    * l_a) - 2.0 * obj_objfun_workspace->K_Cd->contents * (m_a * m_a) * x[12] *
       obj_objfun_workspace->flight_path_angle->contents *
       obj_objfun_workspace->gain_theta->contents) + obj_objfun_workspace->
      K_Cd->contents * (n_a * n_a) * (o_a * o_a)) +
     obj_objfun_workspace->Cd_zero->contents) / 2.0) / obj_objfun_workspace->
    m->contents;
  a = obj_objfun_workspace->gain_motor->contents;
  b_a = obj_objfun_workspace->gain_motor->contents;
  c_a = obj_objfun_workspace->gain_motor->contents;
  d_a = obj_objfun_workspace->gain_motor->contents;
  e_a = obj_objfun_workspace->gain_motor->contents;
  f_a = obj_objfun_workspace->gain_motor->contents;
  g_a = obj_objfun_workspace->gain_motor->contents;
  h_a = obj_objfun_workspace->gain_motor->contents;
  i_a = obj_objfun_workspace->V->contents;
  j_a = obj_objfun_workspace->gain_motor->contents;
  k_a = obj_objfun_workspace->gain_motor->contents;
  l_a = obj_objfun_workspace->gain_motor->contents;
  m_a = obj_objfun_workspace->gain_motor->contents;
  n_a = obj_objfun_workspace->V->contents;
  o_a = obj_objfun_workspace->V->contents;
  sigma_3 = ((((((((((((((((2.0 * obj_objfun_workspace->I_zz->contents *
    obj_objfun_workspace->p->contents * obj_objfun_workspace->r->contents - 2.0 *
    obj_objfun_workspace->I_xx->contents * obj_objfun_workspace->p->contents *
    obj_objfun_workspace->r->contents) - 2.0 * obj_objfun_workspace->
    I_yy->contents * obj_objfun_workspace->dv_global->contents[4]) + 2.0 *
    obj_objfun_workspace->K_p_T->contents * y_sigma_1_tmp * (a * a) *
    obj_objfun_workspace->l_z->contents * l_sigma_1_tmp) + 2.0 *
    obj_objfun_workspace->K_p_T->contents * sigma_5 * (b_a * b_a) *
    obj_objfun_workspace->l_z->contents * m_sigma_1_tmp) + 2.0 *
                        obj_objfun_workspace->K_p_T->contents * ab_sigma_1_tmp *
                        (c_a * c_a) * obj_objfun_workspace->l_z->contents *
                        n_sigma_1_tmp) + 2.0 * obj_objfun_workspace->
                       K_p_T->contents * bb_sigma_1_tmp * (d_a * d_a) *
                       obj_objfun_workspace->l_z->contents * o_sigma_1_tmp) -
                      2.0 * obj_objfun_workspace->K_p_M->contents *
                      y_sigma_1_tmp * (e_a * e_a) * sigma_6 * p_sigma_1_tmp) +
                     2.0 * obj_objfun_workspace->K_p_M->contents * sigma_5 *
                     (f_a * f_a) * g_sigma_1_tmp * q_sigma_1_tmp) - 2.0 *
                    obj_objfun_workspace->K_p_M->contents * ab_sigma_1_tmp *
                    (g_a * g_a) * h_sigma_1_tmp * r_sigma_1_tmp) + 2.0 *
                   obj_objfun_workspace->K_p_M->contents * bb_sigma_1_tmp * (h_a
    * h_a) * i_sigma_1_tmp * s_sigma_1_tmp) + obj_objfun_workspace->
                  Cm_zero->contents * obj_objfun_workspace->S->contents * (i_a *
    i_a) * obj_objfun_workspace->rho->contents *
                  obj_objfun_workspace->wing_chord->contents) + 2.0 *
                 obj_objfun_workspace->K_p_T->contents * (x[0] * x[0]) * (j_a *
    j_a) * obj_objfun_workspace->l_4->contents * sigma_6 * t_sigma_1_tmp) + 2.0 *
                obj_objfun_workspace->K_p_T->contents * (x[1] * x[1]) * (k_a *
    k_a) * obj_objfun_workspace->l_4->contents * g_sigma_1_tmp * u_sigma_1_tmp)
               - 2.0 * obj_objfun_workspace->K_p_T->contents * (x[2] * x[2]) *
               (l_a * l_a) * obj_objfun_workspace->l_3->contents * h_sigma_1_tmp
               * v_sigma_1_tmp) - 2.0 * obj_objfun_workspace->K_p_T->contents *
              (x[3] * x[3]) * (m_a * m_a) * obj_objfun_workspace->l_3->contents *
              i_sigma_1_tmp * w_sigma_1_tmp) - obj_objfun_workspace->
             Cm_alpha->contents * obj_objfun_workspace->S->contents * (n_a * n_a)
             * obj_objfun_workspace->flight_path_angle->contents *
             obj_objfun_workspace->rho->contents *
             obj_objfun_workspace->wing_chord->contents) +
    obj_objfun_workspace->Cm_alpha->contents * obj_objfun_workspace->S->contents
    * x[12] * (o_a * o_a) * obj_objfun_workspace->gain_theta->contents *
    obj_objfun_workspace->rho->contents * obj_objfun_workspace->
    wing_chord->contents;
  a = obj_objfun_workspace->gain_motor->contents;
  b_a = obj_objfun_workspace->gain_motor->contents;
  c_a = obj_objfun_workspace->gain_motor->contents;
  d_a = obj_objfun_workspace->gain_motor->contents;
  e_a = obj_objfun_workspace->gain_motor->contents;
  f_a = obj_objfun_workspace->gain_motor->contents;
  g_a = obj_objfun_workspace->gain_motor->contents;
  h_a = obj_objfun_workspace->gain_motor->contents;
  i_a = obj_objfun_workspace->gain_motor->contents;
  j_a = obj_objfun_workspace->gain_motor->contents;
  k_a = obj_objfun_workspace->gain_motor->contents;
  l_a = obj_objfun_workspace->gain_motor->contents;
  m_a = obj_objfun_workspace->V->contents;
  sigma_4 = ((((((((((((((2.0 * obj_objfun_workspace->I_yy->contents *
    obj_objfun_workspace->q->contents * obj_objfun_workspace->r->contents - 2.0 *
    obj_objfun_workspace->I_xx->contents * obj_objfun_workspace->
    dv_global->contents[3]) - 2.0 * obj_objfun_workspace->I_zz->contents *
    obj_objfun_workspace->q->contents * obj_objfun_workspace->r->contents) + 2.0
                        * obj_objfun_workspace->K_p_M->contents * (x[0] * x[0]) *
                        (a * a) * l_sigma_1_tmp) - 2.0 *
                       obj_objfun_workspace->K_p_M->contents * (x[1] * x[1]) *
                       (b_a * b_a) * m_sigma_1_tmp) + 2.0 *
                      obj_objfun_workspace->K_p_M->contents * (x[2] * x[2]) *
                      (c_a * c_a) * n_sigma_1_tmp) - 2.0 *
                     obj_objfun_workspace->K_p_M->contents * (x[3] * x[3]) *
                     (d_a * d_a) * o_sigma_1_tmp) + 2.0 *
                    obj_objfun_workspace->K_p_T->contents * (x[0] * x[0]) * (e_a
    * e_a) * obj_objfun_workspace->l_1->contents * sigma_6 * t_sigma_1_tmp) -
                   2.0 * obj_objfun_workspace->K_p_T->contents * (x[1] * x[1]) *
                   (f_a * f_a) * obj_objfun_workspace->l_1->contents *
                   g_sigma_1_tmp * u_sigma_1_tmp) - 2.0 *
                  obj_objfun_workspace->K_p_T->contents * (x[2] * x[2]) * (g_a *
    g_a) * obj_objfun_workspace->l_2->contents * h_sigma_1_tmp * v_sigma_1_tmp)
                 + 2.0 * obj_objfun_workspace->K_p_T->contents * (x[3] * x[3]) *
                 (h_a * h_a) * obj_objfun_workspace->l_2->contents *
                 i_sigma_1_tmp * w_sigma_1_tmp) + 2.0 *
                obj_objfun_workspace->K_p_T->contents * (x[0] * x[0]) * (i_a *
    i_a) * obj_objfun_workspace->l_z->contents * sigma_6 * p_sigma_1_tmp) + 2.0 *
               obj_objfun_workspace->K_p_T->contents * (x[1] * x[1]) * (j_a *
    j_a) * obj_objfun_workspace->l_z->contents * g_sigma_1_tmp * q_sigma_1_tmp)
              + 2.0 * obj_objfun_workspace->K_p_T->contents * (x[2] * x[2]) *
              (k_a * k_a) * obj_objfun_workspace->l_z->contents * h_sigma_1_tmp *
              r_sigma_1_tmp) + 2.0 * obj_objfun_workspace->K_p_T->contents * (x
              [3] * x[3]) * (l_a * l_a) * obj_objfun_workspace->l_z->contents *
             i_sigma_1_tmp * s_sigma_1_tmp) + obj_objfun_workspace->
    CL_aileron->contents * obj_objfun_workspace->S->contents * (m_a * m_a) * x
    [14] * obj_objfun_workspace->gain_ailerons->contents *
    obj_objfun_workspace->rho->contents;
  a = obj_objfun_workspace->gain_motor->contents;
  b_a = obj_objfun_workspace->gain_motor->contents;
  c_a = obj_objfun_workspace->gain_motor->contents;
  d_a = obj_objfun_workspace->gain_motor->contents;
  e_a = obj_objfun_workspace->gain_motor->contents;
  f_a = obj_objfun_workspace->gain_motor->contents;
  g_a = obj_objfun_workspace->gain_motor->contents;
  h_a = obj_objfun_workspace->gain_motor->contents;
  i_a = obj_objfun_workspace->gain_motor->contents;
  j_a = obj_objfun_workspace->gain_motor->contents;
  k_a = obj_objfun_workspace->gain_motor->contents;
  l_a = obj_objfun_workspace->gain_motor->contents;
  sigma_5 = (((((((((((((obj_objfun_workspace->I_zz->contents *
    obj_objfun_workspace->dv_global->contents[5] - obj_objfun_workspace->
    I_xx->contents * obj_objfun_workspace->p->contents * obj_objfun_workspace->
    q->contents) + obj_objfun_workspace->I_yy->contents *
                        obj_objfun_workspace->p->contents *
                        obj_objfun_workspace->q->contents) +
                       obj_objfun_workspace->K_p_T->contents * y_sigma_1_tmp *
                       (a * a) * obj_objfun_workspace->l_1->contents *
                       l_sigma_1_tmp) - obj_objfun_workspace->K_p_T->contents *
                      sigma_5 * (b_a * b_a) * obj_objfun_workspace->
                      l_1->contents * m_sigma_1_tmp) -
                     obj_objfun_workspace->K_p_T->contents * ab_sigma_1_tmp *
                     (c_a * c_a) * obj_objfun_workspace->l_2->contents *
                     n_sigma_1_tmp) + obj_objfun_workspace->K_p_T->contents *
                    bb_sigma_1_tmp * (d_a * d_a) * obj_objfun_workspace->
                    l_2->contents * o_sigma_1_tmp) - obj_objfun_workspace->
                   K_p_M->contents * y_sigma_1_tmp * (e_a * e_a) * sigma_6 *
                   t_sigma_1_tmp) + obj_objfun_workspace->K_p_M->contents *
                  sigma_5 * (f_a * f_a) * g_sigma_1_tmp * u_sigma_1_tmp) -
                 obj_objfun_workspace->K_p_M->contents * ab_sigma_1_tmp * (g_a *
    g_a) * h_sigma_1_tmp * v_sigma_1_tmp) + obj_objfun_workspace->
                K_p_M->contents * bb_sigma_1_tmp * (h_a * h_a) * i_sigma_1_tmp *
                w_sigma_1_tmp) - obj_objfun_workspace->K_p_T->contents * (x[0] *
    x[0]) * (i_a * i_a) * obj_objfun_workspace->l_4->contents * sigma_6 *
               p_sigma_1_tmp) - obj_objfun_workspace->K_p_T->contents * (x[1] *
    x[1]) * (j_a * j_a) * obj_objfun_workspace->l_4->contents * g_sigma_1_tmp *
              q_sigma_1_tmp) + obj_objfun_workspace->K_p_T->contents * (x[2] *
              x[2]) * (k_a * k_a) * obj_objfun_workspace->l_3->contents *
             h_sigma_1_tmp * r_sigma_1_tmp) + obj_objfun_workspace->
    K_p_T->contents * (x[3] * x[3]) * (l_a * l_a) * obj_objfun_workspace->
    l_3->contents * i_sigma_1_tmp * s_sigma_1_tmp;
  a = obj_objfun_workspace->Cl_alpha->contents;
  b_a = obj_objfun_workspace->gain_theta->contents;
  c_a = obj_objfun_workspace->Cl_alpha->contents;
  d_a = obj_objfun_workspace->Cl_alpha->contents;
  e_a = obj_objfun_workspace->flight_path_angle->contents;
  sigma_6 = ((obj_objfun_workspace->K_Cd->contents * (a * a) * x_sigma_1_tmp *
              (b_a * b_a) - 2.0 * obj_objfun_workspace->K_Cd->contents * (c_a *
    c_a) * x[12] * obj_objfun_workspace->flight_path_angle->contents *
              obj_objfun_workspace->gain_theta->contents) +
             obj_objfun_workspace->K_Cd->contents * (d_a * d_a) * (e_a * e_a)) +
    obj_objfun_workspace->Cd_zero->contents;
  a = obj_objfun_workspace->V->contents;
  b_a = obj_objfun_workspace->V->contents;
  c_a = obj_objfun_workspace->V->contents;
  d_a = obj_objfun_workspace->V->contents;
  e_a = obj_objfun_workspace->gain_motor->contents;
  f_a = obj_objfun_workspace->gain_motor->contents;
  g_a = obj_objfun_workspace->gain_motor->contents;
  h_a = obj_objfun_workspace->V->contents;
  sigma_6 = (100.0 * (((((f_sigma_1_tmp * (obj_objfun_workspace->S->contents *
    (a * a) * obj_objfun_workspace->rho->contents * sigma_6 * sigma_1_tmp *
    c_sigma_1_tmp / 2.0 - obj_objfun_workspace->Cl_alpha->contents *
    obj_objfun_workspace->S->contents * (b_a * b_a) * obj_objfun_workspace->
    rho->contents * b_sigma_1_tmp * (obj_objfun_workspace->
    flight_path_angle->contents - x[12] * obj_objfun_workspace->
    gain_theta->contents) / 2.0) + e_sigma_1_tmp * d_sigma_1_tmp *
    (obj_objfun_workspace->Cl_alpha->contents * obj_objfun_workspace->
     S->contents * (c_a * c_a) * obj_objfun_workspace->rho->contents *
     sigma_1_tmp * (obj_objfun_workspace->flight_path_angle->contents - x[12] *
                    obj_objfun_workspace->gain_theta->contents) / 2.0 +
     obj_objfun_workspace->S->contents * (d_a * d_a) * obj_objfun_workspace->
     rho->contents * sigma_6 * b_sigma_1_tmp * c_sigma_1_tmp / 2.0)) +
    obj_objfun_workspace->K_p_T->contents * (e_a * e_a) * f_sigma_1_tmp *
    cb_sigma_1_tmp) - obj_objfun_workspace->K_p_T->contents * (f_a * f_a) *
                        e_sigma_1_tmp * d_sigma_1_tmp * (((cos(x[4] *
    obj_objfun_workspace->gain_el->contents) * cos(x[8] *
    obj_objfun_workspace->gain_az->contents) * (x[0] * x[0]) + cos(x[5] *
    obj_objfun_workspace->gain_el->contents) * cos(x[9] *
    obj_objfun_workspace->gain_az->contents) * (x[1] * x[1])) + cos(x[6] *
    obj_objfun_workspace->gain_el->contents) * cos(x[10] *
    obj_objfun_workspace->gain_az->contents) * (x[2] * x[2])) + cos(x[7] *
    obj_objfun_workspace->gain_el->contents) * cos(x[11] *
    obj_objfun_workspace->gain_az->contents) * (x[3] * x[3]))) +
                       obj_objfun_workspace->K_p_T->contents * (g_a * g_a) *
                       d_sigma_1_tmp * j_sigma_1_tmp * (((cos(x[4] *
    obj_objfun_workspace->gain_el->contents) * sin(x[8] *
    obj_objfun_workspace->gain_az->contents) * (x[0] * x[0]) + cos(x[5] *
    obj_objfun_workspace->gain_el->contents) * sin(x[9] *
    obj_objfun_workspace->gain_az->contents) * (x[1] * x[1])) + cos(x[6] *
    obj_objfun_workspace->gain_el->contents) * sin(x[10] *
    obj_objfun_workspace->gain_az->contents) * (x[2] * x[2])) + cos(x[7] *
    obj_objfun_workspace->gain_el->contents) * sin(x[11] *
    obj_objfun_workspace->gain_az->contents) * (x[3] * x[3]))) -
                      obj_objfun_workspace->S->contents * (h_a * h_a) *
                      obj_objfun_workspace->rho->contents * sigma_6 *
                      d_sigma_1_tmp * j_sigma_1_tmp * k_sigma_1_tmp / 2.0) /
             obj_objfun_workspace->m->contents - 100.0 *
             obj_objfun_workspace->dv_global->contents[2]) + 981.0;
  a = obj_objfun_workspace->W_act_motor->contents;
  b_a = x[0] - obj_objfun_workspace->desired_motor_value->contents /
    obj_objfun_workspace->gain_motor->contents;
  c_a = obj_objfun_workspace->W_act_motor->contents;
  d_a = x[1] - obj_objfun_workspace->desired_motor_value->contents /
    obj_objfun_workspace->gain_motor->contents;
  e_a = obj_objfun_workspace->W_act_motor->contents;
  f_a = x[2] - obj_objfun_workspace->desired_motor_value->contents /
    obj_objfun_workspace->gain_motor->contents;
  g_a = obj_objfun_workspace->W_act_motor->contents;
  h_a = x[3] - obj_objfun_workspace->desired_motor_value->contents /
    obj_objfun_workspace->gain_motor->contents;
  i_a = obj_objfun_workspace->W_act_phi->contents;
  j_a = x[13] - obj_objfun_workspace->desired_phi_value->contents /
    obj_objfun_workspace->gain_phi->contents;
  k_a = obj_objfun_workspace->W_act_theta->contents;
  l_a = x[12] - obj_objfun_workspace->desired_theta_value->contents /
    obj_objfun_workspace->gain_theta->contents;
  m_a = obj_objfun_workspace->W_act_tilt_el->contents;
  n_a = x[4] - obj_objfun_workspace->desired_el_value->contents /
    obj_objfun_workspace->gain_el->contents;
  o_a = obj_objfun_workspace->W_act_tilt_el->contents;
  p_a = x[5] - obj_objfun_workspace->desired_el_value->contents /
    obj_objfun_workspace->gain_el->contents;
  q_a = obj_objfun_workspace->W_act_tilt_el->contents;
  r_a = x[6] - obj_objfun_workspace->desired_el_value->contents /
    obj_objfun_workspace->gain_el->contents;
  s_a = obj_objfun_workspace->W_act_tilt_el->contents;
  t_a = x[7] - obj_objfun_workspace->desired_el_value->contents /
    obj_objfun_workspace->gain_el->contents;
  u_a = obj_objfun_workspace->W_act_ailerons->contents;
  v_a = x[14] - obj_objfun_workspace->desired_ailerons_value->contents /
    obj_objfun_workspace->gain_ailerons->contents;
  w_a = obj_objfun_workspace->W_act_tilt_az->contents;
  g_sigma_1_tmp = x[8] - obj_objfun_workspace->desired_az_value->contents /
    obj_objfun_workspace->gain_az->contents;
  h_sigma_1_tmp = obj_objfun_workspace->W_act_tilt_az->contents;
  i_sigma_1_tmp = x[9] - obj_objfun_workspace->desired_az_value->contents /
    obj_objfun_workspace->gain_az->contents;
  l_sigma_1_tmp = obj_objfun_workspace->W_act_tilt_az->contents;
  m_sigma_1_tmp = x[10] - obj_objfun_workspace->desired_az_value->contents /
    obj_objfun_workspace->gain_az->contents;
  n_sigma_1_tmp = obj_objfun_workspace->W_act_tilt_az->contents;
  o_sigma_1_tmp = x[11] - obj_objfun_workspace->desired_az_value->contents /
    obj_objfun_workspace->gain_az->contents;
  p_sigma_1_tmp = obj_objfun_workspace->W_dv_1->contents;
  q_sigma_1_tmp = obj_objfun_workspace->W_dv_2->contents;
  r_sigma_1_tmp = obj_objfun_workspace->W_dv_3->contents;
  s_sigma_1_tmp = obj_objfun_workspace->W_dv_4->contents;
  t_sigma_1_tmp = obj_objfun_workspace->I_xx->contents;
  u_sigma_1_tmp = obj_objfun_workspace->W_dv_5->contents;
  v_sigma_1_tmp = obj_objfun_workspace->I_yy->contents;
  w_sigma_1_tmp = obj_objfun_workspace->W_dv_6->contents;
  y_sigma_1_tmp = obj_objfun_workspace->I_zz->contents;
  *fval = (((((obj_objfun_workspace->gamma_quadratic_du->contents *
               ((((((((((((((a * a * (b_a * b_a) + c_a * c_a * (d_a * d_a)) +
    e_a * e_a * (f_a * f_a)) + g_a * g_a * (h_a * h_a)) + i_a * i_a * (j_a * j_a))
    + k_a * k_a * (l_a * l_a)) + m_a * m_a * (n_a * n_a)) + o_a * o_a * (p_a *
    p_a)) + q_a * q_a * (r_a * r_a)) + s_a * s_a * (t_a * t_a)) + u_a * u_a *
                    (v_a * v_a)) + w_a * w_a * (g_sigma_1_tmp * g_sigma_1_tmp))
                  + h_sigma_1_tmp * h_sigma_1_tmp * (i_sigma_1_tmp *
    i_sigma_1_tmp)) + l_sigma_1_tmp * l_sigma_1_tmp * (m_sigma_1_tmp *
    m_sigma_1_tmp)) + n_sigma_1_tmp * n_sigma_1_tmp * (o_sigma_1_tmp *
    o_sigma_1_tmp)) + p_sigma_1_tmp * p_sigma_1_tmp * (sigma_1 * sigma_1)) +
              q_sigma_1_tmp * q_sigma_1_tmp * (sigma_2 * sigma_2)) +
             r_sigma_1_tmp * r_sigma_1_tmp * (sigma_6 * sigma_6) / 10000.0) +
            s_sigma_1_tmp * s_sigma_1_tmp * (sigma_4 * sigma_4) / (4.0 *
             (t_sigma_1_tmp * t_sigma_1_tmp))) + u_sigma_1_tmp * u_sigma_1_tmp *
           (sigma_3 * sigma_3) / (4.0 * (v_sigma_1_tmp * v_sigma_1_tmp))) +
    w_sigma_1_tmp * w_sigma_1_tmp * (sigma_5 * sigma_5) / (y_sigma_1_tmp *
    y_sigma_1_tmp);
  *status = 1;
  if (rtIsInf(*fval) || rtIsNaN(*fval)) {
    if (rtIsNaN(*fval)) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }

  if (*status == 1) {
    *status = 1;
  }
}

/*
 * Arguments    : const d_struct_T *obj_objfun_workspace
 *                const double x[15]
 *                double grad_workspace[16]
 *                double *fval
 *                int *status
 * Return Type  : void
 */
static void evalObjAndConstrAndDerivatives(const d_struct_T
  *obj_objfun_workspace, const double x[15], double grad_workspace[16], double
  *fval, int *status)
{
  double varargout_2[15];
  compute_cost_and_gradient_fcn(obj_objfun_workspace->dv_global,
    obj_objfun_workspace->gain_theta, obj_objfun_workspace->Cl_alpha,
    obj_objfun_workspace->S, obj_objfun_workspace->V, obj_objfun_workspace->rho,
    obj_objfun_workspace->flight_path_angle, obj_objfun_workspace->Beta,
    obj_objfun_workspace->K_Cd, obj_objfun_workspace->Cd_zero,
    obj_objfun_workspace->gain_phi, obj_objfun_workspace->K_p_T,
    obj_objfun_workspace->gain_motor, obj_objfun_workspace->gain_el,
    obj_objfun_workspace->gain_az, obj_objfun_workspace->m,
    obj_objfun_workspace->I_zz, obj_objfun_workspace->p, obj_objfun_workspace->r,
    obj_objfun_workspace->I_xx, obj_objfun_workspace->I_yy,
    obj_objfun_workspace->l_z, obj_objfun_workspace->K_p_M,
    obj_objfun_workspace->Cm_zero, obj_objfun_workspace->wing_chord,
    obj_objfun_workspace->l_4, obj_objfun_workspace->l_3,
    obj_objfun_workspace->Cm_alpha, obj_objfun_workspace->q,
    obj_objfun_workspace->l_1, obj_objfun_workspace->l_2,
    obj_objfun_workspace->CL_aileron, obj_objfun_workspace->gain_ailerons,
    obj_objfun_workspace->W_dv_4, obj_objfun_workspace->W_dv_6,
    obj_objfun_workspace->W_act_motor, obj_objfun_workspace->gamma_quadratic_du,
    obj_objfun_workspace->desired_motor_value, obj_objfun_workspace->W_dv_5,
    obj_objfun_workspace->W_dv_3, obj_objfun_workspace->W_dv_1,
    obj_objfun_workspace->W_dv_2, obj_objfun_workspace->W_act_tilt_el,
    obj_objfun_workspace->desired_el_value, obj_objfun_workspace->W_act_tilt_az,
    obj_objfun_workspace->desired_az_value, obj_objfun_workspace->W_act_theta,
    obj_objfun_workspace->desired_theta_value, obj_objfun_workspace->W_act_phi,
    obj_objfun_workspace->desired_phi_value,
    obj_objfun_workspace->W_act_ailerons,
    obj_objfun_workspace->desired_ailerons_value, x, fval, varargout_2);
  memcpy(&grad_workspace[0], &varargout_2[0], 15U * sizeof(double));
  *status = 1;
  if (rtIsInf(*fval) || rtIsNaN(*fval)) {
    if (rtIsNaN(*fval)) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    int idx_current;
    bool allFinite;
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 15)) {
      allFinite = ((!rtIsInf(grad_workspace[idx_current])) && (!rtIsNaN
        (grad_workspace[idx_current])));
      idx_current++;
    }

    if (!allFinite) {
      idx_current--;
      if (rtIsNaN(grad_workspace[idx_current])) {
        *status = -3;
      } else if (grad_workspace[idx_current] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }

  if (*status == 1) {
    *status = 1;
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
  int idx;
  int ix0;
  int k;
  bool guard1 = false;
  ix0 = mrows * ncols;
  guard1 = false;
  if (ix0 > 0) {
    for (idx = 0; idx < ncols; idx++) {
      int iy0;
      ix0 = idx << 4;
      iy0 = 31 * idx;
      for (k = 0; k < mrows; k++) {
        obj->QR[iy0 + k] = A[ix0 + k];
      }
    }

    guard1 = true;
  } else if (ix0 == 0) {
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
      ix0 = mrows;
    } else {
      ix0 = ncols;
    }

    obj->minRowCol = ix0;
    memset(&obj->tau[0], 0, 31U * sizeof(double));
    if (ix0 >= 1) {
      qrf(obj->QR, mrows, ncols, ix0, obj->tau);
    }
  }
}

/*
 * Arguments    : d_struct_T *objfun_workspace
 *                const double lb[15]
 *                const double ub[15]
 *                j_struct_T *obj
 * Return Type  : void
 */
static void factoryConstruct(d_struct_T *objfun_workspace, const double lb[15],
  const double ub[15], j_struct_T *obj)
{
  int i;
  bool bv[15];
  bool b;
  obj->objfun.workspace = *objfun_workspace;
  obj->f_1 = 0.0;
  obj->f_2 = 0.0;
  obj->nVar = 15;
  obj->mIneq = 0;
  obj->mEq = 0;
  obj->numEvals = 0;
  obj->SpecifyObjectiveGradient = true;
  obj->SpecifyConstraintGradient = false;
  obj->isEmptyNonlcon = true;
  obj->FiniteDifferenceType = 0;
  for (i = 0; i < 15; i++) {
    bv[i] = obj->hasUB[i];
  }

  b = false;
  i = 0;
  while ((!b) && (i + 1 <= 15)) {
    obj->hasLB[i] = ((!rtIsInf(lb[i])) && (!rtIsNaN(lb[i])));
    bv[i] = ((!rtIsInf(ub[i])) && (!rtIsNaN(ub[i])));
    if (obj->hasLB[i] || bv[i]) {
      b = true;
    }

    i++;
  }

  while (i + 1 <= 15) {
    obj->hasLB[i] = ((!rtIsInf(lb[i])) && (!rtIsNaN(lb[i])));
    bv[i] = ((!rtIsInf(ub[i])) && (!rtIsNaN(ub[i])));
    i++;
  }

  for (i = 0; i < 15; i++) {
    obj->hasUB[i] = bv[i];
  }

  obj->hasBounds = b;
}

/*
 * Arguments    : double workspace[496]
 *                double xCurrent[16]
 *                const k_struct_T *workingset
 *                f_struct_T *qrmanager
 * Return Type  : bool
 */
static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const k_struct_T *workingset, f_struct_T *qrmanager)
{
  double B[496];
  int ar;
  int b_i;
  int iAcol;
  int ia;
  int ic;
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
      workspace[iAcol] = workingset->bwset[iAcol];
      workspace[iAcol + 31] = workingset->bwset[iAcol];
    }

    if (mWConstr != 0) {
      i = ((mWConstr - 1) << 4) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 16) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (ia = iAcol; ia <= i1; ia++) {
          c += workingset->ATwset[ia - 1] * xCurrent[ia - iAcol];
        }

        i1 = (iAcol - 1) >> 4;
        workspace[i1] += -c;
      }
    }

    if (mWConstr >= nVar) {
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (ia = 0; ia < nVar; ia++) {
        iAcol = 31 * ia;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] = workingset->ATwset[ia + (jBcol << 4)];
        }

        qrmanager->jpvt[ia] = ia + 1;
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
      for (k = 0; k <= 31; k += 31) {
        i = k + 1;
        i1 = k + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
        }
      }

      jBcol = -1;
      for (k = 0; k <= 31; k += 31) {
        ar = -1;
        i = k + 1;
        i1 = k + nVar;
        for (ic = i; ic <= i1; ic++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q[(iAcol + ar) + 1] * B[(iAcol + jBcol) + 1];
          }

          workspace[ic - 1] += c;
          ar += 31;
        }

        jBcol += 31;
      }

      for (ar = 0; ar < 2; ar++) {
        jBcol = 31 * ar - 1;
        for (k = nVar; k >= 1; k--) {
          iAcol = 31 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager->QR[k + iAcol];
            for (b_i = 0; b_i <= k - 2; b_i++) {
              i1 = (b_i + jBcol) + 1;
              workspace[i1] -= workspace[i] * qrmanager->QR[(b_i + iAcol) + 1];
            }
          }
        }
      }
    } else {
      factorQR(qrmanager, workingset->ATwset, nVar, mWConstr);
      computeQ_(qrmanager, qrmanager->minRowCol);
      for (ar = 0; ar < 2; ar++) {
        jBcol = 31 * ar;
        for (b_i = 0; b_i < mWConstr; b_i++) {
          iAcol = 31 * b_i;
          ia = b_i + jBcol;
          c = workspace[ia];
          for (k = 0; k < b_i; k++) {
            c -= qrmanager->QR[k + iAcol] * workspace[k + jBcol];
          }

          workspace[ia] = c / qrmanager->QR[b_i + iAcol];
        }
      }

      memcpy(&B[0], &workspace[0], 496U * sizeof(double));
      for (k = 0; k <= 31; k += 31) {
        i = k + 1;
        i1 = k + nVar;
        if (i <= i1) {
          memset(&workspace[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
        }
      }

      jBcol = 0;
      for (k = 0; k <= 31; k += 31) {
        ar = -1;
        i = jBcol + 1;
        i1 = jBcol + mWConstr;
        for (b_i = i; b_i <= i1; b_i++) {
          iAcol = k + 1;
          ia = k + nVar;
          for (ic = iAcol; ic <= ia; ic++) {
            workspace[ic - 1] += B[b_i - 1] * qrmanager->Q[(ar + ic) - k];
          }

          ar += 31;
        }

        jBcol += 31;
      }
    }

    iAcol = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iAcol <= nVar - 1) {
        if (rtIsInf(workspace[iAcol]) || rtIsNaN(workspace[iAcol])) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[iAcol + 31];
          if (rtIsInf(c) || rtIsNaN(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            iAcol++;
          }
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
          memcpy(&xCurrent[0], &workspace[0], nVar * sizeof(double));
        } else {
          memcpy(&xCurrent[0], &workspace[31], nVar * sizeof(double));
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
 *                double *alpha
 *                bool *newBlocking
 *                int *constrType
 *                int *constrIdx
 * Return Type  : void
 */
static void feasibleratiotest(const double solution_xstar[16], const double
  solution_searchDir[16], int workingset_nVar, const double workingset_lb[16],
  const double workingset_ub[16], const int workingset_indexLB[16], const int
  workingset_indexUB[16], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[31], const int
  workingset_nWConstr[5], bool isPhaseOne, double *alpha, bool *newBlocking, int
  *constrType, int *constrIdx)
{
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double pk_corrected;
  double ratio;
  int i;
  int idx;
  int totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
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
      int i1;
      i1 = workingset_indexLB[idx];
      pk_corrected = -solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[i1 - 1] - workingset_lb[i1 - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < *alpha) {
          *alpha = pk_corrected;
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
      if (pk_corrected < *alpha) {
        *alpha = pk_corrected;
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
    for (idx = 0; idx < totalUB; idx++) {
      i = workingset_indexUB[idx];
      pk_corrected = solution_searchDir[i - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[i - 1] - workingset_ub[i - 1]) -
          phaseOneCorrectionX;
        pk_corrected = fmin(fabs(ratio), 1.0E-6 - ratio) / pk_corrected;
        if (pk_corrected < *alpha) {
          *alpha = pk_corrected;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (!isPhaseOne) {
    if ((*newBlocking) && (*alpha > 1.0)) {
      *newBlocking = false;
    }

    *alpha = fmin(*alpha, 1.0);
  }
}

/*
 * Arguments    : d_struct_T *fun_workspace
 *                const double x0[15]
 *                const double lb[15]
 *                const double ub[15]
 *                double x[15]
 *                double *fval
 *                double *exitflag
 *                double *output_iterations
 *                double *output_funcCount
 *                char output_algorithm[3]
 *                double *output_constrviolation
 *                double *output_stepsize
 *                double *output_lssteplength
 *                double *output_firstorderopt
 * Return Type  : void
 */
static void fmincon(d_struct_T *fun_workspace, const double x0[15], const double
                    lb[15], const double ub[15], double x[15], double *fval,
                    double *exitflag, double *output_iterations, double
                    *output_funcCount, char output_algorithm[3], double
                    *output_constrviolation, double *output_stepsize, double
                    *output_lssteplength, double *output_firstorderopt)
{
  b_struct_T MeritFunction;
  d_struct_T FcnEvaluator_objfun_workspace;
  d_struct_T b_fun_workspace;
  e_struct_T memspace;
  f_struct_T QRManager;
  g_struct_T CholManager;
  i_struct_T TrialState;
  j_struct_T unusedExpr;
  k_struct_T WorkingSet;
  l_struct_T FcnEvaluator;
  struct_T QPObjective;
  double scale;
  double y;
  int b_i;
  int colOffsetATw;
  int i;
  int idx;
  int mFixed;
  int mUB;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  output_algorithm[0] = 's';
  output_algorithm[1] = 'q';
  output_algorithm[2] = 'p';
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
  FcnEvaluator_objfun_workspace = *fun_workspace;
  b_fun_workspace = *fun_workspace;
  factoryConstruct(&b_fun_workspace, lb, ub, &unusedExpr);
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
  colOffsetATw = 0;
  mUB = 0;
  mFixed = 0;
  for (idx = 0; idx < 15; idx++) {
    bool guard1 = false;
    y = lb[idx];
    guard1 = false;
    if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
      if (fabs(y - ub[idx]) < 1.0E-6) {
        mFixed++;
        WorkingSet.indexFixed[mFixed - 1] = idx + 1;
      } else {
        colOffsetATw++;
        WorkingSet.indexLB[colOffsetATw - 1] = idx + 1;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      y = ub[idx];
      if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
        mUB++;
        WorkingSet.indexUB[mUB - 1] = idx + 1;
      }
    }
  }

  i = (colOffsetATw + mUB) + mFixed;
  WorkingSet.mConstr = i;
  WorkingSet.mConstrOrig = i;
  WorkingSet.mConstrMax = 31;
  obj_tmp[0] = (signed char)mFixed;
  obj_tmp[1] = 0;
  obj_tmp[2] = 0;
  obj_tmp[3] = (signed char)colOffsetATw;
  obj_tmp[4] = (signed char)mUB;
  b_obj_tmp[0] = (signed char)mFixed;
  b_obj_tmp[1] = 0;
  b_obj_tmp[2] = 0;
  b_obj_tmp[3] = (signed char)(colOffsetATw + 1);
  b_obj_tmp[4] = (signed char)mUB;
  WorkingSet.isActiveIdx[0] = 1;
  WorkingSet.isActiveIdx[1] = mFixed;
  WorkingSet.isActiveIdx[2] = 0;
  WorkingSet.isActiveIdx[3] = 0;
  WorkingSet.isActiveIdx[4] = colOffsetATw;
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
  WorkingSet.isActiveIdxPhaseOne[4] = colOffsetATw + 1;
  WorkingSet.isActiveIdxPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    WorkingSet.isActiveIdxPhaseOne[i + 1] += WorkingSet.isActiveIdxPhaseOne[i];
  }

  for (b_i = 0; b_i < 6; b_i++) {
    WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
    WorkingSet.isActiveIdxRegPhaseOne[b_i] = WorkingSet.isActiveIdxPhaseOne[b_i];
  }

  for (idx = 0; idx < colOffsetATw; idx++) {
    b_i = WorkingSet.indexLB[idx];
    TrialState.xstarsqp[b_i - 1] = fmax(TrialState.xstarsqp[b_i - 1], lb[b_i - 1]);
  }

  for (idx = 0; idx < mUB; idx++) {
    b_i = WorkingSet.indexUB[idx];
    TrialState.xstarsqp[b_i - 1] = fmin(TrialState.xstarsqp[b_i - 1], ub[b_i - 1]);
  }

  for (idx = 0; idx < mFixed; idx++) {
    b_i = WorkingSet.indexFixed[idx];
    TrialState.xstarsqp[b_i - 1] = ub[b_i - 1];
  }

  evalObjAndConstrAndDerivatives(&FcnEvaluator_objfun_workspace,
    TrialState.xstarsqp, TrialState.grad, &TrialState.sqpFval, &i);
  TrialState.FunctionEvaluations = 1;
  for (idx = 0; idx < colOffsetATw; idx++) {
    WorkingSet.lb[WorkingSet.indexLB[idx] - 1] = -lb[WorkingSet.indexLB[idx] - 1]
      + x0[WorkingSet.indexLB[idx] - 1];
  }

  for (idx = 0; idx < mUB; idx++) {
    WorkingSet.ub[WorkingSet.indexUB[idx] - 1] = ub[WorkingSet.indexUB[idx] - 1]
      - x0[WorkingSet.indexUB[idx] - 1];
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
  i = WorkingSet.sizes[0];
  for (idx = 0; idx < i; idx++) {
    WorkingSet.Wid[idx] = 1;
    WorkingSet.Wlocalidx[idx] = idx + 1;
    WorkingSet.isActiveConstr[idx] = true;
    colOffsetATw = idx << 4;
    b_i = WorkingSet.indexFixed[idx];
    if (b_i - 2 >= 0) {
      memset(&WorkingSet.ATwset[colOffsetATw], 0, (((b_i + colOffsetATw) -
               colOffsetATw) - 1) * sizeof(double));
    }

    WorkingSet.ATwset[(WorkingSet.indexFixed[idx] + colOffsetATw) - 1] = 1.0;
    b_i = WorkingSet.indexFixed[idx] + 1;
    mUB = WorkingSet.nVar;
    if (b_i <= mUB) {
      memset(&WorkingSet.ATwset[(b_i + colOffsetATw) + -1], 0, ((((mUB +
                 colOffsetATw) - b_i) - colOffsetATw) + 1) * sizeof(double));
    }

    WorkingSet.bwset[idx] = WorkingSet.ub[WorkingSet.indexFixed[idx] - 1];
  }

  double Hessian[225];
  MeritFunction.initFval = TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initConstrViolationEq = 0.0;
  MeritFunction.initConstrViolationIneq = 0.0;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  FcnEvaluator.objfun.workspace = FcnEvaluator_objfun_workspace;
  FcnEvaluator.nVar = 15;
  FcnEvaluator.mCineq = 0;
  FcnEvaluator.mCeq = 0;
  FcnEvaluator.NonFiniteSupport = true;
  FcnEvaluator.SpecifyObjectiveGradient = true;
  FcnEvaluator.SpecifyConstraintGradient = false;
  FcnEvaluator.ScaleProblem = false;
  b_driver(lb, ub, &TrialState, &MeritFunction, &FcnEvaluator, &memspace,
           &WorkingSet, Hessian, &QRManager, &CholManager, &QPObjective);
  *fval = TrialState.sqpFval;
  *exitflag = TrialState.sqpExitFlag;
  *output_iterations = TrialState.sqpIterations;
  *output_funcCount = TrialState.FunctionEvaluations;
  *output_constrviolation = MeritFunction.nlpPrimalFeasError;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (i = 0; i < 15; i++) {
    double absxk;
    x[i] = TrialState.xstarsqp[i];
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
    int subMatrixDim;
    LD_diagOffset = k << 5;
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      obj->workspace_ = obj->FMat[(LD_diagOffset + jA) + 1];
    }

    y = obj->workspace_;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 33;
          i1 = subMatrixDim + jA;
          for (ijA = i; ijA <= i1 + 33; ijA++) {
            obj->FMat[ijA - 1] += obj->workspace_ * temp;
          }
        }

        jA += 31;
      }
    }

    for (jA = 0; jA <= subMatrixDim; jA++) {
      i = (LD_diagOffset + jA) + 1;
      obj->FMat[i] /= obj->FMat[LD_diagOffset];
    }
  }
}

/*
 * Arguments    : const double H[225]
 *                const double f[16]
 *                i_struct_T *solution
 *                e_struct_T *memspace
 *                k_struct_T *workingset
 *                f_struct_T *qrmanager
 *                g_struct_T *cholmanager
 *                struct_T *objective
 *                const char options_SolverName[7]
 *                double options_StepTolerance
 *                double options_ObjectiveLimit
 *                int runTimeOptions_MaxIterations
 * Return Type  : void
 */
static void iterate(const double H[225], const double f[16], i_struct_T
                    *solution, e_struct_T *memspace, k_struct_T *workingset,
                    f_struct_T *qrmanager, g_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations)
{
  static const char b[7] = { 'f', 'm', 'i', 'n', 'c', 'o', 'n' };

  double c;
  double minLambda;
  double s;
  double temp_tmp;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int ia;
  int idx;
  int iyend;
  int n;
  int nActiveConstr;
  int nVar_tmp_tmp;
  bool subProblemChanged;
  bool updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar_tmp_tmp = workingset->nVar;
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
      int idxRotGCol;
      bool guard1 = false;
      bool guard2 = false;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          nActiveConstr = (workingset->nActiveConstr - 1) << 4;
          iyend = qrmanager->mrows;
          idxRotGCol = qrmanager->ncols + 1;
          if (iyend <= idxRotGCol) {
            idxRotGCol = iyend;
          }

          qrmanager->minRowCol = idxRotGCol;
          idxRotGCol = 31 * qrmanager->ncols;
          if (qrmanager->mrows != 0) {
            iyend = idxRotGCol + qrmanager->mrows;
            if (idxRotGCol + 1 <= iyend) {
              memset(&qrmanager->QR[idxRotGCol], 0, (iyend - idxRotGCol) *
                     sizeof(double));
            }

            n = 31 * (qrmanager->mrows - 1) + 1;
            for (idx = 1; idx <= n; idx += 31) {
              c = 0.0;
              iyend = (idx + qrmanager->mrows) - 1;
              for (ia = idx; ia <= iyend; ia++) {
                c += qrmanager->Q[ia - 1] * workingset->ATwset[(nActiveConstr +
                  ia) - idx];
              }

              iyend = idxRotGCol + div_nde_s32_floor(idx - 1, 31);
              qrmanager->QR[iyend] += c;
            }
          }

          qrmanager->ncols++;
          qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            idxRotGCol = 31 * (qrmanager->ncols - 1);
            n = (idx + idxRotGCol) + 1;
            temp_tmp = qrmanager->QR[n];
            xrotg(&qrmanager->QR[idx + idxRotGCol], &temp_tmp, &c, &s);
            qrmanager->QR[n] = temp_tmp;
            iyend = 31 * idx;
            n = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              for (nActiveConstr = 0; nActiveConstr < n; nActiveConstr++) {
                idxRotGCol = iyend + nActiveConstr;
                minLambda = qrmanager->Q[idxRotGCol + 31];
                temp_tmp = qrmanager->Q[idxRotGCol];
                qrmanager->Q[idxRotGCol + 31] = c * minLambda - s * temp_tmp;
                qrmanager->Q[idxRotGCol] = c * temp_tmp + s * minLambda;
              }
            }
          }
          break;

         case -1:
          deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          factorQR(qrmanager, workingset->ATwset, nVar_tmp_tmp,
                   workingset->nActiveConstr);
          computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        iyend = memcmp(&options_SolverName[0], &b[0], 7);
        compute_deltax(H, solution, memspace, qrmanager, cholmanager, objective,
                       iyend == 0);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((b_xnrm2(nVar_tmp_tmp, solution->searchDir) <
                    options_StepTolerance) || (workingset->nActiveConstr >=
                    nVar_tmp_tmp)) {
          guard2 = true;
        } else {
          feasibleratiotest(solution->xstar, solution->searchDir,
                            workingset->nVar, workingset->lb, workingset->ub,
                            workingset->indexLB, workingset->indexUB,
                            workingset->sizes, workingset->isActiveIdx,
                            workingset->isActiveConstr, workingset->nWConstr,
                            TYPE == 5, &minLambda, &updateFval, &n, &iyend);
          if (updateFval) {
            switch (n) {
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

          if ((nVar_tmp_tmp >= 1) && (!(minLambda == 0.0))) {
            iyend = nVar_tmp_tmp - 1;
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
        memset(&solution->searchDir[0], 0, nVar_tmp_tmp * sizeof(double));
        guard2 = true;
      }

      if (guard2) {
        nActiveConstr = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          bool b_guard1 = false;
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
              bool b_guard2 = false;
              idx = qrmanager->ncols;
              b_guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                iyend = qrmanager->mrows + 31 * (qrmanager->ncols - 1);
                while ((idx > qrmanager->mrows) && (fabs(qrmanager->QR[iyend - 1])
                        >= minLambda)) {
                  idx--;
                  iyend -= 31;
                }

                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  b_guard2 = true;
                }
              } else {
                b_guard2 = true;
              }

              if (b_guard2) {
                iyend = idx + 31 * (idx - 1);
                while ((idx >= 1) && (fabs(qrmanager->QR[iyend - 1]) >=
                                      minLambda)) {
                  idx--;
                  iyend -= 32;
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
            n = qrmanager->ncols;
            xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q,
                  objective->grad, memspace->workspace_double);
            if (qrmanager->ncols != 0) {
              for (idx = n; idx >= 1; idx--) {
                iyend = (idx + (idx - 1) * 31) - 1;
                memspace->workspace_double[idx - 1] /= qrmanager->QR[iyend];
                for (ia = 0; ia <= idx - 2; ia++) {
                  idxRotGCol = (idx - ia) - 2;
                  memspace->workspace_double[idxRotGCol] -=
                    memspace->workspace_double[idx - 1] * qrmanager->QR[(iyend -
                    ia) - 1];
                }
              }
            }

            for (idx = 0; idx < nActiveConstr; idx++) {
              solution->lambda[idx] = -memspace->workspace_double[idx];
            }
          }
        }

        if ((solution->state != -7) || (workingset->nActiveConstr > nVar_tmp_tmp))
        {
          nActiveConstr = -1;
          minLambda = 0.0;
          n = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          iyend = workingset->nActiveConstr;
          for (idx = n; idx <= iyend; idx++) {
            temp_tmp = solution->lambda[idx - 1];
            if (temp_tmp < minLambda) {
              minLambda = temp_tmp;
              nActiveConstr = idx - 1;
            }
          }

          if (nActiveConstr + 1 == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = nActiveConstr + 1;
            subProblemChanged = true;
            iyend = workingset->Wid[nActiveConstr] - 1;
            workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
              Wid[nActiveConstr] - 1] + workingset->Wlocalidx[nActiveConstr]) -
              2] = false;
            workingset->Wid[nActiveConstr] = workingset->Wid
              [workingset->nActiveConstr - 1];
            workingset->Wlocalidx[nActiveConstr] = workingset->
              Wlocalidx[workingset->nActiveConstr - 1];
            n = workingset->nVar;
            for (idx = 0; idx < n; idx++) {
              workingset->ATwset[idx + (nActiveConstr << 4)] =
                workingset->ATwset[idx + ((workingset->nActiveConstr - 1) << 4)];
            }

            workingset->bwset[nActiveConstr] = workingset->bwset
              [workingset->nActiveConstr - 1];
            workingset->nActiveConstr--;
            workingset->nWConstr[iyend]--;
            solution->lambda[nActiveConstr] = 0.0;
          }
        } else {
          nActiveConstr = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          iyend = workingset->nActiveConstr - 1;
          idxRotGCol = workingset->Wid[iyend] - 1;
          workingset->isActiveConstr[(workingset->isActiveIdx[idxRotGCol] +
            workingset->Wlocalidx[iyend]) - 2] = false;
          workingset->nActiveConstr--;
          workingset->nWConstr[idxRotGCol]--;
          solution->lambda[nActiveConstr - 1] = 0.0;
        }

        updateFval = false;
        guard1 = true;
      }

      if (guard1) {
        solution->iterations++;
        iyend = objective->nvar - 1;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = b_maxConstraintViolation(workingset,
            solution->xstar);
          minLambda = solution->maxConstr;
          if (objective->objtype == 5) {
            minLambda = solution->maxConstr - solution->xstar[iyend];
          }

          if (minLambda > 1.0E-6) {
            bool nonDegenerateWset;
            if (iyend >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (iyend + 1) *
                     sizeof(double));
            }

            nonDegenerateWset = feasibleX0ForWorkingSet
              (memspace->workspace_double, solution->searchDir, workingset,
               qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }

            activeSetChangeID = 0;
            minLambda = b_maxConstraintViolation(workingset, solution->searchDir);
            if (minLambda < solution->maxConstr) {
              if (iyend >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (iyend + 1)
                       * sizeof(double));
              }

              solution->maxConstr = minLambda;
            }
          }
        }

        if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
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
  int ia;
  int iac;
  int ix;
  ix = 0;
  if (obj_hasLinear) {
    if (obj_nvar - 1 >= 0) {
      memcpy(&workspace[0], &f[0], obj_nvar * sizeof(double));
    }

    ix = 1;
  }

  if (obj_nvar != 0) {
    int i;
    if (ix != 1) {
      memset(&workspace[0], 0, obj_nvar * sizeof(double));
    }

    ix = 0;
    i = obj_nvar * (obj_nvar - 1) + 1;
    for (iac = 1; obj_nvar < 0 ? iac >= i : iac <= i; iac += obj_nvar) {
      double c;
      int i1;
      c = 0.5 * x[ix];
      i1 = (iac + obj_nvar) - 1;
      for (ia = iac; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        workspace[i2] += H[ia - 1] * c;
      }

      ix++;
    }
  }
}

/*
 * Arguments    : const k_struct_T *obj
 *                const double x[496]
 *                int ix0
 * Return Type  : double
 */
static double maxConstraintViolation(const k_struct_T *obj, const double x[496],
  int ix0)
{
  double v;
  int idx;
  int mFixed;
  int mLB;
  int mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  v = 0.0;
  if (obj->sizes[3] > 0) {
    for (idx = 0; idx < mLB; idx++) {
      int idxLB;
      idxLB = obj->indexLB[idx] - 1;
      v = fmax(v, -x[(ix0 + idxLB) - 1] - obj->lb[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    for (idx = 0; idx < mUB; idx++) {
      mLB = obj->indexUB[idx] - 1;
      v = fmax(v, x[(ix0 + mLB) - 1] - obj->ub[mLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    for (idx = 0; idx < mFixed; idx++) {
      v = fmax(v, fabs(x[(ix0 + obj->indexFixed[idx]) - 2] - obj->ub
                       [obj->indexFixed[idx] - 1]));
    }
  }

  return v;
}

/*
 * Arguments    : const double x[12]
 *                double *ex
 *                int *idx
 * Return Type  : void
 */
static void minimum(const double x[12], double *ex, int *idx)
{
  int k;
  if (!rtIsNaN(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 13)) {
      if (!rtIsNaN(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (*idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    int i;
    *ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 13; k++) {
      double d;
      d = x[k - 1];
      if (*ex > d) {
        *ex = d;
        *idx = k;
      }
    }
  }
}

/*
 * Arguments    : const double A_data[]
 *                const int A_size[2]
 *                const double B[18]
 *                double Y_data[]
 *                int *Y_size
 * Return Type  : void
 */
static void mldivide(const double A_data[], const int A_size[2], const double B
                     [18], double Y_data[], int *Y_size)
{
  double b_A_data[216];
  double b_B[18];
  double tau_data[12];
  double vn1_data[12];
  double vn2_data[12];
  double work_data[12];
  int i;
  int ix0;
  int iy;
  int j;
  int k;
  signed char jpvt_data[12];
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    double absxk;
    double scale;
    double smax;
    double t;
    int A_size_idx_1;
    int b_i;
    int kend;
    int n;
    A_size_idx_1 = A_size[1];
    kend = 18 * A_size[1];
    memcpy(&b_A_data[0], &A_data[0], kend * sizeof(double));
    n = A_size[1];
    kend = A_size[1];
    memset(&tau_data[0], 0, kend * sizeof(double));
    kend = A_size[1];
    memset(&jpvt_data[0], 0, kend * sizeof(signed char));
    for (k = 0; k < n; k++) {
      jpvt_data[k] = (signed char)(k + 1);
    }

    kend = A_size[1];
    memset(&work_data[0], 0, kend * sizeof(double));
    kend = A_size[1];
    memset(&vn1_data[0], 0, kend * sizeof(double));
    kend = A_size[1];
    memset(&vn2_data[0], 0, kend * sizeof(double));
    for (j = 0; j < n; j++) {
      ix0 = j * 18;
      smax = 0.0;
      scale = 3.3121686421112381E-170;
      kend = ix0 + 18;
      for (k = ix0 + 1; k <= kend; k++) {
        absxk = fabs(A_data[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          smax = smax * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          smax += t * t;
        }
      }

      absxk = scale * sqrt(smax);
      vn1_data[j] = absxk;
      vn2_data[j] = absxk;
    }

    for (i = 0; i < n; i++) {
      int ii;
      int ip1;
      int nmi;
      int pvt;
      ip1 = i + 2;
      ii = i * 18 + i;
      nmi = n - i;
      if (nmi < 1) {
        kend = -1;
      } else {
        kend = 0;
        if (nmi > 1) {
          smax = fabs(vn1_data[i]);
          for (k = 2; k <= nmi; k++) {
            scale = fabs(vn1_data[(i + k) - 1]);
            if (scale > smax) {
              kend = k - 1;
              smax = scale;
            }
          }
        }
      }

      pvt = i + kend;
      if (pvt != i) {
        kend = pvt * 18;
        iy = i * 18;
        for (k = 0; k < 18; k++) {
          ix0 = kend + k;
          smax = b_A_data[ix0];
          b_i = iy + k;
          b_A_data[ix0] = b_A_data[b_i];
          b_A_data[b_i] = smax;
        }

        kend = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[i];
        jpvt_data[i] = (signed char)kend;
        vn1_data[pvt] = vn1_data[i];
        vn2_data[pvt] = vn2_data[i];
      }

      t = b_A_data[ii];
      ix0 = ii + 2;
      tau_data[i] = 0.0;
      smax = c_xnrm2(17 - i, b_A_data, ii + 2);
      if (smax != 0.0) {
        absxk = b_A_data[ii];
        scale = rt_hypotd_snf(absxk, smax);
        if (absxk >= 0.0) {
          scale = -scale;
        }

        if (fabs(scale) < 1.0020841800044864E-292) {
          kend = 0;
          b_i = (ii - i) + 18;
          do {
            kend++;
            for (k = ix0; k <= b_i; k++) {
              b_A_data[k - 1] *= 9.9792015476736E+291;
            }

            scale *= 9.9792015476736E+291;
            t *= 9.9792015476736E+291;
          } while ((fabs(scale) < 1.0020841800044864E-292) && (kend < 20));

          scale = rt_hypotd_snf(t, c_xnrm2(17 - i, b_A_data, ii + 2));
          if (t >= 0.0) {
            scale = -scale;
          }

          tau_data[i] = (scale - t) / scale;
          smax = 1.0 / (t - scale);
          for (k = ix0; k <= b_i; k++) {
            b_A_data[k - 1] *= smax;
          }

          for (k = 0; k < kend; k++) {
            scale *= 1.0020841800044864E-292;
          }

          t = scale;
        } else {
          tau_data[i] = (scale - absxk) / scale;
          smax = 1.0 / (absxk - scale);
          b_i = (ii - i) + 18;
          for (k = ix0; k <= b_i; k++) {
            b_A_data[k - 1] *= smax;
          }

          t = scale;
        }
      }

      b_A_data[ii] = t;
      if (i + 1 < n) {
        int lastv;
        b_A_data[ii] = 1.0;
        k = ii + 19;
        if (tau_data[i] != 0.0) {
          bool exitg2;
          lastv = 18 - i;
          kend = (ii - i) + 17;
          while ((lastv > 0) && (b_A_data[kend] == 0.0)) {
            lastv--;
            kend--;
          }

          nmi -= 2;
          exitg2 = false;
          while ((!exitg2) && (nmi + 1 > 0)) {
            int exitg1;
            kend = (ii + nmi * 18) + 18;
            ix0 = kend;
            do {
              exitg1 = 0;
              if (ix0 + 1 <= kend + lastv) {
                if (b_A_data[ix0] != 0.0) {
                  exitg1 = 1;
                } else {
                  ix0++;
                }
              } else {
                nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          nmi = -1;
        }

        if (lastv > 0) {
          if (nmi + 1 != 0) {
            if (nmi >= 0) {
              memset(&work_data[0], 0, (nmi + 1) * sizeof(double));
            }

            b_i = (ii + 18 * nmi) + 19;
            for (iy = k; iy <= b_i; iy += 18) {
              smax = 0.0;
              pvt = (iy + lastv) - 1;
              for (ix0 = iy; ix0 <= pvt; ix0++) {
                smax += b_A_data[ix0 - 1] * b_A_data[(ii + ix0) - iy];
              }

              kend = div_nde_s32_floor((iy - ii) - 19, 18);
              work_data[kend] += smax;
            }
          }

          if (!(-tau_data[i] == 0.0)) {
            kend = ii;
            for (j = 0; j <= nmi; j++) {
              absxk = work_data[j];
              if (absxk != 0.0) {
                smax = absxk * -tau_data[i];
                b_i = kend + 19;
                pvt = lastv + kend;
                for (ix0 = b_i; ix0 <= pvt + 18; ix0++) {
                  b_A_data[ix0 - 1] += b_A_data[((ii + ix0) - kend) - 19] * smax;
                }
              }

              kend += 18;
            }
          }
        }

        b_A_data[ii] = t;
      }

      for (j = ip1; j <= n; j++) {
        kend = i + (j - 1) * 18;
        absxk = vn1_data[j - 1];
        if (absxk != 0.0) {
          smax = fabs(b_A_data[kend]) / absxk;
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }

          scale = absxk / vn2_data[j - 1];
          scale = smax * (scale * scale);
          if (scale <= 1.4901161193847656E-8) {
            absxk = c_xnrm2(17 - i, b_A_data, kend + 2);
            vn1_data[j - 1] = absxk;
            vn2_data[j - 1] = absxk;
          } else {
            vn1_data[j - 1] = absxk * sqrt(smax);
          }
        }
      }
    }

    iy = 0;
    smax = 3.9968028886505635E-14 * fabs(b_A_data[0]);
    while ((iy < A_size_idx_1) && (!(fabs(b_A_data[iy + 18 * iy]) <= smax))) {
      iy++;
    }

    memcpy(&b_B[0], &B[0], 18U * sizeof(double));
    *Y_size = A_size[1];
    memset(&Y_data[0], 0, A_size_idx_1 * sizeof(double));
    for (j = 0; j < A_size_idx_1; j++) {
      if (tau_data[j] != 0.0) {
        smax = b_B[j];
        b_i = j + 2;
        for (i = b_i; i < 19; i++) {
          smax += b_A_data[(i + 18 * j) - 1] * b_B[i - 1];
        }

        smax *= tau_data[j];
        if (smax != 0.0) {
          b_B[j] -= smax;
          for (i = b_i; i < 19; i++) {
            b_B[i - 1] -= b_A_data[(i + 18 * j) - 1] * smax;
          }
        }
      }
    }

    for (i = 0; i < iy; i++) {
      Y_data[jpvt_data[i] - 1] = b_B[i];
    }

    for (j = iy; j >= 1; j--) {
      signed char i1;
      i1 = jpvt_data[j - 1];
      kend = 18 * (j - 1);
      Y_data[i1 - 1] /= b_A_data[(j + kend) - 1];
      for (i = 0; i <= j - 2; i++) {
        ix0 = jpvt_data[i] - 1;
        Y_data[ix0] -= Y_data[jpvt_data[j - 1] - 1] * b_A_data[i + kend];
      }
    }
  }
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
  int i;
  memset(&tau[0], 0, 31U * sizeof(double));
  memset(&work[0], 0, 31U * sizeof(double));
  for (i = 0; i < nfxd; i++) {
    double d;
    int ii;
    int mmi;
    ii = i * 31 + i;
    mmi = m - i;
    if (i + 1 < m) {
      atmp = A[ii];
      d = xzlarfg(mmi, &atmp, A, ii + 2);
      tau[i] = d;
      A[ii] = atmp;
    } else {
      d = 0.0;
      tau[i] = 0.0;
    }

    if (i + 1 < n) {
      atmp = A[ii];
      A[ii] = 1.0;
      xzlarf(mmi, (n - i) - 1, ii + 1, d, A, ii + 32, work);
      A[ii] = atmp;
    }
  }
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * sqrt(y * y + 1.0);
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Arguments    : k_struct_T *obj
 *                int PROBLEM_TYPE
 * Return Type  : void
 */
static void setProblemType(k_struct_T *obj, int PROBLEM_TYPE)
{
  int i;
  int idx;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 15;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
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

      i = obj->sizes[0];
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
        i = obj->sizesNormal[4];
        for (idx = 0; idx <= i; idx++) {
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
          i = obj->sizesRegularized[4];
          for (idx = 0; idx < i; idx++) {
            obj->isActiveConstr[obj->isActiveIdxRegularized[4] + idx] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1];
          }
        }

        i = obj->isActiveIdx[4];
        i1 = obj->isActiveIdxRegularized[4] - 1;
        if (i <= i1) {
          memset(&obj->isActiveConstr[i + -1], 0, ((i1 - i) + 1) * sizeof(bool));
        }

        idxStartIneq = obj->isActiveIdx[2];
        i = obj->nActiveConstr;
        for (idx = idxStartIneq; idx <= i; idx++) {
          idx_lb = ((idx - 1) << 4) - 1;
          if (obj->Wid[idx - 1] == 3) {
            i1 = obj->Wlocalidx[idx - 1] + 14;
            if (i1 >= 16) {
              memset(&obj->ATwset[idx_lb + 16], 0, (((i1 + idx_lb) - idx_lb) -
                      15) * sizeof(double));
            }

            obj->ATwset[(obj->Wlocalidx[idx - 1] + idx_lb) + 15] = -1.0;
            i1 = obj->Wlocalidx[idx - 1] + 16;
            if (i1 <= 15) {
              memset(&obj->ATwset[i1 + idx_lb], 0, (((idx_lb - i1) - idx_lb) +
                      16) * sizeof(double));
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

      i = obj->sizes[0];
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
        i = obj->sizesNormal[4];
        for (idx = 0; idx <= i; idx++) {
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
  int idx;
  if (WorkingSet_nActiveConstr != 0) {
    int idxOffset;
    int mAll;
    mAll = ((WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4]) -
      1;
    for (idx = 0; idx <= mAll; idx++) {
      workspace[idx] = lambda[idx];
      lambda[idx] = 0.0;
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
 *                i_struct_T *TrialState
 *                b_struct_T *MeritFunction
 *                e_struct_T *memspace
 *                k_struct_T *WorkingSet
 *                f_struct_T *QRManager
 *                g_struct_T *CholManager
 *                struct_T *QPObjective
 *                h_struct_T *qpoptions
 * Return Type  : bool
 */
static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], i_struct_T *TrialState, b_struct_T
                 *MeritFunction, e_struct_T *memspace, k_struct_T *WorkingSet,
                 f_struct_T *QRManager, g_struct_T *CholManager, struct_T
                 *QPObjective, h_struct_T *qpoptions)
{
  h_struct_T b_qpoptions;
  double dv[16];
  double oldDirIdx;
  double s;
  int idx;
  int idxStartIneq;
  int mUB;
  int nVarOrig;
  int nVar_tmp_tmp;
  bool checkBoundViolation;
  bool stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet->nVar - 1;
  if (*STEP_TYPE != 3) {
    memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (nVar_tmp_tmp + 1) *
           sizeof(double));
  } else if (nVar_tmp_tmp >= 0) {
    memcpy(&TrialState->searchDir[0], &TrialState->xstar[0], (nVar_tmp_tmp + 1) *
           sizeof(double));
  }

  int exitg1;
  bool guard1 = false;
  do {
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
        if (nVar_tmp_tmp >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (nVar_tmp_tmp +
                  1) * sizeof(double));
        }

        guard1 = true;
      }
      break;

     case 2:
      {
        double beta;
        idxStartIneq = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
        mUB = WorkingSet->nActiveConstr;
        for (nVarOrig = idxStartIneq; nVarOrig <= mUB; nVarOrig++) {
          WorkingSet->isActiveConstr[(WorkingSet->isActiveIdx[WorkingSet->
            Wid[nVarOrig - 1] - 1] + WorkingSet->Wlocalidx[nVarOrig - 1]) - 2] =
            false;
        }

        WorkingSet->nWConstr[2] = 0;
        WorkingSet->nWConstr[3] = 0;
        WorkingSet->nWConstr[4] = 0;
        WorkingSet->nActiveConstr = WorkingSet->nWConstr[0] +
          WorkingSet->nWConstr[1];
        memcpy(&dv[0], &TrialState->xstar[0], 16U * sizeof(double));
        idxStartIneq = WorkingSet->sizes[3];
        mUB = WorkingSet->sizes[4];
        for (idx = 0; idx < idxStartIneq; idx++) {
          oldDirIdx = WorkingSet->lb[WorkingSet->indexLB[idx] - 1];
          if (-dv[WorkingSet->indexLB[idx] - 1] > oldDirIdx) {
            if (rtIsInf(ub[WorkingSet->indexLB[idx] - 1])) {
              dv[WorkingSet->indexLB[idx] - 1] = -oldDirIdx + fabs(oldDirIdx);
            } else {
              dv[WorkingSet->indexLB[idx] - 1] = (WorkingSet->ub
                [WorkingSet->indexLB[idx] - 1] - oldDirIdx) / 2.0;
            }
          }
        }

        for (idx = 0; idx < mUB; idx++) {
          oldDirIdx = WorkingSet->ub[WorkingSet->indexUB[idx] - 1];
          if (dv[WorkingSet->indexUB[idx] - 1] > oldDirIdx) {
            if (rtIsInf(lb[WorkingSet->indexUB[idx] - 1])) {
              dv[WorkingSet->indexUB[idx] - 1] = oldDirIdx - fabs(oldDirIdx);
            } else {
              dv[WorkingSet->indexUB[idx] - 1] = (oldDirIdx - WorkingSet->
                lb[WorkingSet->indexUB[idx] - 1]) / 2.0;
            }
          }
        }

        memcpy(&TrialState->xstar[0], &dv[0], 16U * sizeof(double));
        nVarOrig = WorkingSet->nVar;
        beta = 0.0;
        for (idx = 0; idx < nVarOrig; idx++) {
          beta += Hessian[idx + 15 * idx];
        }

        beta /= (double)WorkingSet->nVar;
        if (TrialState->sqpIterations <= 1) {
          mUB = QPObjective->nvar;
          if (QPObjective->nvar < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (QPObjective->nvar > 1) {
              oldDirIdx = fabs(TrialState->grad[0]);
              for (idx = 2; idx <= mUB; idx++) {
                s = fabs(TrialState->grad[idx - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = idx;
                  oldDirIdx = s;
                }
              }
            }
          }

          oldDirIdx = 100.0 * fmax(1.0, fabs(TrialState->grad[idxStartIneq - 1]));
        } else {
          mUB = WorkingSet->mConstr;
          if (WorkingSet->mConstr < 1) {
            idxStartIneq = 0;
          } else {
            idxStartIneq = 1;
            if (WorkingSet->mConstr > 1) {
              oldDirIdx = fabs(TrialState->lambdasqp[0]);
              for (idx = 2; idx <= mUB; idx++) {
                s = fabs(TrialState->lambdasqp[idx - 1]);
                if (s > oldDirIdx) {
                  idxStartIneq = idx;
                  oldDirIdx = s;
                }
              }
            }
          }

          oldDirIdx = fabs(TrialState->lambdasqp[idxStartIneq - 1]);
        }

        QPObjective->nvar = WorkingSet->nVar;
        QPObjective->beta = beta;
        QPObjective->rho = oldDirIdx;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 4;
        setProblemType(WorkingSet, 2);
        idxStartIneq = qpoptions->MaxIterations;
        qpoptions->MaxIterations = (qpoptions->MaxIterations + WorkingSet->nVar)
          - nVarOrig;
        memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
        b_qpoptions = *qpoptions;
        driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
               CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
        qpoptions->MaxIterations = idxStartIneq;
        if (TrialState->state != -6) {
          MeritFunction->phi = TrialState->sqpFval;
          MeritFunction->linearizedConstrViol = 0.0;
          MeritFunction->penaltyParam = 1.0;
          MeritFunction->phiPrimePlus = fmin((TrialState->fstar - oldDirIdx *
            0.0) - beta / 2.0 * 0.0, 0.0);
          mUB = WorkingSet->isActiveIdx[2];
          idxStartIneq = WorkingSet->nActiveConstr;
          for (idx = mUB; idx <= idxStartIneq; idx++) {
            if (WorkingSet->Wid[idx - 1] == 3) {
              TrialState->lambda[idx - 1] *= (double)memspace->
                workspace_int[WorkingSet->Wlocalidx[idx - 1] - 1];
            }
          }
        }

        QPObjective->nvar = nVarOrig;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 3;
        setProblemType(WorkingSet, 3);
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                     WorkingSet->Wlocalidx, memspace->workspace_double);
        if (nVar_tmp_tmp >= 0) {
          memcpy(&TrialState->delta_x[0], &TrialState->xstar[0], (nVar_tmp_tmp +
                  1) * sizeof(double));
        }

        guard1 = true;
      }
      break;

     default:
      idxStartIneq = WorkingSet->nVar - 1;
      memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0],
             (idxStartIneq + 1) * sizeof(double));
      memcpy(&TrialState->socDirection[0], &TrialState->xstar[0], (idxStartIneq
              + 1) * sizeof(double));
      memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambda[0], 31U *
             sizeof(double));
      memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (idxStartIneq + 1)
             * sizeof(double));
      memcpy(&dv[0], &TrialState->grad[0], 16U * sizeof(double));
      b_qpoptions = *qpoptions;
      driver(Hessian, dv, TrialState, memspace, WorkingSet, QRManager,
             CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
      for (idx = 0; idx <= idxStartIneq; idx++) {
        oldDirIdx = TrialState->socDirection[idx];
        TrialState->socDirection[idx] = TrialState->xstar[idx] -
          TrialState->socDirection[idx];
        TrialState->xstar[idx] = oldDirIdx;
      }

      stepSuccess = (b_xnrm2(idxStartIneq + 1, TrialState->socDirection) <= 2.0 *
                     b_xnrm2(idxStartIneq + 1, TrialState->xstar));
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
        for (idx = 0; idx <= nVar_tmp_tmp; idx++) {
          TrialState->delta_x[idx] = TrialState->xstar[idx] +
            TrialState->socDirection[idx];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        oldDirIdx = 0.0;
        s = 1.0;
        for (idx = 0; idx < 15; idx++) {
          oldDirIdx = fmax(oldDirIdx, fabs(TrialState->grad[idx]));
          s = fmax(s, fabs(TrialState->xstar[idx]));
        }

        oldDirIdx = fmax(2.2204460492503131E-16, oldDirIdx / s);
        for (nVarOrig = 0; nVarOrig < 15; nVarOrig++) {
          idxStartIneq = 15 * nVarOrig;
          for (idx = 0; idx < nVarOrig; idx++) {
            Hessian[idxStartIneq + idx] = 0.0;
          }

          Hessian[nVarOrig + 15 * nVarOrig] = oldDirIdx;
          idxStartIneq += nVarOrig;
          mUB = 13 - nVarOrig;
          if (mUB >= 0) {
            memset(&Hessian[idxStartIneq + 1], 0, (((mUB + idxStartIneq) -
                     idxStartIneq) + 1) * sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    idxStartIneq = WorkingSet->sizes[3];
    mUB = WorkingSet->sizes[4];
    memcpy(&dv[0], &TrialState->delta_x[0], 16U * sizeof(double));
    for (idx = 0; idx < idxStartIneq; idx++) {
      oldDirIdx = dv[WorkingSet->indexLB[idx] - 1];
      s = (TrialState->xstarsqp[WorkingSet->indexLB[idx] - 1] + oldDirIdx) -
        lb[WorkingSet->indexLB[idx] - 1];
      if (s < 0.0) {
        dv[WorkingSet->indexLB[idx] - 1] = oldDirIdx - s;
        TrialState->xstar[WorkingSet->indexLB[idx] - 1] -= s;
      }
    }

    for (idx = 0; idx < mUB; idx++) {
      oldDirIdx = dv[WorkingSet->indexUB[idx] - 1];
      s = (ub[WorkingSet->indexUB[idx] - 1] - TrialState->xstarsqp
           [WorkingSet->indexUB[idx] - 1]) - oldDirIdx;
      if (s < 0.0) {
        dv[WorkingSet->indexUB[idx] - 1] = oldDirIdx + s;
        TrialState->xstar[WorkingSet->indexUB[idx] - 1] += s;
      }
    }

    memcpy(&TrialState->delta_x[0], &dv[0], 16U * sizeof(double));
  }

  return stepSuccess;
}

/*
 * Arguments    : const double A[72]
 *                double U[72]
 *                double s[6]
 *                double V[36]
 * Return Type  : void
 */
static void svd(const double A[72], double U[72], double s[6], double V[36])
{
  double b_A[72];
  double Vf[36];
  double work[12];
  double b_s[6];
  double e[6];
  double nrm;
  double rt;
  double sm;
  double snorm;
  double sqds;
  int i;
  int ii;
  int jj;
  int k;
  int m;
  int q;
  int qp1;
  int qp1jj;
  int qq;
  memcpy(&b_A[0], &A[0], 72U * sizeof(double));
  for (i = 0; i < 6; i++) {
    b_s[i] = 0.0;
    e[i] = 0.0;
  }

  memset(&work[0], 0, 12U * sizeof(double));
  memset(&U[0], 0, 72U * sizeof(double));
  memset(&Vf[0], 0, 36U * sizeof(double));
  for (q = 0; q < 6; q++) {
    bool apply_transform;
    qp1 = q + 2;
    qq = (q + 12 * q) + 1;
    apply_transform = false;
    nrm = d_xnrm2(12 - q, b_A, qq);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq - 1] < 0.0) {
        nrm = -nrm;
      }

      b_s[q] = nrm;
      if (fabs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qp1jj = (qq - q) + 11;
        for (k = qq; k <= qp1jj; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        qp1jj = (qq - q) + 11;
        for (k = qq; k <= qp1jj; k++) {
          b_A[k - 1] /= b_s[q];
        }
      }

      b_A[qq - 1]++;
      b_s[q] = -b_s[q];
    } else {
      b_s[q] = 0.0;
    }

    for (jj = qp1; jj < 7; jj++) {
      i = q + 12 * (jj - 1);
      if (apply_transform) {
        xaxpy(12 - q, -(xdotc(12 - q, b_A, qq, b_A, i + 1) / b_A[q + 12 * q]),
              qq, b_A, i + 1);
      }

      e[jj - 1] = b_A[i];
    }

    for (ii = q + 1; ii < 13; ii++) {
      i = (ii + 12 * q) - 1;
      U[i] = b_A[i];
    }

    if (q + 1 <= 4) {
      nrm = e_xnrm2(5 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }

        nrm = e[q];
        if (fabs(e[q]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[q];
          for (k = qp1; k < 7; k++) {
            e[k - 1] *= nrm;
          }
        } else {
          for (k = qp1; k < 7; k++) {
            e[k - 1] /= nrm;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (ii = qp1; ii < 13; ii++) {
          work[ii - 1] = 0.0;
        }

        for (jj = qp1; jj < 7; jj++) {
          b_xaxpy(11 - q, e[jj - 1], b_A, (q + 12 * (jj - 1)) + 2, work, q + 2);
        }

        for (jj = qp1; jj < 7; jj++) {
          c_xaxpy(11 - q, -e[jj - 1] / e[q + 1], work, q + 2, b_A, (q + 12 * (jj
                    - 1)) + 2);
        }
      }

      for (ii = qp1; ii < 7; ii++) {
        Vf[(ii + 6 * q) - 1] = e[ii - 1];
      }
    }
  }

  m = 4;
  e[4] = b_A[64];
  e[5] = 0.0;
  for (q = 5; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 12 * q;
    if (b_s[q] != 0.0) {
      for (jj = qp1; jj < 7; jj++) {
        i = (q + 12 * (jj - 1)) + 1;
        xaxpy(12 - q, -(xdotc(12 - q, U, qq + 1, U, i) / U[qq]), qq + 1, U, i);
      }

      for (ii = q + 1; ii < 13; ii++) {
        i = (ii + 12 * q) - 1;
        U[i] = -U[i];
      }

      U[qq]++;
      for (ii = 0; ii < q; ii++) {
        U[ii + 12 * q] = 0.0;
      }
    } else {
      memset(&U[q * 12], 0, 12U * sizeof(double));
      U[qq] = 1.0;
    }
  }

  for (q = 5; q >= 0; q--) {
    if ((q + 1 <= 4) && (e[q] != 0.0)) {
      qp1 = q + 2;
      i = (q + 6 * q) + 2;
      for (jj = qp1; jj < 7; jj++) {
        qp1jj = (q + 6 * (jj - 1)) + 2;
        d_xaxpy(5 - q, -(b_xdotc(5 - q, Vf, i, Vf, qp1jj) / Vf[i - 1]), i, Vf,
                qp1jj);
      }
    }

    for (ii = 0; ii < 6; ii++) {
      Vf[ii + 6 * q] = 0.0;
    }

    Vf[q + 6 * q] = 1.0;
  }

  qq = 0;
  snorm = 0.0;
  for (q = 0; q < 6; q++) {
    nrm = b_s[q];
    if (nrm != 0.0) {
      rt = fabs(nrm);
      nrm /= rt;
      b_s[q] = rt;
      if (q + 1 < 6) {
        e[q] /= nrm;
      }

      i = 12 * q;
      qp1jj = i + 12;
      for (k = i + 1; k <= qp1jj; k++) {
        U[k - 1] *= nrm;
      }
    }

    if (q + 1 < 6) {
      nrm = e[q];
      if (nrm != 0.0) {
        rt = fabs(nrm);
        nrm = rt / nrm;
        e[q] = rt;
        b_s[q + 1] *= nrm;
        i = 6 * (q + 1);
        qp1jj = i + 6;
        for (k = i + 1; k <= qp1jj; k++) {
          Vf[k - 1] *= nrm;
        }
      }
    }

    snorm = fmax(snorm, fmax(fabs(b_s[q]), fabs(e[q])));
  }

  while ((m + 2 > 0) && (qq < 75)) {
    bool exitg1;
    jj = m + 1;
    ii = m + 1;
    exitg1 = false;
    while (!(exitg1 || (ii == 0))) {
      nrm = fabs(e[ii - 1]);
      if ((nrm <= 2.2204460492503131E-16 * (fabs(b_s[ii - 1]) + fabs(b_s[ii]))) ||
          (nrm <= 1.0020841800044864E-292) || ((qq > 20) && (nrm <=
            2.2204460492503131E-16 * snorm))) {
        e[ii - 1] = 0.0;
        exitg1 = true;
      } else {
        ii--;
      }
    }

    if (ii == m + 1) {
      i = 4;
    } else {
      qp1jj = m + 2;
      i = m + 2;
      exitg1 = false;
      while ((!exitg1) && (i >= ii)) {
        qp1jj = i;
        if (i == ii) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (i < m + 2) {
            nrm = fabs(e[i - 1]);
          }

          if (i > ii + 1) {
            nrm += fabs(e[i - 2]);
          }

          rt = fabs(b_s[i - 1]);
          if ((rt <= 2.2204460492503131E-16 * nrm) || (rt <=
               1.0020841800044864E-292)) {
            b_s[i - 1] = 0.0;
            exitg1 = true;
          } else {
            i--;
          }
        }
      }

      if (qp1jj == ii) {
        i = 3;
      } else if (qp1jj == m + 2) {
        i = 1;
      } else {
        i = 2;
        ii = qp1jj;
      }
    }

    switch (i) {
     case 1:
      {
        rt = e[m];
        e[m] = 0.0;
        for (k = jj; k >= ii + 1; k--) {
          xrotg(&b_s[k - 1], &rt, &sm, &sqds);
          if (k > ii + 1) {
            double b;
            b = e[k - 2];
            rt = -sqds * b;
            e[k - 2] = b * sm;
          }

          xrot(Vf, 6 * (k - 1) + 1, 6 * (m + 1) + 1, sm, sqds);
        }
      }
      break;

     case 2:
      {
        rt = e[ii - 1];
        e[ii - 1] = 0.0;
        for (k = ii + 1; k <= m + 2; k++) {
          double b;
          xrotg(&b_s[k - 1], &rt, &sm, &sqds);
          b = e[k - 1];
          rt = -sqds * b;
          e[k - 1] = b * sm;
          b_xrot(U, 12 * (k - 1) + 1, 12 * (ii - 1) + 1, sm, sqds);
        }
      }
      break;

     case 3:
      {
        double b;
        double scale;
        nrm = b_s[m + 1];
        scale = fmax(fmax(fmax(fmax(fabs(nrm), fabs(b_s[m])), fabs(e[m])), fabs
                          (b_s[ii])), fabs(e[ii]));
        sm = nrm / scale;
        nrm = b_s[m] / scale;
        rt = e[m] / scale;
        sqds = b_s[ii] / scale;
        b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          rt = sqrt(b * b + nrm);
          if (b < 0.0) {
            rt = -rt;
          }

          rt = nrm / (b + rt);
        } else {
          rt = 0.0;
        }

        rt += (sqds + sm) * (sqds - sm);
        nrm = sqds * (e[ii] / scale);
        for (k = ii + 1; k <= jj; k++) {
          xrotg(&rt, &nrm, &sm, &sqds);
          if (k > ii + 1) {
            e[k - 2] = rt;
          }

          nrm = e[k - 1];
          b = b_s[k - 1];
          e[k - 1] = sm * nrm - sqds * b;
          rt = sqds * b_s[k];
          b_s[k] *= sm;
          xrot(Vf, 6 * (k - 1) + 1, 6 * k + 1, sm, sqds);
          b_s[k - 1] = sm * b + sqds * nrm;
          xrotg(&b_s[k - 1], &rt, &sm, &sqds);
          rt = sm * e[k - 1] + sqds * b_s[k];
          b_s[k] = -sqds * e[k - 1] + sm * b_s[k];
          nrm = sqds * e[k];
          e[k] *= sm;
          b_xrot(U, 12 * (k - 1) + 1, 12 * k + 1, sm, sqds);
        }

        e[m] = rt;
        qq++;
      }
      break;

     default:
      if (b_s[ii] < 0.0) {
        b_s[ii] = -b_s[ii];
        i = 6 * ii;
        qp1jj = i + 6;
        for (k = i + 1; k <= qp1jj; k++) {
          Vf[k - 1] = -Vf[k - 1];
        }
      }

      qp1 = ii + 1;
      while ((ii + 1 < 6) && (b_s[ii] < b_s[qp1])) {
        rt = b_s[ii];
        b_s[ii] = b_s[qp1];
        b_s[qp1] = rt;
        xswap(Vf, 6 * ii + 1, 6 * (ii + 1) + 1);
        b_xswap(U, 12 * ii + 1, 12 * (ii + 1) + 1);
        ii = qp1;
        qp1++;
      }

      qq = 0;
      m--;
      break;
    }
  }

  for (k = 0; k < 6; k++) {
    s[k] = b_s[k];
    for (i = 0; i < 6; i++) {
      qp1jj = i + 6 * k;
      V[qp1jj] = Vf[qp1jj];
    }
  }
}

/*
 * Arguments    : b_struct_T *MeritFunction
 *                const k_struct_T *WorkingSet
 *                i_struct_T *TrialState
 *                const double lb[15]
 *                const double ub[15]
 *                bool *Flags_gradOK
 *                bool *Flags_fevalOK
 *                bool *Flags_done
 *                bool *Flags_stepAccepted
 *                bool *Flags_failedLineSearch
 *                int *Flags_stepType
 * Return Type  : void
 */
static void test_exit(b_struct_T *MeritFunction, const k_struct_T *WorkingSet,
                      i_struct_T *TrialState, const double lb[15], const double
                      ub[15], bool *Flags_gradOK, bool *Flags_fevalOK, bool
                      *Flags_done, bool *Flags_stepAccepted, bool
                      *Flags_failedLineSearch, int *Flags_stepType)
{
  double s;
  double smax;
  int idx_max;
  int k;
  int mLB;
  int mLambda;
  int mUB;
  int nVar;
  bool exitg1;
  bool isFeasible;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda = ((WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4])
    - 1;
  if (mLambda >= 0) {
    memcpy(&TrialState->lambdaStopTest[0], &TrialState->lambdasqp[0], (mLambda +
            1) * sizeof(double));
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

  s = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (rtIsInf(s)) {
    s = 1.0;
  }

  smax = 0.0;
  for (idx_max = 0; idx_max < mLB; idx_max++) {
    nVar = WorkingSet->indexLB[idx_max] - 1;
    smax = fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }

  for (idx_max = 0; idx_max < mUB; idx_max++) {
    nVar = WorkingSet->indexUB[idx_max] - 1;
    smax = fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }

  MeritFunction->nlpPrimalFeasError = smax;
  MeritFunction->feasRelativeFactor = fmax(1.0, smax);
  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  *Flags_gradOK = true;
  smax = 0.0;
  idx_max = 0;
  exitg1 = false;
  while ((!exitg1) && (idx_max <= WorkingSet->nVar - 1)) {
    *Flags_gradOK = ((!rtIsInf(TrialState->gradLag[idx_max])) && (!rtIsNaN
      (TrialState->gradLag[idx_max])));
    if (!*Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag[idx_max]));
      idx_max++;
    }
  }

  MeritFunction->nlpDualFeasError = smax;
  if (!*Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = smax;
    if (mLambda >= 0) {
      memcpy(&TrialState->lambdaStopTestPrev[0], &TrialState->lambdaStopTest[0],
             (mLambda + 1) * sizeof(double));
    }

    if (isFeasible && (smax <= 1.0E-9 * s)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }
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
    if (!freq_not_empty) {
      freq_not_empty = true;
      coderInitTimeFunctions(&freq);
    }

    coderTimeClockGettimeMonotonic(&savedTime, freq);
    savedTime_not_empty = true;
  }

  savedTime.tv_sec = newTime_tv_sec;
  savedTime.tv_nsec = newTime_tv_nsec;
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
  b_timeKeeper(&tstart_tv_sec, &tstart_tv_nsec);
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  return (b_timespec.tv_sec - tstart_tv_sec) + (b_timespec.tv_nsec -
    tstart_tv_nsec) / 1.0E+9;
}

/*
 * Arguments    : int n
 *                double a
 *                int ix0
 *                double y[72]
 *                int iy0
 * Return Type  : void
 */
static void xaxpy(int n, double a, int ix0, double y[72], int iy0)
{
  int k;
  if (!(a == 0.0)) {
    int i;
    i = n - 1;
    for (k = 0; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
    }
  }
}

/*
 * Arguments    : int n
 *                const double x[72]
 *                int ix0
 *                const double y[72]
 *                int iy0
 * Return Type  : double
 */
static double xdotc(int n, const double x[72], int ix0, const double y[72], int
                    iy0)
{
  double d;
  int k;
  d = 0.0;
  for (k = 0; k < n; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
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
        memset(&C[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
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
 *                const double A[961]
 *                const double x[16]
 *                double y[496]
 * Return Type  : void
 */
static void xgemv(int m, int n, const double A[961], const double x[16], double
                  y[496])
{
  int ia;
  int iac;
  if (m != 0) {
    int i;
    memset(&y[0], 0, n * sizeof(double));
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
            A[i1] += -c;
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
 * Arguments    : double x[36]
 *                int ix0
 *                int iy0
 *                double c
 *                double s
 * Return Type  : void
 */
static void xrot(double x[36], int ix0, int iy0, double c, double s)
{
  int k;
  for (k = 0; k < 6; k++) {
    double b_temp_tmp;
    double d_temp_tmp;
    int c_temp_tmp;
    int temp_tmp;
    temp_tmp = (iy0 + k) - 1;
    b_temp_tmp = x[temp_tmp];
    c_temp_tmp = (ix0 + k) - 1;
    d_temp_tmp = x[c_temp_tmp];
    x[temp_tmp] = c * b_temp_tmp - s * d_temp_tmp;
    x[c_temp_tmp] = c * d_temp_tmp + s * b_temp_tmp;
  }
}

/*
 * Arguments    : double *a
 *                double *b
 *                double *c
 *                double *s
 * Return Type  : void
 */
static void xrotg(double *a, double *b, double *c, double *s)
{
  double absa;
  double absb;
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
    *c = 1.0;
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

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }
}

/*
 * Arguments    : double x[36]
 *                int ix0
 *                int iy0
 * Return Type  : void
 */
static void xswap(double x[36], int ix0, int iy0)
{
  int k;
  for (k = 0; k < 6; k++) {
    double temp;
    int i;
    int temp_tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    i = (iy0 + k) - 1;
    x[temp_tmp] = x[i];
    x[i] = temp;
  }
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
        memset(&work[0], 0, (lastc + 1) * sizeof(double));
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

    if (!(-tau == 0.0)) {
      i = ic0;
      for (iac = 0; iac <= lastc; iac++) {
        if (work[iac] != 0.0) {
          c = work[iac] * -tau;
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
      beta1 = rt_hypotd_snf(*alpha1, xnorm);
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

        beta1 = rt_hypotd_snf(*alpha1, xnrm2(n - 1, x, ix0));
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
 * NONLINEAR CA controller
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
 *                double Psi
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
 *                double dv[6]
 *                double p
 *                double q
 *                double r
 *                double Cm_zero
 *                double Cl_alpha
 *                double Cd_zero
 *                double K_Cd
 *                double Cm_alpha
 *                double rho
 *                double V
 *                double S
 *                double wing_chord
 *                double flight_path_angle
 *                double max_alpha
 *                double min_alpha
 *                double Beta
 *                double gamma_quadratic_du
 *                double gamma_quadratic_wls
 *                double desired_motor_value
 *                double desired_el_value
 *                double desired_az_value
 *                double desired_phi_value
 *                double desired_theta_value
 *                double controller_id
 *                double verbose
 *                double u_out[14]
 *                double residuals[6]
 *                double *elapsed_time
 *                double *N_iterations
 *                double *N_evaluation
 *                double *exitflag
 * Return Type  : void
 */
void Global_controller_fcn_earth_rf_journal(double K_p_T, double K_p_M, double m,
  double I_xx, double I_yy, double I_zz, double l_1, double l_2, double l_3,
  double l_4, double l_z, double Phi, double Theta, double Psi, double Omega_1,
  double Omega_2, double Omega_3, double Omega_4, double b_1, double b_2, double
  b_3, double b_4, double g_1, double g_2, double g_3, double g_4, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_act_theta_const, double W_act_theta_speed,
  double W_act_phi_const, double W_act_phi_speed, double W_dv_1, double W_dv_2,
  double W_dv_3, double W_dv_4, double W_dv_5, double W_dv_6, double max_omega,
  double min_omega, double max_b, double min_b, double max_g, double min_g,
  double max_theta, double min_theta, double max_phi, double dv[6], double p,
  double q, double r, double Cm_zero, double Cl_alpha, double Cd_zero, double
  K_Cd, double Cm_alpha, double rho, double V, double S, double wing_chord,
  double flight_path_angle, double max_alpha, double min_alpha, double Beta,
  double gamma_quadratic_du, double gamma_quadratic_wls, double
  desired_motor_value, double desired_el_value, double desired_az_value, double
  desired_phi_value, double desired_theta_value, double controller_id, double
  verbose, double u_out[14], double residuals[6], double *elapsed_time, double
  *N_iterations, double *N_evaluation, double *exitflag)
{
  double u_out_local_data[210];
  double B_eval[72];
  double u_out_local[15];
  double u_out_scaled[14];
  double actual_u[12];
  double desired_u[12];
  double varargin_2[12];
  double absx;
  double absxk;
  double gain_az;
  double gain_el;
  double gain_motor;
  double t;
  double tol;
  int ar;
  int b_i;
  int br;
  int i;
  int i1;
  int i2;
  int ib;
  int ic;
  if (!isInitialized_Global_controller_fcn_earth_rf_journal) {
    Global_controller_fcn_earth_rf_journal_initialize();
  }

  if (controller_id == 1.0) {
    double R_gc_tmp[9];
    double y[3];
    tol = sin(Psi);
    absx = cos(Psi);
    R_gc_tmp[0] = absx;
    R_gc_tmp[3] = tol;
    R_gc_tmp[6] = 0.0;
    R_gc_tmp[1] = -tol;
    R_gc_tmp[4] = absx;
    R_gc_tmp[7] = 0.0;
    R_gc_tmp[2] = 0.0;
    R_gc_tmp[5] = 0.0;
    R_gc_tmp[8] = 1.0;
    absxk = dv[0];
    tol = dv[1];
    absx = dv[2];
    for (i1 = 0; i1 < 3; i1++) {
      y[i1] = (R_gc_tmp[i1] * absxk + R_gc_tmp[i1 + 3] * tol) + R_gc_tmp[i1 + 6]
        * absx;
    }

    dv[0] = y[0];
    dv[1] = y[1];
    dv[2] = y[2];
    c_Nonlinear_controller_fcn_cont(K_p_T, K_p_M, m, I_xx, I_yy, I_zz, l_1, l_2,
      l_3, l_4, l_z, Phi, Theta, Omega_1, Omega_2, Omega_3, Omega_4, b_1, b_2,
      b_3, b_4, g_1, g_2, g_3, g_4, W_act_motor_const, W_act_motor_speed,
      W_act_tilt_el_const, W_act_tilt_el_speed, W_act_tilt_az_const,
      W_act_tilt_az_speed, W_act_theta_const, W_act_theta_speed, W_act_phi_const,
      W_act_phi_speed, W_dv_1, W_dv_2, W_dv_3, W_dv_4, W_dv_5, W_dv_6, max_omega,
      min_omega, max_b, min_b, max_g, min_g, max_theta, min_theta, max_phi, dv,
      p, q, r, Cm_zero, Cl_alpha, Cd_zero, K_Cd, Cm_alpha, rho, V, S, wing_chord,
      flight_path_angle, max_alpha, min_alpha, Beta, gamma_quadratic_du,
      desired_motor_value, desired_el_value, desired_az_value,
      desired_theta_value * 3.1415926535897931 / 180.0, desired_phi_value *
      3.1415926535897931 / 180.0, verbose, u_out_local, residuals, elapsed_time,
      N_iterations, N_evaluation, exitflag);
    memcpy(&u_out_local_data[0], &u_out_local[0], 14U * sizeof(double));

    /* WLS CA controller */
  } else if (controller_id == 2.0) {
    double b_dv[6];
    for (i = 0; i < 6; i++) {
      b_dv[i] = dv[i];
    }

    c_WLS_controller_fcn_earth_rf_j(K_p_T, K_p_M, m, I_xx, I_yy, I_zz, l_1, l_2,
      l_3, l_4, l_z, Phi, Theta, Psi, Omega_1, Omega_2, Omega_3, Omega_4, b_1,
      b_2, b_3, b_4, g_1, g_2, g_3, g_4, W_act_motor_const, W_act_motor_speed,
      W_act_tilt_el_const, W_act_tilt_el_speed, W_act_tilt_az_const,
      W_act_tilt_az_speed, W_dv_1, W_dv_2, W_dv_3, W_dv_4, W_dv_5, W_dv_6,
      max_omega, min_omega, max_b, min_b, max_g, min_g, b_dv, p, q, r, Cm_zero,
      Cl_alpha, Cd_zero, K_Cd, Cm_alpha, rho, V, S, wing_chord,
      flight_path_angle, max_alpha, Beta, gamma_quadratic_wls,
      desired_motor_value, desired_el_value, desired_az_value, desired_phi_value,
      desired_theta_value, u_out_scaled, residuals, elapsed_time, N_iterations,
      N_evaluation, exitflag);
    memcpy(&u_out_local_data[0], &u_out_scaled[0], 14U * sizeof(double));

    /* Max and min alpha are the new constraint for the tilt angles.  */
    /* PINV controller */
  } else if (controller_id == 3.0) {
    double A[72];
    double u_out_feasible[12];
    double u_scaled[12];
    double x[12];
    double b_dv[6];
    double B_eval_tmp;
    double B_eval_tmp_tmp;
    double ab_B_eval_tmp;
    double b_B_eval_tmp;
    double b_B_eval_tmp_tmp;
    double bb_B_eval_tmp;
    double c_B_eval_tmp;
    double c_B_eval_tmp_tmp;
    double cb_B_eval_tmp;
    double d_B_eval_tmp;
    double d_B_eval_tmp_tmp;
    double db_B_eval_tmp;
    double e_B_eval_tmp;
    double e_B_eval_tmp_tmp;
    double eb_B_eval_tmp;
    double f_B_eval_tmp;
    double f_B_eval_tmp_tmp;
    double fb_B_eval_tmp;
    double g_B_eval_tmp;
    double g_B_eval_tmp_tmp;
    double gb_B_eval_tmp;
    double h_B_eval_tmp;
    double hb_B_eval_tmp;
    double i_B_eval_tmp;
    double ib_B_eval_tmp;
    double j_B_eval_tmp;
    double jb_B_eval_tmp;
    double k_B_eval_tmp;
    double kb_B_eval_tmp;
    double l_B_eval_tmp;
    double lb_B_eval_tmp;
    double m_B_eval_tmp;
    double mb_B_eval_tmp;
    double n_B_eval_tmp;
    double nb_B_eval_tmp;
    double o_B_eval_tmp;
    double ob_B_eval_tmp;
    double p_B_eval_tmp;
    double pb_B_eval_tmp;
    double q_B_eval_tmp;
    double qb_B_eval_tmp;
    double r_B_eval_tmp;
    double rb_B_eval_tmp;
    double s_B_eval_tmp;
    double sb_B_eval_tmp;
    double t_B_eval_tmp;
    double tb_B_eval_tmp;
    double u_B_eval_tmp;
    double ub_B_eval_tmp;
    double v_B_eval_tmp;
    double vb_B_eval_tmp;
    double w_B_eval_tmp;
    double wb_B_eval_tmp;
    double x_B_eval_tmp;
    double y_B_eval_tmp;
    bool b_x[12];
    bool b_p;
    if (desired_motor_value < 120.0) {
      desired_motor_value = (((Omega_1 + Omega_2) + Omega_3) + Omega_4) / 4.0;
    }

    /*  Testing parameters: */
    /*  (9e-06, 1.31e-07, 2.35, 0.15, 0.13, 0.2, 0.231, 0.231, 0.39, 0.39, 0, ... */
    /*  0, 0, 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0,... */
    /*  1, 1, 1, 0,... */
    /*  1, 100, 400, -50, 500, -60,... */
    /*  0.01, 0.01, 0.01, 0.1, 0.1, 0.01,... */
    /*  900, 0, 25, -130, 45, -45, 100, -30, 40, [0 0 -5 0 0 0]', 0, 0, 0, 0.1, 5.18, 0.38, 0.2, ... */
    /*  -.1, 1.225, 0, 0.55, 0.33, 0, 15, -15, 0, 1e-07,... */
    /*  0, 0, 0, 0, 0)   */
    tic();

    /*  Assign geometrical and create variables */
    gain_motor = max_omega / 2.0;
    gain_el = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
    gain_az = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;

    /*  Identify and evaluate the effectiveness matrix [6,12] */
    B_eval_tmp = cos(Psi);
    B_eval_tmp_tmp = cos(Phi);
    b_B_eval_tmp = sin(Psi);
    b_B_eval_tmp_tmp = sin(Phi);
    absx = sin(Theta);
    c_B_eval_tmp = cos(Theta);
    d_B_eval_tmp = cos(g_1);
    e_B_eval_tmp = sin(b_1);
    f_B_eval_tmp = cos(b_1);
    g_B_eval_tmp = sin(g_1);
    h_B_eval_tmp = cos(g_2);
    i_B_eval_tmp = sin(b_2);
    j_B_eval_tmp = cos(b_2);
    k_B_eval_tmp = sin(g_2);
    l_B_eval_tmp = cos(g_3);
    m_B_eval_tmp = sin(b_3);
    n_B_eval_tmp = cos(b_3);
    o_B_eval_tmp = sin(g_3);
    p_B_eval_tmp = cos(g_4);
    q_B_eval_tmp = sin(b_4);
    r_B_eval_tmp = cos(b_4);
    s_B_eval_tmp = sin(g_4);
    t_B_eval_tmp = 2.0 * K_p_T * Omega_1;
    B_eval[0] = -((t_B_eval_tmp * B_eval_tmp * c_B_eval_tmp * e_B_eval_tmp +
                   t_B_eval_tmp * f_B_eval_tmp * d_B_eval_tmp *
                   (b_B_eval_tmp_tmp * b_B_eval_tmp + B_eval_tmp_tmp *
                    B_eval_tmp * absx)) + 2.0 * K_p_T * Omega_1 * cos(b_1) *
                  g_B_eval_tmp * (B_eval_tmp_tmp * b_B_eval_tmp - B_eval_tmp *
      b_B_eval_tmp_tmp * absx)) / m;
    u_B_eval_tmp = 2.0 * K_p_T * Omega_2;
    tol = b_B_eval_tmp_tmp * sin(Psi);
    absxk = B_eval_tmp_tmp * cos(Psi);
    v_B_eval_tmp = tol + absxk * absx;
    t = B_eval_tmp_tmp * sin(Psi);
    c_B_eval_tmp_tmp = cos(Psi) * b_B_eval_tmp_tmp;
    w_B_eval_tmp = t - c_B_eval_tmp_tmp * absx;
    B_eval[6] = -((u_B_eval_tmp * B_eval_tmp * c_B_eval_tmp * i_B_eval_tmp +
                   u_B_eval_tmp * j_B_eval_tmp * h_B_eval_tmp * v_B_eval_tmp) +
                  2.0 * K_p_T * Omega_2 * cos(b_2) * k_B_eval_tmp * w_B_eval_tmp)
      / m;
    x_B_eval_tmp = 2.0 * K_p_T * Omega_3;
    B_eval[12] = -((x_B_eval_tmp * B_eval_tmp * c_B_eval_tmp * m_B_eval_tmp +
                    x_B_eval_tmp * n_B_eval_tmp * l_B_eval_tmp * v_B_eval_tmp) +
                   2.0 * K_p_T * Omega_3 * cos(b_3) * o_B_eval_tmp *
                   w_B_eval_tmp) / m;
    y_B_eval_tmp = 2.0 * K_p_T * Omega_4;
    B_eval[18] = -((y_B_eval_tmp * B_eval_tmp * c_B_eval_tmp * q_B_eval_tmp +
                    y_B_eval_tmp * r_B_eval_tmp * p_B_eval_tmp * v_B_eval_tmp) +
                   2.0 * K_p_T * Omega_4 * cos(b_4) * s_B_eval_tmp *
                   w_B_eval_tmp) / m;
    d_B_eval_tmp_tmp = Omega_1 * Omega_1;
    ab_B_eval_tmp = K_p_T * d_B_eval_tmp_tmp;
    bb_B_eval_tmp = ab_B_eval_tmp * d_B_eval_tmp * e_B_eval_tmp;
    cb_B_eval_tmp = ab_B_eval_tmp * e_B_eval_tmp * g_B_eval_tmp;
    B_eval[24] = ((bb_B_eval_tmp * v_B_eval_tmp - ab_B_eval_tmp * B_eval_tmp *
                   c_B_eval_tmp * f_B_eval_tmp) + cb_B_eval_tmp * w_B_eval_tmp) /
      m;
    e_B_eval_tmp_tmp = Omega_2 * Omega_2;
    db_B_eval_tmp = K_p_T * e_B_eval_tmp_tmp;
    eb_B_eval_tmp = db_B_eval_tmp * h_B_eval_tmp * i_B_eval_tmp;
    fb_B_eval_tmp = db_B_eval_tmp * i_B_eval_tmp * k_B_eval_tmp;
    B_eval[30] = ((eb_B_eval_tmp * v_B_eval_tmp - db_B_eval_tmp * B_eval_tmp *
                   c_B_eval_tmp * j_B_eval_tmp) + fb_B_eval_tmp * w_B_eval_tmp) /
      m;
    f_B_eval_tmp_tmp = Omega_3 * Omega_3;
    gb_B_eval_tmp = K_p_T * f_B_eval_tmp_tmp;
    hb_B_eval_tmp = gb_B_eval_tmp * l_B_eval_tmp * m_B_eval_tmp;
    ib_B_eval_tmp = gb_B_eval_tmp * m_B_eval_tmp * o_B_eval_tmp;
    B_eval[36] = ((hb_B_eval_tmp * v_B_eval_tmp - gb_B_eval_tmp * B_eval_tmp *
                   c_B_eval_tmp * n_B_eval_tmp) + ib_B_eval_tmp * w_B_eval_tmp) /
      m;
    g_B_eval_tmp_tmp = Omega_4 * Omega_4;
    jb_B_eval_tmp = K_p_T * g_B_eval_tmp_tmp;
    kb_B_eval_tmp = jb_B_eval_tmp * p_B_eval_tmp * q_B_eval_tmp;
    lb_B_eval_tmp = jb_B_eval_tmp * q_B_eval_tmp * s_B_eval_tmp;
    B_eval[42] = ((kb_B_eval_tmp * v_B_eval_tmp - jb_B_eval_tmp * B_eval_tmp *
                   c_B_eval_tmp * r_B_eval_tmp) + lb_B_eval_tmp * w_B_eval_tmp) /
      m;
    B_eval_tmp = ab_B_eval_tmp * f_B_eval_tmp;
    mb_B_eval_tmp = B_eval_tmp * d_B_eval_tmp;
    nb_B_eval_tmp = B_eval_tmp * g_B_eval_tmp;
    B_eval[48] = -(mb_B_eval_tmp * w_B_eval_tmp - nb_B_eval_tmp * v_B_eval_tmp) /
      m;
    ob_B_eval_tmp = db_B_eval_tmp * j_B_eval_tmp;
    pb_B_eval_tmp = ob_B_eval_tmp * h_B_eval_tmp;
    qb_B_eval_tmp = ob_B_eval_tmp * k_B_eval_tmp;
    B_eval[54] = -(pb_B_eval_tmp * w_B_eval_tmp - qb_B_eval_tmp * v_B_eval_tmp) /
      m;
    rb_B_eval_tmp = gb_B_eval_tmp * n_B_eval_tmp;
    sb_B_eval_tmp = rb_B_eval_tmp * l_B_eval_tmp;
    tb_B_eval_tmp = rb_B_eval_tmp * o_B_eval_tmp;
    B_eval[60] = -(sb_B_eval_tmp * w_B_eval_tmp - tb_B_eval_tmp * v_B_eval_tmp) /
      m;
    ub_B_eval_tmp = jb_B_eval_tmp * r_B_eval_tmp;
    vb_B_eval_tmp = ub_B_eval_tmp * p_B_eval_tmp;
    wb_B_eval_tmp = ub_B_eval_tmp * s_B_eval_tmp;
    B_eval[66] = -(vb_B_eval_tmp * w_B_eval_tmp - wb_B_eval_tmp * v_B_eval_tmp) /
      m;
    B_eval[1] = ((2.0 * K_p_T * Omega_1 * cos(b_1) * cos(g_1) *
                  (c_B_eval_tmp_tmp - t * absx) - t_B_eval_tmp * c_B_eval_tmp *
                  b_B_eval_tmp * e_B_eval_tmp) + 2.0 * K_p_T * Omega_1 * cos(b_1)
                 * sin(g_1) * (absxk + tol * absx)) / m;
    v_B_eval_tmp = cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta);
    w_B_eval_tmp = cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta);
    B_eval[7] = ((2.0 * K_p_T * Omega_2 * cos(b_2) * cos(g_2) * v_B_eval_tmp -
                  u_B_eval_tmp * c_B_eval_tmp * b_B_eval_tmp * i_B_eval_tmp) +
                 2.0 * K_p_T * Omega_2 * cos(b_2) * sin(g_2) * w_B_eval_tmp) / m;
    B_eval[13] = ((2.0 * K_p_T * Omega_3 * cos(b_3) * cos(g_3) * v_B_eval_tmp -
                   x_B_eval_tmp * c_B_eval_tmp * b_B_eval_tmp * m_B_eval_tmp) +
                  2.0 * K_p_T * Omega_3 * cos(b_3) * sin(g_3) * w_B_eval_tmp) /
      m;
    B_eval[19] = ((2.0 * K_p_T * Omega_4 * cos(b_4) * cos(g_4) * v_B_eval_tmp -
                   y_B_eval_tmp * c_B_eval_tmp * b_B_eval_tmp * q_B_eval_tmp) +
                  2.0 * K_p_T * Omega_4 * cos(b_4) * sin(g_4) * w_B_eval_tmp) /
      m;
    t = ab_B_eval_tmp * c_B_eval_tmp;
    B_eval[25] = -((t * b_B_eval_tmp * f_B_eval_tmp + bb_B_eval_tmp *
                    v_B_eval_tmp) + cb_B_eval_tmp * w_B_eval_tmp) / m;
    bb_B_eval_tmp = db_B_eval_tmp * c_B_eval_tmp;
    B_eval[31] = -((bb_B_eval_tmp * b_B_eval_tmp * j_B_eval_tmp + eb_B_eval_tmp *
                    v_B_eval_tmp) + fb_B_eval_tmp * w_B_eval_tmp) / m;
    cb_B_eval_tmp = gb_B_eval_tmp * c_B_eval_tmp;
    B_eval[37] = -((cb_B_eval_tmp * b_B_eval_tmp * n_B_eval_tmp + hb_B_eval_tmp *
                    v_B_eval_tmp) + ib_B_eval_tmp * w_B_eval_tmp) / m;
    eb_B_eval_tmp = jb_B_eval_tmp * c_B_eval_tmp;
    B_eval[43] = -((eb_B_eval_tmp * b_B_eval_tmp * r_B_eval_tmp + kb_B_eval_tmp *
                    v_B_eval_tmp) + lb_B_eval_tmp * w_B_eval_tmp) / m;
    B_eval[49] = (mb_B_eval_tmp * w_B_eval_tmp - nb_B_eval_tmp * v_B_eval_tmp) /
      m;
    B_eval[55] = (pb_B_eval_tmp * w_B_eval_tmp - qb_B_eval_tmp * v_B_eval_tmp) /
      m;
    B_eval[61] = (sb_B_eval_tmp * w_B_eval_tmp - tb_B_eval_tmp * v_B_eval_tmp) /
      m;
    B_eval[67] = (vb_B_eval_tmp * w_B_eval_tmp - wb_B_eval_tmp * v_B_eval_tmp) /
      m;
    B_eval[2] = ((t_B_eval_tmp * e_B_eval_tmp * v_B_eval_tmp + 2.0 * K_p_T
                  * Omega_1 * cos(Theta) * b_B_eval_tmp_tmp * f_B_eval_tmp *
                  g_B_eval_tmp) - t_B_eval_tmp * B_eval_tmp_tmp * c_B_eval_tmp *
                 f_B_eval_tmp * d_B_eval_tmp) / m;
    B_eval[8] = ((u_B_eval_tmp * i_B_eval_tmp * v_B_eval_tmp + 2.0 * K_p_T
                  * Omega_2 * cos(Theta) * b_B_eval_tmp_tmp * j_B_eval_tmp *
                  k_B_eval_tmp) - u_B_eval_tmp * B_eval_tmp_tmp * c_B_eval_tmp *
                 j_B_eval_tmp * h_B_eval_tmp) / m;
    B_eval[14] = ((x_B_eval_tmp * m_B_eval_tmp * v_B_eval_tmp + 2.0 * K_p_T
                   * Omega_3 * cos(Theta) * b_B_eval_tmp_tmp * n_B_eval_tmp *
                   o_B_eval_tmp) - x_B_eval_tmp * B_eval_tmp_tmp * c_B_eval_tmp *
                  n_B_eval_tmp * l_B_eval_tmp) / m;
    B_eval[20] = ((y_B_eval_tmp * q_B_eval_tmp * v_B_eval_tmp + 2.0 * K_p_T
                   * Omega_4 * cos(Theta) * b_B_eval_tmp_tmp * r_B_eval_tmp *
                   s_B_eval_tmp) - y_B_eval_tmp * B_eval_tmp_tmp * c_B_eval_tmp *
                  r_B_eval_tmp * p_B_eval_tmp) / m;
    b_B_eval_tmp = ab_B_eval_tmp * B_eval_tmp_tmp * c_B_eval_tmp;
    w_B_eval_tmp = t * b_B_eval_tmp_tmp;
    B_eval[26] = ((B_eval_tmp * v_B_eval_tmp + b_B_eval_tmp * d_B_eval_tmp *
                   e_B_eval_tmp) - w_B_eval_tmp * e_B_eval_tmp * g_B_eval_tmp) /
      m;
    B_eval_tmp = db_B_eval_tmp * B_eval_tmp_tmp * c_B_eval_tmp;
    bb_B_eval_tmp *= b_B_eval_tmp_tmp;
    B_eval[32] = ((ob_B_eval_tmp * v_B_eval_tmp + B_eval_tmp * h_B_eval_tmp *
                   i_B_eval_tmp) - bb_B_eval_tmp * i_B_eval_tmp * k_B_eval_tmp) /
      m;
    fb_B_eval_tmp = gb_B_eval_tmp * B_eval_tmp_tmp * c_B_eval_tmp;
    cb_B_eval_tmp *= b_B_eval_tmp_tmp;
    B_eval[38] = ((rb_B_eval_tmp * v_B_eval_tmp + fb_B_eval_tmp * l_B_eval_tmp *
                   m_B_eval_tmp) - cb_B_eval_tmp * m_B_eval_tmp * o_B_eval_tmp) /
      m;
    c_B_eval_tmp *= jb_B_eval_tmp * B_eval_tmp_tmp;
    eb_B_eval_tmp *= b_B_eval_tmp_tmp;
    B_eval[44] = ((ub_B_eval_tmp * v_B_eval_tmp + c_B_eval_tmp * p_B_eval_tmp *
                   q_B_eval_tmp) - eb_B_eval_tmp * q_B_eval_tmp * s_B_eval_tmp) /
      m;
    B_eval[50] = (b_B_eval_tmp * f_B_eval_tmp * g_B_eval_tmp + w_B_eval_tmp *
                  f_B_eval_tmp * d_B_eval_tmp) / m;
    B_eval[56] = (B_eval_tmp * j_B_eval_tmp * k_B_eval_tmp + bb_B_eval_tmp *
                  j_B_eval_tmp * h_B_eval_tmp) / m;
    B_eval[62] = (fb_B_eval_tmp * n_B_eval_tmp * o_B_eval_tmp + cb_B_eval_tmp *
                  n_B_eval_tmp * l_B_eval_tmp) / m;
    B_eval[68] = (c_B_eval_tmp * r_B_eval_tmp * s_B_eval_tmp + eb_B_eval_tmp *
                  r_B_eval_tmp * p_B_eval_tmp) / m;
    B_eval_tmp = t_B_eval_tmp * l_z;
    b_B_eval_tmp = 2.0 * K_p_M * Omega_1;
    c_B_eval_tmp = t_B_eval_tmp * l_1;
    B_eval[3] = ((b_B_eval_tmp * e_B_eval_tmp + B_eval_tmp * f_B_eval_tmp *
                  g_B_eval_tmp) + c_B_eval_tmp * f_B_eval_tmp * d_B_eval_tmp) /
      I_xx;
    v_B_eval_tmp = u_B_eval_tmp * l_z;
    w_B_eval_tmp = 2.0 * K_p_M * Omega_2;
    bb_B_eval_tmp = u_B_eval_tmp * l_1;
    B_eval[9] = -((w_B_eval_tmp * i_B_eval_tmp - v_B_eval_tmp * j_B_eval_tmp *
                   k_B_eval_tmp) + bb_B_eval_tmp * j_B_eval_tmp * h_B_eval_tmp) /
      I_xx;
    cb_B_eval_tmp = 2.0 * K_p_M * Omega_3;
    eb_B_eval_tmp = x_B_eval_tmp * l_z;
    fb_B_eval_tmp = x_B_eval_tmp * l_2;
    B_eval[15] = ((cb_B_eval_tmp * m_B_eval_tmp + eb_B_eval_tmp * n_B_eval_tmp *
                   o_B_eval_tmp) - fb_B_eval_tmp * n_B_eval_tmp * l_B_eval_tmp) /
      I_xx;
    hb_B_eval_tmp = y_B_eval_tmp * l_z;
    ib_B_eval_tmp = 2.0 * K_p_M * Omega_4;
    kb_B_eval_tmp = y_B_eval_tmp * l_2;
    B_eval[21] = ((hb_B_eval_tmp * r_B_eval_tmp * s_B_eval_tmp - ib_B_eval_tmp *
                   q_B_eval_tmp) + kb_B_eval_tmp * r_B_eval_tmp * p_B_eval_tmp) /
      I_xx;
    lb_B_eval_tmp = ab_B_eval_tmp * l_z;
    mb_B_eval_tmp = ab_B_eval_tmp * l_1;
    nb_B_eval_tmp = K_p_M * d_B_eval_tmp_tmp;
    ob_B_eval_tmp = nb_B_eval_tmp * f_B_eval_tmp;
    B_eval[27] = -((mb_B_eval_tmp * d_B_eval_tmp * e_B_eval_tmp - ob_B_eval_tmp)
                   + lb_B_eval_tmp * e_B_eval_tmp * g_B_eval_tmp) / I_xx;
    pb_B_eval_tmp = db_B_eval_tmp * l_z;
    qb_B_eval_tmp = db_B_eval_tmp * l_1;
    rb_B_eval_tmp = K_p_M * e_B_eval_tmp_tmp;
    sb_B_eval_tmp = rb_B_eval_tmp * j_B_eval_tmp;
    B_eval[33] = -((sb_B_eval_tmp - qb_B_eval_tmp * h_B_eval_tmp * i_B_eval_tmp)
                   + pb_B_eval_tmp * i_B_eval_tmp * k_B_eval_tmp) / I_xx;
    tb_B_eval_tmp = gb_B_eval_tmp * l_z;
    ub_B_eval_tmp = gb_B_eval_tmp * l_2;
    vb_B_eval_tmp = K_p_M * f_B_eval_tmp_tmp;
    wb_B_eval_tmp = vb_B_eval_tmp * n_B_eval_tmp;
    B_eval[39] = ((wb_B_eval_tmp + ub_B_eval_tmp * l_B_eval_tmp * m_B_eval_tmp)
                  - tb_B_eval_tmp * m_B_eval_tmp * o_B_eval_tmp) / I_xx;
    t = jb_B_eval_tmp * l_z;
    absxk = jb_B_eval_tmp * l_2;
    absx = K_p_M * g_B_eval_tmp_tmp;
    tol = absx * r_B_eval_tmp;
    B_eval[45] = -((tol + absxk * p_B_eval_tmp * q_B_eval_tmp) + t *
                   q_B_eval_tmp * s_B_eval_tmp) / I_xx;
    lb_B_eval_tmp *= f_B_eval_tmp;
    mb_B_eval_tmp *= f_B_eval_tmp;
    B_eval[51] = (lb_B_eval_tmp * d_B_eval_tmp - mb_B_eval_tmp * g_B_eval_tmp) /
      I_xx;
    pb_B_eval_tmp *= j_B_eval_tmp;
    qb_B_eval_tmp *= j_B_eval_tmp;
    B_eval[57] = (pb_B_eval_tmp * h_B_eval_tmp + qb_B_eval_tmp * k_B_eval_tmp) /
      I_xx;
    tb_B_eval_tmp *= n_B_eval_tmp;
    ub_B_eval_tmp *= n_B_eval_tmp;
    B_eval[63] = (tb_B_eval_tmp * l_B_eval_tmp + ub_B_eval_tmp * o_B_eval_tmp) /
      I_xx;
    t *= r_B_eval_tmp;
    absxk *= r_B_eval_tmp;
    B_eval[69] = (t * p_B_eval_tmp - absxk * s_B_eval_tmp) / I_xx;
    B_eval[4] = ((B_eval_tmp * e_B_eval_tmp - b_B_eval_tmp * f_B_eval_tmp *
                  g_B_eval_tmp) + t_B_eval_tmp * l_4 * f_B_eval_tmp *
                 d_B_eval_tmp) / I_yy;
    B_eval[10] = ((v_B_eval_tmp * i_B_eval_tmp + w_B_eval_tmp * j_B_eval_tmp *
                   k_B_eval_tmp) + u_B_eval_tmp * l_4 * j_B_eval_tmp *
                  h_B_eval_tmp) / I_yy;
    B_eval[16] = -((cb_B_eval_tmp * n_B_eval_tmp * o_B_eval_tmp - eb_B_eval_tmp *
                    m_B_eval_tmp) + x_B_eval_tmp * l_3 * n_B_eval_tmp *
                   l_B_eval_tmp) / I_yy;
    B_eval[22] = ((hb_B_eval_tmp * q_B_eval_tmp + ib_B_eval_tmp * r_B_eval_tmp *
                   s_B_eval_tmp) - y_B_eval_tmp * l_3 * r_B_eval_tmp *
                  p_B_eval_tmp) / I_yy;
    B_eval_tmp = ab_B_eval_tmp * l_4;
    B_eval[28] = ((nb_B_eval_tmp * e_B_eval_tmp * g_B_eval_tmp + lb_B_eval_tmp)
                  - B_eval_tmp * d_B_eval_tmp * e_B_eval_tmp) / I_yy;
    b_B_eval_tmp = db_B_eval_tmp * l_4;
    B_eval[34] = -((rb_B_eval_tmp * i_B_eval_tmp * k_B_eval_tmp - pb_B_eval_tmp)
                   + b_B_eval_tmp * h_B_eval_tmp * i_B_eval_tmp) / I_yy;
    t_B_eval_tmp = gb_B_eval_tmp * l_3;
    B_eval[40] = ((vb_B_eval_tmp * m_B_eval_tmp * o_B_eval_tmp + tb_B_eval_tmp)
                  + t_B_eval_tmp * l_B_eval_tmp * m_B_eval_tmp) / I_yy;
    u_B_eval_tmp = jb_B_eval_tmp * l_3;
    B_eval[46] = ((t - absx * q_B_eval_tmp * s_B_eval_tmp) + u_B_eval_tmp *
                  p_B_eval_tmp * q_B_eval_tmp) / I_yy;
    f_B_eval_tmp *= B_eval_tmp;
    B_eval[52] = -(ob_B_eval_tmp * d_B_eval_tmp + f_B_eval_tmp * g_B_eval_tmp) /
      I_yy;
    j_B_eval_tmp *= b_B_eval_tmp;
    B_eval[58] = (sb_B_eval_tmp * h_B_eval_tmp - j_B_eval_tmp * k_B_eval_tmp) /
      I_yy;
    n_B_eval_tmp *= t_B_eval_tmp;
    B_eval[64] = -(wb_B_eval_tmp * l_B_eval_tmp - n_B_eval_tmp * o_B_eval_tmp) /
      I_yy;
    r_B_eval_tmp *= u_B_eval_tmp;
    B_eval[70] = (tol * p_B_eval_tmp + r_B_eval_tmp * s_B_eval_tmp) / I_yy;
    B_eval[5] = ((2.0 * K_p_M * Omega_1 * cos(b_1) * d_B_eval_tmp - c_B_eval_tmp
                  * e_B_eval_tmp) + 2.0 * K_p_T * Omega_1 * l_4 * cos(b_1) *
                 g_B_eval_tmp) / I_zz;
    B_eval[11] = ((bb_B_eval_tmp * i_B_eval_tmp - 2.0 * K_p_M * Omega_2 * cos
                   (b_2) * h_B_eval_tmp) + 2.0 * K_p_T * Omega_2 * l_4 * cos(b_2)
                  * k_B_eval_tmp) / I_zz;
    B_eval[17] = ((fb_B_eval_tmp * m_B_eval_tmp + 2.0 * K_p_M * Omega_3 * cos
                   (b_3) * l_B_eval_tmp) - 2.0 * K_p_T * Omega_3 * l_3 * cos(b_3)
                  * o_B_eval_tmp) / I_zz;
    B_eval[23] = -((kb_B_eval_tmp * q_B_eval_tmp + 2.0 * K_p_M * Omega_4 * cos
                    (b_4) * p_B_eval_tmp) + 2.0 * K_p_T * Omega_4 * l_3 * cos
                   (b_4) * s_B_eval_tmp) / I_zz;
    B_eval[29] = -((mb_B_eval_tmp + nb_B_eval_tmp * d_B_eval_tmp * e_B_eval_tmp)
                   + B_eval_tmp * e_B_eval_tmp * g_B_eval_tmp) / I_zz;
    B_eval[35] = ((qb_B_eval_tmp + rb_B_eval_tmp * h_B_eval_tmp * i_B_eval_tmp)
                  - b_B_eval_tmp * i_B_eval_tmp * k_B_eval_tmp) / I_zz;
    B_eval[41] = ((ub_B_eval_tmp - vb_B_eval_tmp * l_B_eval_tmp * m_B_eval_tmp)
                  + t_B_eval_tmp * m_B_eval_tmp * o_B_eval_tmp) / I_zz;
    B_eval[47] = ((absx * p_B_eval_tmp * q_B_eval_tmp - absxk) + u_B_eval_tmp *
                  q_B_eval_tmp * s_B_eval_tmp) / I_zz;
    B_eval[53] = -(ob_B_eval_tmp * g_B_eval_tmp - f_B_eval_tmp * d_B_eval_tmp) /
      I_zz;
    B_eval[59] = (sb_B_eval_tmp * k_B_eval_tmp + j_B_eval_tmp * h_B_eval_tmp) /
      I_zz;
    B_eval[65] = -(wb_B_eval_tmp * o_B_eval_tmp + n_B_eval_tmp * l_B_eval_tmp) /
      I_zz;
    B_eval[71] = (tol * s_B_eval_tmp - r_B_eval_tmp * p_B_eval_tmp) / I_zz;
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
    desired_u[0] = desired_motor_value;
    desired_u[1] = desired_motor_value;
    desired_u[2] = desired_motor_value;
    desired_u[3] = desired_motor_value;
    desired_u[4] = desired_el_value;
    desired_u[5] = desired_el_value;
    desired_u[6] = desired_el_value;
    desired_u[7] = desired_el_value;
    desired_u[8] = desired_az_value;
    desired_u[9] = desired_az_value;
    desired_u[10] = desired_az_value;
    desired_u[11] = desired_az_value;
    for (i1 = 0; i1 < 12; i1++) {
      u_scaled[i1] = actual_u[i1] - desired_u[i1];
    }

    for (i1 = 0; i1 < 6; i1++) {
      absxk = 0.0;
      for (i2 = 0; i2 < 12; i2++) {
        absxk += B_eval[i1 + 6 * i2] * u_scaled[i2];
      }

      b_dv[i1] = dv[i1] + absxk;
    }

    /*  Apply gains for the normalization of the effectiveness matrix: */
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_i = i2 + 6 * i1;
        B_eval[b_i] *= gain_motor;
        b_i = i2 + 6 * (i1 + 4);
        B_eval[b_i] *= gain_el;
        b_i = i2 + 6 * (i1 + 8);
        B_eval[b_i] *= gain_az;
      }
    }

    /*  Apply basic inversion to compute the control input array: */
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 12; i2++) {
        A[i2 + 12 * i1] = B_eval[i1 + 6 * i2];
      }
    }

    b_p = true;
    for (ar = 0; ar < 72; ar++) {
      B_eval[ar] = 0.0;
      if ((!b_p) || (rtIsInf(A[ar]) || rtIsNaN(A[ar]))) {
        b_p = false;
      }
    }

    if (!b_p) {
      for (i1 = 0; i1 < 72; i1++) {
        B_eval[i1] = rtNaN;
      }
    } else {
      double U[72];
      double b_V[36];
      double s[6];
      int b_r;
      svd(A, U, s, b_V);
      absx = fabs(s[0]);
      if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
        if (absx <= 2.2250738585072014E-308) {
          tol = 4.94065645841247E-324;
        } else {
          frexp(absx, &br);
          tol = ldexp(1.0, br - 53);
        }
      } else {
        tol = rtNaN;
      }

      tol *= 12.0;
      b_r = -1;
      ar = 0;
      while ((ar < 6) && (s[ar] > tol)) {
        b_r++;
        ar++;
      }

      if (b_r + 1 > 0) {
        b_i = 1;
        for (br = 0; br <= b_r; br++) {
          absx = 1.0 / s[br];
          i1 = b_i + 5;
          for (ar = b_i; ar <= i1; ar++) {
            b_V[ar - 1] *= absx;
          }

          b_i += 6;
        }

        for (b_i = 0; b_i <= 66; b_i += 6) {
          i1 = b_i + 1;
          i2 = b_i + 6;
          if (i1 <= i2) {
            memset(&B_eval[i1 + -1], 0, ((i2 - i1) + 1) * sizeof(double));
          }
        }

        br = 0;
        for (b_i = 0; b_i <= 66; b_i += 6) {
          ar = -1;
          br++;
          i1 = br + 12 * b_r;
          for (ib = br; ib <= i1; ib += 12) {
            int i3;
            i2 = b_i + 1;
            i3 = b_i + 6;
            for (ic = i2; ic <= i3; ic++) {
              B_eval[ic - 1] += U[ib - 1] * b_V[(ar + ic) - b_i];
            }

            ar += 6;
          }
        }
      }
    }

    for (i1 = 0; i1 < 12; i1++) {
      absxk = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        absxk += B_eval[i2 + 6 * i1] * b_dv[i2];
      }

      u_scaled[i1] = absxk;
    }

    /* Scale back the increment to remove the normalization */
    u_scaled[0] *= gain_motor;
    u_scaled[4] *= gain_el;
    u_scaled[8] *= gain_az;
    u_scaled[1] *= gain_motor;
    u_scaled[5] *= gain_el;
    u_scaled[9] *= gain_az;
    u_scaled[2] *= gain_motor;
    u_scaled[6] *= gain_el;
    u_scaled[10] *= gain_az;
    u_scaled[3] *= gain_motor;
    u_scaled[7] *= gain_el;
    u_scaled[11] *= gain_az;
    varargin_2[0] = max_omega;
    varargin_2[1] = max_omega;
    varargin_2[2] = max_omega;
    varargin_2[3] = max_omega;
    varargin_2[4] = max_b;
    varargin_2[5] = max_b;
    varargin_2[6] = max_b;
    varargin_2[7] = max_b;
    varargin_2[8] = max_g;
    varargin_2[9] = max_g;
    varargin_2[10] = max_g;
    varargin_2[11] = max_g;
    for (ar = 0; ar < 12; ar++) {
      absxk = u_scaled[ar] + desired_u[ar];
      desired_u[ar] = absxk;
      x[ar] = fmin(absxk, varargin_2[ar]);
    }

    varargin_2[0] = min_omega;
    varargin_2[1] = min_omega;
    varargin_2[2] = min_omega;
    varargin_2[3] = min_omega;
    varargin_2[4] = min_b;
    varargin_2[5] = min_b;
    varargin_2[6] = min_b;
    varargin_2[7] = min_b;
    varargin_2[8] = min_g;
    varargin_2[9] = min_g;
    varargin_2[10] = min_g;
    varargin_2[11] = min_g;
    for (ar = 0; ar < 12; ar++) {
      u_out_feasible[ar] = fmax(x[ar], varargin_2[ar]);
    }

    c_compute_acc_nonlinear_earth_r(actual_u, Theta, Phi, Psi, p, q, r, K_p_T,
      K_p_M, m, I_xx, I_yy, I_zz, l_1, l_2, l_3, l_4, l_z, Cl_alpha, Cd_zero,
      K_Cd, Cm_alpha, Cm_zero, rho, V, S, wing_chord, flight_path_angle, Beta,
      residuals);
    c_compute_acc_nonlinear_earth_r(u_out_feasible, Theta, Phi, Psi, p, q, r,
      K_p_T, K_p_M, m, I_xx, I_yy, I_zz, l_1, l_2, l_3, l_4, l_z, Cl_alpha,
      Cd_zero, K_Cd, Cm_alpha, Cm_zero, rho, V, S, wing_chord, flight_path_angle,
      Beta, b_dv);
    for (i1 = 0; i1 < 6; i1++) {
      residuals[i1] = (dv[i1] + residuals[i1]) - b_dv[i1];
    }

    *elapsed_time = toc();
    for (b_i = 0; b_i < 12; b_i++) {
      b_x[b_i] = rtIsNaN(u_scaled[b_i]);
    }

    b_i = b_x[0];
    for (ar = 0; ar < 11; ar++) {
      b_i += b_x[ar + 1];
    }

    if (b_i > 0.5) {
      *exitflag = -1.0;
    } else {
      *exitflag = 1.0;
    }

    memcpy(&u_out_scaled[0], &desired_u[0], 12U * sizeof(double));
    u_out_scaled[12] = desired_phi_value * 3.1415926535897931 / 180.0;
    u_out_scaled[13] = desired_theta_value * 3.1415926535897931 / 180.0;
    memcpy(&u_out_local_data[0], &u_out_scaled[0], 14U * sizeof(double));
    *N_iterations = 1.0;
    *N_evaluation = 1.0;
  } else {
    memset(&u_out_local_data[0], 0, 12U * sizeof(double));
    for (b_i = 0; b_i < 6; b_i++) {
      residuals[b_i] = 0.0;
    }

    *elapsed_time = 0.0;
    *N_iterations = 0.0;
    *N_evaluation = 0.0;
    *exitflag = -10.0;
  }

  memcpy(&u_out[0], &u_out_local_data[0], 14U * sizeof(double));

  /*  Print infos */
  if (verbose != 0.0) {
    printf("\n Solution = ");
    fflush(stdout);
    for (b_i = 0; b_i < 12; b_i++) {
      printf(" %f ", u_out_local_data[b_i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time = %f \n", *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations = %f \n", *N_iterations);
    fflush(stdout);
    printf("\n Number of evaluation = %f \n", *N_evaluation);
    fflush(stdout);
    printf("\n Residuals = ");
    fflush(stdout);
    for (b_i = 0; b_i < 6; b_i++) {
      printf(" %f ", residuals[b_i]);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    absx = 0.0;
    tol = 3.3121686421112381E-170;
    for (ar = 0; ar < 6; ar++) {
      absxk = fabs(residuals[ar]);
      if (absxk > tol) {
        t = tol / absxk;
        absx = absx * t * t + 1.0;
        tol = absxk;
      } else {
        t = absxk / tol;
        absx += t * t;
      }
    }

    absx = tol * sqrt(absx);
    printf("\n Residual norm = %f \n", absx);
    fflush(stdout);
    memcpy(&u_out_scaled[0], &u_out[0], 14U * sizeof(double));
    gain_motor = max_omega / 2.0;
    gain_el = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
    gain_az = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
    u_out_scaled[0] = u_out[0] / gain_motor;
    u_out_scaled[4] /= gain_el;
    u_out_scaled[8] /= gain_az;
    u_out_scaled[1] = u_out[1] / gain_motor;
    u_out_scaled[5] /= gain_el;
    u_out_scaled[9] /= gain_az;
    u_out_scaled[2] = u_out[2] / gain_motor;
    u_out_scaled[6] /= gain_el;
    u_out_scaled[10] /= gain_az;
    u_out_scaled[3] = u_out[3] / gain_motor;
    u_out_scaled[7] /= gain_el;
    u_out_scaled[11] /= gain_az;
    absx = 0.0;
    tol = 3.3121686421112381E-170;
    for (ar = 0; ar < 14; ar++) {
      absxk = fabs(u_out_scaled[ar]);
      if (absxk > tol) {
        t = tol / absxk;
        absx = absx * t * t + 1.0;
        tol = absxk;
      } else {
        t = absxk / tol;
        absx += t * t;
      }
    }

    printf("\n Solution scaled norm = %f \n", tol * sqrt(absx));
    fflush(stdout);
    printf("\n Exit flag optimizer = %f \n", *exitflag);
    fflush(stdout);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Global_controller_fcn_earth_rf_journal_initialize(void)
{
  savedTime_not_empty = false;
  freq_not_empty = false;
  isInitialized_Global_controller_fcn_earth_rf_journal = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Global_controller_fcn_earth_rf_journal_terminate(void)
{
  isInitialized_Global_controller_fcn_earth_rf_journal = false;
}

/*
 * File trailer for Global_controller_fcn_earth_rf_journal.c
 *
 * [EOF]
 */
