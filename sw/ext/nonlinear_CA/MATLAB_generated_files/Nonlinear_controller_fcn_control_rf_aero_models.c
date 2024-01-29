/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Nonlinear_controller_fcn_control_rf_aero_models.c
 *
 * Code generation for function 'Nonlinear_controller_fcn_control_rf_aero_models'
 *
 */

/* Include files */
#include "Nonlinear_controller_fcn_control_rf_aero_models.h"
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

#ifndef typedef_c_struct_T
#define typedef_c_struct_T

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
  captured_var *gain_airspeed;
  captured_var *gain_ailerons;
  captured_var *gamma_quadratic_du;
  captured_var *l_1;
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
} c_struct_T;

#endif                                 /* typedef_c_struct_T */

#ifndef typedef_nested_function
#define typedef_nested_function

typedef struct {
  c_struct_T workspace;
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

#ifndef typedef_d_struct_T
#define typedef_d_struct_T

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
} d_struct_T;

#endif                                 /* typedef_d_struct_T */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T

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
} e_struct_T;

#endif                                 /* typedef_e_struct_T */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T

typedef struct {
  double workspace_double[496];
  int workspace_int[31];
  int workspace_sort[31];
} f_struct_T;

#endif                                 /* typedef_f_struct_T */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T

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
} g_struct_T;

#endif                                 /* typedef_g_struct_T */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

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
} h_struct_T;

#endif                                 /* typedef_h_struct_T */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T

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
} i_struct_T;

#endif                                 /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T

typedef struct {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
} j_struct_T;

#endif                                 /* typedef_j_struct_T */

#ifndef typedef_k_struct_T
#define typedef_k_struct_T

typedef struct {
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
} k_struct_T;

#endif                                 /* typedef_k_struct_T */

/* Variable Definitions */
static double freq;
static bool freq_not_empty;
static coderTimespec savedTime;
static bool savedTime_not_empty;
static bool isInitialized_Nonlinear_controller_fcn_control_rf_aero_models =
  false;

/* Function Declarations */
static bool BFGSUpdate(int nvar, double Bk[225], const double sk[16], double yk
  [16], double workspace[496]);
static void PresolveWorkingSet(h_struct_T *solution, f_struct_T *memspace,
  i_struct_T *workingset, d_struct_T *qrmanager);
static void RemoveDependentIneq_(i_struct_T *workingset, d_struct_T *qrmanager,
  f_struct_T *memspace, double tolfactor);
static void addBoundToActiveSetMatrix_(i_struct_T *obj, int TYPE, int idx_local);
static void b_computeGradLag(double workspace[496], int nVar, const double grad
  [16], const int finiteFixed[16], int mFixed, const int finiteLB[16], int mLB,
  const int finiteUB[16], int mUB, const double lambda[31]);
static void b_driver(const double lb[15], const double ub[15], h_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, f_struct_T
                     *memspace, i_struct_T *WorkingSet, double Hessian[225],
                     d_struct_T *QRManager, e_struct_T *CholManager, struct_T
                     *QPObjective);
static double b_maxConstraintViolation(const i_struct_T *obj, const double x[16]);
static double b_norm(const double x[15]);
static void b_test_exit(k_struct_T *Flags, f_struct_T *memspace, b_struct_T
  *MeritFunction, const i_struct_T *WorkingSet, h_struct_T *TrialState,
  d_struct_T *QRManager, const double lb[15], const double ub[15]);
static double b_timeKeeper(double *outTime_tv_nsec);
static void b_xgemm(int m, int n, int k, const double A[961], int ia0, const
                    double B[496], double C[961]);
static void b_xgemv(int m, int n, const double A[961], const double x[16],
                    double y[496]);
static double b_xnrm2(int n, const double x[16]);
static void c_CoderTimeAPI_callCoderClockGe(void);
static double c_computeObjectiveAndUserGradie(const c_struct_T
  *c_obj_next_next_next_next_next_, const double x[15], double grad_workspace[16],
  int *status);
static double c_norm(const double x[6]);
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
static void computeQ_(d_struct_T *obj, int nrows);
static void compute_acc(double CL_aileron, double Cd_zero, double Cl_alpha,
  double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz, double
  K_Cd, double Omega_1, double Omega_2, double Omega_3, double Omega_4, double
  Omega_1_scaled, double Omega_2_scaled, double Phi, double S, double Theta,
  double V, double b_1, double b_2, double b_3, double b_4, double
  delta_ailerons, double flight_path_angle, double g_1, double g_2, double g_3,
  double g_4, double gain_airspeed, double l_1, double l_3, double l_4, double
  l_z, double m, double p, double prop_R, double prop_Cd_0, double prop_Cl_0,
  double prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma,
  double prop_theta, double q, double r, double rho, double wing_span, double
  wing_chord, double computed_acc[6]);
static double compute_cost_and_gradient(double Beta, double CL_aileron, double
  Cd_zero, double Cl_alpha, double Cm_zero, double Cm_alpha, double I_xx, double
  I_yy, double I_zz, double K_Cd, double Omega_1_scaled, double Omega_2_scaled,
  double Omega_3_scaled, double Omega_4_scaled, double Phi_scaled, double S,
  double Theta_scaled, double V, double W_act_phi, double W_act_theta, double
  W_act_motor, double W_dv_1, double W_dv_2, double W_dv_3, double W_dv_4,
  double W_dv_5, double W_dv_6, double W_act_tilt_el, double W_act_tilt_az,
  double W_act_ailerons, double b_1_scaled, double b_2_scaled, double b_3_scaled,
  double b_4_scaled, double delta_ailerons_scaled, double desired_el_value,
  double desired_az_value, double desired_phi_value, double desired_theta_value,
  double desired_motor_value, double desired_ailerons_value, double dv_global_1,
  double dv_global_2, double dv_global_3, double dv_global_4, double dv_global_5,
  double dv_global_6, double flight_path_angle, double g_1_scaled, double
  g_2_scaled, double g_3_scaled, double g_4_scaled, double gain_el, double
  gain_az, double gain_phi, double gain_theta, double gain_motor, double
  gain_airspeed, double gain_ailerons, double gamma_quadratic_du, double l_1,
  double l_3, double l_4, double l_z, double m, double p, double power_Cd_0,
  double power_Cd_a, double prop_R, double prop_Cd_0, double prop_Cl_0, double
  prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma, double
  prop_theta, double q, double r, double rho, double wing_span, double
  wing_chord, double gradient_data[], int *gradient_size);
static void compute_deltax(const double H[225], h_struct_T *solution, f_struct_T
  *memspace, const d_struct_T *qrmanager, e_struct_T *cholmanager, const
  struct_T *objective, bool alwaysPositiveDef);
static void countsort(int x[31], int xLen, int workspace[31], int xMin, int xMax);
static void deleteColMoveEnd(d_struct_T *obj, int idx);
static int div_nde_s32_floor(int numerator);
static void driver(const double H[225], const double f[16], h_struct_T *solution,
                   f_struct_T *memspace, i_struct_T *workingset, d_struct_T
                   *qrmanager, e_struct_T *cholmanager, struct_T *objective,
                   j_struct_T *options, int runTimeOptions_MaxIterations);
static double evalObjAndConstr(const c_struct_T *c_obj_next_next_next_next_next_,
  const double x[15], int *status);
static void factorQR(d_struct_T *obj, const double A[496], int mrows, int ncols);
static void factoryConstruct(c_struct_T *objfun_workspace, const double lb[15],
  const double ub[15], g_struct_T *obj);
static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const i_struct_T *workingset, d_struct_T *qrmanager);
static double feasibleratiotest(const double solution_xstar[16], const double
  solution_searchDir[16], int workingset_nVar, const double workingset_lb[16],
  const double workingset_ub[16], const int workingset_indexLB[16], const int
  workingset_indexUB[16], const int workingset_sizes[5], const int
  workingset_isActiveIdx[6], const bool workingset_isActiveConstr[31], const int
  workingset_nWConstr[5], bool isPhaseOne, bool *newBlocking, int *constrType,
  int *constrIdx);
static double fmincon(c_struct_T *fun_workspace, double x0[15], const double lb
                      [15], const double ub[15], double *exitflag, double
                      *output_iterations, double *output_funcCount, char
                      output_algorithm[3], double *output_constrviolation,
                      double *output_stepsize, double *output_lssteplength,
                      double *output_firstorderopt);
static double ft_1(const double ct[262], double gradient_data[], int
                   *gradient_size);
static void fullColLDL2_(e_struct_T *obj, int NColsRemain);
static void iterate(const double H[225], const double f[16], h_struct_T
                    *solution, f_struct_T *memspace, i_struct_T *workingset,
                    d_struct_T *qrmanager, e_struct_T *cholmanager, struct_T
                    *objective, const char options_SolverName[7], double
                    options_StepTolerance, double options_ObjectiveLimit, int
                    runTimeOptions_MaxIterations);
static void linearForm_(bool obj_hasLinear, int obj_nvar, double workspace[496],
  const double H[225], const double f[16], const double x[16]);
static double maxConstraintViolation(const i_struct_T *obj, const double x[496],
  int ix0);
static double maximum(const double x[2]);
static double minimum(const double x[2]);
static void qrf(double A[961], int m, int n, int nfxd, double tau[31]);
static void removeConstr(i_struct_T *obj, int idx_global);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static void setProblemType(i_struct_T *obj, int PROBLEM_TYPE);
static void solve(const e_struct_T *obj, double rhs[16]);
static void sortLambdaQP(double lambda[31], int WorkingSet_nActiveConstr, const
  int WorkingSet_sizes[5], const int WorkingSet_isActiveIdx[6], const int
  WorkingSet_Wid[31], const int WorkingSet_Wlocalidx[31], double workspace[496]);
static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], h_struct_T *TrialState, b_struct_T
                 *MeritFunction, f_struct_T *memspace, i_struct_T *WorkingSet,
                 d_struct_T *QRManager, e_struct_T *CholManager, struct_T
                 *QPObjective, j_struct_T *qpoptions);
static bool test_exit(b_struct_T *MeritFunction, const i_struct_T *WorkingSet,
                      h_struct_T *TrialState, const double lb[15], const double
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

    if (!(1.0 - theta == 0.0)) {
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
    if (!(curvatureS == 0.0)) {
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
    if (!(curvatureS == 0.0)) {
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

static void PresolveWorkingSet(h_struct_T *solution, f_struct_T *memspace,
  i_struct_T *workingset, d_struct_T *qrmanager)
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
      tol = b_maxConstraintViolation(workingset, solution->xstar);
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

static void RemoveDependentIneq_(i_struct_T *workingset, d_struct_T *qrmanager,
  f_struct_T *memspace, double tolfactor)
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

static void addBoundToActiveSetMatrix_(i_struct_T *obj, int TYPE, int idx_local)
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

static void b_driver(const double lb[15], const double ub[15], h_struct_T
                     *TrialState, b_struct_T *MeritFunction, const
                     i_coder_internal_stickyStruct *FcnEvaluator, f_struct_T
                     *memspace, i_struct_T *WorkingSet, double Hessian[225],
                     d_struct_T *QRManager, e_struct_T *CholManager, struct_T
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

  j_struct_T b_expl_temp;
  j_struct_T expl_temp;
  k_struct_T Flags;
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

      expl_temp.ObjectiveLimit = rtMinusInf;
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

        TrialState->sqpFval = evalObjAndConstr
          (&FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace,
           TrialState->xstarsqp, &ineqStart);
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
        int exitflagLnSrch;
        bool evalWellDefined;
        bool socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }

        evalWellDefined = Flags.fevalOK;
        i = WorkingSet->nVar;
        alpha = 1.0;
        exitflagLnSrch = 1;
        phi_alpha = MeritFunction->phiFullStep;
        ineqStart = (unsigned char)WorkingSet->nVar;
        if (ineqStart - 1 >= 0) {
          memcpy(&TrialState->searchDir[0], &TrialState->delta_x[0], (unsigned
                  int)ineqStart * sizeof(double));
        }

        int exitg1;
        do {
          exitg1 = 0;
          if (TrialState->FunctionEvaluations < 1000 && toc() <= max_time_optimizer_s) {
            if (evalWellDefined && (phi_alpha <= MeritFunction->phi + alpha *
                                    0.0001 * MeritFunction->phiPrimePlus)) {
              exitg1 = 1;
            } else {
              bool exitg2;
              bool tooSmallX;
              alpha *= 0.7;
              ineqStart = (unsigned char)i;
              for (ixlast = 0; ixlast < ineqStart; ixlast++) {
                TrialState->delta_x[ixlast] = alpha * TrialState->xstar[ixlast];
              }

              if (socTaken) {
                phi_alpha = alpha * alpha;
                if ((i >= 1) && (!(phi_alpha == 0.0))) {
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
                if (1.0E-12 * fmax(1.0, fabs(TrialState->xstarsqp[ixlast])) <=
                    fabs(TrialState->delta_x[ixlast])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  ixlast++;
                }
              }

              if (tooSmallX) {
                exitflagLnSrch = -2;
                exitg1 = 1;
              } else {
                for (ixlast = 0; ixlast < ineqStart; ixlast++) {
                  TrialState->xstarsqp[ixlast] = TrialState->xstarsqp_old[ixlast]
                    + TrialState->delta_x[ixlast];
                }

                TrialState->sqpFval = evalObjAndConstr
                  (&FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace,
                   TrialState->xstarsqp, &ixlast);
                TrialState->FunctionEvaluations++;
                evalWellDefined = (ixlast == 1);
                if (evalWellDefined) {
                  phi_alpha = TrialState->sqpFval;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            exitflagLnSrch = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);

        Flags.fevalOK = evalWellDefined;
        TrialState->steplength = alpha;
        if (exitflagLnSrch > 0) {
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

      TrialState->sqpFval = c_computeObjectiveAndUserGradie
        (&FcnEvaluator->next.next.next.next.next.next.next.next.value.workspace,
         TrialState->xstarsqp, TrialState->grad, &ineqStart);
      if (ineqStart == 1) {
        ineqStart = 1;
      }

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

static double b_maxConstraintViolation(const i_struct_T *obj, const double x[16])
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

static void b_test_exit(k_struct_T *Flags, f_struct_T *memspace, b_struct_T
  *MeritFunction, const i_struct_T *WorkingSet, h_struct_T *TrialState,
  d_struct_T *QRManager, const double lb[15], const double ub[15])
{
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
  if (rtIsInf(optimRelativeFactor)) {
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
    dxTooSmall = ((!rtIsInf(TrialState->gradLag[idx_max])) && (!rtIsNaN
      (TrialState->gradLag[idx_max])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag[idx_max]));
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
    smax = fmax(smax, MeritFunction->nlpComplError);
    MeritFunction->firstOrderOpt = smax;
    if (TrialState->sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      b_computeGradLag(memspace->workspace_double, WorkingSet->nVar,
                       TrialState->grad, WorkingSet->indexFixed,
                       WorkingSet->sizes[0], WorkingSet->indexLB,
                       WorkingSet->sizes[3], WorkingSet->indexUB,
                       WorkingSet->sizes[4], TrialState->lambdaStopTestPrev);
      s = 0.0;
      idx_max = 0;
      while ((idx_max <= i1 - 1) && ((!rtIsInf(memspace->
                workspace_double[idx_max])) && (!rtIsNaN
               (memspace->workspace_double[idx_max])))) {
        s = fmax(s, fabs(memspace->workspace_double[idx_max]));
        idx_max++;
      }

      nlpComplErrorTmp = computeComplError(TrialState->xstarsqp,
        WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
        WorkingSet->sizes[4], ub, TrialState->lambdaStopTestPrev,
        WorkingSet->sizes[0] + 1);
      d = fmax(s, nlpComplErrorTmp);
      if (d < smax) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
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

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-12 *
                       optimRelativeFactor) && (MeritFunction->nlpComplError <=
         1.0E-12 * optimRelativeFactor)) {
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
            if (1.0E-12 * fmax(1.0, fabs(TrialState->xstarsqp[idx_max])) <= fabs
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

                b_xgemv(WorkingSet->nVar, WorkingSet->nVar, QRManager->Q,
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
                smax = 0.0;
                idx_max = 0;
                while ((idx_max <= i1 - 1) && ((!rtIsInf
                         (memspace->workspace_double[idx_max])) && (!rtIsNaN
                         (memspace->workspace_double[idx_max])))) {
                  smax = fmax(smax, fabs(memspace->workspace_double[idx_max]));
                  idx_max++;
                }

                s = computeComplError(TrialState->xstarsqp, WorkingSet->indexLB,
                                      WorkingSet->sizes[3], lb,
                                      WorkingSet->indexUB, WorkingSet->sizes[4],
                                      ub, TrialState->lambda, WorkingSet->sizes
                                      [0] + 1);
                if ((smax <= 1.0E-12 * optimRelativeFactor) && (s <= 1.0E-12 *
                     optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = smax;
                  MeritFunction->nlpComplError = s;
                  MeritFunction->firstOrderOpt = fmax(smax, s);
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
          if (TrialState->sqpIterations >= 300 || toc() >= max_time_optimizer_s) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 1000 || toc() >= max_time_optimizer_s) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

static double b_timeKeeper(double *outTime_tv_nsec)
{
  double outTime_tv_sec;
  outTime_tv_sec = savedTime.tv_sec;
  *outTime_tv_nsec = savedTime.tv_nsec;
  return outTime_tv_sec;
}

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

static void b_xgemv(int m, int n, const double A[961], const double x[16],
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

      i1 = div_nde_s32_floor(iac - 1);
      y[i1] += c;
    }
  }
}

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

static void c_CoderTimeAPI_callCoderClockGe(void)
{
  freq_not_empty = false;
}

static double c_computeObjectiveAndUserGradie(const c_struct_T
  *c_obj_next_next_next_next_next_, const double x[15], double grad_workspace[16],
  int *status)
{
  double gradient_data[15];
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double fval;
  int idx_current;
  bool allFinite;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  fval = compute_cost_and_gradient(c_obj_next_next_next_next_next_->
    Beta->contents, c_obj_next_next_next_next_next_->CL_aileron->contents,
    c_obj_next_next_next_next_next_->Cd_zero->contents,
    c_obj_next_next_next_next_next_->Cl_alpha->contents,
    c_obj_next_next_next_next_next_->Cm_zero->contents,
    c_obj_next_next_next_next_next_->Cm_alpha->contents,
    c_obj_next_next_next_next_next_->I_xx->contents,
    c_obj_next_next_next_next_next_->I_yy->contents,
    c_obj_next_next_next_next_next_->I_zz->contents,
    c_obj_next_next_next_next_next_->K_Cd->contents, x[0], x[1], x[2], x[3], x
    [13], c_obj_next_next_next_next_next_->S->contents, x[12],
    c_obj_next_next_next_next_next_->V->contents,
    c_obj_next_next_next_next_next_->W_act_phi->contents,
    c_obj_next_next_next_next_next_->W_act_theta->contents,
    c_obj_next_next_next_next_next_->W_act_motor->contents,
    c_obj_next_next_next_next_next_->W_dv_1->contents,
    c_obj_next_next_next_next_next_->W_dv_2->contents,
    c_obj_next_next_next_next_next_->W_dv_3->contents,
    c_obj_next_next_next_next_next_->W_dv_4->contents,
    c_obj_next_next_next_next_next_->W_dv_5->contents,
    c_obj_next_next_next_next_next_->W_dv_6->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_el->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_az->contents,
    c_obj_next_next_next_next_next_->W_act_ailerons->contents, x[4], x[5], x[6],
    x[7], x[14], c_obj_next_next_next_next_next_->desired_el_value->contents,
    c_obj_next_next_next_next_next_->desired_az_value->contents,
    c_obj_next_next_next_next_next_->desired_phi_value->contents,
    c_obj_next_next_next_next_next_->desired_theta_value->contents,
    c_obj_next_next_next_next_next_->desired_motor_value->contents,
    c_obj_next_next_next_next_next_->desired_ailerons_value->contents,
    dv_global_1, dv_global_2, dv_global_3, dv_global_4, dv_global_5, dv_global_6,
    c_obj_next_next_next_next_next_->flight_path_angle->contents, x[8], x[9], x
    [10], x[11], c_obj_next_next_next_next_next_->gain_el->contents,
    c_obj_next_next_next_next_next_->gain_az->contents,
    c_obj_next_next_next_next_next_->gain_phi->contents,
    c_obj_next_next_next_next_next_->gain_theta->contents,
    c_obj_next_next_next_next_next_->gain_motor->contents,
    c_obj_next_next_next_next_next_->gain_airspeed->contents,
    c_obj_next_next_next_next_next_->gain_ailerons->contents,
    c_obj_next_next_next_next_next_->gamma_quadratic_du->contents,
    c_obj_next_next_next_next_next_->l_1->contents,
    c_obj_next_next_next_next_next_->l_3->contents,
    c_obj_next_next_next_next_next_->l_4->contents,
    c_obj_next_next_next_next_next_->l_z->contents,
    c_obj_next_next_next_next_next_->m->contents,
    c_obj_next_next_next_next_next_->p->contents,
    c_obj_next_next_next_next_next_->power_Cd_0->contents,
    c_obj_next_next_next_next_next_->power_Cd_a->contents,
    c_obj_next_next_next_next_next_->prop_R->contents,
    c_obj_next_next_next_next_next_->prop_Cd_0->contents,
    c_obj_next_next_next_next_next_->prop_Cl_0->contents,
    c_obj_next_next_next_next_next_->prop_Cd_a->contents,
    c_obj_next_next_next_next_next_->prop_Cl_a->contents,
    c_obj_next_next_next_next_next_->prop_delta->contents,
    c_obj_next_next_next_next_next_->prop_sigma->contents,
    c_obj_next_next_next_next_next_->prop_theta->contents,
    c_obj_next_next_next_next_next_->q->contents,
    c_obj_next_next_next_next_next_->r->contents,
    c_obj_next_next_next_next_next_->rho->contents,
    c_obj_next_next_next_next_next_->wing_span->contents,
    c_obj_next_next_next_next_next_->wing_chord->contents, gradient_data,
    &idx_current);

  /*  [cost_primary,gradient_primary] = compute_primary_cost_and_gradient(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero, ... */
  /*      Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S, ... */
  /*      Theta_scaled,V,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,b_1_scaled,b_2_scaled,b_3_scaled,b_4_scaled,delta_ailerons_scaled, ... */
  /*      dv_global_1,dv_global_2,dv_global_3,dv_global_4,dv_global_5,dv_global_6,flight_path_angle,g_1_scaled,g_2_scaled, ... */
  /*      g_3_scaled,g_4_scaled,gain_el,gain_az,gain_phi,gain_theta,gain_motor,gain_airspeed,gain_ailerons,l_1,l_3,l_4,l_z,m, ... */
  /*      p,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,prop_delta,prop_sigma,prop_theta,q,r,rho,wing_span,wing_chord); */
  /*   */
  /*  [cost_secondary,gradient_secondary] = compute_secondary_cost_and_gradient(Omega_1_scaled,Omega_2_scaled,Omega_3_scaled, ... */
  /*      Omega_4_scaled,Phi_scaled,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_act_tilt_el,W_act_tilt_az,W_act_ailerons, ... */
  /*      b_1_scaled,b_2_scaled,b_3_scaled,b_4_scaled,delta_ailerons_scaled,desired_el_value,desired_az_value,desired_phi_value, ... */
  /*      desired_theta_value,desired_motor_value,desired_ailerons_value,flight_path_angle,g_1_scaled,g_2_scaled,g_3_scaled, ... */
  /*      g_4_scaled,gain_el,gain_az,gain_phi,gain_theta,gain_motor,gain_ailerons,gamma_quadratic_du,power_Cd_0,power_Cd_a, ... */
  /*      prop_R,prop_Cl_0,prop_Cl_a,prop_delta,prop_sigma,prop_theta,rho); */
  memcpy(&grad_workspace[0], &gradient_data[0], 15U * sizeof(double));
  *status = 1;
  allFinite = rtIsNaN(fval);
  if (rtIsInf(fval) || allFinite) {
    if (allFinite) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
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

  return fval;
}

static double c_norm(const double x[6])
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

static void computeQ_(d_struct_T *obj, int nrows)
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

              iQR0 = div_nde_s32_floor((n - iaii) - 31);
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

static void compute_acc(double CL_aileron, double Cd_zero, double Cl_alpha,
  double Cm_zero, double Cm_alpha, double I_xx, double I_yy, double I_zz, double
  K_Cd, double Omega_1, double Omega_2, double Omega_3, double Omega_4, double
  Omega_1_scaled, double Omega_2_scaled, double Phi, double S, double Theta,
  double V, double b_1, double b_2, double b_3, double b_4, double
  delta_ailerons, double flight_path_angle, double g_1, double g_2, double g_3,
  double g_4, double gain_airspeed, double l_1, double l_3, double l_4, double
  l_z, double m, double p, double prop_R, double prop_Cd_0, double prop_Cl_0,
  double prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma,
  double prop_theta, double q, double r, double rho, double wing_span, double
  wing_chord, double computed_acc[6])
{
  double t10;
  double t11;
  double t117_tmp;
  double t117_tmp_tmp;
  double t12;
  double t13;
  double t14;
  double t143;
  double t15;
  double t16;
  double t169;
  double t17;
  double t170;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t222;
  double t23;
  double t237;
  double t238;
  double t239;
  double t24;
  double t26;
  double t27;
  double t274;
  double t28;
  double t29;
  double t3;
  double t30;
  double t33;
  double t34;
  double t35;
  double t36;
  double t37;
  double t39;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t48;
  double t49;
  double t5;
  double t52;
  double t54;
  double t55;
  double t56;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t62;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t71;
  double t73;
  double t74;
  double t75;
  double t76;
  double t77;
  double t78;
  double t79;
  double t8;
  double t80;
  double t85;
  double t86;
  double t87;
  double t88;
  double t89;
  double t9;
  double t91_tmp;

  /* COMPUTE_ACC */
  /*     COMPUTED_ACC = COMPUTE_ACC(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1,Omega_2,Omega_3,Omega_4,Omega_1_scaled,Omega_2_scaled,Phi,S,Theta,V,B_1,B_2,B_3,B_4,DELTA_AILERONS,FLIGHT_PATH_ANGLE,G_1,G_2,G_3,G_4,GAIN_AIRSPEED,L_1,L_3,L_4,L_Z,M,P,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_SPAN,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     25-Jan-2024 12:06:07 */
  t3 = cos(Phi);
  t5 = cos(Theta);
  t6 = sin(Phi);
  t7 = cos(b_1);
  t8 = cos(b_2);
  t9 = cos(b_3);
  t10 = cos(b_4);
  t11 = sin(Theta);
  t12 = cos(g_1);
  t13 = cos(g_2);
  t14 = cos(g_3);
  t15 = cos(g_4);
  t16 = sin(b_1);
  t17 = sin(b_2);
  t18 = sin(b_3);
  t19 = sin(b_4);
  t20 = sin(g_1);
  t21 = sin(g_2);
  t22 = sin(g_3);
  t23 = sin(g_4);
  t24 = log(prop_delta);
  t26 = Omega_1 * Omega_1;
  t27 = Omega_2 * Omega_2;
  t28 = Omega_3 * Omega_3;
  t29 = Omega_4 * Omega_4;
  t30 = V * V;
  t33 = rt_powd_snf(prop_R, 3.0);
  t34 = rt_powd_snf(prop_R, 4.0);
  t37 = prop_Cd_0 * prop_delta * 2.0;
  t39 = 1.0 / Omega_1;
  t41 = 1.0 / Omega_2;
  t43 = 1.0 / Omega_3;
  t45 = 1.0 / Omega_4;
  t48 = 1.0 / (gain_airspeed * gain_airspeed);
  t49 = 1.0 / m;
  t52 = prop_delta * 16.0;
  t54 = 1.0 / prop_R;
  t56 = 1.0 / prop_delta;
  t35 = t9 * t9;
  t36 = t10 * t10;
  t40 = 1.0 / t26;
  t42 = 1.0 / t27;
  t44 = 1.0 / t28;
  t46 = 1.0 / t29;
  t55 = t54 * t54;
  t222 = prop_Cl_0 * prop_delta;
  t58 = t222 * (prop_delta + 1.0);
  t59 = Theta - flight_path_angle;
  t66 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t67 = prop_Cl_a * prop_sigma * (prop_delta - 1.0) / 8.0;
  t60 = cos(t59);
  t61 = sin(t59);
  t62 = t58 * 8.0;
  t68 = (b_1 + 1.5707963267948966) + t59;
  t69 = (b_2 + 1.5707963267948966) + t59;
  t70 = (b_3 + 1.5707963267948966) + t59;
  t71 = (b_4 + 1.5707963267948966) + t59;
  t73 = cos(t68);
  t74 = cos(t69);
  t75 = cos(t70);
  t76 = cos(t71);
  t77 = sin(t68);
  t78 = sin(t69);
  t79 = sin(t70);
  t80 = sin(t71);
  t91_tmp = Cl_alpha * S * rho * t30 * t59;
  t85 = t77 * t77;
  t86 = t78 * t78;
  t87 = t79 * t79;
  t88 = t80 * t80;
  t89 = Cd_zero + K_Cd * (Cl_alpha * Cl_alpha) * (t59 * t59);
  t117_tmp_tmp = S * rho;
  t117_tmp = t117_tmp_tmp * t30;
  t143 = t91_tmp * t60 / 2.0 + t117_tmp * t61 * t89 / 2.0;
  t274 = t30 * t40 * t55;
  t68 = V * prop_Cl_a * prop_sigma;
  t69 = prop_Cl_0 * prop_sigma * t24 * t30;
  t70 = prop_sigma * (prop_delta - 1.0) * t56;
  t169 = (t67 + V * t39 * t54 * t73 / 2.0) + sqrt(((t274 * (t73 * t73) * 16.0 +
    t68 * t39 * (prop_delta - 1.0) * t54 * t73 * 8.0) + t69 * t40 * t55 * t85 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t85 * 8.0))))
    / 8.0;
  t274 = t30 * t42 * t55;
  t170 = (t67 + V * t41 * t54 * t74 / 2.0) + sqrt(((t274 * (t74 * t74) * 16.0 +
    t68 * t41 * (prop_delta - 1.0) * t54 * t74 * 8.0) + t69 * t42 * t55 * t86 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t86 * 8.0))))
    / 8.0;
  t274 = t30 * t44 * t55;
  t71 = (t67 + V * t43 * t54 * t75 / 2.0) + sqrt(((t274 * (t75 * t75) * 16.0 +
    t68 * t43 * (prop_delta - 1.0) * t54 * t75 * 8.0) + t69 * t44 * t55 * t87 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t87 * 8.0))))
    / 8.0;
  t274 = t30 * t46 * t55;
  t75 = (t67 + V * t45 * t54 * t76 / 2.0) + sqrt(((t274 * (t76 * t76) * 16.0 +
    t68 * t45 * (prop_delta - 1.0) * t54 * t76 * 8.0) + t69 * t46 * t55 * t88 *
    -8.0) - t70 * (t62 + prop_Cl_a * (t66 + prop_theta * (t52 + t274 * t88 * 8.0))))
    / 8.0;
  t74 = t222 * t24;
  t274 = prop_Cl_a - prop_Cd_a * 2.0;
  t39 = prop_Cd_a * prop_theta * 2.0;
  t237 = t74 * t169 + (prop_delta - 1.0) * (t37 + prop_theta * (t169 * t274 +
    t39));
  t238 = t74 * t170 + (prop_delta - 1.0) * (t37 + prop_theta * (t170 * t274 +
    t39));
  t239 = t74 * t71 + (prop_delta - 1.0) * (t37 + prop_theta * (t71 * t274 + t39));
  t37 = t74 * t75 + (prop_delta - 1.0) * (t37 + prop_theta * (t75 * t274 + t39));
  t274 = t74 * t30;
  t39 = prop_Cl_a * prop_theta * t30;
  t73 = prop_Cl_a * prop_delta;
  t41 = t274 * t40 * t55 * t85 + (prop_delta - 1.0) * ((t58 + t39 * t40 * t55 *
    t85) - t73 * (-prop_theta + t169) * 2.0);
  t222 = t274 * t42 * t55 * t86 + (prop_delta - 1.0) * ((t58 + t39 * t42 * t55 *
    t86) - t73 * (-prop_theta + t170) * 2.0);
  t66 = t274 * t44 * t55 * t87 + (prop_delta - 1.0) * ((t58 + t39 * t44 * t55 *
    t87) - t73 * (-prop_theta + t71) * 2.0);
  t71 = t274 * t46 * t55 * t88 + (prop_delta - 1.0) * ((t58 + t39 * t46 * t55 *
    t88) - t73 * (-prop_theta + t75) * 2.0);
  t274 = prop_sigma * rho;
  t73 = Omega_1 * V * prop_sigma * rho;
  t67 = t274 * t16 * t26 * t34 * t56 * t41 * 3.1415926535897931 / 4.0 + t73 * t7
    * t33 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t69 = Omega_2 * V * prop_sigma * rho;
  t62 = t274 * t17 * t27 * t34 * t56 * t222 * 3.1415926535897931 / 4.0 + t69 *
    t8 * t33 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t70 = Omega_3 * V * prop_sigma * rho;
  t76 = t274 * t18 * t28 * t34 * t56 * t66 * 3.1415926535897931 / 4.0 + t70 * t9
    * t33 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t74 = Omega_4 * V * prop_sigma * rho;
  t24 = t274 * t19 * t29 * t34 * t56 * t71 * 3.1415926535897931 / 4.0 + t74 *
    t10 * t33 * t56 * t80 * t37 * 3.1415926535897931 / 4.0;
  t75 = t274 * t7;
  t54 = t75 * t12 * t26 * t34 * t56 * t41 * 3.1415926535897931 / 4.0 - t73 * t12
    * t16 * t33 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t68 = t274 * t8;
  t52 = t68 * t13 * t27 * t34 * t56 * t222 * 3.1415926535897931 / 4.0 - t69 *
    t13 * t17 * t33 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t39 = t274 * t9;
  t45 = t39 * t14 * t28 * t34 * t56 * t66 * 3.1415926535897931 / 4.0 - t70 * t14
    * t18 * t33 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t274 *= t10;
  t43 = t274 * t15 * t29 * t34 * t56 * t71 * 3.1415926535897931 / 4.0 - t74 *
    t15 * t19 * t33 * t56 * t80 * t37 * 3.1415926535897931 / 4.0;
  t41 = t75 * t20 * t26 * t34 * t56 * t41 * 3.1415926535897931 / 4.0 - t73 * t16
    * t20 * t33 * t56 * t77 * t237 * 3.1415926535897931 / 4.0;
  t75 = t68 * t21 * t27 * t34 * t56 * t222 * 3.1415926535897931 / 4.0 - t69 *
    t17 * t21 * t33 * t56 * t78 * t238 * 3.1415926535897931 / 4.0;
  t73 = t39 * t22 * t28 * t34 * t56 * t66 * 3.1415926535897931 / 4.0 - t70 * t18
    * t22 * t33 * t56 * t79 * t239 * 3.1415926535897931 / 4.0;
  t70 = t274 * t23 * t29 * t34 * t56 * t71 * 3.1415926535897931 / 4.0 - t74 *
    t19 * t23 * t33 * t56 * t80 * t37 * 3.1415926535897931 / 4.0;
  t71 = ((t67 + t62) + t76) + t24;
  t274 = ((t41 + t75) + t73) + t70;
  t39 = ((t54 + t52) + t45) + t43;
  t74 = t3 * t11;
  t68 = t91_tmp * t61 / 2.0 - t117_tmp * t60 * t89 / 2.0;
  t69 = t117_tmp_tmp * 0.0;
  computed_acc[0] = t49 * (((((t5 * t71 + t5 * t68) - t74 * t143) + t74 * t39) -
    t6 * t11 * t274) - t69 * t6 * t11 * t30 * t89 / 2.0);
  computed_acc[1] = -t49 * (((-t6 * t143 + t3 * t274) + t6 * t39) + t117_tmp_tmp
    * t3 * 0.0 * t30 * t89 / 2.0);
  t74 = t3 * t5;
  computed_acc[2] = -t49 * (((((t11 * t71 + t11 * t68) + t74 * t143) - t74 * t39)
    + t5 * t6 * t274) + t69 * t5 * t6 * t30 * t89 / 2.0) + 9.81;
  t74 = Omega_1_scaled * t7;
  t68 = Omega_2_scaled * t8;
  computed_acc[3] = -(((((((((((l_1 * t54 - l_1 * t52) - l_1 * t45) + l_1 * t43)
    + l_z * t41) + l_z * t75) + l_z * t73) + l_z * t70) - I_yy * q * r) + I_zz *
                        q * r) - CL_aileron * S * delta_ailerons * rho * t30 /
                       2.0) + t117_tmp * wing_span * ((((((t74 *
    0.01228634392023026 - t68 * 0.01228634392023026) + Omega_1_scaled *
    Omega_1_scaled * t7 * 0.0075456152077779167) - Omega_2_scaled *
    Omega_2_scaled * t8 * 0.0075456152077779167) + Omega_1_scaled * t16 * t36 *
    0.0046429750925043979) + (((Omega_2_scaled * t17 * t35 *
    -0.0046429750925043979 + Omega_2_scaled * t18 * t35 * 0.0064381447596962606)
    - Omega_1_scaled * t19 * t36 * 0.0064381447596962606) - Omega_1_scaled * (t7
    * t7) * t19 * 0.0039349871274520724)) + ((Omega_2_scaled * (t8 * t8) * t18 *
    0.0039349871274520724 - t74 * t30 * t48 * 0.020516396677824081) + t68 * t30 *
    t48 * 0.020516396677824081)) / 2.0) / I_xx;
  t74 = I_xx * p;
  computed_acc[4] = -((((((((((l_4 * t54 - l_3 * t45) + l_4 * t52) - l_3 * t43)
    + l_z * t67) + l_z * t62) + l_z * t76) + l_z * t24) + t74 * r) - I_zz * p *
                       r) - t117_tmp * wing_chord * (Cm_zero + Cm_alpha * t59) /
                      2.0) / I_yy;
  computed_acc[5] = (((((((((l_1 * t67 - l_1 * t62) - l_1 * t76) + l_1 * t24) -
    l_4 * t41) + l_3 * t73) - l_4 * t75) + l_3 * t70) + t74 * q) - I_yy * p * q)
    / I_zz;
}

static double compute_cost_and_gradient(double Beta, double CL_aileron, double
  Cd_zero, double Cl_alpha, double Cm_zero, double Cm_alpha, double I_xx, double
  I_yy, double I_zz, double K_Cd, double Omega_1_scaled, double Omega_2_scaled,
  double Omega_3_scaled, double Omega_4_scaled, double Phi_scaled, double S,
  double Theta_scaled, double V, double W_act_phi, double W_act_theta, double
  W_act_motor, double W_dv_1, double W_dv_2, double W_dv_3, double W_dv_4,
  double W_dv_5, double W_dv_6, double W_act_tilt_el, double W_act_tilt_az,
  double W_act_ailerons, double b_1_scaled, double b_2_scaled, double b_3_scaled,
  double b_4_scaled, double delta_ailerons_scaled, double desired_el_value,
  double desired_az_value, double desired_phi_value, double desired_theta_value,
  double desired_motor_value, double desired_ailerons_value, double dv_global_1,
  double dv_global_2, double dv_global_3, double dv_global_4, double dv_global_5,
  double dv_global_6, double flight_path_angle, double g_1_scaled, double
  g_2_scaled, double g_3_scaled, double g_4_scaled, double gain_el, double
  gain_az, double gain_phi, double gain_theta, double gain_motor, double
  gain_airspeed, double gain_ailerons, double gamma_quadratic_du, double l_1,
  double l_3, double l_4, double l_z, double m, double p, double power_Cd_0,
  double power_Cd_a, double prop_R, double prop_Cd_0, double prop_Cl_0, double
  prop_Cd_a, double prop_Cl_a, double prop_delta, double prop_sigma, double
  prop_theta, double q, double r, double rho, double wing_span, double
  wing_chord, double gradient_data[], int *gradient_size)
{
  double b_CL_aileron[262];
  double b_gradient_data[15];
  double cost;
  double t10;
  double t101;
  double t103;
  double t104;
  double t105;
  double t11;
  double t110;
  double t112;
  double t113;
  double t118;
  double t119;
  double t12;
  double t120;
  double t121;
  double t124;
  double t126;
  double t127;
  double t13;
  double t134;
  double t139;
  double t14;
  double t140;
  double t140_tmp;
  double t150;
  double t151;
  double t152;
  double t153;
  double t154;
  double t155;
  double t156;
  double t157;
  double t158;
  double t159;
  double t159_tmp;
  double t16;
  double t161;
  double t162;
  double t163;
  double t164;
  double t165;
  double t166;
  double t167;
  double t168;
  double t17;
  double t18;
  double t188;
  double t188_tmp_tmp;
  double t19;
  double t199;
  double t2;
  double t20;
  double t21;
  double t22;
  double t220;
  double t220_tmp;
  double t23;
  double t24;
  double t242;
  double t242_tmp;
  double t242_tmp_tmp;
  double t25;
  double t3;
  double t350;
  double t351;
  double t38;
  double t388_tmp;
  double t389_tmp;
  double t39;
  double t390_tmp;
  double t391_tmp;
  double t393;
  double t394;
  double t395;
  double t4;
  double t408;
  double t409;
  double t410;
  double t411;
  double t416;
  double t417;
  double t418;
  double t419;
  double t42;
  double t46;
  double t48;
  double t49;
  double t5;
  double t52;
  double t528;
  double t529;
  double t530;
  double t560;
  double t561;
  double t562;
  double t566;
  double t57;
  double t58;
  double t59;
  double t6;
  double t60;
  double t61;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t77;
  double t78;
  double t79;
  double t8;
  double t80;
  double t81;
  double t82;
  double t83;
  double t84;
  double t85;
  double t86;
  double t87;
  double t88;
  double t9;
  double t94;
  double t95;
  double t96;

  /* COMPUTE_COST_AND_GRADIENT */
  /*     [COST,GRADIENT] = COMPUTE_COST_AND_GRADIENT(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero,Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,W_act_tilt_el,W_act_tilt_az,W_act_ailerons,B_1_SCALED,B_2_SCALED,B_3_SCALED,B_4_SCALED,DELTA_AILERONS_SCALED,DESIRED_EL_VALUE,DESIRED_AZ_VALUE,DESIRED_PHI_VALUE,DESIRED_THETA_VALUE,DESIRED_MOTOR_VALUE,DESIRED_AILERONS_VALUE,DV_GLOBAL_1,DV_GLOBAL_2,DV_GLOBAL_3,DV_GLOBAL_4,DV_GLOBAL_5,DV_GLOBAL_6,FLIGHT_PATH_ANGLE,G_1_SCALED,G_2_SCALED,G_3_SCALED,G_4_SCALED,GAIN_EL,GAIN_AZ,GAIN_PHI,GAIN_THETA,GAIN_MOTOR,GAIN_AIRSPEED,GAIN_AILERONS,GAMMA_QUADRATIC_DU,L_1,L_3,L_4,L_Z,M,P,power_Cd_0,power_Cd_a,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,PROP_DELTA,PROP_SIGMA,PROP_THETA,Q,R,RHO,WING_SPAN,WING_CHORD) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2. */
  /*     25-Jan-2024 12:05:53 */
  t2 = cos(Beta);
  t3 = sin(Beta);
  t4 = log(prop_delta);
  t5 = Phi_scaled * gain_phi;
  t6 = Theta_scaled * gain_theta;
  t7 = b_1_scaled * gain_el;
  t8 = b_2_scaled * gain_el;
  t9 = b_3_scaled * gain_el;
  t10 = b_4_scaled * gain_el;
  t11 = g_1_scaled * gain_az;
  t12 = g_2_scaled * gain_az;
  t13 = g_3_scaled * gain_az;
  t14 = g_4_scaled * gain_az;
  t16 = Cl_alpha * Cl_alpha;
  t17 = Omega_1_scaled * Omega_1_scaled;
  t18 = rt_powd_snf(Omega_1_scaled, 3.0);
  t19 = Omega_2_scaled * Omega_2_scaled;
  t20 = rt_powd_snf(Omega_2_scaled, 3.0);
  t21 = Omega_3_scaled * Omega_3_scaled;
  t22 = rt_powd_snf(Omega_3_scaled, 3.0);
  t23 = Omega_4_scaled * Omega_4_scaled;
  t24 = rt_powd_snf(Omega_4_scaled, 3.0);
  t25 = V * V;
  t38 = gain_motor * gain_motor;
  t39 = prop_Cd_a * 2.0;
  t42 = rt_powd_snf(prop_R, 4.0);
  t46 = prop_Cd_0 * prop_delta * 2.0;
  t77 = 1.0 / Omega_1_scaled;
  t79 = 1.0 / Omega_2_scaled;
  t82 = 1.0 / Omega_3_scaled;
  t85 = 1.0 / Omega_4_scaled;
  t94 = 1.0 / gain_motor;
  t96 = 1.0 / (gain_airspeed * gain_airspeed);
  t101 = prop_delta * 16.0;
  t103 = 1.0 / prop_R;
  t105 = 1.0 / prop_delta;
  t48 = cos(t5);
  t49 = cos(t6);
  t52 = sin(t5);
  t57 = cos(t7);
  t58 = cos(t8);
  t59 = cos(t9);
  t60 = cos(t10);
  t61 = sin(t6);
  t66 = sin(t7);
  t67 = sin(t8);
  t68 = sin(t9);
  t69 = sin(t10);
  t78 = 1.0 / t17;
  t80 = 1.0 / t18;
  t81 = 1.0 / t19;
  t83 = 1.0 / t20;
  t84 = 1.0 / t21;
  t86 = 1.0 / t22;
  t87 = 1.0 / t23;
  t88 = 1.0 / t24;
  t95 = 1.0 / t38;
  t104 = t103 * t103;
  t566 = prop_Cl_0 * prop_delta;
  t110 = t566 * (prop_delta + 1.0);
  t112 = desired_el_value * (1.0 / gain_el);
  t113 = desired_az_value * (1.0 / gain_az);
  t139 = prop_delta * prop_sigma * -prop_Cl_a * (prop_delta - 1.0);
  t140_tmp = prop_Cl_a * prop_sigma;
  t140 = t140_tmp * (prop_delta - 1.0) / 8.0;
  t5 = ((t6 + t7) - flight_path_angle) + 1.5707963267948966;
  t7 = ((t6 + t8) - flight_path_angle) + 1.5707963267948966;
  t8 = ((t6 + t9) - flight_path_angle) + 1.5707963267948966;
  t9 = ((t6 + t10) - flight_path_angle) + 1.5707963267948966;
  t118 = t57 * t57;
  t119 = t58 * t58;
  t120 = t59 * t59;
  t121 = t60 * t60;
  t124 = flight_path_angle - t6;
  t10 = t110 * 8.0;
  t134 = power_Cd_0 * prop_delta + power_Cd_a * (prop_theta * prop_theta);
  t151 = cos(t5);
  t152 = cos(t7);
  t153 = cos(t8);
  t154 = cos(t9);
  t155 = sin(t5);
  t156 = sin(t7);
  t157 = sin(t8);
  t158 = sin(t9);
  t126 = cos(t124);
  t127 = sin(t124);
  t161 = t151 * t151;
  t162 = t152 * t152;
  t163 = t153 * t153;
  t164 = t154 * t154;
  t165 = t155 * t155;
  t166 = t156 * t156;
  t167 = t157 * t157;
  t168 = t158 * t158;
  t7 = Cl_alpha * S;
  t159_tmp = t7 * gain_theta * rho * t25;
  t159 = t159_tmp * t126 / 2.0;
  t220_tmp = prop_Cl_a * prop_theta * t25;
  t220 = t220_tmp * t80 * t95 * t104 * t165 * 2.0;
  t242_tmp_tmp = t566 * t4;
  t242_tmp = t242_tmp_tmp * t25;
  t242 = t242_tmp * t80 * t95 * t104 * t165 * 2.0;
  t150 = Cd_zero + K_Cd * t16 * (t124 * t124);
  t188_tmp_tmp = S * rho;
  t5 = t188_tmp_tmp * t2 * t25;
  t188 = t5 * t127 * t150 / 2.0;
  t199 = t5 * t126 * t150 / 2.0;
  t5 = t7 * rho * t25 * t124;
  t350 = t5 * t126 / 2.0 + t188;
  t351 = t5 * t127 / 2.0 - t199;
  t388_tmp = t25 * t78 * t95 * t104;
  t6 = V * prop_Cl_a * prop_sigma;
  t566 = prop_Cl_0 * prop_sigma * t4 * t25;
  t5 = prop_sigma * (prop_delta - 1.0) * t105;
  t7 = sqrt(((t388_tmp * t161 * 16.0 + t6 * t77 * t94 * (prop_delta - 1.0) *
              t103 * t151 * 8.0) + t566 * t78 * t95 * t104 * t165 * -8.0) - t5 *
            (t10 + prop_Cl_a * (t139 + prop_theta * (t101 + t388_tmp * t165 *
    8.0))));
  t389_tmp = t25 * t81 * t95 * t104;
  t8 = sqrt(((t389_tmp * t162 * 16.0 + t6 * t79 * t94 * (prop_delta - 1.0) *
              t103 * t152 * 8.0) + t566 * t81 * t95 * t104 * t166 * -8.0) - t5 *
            (t10 + prop_Cl_a * (t139 + prop_theta * (t101 + t389_tmp * t166 *
    8.0))));
  t390_tmp = t25 * t84 * t95 * t104;
  t9 = sqrt(((t390_tmp * t163 * 16.0 + t6 * t82 * t94 * (prop_delta - 1.0) *
              t103 * t153 * 8.0) + t566 * t84 * t95 * t104 * t167 * -8.0) - t5 *
            (t10 + prop_Cl_a * (t139 + prop_theta * (t101 + t390_tmp * t167 *
    8.0))));
  t391_tmp = t25 * t87 * t95 * t104;
  t5 = sqrt(((t391_tmp * t164 * 16.0 + t6 * t85 * t94 * (prop_delta - 1.0) *
              t103 * t154 * 8.0) + t566 * t87 * t95 * t104 * t168 * -8.0) - t5 *
            (t10 + prop_Cl_a * (t139 + prop_theta * (t101 + t391_tmp * t168 *
    8.0))));
  t139 = 1.0 / t7;
  t393 = 1.0 / t8;
  t394 = 1.0 / t9;
  t395 = 1.0 / t5;
  t408 = (t140 + V * t77 * t94 * t103 * t151 / 2.0) + t7 / 8.0;
  t409 = (t140 + V * t79 * t94 * t103 * t152 / 2.0) + t8 / 8.0;
  t410 = (t140 + V * t82 * t94 * t103 * t153 / 2.0) + t9 / 8.0;
  t411 = (t140 + V * t85 * t94 * t103 * t154 / 2.0) + t5 / 8.0;
  t416 = -prop_theta + t408;
  t417 = -prop_theta + t409;
  t418 = -prop_theta + t410;
  t419 = -prop_theta + t411;
  t5 = t140_tmp * prop_theta * t25;
  t528 = V * t78 * t94 * t103 * t151 / 2.0 + (((t25 * t80 * t95 * t104 * t161 *
    32.0 + t6 * t78 * t94 * (prop_delta - 1.0) * t103 * t151 * 8.0) - t566 * t80
    * t95 * t104 * t165 * 16.0) - t5 * t80 * t95 * (prop_delta - 1.0) * t104 *
    t105 * t165 * 16.0) * t139 / 16.0;
  t529 = V * t81 * t94 * t103 * t152 / 2.0 + (((t25 * t83 * t95 * t104 * t162 *
    32.0 + t6 * t81 * t94 * (prop_delta - 1.0) * t103 * t152 * 8.0) - t566 * t83
    * t95 * t104 * t166 * 16.0) - t5 * t83 * t95 * (prop_delta - 1.0) * t104 *
    t105 * t166 * 16.0) * t393 / 16.0;
  t530 = V * t84 * t94 * t103 * t153 / 2.0 + (((t25 * t86 * t95 * t104 * t163 *
    32.0 + t6 * t84 * t94 * (prop_delta - 1.0) * t103 * t153 * 8.0) - t566 * t86
    * t95 * t104 * t167 * 16.0) - t5 * t86 * t95 * (prop_delta - 1.0) * t104 *
    t105 * t167 * 16.0) * t394 / 16.0;
  t164 = V * t87 * t94 * t103 * t154 / 2.0 + (((t25 * t88 * t95 * t104 * t164 *
    32.0 + t6 * t87 * t94 * (prop_delta - 1.0) * t103 * t154 * 8.0) - t566 * t88
    * t95 * t104 * t168 * 16.0) - t5 * t88 * t95 * (prop_delta - 1.0) * t104 *
    t105 * t168 * 16.0) * t395 / 16.0;
  t5 = V * gain_el;
  t7 = t5 * prop_Cl_a * prop_sigma;
  t8 = gain_el * t25;
  t140 = gain_el * prop_Cl_0;
  t9 = t140 * prop_sigma * t4 * t25;
  t140_tmp = gain_el * prop_Cl_a;
  t10 = t140_tmp * prop_sigma * prop_theta * t25;
  t560 = t5 * t77 * t94 * t103 * t155 / 2.0 + t139 * (((t7 * t77 * t94 *
    (prop_delta - 1.0) * t103 * t155 * 8.0 + t8 * t78 * t95 * t104 * t151 * t155
    * 32.0) + t9 * t78 * t95 * t104 * t151 * t155 * 16.0) + t10 * t78 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t151 * t155 * 16.0) / 16.0;
  t561 = t5 * t79 * t94 * t103 * t156 / 2.0 + t393 * (((t7 * t79 * t94 *
    (prop_delta - 1.0) * t103 * t156 * 8.0 + t8 * t81 * t95 * t104 * t152 * t156
    * 32.0) + t9 * t81 * t95 * t104 * t152 * t156 * 16.0) + t10 * t81 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t152 * t156 * 16.0) / 16.0;
  t562 = t5 * t82 * t94 * t103 * t157 / 2.0 + t394 * (((t7 * t82 * t94 *
    (prop_delta - 1.0) * t103 * t157 * 8.0 + t8 * t84 * t95 * t104 * t153 * t157
    * 32.0) + t9 * t84 * t95 * t104 * t153 * t157 * 16.0) + t10 * t84 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t153 * t157 * 16.0) / 16.0;
  t162 = t5 * t85 * t94 * t103 * t158 / 2.0 + t395 * (((t7 * t85 * t94 *
    (prop_delta - 1.0) * t103 * t158 * 8.0 + t8 * t87 * t95 * t104 * t154 * t158
    * 32.0) + t9 * t87 * t95 * t104 * t154 * t158 * 16.0) + t10 * t87 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t154 * t158 * 16.0) / 16.0;
  t5 = V * gain_theta;
  t7 = t5 * prop_Cl_a * prop_sigma;
  t8 = gain_theta * t25;
  t161 = gain_theta * prop_Cl_0;
  t9 = t161 * prop_sigma * t4 * t25;
  t163 = gain_theta * prop_Cl_a;
  t10 = t163 * prop_sigma * prop_theta * t25;
  t101 = t5 * t77 * t94 * t103 * t155 / 2.0 + t139 * (((t7 * t77 * t94 *
    (prop_delta - 1.0) * t103 * t155 * 8.0 + t8 * t78 * t95 * t104 * t151 * t155
    * 32.0) + t9 * t78 * t95 * t104 * t151 * t155 * 16.0) + t10 * t78 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t151 * t155 * 16.0) / 16.0;
  t139 = t5 * t79 * t94 * t103 * t156 / 2.0 + t393 * (((t7 * t79 * t94 *
    (prop_delta - 1.0) * t103 * t156 * 8.0 + t8 * t81 * t95 * t104 * t152 * t156
    * 32.0) + t9 * t81 * t95 * t104 * t152 * t156 * 16.0) + t10 * t81 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t152 * t156 * 16.0) / 16.0;
  t566 = t5 * t82 * t94 * t103 * t157 / 2.0 + t394 * (((t7 * t82 * t94 *
    (prop_delta - 1.0) * t103 * t157 * 8.0 + t8 * t84 * t95 * t104 * t153 * t157
    * 32.0) + t9 * t84 * t95 * t104 * t153 * t157 * 16.0) + t10 * t84 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t153 * t157 * 16.0) / 16.0;
  t9 = t5 * t85 * t94 * t103 * t158 / 2.0 + t395 * (((t7 * t85 * t94 *
    (prop_delta - 1.0) * t103 * t158 * 8.0 + t8 * t87 * t95 * t104 * t154 * t158
    * 32.0) + t9 * t87 * t95 * t104 * t154 * t158 * 16.0) + t10 * t87 * t95 *
    (prop_delta - 1.0) * t104 * t105 * t154 * t158 * 16.0) / 16.0;
  t10 = prop_Cl_a * prop_delta;
  t6 = t10 * t528 * 2.0;
  b_CL_aileron[0] = CL_aileron;
  b_CL_aileron[1] = Cm_alpha;
  b_CL_aileron[2] = K_Cd;
  b_CL_aileron[3] = Omega_1_scaled;
  b_CL_aileron[4] = Omega_2_scaled;
  b_CL_aileron[5] = Omega_3_scaled;
  b_CL_aileron[6] = Omega_4_scaled;
  b_CL_aileron[7] = Phi_scaled;
  b_CL_aileron[8] = S;
  b_CL_aileron[9] = Theta_scaled;
  b_CL_aileron[10] = V;
  b_CL_aileron[11] = b_1_scaled;
  b_CL_aileron[12] = b_2_scaled;
  b_CL_aileron[13] = b_3_scaled;
  b_CL_aileron[14] = b_4_scaled;
  b_CL_aileron[15] = delta_ailerons_scaled;
  b_CL_aileron[16] = desired_ailerons_value;
  b_CL_aileron[17] = desired_phi_value;
  b_CL_aileron[18] = desired_theta_value;
  b_CL_aileron[19] = dv_global_1;
  b_CL_aileron[20] = dv_global_2;
  b_CL_aileron[21] = dv_global_3;
  b_CL_aileron[22] = dv_global_4;
  b_CL_aileron[23] = dv_global_5;
  b_CL_aileron[24] = dv_global_6;
  b_CL_aileron[25] = g_1_scaled;
  b_CL_aileron[26] = g_2_scaled;
  b_CL_aileron[27] = g_3_scaled;
  b_CL_aileron[28] = g_4_scaled;
  b_CL_aileron[29] = gain_ailerons;
  b_CL_aileron[30] = gain_az;
  b_CL_aileron[31] = gain_el;
  b_CL_aileron[32] = gain_motor;
  b_CL_aileron[33] = gain_phi;
  b_CL_aileron[34] = gain_theta;
  b_CL_aileron[35] = gamma_quadratic_du;
  b_CL_aileron[36] = l_1;
  b_CL_aileron[37] = l_3;
  b_CL_aileron[38] = l_4;
  b_CL_aileron[39] = l_z;
  b_CL_aileron[40] = power_Cd_a;
  b_CL_aileron[41] = prop_Cl_0;
  b_CL_aileron[42] = prop_Cl_a;
  b_CL_aileron[43] = prop_sigma;
  b_CL_aileron[44] = prop_theta;
  b_CL_aileron[45] = rho;
  b_CL_aileron[46] = prop_delta - 1.0;
  b_CL_aileron[47] = t104;
  b_CL_aileron[48] = t105;
  b_CL_aileron[49] = 1.5707963267948966;
  b_CL_aileron[50] = t110;
  b_CL_aileron[51] = desired_motor_value * t94;
  b_CL_aileron[52] = -(I_zz * p * r);
  b_CL_aileron[53] = -(I_yy * q * r);
  b_CL_aileron[54] = t118;
  b_CL_aileron[55] = t119;
  b_CL_aileron[56] = t120;
  b_CL_aileron[57] = t121;
  b_CL_aileron[58] = t124;
  b_CL_aileron[59] = -t112;
  b_CL_aileron[60] = -(t112 * 2.0);
  b_CL_aileron[61] = -t113;
  b_CL_aileron[62] = -(t113 * 2.0);
  b_CL_aileron[63] = t134;
  b_CL_aileron[64] = power_Cd_0 * ((prop_delta + 1.0) + prop_delta * prop_delta)
    * 2.0;
  b_CL_aileron[65] = prop_delta / 6.0 - 0.16666666666666666;
  b_CL_aileron[66] = -(CL_aileron * S * delta_ailerons_scaled * gain_ailerons *
                       rho * t25 / 2.0);
  b_CL_aileron[67] = t150;
  b_CL_aileron[68] = t151;
  b_CL_aileron[69] = t152;
  b_CL_aileron[70] = t153;
  b_CL_aileron[71] = t154;
  b_CL_aileron[72] = t155;
  b_CL_aileron[73] = t156;
  b_CL_aileron[74] = t157;
  b_CL_aileron[75] = t158;
  b_CL_aileron[76] = t16;
  b_CL_aileron[77] = t165;
  b_CL_aileron[78] = t166;
  b_CL_aileron[79] = t167;
  b_CL_aileron[80] = t168;
  b_CL_aileron[81] = t17;
  t8 = t188_tmp_tmp * t25;
  b_CL_aileron[82] = -(t8 * (Cm_zero - Cm_alpha * t124) * wing_chord / 2.0);
  t7 = t188_tmp_tmp * t3 * t25;
  b_CL_aileron[83] = t7 * t48 * t150 / 2.0;
  b_CL_aileron[84] = t18;
  b_CL_aileron[85] = t7 * t49 * t52 * t150 / 2.0;
  b_CL_aileron[86] = t19;
  b_CL_aileron[87] = t20;
  b_CL_aileron[88] = -(t7 * t52 * t61 * t150 / 2.0);
  b_CL_aileron[89] = t220_tmp * t78 * t95 * t104 * t165;
  b_CL_aileron[90] = t220_tmp * t81 * t95 * t104 * t166;
  b_CL_aileron[91] = t220_tmp * t84 * t95 * t104 * t167;
  b_CL_aileron[92] = t220_tmp * t87 * t95 * t104 * t168;
  b_CL_aileron[93] = t21;
  b_CL_aileron[94] = t22;
  b_CL_aileron[95] = t220;
  b_CL_aileron[96] = t220_tmp * t83 * t95 * t104 * t166 * 2.0;
  b_CL_aileron[97] = t220_tmp * t86 * t95 * t104 * t167 * 2.0;
  b_CL_aileron[98] = t220_tmp * t88 * t95 * t104 * t168 * 2.0;
  b_CL_aileron[99] = t242_tmp * t78 * t95 * t104 * t165;
  b_CL_aileron[100] = t242_tmp * t81 * t95 * t104 * t166;
  b_CL_aileron[101] = t242_tmp * t84 * t95 * t104 * t167;
  b_CL_aileron[102] = t242_tmp * t87 * t95 * t104 * t168;
  b_CL_aileron[103] = t23;
  b_CL_aileron[104] = t24;
  b_CL_aileron[105] = t242;
  b_CL_aileron[106] = t242_tmp * t83 * t95 * t104 * t166 * 2.0;
  b_CL_aileron[107] = t242_tmp * t86 * t95 * t104 * t167 * 2.0;
  b_CL_aileron[108] = t242_tmp * t88 * t95 * t104 * t168 * 2.0;
  b_CL_aileron[109] = t25;
  b_CL_aileron[110] = W_act_phi * W_act_phi;
  b_CL_aileron[111] = W_act_theta * W_act_theta;
  b_CL_aileron[112] = W_act_motor * W_act_motor;
  b_CL_aileron[113] = W_dv_1 * W_dv_1;
  b_CL_aileron[114] = t3;
  b_CL_aileron[115] = W_dv_2 * W_dv_2;
  b_CL_aileron[116] = t388_tmp * t105 * t134 * t165 * 3.0;
  b_CL_aileron[117] = t389_tmp * t105 * t134 * t166 * 3.0;
  b_CL_aileron[118] = t390_tmp * t105 * t134 * t167 * 3.0;
  b_CL_aileron[119] = t391_tmp * t105 * t134 * t168 * 3.0;
  b_CL_aileron[120] = W_dv_3 * W_dv_3;
  b_CL_aileron[121] = W_dv_4 * W_dv_4;
  b_CL_aileron[122] = W_dv_5 * W_dv_5;
  t7 = t140 * prop_delta * t4 * t25;
  b_CL_aileron[123] = t7 * t78 * t95 * t104 * t151 * t155 * 2.0;
  b_CL_aileron[124] = t7 * t81 * t95 * t104 * t152 * t156 * 2.0;
  b_CL_aileron[125] = t7 * t84 * t95 * t104 * t153 * t157 * 2.0;
  b_CL_aileron[126] = t7 * t87 * t95 * t104 * t154 * t158 * 2.0;
  t7 = t161 * prop_delta * t4 * t25;
  b_CL_aileron[127] = t7 * t78 * t95 * t104 * t151 * t155 * 2.0;
  b_CL_aileron[128] = t7 * t81 * t95 * t104 * t152 * t156 * 2.0;
  b_CL_aileron[129] = t7 * t84 * t95 * t104 * t153 * t157 * 2.0;
  b_CL_aileron[130] = t7 * t87 * t95 * t104 * t154 * t158 * 2.0;
  b_CL_aileron[131] = W_dv_6 * W_dv_6;
  b_CL_aileron[132] = W_act_tilt_el * W_act_tilt_el;
  b_CL_aileron[133] = t350;
  b_CL_aileron[134] = W_act_tilt_az * W_act_tilt_az;
  b_CL_aileron[135] = t52 * t350;
  b_CL_aileron[136] = t49 * t351;
  b_CL_aileron[137] = t61 * t351;
  b_CL_aileron[138] = t48 * t61 * t350;
  b_CL_aileron[139] = -(t48 * t49 * t350);
  b_CL_aileron[140] = W_act_ailerons * W_act_ailerons;
  t7 = K_Cd * S * gain_theta * rho * t2 * t16 * t25 * t124;
  b_CL_aileron[141] = ((t159 + t159_tmp * t124 * t127 * -0.5) + t7 * t127) +
    gain_theta * t199;
  b_CL_aileron[142] = ((t159_tmp * t127 / 2.0 + t124 * t159) - t7 * t126) +
    gain_theta * t188;
  b_CL_aileron[143] = t38;
  b_CL_aileron[144] = t39;
  b_CL_aileron[145] = prop_delta + 1.0;
  b_CL_aileron[146] = rt_powd_snf(prop_R, 3.0);
  b_CL_aileron[147] = prop_Cl_a * t408 * 6.0;
  b_CL_aileron[148] = prop_Cl_a * t409 * 6.0;
  b_CL_aileron[149] = prop_Cl_a * t410 * 6.0;
  b_CL_aileron[150] = prop_Cl_a * t411 * 6.0;
  b_CL_aileron[151] = t416;
  b_CL_aileron[152] = t417;
  b_CL_aileron[153] = t418;
  b_CL_aileron[154] = t419;
  b_CL_aileron[155] = t42;
  b_CL_aileron[156] = t242_tmp_tmp * t408;
  b_CL_aileron[157] = t242_tmp_tmp * t409;
  b_CL_aileron[158] = t242_tmp_tmp * t410;
  b_CL_aileron[159] = t242_tmp_tmp * t411;
  t7 = prop_Cl_0 * (prop_delta + 1.0);
  b_CL_aileron[160] = t7 * t408 * 3.0;
  b_CL_aileron[161] = t7 * t409 * 3.0;
  b_CL_aileron[162] = t7 * t410 * 3.0;
  b_CL_aileron[163] = t7 * t411 * 3.0;
  b_CL_aileron[164] = rt_powd_snf(prop_R, 5.0);
  b_CL_aileron[165] = -(power_Cd_a * t416 * 6.0);
  b_CL_aileron[166] = -(power_Cd_a * t417 * 6.0);
  b_CL_aileron[167] = -(power_Cd_a * t418 * 6.0);
  b_CL_aileron[168] = -(power_Cd_a * t419 * 6.0);
  b_CL_aileron[169] = -(t10 * t416 * 2.0);
  b_CL_aileron[170] = -(t10 * t417 * 2.0);
  b_CL_aileron[171] = -(t10 * t418 * 2.0);
  b_CL_aileron[172] = -(t10 * t419 * 2.0);
  t7 = prop_Cl_a - t39;
  t5 = prop_Cd_a * prop_theta * 2.0;
  b_CL_aileron[173] = t46 + prop_theta * (t408 * t7 + t5);
  b_CL_aileron[174] = t46 + prop_theta * (t409 * t7 + t5);
  b_CL_aileron[175] = t46 + prop_theta * (t410 * t7 + t5);
  b_CL_aileron[176] = t46 + prop_theta * (t411 * t7 + t5);
  b_CL_aileron[177] = t48;
  b_CL_aileron[178] = t49;
  t5 = I_xx * p;
  b_CL_aileron[179] = t5 * q;
  b_CL_aileron[180] = I_yy * p * q;
  b_CL_aileron[181] = t52;
  b_CL_aileron[182] = t528;
  b_CL_aileron[183] = t529;
  b_CL_aileron[184] = t5 * r;
  b_CL_aileron[185] = t530;
  b_CL_aileron[186] = t164;
  b_CL_aileron[187] = t6;
  b_CL_aileron[188] = t10 * t529 * 2.0;
  b_CL_aileron[189] = t10 * t530 * 2.0;
  b_CL_aileron[190] = t10 * t164 * 2.0;
  b_CL_aileron[191] = t242_tmp_tmp * t528;
  b_CL_aileron[192] = t242_tmp_tmp * t529;
  b_CL_aileron[193] = t242_tmp_tmp * t530;
  b_CL_aileron[194] = t242_tmp_tmp * t164;
  t5 = prop_theta * (prop_delta - 1.0);
  b_CL_aileron[195] = t5 * t528 * t7;
  b_CL_aileron[196] = t5 * t529 * t7;
  b_CL_aileron[197] = t5 * t530 * t7;
  b_CL_aileron[198] = t5 * t164 * t7;
  b_CL_aileron[199] = I_zz * q * r;
  b_CL_aileron[200] = t560;
  b_CL_aileron[201] = t561;
  b_CL_aileron[202] = t562;
  b_CL_aileron[203] = t162;
  b_CL_aileron[204] = t101;
  b_CL_aileron[205] = t139;
  b_CL_aileron[206] = t566;
  b_CL_aileron[207] = t9;
  b_CL_aileron[208] = t57;
  b_CL_aileron[209] = t58;
  b_CL_aileron[210] = t242_tmp_tmp * t560;
  b_CL_aileron[211] = t242_tmp_tmp * t561;
  b_CL_aileron[212] = t242_tmp_tmp * t562;
  b_CL_aileron[213] = t242_tmp_tmp * t162;
  b_CL_aileron[214] = t242_tmp_tmp * t101;
  b_CL_aileron[215] = t242_tmp_tmp * t139;
  b_CL_aileron[216] = t242_tmp_tmp * t566;
  b_CL_aileron[217] = t242_tmp_tmp * t9;
  b_CL_aileron[218] = t59;
  b_CL_aileron[219] = t60;
  b_CL_aileron[220] = t61;
  t7 = t140_tmp * prop_theta * t25;
  b_CL_aileron[221] = t7 * t78 * t95 * t104 * t151 * t155 * 2.0 + t10 * t560 *
    2.0;
  b_CL_aileron[222] = t7 * t81 * t95 * t104 * t152 * t156 * 2.0 + t10 * t561 *
    2.0;
  b_CL_aileron[223] = t7 * t84 * t95 * t104 * t153 * t157 * 2.0 + t10 * t562 *
    2.0;
  b_CL_aileron[224] = t7 * t87 * t95 * t104 * t154 * t158 * 2.0 + t10 * t162 *
    2.0;
  t7 = t163 * prop_theta * t25;
  b_CL_aileron[225] = t7 * t78 * t95 * t104 * t151 * t155 * 2.0 + t10 * t101 *
    2.0;
  b_CL_aileron[226] = t7 * t81 * t95 * t104 * t152 * t156 * 2.0 + t10 * t139 *
    2.0;
  b_CL_aileron[227] = t7 * t84 * t95 * t104 * t153 * t157 * 2.0 + t10 * t566 *
    2.0;
  b_CL_aileron[228] = t7 * t87 * t95 * t104 * t154 * t158 * 2.0 + t10 * t9 * 2.0;
  b_CL_aileron[229] = cos(t11);
  b_CL_aileron[230] = prop_sigma * rho * t17 * t38 * t42 * t66 * t105 *
    3.1415926535897931 * (t242 + (prop_delta - 1.0) * (t220 - t6)) * -0.25;
  b_CL_aileron[231] = cos(t12);
  b_CL_aileron[232] = cos(t13);
  t7 = Omega_2_scaled * t68;
  t5 = Omega_1_scaled * t69;
  b_CL_aileron[233] = t8 * (((((((((((Omega_1_scaled * t57 * 0.01228634392023026
    - Omega_2_scaled * t58 * 0.01228634392023026) + t17 * t57 *
    0.0075456152077779167) - t19 * t58 * 0.0075456152077779167) + t7 * t120 *
    0.0064381447596962606) + t7 * t119 * 0.0039349871274520724) + Omega_1_scaled
    * t66 * t121 * 0.0046429750925043979) + Omega_2_scaled * t25 * t58 * t96 *
    0.020516396677824081) - t5 * t121 * 0.0064381447596962606) - t5 * t118 *
    0.0039349871274520724) - Omega_2_scaled * t67 * t120 * 0.0046429750925043979)
    - Omega_1_scaled * t25 * t57 * t96 * 0.020516396677824081) * wing_span / 2.0;
  b_CL_aileron[234] = cos(t14);
  b_CL_aileron[235] = t66;
  b_CL_aileron[236] = t67;
  b_CL_aileron[237] = t68;
  b_CL_aileron[238] = t69;
  b_CL_aileron[239] = sin(t11);
  b_CL_aileron[240] = sin(t12);
  b_CL_aileron[241] = sin(t13);
  b_CL_aileron[242] = sin(t14);
  b_CL_aileron[243] = 1.0 / I_xx;
  b_CL_aileron[244] = 1.0 / I_yy;
  b_CL_aileron[245] = 1.0 / I_zz;
  b_CL_aileron[246] = t78;
  b_CL_aileron[247] = t80;
  b_CL_aileron[248] = t81;
  b_CL_aileron[249] = t83;
  b_CL_aileron[250] = t84;
  b_CL_aileron[251] = t86;
  b_CL_aileron[252] = t87;
  b_CL_aileron[253] = t88;
  b_CL_aileron[254] = 1.0 / gain_phi;
  b_CL_aileron[255] = 1.0 / gain_theta;
  b_CL_aileron[256] = t95;
  b_CL_aileron[257] = t96;
  b_CL_aileron[258] = 1.0 / gain_ailerons;
  b_CL_aileron[259] = 1.0 / m;
  b_CL_aileron[260] = wing_chord;
  b_CL_aileron[261] = wing_span;
  cost = ft_1(b_CL_aileron, b_gradient_data, gradient_size);
  if (*gradient_size - 1 >= 0) {
    memcpy(&gradient_data[0], &b_gradient_data[0], (unsigned int)*gradient_size *
           sizeof(double));
  }

  return cost;
}

static void compute_deltax(const double H[225], h_struct_T *solution, f_struct_T
  *memspace, const d_struct_T *qrmanager, e_struct_T *cholmanager, const
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
          i = objective->nvar + 1;
          for (jA = 0; jA < mNull_tmp; jA++) {
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

              nVars = div_nde_s32_floor(jjA - nullStartIdx_tmp);
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

static void deleteColMoveEnd(d_struct_T *obj, int idx)
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

static int div_nde_s32_floor(int numerator)
{
  int i;
  if ((numerator < 0) && (numerator % 31 != 0)) {
    i = -1;
  } else {
    i = 0;
  }

  return numerator / 31 + i;
}

static void driver(const double H[225], const double f[16], h_struct_T *solution,
                   f_struct_T *memspace, i_struct_T *workingset, d_struct_T
                   *qrmanager, e_struct_T *cholmanager, struct_T *objective,
                   j_struct_T *options, int runTimeOptions_MaxIterations)
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
    solution->maxConstr = b_maxConstraintViolation(workingset, solution->xstar);
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
            mConstr = (unsigned char)nVar_tmp;
            if (mConstr - 1 >= 0) {
              memcpy(&solution->searchDir[0], &solution->xstar[0], (unsigned int)
                     mConstr * sizeof(double));
            }

            PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = b_maxConstraintViolation(workingset, solution->xstar);
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

static double evalObjAndConstr(const c_struct_T *c_obj_next_next_next_next_next_,
  const double x[15], int *status)
{
  double gradient_data[15];
  double dv_global_1;
  double dv_global_2;
  double dv_global_3;
  double dv_global_4;
  double dv_global_5;
  double dv_global_6;
  double fval;
  int gradient_size;
  bool b;
  dv_global_1 = c_obj_next_next_next_next_next_->dv_global->contents[0];
  dv_global_2 = c_obj_next_next_next_next_next_->dv_global->contents[1];
  dv_global_3 = c_obj_next_next_next_next_next_->dv_global->contents[2];
  dv_global_4 = c_obj_next_next_next_next_next_->dv_global->contents[3];
  dv_global_5 = c_obj_next_next_next_next_next_->dv_global->contents[4];
  dv_global_6 = c_obj_next_next_next_next_next_->dv_global->contents[5];
  fval = compute_cost_and_gradient(c_obj_next_next_next_next_next_->
    Beta->contents, c_obj_next_next_next_next_next_->CL_aileron->contents,
    c_obj_next_next_next_next_next_->Cd_zero->contents,
    c_obj_next_next_next_next_next_->Cl_alpha->contents,
    c_obj_next_next_next_next_next_->Cm_zero->contents,
    c_obj_next_next_next_next_next_->Cm_alpha->contents,
    c_obj_next_next_next_next_next_->I_xx->contents,
    c_obj_next_next_next_next_next_->I_yy->contents,
    c_obj_next_next_next_next_next_->I_zz->contents,
    c_obj_next_next_next_next_next_->K_Cd->contents, x[0], x[1], x[2], x[3], x
    [13], c_obj_next_next_next_next_next_->S->contents, x[12],
    c_obj_next_next_next_next_next_->V->contents,
    c_obj_next_next_next_next_next_->W_act_phi->contents,
    c_obj_next_next_next_next_next_->W_act_theta->contents,
    c_obj_next_next_next_next_next_->W_act_motor->contents,
    c_obj_next_next_next_next_next_->W_dv_1->contents,
    c_obj_next_next_next_next_next_->W_dv_2->contents,
    c_obj_next_next_next_next_next_->W_dv_3->contents,
    c_obj_next_next_next_next_next_->W_dv_4->contents,
    c_obj_next_next_next_next_next_->W_dv_5->contents,
    c_obj_next_next_next_next_next_->W_dv_6->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_el->contents,
    c_obj_next_next_next_next_next_->W_act_tilt_az->contents,
    c_obj_next_next_next_next_next_->W_act_ailerons->contents, x[4], x[5], x[6],
    x[7], x[14], c_obj_next_next_next_next_next_->desired_el_value->contents,
    c_obj_next_next_next_next_next_->desired_az_value->contents,
    c_obj_next_next_next_next_next_->desired_phi_value->contents,
    c_obj_next_next_next_next_next_->desired_theta_value->contents,
    c_obj_next_next_next_next_next_->desired_motor_value->contents,
    c_obj_next_next_next_next_next_->desired_ailerons_value->contents,
    dv_global_1, dv_global_2, dv_global_3, dv_global_4, dv_global_5, dv_global_6,
    c_obj_next_next_next_next_next_->flight_path_angle->contents, x[8], x[9], x
    [10], x[11], c_obj_next_next_next_next_next_->gain_el->contents,
    c_obj_next_next_next_next_next_->gain_az->contents,
    c_obj_next_next_next_next_next_->gain_phi->contents,
    c_obj_next_next_next_next_next_->gain_theta->contents,
    c_obj_next_next_next_next_next_->gain_motor->contents,
    c_obj_next_next_next_next_next_->gain_airspeed->contents,
    c_obj_next_next_next_next_next_->gain_ailerons->contents,
    c_obj_next_next_next_next_next_->gamma_quadratic_du->contents,
    c_obj_next_next_next_next_next_->l_1->contents,
    c_obj_next_next_next_next_next_->l_3->contents,
    c_obj_next_next_next_next_next_->l_4->contents,
    c_obj_next_next_next_next_next_->l_z->contents,
    c_obj_next_next_next_next_next_->m->contents,
    c_obj_next_next_next_next_next_->p->contents,
    c_obj_next_next_next_next_next_->power_Cd_0->contents,
    c_obj_next_next_next_next_next_->power_Cd_a->contents,
    c_obj_next_next_next_next_next_->prop_R->contents,
    c_obj_next_next_next_next_next_->prop_Cd_0->contents,
    c_obj_next_next_next_next_next_->prop_Cl_0->contents,
    c_obj_next_next_next_next_next_->prop_Cd_a->contents,
    c_obj_next_next_next_next_next_->prop_Cl_a->contents,
    c_obj_next_next_next_next_next_->prop_delta->contents,
    c_obj_next_next_next_next_next_->prop_sigma->contents,
    c_obj_next_next_next_next_next_->prop_theta->contents,
    c_obj_next_next_next_next_next_->q->contents,
    c_obj_next_next_next_next_next_->r->contents,
    c_obj_next_next_next_next_next_->rho->contents,
    c_obj_next_next_next_next_next_->wing_span->contents,
    c_obj_next_next_next_next_next_->wing_chord->contents, gradient_data,
    &gradient_size);

  /*  [cost_primary,gradient_primary] = compute_primary_cost_and_gradient(Beta,CL_aileron,Cd_zero,Cl_alpha,Cm_zero, ... */
  /*      Cm_alpha,I_xx,I_yy,I_zz,K_Cd,Omega_1_scaled,Omega_2_scaled,Omega_3_scaled,Omega_4_scaled,Phi_scaled,S, ... */
  /*      Theta_scaled,V,W_dv_1,W_dv_2,W_dv_3,W_dv_4,W_dv_5,W_dv_6,b_1_scaled,b_2_scaled,b_3_scaled,b_4_scaled,delta_ailerons_scaled, ... */
  /*      dv_global_1,dv_global_2,dv_global_3,dv_global_4,dv_global_5,dv_global_6,flight_path_angle,g_1_scaled,g_2_scaled, ... */
  /*      g_3_scaled,g_4_scaled,gain_el,gain_az,gain_phi,gain_theta,gain_motor,gain_airspeed,gain_ailerons,l_1,l_3,l_4,l_z,m, ... */
  /*      p,prop_R,prop_Cd_0,prop_Cl_0,prop_Cd_a,prop_Cl_a,prop_delta,prop_sigma,prop_theta,q,r,rho,wing_span,wing_chord); */
  /*   */
  /*  [cost_secondary,gradient_secondary] = compute_secondary_cost_and_gradient(Omega_1_scaled,Omega_2_scaled,Omega_3_scaled, ... */
  /*      Omega_4_scaled,Phi_scaled,Theta_scaled,V,W_act_phi,W_act_theta,W_act_motor,W_act_tilt_el,W_act_tilt_az,W_act_ailerons, ... */
  /*      b_1_scaled,b_2_scaled,b_3_scaled,b_4_scaled,delta_ailerons_scaled,desired_el_value,desired_az_value,desired_phi_value, ... */
  /*      desired_theta_value,desired_motor_value,desired_ailerons_value,flight_path_angle,g_1_scaled,g_2_scaled,g_3_scaled, ... */
  /*      g_4_scaled,gain_el,gain_az,gain_phi,gain_theta,gain_motor,gain_ailerons,gamma_quadratic_du,power_Cd_0,power_Cd_a, ... */
  /*      prop_R,prop_Cl_0,prop_Cl_a,prop_delta,prop_sigma,prop_theta,rho); */
  *status = 1;
  b = rtIsNaN(fval);
  if (rtIsInf(fval) || b) {
    if (b) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }

  if (*status == 1) {
    *status = 1;
  }

  return fval;
}

static void factorQR(d_struct_T *obj, const double A[496], int mrows, int ncols)
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

static void factoryConstruct(c_struct_T *objfun_workspace, const double lb[15],
  const double ub[15], g_struct_T *obj)
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

static bool feasibleX0ForWorkingSet(double workspace[496], double xCurrent[16],
  const i_struct_T *workingset, d_struct_T *qrmanager)
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

static double fmincon(c_struct_T *fun_workspace, double x0[15], const double lb
                      [15], const double ub[15], double *exitflag, double
                      *output_iterations, double *output_funcCount, char
                      output_algorithm[3], double *output_constrviolation,
                      double *output_stepsize, double *output_lssteplength,
                      double *output_firstorderopt)
{
  b_struct_T MeritFunction;
  c_struct_T b_fun_workspace;
  c_struct_T c_FcnEvaluator_next_next_next_n;
  d_struct_T QRManager;
  e_struct_T CholManager;
  f_struct_T memspace;
  g_struct_T unusedExpr;
  h_struct_T TrialState;
  i_coder_internal_stickyStruct r;
  i_struct_T WorkingSet;
  struct_T QPObjective;
  double fval;
  int b_i;
  int i;
  int idx;
  int mUB;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  bool exitg1;
  *exitflag = rtInf;
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
    fval = rtInf;
    *output_iterations = 0.0;
    *output_funcCount = 0.0;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    *output_constrviolation = rtInf;
    *output_stepsize = rtInf;
    *output_lssteplength = rtInf;
    *output_firstorderopt = rtInf;
  } else {
    double scale;
    double y;
    int mFixed;
    int mLB;
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
    c_FcnEvaluator_next_next_next_n = *fun_workspace;
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
    mLB = 0;
    mUB = 0;
    mFixed = 0;
    for (idx = 0; idx < 15; idx++) {
      bool guard1;
      y = lb[idx];
      guard1 = false;
      if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
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

      if (guard1) {
        y = ub[idx];
        if ((!rtIsInf(y)) && (!rtIsNaN(y))) {
          mUB++;
          WorkingSet.indexUB[mUB - 1] = idx + 1;
        }
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

    TrialState.sqpFval = c_computeObjectiveAndUserGradie
      (&c_FcnEvaluator_next_next_next_n, TrialState.xstarsqp, TrialState.grad,
       &i);
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
    for (mUB = 0; mUB < b_i; mUB++) {
      WorkingSet.Wid[mUB] = 1;
      WorkingSet.Wlocalidx[mUB] = mUB + 1;
      WorkingSet.isActiveConstr[mUB] = true;
      i = mUB << 4;
      idx = WorkingSet.indexFixed[mUB];
      if (idx - 2 >= 0) {
        memset(&WorkingSet.ATwset[i], 0, (unsigned int)(((idx + i) - i) - 1) *
               sizeof(double));
      }

      WorkingSet.ATwset[(WorkingSet.indexFixed[mUB] + i) - 1] = 1.0;
      idx = WorkingSet.indexFixed[mUB] + 1;
      mLB = WorkingSet.nVar;
      if (idx <= mLB) {
        memset(&WorkingSet.ATwset[(idx + i) + -1], 0, (unsigned int)((((mLB + i)
                  - idx) - i) + 1) * sizeof(double));
      }

      WorkingSet.bwset[mUB] = WorkingSet.ub[WorkingSet.indexFixed[mUB] - 1];
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
    r.next.next.next.next.next.next.next.next.value.workspace =
      c_FcnEvaluator_next_next_next_n;
    b_driver(lb, ub, &TrialState, &MeritFunction, &r, &memspace, &WorkingSet,
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

static double ft_1(const double ct[262], double gradient_data[], int
                   *gradient_size)
{
  double b_t708_tmp;
  double b_t762_tmp;
  double b_t763_tmp;
  double b_t764_tmp;
  double b_t765_tmp;
  double b_t914_tmp;
  double cost;
  double t476;
  double t477;
  double t478;
  double t479;
  double t484;
  double t484_tmp;
  double t485;
  double t485_tmp;
  double t486;
  double t486_tmp;
  double t487;
  double t504;
  double t505;
  double t506;
  double t507;
  double t508;
  double t509;
  double t510;
  double t511;
  double t608;
  double t609;
  double t610;
  double t611;
  double t640;
  double t641;
  double t642;
  double t643;
  double t646;
  double t646_tmp;
  double t647;
  double t648;
  double t649;
  double t650;
  double t651;
  double t652;
  double t653;
  double t670;
  double t671;
  double t672;
  double t673;
  double t674;
  double t675;
  double t676;
  double t677;
  double t706;
  double t706_tmp_tmp;
  double t707;
  double t707_tmp;
  double t707_tmp_tmp;
  double t708;
  double t708_tmp;
  double t709;
  double t709_tmp;
  double t710;
  double t710_tmp;
  double t711;
  double t711_tmp;
  double t712;
  double t712_tmp;
  double t713;
  double t713_tmp;
  double t722;
  double t722_tmp;
  double t723;
  double t723_tmp;
  double t724;
  double t724_tmp;
  double t725;
  double t738;
  double t739;
  double t740;
  double t741;
  double t762;
  double t762_tmp;
  double t762_tmp_tmp;
  double t762_tmp_tmp_tmp;
  double t763;
  double t763_tmp;
  double t763_tmp_tmp;
  double t763_tmp_tmp_tmp;
  double t764;
  double t764_tmp;
  double t764_tmp_tmp;
  double t764_tmp_tmp_tmp;
  double t765;
  double t765_tmp;
  double t765_tmp_tmp;
  double t765_tmp_tmp_tmp;
  double t770;
  double t770_tmp;
  double t771;
  double t771_tmp;
  double t772;
  double t772_tmp;
  double t773;
  double t773_tmp;
  double t798;
  double t799;
  double t800;
  double t801;
  double t826;
  double t828;
  double t829;
  double t840;
  double t841;
  double t842;
  double t843;
  double t844;
  double t845;
  double t846;
  double t847;
  double t856;
  double t857;
  double t858;
  double t859;
  double t868;
  double t869;
  double t870;
  double t871;
  double t902;
  double t903;
  double t904;
  double t905;
  double t910;
  double t910_tmp;
  double t911;
  double t911_tmp;
  double t912;
  double t912_tmp;
  double t913;
  double t913_tmp;
  double t914;
  double t914_tmp;
  double t915;
  double t915_tmp;
  double t916;
  double t916_tmp;
  double t917;
  double t917_tmp;
  double t926;
  double t927;
  double t928;
  double t929;
  double t930;
  double t931;
  double t932;
  double t933;
  double t934;
  double t935;
  double t936;
  double t937;
  double t938;
  double t939;
  double t940;
  double t941;
  double t942;
  double t942_tmp;
  double t943;
  double t950;
  double t952;
  double t955;
  double t958;
  double t965;
  double t971;
  double t975;
  double t984;
  t798 = ct[191] + ct[195];
  t799 = ct[192] + ct[196];
  t800 = ct[193] + ct[197];
  t801 = ct[194] + ct[198];
  t608 = ct[147] + ct[165];
  t609 = ct[148] + ct[166];
  t610 = ct[149] + ct[167];
  t611 = ct[150] + ct[168];
  t640 = ct[156] + ct[46] * ct[173];
  t641 = ct[157] + ct[46] * ct[174];
  t642 = ct[158] + ct[46] * ct[175];
  t643 = ct[159] + ct[46] * ct[176];
  t670 = ct[123] + ct[46] * ct[221];
  t671 = ct[124] + ct[46] * ct[222];
  t672 = ct[125] + ct[46] * ct[223];
  t673 = ct[126] + ct[46] * ct[224];
  t674 = ct[127] + ct[46] * ct[225];
  t675 = ct[128] + ct[46] * ct[226];
  t676 = ct[129] + ct[46] * ct[227];
  t677 = ct[130] + ct[46] * ct[228];
  t476 = ct[99] + ct[46] * ((ct[50] + ct[89]) + ct[169]);
  t477 = ct[100] + ct[46] * ((ct[50] + ct[90]) + ct[170]);
  t478 = ct[101] + ct[46] * ((ct[50] + ct[91]) + ct[171]);
  t479 = ct[102] + ct[46] * ((ct[50] + ct[92]) + ct[172]);
  t646_tmp = ct[10] * ct[32] * ct[43] * ct[45] * ct[146];
  t646 = t646_tmp * ct[208] * ct[48] * ct[72] * t640 * 3.1415926535897931 / 4.0;
  t647 = t646_tmp * ct[209] * ct[48] * ct[73] * t641 * 3.1415926535897931 / 4.0;
  t648 = t646_tmp * ct[218] * ct[48] * ct[74] * t642 * 3.1415926535897931 / 4.0;
  t649 = t646_tmp * ct[219] * ct[48] * ct[75] * t643 * 3.1415926535897931 / 4.0;
  t706_tmp_tmp = ct[3] * ct[10];
  t984 = t706_tmp_tmp * ct[32] * ct[43] * ct[45] * ct[146];
  t943 = t984 * ct[229] * ct[235] * ct[48] * ct[72];
  t706 = t943 * t640 * 3.1415926535897931 * -0.25;
  t707_tmp_tmp = ct[4] * ct[10];
  t971 = t707_tmp_tmp * ct[32] * ct[43] * ct[45] * ct[146];
  t707_tmp = t971 * ct[231] * ct[236] * ct[48] * ct[73];
  t707 = t707_tmp * t641 * 3.1415926535897931 * -0.25;
  t955 = ct[5] * ct[10];
  t708_tmp = t955 * ct[32] * ct[43] * ct[45] * ct[146];
  b_t708_tmp = t708_tmp * ct[232] * ct[237] * ct[48] * ct[74];
  t708 = b_t708_tmp * t642 * 3.1415926535897931 * -0.25;
  t958 = ct[6] * ct[10];
  t826 = t958 * ct[32] * ct[43] * ct[45] * ct[146];
  t709_tmp = t826 * ct[234] * ct[238] * ct[48] * ct[75];
  t709 = t709_tmp * t643 * 3.1415926535897931 * -0.25;
  t710_tmp = t984 * ct[235] * ct[239] * ct[48] * ct[72];
  t710 = t710_tmp * t640 * 3.1415926535897931 * -0.25;
  t711_tmp = t971 * ct[236] * ct[240] * ct[48] * ct[73];
  t711 = t711_tmp * t641 * 3.1415926535897931 * -0.25;
  t712_tmp = t708_tmp * ct[237] * ct[241] * ct[48] * ct[74];
  t712 = t712_tmp * t642 * 3.1415926535897931 * -0.25;
  t713_tmp = t826 * ct[238] * ct[242] * ct[48] * ct[75];
  t713 = t713_tmp * t643 * 3.1415926535897931 * -0.25;
  t722_tmp = t706_tmp_tmp * ct[34] * ct[32] * ct[43] * ct[45] * ct[146];
  t722 = t722_tmp * ct[229] * ct[235] * ct[48] * ct[68] * t640 *
    3.1415926535897931 / 4.0;
  t723_tmp = t707_tmp_tmp * ct[34] * ct[32] * ct[43] * ct[45] * ct[146];
  t723 = t723_tmp * ct[231] * ct[236] * ct[48] * ct[69] * t641 *
    3.1415926535897931 / 4.0;
  t724_tmp = t955 * ct[34] * ct[32] * ct[43] * ct[45] * ct[146];
  t724 = t724_tmp * ct[232] * ct[237] * ct[48] * ct[70] * t642 *
    3.1415926535897931 / 4.0;
  t828 = t958 * ct[34] * ct[32] * ct[43] * ct[45] * ct[146];
  t725 = t828 * ct[234] * ct[238] * ct[48] * ct[71] * t643 * 3.1415926535897931 /
    4.0;
  t738 = t722_tmp * ct[235] * ct[239] * ct[48] * ct[68] * t640 *
    3.1415926535897931 / 4.0;
  t739 = t723_tmp * ct[236] * ct[240] * ct[48] * ct[69] * t641 *
    3.1415926535897931 / 4.0;
  t740 = t724_tmp * ct[237] * ct[241] * ct[48] * ct[70] * t642 *
    3.1415926535897931 / 4.0;
  t741 = t828 * ct[238] * ct[242] * ct[48] * ct[71] * t643 * 3.1415926535897931 /
    4.0;
  t762_tmp = ct[43] * ct[45];
  t762_tmp_tmp_tmp = t762_tmp * ct[81] * ct[143];
  t762_tmp_tmp = t762_tmp_tmp_tmp * ct[155];
  t950 = t762_tmp_tmp * ct[208];
  b_t762_tmp = t950 * ct[229] * ct[48];
  t762 = b_t762_tmp * t674 * 3.1415926535897931 / 4.0;
  t763_tmp_tmp_tmp = t762_tmp * ct[86] * ct[143];
  t763_tmp_tmp = t763_tmp_tmp_tmp * ct[155];
  t763_tmp = t763_tmp_tmp * ct[209];
  b_t763_tmp = t763_tmp * ct[231] * ct[48];
  t763 = b_t763_tmp * t675 * 3.1415926535897931 / 4.0;
  t764_tmp_tmp_tmp = t762_tmp * ct[93] * ct[143];
  t764_tmp_tmp = t764_tmp_tmp_tmp * ct[155];
  t764_tmp = t764_tmp_tmp * ct[218];
  b_t764_tmp = t764_tmp * ct[232] * ct[48];
  t764 = b_t764_tmp * t676 * 3.1415926535897931 / 4.0;
  t765_tmp_tmp_tmp = t762_tmp * ct[103] * ct[143];
  t765_tmp_tmp = t765_tmp_tmp_tmp * ct[155];
  t765_tmp = t765_tmp_tmp * ct[219];
  b_t765_tmp = t765_tmp * ct[234] * ct[48];
  t765 = b_t765_tmp * t677 * 3.1415926535897931 / 4.0;
  t770_tmp = t950 * ct[239] * ct[48];
  t770 = t770_tmp * t674 * 3.1415926535897931 / 4.0;
  t771_tmp = t763_tmp * ct[240] * ct[48];
  t771 = t771_tmp * t675 * 3.1415926535897931 / 4.0;
  t772_tmp = t764_tmp * ct[241] * ct[48];
  t772 = t772_tmp * t676 * 3.1415926535897931 / 4.0;
  t773_tmp = t765_tmp * ct[242] * ct[48];
  t773 = t773_tmp * t677 * 3.1415926535897931 / 4.0;
  t484_tmp = t762_tmp_tmp * ct[235] * ct[48];
  t484 = t484_tmp * t476 * 3.1415926535897931 / 4.0;
  t485_tmp = t763_tmp_tmp * ct[236] * ct[48];
  t485 = t485_tmp * t477 * 3.1415926535897931 / 4.0;
  t486_tmp = t764_tmp_tmp * ct[237] * ct[48];
  t486 = t486_tmp * t478 * 3.1415926535897931 / 4.0;
  t952 = t765_tmp_tmp * ct[238] * ct[48];
  t487 = t952 * t479 * 3.1415926535897931 / 4.0;
  t504 = b_t762_tmp * t476 * 3.1415926535897931 / 4.0;
  t505 = b_t763_tmp * t477 * 3.1415926535897931 / 4.0;
  t506 = b_t764_tmp * t478 * 3.1415926535897931 / 4.0;
  t507 = b_t765_tmp * t479 * 3.1415926535897931 / 4.0;
  t508 = t770_tmp * t476 * 3.1415926535897931 / 4.0;
  t509 = t771_tmp * t477 * 3.1415926535897931 / 4.0;
  t510 = t772_tmp * t478 * 3.1415926535897931 / 4.0;
  t511 = t773_tmp * t479 * 3.1415926535897931 / 4.0;
  t650 = ct[3] * t646;
  t651 = ct[4] * t647;
  t652 = ct[5] * t648;
  t653 = ct[6] * t649;
  t902 = ((ct[64] + ct[116]) + ct[160]) - ct[151] * t608;
  t903 = ((ct[64] + ct[117]) + ct[161]) - ct[152] * t609;
  t904 = ((ct[64] + ct[118]) + ct[162]) - ct[153] * t610;
  t905 = ((ct[64] + ct[119]) + ct[163]) - ct[154] * t611;
  t914_tmp = ct[44] * ct[46];
  t965 = ct[42] - ct[144];
  t764_tmp = t984 * ct[208] * ct[48] * ct[72];
  t984 = t764_tmp * 3.1415926535897931;
  b_t914_tmp = ct[214] + t914_tmp * ct[204] * t965;
  t914 = (t722_tmp * ct[208] * ct[48] * ct[68] * t640 * 3.1415926535897931 / 4.0
          + t484_tmp * t674 * 3.1415926535897931 / 4.0) + t984 * b_t914_tmp *
    -0.25;
  t765_tmp = t971 * ct[209] * ct[48] * ct[73];
  t971 = t765_tmp * 3.1415926535897931;
  t915_tmp = ct[215] + t914_tmp * ct[205] * t965;
  t915 = (t723_tmp * ct[209] * ct[48] * ct[69] * t641 * 3.1415926535897931 / 4.0
          + t485_tmp * t675 * 3.1415926535897931 / 4.0) + t971 * t915_tmp *
    -0.25;
  t762_tmp_tmp = t708_tmp * ct[218] * ct[48] * ct[74];
  t723_tmp = t762_tmp_tmp * 3.1415926535897931;
  t916_tmp = ct[216] + t914_tmp * ct[206] * t965;
  t916 = (t724_tmp * ct[218] * ct[48] * ct[70] * t642 * 3.1415926535897931 / 4.0
          + t486_tmp * t676 * 3.1415926535897931 / 4.0) + t723_tmp * t916_tmp *
    -0.25;
  t763_tmp_tmp = t826 * ct[219] * ct[48] * ct[75];
  t708_tmp = t763_tmp_tmp * 3.1415926535897931;
  t917_tmp = ct[217] + t914_tmp * ct[207] * t965;
  t917 = (t828 * ct[219] * ct[48] * ct[71] * t643 * 3.1415926535897931 / 4.0 +
          t952 * t677 * 3.1415926535897931 / 4.0) + t708_tmp * t917_tmp * -0.25;
  t826 = t484 + t650;
  t677 = t485 + t651;
  t828 = t486 + t652;
  t829 = t487 + t653;
  t840 = t504 + t706;
  t841 = t505 + t707;
  t842 = t506 + t708;
  t843 = t507 + t709;
  t844 = t508 + t710;
  t845 = t509 + t711;
  t846 = t510 + t712;
  t847 = t511 + t713;
  t763_tmp = ct[3] * ct[43] * ct[45] * ct[143] * ct[155];
  t926 = ((t763_tmp * ct[235] * ct[48] * ct[49] * t476 + ct[230]) + t646) -
    t764_tmp * t798 * 3.1415926535897931 / 4.0;
  t764_tmp_tmp = ct[4] * ct[43] * ct[45] * ct[143] * ct[155];
  t675 = ct[106] + ct[46] * (ct[96] - ct[188]);
  t927 = ((t764_tmp_tmp * ct[236] * ct[48] * ct[49] * t477 + t485_tmp *
           3.1415926535897931 * t675 * -0.25) + t647) - t765_tmp * t799 *
    3.1415926535897931 / 4.0;
  t765_tmp_tmp = ct[5] * ct[43] * ct[45] * ct[143] * ct[155];
  t722_tmp = ct[107] + ct[46] * (ct[97] - ct[189]);
  t928 = ((t765_tmp_tmp * ct[237] * ct[48] * ct[49] * t478 + t486_tmp *
           3.1415926535897931 * t722_tmp * -0.25) + t648) - t762_tmp_tmp * t800 *
    3.1415926535897931 / 4.0;
  t950 = ct[6] * ct[43] * ct[45] * ct[143] * ct[155];
  t674 = ct[108] + ct[46] * (ct[98] - ct[190]);
  t929 = ((t950 * ct[238] * ct[48] * ct[49] * t479 + t952 * 3.1415926535897931 *
           t674 * -0.25) + t649) - t763_tmp_tmp * t801 * 3.1415926535897931 /
    4.0;
  t763_tmp *= ct[208];
  t764_tmp = ct[105] + ct[46] * (ct[95] - ct[187]);
  t930 = ((t763_tmp * ct[229] * ct[48] * ct[49] * t476 + b_t762_tmp *
           3.1415926535897931 * t764_tmp * -0.25) - t646_tmp * ct[229] * ct[235]
          * ct[48] * ct[72] * t640 * 3.1415926535897931 / 4.0) + t943 * t798 *
    3.1415926535897931 / 4.0;
  t765_tmp = t764_tmp_tmp * ct[209];
  t931 = ((t765_tmp * ct[231] * ct[48] * ct[49] * t477 + b_t763_tmp *
           3.1415926535897931 * t675 * -0.25) - t646_tmp * ct[231] * ct[236] *
          ct[48] * ct[73] * t641 * 3.1415926535897931 / 4.0) + t707_tmp * t799 *
    3.1415926535897931 / 4.0;
  t762_tmp_tmp = t765_tmp_tmp * ct[218];
  t932 = ((t762_tmp_tmp * ct[232] * ct[48] * ct[49] * t478 + b_t764_tmp *
           3.1415926535897931 * t722_tmp * -0.25) - t646_tmp * ct[232] * ct[237]
          * ct[48] * ct[74] * t642 * 3.1415926535897931 / 4.0) + b_t708_tmp *
    t800 * 3.1415926535897931 / 4.0;
  t763_tmp_tmp = t950 * ct[219];
  t933 = ((t763_tmp_tmp * ct[234] * ct[48] * ct[49] * t479 + b_t765_tmp *
           3.1415926535897931 * t674 * -0.25) - t646_tmp * ct[234] * ct[238] *
          ct[48] * ct[75] * t643 * 3.1415926535897931 / 4.0) + t709_tmp * t801 *
    3.1415926535897931 / 4.0;
  t934 = ((t763_tmp * ct[239] * ct[48] * ct[49] * t476 + t770_tmp *
           3.1415926535897931 * t764_tmp * -0.25) - t646_tmp * ct[235] * ct[239]
          * ct[48] * ct[72] * t640 * 3.1415926535897931 / 4.0) + t710_tmp * t798
    * 3.1415926535897931 / 4.0;
  t935 = ((t765_tmp * ct[240] * ct[48] * ct[49] * t477 + t771_tmp *
           3.1415926535897931 * t675 * -0.25) - t646_tmp * ct[236] * ct[240] *
          ct[48] * ct[73] * t641 * 3.1415926535897931 / 4.0) + t711_tmp * t799 *
    3.1415926535897931 / 4.0;
  t936 = ((t762_tmp_tmp * ct[241] * ct[48] * ct[49] * t478 + t772_tmp *
           3.1415926535897931 * t722_tmp * -0.25) - t646_tmp * ct[237] * ct[241]
          * ct[48] * ct[74] * t642 * 3.1415926535897931 / 4.0) + t712_tmp * t800
    * 3.1415926535897931 / 4.0;
  t937 = ((t763_tmp_tmp * ct[242] * ct[48] * ct[49] * t479 + t773_tmp *
           3.1415926535897931 * t674 * -0.25) - t646_tmp * ct[238] * ct[242] *
          ct[48] * ct[75] * t643 * 3.1415926535897931 / 4.0) + t713_tmp * t801 *
    3.1415926535897931 / 4.0;
  t763_tmp_tmp = t706_tmp_tmp * ct[31] * ct[32] * ct[43] * ct[45] * ct[146];
  t763_tmp = ct[31] * ct[43] * ct[45];
  t676 = ct[210] + t914_tmp * ct[200] * t965;
  t724_tmp = t763_tmp_tmp * ct[235];
  t938 = (((t763_tmp * ct[81] * ct[143] * ct[155] * ct[208] * ct[48] * t476 *
            3.1415926535897931 / 4.0 + t763_tmp_tmp * ct[208] * ct[48] * ct[68] *
            t640 * 3.1415926535897931 / 4.0) - t724_tmp * ct[48] * ct[72] * t640
           * 3.1415926535897931 / 4.0) + t484_tmp * t670 * 3.1415926535897931 /
          4.0) + t984 * t676 * -0.25;
  t762_tmp_tmp = t707_tmp_tmp * ct[31] * ct[32] * ct[43] * ct[45] * ct[146];
  t722_tmp = ct[211] + t914_tmp * ct[201] * t965;
  t675 = t762_tmp_tmp * ct[236];
  t939 = (((t763_tmp * ct[86] * ct[143] * ct[155] * ct[209] * ct[48] * t477 *
            3.1415926535897931 / 4.0 + t762_tmp_tmp * ct[209] * ct[48] * ct[69] *
            t641 * 3.1415926535897931 / 4.0) - t675 * ct[48] * ct[73] * t641 *
           3.1415926535897931 / 4.0) + t485_tmp * t671 * 3.1415926535897931 /
          4.0) + t971 * t722_tmp * -0.25;
  t765_tmp = t955 * ct[31] * ct[32] * ct[43] * ct[45] * ct[146];
  t950 = ct[212] + t914_tmp * ct[202] * t965;
  t674 = t765_tmp * ct[237];
  t940 = (((t763_tmp * ct[93] * ct[143] * ct[155] * ct[218] * ct[48] * t478 *
            3.1415926535897931 / 4.0 + t765_tmp * ct[218] * ct[48] * ct[70] *
            t642 * 3.1415926535897931 / 4.0) - t674 * ct[48] * ct[74] * t642 *
           3.1415926535897931 / 4.0) + t486_tmp * t672 * 3.1415926535897931 /
          4.0) + t723_tmp * t950 * -0.25;
  t764_tmp = t958 * ct[31] * ct[32] * ct[43] * ct[45] * ct[146];
  t764_tmp_tmp = ct[213] + t914_tmp * ct[203] * t965;
  t765_tmp_tmp = t764_tmp * ct[238];
  t941 = (((t763_tmp * ct[103] * ct[143] * ct[155] * ct[219] * ct[48] * t479 *
            3.1415926535897931 / 4.0 + t764_tmp * ct[219] * ct[48] * ct[71] *
            t643 * 3.1415926535897931 / 4.0) - t765_tmp_tmp * ct[48] * ct[75] *
           t643 * 3.1415926535897931 / 4.0) + t952 * t673 * 3.1415926535897931 /
          4.0) + t708_tmp * t764_tmp_tmp * -0.25;
  t975 = ((t914 + t915) + t916) + t917;
  t856 = ct[30] * t508 + ct[30] * t710;
  t857 = ct[30] * t509 + ct[30] * t711;
  t858 = ct[30] * t510 + ct[30] * t712;
  t859 = ct[30] * t511 + ct[30] * t713;
  t868 = ct[30] * t504 + ct[30] * t706;
  t869 = ct[30] * t505 + ct[30] * t707;
  t870 = ct[30] * t506 + ct[30] * t708;
  t871 = ct[30] * t507 + ct[30] * t709;
  t910_tmp = t762_tmp * ct[84] * ct[143] * ct[164] * ct[49] * ct[65];
  t910 = ct[51] + t910_tmp * t902;
  t911_tmp = t762_tmp * ct[87] * ct[143] * ct[164] * ct[49] * ct[65];
  t911 = ct[51] + t911_tmp * t903;
  t912_tmp = t762_tmp * ct[94] * ct[143] * ct[164] * ct[49] * ct[65];
  t912 = ct[51] + t912_tmp * t904;
  t913_tmp = t762_tmp * ct[104] * ct[143] * ct[164] * ct[49] * ct[65];
  t913 = ct[51] + t913_tmp * t905;
  t763_tmp = ct[31] * ct[229];
  t942_tmp = t943 * 3.1415926535897931;
  t942 = (((t763_tmp * t484 + t763_tmp_tmp * ct[229] * ct[235] * ct[48] * ct[68]
            * t640 * 3.1415926535897931 / 4.0) + t763_tmp * t650) - b_t762_tmp *
          t670 * 3.1415926535897931 / 4.0) + t942_tmp * t676 * -0.25;
  t763_tmp = ct[31] * ct[231];
  t707_tmp *= 3.1415926535897931;
  t943 = (((t763_tmp * t485 + t762_tmp_tmp * ct[231] * ct[236] * ct[48] * ct[69]
            * t641 * 3.1415926535897931 / 4.0) + t763_tmp * t651) - b_t763_tmp *
          t671 * 3.1415926535897931 / 4.0) + t707_tmp * t722_tmp * -0.25;
  t763_tmp = ct[31] * ct[232];
  b_t762_tmp = b_t708_tmp * 3.1415926535897931;
  t709 = (((t763_tmp * t486 + t765_tmp * ct[232] * ct[237] * ct[48] * ct[70] *
            t642 * 3.1415926535897931 / 4.0) + t763_tmp * t652) - b_t764_tmp *
          t672 * 3.1415926535897931 / 4.0) + b_t762_tmp * t950 * -0.25;
  t763_tmp = ct[31] * ct[234];
  t762_tmp = t709_tmp * 3.1415926535897931;
  t708 = (((t763_tmp * t487 + t764_tmp * ct[234] * ct[238] * ct[48] * ct[71] *
            t643 * 3.1415926535897931 / 4.0) + t763_tmp * t653) - b_t765_tmp *
          t673 * 3.1415926535897931 / 4.0) + t762_tmp * t764_tmp_tmp * -0.25;
  t763_tmp = ct[31] * ct[239];
  t707 = t710_tmp * 3.1415926535897931;
  t713 = (((t763_tmp * t484 + t724_tmp * ct[239] * ct[48] * ct[68] * t640 *
            3.1415926535897931 / 4.0) + t763_tmp * t650) - t770_tmp * t670 *
          3.1415926535897931 / 4.0) + t707 * t676 * -0.25;
  t763_tmp = ct[31] * ct[240];
  t706 = t711_tmp * 3.1415926535897931;
  t711 = (((t763_tmp * t485 + t675 * ct[240] * ct[48] * ct[69] * t641 *
            3.1415926535897931 / 4.0) + t763_tmp * t651) - t771_tmp * t671 *
          3.1415926535897931 / 4.0) + t706 * t722_tmp * -0.25;
  t763_tmp = ct[31] * ct[241];
  t712 = t712_tmp * 3.1415926535897931;
  t710 = (((t763_tmp * t486 + t674 * ct[241] * ct[48] * ct[70] * t642 *
            3.1415926535897931 / 4.0) + t763_tmp * t652) - t772_tmp * t672 *
          3.1415926535897931 / 4.0) + t712 * t950 * -0.25;
  t763_tmp = ct[31] * ct[242];
  t505 = t713_tmp * 3.1415926535897931;
  t477 = (((t763_tmp * t487 + t765_tmp_tmp * ct[242] * ct[48] * ct[71] * t643 *
            3.1415926535897931 / 4.0) + t763_tmp * t653) - t773_tmp * t673 *
          3.1415926535897931 / 4.0) + t505 * t764_tmp_tmp * -0.25;
  t950 = ((t826 + t677) + t828) + t829;
  t479 = ((t844 + t845) + t846) + t847;
  t506 = ((t840 + t841) + t842) + t843;
  t507 = ct[220] * t950;
  t952 = ct[178] * t950;
  t955 = ct[181] * t506;
  t508 = ct[177] * t479;
  t509 = ct[177] * ct[220];
  t478 = t509 * t506;
  t510 = ct[178] * ct[181];
  t958 = t510 * t479;
  t511 = ct[181] * ct[220];
  t504 = -(t511 * t479);
  t965 = ct[24] + -ct[245] * (((((((((ct[179] - ct[180]) + ct[36] * t826) - ct
    [36] * t677) - ct[36] * t828) + ct[36] * t829) - ct[38] * t844) - ct[38] *
    t845) + ct[37] * t846) + ct[37] * t847);
  t800 = ct[23] + ct[244] * ((((((((((ct[52] + ct[184]) + ct[82]) + ct[39] *
    t826) + ct[39] * t677) + ct[39] * t828) + ct[39] * t829) + ct[38] * t840) +
    ct[38] * t841) - ct[37] * t842) - ct[37] * t843);
  t971 = ct[20] + ct[259] * (((ct[83] + ct[135]) + t955) + t508);
  t801 = ct[22] + ct[243] * (((((((((((ct[53] + ct[199]) + ct[66]) + ct[233]) +
    ct[39] * t844) + ct[39] * t845) + ct[39] * t846) + ct[39] * t847) + ct[36] *
    t840) + ct[36] * t843) - ct[36] * t841) - ct[36] * t842);
  t646_tmp = ct[177] * ct[178];
  t484_tmp = t646_tmp * t506;
  t984 = (ct[21] + ct[259] * (((((ct[85] + ct[137]) + ct[139]) + t507) + t958) -
           t484_tmp)) - 9.81;
  t674 = ct[19] - ct[259] * (((((ct[88] + ct[136]) + ct[138]) + t952) + t478) +
    t504);
  t706_tmp_tmp = ct[17] * ct[254];
  t763_tmp = ct[7] - t706_tmp_tmp;
  t485_tmp = ct[18] * ct[255];
  t764_tmp = ct[9] - t485_tmp;
  t476 = ct[16] * ct[258];
  t765_tmp = ct[15] - t476;
  t762_tmp_tmp = ct[11] + ct[59];
  t763_tmp_tmp = ct[12] + ct[59];
  t764_tmp_tmp = ct[13] + ct[59];
  t765_tmp_tmp = ct[14] + ct[59];
  t950 = ct[25] + ct[61];
  t723_tmp = ct[26] + ct[61];
  t708_tmp = ct[27] + ct[61];
  t676 = ct[28] + ct[61];
  t722_tmp = ct[35] * ct[112];
  t798 = ct[35] * ct[132];
  t799 = ct[35] * ct[134];
  t707_tmp_tmp = ct[35] * ct[111];
  t486_tmp = ct[35] * ct[110];
  t914_tmp = ct[35] * ct[140];
  cost = (((((((((((((((((((ct[131] * (t965 * t965) + ct[115] * (t971 * t971)) +
    ct[122] * (t800 * t800)) + ct[121] * (t801 * t801)) + ct[113] * (t674 * t674))
                        + ct[120] * (t984 * t984)) + t722_tmp * (t910 * t910)) +
                      t722_tmp * (t911 * t911)) + t722_tmp * (t912 * t912)) +
                    t722_tmp * (t913 * t913)) + t486_tmp * (t763_tmp * t763_tmp))
                  + t707_tmp_tmp * (t764_tmp * t764_tmp)) + t914_tmp * (t765_tmp
    * t765_tmp)) + t798 * (t762_tmp_tmp * t762_tmp_tmp)) + t798 * (t763_tmp_tmp *
    t763_tmp_tmp)) + t798 * (t764_tmp_tmp * t764_tmp_tmp)) + t798 *
             (t765_tmp_tmp * t765_tmp_tmp)) + t799 * (t950 * t950)) + t799 *
           (t723_tmp * t723_tmp)) + t799 * (t708_tmp * t708_tmp)) + t799 * (t676
    * t676);
  *gradient_size = 15;
  t828 = ct[41] * ct[145];
  t646 = ct[121] * ct[243] * t801;
  t647 = ct[8] * ct[45] * ct[109] * ct[261];
  t648 = ct[131] * ct[245] * t965;
  t649 = ct[122] * ct[244] * t800;
  t677 = ct[115] * ct[259] * t971;
  t826 = ct[120] * ct[259] * t984;
  t724_tmp = ct[113] * ct[259] * t674;
  gradient_data[0] = (((((t722_tmp * t910 * (t910_tmp * (((-ct[182] * t608 + ct
    [151] * (ct[40] * ct[182] * 6.0 - ct[42] * ct[182] * 6.0)) + t828 * ct[182] *
    3.0) + ct[109] * ct[247] * ct[256] * ct[47] * ct[48] * ct[63] * ct[77] * 6.0)
    - t762_tmp_tmp_tmp * ct[164] * ct[65] * t902 * 3.1415926535897931 * 1.5) *
    -2.0 + t646 * ((ct[36] * t930 + ct[39] * t934) + t647 * (((((ct[208] *
    0.01228634392023026 + ct[3] * ct[208] * 0.01509123041555583) + ct[57] * ct
    [235] * 0.0046429750925043979) - ct[54] * ct[238] * 0.0039349871274520724) -
    ct[57] * ct[238] * 0.0064381447596962606) + ct[109] * ct[208] * ct[257] *
    -0.020516396677824081) / 2.0) * 2.0) - t648 * (ct[36] * t926 - ct[38] * t934)
    * 2.0) + t649 * (ct[38] * t930 + ct[39] * t926) * 2.0) + t677 * (ct[177] *
    t934 + ct[181] * t930) * 2.0) + t826 * ((ct[220] * t926 - t646_tmp * t930) +
    t510 * t934) * 2.0) - t724_tmp * ((ct[178] * t926 + t509 * t930) - t511 *
    t934) * 2.0;
  gradient_data[1] = (((((t722_tmp * t911 * (t911_tmp * (((-ct[183] * t609 + ct
    [152] * (ct[40] * ct[183] * 6.0 - ct[42] * ct[183] * 6.0)) + t828 * ct[183] *
    3.0) + ct[109] * ct[249] * ct[256] * ct[47] * ct[48] * ct[63] * ct[78] * 6.0)
    - t763_tmp_tmp_tmp * ct[164] * ct[65] * t903 * 3.1415926535897931 * 1.5) *
    -2.0 - t646 * ((ct[36] * t931 - ct[39] * t935) + t647 * (((((ct[209] *
    0.01228634392023026 + ct[4] * ct[209] * 0.01509123041555583) + ct[56] * ct
    [236] * 0.0046429750925043979) - ct[55] * ct[237] * 0.0039349871274520724) -
    ct[56] * ct[237] * 0.0064381447596962606) + ct[109] * ct[209] * ct[257] *
    -0.020516396677824081) / 2.0) * 2.0) + t648 * (ct[36] * t927 + ct[38] * t935)
    * 2.0) + t649 * (ct[38] * t931 + ct[39] * t927) * 2.0) + t677 * (ct[177] *
    t935 + ct[181] * t931) * 2.0) + t826 * ((ct[220] * t927 - t646_tmp * t931) +
    t510 * t935) * 2.0) - t724_tmp * ((ct[178] * t927 + t509 * t931) - t511 *
    t935) * 2.0;
  gradient_data[2] = (((((t722_tmp * t912 * (t912_tmp * (((-ct[185] * t610 + ct
    [153] * (ct[40] * ct[185] * 6.0 - ct[42] * ct[185] * 6.0)) + t828 * ct[185] *
    3.0) + ct[109] * ct[251] * ct[256] * ct[47] * ct[48] * ct[63] * ct[79] * 6.0)
    - t764_tmp_tmp_tmp * ct[164] * ct[65] * t904 * 3.1415926535897931 * 1.5) *
    -2.0 + t648 * (ct[36] * t928 - ct[37] * t936) * 2.0) - t649 * (ct[37] * t932
    - ct[39] * t928) * 2.0) - t646 * (ct[36] * t932 - ct[39] * t936) * 2.0) +
                       t677 * (ct[177] * t936 + ct[181] * t932) * 2.0) + t826 *
                      ((ct[220] * t928 - t646_tmp * t932) + t510 * t936) * 2.0)
    - t724_tmp * ((ct[178] * t928 + t509 * t932) - t511 * t936) * 2.0;
  gradient_data[3] = (((((t722_tmp * t913 * (t913_tmp * (((-ct[186] * t611 + ct
    [154] * (ct[40] * ct[186] * 6.0 - ct[42] * ct[186] * 6.0)) + t828 * ct[186] *
    3.0) + ct[109] * ct[253] * ct[256] * ct[47] * ct[48] * ct[63] * ct[80] * 6.0)
    - t765_tmp_tmp_tmp * ct[164] * ct[65] * t905 * 3.1415926535897931 * 1.5) *
    -2.0 - t648 * (ct[36] * t929 + ct[37] * t937) * 2.0) - t649 * (ct[37] * t933
    - ct[39] * t929) * 2.0) + t646 * (ct[36] * t933 + ct[39] * t937) * 2.0) +
                       t677 * (ct[177] * t937 + ct[181] * t933) * 2.0) + t826 *
                      ((ct[220] * t929 - t646_tmp * t933) + t510 * t937) * 2.0)
    - t724_tmp * ((ct[178] * t929 + t509 * t933) - t511 * t937) * 2.0;
  t723_tmp = ct[3] * ct[31];
  t708_tmp = t723_tmp * ct[208];
  t676 = ct[35] * ct[43] * ct[45];
  t675 = ct[31] * ct[109];
  t971 = t676 * ct[84] * ct[112] * ct[143] * ct[164] * ct[65] * t910 *
    3.1415926535897931;
  gradient_data[4] = ((((((t798 * (ct[11] * 2.0 + ct[60]) - t646 * ((ct[36] *
    t942 + ct[39] * t713) - t647 * ((((t723_tmp * ct[235] * -0.01228634392023026
    - ct[31] * ct[81] * ct[235] * 0.0075456152077779167) + t708_tmp * ct[57] *
    0.0046429750925043979) + t723_tmp * ct[109] * ct[235] * ct[257] *
    0.020516396677824081) + t708_tmp * ct[235] * ct[238] * 0.0078699742549041447)
    / 2.0) * 2.0) - t648 * (ct[36] * t938 + ct[38] * t713) * 2.0) - t649 * (ct
    [38] * t942 - ct[39] * t938) * 2.0) - t677 * (ct[177] * t713 + ct[181] *
    t942) * 2.0) + t826 * ((ct[220] * t938 + t646_tmp * t942) - t510 * t713) *
                       2.0) - t724_tmp * ((ct[178] * t938 - t509 * t942) + t511 *
    t713) * 2.0) + t971 * (((ct[200] * t608 - ct[151] * (ct[40] * ct[200] * 6.0
    - ct[42] * ct[200] * 6.0)) - t828 * ct[200] * 3.0) + t675 * ct[246] * ct[256]
    * ct[47] * ct[48] * ct[63] * ct[68] * ct[72] * 6.0);
  t708_tmp = ct[4] * ct[31];
  t722_tmp = t708_tmp * ct[209];
  t674 = t676 * ct[87] * ct[112] * ct[143] * ct[164] * ct[65] * t911 *
    3.1415926535897931;
  gradient_data[5] = ((((((t798 * (ct[12] * 2.0 + ct[60]) - t646 * ((-ct[36] *
    t943 + ct[39] * t711) + t647 * ((((t708_tmp * ct[236] * -0.01228634392023026
    - ct[31] * ct[86] * ct[236] * 0.0075456152077779167) + t722_tmp * ct[56] *
    0.0046429750925043979) + t708_tmp * ct[109] * ct[236] * ct[257] *
    0.020516396677824081) + t722_tmp * ct[236] * ct[237] * 0.0078699742549041447)
    / 2.0) * 2.0) + t648 * (ct[36] * t939 - ct[38] * t711) * 2.0) - t649 * (ct
    [38] * t943 - ct[39] * t939) * 2.0) - t677 * (ct[177] * t711 + ct[181] *
    t943) * 2.0) + t826 * ((ct[220] * t939 + t646_tmp * t943) - t510 * t711) *
                       2.0) - t724_tmp * ((ct[178] * t939 - t509 * t943) + t511 *
    t711) * 2.0) + t674 * (((ct[201] * t609 - ct[152] * (ct[40] * ct[201] * 6.0
    - ct[42] * ct[201] * 6.0)) - t828 * ct[201] * 3.0) + t675 * ct[248] * ct[256]
    * ct[47] * ct[48] * ct[63] * ct[69] * ct[73] * 6.0);
  t722_tmp = t708_tmp * ct[218];
  t984 = t676 * ct[94] * ct[112] * ct[143] * ct[164] * ct[65] * t912 *
    3.1415926535897931;
  gradient_data[6] = (t798 * (ct[13] * 2.0 + ct[60]) + t646 * ((ct[36] * t709 -
    ct[39] * t710) + t647 * (((t708_tmp * rt_powd_snf(ct[218], 3.0) *
    0.0064381447596962606 - t722_tmp * (ct[237] * ct[237]) *
    0.012876289519392519) + t722_tmp * ct[55] * 0.0039349871274520724) +
    t722_tmp * ct[236] * ct[237] * 0.0092859501850087959) / 2.0) * 2.0) +
    (((((t648 * (ct[36] * t940 + ct[37] * t710) * 2.0 + t649 * (ct[37] * t709 +
          ct[39] * t940) * 2.0) - t677 * (ct[177] * t710 + ct[181] * t709) * 2.0)
       + t826 * ((ct[220] * t940 + t646_tmp * t709) - t510 * t710) * 2.0) -
      t724_tmp * ((ct[178] * t940 - t509 * t709) + t511 * t710) * 2.0) + t984 *
     (((ct[202] * t610 - ct[153] * (ct[40] * ct[202] * 6.0 - ct[42] * ct[202] *
         6.0)) - t828 * ct[202] * 3.0) + t675 * ct[250] * ct[256] * ct[47] * ct
      [48] * ct[63] * ct[70] * ct[74] * 6.0));
  t708_tmp = t723_tmp * ct[219];
  t676 = t676 * ct[104] * ct[112] * ct[143] * ct[164] * ct[65] * t913 *
    3.1415926535897931;
  gradient_data[7] = (t798 * (ct[14] * 2.0 + ct[60]) - t646 * ((ct[36] * t708 +
    ct[39] * t477) + t647 * (((t723_tmp * rt_powd_snf(ct[219], 3.0) *
    0.0064381447596962606 - t708_tmp * (ct[238] * ct[238]) *
    0.012876289519392519) + t708_tmp * ct[54] * 0.0039349871274520724) +
    t708_tmp * ct[235] * ct[238] * 0.0092859501850087959) / 2.0) * 2.0) +
    (((((t648 * (ct[36] * t941 - ct[37] * t477) * -2.0 + t649 * (ct[37] * t708 +
          ct[39] * t941) * 2.0) - t677 * (ct[177] * t477 + ct[181] * t708) * 2.0)
       + t826 * ((ct[220] * t941 + t646_tmp * t708) - t510 * t477) * 2.0) -
      t724_tmp * ((ct[178] * t941 - t509 * t708) + t511 * t477) * 2.0) + t676 *
     (((ct[203] * t611 - ct[154] * (ct[40] * ct[203] * 6.0 - ct[42] * ct[203] *
         6.0)) - t828 * ct[203] * 3.0) + t675 * ct[252] * ct[256] * ct[47] * ct
      [48] * ct[63] * ct[71] * ct[75] * 6.0));
  t647 = ct[38] * ct[122] * ct[244];
  t723_tmp = ct[38] * ct[131] * ct[245];
  gradient_data[8] = (((((t799 * (ct[25] * 2.0 + ct[62]) + t826 * (t646_tmp *
    t856 + t510 * t868) * 2.0) + t724_tmp * (t509 * t856 + t511 * t868) * 2.0) -
                        t646 * (ct[36] * t856 - ct[39] * t868) * 2.0) - t677 *
                       (ct[181] * t856 - ct[177] * t868) * 2.0) - t647 * t856 *
                      t800 * 2.0) + t723_tmp * t868 * t965 * 2.0;
  gradient_data[9] = (((((t799 * (ct[26] * 2.0 + ct[62]) + t826 * (t646_tmp *
    t857 + t510 * t869) * 2.0) + t724_tmp * (t509 * t857 + t511 * t869) * 2.0) +
                        t646 * (ct[36] * t857 + ct[39] * t869) * 2.0) - t677 *
                       (ct[181] * t857 - ct[177] * t869) * 2.0) - t647 * t857 *
                      t800 * 2.0) + t723_tmp * t869 * t965 * 2.0;
  t647 = ct[37] * ct[122] * ct[244];
  t723_tmp = ct[37] * ct[131] * ct[245];
  gradient_data[10] = (((((t799 * (ct[27] * 2.0 + ct[62]) + t826 * (t646_tmp *
    t858 + t510 * t870) * 2.0) + t724_tmp * (t509 * t858 + t511 * t870) * 2.0) +
    t646 * (ct[36] * t858 + ct[39] * t870) * 2.0) - t677 * (ct[181] * t858 - ct
    [177] * t870) * 2.0) + t647 * t858 * t800 * 2.0) - t723_tmp * t870 * t965 *
    2.0;
  gradient_data[11] = (((((t799 * (ct[28] * 2.0 + ct[62]) + t826 * (t646_tmp *
    t859 + t510 * t871) * 2.0) + t724_tmp * (t509 * t859 + t511 * t871) * 2.0) -
    t646 * (ct[36] * t859 - ct[39] * t871) * 2.0) - t677 * (ct[181] * t859 - ct
    [177] * t871) * 2.0) + t647 * t859 * t800 * 2.0) - t723_tmp * t871 * t965 *
    2.0;
  t647 = t707 * b_t914_tmp / 4.0;
  t723_tmp = t706 * t915_tmp / 4.0;
  t708_tmp = t712 * t916_tmp / 4.0;
  t675 = t505 * t917_tmp / 4.0;
  t722_tmp = t942_tmp * b_t914_tmp / 4.0;
  t765_tmp_tmp = t707_tmp * t915_tmp / 4.0;
  t764_tmp_tmp = b_t762_tmp * t916_tmp / 4.0;
  t950 = t762_tmp * t917_tmp / 4.0;
  t763_tmp = (-t738 + t770) + t647;
  t764_tmp = (-t739 + t771) + t723_tmp;
  t765_tmp = (-t740 + t772) + t708_tmp;
  t762_tmp_tmp = (-t741 + t773) + t675;
  t763_tmp_tmp = ((((((((((-t722 - t723) - t724) - t725) + t762) + t763) + t764)
                     + t765) + t722_tmp) + t765_tmp_tmp) + t764_tmp_tmp) + t950;
  t647 = ((((((((((-t738 - t739) - t740) - t741) + t770) + t771) + t772) + t773)
            + t647) + t723_tmp) + t708_tmp) + t675;
  t723_tmp = ct[2] * ct[8] * ct[34] * ct[45] * ct[114] * ct[76] * ct[109];
  t708_tmp = (-t722 + t762) + t722_tmp;
  t675 = (-t723 + t763) + t765_tmp_tmp;
  t722_tmp = (-t724 + t764) + t764_tmp_tmp;
  t765_tmp_tmp = (-t725 + t765) + t950;
  t764_tmp_tmp = ct[34] * ct[109];
  gradient_data[12] = ((((t707_tmp_tmp * (ct[9] * 2.0 - t485_tmp * 2.0) + t826 *
    ((((((((ct[34] * ct[136] + ct[34] * ct[138]) + ct[34] * t952) + ct[34] *
          t478) + ct[34] * t504) - ct[142] * ct[220]) + ct[220] * t975) -
      t646_tmp * t763_tmp_tmp) + (((t510 * t647 + t646_tmp * ct[141]) - ct[8] *
    ct[34] * ct[45] * ct[114] * ct[109] * ct[181] * ct[220] * ct[67] / 2.0) -
    t723_tmp * ct[178] * ct[181] * ct[58])) * 2.0) - t648 * (((((((ct[36] * t914
    - ct[36] * t915) - ct[36] * t916) + ct[36] * t917) - ct[38] * t763_tmp) -
    ct[38] * t764_tmp) + ct[37] * t765_tmp) + ct[37] * t762_tmp_tmp) * 2.0) +
                        t646 * (((((ct[36] * t708_tmp - ct[36] * t675) - ct[36] *
    t722_tmp) + ct[36] * t765_tmp_tmp) + ct[39] * t763_tmp) + ((ct[39] *
    t764_tmp + ct[39] * t765_tmp) + ct[39] * t762_tmp_tmp)) * 2.0) + ((t724_tmp *
    (((((((((ct[34] * ct[85] + ct[34] * ct[137]) + ct[34] * ct[139]) + ct[34] *
           t507) + ct[34] * t958) + ct[34] * -t484_tmp) + ct[142] * ct[178]) -
       ct[178] * t975) - t509 * t763_tmp_tmp) + ((t511 * t647 + t509 * ct[141])
    - t723_tmp * ct[181] * ct[220] * ct[58])) * 2.0 - t677 * ((ct[141] * ct[181]
    - ct[181] * t763_tmp_tmp) + (-ct[177] * t647 + t723_tmp * ct[177] * ct[58]))
    * 2.0) + t649 * ((((((((ct[39] * t914 + ct[39] * t915) + ct[39] * t916) +
    ct[39] * t917) + ct[38] * t708_tmp) + ct[38] * t675) - ct[37] * t722_tmp) -
                      ct[37] * t765_tmp_tmp) - ct[1] * ct[8] * ct[34] * ct[45] *
                     ct[109] * ct[260] / 2.0) * 2.0)) + (((t971 * (((ct[204] *
    t608 - ct[151] * (ct[40] * ct[204] * 6.0 - ct[42] * ct[204] * 6.0)) - t828 *
    ct[204] * 3.0) + t764_tmp_tmp * ct[246] * ct[256] * ct[47] * ct[48] * ct[63]
    * ct[68] * ct[72] * 6.0) + t674 * (((ct[205] * t609 - ct[152] * (ct[40] *
    ct[205] * 6.0 - ct[42] * ct[205] * 6.0)) - t828 * ct[205] * 3.0) +
    t764_tmp_tmp * ct[248] * ct[256] * ct[47] * ct[48] * ct[63] * ct[69] * ct[73]
    * 6.0)) + t984 * (((ct[206] * t610 - ct[153] * (ct[40] * ct[206] * 6.0 - ct
    [42] * ct[206] * 6.0)) - t828 * ct[206] * 3.0) + t764_tmp_tmp * ct[250] *
                      ct[256] * ct[47] * ct[48] * ct[63] * ct[70] * ct[74] * 6.0))
    + t676 * (((ct[207] * t611 - ct[154] * (ct[40] * ct[207] * 6.0 - ct[42] *
    ct[207] * 6.0)) - t828 * ct[207] * 3.0) + t764_tmp_tmp * ct[252] * ct[256] *
              ct[47] * ct[48] * ct[63] * ct[71] * ct[75] * 6.0));
  t828 = ct[33] * ct[177];
  t646 = ct[33] * ct[178];
  t647 = ct[33] * ct[220];
  gradient_data[13] = ((t486_tmp * (ct[7] * 2.0 - t706_tmp_tmp * 2.0) + t677 *
                        (((t828 * ct[133] + t828 * t506) - ct[33] * ct[181] *
    t479) - ct[8] * ct[33] * ct[45] * ct[114] * ct[109] * ct[181] * ct[67] / 2.0)
                        * 2.0) + t826 * (((t646 * ct[83] + t646 * ct[135]) +
    t646 * t955) + t646 * t508) * 2.0) + t724_tmp * (((t647 * ct[83] + t647 *
    ct[135]) + t647 * t955) + t647 * t508) * 2.0;
  gradient_data[14] = t914_tmp * (ct[15] * 2.0 - t476 * 2.0) - ct[0] * ct[8] *
    ct[29] * ct[45] * ct[109] * ct[121] * ct[243] * t801;
  return cost;
}

static void fullColLDL2_(e_struct_T *obj, int NColsRemain)
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

static void iterate(const double H[225], const double f[16], h_struct_T
                    *solution, f_struct_T *memspace, i_struct_T *workingset,
                    d_struct_T *qrmanager, e_struct_T *cholmanager, struct_T
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

                nActiveConstr = idxQR + div_nde_s32_floor(iyend - 1);
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

          if ((nVar >= 1) && (!(minLambda == 0.0))) {
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
            b_xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q,
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
          solution->maxConstr = b_maxConstraintViolation(workingset,
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
            minLambda = b_maxConstraintViolation(workingset, solution->searchDir);
            if (minLambda < solution->maxConstr) {
              if (i - 1 >= 0) {
                memcpy(&solution->xstar[0], &solution->searchDir[0], (unsigned
                        int)i * sizeof(double));
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

static double maxConstraintViolation(const i_struct_T *obj, const double x[496],
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

static double maximum(const double x[2])
{
  double ex;
  if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

static double minimum(const double x[2])
{
  double ex;
  if ((x[0] > x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

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

static void removeConstr(i_struct_T *obj, int idx_global)
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

static double rt_hypotd_snf(double u0, double u1)
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
  } else if (rtIsNaN(b)) {
    y = rtNaN;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static void setProblemType(i_struct_T *obj, int PROBLEM_TYPE)
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

static void solve(const e_struct_T *obj, double rhs[16])
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

static bool step(int *STEP_TYPE, double Hessian[225], const double lb[15], const
                 double ub[15], h_struct_T *TrialState, b_struct_T
                 *MeritFunction, f_struct_T *memspace, i_struct_T *WorkingSet,
                 d_struct_T *QRManager, e_struct_T *CholManager, struct_T
                 *QPObjective, j_struct_T *qpoptions)
{
  j_struct_T b_qpoptions;
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
            if (rtIsInf(ub[WorkingSet->indexLB[k] - 1])) {
              dv[WorkingSet->indexLB[k] - 1] = -s + fabs(s);
            } else {
              dv[WorkingSet->indexLB[k] - 1] = (WorkingSet->ub
                [WorkingSet->indexLB[k] - 1] - s) / 2.0;
            }
          }
        }

        idxEndIneq_tmp_tmp = (unsigned char)WorkingSet->sizes[4];
        for (k = 0; k < idxEndIneq_tmp_tmp; k++) {
          s = WorkingSet->ub[WorkingSet->indexUB[k] - 1];
          if (dv[WorkingSet->indexUB[k] - 1] > s) {
            if (rtIsInf(lb[WorkingSet->indexUB[k] - 1])) {
              dv[WorkingSet->indexUB[k] - 1] = s - fabs(s);
            } else {
              dv[WorkingSet->indexUB[k] - 1] = (s - WorkingSet->lb
                [WorkingSet->indexUB[k] - 1]) / 2.0;
            }
          }
        }

        memcpy(&TrialState->xstar[0], &dv[0], 16U * sizeof(double));
        idxEndIneq_tmp_tmp = WorkingSet->nVar;
        beta = 0.0;
        idxStartIneq = (unsigned char)WorkingSet->nVar;
        for (k = 0; k < idxStartIneq; k++) {
          beta += Hessian[k + 15 * k];
        }

        beta /= (double)WorkingSet->nVar;
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
        QPObjective->beta = beta;
        QPObjective->rho = s;
        QPObjective->hasLinear = true;
        QPObjective->objtype = 4;
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
          MeritFunction->phiPrimePlus = fmin((TrialState->fstar - s * 0.0) -
            beta / 2.0 * 0.0, 0.0);
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
        oldDirIdx = 0.0;
        s = 1.0;
        for (k = 0; k < 15; k++) {
          oldDirIdx = fmax(oldDirIdx, fabs(TrialState->grad[k]));
          s = fmax(s, fabs(TrialState->xstar[k]));
        }

        s = fmax(2.2204460492503131E-16, oldDirIdx / s);
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

static bool test_exit(b_struct_T *MeritFunction, const i_struct_T *WorkingSet,
                      h_struct_T *TrialState, const double lb[15], const double
                      ub[15], bool *Flags_fevalOK, bool *Flags_done, bool
                      *Flags_stepAccepted, bool *Flags_failedLineSearch, int
                      *Flags_stepType)
{
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

  s = fmax(1.0, fabs(TrialState->grad[idx_max - 1]));
  if (rtIsInf(s)) {
    s = 1.0;
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
    Flags_gradOK = ((!rtIsInf(TrialState->gradLag[k])) && (!rtIsNaN
      (TrialState->gradLag[k])));
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = fmax(smax, fabs(TrialState->gradLag[k]));
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

    if (isFeasible && (smax <= 1.0E-12 * s)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }

  return Flags_gradOK;
}

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

static void timeKeeper_init(void)
{
  savedTime_not_empty = false;
}

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

            i1 = (idxAjj + div_nde_s32_floor((iac - idxA1j) - 32) * 31) + 31;
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

        i = div_nde_s32_floor(iac - ic0);
        work[i] += c;
      }
    }

    if (!(-tau == 0.0)) {
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

void Nonlinear_controller_fcn_control_rf_aero_models(double m, double I_xx,
  double I_yy, double I_zz, double l_1, double l_2, double l_3, double l_4,
  double l_z, double Phi, double Theta, double Omega_1, double Omega_2, double
  Omega_3, double Omega_4, double b_1, double b_2, double b_3, double b_4,
  double g_1, double g_2, double g_3, double g_4, double delta_ailerons, double
  W_act_motor_const, double W_act_motor_speed, double W_act_tilt_el_const,
  double W_act_tilt_el_speed, double W_act_tilt_az_const, double
  W_act_tilt_az_speed, double W_act_theta_const, double W_act_theta_speed,
  double W_act_phi_const, double W_act_phi_speed, double W_act_ailerons_const,
  double W_act_ailerons_speed, double W_dv_1, double W_dv_2, double W_dv_3,
  double W_dv_4, double W_dv_5, double W_dv_6, double max_omega, double
  min_omega, double max_b, double min_b, double max_g, double min_g, double
  max_theta, double min_theta, double max_phi, double max_delta_ailerons, double
  min_delta_ailerons, const double dv[6], double p, double q, double r, double
  Cm_zero, double Cl_alpha, double Cd_zero, double K_Cd, double Cm_alpha, double
  CL_aileron, double rho, double V, double S, double wing_chord, double
  wing_span, double flight_path_angle, double max_alpha, double min_alpha,
  double Beta, double gamma_quadratic_du, double desired_motor_value, double
  desired_el_value, double desired_az_value, double desired_theta_value, double
  desired_phi_value, double desired_ailerons_value, double k_alt_tilt_constraint,
  double min_alt_tilt_constraint, double lidar_alt_corrected, double
  approach_mode, double verbose, double aoa_protection_speed, double
  transition_speed, double power_Cd_0, double power_Cd_a, double prop_Cl_0,
  double prop_Cl_a, double prop_Cd_0, double prop_Cd_a, double prop_Cm_0, double
  prop_Cm_a, double prop_sigma, double prop_c_tip, double prop_delta, double
  prop_theta, double prop_R, double gain_increase_AoA, double u_out[15], double
  residuals[6], double *elapsed_time, double *N_iterations, double *N_evaluation,
  double *exitflag)
{
  b_captured_var dv_global;
  c_struct_T b_expl_temp;
  c_struct_T expl_temp;
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
  captured_var b_S;
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
  captured_var b_l_3;
  captured_var b_l_4;
  captured_var b_l_z;
  captured_var b_m;
  captured_var b_p;
  captured_var b_power_Cd_0;
  captured_var b_power_Cd_a;
  captured_var b_prop_Cd_0;
  captured_var b_prop_Cd_a;
  captured_var b_prop_Cl_0;
  captured_var b_prop_Cl_a;
  captured_var b_prop_R;
  captured_var b_prop_delta;
  captured_var b_prop_sigma;
  captured_var b_prop_theta;
  captured_var b_q;
  captured_var b_r;
  captured_var b_rho;
  captured_var b_wing_chord;
  captured_var b_wing_span;
  captured_var gain_ailerons;
  captured_var gain_airspeed;
  captured_var gain_az;
  captured_var gain_el;
  captured_var gain_motor;
  captured_var gain_phi;
  captured_var gain_theta;
  double u_max[15];
  double u_min[15];
  double current_accelerations[6];
  double final_accelerations[6];
  double b_gain_airspeed;
  double b_gain_motor;
  double b_max_approach;
  double b_max_tilt_value_approach;
  double b_min_approach;
  double c_CL_aileron;
  double c_Cd_zero;
  double c_Cl_alpha;
  double c_Cm_alpha;
  double c_Cm_zero;
  double c_I_zz;
  double c_K_Cd;
  double c_S;
  double c_V;
  double c_flight_path_angle;
  double c_l_1;
  double c_l_3;
  double c_l_4;
  double c_l_z;
  double c_prop_Cd_0;
  double c_prop_Cd_a;
  double c_prop_Cl_0;
  double c_prop_Cl_a;
  double c_prop_R;
  double c_prop_delta;
  double c_prop_sigma;
  double c_prop_theta;
  double c_rho;
  double c_wing_chord;
  double c_wing_span;
  double g_max_approach;
  double g_min_approach;
  double max_theta_protection;
  double min_theta_protection;
  int i;
  char c_expl_temp[3];
  (void)l_2;
  (void)prop_Cm_0;
  (void)prop_Cm_a;
  (void)prop_c_tip;
  if (!isInitialized_Nonlinear_controller_fcn_control_rf_aero_models) {
    Nonlinear_controller_fcn_control_rf_aero_models_initialize();
  }

  b_m.contents = m;
  b_I_xx.contents = I_xx;
  b_I_yy.contents = I_yy;
  b_I_zz.contents = I_zz;
  b_l_1.contents = l_1;
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
  b_CL_aileron.contents = CL_aileron;
  b_rho.contents = rho;
  b_V.contents = V;
  b_S.contents = S;
  b_wing_chord.contents = wing_chord;
  b_wing_span.contents = wing_span;
  b_flight_path_angle.contents = flight_path_angle;
  b_Beta.contents = Beta;
  b_gamma_quadratic_du.contents = gamma_quadratic_du;
  b_desired_motor_value.contents = desired_motor_value;
  b_desired_el_value.contents = desired_el_value;
  b_desired_az_value.contents = desired_az_value;
  b_desired_theta_value.contents = desired_theta_value;
  b_desired_phi_value.contents = desired_phi_value;
  b_desired_ailerons_value.contents = desired_ailerons_value;
  b_power_Cd_0.contents = power_Cd_0;
  b_power_Cd_a.contents = power_Cd_a;
  b_prop_Cl_0.contents = prop_Cl_0;
  b_prop_Cl_a.contents = prop_Cl_a;
  b_prop_Cd_0.contents = prop_Cd_0;
  b_prop_Cd_a.contents = prop_Cd_a;
  b_prop_sigma.contents = prop_sigma;
  b_prop_delta.contents = prop_delta;
  b_prop_theta.contents = prop_theta;
  b_prop_R.contents = prop_R;

  /*  Create variables necessary for the optimization */
  b_min_approach = b_V.contents;
  b_V.contents = fmax(2.0, b_min_approach);

  /*  To prevent oscillations during hover */
  /*  gain_increase_AoA = 5.6568*pi/180; */
  min_alpha += gain_increase_AoA * fabs(sin(Phi));
  if (b_V.contents > aoa_protection_speed) {
    b_min_approach = (max_alpha + b_flight_path_angle.contents) * 180.0 /
      3.1415926535897931;
    max_theta_protection = fmin(max_theta, b_min_approach);
    b_min_approach = (min_alpha + b_flight_path_angle.contents) * 180.0 /
      3.1415926535897931;
    min_theta_protection = fmax(min_theta, b_min_approach);
  } else {
    max_theta_protection = max_theta;
    min_theta_protection = min_theta;
  }

  /*  max_theta_protection = max_theta; */
  /*  min_theta_protection = min_theta; */
  /* 15 m/s is the maximum airspeed at which the model provides valid results. */
  gain_motor.contents = max_omega / 2.0;
  gain_el.contents = (max_b - min_b) * 3.1415926535897931 / 180.0 / 2.0;
  gain_az.contents = (max_g - min_g) * 3.1415926535897931 / 180.0 / 2.0;
  gain_theta.contents = (max_theta_protection - min_theta_protection) *
    3.1415926535897931 / 180.0 / 2.0;
  gain_phi.contents = max_phi * 3.1415926535897931 / 180.0;
  gain_ailerons.contents = (max_delta_ailerons - min_delta_ailerons) *
    3.1415926535897931 / 180.0 / 2.0;
  gain_airspeed.contents = 15.0;
  u_out[4] = b_1;
  u_out[5] = b_2;
  u_out[6] = b_3;
  u_out[7] = b_4;
  u_out[8] = g_1;
  u_out[9] = g_2;
  u_out[10] = g_3;
  u_out[11] = g_4;
  u_out[12] = Theta;
  u_out[13] = Phi;
  u_out[14] = delta_ailerons;

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
    double max_tilt_value_approach[2];
    max_tilt_value_approach[0] = 0.0;
    max_tilt_value_approach[1] = k_alt_tilt_constraint * lidar_alt_corrected -
      min_alt_tilt_constraint * k_alt_tilt_constraint;
    b_max_tilt_value_approach = maximum(max_tilt_value_approach);

    /* Elevation angle */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_b;
    b_max_approach = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_b;
    b_min_approach = maximum(max_tilt_value_approach);

    /* Azimuth angle */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_g;
    g_max_approach = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_g;
    g_min_approach = maximum(max_tilt_value_approach);
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
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_theta_protection;
    u_max[12] = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = min_theta_protection;
    u_min[12] = maximum(max_tilt_value_approach);

    /* Roll angle  */
    max_tilt_value_approach[0] = b_max_tilt_value_approach;
    max_tilt_value_approach[1] = max_phi;
    u_max[13] = minimum(max_tilt_value_approach);
    max_tilt_value_approach[0] = -b_max_tilt_value_approach;
    max_tilt_value_approach[1] = -max_phi;
    u_min[13] = maximum(max_tilt_value_approach);
  }

  for (i = 0; i < 11; i++) {
    u_max[i + 4] = u_max[i + 4] * 3.1415926535897931 / 180.0;
    u_min[i + 4] = u_min[i + 4] * 3.1415926535897931 / 180.0;
  }

  u_max[0] /= gain_motor.contents;
  u_min[0] /= gain_motor.contents;
  u_max[4] /= gain_el.contents;
  u_min[4] /= gain_el.contents;
  u_max[8] /= gain_az.contents;
  u_min[8] /= gain_az.contents;
  u_max[1] /= gain_motor.contents;
  u_min[1] /= gain_motor.contents;
  u_max[5] /= gain_el.contents;
  u_min[5] /= gain_el.contents;
  u_max[9] /= gain_az.contents;
  u_min[9] /= gain_az.contents;
  u_max[2] /= gain_motor.contents;
  u_min[2] /= gain_motor.contents;
  u_max[6] /= gain_el.contents;
  u_min[6] /= gain_el.contents;
  u_max[10] /= gain_az.contents;
  u_min[10] /= gain_az.contents;
  u_max[3] /= gain_motor.contents;
  u_min[3] /= gain_motor.contents;
  u_max[7] /= gain_el.contents;
  u_min[7] /= gain_el.contents;
  u_max[11] /= gain_az.contents;
  u_min[11] /= gain_az.contents;
  u_max[12] /= gain_theta.contents;
  u_min[12] /= gain_theta.contents;
  u_max[13] /= gain_phi.contents;
  u_min[13] /= gain_phi.contents;
  u_max[14] /= gain_ailerons.contents;
  u_min[14] /= gain_ailerons.contents;
  u_out[0] = Omega_1 / gain_motor.contents;
  u_out[4] /= gain_el.contents;
  u_out[8] /= gain_az.contents;
  u_out[1] = Omega_2 / gain_motor.contents;
  u_out[5] /= gain_el.contents;
  u_out[9] /= gain_az.contents;
  u_out[2] = Omega_3 / gain_motor.contents;
  u_out[6] /= gain_el.contents;
  u_out[10] /= gain_az.contents;
  u_out[3] = Omega_4 / gain_motor.contents;
  u_out[7] /= gain_el.contents;
  u_out[11] /= gain_az.contents;
  u_out[12] /= gain_theta.contents;
  u_out[13] /= gain_phi.contents;
  u_out[14] /= gain_ailerons.contents;

  /*  Apply Nonlinear optimization algorithm: */
  b_max_tilt_value_approach = b_p.contents;
  b_max_approach = b_q.contents;
  b_min_approach = b_r.contents;
  g_max_approach = b_m.contents;
  g_min_approach = b_I_xx.contents;
  max_theta_protection = b_I_yy.contents;
  c_I_zz = b_I_zz.contents;
  c_l_1 = b_l_1.contents;
  c_l_3 = b_l_3.contents;
  c_l_4 = b_l_4.contents;
  c_l_z = b_l_z.contents;
  c_Cl_alpha = b_Cl_alpha.contents;
  c_Cd_zero = b_Cd_zero.contents;
  c_K_Cd = b_K_Cd.contents;
  c_Cm_alpha = b_Cm_alpha.contents;
  c_Cm_zero = b_Cm_zero.contents;
  c_CL_aileron = b_CL_aileron.contents;
  c_rho = b_rho.contents;
  c_V = b_V.contents;
  c_S = b_S.contents;
  c_wing_chord = b_wing_chord.contents;
  c_wing_span = b_wing_span.contents;
  c_flight_path_angle = b_flight_path_angle.contents;
  b_gain_motor = gain_motor.contents;
  b_gain_airspeed = gain_airspeed.contents;
  c_prop_Cl_0 = b_prop_Cl_0.contents;
  c_prop_Cl_a = b_prop_Cl_a.contents;
  c_prop_Cd_0 = b_prop_Cd_0.contents;
  c_prop_Cd_a = b_prop_Cd_a.contents;
  c_prop_sigma = b_prop_sigma.contents;
  c_prop_delta = b_prop_delta.contents;
  c_prop_theta = b_prop_theta.contents;
  c_prop_R = b_prop_R.contents;

  /*  Omega_3_scaled = Omega_3 / gain_motor; */
  /*  Omega_4_scaled = Omega_4 / gain_motor; */
  /*  b_1_scaled = b_1 / gain_el; */
  /*  b_2_scaled = b_2 / gain_el; */
  /*  b_3_scaled = b_3 / gain_el; */
  /*  b_4_scaled = b_4 / gain_el; */
  /*  g_1_scaled = g_1 / gain_az; */
  /*  g_2_scaled = g_2 / gain_az; */
  /*  g_3_scaled = g_3 / gain_az; */
  /*  g_4_scaled = g_4 / gain_az; */
  /*  Theta_scaled = Theta / gain_theta; */
  /*  Phi_scaled = Phi / gain_phi; */
  /*  gain_ailerons = 1; */
  /*  New aero models */
  compute_acc(c_CL_aileron, c_Cd_zero, c_Cl_alpha, c_Cm_zero, c_Cm_alpha,
              g_min_approach, max_theta_protection, c_I_zz, c_K_Cd, Omega_1,
              Omega_2, Omega_3, Omega_4, Omega_1 / b_gain_motor, Omega_2 /
              b_gain_motor, Phi, c_S, Theta, c_V, b_1, b_2, b_3, b_4,
              delta_ailerons, c_flight_path_angle, g_1, g_2, g_3, g_4,
              b_gain_airspeed, c_l_1, c_l_3, c_l_4, c_l_z, g_max_approach,
              b_max_tilt_value_approach, c_prop_R, c_prop_Cd_0, c_prop_Cl_0,
              c_prop_Cd_a, c_prop_Cl_a, c_prop_delta, c_prop_sigma, c_prop_theta,
              b_max_approach, b_min_approach, c_rho, c_wing_span, c_wing_chord,
              current_accelerations);
  for (i = 0; i < 6; i++) {
    dv_global.contents[i] = dv[i] + current_accelerations[i];
  }

  /* Pseudo-control hedging:  */
  if (b_V.contents > transition_speed) {
    b_max_approach = b_Cl_alpha.contents * min_alpha;
    b_max_tilt_value_approach = b_V.contents;
    b_max_approach = b_max_approach * 0.5 * b_rho.contents * b_S.contents *
      (b_max_tilt_value_approach * b_max_tilt_value_approach) * cos
      (min_theta_protection * 3.1415926535897931 / 180.0);
    b_max_approach = 9.81 - b_max_approach / b_m.contents;
    b_max_tilt_value_approach = 0.0;

    /*  min_vert_acc_abs = 5;  */
    /*  gain_remove_vert_acc = 8; */
    /*  max_vert_acc_fwd = max(min_vert_acc_abs,max_vert_acc_fwd - gain_remove_vert_acc*abs( sin(Phi) ) ); */
    if (dv_global.contents[2] >= b_max_approach) {
      b_max_tilt_value_approach = dv_global.contents[2] - b_max_approach;
    }

    dv_global.contents[2] -= b_max_tilt_value_approach;
  }

  /* Compute weights for actuators and make sure they are always positive */
  b_min_approach = W_act_motor_const + W_act_motor_speed * b_V.contents;
  W_act_motor.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_tilt_el_const + W_act_tilt_el_speed * b_V.contents;
  W_act_tilt_el.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_tilt_az_const + W_act_tilt_az_speed * b_V.contents;
  W_act_tilt_az.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_theta_const + W_act_theta_speed * b_V.contents;
  W_act_theta.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_phi_const + W_act_phi_speed * b_V.contents;
  W_act_phi.contents = fmax(0.0, b_min_approach);
  b_min_approach = W_act_ailerons_const + W_act_ailerons_speed * b_V.contents;
  W_act_ailerons.contents = fmax(0.0, b_min_approach);

  /* Default values for the optimizer: */
  tic();
  expl_temp.wing_chord = &b_wing_chord;
  expl_temp.wing_span = &b_wing_span;
  expl_temp.rho = &b_rho;
  expl_temp.r = &b_r;
  expl_temp.q = &b_q;
  expl_temp.prop_theta = &b_prop_theta;
  expl_temp.prop_sigma = &b_prop_sigma;
  expl_temp.prop_delta = &b_prop_delta;
  expl_temp.prop_Cl_a = &b_prop_Cl_a;
  expl_temp.prop_Cd_a = &b_prop_Cd_a;
  expl_temp.prop_Cl_0 = &b_prop_Cl_0;
  expl_temp.prop_Cd_0 = &b_prop_Cd_0;
  expl_temp.prop_R = &b_prop_R;
  expl_temp.power_Cd_a = &b_power_Cd_a;
  expl_temp.power_Cd_0 = &b_power_Cd_0;
  expl_temp.p = &b_p;
  expl_temp.m = &b_m;
  expl_temp.l_z = &b_l_z;
  expl_temp.l_4 = &b_l_4;
  expl_temp.l_3 = &b_l_3;
  expl_temp.l_1 = &b_l_1;
  expl_temp.gamma_quadratic_du = &b_gamma_quadratic_du;
  expl_temp.gain_ailerons = &gain_ailerons;
  expl_temp.gain_airspeed = &gain_airspeed;
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
  fmincon(&b_expl_temp, u_out, u_min, u_max, exitflag, N_iterations,
          N_evaluation, c_expl_temp, &b_max_approach, &b_min_approach,
          &g_max_approach, &g_min_approach);
  *elapsed_time = toc();
  b_min_approach = gain_motor.contents;
  b_max_tilt_value_approach = gain_el.contents;
  b_max_approach = gain_az.contents;
  u_out[0] *= b_min_approach;
  u_out[4] *= b_max_tilt_value_approach;
  u_out[8] *= b_max_approach;
  u_out[1] *= b_min_approach;
  u_out[5] *= b_max_tilt_value_approach;
  u_out[9] *= b_max_approach;
  u_out[2] *= b_min_approach;
  u_out[6] *= b_max_tilt_value_approach;
  u_out[10] *= b_max_approach;
  u_out[3] *= b_min_approach;
  u_out[7] *= b_max_tilt_value_approach;
  u_out[11] *= b_max_approach;
  u_out[12] *= gain_theta.contents;
  u_out[13] *= gain_phi.contents;
  u_out[14] *= gain_ailerons.contents;
  b_max_tilt_value_approach = b_p.contents;
  b_max_approach = b_q.contents;
  b_min_approach = b_r.contents;
  g_max_approach = b_m.contents;
  g_min_approach = b_I_xx.contents;
  max_theta_protection = b_I_yy.contents;
  c_I_zz = b_I_zz.contents;
  c_l_1 = b_l_1.contents;
  c_l_3 = b_l_3.contents;
  c_l_4 = b_l_4.contents;
  c_l_z = b_l_z.contents;
  c_Cl_alpha = b_Cl_alpha.contents;
  c_Cd_zero = b_Cd_zero.contents;
  c_K_Cd = b_K_Cd.contents;
  c_Cm_alpha = b_Cm_alpha.contents;
  c_Cm_zero = b_Cm_zero.contents;
  c_CL_aileron = b_CL_aileron.contents;
  c_rho = b_rho.contents;
  c_V = b_V.contents;
  c_S = b_S.contents;
  c_wing_chord = b_wing_chord.contents;
  c_wing_span = b_wing_span.contents;
  c_flight_path_angle = b_flight_path_angle.contents;
  b_gain_motor = gain_motor.contents;
  b_gain_airspeed = gain_airspeed.contents;
  c_prop_Cl_0 = b_prop_Cl_0.contents;
  c_prop_Cl_a = b_prop_Cl_a.contents;
  c_prop_Cd_0 = b_prop_Cd_0.contents;
  c_prop_Cd_a = b_prop_Cd_a.contents;
  c_prop_sigma = b_prop_sigma.contents;
  c_prop_delta = b_prop_delta.contents;
  c_prop_theta = b_prop_theta.contents;
  c_prop_R = b_prop_R.contents;

  /*  Omega_3_scaled = Omega_3 / gain_motor; */
  /*  Omega_4_scaled = Omega_4 / gain_motor; */
  /*  b_1_scaled = b_1 / gain_el; */
  /*  b_2_scaled = b_2 / gain_el; */
  /*  b_3_scaled = b_3 / gain_el; */
  /*  b_4_scaled = b_4 / gain_el; */
  /*  g_1_scaled = g_1 / gain_az; */
  /*  g_2_scaled = g_2 / gain_az; */
  /*  g_3_scaled = g_3 / gain_az; */
  /*  g_4_scaled = g_4 / gain_az; */
  /*  Theta_scaled = Theta / gain_theta; */
  /*  Phi_scaled = Phi / gain_phi; */
  /*  gain_ailerons = 1; */
  /*  New aero models */
  compute_acc(c_CL_aileron, c_Cd_zero, c_Cl_alpha, c_Cm_zero, c_Cm_alpha,
              g_min_approach, max_theta_protection, c_I_zz, c_K_Cd, u_out[0],
              u_out[1], u_out[2], u_out[3], u_out[0] / b_gain_motor, u_out[1] /
              b_gain_motor, u_out[13], c_S, u_out[12], c_V, u_out[4], u_out[5],
              u_out[6], u_out[7], u_out[14], c_flight_path_angle, u_out[8],
              u_out[9], u_out[10], u_out[11], b_gain_airspeed, c_l_1, c_l_3,
              c_l_4, c_l_z, g_max_approach, b_max_tilt_value_approach, c_prop_R,
              c_prop_Cd_0, c_prop_Cl_0, c_prop_Cd_a, c_prop_Cl_a, c_prop_delta,
              c_prop_sigma, c_prop_theta, b_max_approach, b_min_approach, c_rho,
              c_wing_span, c_wing_chord, final_accelerations);
  for (i = 0; i < 6; i++) {
    residuals[i] = dv_global.contents[i] - final_accelerations[i];
  }

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
    printf(" %f ", u_out[12] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Phi [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[13] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf(" Ailerons deflection [deg] =  ");
    fflush(stdout);
    printf(" %f ", u_out[14] * 180.0 / 3.1415926535897931);
    fflush(stdout);
    printf("\n");
    fflush(stdout);
    printf("\n Elapsed time = %f \n", *elapsed_time);
    fflush(stdout);
    printf("\n Number of iterations / evaluations = %f ", *N_iterations);
    fflush(stdout);
    printf("/ %f \n", *N_evaluation);
    fflush(stdout);
    memcpy(&u_max[0], &u_out[0], 15U * sizeof(double));
    u_max[0] = u_out[0] / gain_motor.contents;
    u_max[4] /= gain_el.contents;
    u_max[8] /= gain_az.contents;
    u_max[1] = u_out[1] / gain_motor.contents;
    u_max[5] /= gain_el.contents;
    u_max[9] /= gain_az.contents;
    u_max[2] = u_out[2] / gain_motor.contents;
    u_max[6] /= gain_el.contents;
    u_max[10] /= gain_az.contents;
    u_max[3] = u_out[3] / gain_motor.contents;
    u_max[7] /= gain_el.contents;
    u_max[11] /= gain_az.contents;
    u_max[12] /= gain_theta.contents;
    u_max[13] /= gain_phi.contents;
    u_max[14] /= gain_ailerons.contents;
    printf("\n Solution scaled norm = %f \n", b_norm(u_max));
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
      b_max_tilt_value_approach = dv_global.contents[i];
      printf(" %f ", b_max_tilt_value_approach);
      fflush(stdout);
    }

    printf("\n");
    fflush(stdout);
    printf("\n Achieved accelerations =   ");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      printf(" %f ", final_accelerations[i]);
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
    printf(" %f \n", c_norm(residuals));
    fflush(stdout);
  }
}

void Nonlinear_controller_fcn_control_rf_aero_models_initialize(void)
{
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  isInitialized_Nonlinear_controller_fcn_control_rf_aero_models = true;
}

void Nonlinear_controller_fcn_control_rf_aero_models_terminate(void)
{
  isInitialized_Nonlinear_controller_fcn_control_rf_aero_models = false;
}

/* End of code generation (Nonlinear_controller_fcn_control_rf_aero_models.c) */
