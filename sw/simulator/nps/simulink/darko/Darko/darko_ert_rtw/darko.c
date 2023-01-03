/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: darko.c
 *
 * Code generated for Simulink model 'darko'.
 *
 * Model version                  : 1.10
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Mon Dec 12 08:31:49 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "darko.h"
#include "rtwtypes.h"
#include <emmintrin.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Forward declaration for local functions */
static real_T norm(const real_T x[4]);
static real_T norm_j(const real_T x[3]);
static void aero(const real_T x[13], const real_T T[3], real_T del, const real_T
                 w[3], const real_T b_drone_PHI[36], real_T b_drone_RHO, real_T
                 b_drone_WET_SURFACE, real_T b_drone_DRY_SURFACE, real_T
                 b_drone_PHI_n, real_T b_drone_CHORD, real_T b_drone_WINGSPAN,
                 real_T b_drone_PROP_RADIUS, const real_T
                 b_drone_ELEVON_MEFFICIENCY[3], const real_T
                 b_drone_ELEVON_FEFFICIENCY[3], real_T Fb[3], real_T Mb[3]);
static void mldivide(const real_T A[9], const real_T B_2[3], real_T Y[3]);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T norm(const real_T x[4])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T norm_j(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void aero(const real_T x[13], const real_T T[3], real_T del, const real_T
                 w[3], const real_T b_drone_PHI[36], real_T b_drone_RHO, real_T
                 b_drone_WET_SURFACE, real_T b_drone_DRY_SURFACE, real_T
                 b_drone_PHI_n, real_T b_drone_CHORD, real_T b_drone_WINGSPAN,
                 real_T b_drone_PROP_RADIUS, const real_T
                 b_drone_ELEVON_MEFFICIENCY[3], const real_T
                 b_drone_ELEVON_FEFFICIENCY[3], real_T Fb[3], real_T Mb[3])
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  real_T B_0[9];
  real_T S[9];
  real_T S_0[9];
  real_T Sp[9];
  real_T b_a[9];
  real_T b_a_1[9];
  real_T c_a[9];
  real_T d_a_0[9];
  real_T e_a[9];
  real_T B_1[3];
  real_T b_a_0[3];
  real_T d_a[3];
  real_T f_a[3];
  real_T v[3];
  real_T v_0[3];
  real_T v_1[3];
  real_T v_2[3];
  real_T vinf[3];
  real_T B_tmp;
  real_T B_tmp_0;
  real_T B_tmp_1;
  real_T B_tmp_2;
  real_T B_tmp_3;
  real_T B_tmp_4;
  real_T B_tmp_5;
  real_T B_tmp_6;
  real_T qin_idx_0;
  real_T qin_idx_1;
  real_T qin_idx_2;
  real_T qin_idx_3;
  int32_T b_a_tmp;
  int32_T i;
  int32_T i_0;
  qin_idx_3 = norm(&x[6]);
  qin_idx_0 = x[6] / qin_idx_3;
  qin_idx_1 = x[7] / qin_idx_3;
  qin_idx_2 = x[8] / qin_idx_3;
  qin_idx_3 = x[9] / qin_idx_3;
  B_tmp_1 = qin_idx_0 * qin_idx_0;
  B_tmp_2 = qin_idx_1 * qin_idx_1;
  B_tmp_3 = qin_idx_2 * qin_idx_2;
  B_tmp_4 = qin_idx_3 * qin_idx_3;
  B_0[0] = ((B_tmp_1 + B_tmp_2) - B_tmp_3) - B_tmp_4;
  B_tmp = qin_idx_1 * qin_idx_2;
  B_tmp_0 = qin_idx_0 * qin_idx_3;
  B_0[3] = (B_tmp + B_tmp_0) * 2.0;
  B_tmp_5 = qin_idx_1 * qin_idx_3;
  B_tmp_6 = qin_idx_0 * qin_idx_2;
  B_0[6] = (B_tmp_5 - B_tmp_6) * 2.0;
  B_0[1] = (B_tmp - B_tmp_0) * 2.0;
  B_tmp_1 -= B_tmp_2;
  B_0[4] = (B_tmp_1 + B_tmp_3) - B_tmp_4;
  B_tmp_2 = qin_idx_2 * qin_idx_3;
  B_tmp = qin_idx_0 * qin_idx_1;
  B_0[7] = (B_tmp_2 + B_tmp) * 2.0;
  B_0[2] = (B_tmp_5 + B_tmp_6) * 2.0;
  B_0[5] = (B_tmp_2 - B_tmp) * 2.0;
  B_0[8] = (B_tmp_1 - B_tmp_3) + B_tmp_4;
  qin_idx_3 = x[0] - w[0];
  qin_idx_0 = x[1] - w[1];
  qin_idx_1 = x[2] - w[2];
  for (i = 0; i <= 0; i += 2) {
    tmp = _mm_loadu_pd(&B_0[i]);
    tmp_0 = _mm_loadu_pd(&B_0[i + 3]);
    tmp_1 = _mm_loadu_pd(&B_0[i + 6]);
    _mm_storeu_pd(&vinf[i], _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(qin_idx_1)),
      _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd(qin_idx_0)), _mm_add_pd
                 (_mm_mul_pd(tmp, _mm_set1_pd(qin_idx_3)), _mm_set1_pd(0.0)))));
  }

  for (i = 2; i < 3; i++) {
    vinf[i] = (B_0[i + 3] * qin_idx_0 + B_0[i] * qin_idx_3) + B_0[i + 6] *
      qin_idx_1;
  }

  qin_idx_3 = b_drone_WET_SURFACE + b_drone_DRY_SURFACE;
  memset(&B_0[0], 0, 9U * sizeof(real_T));
  B_0[0] = b_drone_WINGSPAN;
  B_0[4] = b_drone_CHORD;
  B_0[8] = b_drone_WINGSPAN;
  qin_idx_2 = norm_j(vinf);
  qin_idx_0 = 0.5 * b_drone_WET_SURFACE / (b_drone_PROP_RADIUS *
    b_drone_PROP_RADIUS * 3.1415926535897931);
  for (i_0 = 0; i_0 <= 0; i_0 += 2) {
    tmp = _mm_loadu_pd(&B_0[i_0]);
    tmp_0 = _mm_loadu_pd(&B_0[i_0 + 3]);
    tmp_1 = _mm_loadu_pd(&B_0[i_0 + 6]);
    _mm_storeu_pd(&v[i_0], _mm_mul_pd(_mm_set1_pd(del), _mm_loadu_pd
      (&b_drone_ELEVON_FEFFICIENCY[i_0])));
    _mm_storeu_pd(&B_1[i_0], _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(x[5])),
      _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd(x[4])), _mm_add_pd(_mm_mul_pd(tmp,
      _mm_set1_pd(x[3])), _mm_set1_pd(0.0)))));
  }

  for (i_0 = 2; i_0 < 3; i_0++) {
    v[i_0] = del * b_drone_ELEVON_FEFFICIENCY[i_0];
    B_1[i_0] = (B_0[i_0 + 3] * x[4] + B_0[i_0] * x[3]) + B_0[i_0 + 6] * x[5];
  }

  qin_idx_1 = norm_j(B_1);
  qin_idx_2 = sqrt(qin_idx_1 * qin_idx_1 * b_drone_PHI_n + qin_idx_2 * qin_idx_2);
  qin_idx_1 = -0.5 * b_drone_RHO * qin_idx_3 * qin_idx_2;
  qin_idx_2 *= 0.5 * b_drone_RHO * qin_idx_3;
  v_0[0] = v[1] * vinf[2] - vinf[1] * v[2];
  v_0[1] = vinf[0] * v[2] - v[0] * vinf[2];
  v_0[2] = v[0] * vinf[1] - vinf[0] * v[1];
  v_1[0] = v[1] * x[5] - v[2] * x[4];
  v_1[1] = v[2] * x[3] - v[0] * x[5];
  v_1[2] = v[0] * x[4] - v[1] * x[3];
  v_2[0] = v[1] * T[2] - T[1] * v[2];
  v_2[1] = T[0] * v[2] - v[0] * T[2];
  v_2[2] = v[0] * T[1] - T[0] * v[1];
  for (i_0 = 0; i_0 < 3; i_0++) {
    B_1[i_0] = 0.0;
    b_a_0[i_0] = 0.0;
    qin_idx_3 = 0.0;
    d_a[i_0] = 0.0;
    B_tmp_1 = 0.0;
    f_a[i_0] = 0.0;
    for (i = 0; i < 3; i++) {
      b_a_tmp = 3 * i + i_0;
      b_a[b_a_tmp] = 0.0;
      B_tmp_2 = b_drone_PHI[i_0 + 3] * qin_idx_2 * B_0[3 * i];
      b_a[b_a_tmp] += B_tmp_2;
      B_tmp_3 = b_drone_PHI[i_0 + 9] * qin_idx_2 * B_0[3 * i + 1];
      b_a[b_a_tmp] += B_tmp_3;
      B_tmp_4 = b_drone_PHI[i_0 + 15] * qin_idx_2 * B_0[3 * i + 2];
      b_a[b_a_tmp] += B_tmp_4;
      B_tmp_6 = b_drone_PHI[6 * i + i_0];
      B_tmp = B_tmp_6 * qin_idx_0;
      qin_idx_3 += B_tmp * T[i];
      e_a[b_a_tmp] = 0.0;
      e_a[b_a_tmp] += B_tmp_2;
      e_a[b_a_tmp] += B_tmp_3;
      e_a[b_a_tmp] += B_tmp_4;
      B_tmp_1 += e_a[b_a_tmp] * v_1[i];
      B_1[i_0] += B_tmp_6 * qin_idx_1 * vinf[i];
      b_a_0[i_0] += x[i + 3] * b_a[b_a_tmp];
      d_a[i_0] += B_tmp_6 * qin_idx_2 * v_0[i];
      f_a[i_0] += B_tmp * v_2[i];
    }

    Fb[i_0] = ((((B_1[i_0] - b_a_0[i_0]) - qin_idx_3) + d_a[i_0]) + B_tmp_1) +
      f_a[i_0];
    v[i_0] = del * b_drone_ELEVON_MEFFICIENCY[i_0];
  }

  v_0[0] = v[1] * vinf[2] - vinf[1] * v[2];
  v_0[1] = vinf[0] * v[2] - v[0] * vinf[2];
  v_0[2] = v[0] * vinf[1] - vinf[0] * v[1];
  v_1[0] = v[1] * x[5] - v[2] * x[4];
  v_1[1] = v[2] * x[3] - v[0] * x[5];
  v_1[2] = v[0] * x[4] - v[1] * x[3];
  v_2[0] = v[1] * T[2] - T[1] * v[2];
  v_2[1] = T[0] * v[2] - v[0] * T[2];
  v_2[2] = v[0] * T[1] - T[0] * v[1];
  for (i = 0; i < 3; i++) {
    int32_T n_a_tmp;
    for (i_0 = 0; i_0 < 3; i_0++) {
      n_a_tmp = 3 * i_0 + i;
      e_a[n_a_tmp] = 0.0;
      b_a[n_a_tmp] = 0.0;
      B_tmp_1 = B_0[i];
      e_a[n_a_tmp] += b_drone_PHI[6 * i_0 + 3] * (qin_idx_1 * B_tmp_1);
      b_a_tmp = (i_0 + 3) * 6;
      b_a[n_a_tmp] += b_drone_PHI[b_a_tmp + 3] * (qin_idx_2 * B_tmp_1);
      B_tmp_1 = B_0[i + 3];
      e_a[n_a_tmp] += b_drone_PHI[6 * i_0 + 4] * (qin_idx_1 * B_tmp_1);
      b_a[n_a_tmp] += b_drone_PHI[b_a_tmp + 4] * (qin_idx_2 * B_tmp_1);
      B_tmp_1 = B_0[i + 6];
      e_a[n_a_tmp] += b_drone_PHI[6 * i_0 + 5] * (qin_idx_1 * B_tmp_1);
      b_a[n_a_tmp] += b_drone_PHI[b_a_tmp + 5] * (qin_idx_2 * B_tmp_1);
    }

    B_1[i] = 0.0;
    b_a_0[i] = 0.0;
    qin_idx_3 = 0.0;
    d_a[i] = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      b_a_tmp = 3 * i_0 + i;
      b_a_1[b_a_tmp] = 0.0;
      b_a_1[b_a_tmp] += B_0[3 * i_0] * b_a[i];
      b_a_1[b_a_tmp] += B_0[3 * i_0 + 1] * b_a[i + 3];
      b_a_1[b_a_tmp] += B_0[3 * i_0 + 2] * b_a[i + 6];
      c_a[b_a_tmp] = 0.0;
      d_a_0[b_a_tmp] = 0.0;
      B_tmp_1 = B_0[i];
      B_tmp_2 = b_drone_PHI[6 * i_0 + 3];
      c_a[b_a_tmp] += qin_idx_0 * B_tmp_1 * B_tmp_2;
      d_a_0[b_a_tmp] += qin_idx_2 * B_tmp_1 * B_tmp_2;
      B_tmp_1 = B_0[i + 3];
      B_tmp_2 = b_drone_PHI[6 * i_0 + 4];
      c_a[b_a_tmp] += qin_idx_0 * B_tmp_1 * B_tmp_2;
      d_a_0[b_a_tmp] += qin_idx_2 * B_tmp_1 * B_tmp_2;
      B_tmp_1 = B_0[i + 6];
      B_tmp_2 = b_drone_PHI[6 * i_0 + 5];
      c_a[b_a_tmp] += qin_idx_0 * B_tmp_1 * B_tmp_2;
      d_a_0[b_a_tmp] += qin_idx_2 * B_tmp_1 * B_tmp_2;
      qin_idx_3 += c_a[b_a_tmp] * T[i_0];
      S[b_a_tmp] = 0.0;
      n_a_tmp = (i_0 + 3) * 6;
      S[b_a_tmp] += b_drone_PHI[n_a_tmp + 3] * (qin_idx_2 * B_0[i]);
      S[b_a_tmp] += B_0[i + 3] * qin_idx_2 * b_drone_PHI[n_a_tmp + 4];
      S[b_a_tmp] += B_0[i + 6] * qin_idx_2 * b_drone_PHI[n_a_tmp + 5];
      B_1[i] += e_a[b_a_tmp] * vinf[i_0];
      b_a_0[i] += x[i_0 + 3] * b_a_1[b_a_tmp];
      d_a[i] += d_a_0[b_a_tmp] * v_0[i_0];
    }

    v[i] = (B_1[i] - b_a_0[i]) - qin_idx_3;
    qin_idx_3 = 0.0;
    f_a[i] = 0.0;
    for (i_0 = 0; i_0 < 3; i_0++) {
      n_a_tmp = 3 * i_0 + i;
      S_0[n_a_tmp] = 0.0;
      S_0[n_a_tmp] += B_0[3 * i_0] * S[i];
      S_0[n_a_tmp] += B_0[3 * i_0 + 1] * S[i + 3];
      S_0[n_a_tmp] += B_0[3 * i_0 + 2] * S[i + 6];
      Sp[n_a_tmp] = 0.0;
      Sp[n_a_tmp] += b_drone_PHI[6 * i_0 + 3] * (qin_idx_0 * B_0[i]);
      Sp[n_a_tmp] += B_0[i + 3] * qin_idx_0 * b_drone_PHI[6 * i_0 + 4];
      Sp[n_a_tmp] += B_0[i + 6] * qin_idx_0 * b_drone_PHI[6 * i_0 + 5];
      qin_idx_3 += S_0[n_a_tmp] * v_1[i_0];
      f_a[i] += Sp[n_a_tmp] * v_2[i_0];
    }

    Mb[i] = ((v[i] + d_a[i]) + qin_idx_3) + f_a[i];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void mldivide(const real_T A[9], const real_T B_2[3], real_T Y[3])
{
  real_T b_A[9];
  real_T a21;
  real_T maxval;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  memcpy(&b_A[0], &A[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(A[0]);
  a21 = fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r1 + 3] * b_A[r2];
  b_A[r3 + 3] -= b_A[r1 + 3] * b_A[r3];
  b_A[r2 + 6] -= b_A[r1 + 6] * b_A[r2];
  b_A[r3 + 6] -= b_A[r1 + 6] * b_A[r3];
  if (fabs(b_A[r3 + 3]) > fabs(b_A[r2 + 3])) {
    int32_T rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[1] = B_2[r2] - B_2[r1] * b_A[r2];
  Y[2] = (B_2[r3] - B_2[r1] * b_A[r3]) - b_A[r3 + 3] * Y[1];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B_2[r1] - b_A[r1 + 6] * Y[2];
  Y[1] -= b_A[r2 + 6] * Y[2];
  Y[1] /= b_A[r2 + 3];
  Y[0] -= b_A[r1 + 3] * Y[1];
  Y[0] /= b_A[r1];
}

/* Model step function */
void darko_step(void)
{
  real_T b[9];
  real_T A1[3];
  real_T A2[3];
  real_T M1[3];
  real_T M2[3];
  real_T T1[3];
  real_T T2[3];
  real_T b_b[3];
  real_T b_tmp[3];
  static const real_T e[9] = { 0.007018, 0.0, 0.0, 0.0, 0.002785, 0.0, 0.0, 0.0,
    0.00606 };

  real_T rtb_dxdt[13];
  real_T M1_0[3];
  real_T b_b_0[3];
  real_T b_tmp_0;
  real_T b_tmp_1;
  real_T b_tmp_2;
  real_T b_tmp_3;
  real_T b_tmp_4;
  real_T b_tmp_5;
  real_T b_tmp_6;
  real_T b_tmp_7;
  real_T qin_idx_0;
  real_T qin_idx_1;
  real_T qin_idx_2;
  real_T qin_idx_3;
  real_T rtb_u_idx_0;
  real_T rtb_u_idx_1;
  int32_T i;
  int8_T T1_tmp[3];
  static const real_T d_PHI[36] = { 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    0.0, 0.0, 0.0, -0.0023636363636363642, 0.0, 0.0, 3.938822097203273, 0.0,
    0.39388220972032734, 0.0, 0.0, 0.0, 0.0, 0.1396, 0.0, 0.0405, 0.0, 0.0,
    -0.0023636363636363642, 0.0, 0.63575, 0.0, 0.0, 0.39388220972032734, 0.0,
    0.05725, 0.0, 0.00195 };

  static const real_T d_ELEVON_MEFFICIENCY[3] = { 0.0, 0.93, 0.0 };

  static const real_T d_ELEVON_FEFFICIENCY[3] = { 0.0, 0.48, 0.0 };

  static const real_T e_0[9] = { 0.007018, 0.0, 0.0, 0.0, 0.002785, 0.0, 0.0,
    0.0, 0.00606 };

  static const real_T f[9] = { -0.007018, -0.0, -0.0, -0.0, -0.002785, -0.0,
    -0.0, -0.0, -0.00606 };

  static const real_T g[3] = { 0.0, 0.0, 9.81 };

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.p[0] = rtDW.DiscreteTimeIntegrator_DSTATE[0];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.v[0] = rtDW.DiscreteTimeIntegrator_DSTATE[3];

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.p[1] = rtDW.DiscreteTimeIntegrator_DSTATE[1];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.v[1] = rtDW.DiscreteTimeIntegrator_DSTATE[4];

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.p[2] = rtDW.DiscreteTimeIntegrator_DSTATE[2];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.v[2] = rtDW.DiscreteTimeIntegrator_DSTATE[5];

  /* Outport: '<Root>/q' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.q[0] = rtDW.DiscreteTimeIntegrator_DSTATE[6];
  rtY.q[1] = rtDW.DiscreteTimeIntegrator_DSTATE[7];
  rtY.q[2] = rtDW.DiscreteTimeIntegrator_DSTATE[8];
  rtY.q[3] = rtDW.DiscreteTimeIntegrator_DSTATE[9];

  /* Outport: '<Root>/omega' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.omega[0] = rtDW.DiscreteTimeIntegrator_DSTATE[10];
  rtY.omega[1] = rtDW.DiscreteTimeIntegrator_DSTATE[11];
  rtY.omega[2] = rtDW.DiscreteTimeIntegrator_DSTATE[12];

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/u'
   */
  rtb_u_idx_0 = -rtU.u[0] * 1000.0;
  rtb_u_idx_1 = rtU.u[1] * 1000.0;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  Inport: '<Root>/u'
   *  Inport: '<Root>/w'
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  b_tmp_2 = norm(&rtDW.DiscreteTimeIntegrator_DSTATE[6]);
  qin_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE[6] / b_tmp_2;
  qin_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE[7] / b_tmp_2;
  qin_idx_2 = rtDW.DiscreteTimeIntegrator_DSTATE[8] / b_tmp_2;
  qin_idx_3 = rtDW.DiscreteTimeIntegrator_DSTATE[9] / b_tmp_2;
  b_tmp_2 = qin_idx_0 * qin_idx_0;
  b_tmp_3 = qin_idx_1 * qin_idx_1;
  b_tmp_4 = qin_idx_2 * qin_idx_2;
  b_tmp_5 = qin_idx_3 * qin_idx_3;
  b[0] = ((b_tmp_2 + b_tmp_3) - b_tmp_4) - b_tmp_5;
  b_tmp_0 = qin_idx_1 * qin_idx_2;
  b_tmp_1 = qin_idx_0 * qin_idx_3;
  b[3] = (b_tmp_0 + b_tmp_1) * 2.0;
  b_tmp_6 = qin_idx_1 * qin_idx_3;
  b_tmp_7 = qin_idx_0 * qin_idx_2;
  b[6] = (b_tmp_6 - b_tmp_7) * 2.0;
  b[1] = (b_tmp_0 - b_tmp_1) * 2.0;
  b_tmp_2 -= b_tmp_3;
  b[4] = (b_tmp_2 + b_tmp_4) - b_tmp_5;
  b_tmp_3 = qin_idx_2 * qin_idx_3;
  b_tmp_0 = qin_idx_0 * qin_idx_1;
  b[7] = (b_tmp_3 + b_tmp_0) * 2.0;
  b[2] = (b_tmp_6 + b_tmp_7) * 2.0;
  b[5] = (b_tmp_3 - b_tmp_0) * 2.0;
  b[8] = (b_tmp_2 - b_tmp_4) + b_tmp_5;
  qin_idx_2 = rtb_u_idx_0 * rtb_u_idx_0;
  qin_idx_0 = qin_idx_2 * 5.13E-6;
  qin_idx_3 = rtb_u_idx_1 * rtb_u_idx_1;
  qin_idx_1 = qin_idx_3 * 5.13E-6;
  T1[0] = qin_idx_0;
  T2[0] = qin_idx_1;
  T1_tmp[0] = 1;
  T1[1] = qin_idx_0 * 0.0;
  T2[1] = qin_idx_1 * 0.0;
  T1_tmp[1] = 0;
  T1[2] = qin_idx_0 * 0.0;
  T2[2] = qin_idx_1 * 0.0;
  T1_tmp[2] = 0;
  if (rtIsNaN(-rtb_u_idx_0)) {
    b_tmp_2 = (rtNaN);
  } else if (-rtb_u_idx_0 < 0.0) {
    b_tmp_2 = -1.0;
  } else {
    b_tmp_2 = (-rtb_u_idx_0 > 0.0);
  }

  qin_idx_0 = b_tmp_2 * 2.64E-7 * qin_idx_2;
  if (rtIsNaN(-rtb_u_idx_1)) {
    b_tmp_2 = (rtNaN);
  } else if (-rtb_u_idx_1 < 0.0) {
    b_tmp_2 = -1.0;
  } else {
    b_tmp_2 = (-rtb_u_idx_1 > 0.0);
  }

  qin_idx_1 = b_tmp_2 * 2.64E-7 * qin_idx_3;
  aero(rtDW.DiscreteTimeIntegrator_DSTATE, T1, rtU.u[2] * 30.0 *
       0.017453292519943295, rtU.w, d_PHI, 1.225, 0.0743, 0.0, 0.0, 0.13, 0.55,
       0.125, d_ELEVON_MEFFICIENCY, d_ELEVON_FEFFICIENCY, A1, M1);
  aero(rtDW.DiscreteTimeIntegrator_DSTATE, T2, rtU.u[3] * 30.0 *
       0.017453292519943295, rtU.w, d_PHI, 1.225, 0.0743, 0.0, 0.0, 0.13, 0.55,
       0.125, d_ELEVON_MEFFICIENCY, d_ELEVON_FEFFICIENCY, A2, M2);
  rtb_u_idx_0 = (rtb_u_idx_0 + rtDW.DiscreteTimeIntegrator_DSTATE[10]) *
    5.1116E-6;
  b_tmp[0] = 0.0;
  b_tmp[1] = rtDW.DiscreteTimeIntegrator_DSTATE[12];
  b_tmp[2] = -rtDW.DiscreteTimeIntegrator_DSTATE[11];
  rtb_u_idx_1 = (rtb_u_idx_1 + rtDW.DiscreteTimeIntegrator_DSTATE[10]) *
    5.1116E-6;
  for (i = 0; i <= 0; i += 2) {
    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    _mm_storeu_pd(&b_b[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&e[i + 6]),
      _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[12])), _mm_add_pd
      (_mm_mul_pd(_mm_loadu_pd(&e[i + 3]), _mm_set1_pd
                  (rtDW.DiscreteTimeIntegrator_DSTATE[11])), _mm_add_pd
       (_mm_mul_pd(_mm_loadu_pd(&e[i]), _mm_set1_pd
                   (rtDW.DiscreteTimeIntegrator_DSTATE[10])), _mm_set1_pd(0.0)))));
  }

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  for (i = 2; i < 3; i++) {
    b_b[i] = (e_0[i + 3] * rtDW.DiscreteTimeIntegrator_DSTATE[11] + e_0[i] *
              rtDW.DiscreteTimeIntegrator_DSTATE[10]) + e_0[i + 6] *
      rtDW.DiscreteTimeIntegrator_DSTATE[12];
  }

  qin_idx_2 = ((T1[0] + T2[0]) + A1[0]) + A2[0];
  qin_idx_3 = ((T1[1] + T2[1]) + A1[1]) + A2[1];
  b_tmp_3 = ((T1[2] + T2[2]) + A1[2]) + A2[2];
  b_b_0[0] = b_b[2] * rtDW.DiscreteTimeIntegrator_DSTATE[11] - b_b[1] *
    rtDW.DiscreteTimeIntegrator_DSTATE[12];
  b_b_0[1] = b_b[0] * rtDW.DiscreteTimeIntegrator_DSTATE[12] - b_b[2] *
    rtDW.DiscreteTimeIntegrator_DSTATE[10];
  b_b_0[2] = b_b[1] * rtDW.DiscreteTimeIntegrator_DSTATE[10] - b_b[0] *
    rtDW.DiscreteTimeIntegrator_DSTATE[11];
  mldivide(f, b_b_0, b_b);
  b_b_0[0] = -0.155 * A1[2] - 0.0 * A1[1];
  b_b_0[1] = 0.0 * A1[0] - 0.0 * A1[2];
  b_b_0[2] = 0.0 * A1[1] - -0.155 * A1[0];
  A1[0] = 0.155 * A2[2] - 0.0 * A2[1];
  A1[1] = 0.0 * A2[0] - 0.0 * A2[2];
  A1[2] = 0.0 * A2[1] - 0.155 * A2[0];
  A2[0] = -0.155 * T1[2] - 0.0 * T1[1];
  A2[1] = 0.0 * T1[0] - 0.065 * T1[2];
  A2[2] = 0.065 * T1[1] - -0.155 * T1[0];
  T1[0] = 0.155 * T2[2] - 0.0 * T2[1];
  T1[1] = 0.0 * T2[0] - 0.065 * T2[2];
  T1[2] = 0.065 * T2[1] - 0.155 * T2[0];
  for (i = 0; i < 3; i++) {
    int32_T T1_tmp_0;
    b_tmp_2 = b_tmp[i];
    T1_tmp_0 = T1_tmp[i];
    T2[i] = ((b[3 * i + 1] * 2.0325203252032522 * qin_idx_3 + b[3 * i] *
              2.0325203252032522 * qin_idx_2) + b[3 * i + 2] *
             2.0325203252032522 * b_tmp_3) + g[i];
    M1_0[i] = ((((qin_idx_0 * (real_T)T1_tmp_0 - rtb_u_idx_0 * b_tmp_2) +
                 (qin_idx_1 * (real_T)T1_tmp_0 - rtb_u_idx_1 * b_tmp_2)) + A2[i])
               + T1[i]) + (((M1[i] + M2[i]) + b_b_0[i]) + A1[i]);
  }

  mldivide(e_0, M1_0, b_b_0);
  rtb_dxdt[0] = rtDW.DiscreteTimeIntegrator_DSTATE[3];
  rtb_dxdt[3] = T2[0];
  rtb_dxdt[1] = rtDW.DiscreteTimeIntegrator_DSTATE[4];
  rtb_dxdt[4] = T2[1];
  rtb_dxdt[2] = rtDW.DiscreteTimeIntegrator_DSTATE[5];
  rtb_dxdt[5] = T2[2];
  rtb_dxdt[6] = ((rtDW.DiscreteTimeIntegrator_DSTATE[7] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[10] +
                  rtDW.DiscreteTimeIntegrator_DSTATE[8] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[11]) +
                 rtDW.DiscreteTimeIntegrator_DSTATE[9] *
                 rtDW.DiscreteTimeIntegrator_DSTATE[12]) * -0.5;
  rtb_dxdt[7] = (rtDW.DiscreteTimeIntegrator_DSTATE[6] *
                 rtDW.DiscreteTimeIntegrator_DSTATE[10] -
                 (rtDW.DiscreteTimeIntegrator_DSTATE[9] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[11] -
                  rtDW.DiscreteTimeIntegrator_DSTATE[8] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[12])) * 0.5;
  rtb_dxdt[8] = (rtDW.DiscreteTimeIntegrator_DSTATE[6] *
                 rtDW.DiscreteTimeIntegrator_DSTATE[11] -
                 (rtDW.DiscreteTimeIntegrator_DSTATE[7] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[12] -
                  rtDW.DiscreteTimeIntegrator_DSTATE[9] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[10])) * 0.5;
  rtb_dxdt[9] = (rtDW.DiscreteTimeIntegrator_DSTATE[6] *
                 rtDW.DiscreteTimeIntegrator_DSTATE[12] -
                 (rtDW.DiscreteTimeIntegrator_DSTATE[8] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[10] -
                  rtDW.DiscreteTimeIntegrator_DSTATE[7] *
                  rtDW.DiscreteTimeIntegrator_DSTATE[11])) * 0.5;
  rtb_dxdt[10] = b_b[0] + b_b_0[0];

  /* Outport: '<Root>/accel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.accel[0] = T2[0];

  /* Outport: '<Root>/rotaccel' */
  rtY.rotaccel[0] = rtb_dxdt[10];

  /* MATLAB Function: '<Root>/MATLAB Function' */
  rtb_dxdt[11] = b_b[1] + b_b_0[1];

  /* Outport: '<Root>/accel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.accel[1] = T2[1];

  /* Outport: '<Root>/rotaccel' */
  rtY.rotaccel[1] = rtb_dxdt[11];

  /* MATLAB Function: '<Root>/MATLAB Function' */
  rtb_dxdt[12] = b_b[2] + b_b_0[2];

  /* Outport: '<Root>/accel' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtY.accel[2] = T2[2];

  /* Outport: '<Root>/rotaccel' */
  rtY.rotaccel[2] = rtb_dxdt[12];
  for (i = 0; i <= 10; i += 2) {
    __m128d tmp;
    __m128d tmp_0;

    /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
    tmp = _mm_loadu_pd(&rtb_dxdt[i]);
    tmp_0 = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i]);
    _mm_storeu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i], _mm_add_pd(_mm_mul_pd
      (_mm_set1_pd(0.000977), tmp), tmp_0));
  }

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  for (i = 12; i < 13; i++) {
    rtDW.DiscreteTimeIntegrator_DSTATE[i] += 0.000977 * rtb_dxdt[i];
  }
}

/* Model initialize function */
void darko_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  memcpy(&rtDW.DiscreteTimeIntegrator_DSTATE[0],
         &rtConstP.DiscreteTimeIntegrator_IC[0], 13U * sizeof(real_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
