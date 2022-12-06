/*
 * darko.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "darko".
 *
 * Model version              : 1.6
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Tue Dec  6 10:09:29 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "darko.h"
#include "rtwtypes.h"
#include <string.h>
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include "darko_private.h"

/* Block states (default storage) */
DW_darko_T darko_DW;

/* External inputs (root inport signals with default storage) */
ExtU_darko_T darko_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_darko_T darko_Y;

/* Real-time model */
static RT_MODEL_darko_T darko_M_;
RT_MODEL_darko_T *const darko_M = &darko_M_;

/* Forward declaration for local functions */
static real_T darko_norm(const real_T x[4]);
static real_T darko_norm_j(const real_T x[3]);
static void darko_aero(const real_T x[13], const real_T T[3], real_T del, const
  real_T w[3], const real_T b_drone_PHI[36], const real_T
  b_drone_ELEVON_MEFFICIENCY[3], const real_T b_drone_ELEVON_FEFFICIENCY[3],
  real_T Fb[3], real_T Mb[3]);
static void darko_mldivide(const real_T A[9], const real_T B[3], real_T Y[3]);

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T darko_norm(const real_T x[4])
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
static real_T darko_norm_j(const real_T x[3])
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
static void darko_aero(const real_T x[13], const real_T T[3], real_T del, const
  real_T w[3], const real_T b_drone_PHI[36], const real_T
  b_drone_ELEVON_MEFFICIENCY[3], const real_T b_drone_ELEVON_FEFFICIENCY[3],
  real_T Fb[3], real_T Mb[3])
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  real_T B[9];
  real_T Fb_tmp[9];
  real_T b_a[9];
  real_T b_a_1[9];
  real_T c_a_0[9];
  real_T d_a[9];
  real_T j_a_1[9];
  real_T B_0[3];
  real_T Fb_tmp_0[3];
  real_T b_a_0[3];
  real_T b_a_2[3];
  real_T c_a[3];
  real_T j_a[3];
  real_T j_a_0[3];
  real_T j_a_2[3];
  real_T j_a_3[3];
  real_T k_a_tmp[3];
  real_T k_a_tmp_0[3];
  real_T k_a_tmp_1[3];
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
  int32_T Fb_tmp_tmp;
  int32_T b_a_tmp;
  int32_T i;
  int32_T i_0;
  qin_idx_3 = darko_norm(&x[6]);
  qin_idx_0 = x[6] / qin_idx_3;
  qin_idx_1 = x[7] / qin_idx_3;
  qin_idx_2 = x[8] / qin_idx_3;
  qin_idx_3 = x[9] / qin_idx_3;
  B_tmp_1 = qin_idx_0 * qin_idx_0;
  B_tmp_2 = qin_idx_1 * qin_idx_1;
  B_tmp_3 = qin_idx_2 * qin_idx_2;
  B_tmp_4 = qin_idx_3 * qin_idx_3;
  B[0] = ((B_tmp_1 + B_tmp_2) - B_tmp_3) - B_tmp_4;
  B_tmp = qin_idx_1 * qin_idx_2;
  B_tmp_0 = qin_idx_0 * qin_idx_3;
  B[3] = (B_tmp + B_tmp_0) * 2.0;
  B_tmp_5 = qin_idx_1 * qin_idx_3;
  B_tmp_6 = qin_idx_0 * qin_idx_2;
  B[6] = (B_tmp_5 - B_tmp_6) * 2.0;
  B[1] = (B_tmp - B_tmp_0) * 2.0;
  B_tmp_1 -= B_tmp_2;
  B[4] = (B_tmp_1 + B_tmp_3) - B_tmp_4;
  B_tmp_2 = qin_idx_2 * qin_idx_3;
  B_tmp = qin_idx_0 * qin_idx_1;
  B[7] = (B_tmp_2 + B_tmp) * 2.0;
  B[2] = (B_tmp_5 + B_tmp_6) * 2.0;
  B[5] = (B_tmp_2 - B_tmp) * 2.0;
  B[8] = (B_tmp_1 - B_tmp_3) + B_tmp_4;
  qin_idx_3 = x[0] - w[0];
  qin_idx_0 = x[1] - w[1];
  qin_idx_1 = x[2] - w[2];
  for (i = 0; i <= 0; i += 2) {
    tmp = _mm_loadu_pd(&B[i]);
    tmp_0 = _mm_loadu_pd(&B[i + 3]);
    tmp_1 = _mm_loadu_pd(&B[i + 6]);
    _mm_storeu_pd(&vinf[i], _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(qin_idx_1)),
      _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd(qin_idx_0)), _mm_add_pd
                 (_mm_mul_pd(tmp, _mm_set1_pd(qin_idx_3)), _mm_set1_pd(0.0)))));
  }

  for (i = 2; i < 3; i++) {
    vinf[i] = (B[i + 3] * qin_idx_0 + B[i] * qin_idx_3) + B[i + 6] * qin_idx_1;
  }

  memset(&B[0], 0, 9U * sizeof(real_T));
  B[0] = 0.55;
  B[4] = 0.13;
  B[8] = 0.55;
  qin_idx_3 = darko_norm_j(vinf);
  for (i_0 = 0; i_0 <= 0; i_0 += 2) {
    tmp = _mm_loadu_pd(&B[i_0]);
    tmp_0 = _mm_loadu_pd(&B[i_0 + 3]);
    tmp_1 = _mm_loadu_pd(&B[i_0 + 6]);
    _mm_storeu_pd(&k_a_tmp[i_0], _mm_mul_pd(_mm_set1_pd(del), _mm_loadu_pd
      (&b_drone_ELEVON_FEFFICIENCY[i_0])));
    _mm_storeu_pd(&B_0[i_0], _mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(x[5])),
      _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd(x[4])), _mm_add_pd(_mm_mul_pd(tmp,
      _mm_set1_pd(x[3])), _mm_set1_pd(0.0)))));
  }

  for (i_0 = 2; i_0 < 3; i_0++) {
    k_a_tmp[i_0] = del * b_drone_ELEVON_FEFFICIENCY[i_0];
    B_0[i_0] = (B[i_0 + 3] * x[4] + B[i_0] * x[3]) + B[i_0 + 6] * x[5];
  }

  qin_idx_0 = darko_norm_j(B_0);
  qin_idx_3 = sqrt(qin_idx_0 * qin_idx_0 * 0.0 + qin_idx_3 * qin_idx_3);
  qin_idx_1 = -0.045508750000000008 * qin_idx_3;
  qin_idx_0 = 0.045508750000000008 * qin_idx_3;
  for (i = 0; i < 3; i++) {
    j_a[i] = 0.0;
    b_a_0[i] = 0.0;
    for (Fb_tmp_tmp = 0; Fb_tmp_tmp < 3; Fb_tmp_tmp++) {
      Fb_tmp[Fb_tmp_tmp + 3 * i] = b_drone_PHI[6 * i + Fb_tmp_tmp] *
        0.75681358539058075;
      b_a_tmp = 3 * Fb_tmp_tmp + i;
      b_a[b_a_tmp] = 0.0;
      b_a[b_a_tmp] += b_drone_PHI[i + 3] * qin_idx_0 * B[3 * Fb_tmp_tmp];
      b_a[b_a_tmp] += b_drone_PHI[i + 9] * qin_idx_0 * B[3 * Fb_tmp_tmp + 1];
      b_a[b_a_tmp] += b_drone_PHI[i + 15] * qin_idx_0 * B[3 * Fb_tmp_tmp + 2];
      j_a[i] += b_drone_PHI[6 * Fb_tmp_tmp + i] * qin_idx_1 * vinf[Fb_tmp_tmp];
      b_a_0[i] += x[Fb_tmp_tmp + 3] * b_a[b_a_tmp];
    }
  }

  B_0[0] = k_a_tmp[1] * vinf[2] - vinf[1] * k_a_tmp[2];
  B_0[1] = vinf[0] * k_a_tmp[2] - k_a_tmp[0] * vinf[2];
  B_0[2] = k_a_tmp[0] * vinf[1] - vinf[0] * k_a_tmp[1];
  k_a_tmp_0[0] = k_a_tmp[1] * x[5] - k_a_tmp[2] * x[4];
  k_a_tmp_0[1] = k_a_tmp[2] * x[3] - k_a_tmp[0] * x[5];
  k_a_tmp_0[2] = k_a_tmp[0] * x[4] - k_a_tmp[1] * x[3];
  k_a_tmp_1[0] = k_a_tmp[1] * T[2] - T[1] * k_a_tmp[2];
  k_a_tmp_1[1] = T[0] * k_a_tmp[2] - k_a_tmp[0] * T[2];
  k_a_tmp_1[2] = k_a_tmp[0] * T[1] - T[0] * k_a_tmp[1];
  for (i_0 = 0; i_0 < 3; i_0++) {
    qin_idx_3 = 0.0;
    c_a[i_0] = 0.0;
    qin_idx_2 = 0.0;
    Fb_tmp_0[i_0] = 0.0;
    k_a_tmp[i_0] = del * b_drone_ELEVON_MEFFICIENCY[i_0];
    for (i = 0; i < 3; i++) {
      Fb_tmp_tmp = 3 * i + i_0;
      qin_idx_3 += Fb_tmp[Fb_tmp_tmp] * T[i];
      d_a[Fb_tmp_tmp] = 0.0;
      d_a[Fb_tmp_tmp] += b_drone_PHI[i_0 + 3] * qin_idx_0 * B[3 * i];
      d_a[Fb_tmp_tmp] += b_drone_PHI[i_0 + 9] * qin_idx_0 * B[3 * i + 1];
      d_a[Fb_tmp_tmp] += b_drone_PHI[i_0 + 15] * qin_idx_0 * B[3 * i + 2];
      qin_idx_2 += d_a[Fb_tmp_tmp] * k_a_tmp_0[i];
      c_a[i_0] += b_drone_PHI[6 * i + i_0] * qin_idx_0 * B_0[i];
      B_tmp_1 = Fb_tmp[Fb_tmp_tmp];
      Fb_tmp[Fb_tmp_tmp] = 0.0;
      j_a_1[Fb_tmp_tmp] = 0.0;
      b_a[Fb_tmp_tmp] = 0.0;
      B_tmp_2 = B[i_0];
      B_tmp_3 = b_drone_PHI[6 * i + 3];
      Fb_tmp[Fb_tmp_tmp] += 0.75681358539058075 * B_tmp_2 * B_tmp_3;
      j_a_1[Fb_tmp_tmp] += qin_idx_1 * B_tmp_2 * B_tmp_3;
      b_a_tmp = (i + 3) * 6;
      b_a[Fb_tmp_tmp] += b_drone_PHI[b_a_tmp + 3] * (qin_idx_0 * B_tmp_2);
      B_tmp_2 = B[i_0 + 3];
      B_tmp_3 = b_drone_PHI[6 * i + 4];
      Fb_tmp[Fb_tmp_tmp] += 0.75681358539058075 * B_tmp_2 * B_tmp_3;
      j_a_1[Fb_tmp_tmp] += qin_idx_1 * B_tmp_2 * B_tmp_3;
      b_a[Fb_tmp_tmp] += b_drone_PHI[b_a_tmp + 4] * (qin_idx_0 * B_tmp_2);
      B_tmp_2 = B[i_0 + 6];
      B_tmp_3 = b_drone_PHI[6 * i + 5];
      Fb_tmp[Fb_tmp_tmp] += 0.75681358539058075 * B_tmp_2 * B_tmp_3;
      j_a_1[Fb_tmp_tmp] += qin_idx_1 * B_tmp_2 * B_tmp_3;
      b_a[Fb_tmp_tmp] += b_drone_PHI[b_a_tmp + 5] * (qin_idx_0 * B_tmp_2);
      Fb_tmp_0[i_0] += B_tmp_1 * k_a_tmp_1[i];
    }

    j_a_0[i_0] = (((j_a[i_0] - b_a_0[i_0]) - qin_idx_3) + c_a[i_0]) + qin_idx_2;
    Fb[i_0] = j_a_0[i_0] + Fb_tmp_0[i_0];
    j_a_2[i_0] = 0.0;
    b_a_2[i_0] = 0.0;
    qin_idx_3 = 0.0;
    for (i = 0; i < 3; i++) {
      b_a_tmp = 3 * i + i_0;
      b_a_1[b_a_tmp] = 0.0;
      c_a_0[b_a_tmp] = 0.0;
      b_a_1[b_a_tmp] += B[3 * i] * b_a[i_0];
      c_a_0[b_a_tmp] += b_drone_PHI[6 * i + 3] * (qin_idx_0 * B[i_0]);
      b_a_1[b_a_tmp] += B[3 * i + 1] * b_a[i_0 + 3];
      c_a_0[b_a_tmp] += B[i_0 + 3] * qin_idx_0 * b_drone_PHI[6 * i + 4];
      b_a_1[b_a_tmp] += B[3 * i + 2] * b_a[i_0 + 6];
      c_a_0[b_a_tmp] += B[i_0 + 6] * qin_idx_0 * b_drone_PHI[6 * i + 5];
      qin_idx_3 += Fb_tmp[b_a_tmp] * T[i];
      j_a_2[i_0] += j_a_1[b_a_tmp] * vinf[i];
      b_a_2[i_0] += x[i + 3] * b_a_1[b_a_tmp];
    }

    j_a_3[i_0] = (j_a_2[i_0] - b_a_2[i_0]) - qin_idx_3;
  }

  B_0[0] = k_a_tmp[1] * vinf[2] - vinf[1] * k_a_tmp[2];
  B_0[1] = vinf[0] * k_a_tmp[2] - k_a_tmp[0] * vinf[2];
  B_0[2] = k_a_tmp[0] * vinf[1] - vinf[0] * k_a_tmp[1];
  k_a_tmp_0[0] = k_a_tmp[1] * x[5] - k_a_tmp[2] * x[4];
  k_a_tmp_0[1] = k_a_tmp[2] * x[3] - k_a_tmp[0] * x[5];
  k_a_tmp_0[2] = k_a_tmp[0] * x[4] - k_a_tmp[1] * x[3];
  qin_idx_1 = k_a_tmp[1] * T[2] - T[1] * k_a_tmp[2];
  qin_idx_2 = T[0] * k_a_tmp[2] - k_a_tmp[0] * T[2];
  B_tmp_1 = k_a_tmp[0] * T[1] - T[0] * k_a_tmp[1];
  for (i = 0; i < 3; i++) {
    c_a[i] = 0.0;
    for (Fb_tmp_tmp = 0; Fb_tmp_tmp < 3; Fb_tmp_tmp++) {
      i_0 = 3 * Fb_tmp_tmp + i;
      b_a[i_0] = 0.0;
      b_a_tmp = (Fb_tmp_tmp + 3) * 6;
      b_a[i_0] += b_drone_PHI[b_a_tmp + 3] * (qin_idx_0 * B[i]);
      b_a[i_0] += B[i + 3] * qin_idx_0 * b_drone_PHI[b_a_tmp + 4];
      b_a[i_0] += B[i + 6] * qin_idx_0 * b_drone_PHI[b_a_tmp + 5];
      c_a[i] += c_a_0[i_0] * B_0[Fb_tmp_tmp];
    }

    qin_idx_3 = 0.0;
    for (Fb_tmp_tmp = 0; Fb_tmp_tmp < 3; Fb_tmp_tmp++) {
      i_0 = 3 * Fb_tmp_tmp + i;
      d_a[i_0] = 0.0;
      d_a[i_0] += B[3 * Fb_tmp_tmp] * b_a[i];
      d_a[i_0] += B[3 * Fb_tmp_tmp + 1] * b_a[i + 3];
      d_a[i_0] += B[3 * Fb_tmp_tmp + 2] * b_a[i + 6];
      qin_idx_3 += d_a[i_0] * k_a_tmp_0[Fb_tmp_tmp];
    }

    Mb[i] = ((Fb_tmp[i + 3] * qin_idx_2 + Fb_tmp[i] * qin_idx_1) + Fb_tmp[i + 6]
             * B_tmp_1) + ((j_a_3[i] + c_a[i]) + qin_idx_3);
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void darko_mldivide(const real_T A[9], const real_T B[3], real_T Y[3])
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
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - b_A[r3 + 3] * Y[1];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B[r1] - b_A[r1 + 6] * Y[2];
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

  real_T tmp[13];
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
  darko_Y.p[0] = darko_DW.DiscreteTimeIntegrator_DSTATE[0];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  darko_Y.v[0] = darko_DW.DiscreteTimeIntegrator_DSTATE[3];

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  darko_Y.p[1] = darko_DW.DiscreteTimeIntegrator_DSTATE[1];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  darko_Y.v[1] = darko_DW.DiscreteTimeIntegrator_DSTATE[4];

  /* Outport: '<Root>/p' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  darko_Y.p[2] = darko_DW.DiscreteTimeIntegrator_DSTATE[2];

  /* Outport: '<Root>/v' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  darko_Y.v[2] = darko_DW.DiscreteTimeIntegrator_DSTATE[5];

  /* Outport: '<Root>/q' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  darko_Y.q[0] = darko_DW.DiscreteTimeIntegrator_DSTATE[6];
  darko_Y.q[1] = darko_DW.DiscreteTimeIntegrator_DSTATE[7];
  darko_Y.q[2] = darko_DW.DiscreteTimeIntegrator_DSTATE[8];
  darko_Y.q[3] = darko_DW.DiscreteTimeIntegrator_DSTATE[9];

  /* Outport: '<Root>/omega' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  darko_Y.omega[0] = darko_DW.DiscreteTimeIntegrator_DSTATE[10];
  darko_Y.omega[1] = darko_DW.DiscreteTimeIntegrator_DSTATE[11];
  darko_Y.omega[2] = darko_DW.DiscreteTimeIntegrator_DSTATE[12];

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/u'
   */
  rtb_u_idx_0 = -darko_U.u[2] * 1000.0;
  rtb_u_idx_1 = darko_U.u[2] * 1000.0;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  Inport: '<Root>/u'
   *  Inport: '<Root>/w'
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  b_tmp_2 = darko_norm(&darko_DW.DiscreteTimeIntegrator_DSTATE[6]);
  qin_idx_0 = darko_DW.DiscreteTimeIntegrator_DSTATE[6] / b_tmp_2;
  qin_idx_1 = darko_DW.DiscreteTimeIntegrator_DSTATE[7] / b_tmp_2;
  qin_idx_2 = darko_DW.DiscreteTimeIntegrator_DSTATE[8] / b_tmp_2;
  qin_idx_3 = darko_DW.DiscreteTimeIntegrator_DSTATE[9] / b_tmp_2;
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
  darko_aero(darko_DW.DiscreteTimeIntegrator_DSTATE, T1, darko_U.u[2] * 30.0 *
             0.017453292519943295, darko_U.w, d_PHI, d_ELEVON_MEFFICIENCY,
             d_ELEVON_FEFFICIENCY, A1, M1);
  darko_aero(darko_DW.DiscreteTimeIntegrator_DSTATE, T2, darko_U.u[3] * 30.0 *
             0.017453292519943295, darko_U.w, d_PHI, d_ELEVON_MEFFICIENCY,
             d_ELEVON_FEFFICIENCY, A2, M2);
  rtb_u_idx_0 = (rtb_u_idx_0 + darko_DW.DiscreteTimeIntegrator_DSTATE[10]) *
    5.1116E-6;
  b_tmp[0] = 0.0;
  b_tmp[1] = darko_DW.DiscreteTimeIntegrator_DSTATE[12];
  b_tmp[2] = -darko_DW.DiscreteTimeIntegrator_DSTATE[11];
  rtb_u_idx_1 = (rtb_u_idx_1 + darko_DW.DiscreteTimeIntegrator_DSTATE[10]) *
    5.1116E-6;
  for (i = 0; i <= 0; i += 2) {
    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    _mm_storeu_pd(&b_b[i], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&e[i + 6]),
      _mm_set1_pd(darko_DW.DiscreteTimeIntegrator_DSTATE[12])), _mm_add_pd
      (_mm_mul_pd(_mm_loadu_pd(&e[i + 3]), _mm_set1_pd
                  (darko_DW.DiscreteTimeIntegrator_DSTATE[11])), _mm_add_pd
       (_mm_mul_pd(_mm_loadu_pd(&e[i]), _mm_set1_pd
                   (darko_DW.DiscreteTimeIntegrator_DSTATE[10])), _mm_set1_pd
        (0.0)))));
  }

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   * */
  for (i = 2; i < 3; i++) {
    b_b[i] = (e_0[i + 3] * darko_DW.DiscreteTimeIntegrator_DSTATE[11] + e_0[i] *
              darko_DW.DiscreteTimeIntegrator_DSTATE[10]) + e_0[i + 6] *
      darko_DW.DiscreteTimeIntegrator_DSTATE[12];
  }

  qin_idx_2 = ((T1[0] + T2[0]) + A1[0]) + A2[0];
  qin_idx_3 = ((T1[1] + T2[1]) + A1[1]) + A2[1];
  b_tmp_3 = ((T1[2] + T2[2]) + A1[2]) + A2[2];
  b_b_0[0] = b_b[2] * darko_DW.DiscreteTimeIntegrator_DSTATE[11] - b_b[1] *
    darko_DW.DiscreteTimeIntegrator_DSTATE[12];
  b_b_0[1] = b_b[0] * darko_DW.DiscreteTimeIntegrator_DSTATE[12] - b_b[2] *
    darko_DW.DiscreteTimeIntegrator_DSTATE[10];
  b_b_0[2] = b_b[1] * darko_DW.DiscreteTimeIntegrator_DSTATE[10] - b_b[0] *
    darko_DW.DiscreteTimeIntegrator_DSTATE[11];
  darko_mldivide(f, b_b_0, b_b);
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

  darko_mldivide(e_0, M1_0, b_b_0);
  tmp[0] = 0.002 * darko_DW.DiscreteTimeIntegrator_DSTATE[3];
  tmp[3] = 0.002 * T2[0];
  tmp[1] = 0.002 * darko_DW.DiscreteTimeIntegrator_DSTATE[4];
  tmp[4] = 0.002 * T2[1];
  tmp[2] = 0.002 * darko_DW.DiscreteTimeIntegrator_DSTATE[5];
  tmp[5] = 0.002 * T2[2];
  tmp[6] = ((darko_DW.DiscreteTimeIntegrator_DSTATE[7] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[10] +
             darko_DW.DiscreteTimeIntegrator_DSTATE[8] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[11]) +
            darko_DW.DiscreteTimeIntegrator_DSTATE[9] *
            darko_DW.DiscreteTimeIntegrator_DSTATE[12]) * -0.5 * 0.002;
  tmp[7] = (darko_DW.DiscreteTimeIntegrator_DSTATE[6] *
            darko_DW.DiscreteTimeIntegrator_DSTATE[10] -
            (darko_DW.DiscreteTimeIntegrator_DSTATE[9] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[11] -
             darko_DW.DiscreteTimeIntegrator_DSTATE[8] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[12])) * 0.5 * 0.002;
  tmp[8] = (darko_DW.DiscreteTimeIntegrator_DSTATE[6] *
            darko_DW.DiscreteTimeIntegrator_DSTATE[11] -
            (darko_DW.DiscreteTimeIntegrator_DSTATE[7] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[12] -
             darko_DW.DiscreteTimeIntegrator_DSTATE[9] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[10])) * 0.5 * 0.002;
  tmp[9] = (darko_DW.DiscreteTimeIntegrator_DSTATE[6] *
            darko_DW.DiscreteTimeIntegrator_DSTATE[12] -
            (darko_DW.DiscreteTimeIntegrator_DSTATE[8] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[10] -
             darko_DW.DiscreteTimeIntegrator_DSTATE[7] *
             darko_DW.DiscreteTimeIntegrator_DSTATE[11])) * 0.5 * 0.002;
  tmp[10] = (b_b[0] + b_b_0[0]) * 0.002;
  tmp[11] = (b_b[1] + b_b_0[1]) * 0.002;
  tmp[12] = (b_b[2] + b_b_0[2]) * 0.002;
  for (i = 0; i <= 10; i += 2) {
    __m128d tmp_0;
    __m128d tmp_1;

    /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
    tmp_0 = _mm_loadu_pd(&darko_DW.DiscreteTimeIntegrator_DSTATE[i]);
    tmp_1 = _mm_loadu_pd(&tmp[i]);
    _mm_storeu_pd(&darko_DW.DiscreteTimeIntegrator_DSTATE[i], _mm_add_pd(tmp_0,
      tmp_1));
  }

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  for (i = 12; i < 13; i++) {
    darko_DW.DiscreteTimeIntegrator_DSTATE[i] += tmp[i];
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(darko_M->rtwLogInfo, (&darko_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.002s, 0.0s] */
    if ((rtmGetTFinal(darko_M)!=-1) &&
        !((rtmGetTFinal(darko_M)-darko_M->Timing.taskTime0) >
          darko_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(darko_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++darko_M->Timing.clockTick0)) {
    ++darko_M->Timing.clockTickH0;
  }

  darko_M->Timing.taskTime0 = darko_M->Timing.clockTick0 *
    darko_M->Timing.stepSize0 + darko_M->Timing.clockTickH0 *
    darko_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void darko_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)darko_M, 0,
                sizeof(RT_MODEL_darko_T));
  rtmSetTFinal(darko_M, 10.0);
  darko_M->Timing.stepSize0 = 0.002;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    darko_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(darko_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(darko_M->rtwLogInfo, (NULL));
    rtliSetLogT(darko_M->rtwLogInfo, "tout");
    rtliSetLogX(darko_M->rtwLogInfo, "");
    rtliSetLogXFinal(darko_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(darko_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(darko_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(darko_M->rtwLogInfo, 0);
    rtliSetLogDecimation(darko_M->rtwLogInfo, 1);
    rtliSetLogY(darko_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(darko_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(darko_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&darko_DW, 0,
                sizeof(DW_darko_T));

  /* external inputs */
  (void)memset(&darko_U, 0, sizeof(ExtU_darko_T));

  /* external outputs */
  (void)memset(&darko_Y, 0, sizeof(ExtY_darko_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(darko_M->rtwLogInfo, 0.0, rtmGetTFinal
    (darko_M), darko_M->Timing.stepSize0, (&rtmGetErrorStatus(darko_M)));

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  memcpy(&darko_DW.DiscreteTimeIntegrator_DSTATE[0],
         &darko_ConstP.DiscreteTimeIntegrator_IC[0], 13U * sizeof(real_T));
}

/* Model terminate function */
void darko_terminate(void)
{
  /* (no terminate code required) */
}
