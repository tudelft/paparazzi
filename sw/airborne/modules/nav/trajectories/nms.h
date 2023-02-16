/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'project'                       *
 ******************************************************************************/

#ifndef PROJECT__MIN_SNAP__H
#define PROJECT__MIN_SNAP__H

double get_x1(double alpha, double t);
double get_y1(double alpha, double t);
double get_z1(double alpha, double t);
double get_psi1(double alpha, double t);
double get_vx1(double alpha, double t);
double get_vy1(double alpha, double t);
double get_vz1(double alpha, double t);
double get_ax1(double alpha, double t);
double get_ay1(double alpha, double t);
double get_az1(double alpha, double t);

double get_x2(double alpha, double t);
double get_y2(double alpha, double t);
double get_z2(double alpha, double t);
double get_psi2(double alpha, double t);
double get_vx2(double alpha, double t);
double get_vy2(double alpha, double t);
double get_vz2(double alpha, double t);
double get_ax2(double alpha, double t);
double get_ay2(double alpha, double t);
double get_az2(double alpha, double t);

double get_x3(double alpha, double t);
double get_y3(double alpha, double t);
double get_z3(double alpha, double t);
double get_psi3(double alpha, double t);
double get_vx3(double alpha, double t);
double get_vy3(double alpha, double t);
double get_vz3(double alpha, double t);
double get_ax3(double alpha, double t);
double get_ay3(double alpha, double t);
double get_az3(double alpha, double t);

inline double get_x(double alpha, double t, int nr) {
  if (nr == 1) return get_x2(alpha,t);
  if (nr == 2) return get_x3(alpha,t);
  return get_x1(alpha,t);
}
inline double get_y(double alpha, double t, int nr) {
  if (nr == 1) return get_y2(alpha,t);
  if (nr == 2) return get_y3(alpha,t);
  return get_y1(alpha,t);
}
inline double get_z(double alpha, double t, int nr) {
  if (nr == 1) return get_z2(alpha,t);
  if (nr == 2) return get_z3(alpha,t);
  return get_z1(alpha,t);
}
inline double get_psi(double alpha, double t, int nr) {
  if (nr == 1) return get_psi2(alpha,t);
  if (nr == 2) return get_psi3(alpha,t);
  return get_psi1(alpha,t);
}
inline double get_vx(double alpha, double t, int nr) {
  if (nr == 1) return get_vx2(alpha,t);
  if (nr == 2) return get_vx3(alpha,t);
  return get_vx1(alpha,t);
}
inline double get_vy(double alpha, double t, int nr) {
  if (nr == 1) return get_vy2(alpha,t);
  if (nr == 2) return get_vy3(alpha,t);
  return get_vy1(alpha,t);
}
inline double get_vz(double alpha, double t, int nr) {
  if (nr == 1) return get_vz2(alpha,t);
  if (nr == 2) return get_vz3(alpha,t);
  return get_vz1(alpha,t);
}
inline double get_ax(double alpha, double t, int nr) {
  if (nr == 1) return get_ax2(alpha,t);
  if (nr == 2) return get_ax3(alpha,t);
  return get_ax1(alpha,t);
}
inline double get_ay(double alpha, double t, int nr) {
  if (nr == 1) return get_ay2(alpha,t);
  if (nr == 2) return get_ay3(alpha,t);
  return get_ay1(alpha,t);
}
inline double get_az(double alpha, double t, int nr) {
  if (nr == 1) return get_az2(alpha,t);
  if (nr == 2) return get_az3(alpha,t);
  return get_az1(alpha,t);
}

#endif