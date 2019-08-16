/*
 * Copyright (C) mavlab
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/qpoas/qpoas.c"
 * @author mavlab
 * opti
 */

#include "modules/qpoas/qpoas.h"
#include "sw/ext/qpOASES-3.2.1/include/qpOASES.hpp"

// Eigen headers
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Dense>
#pragma GCC diagnostic pop


#define MAX_N 100
using namespace Eigen;

void qp_init(void) {
  float pos0[2] = {10, 2};
  float posf[2] = {0 ,0};

  float vel0[2] = {0, 0};
  float velf[2] = {0, 0};
  float dt = 0.1;
  float T = sqrt(pow((pos0[0] - posf[0]),2) + pow((pos0[1] - posf[1]),2)) / 4.0;
  unsigned int N = round(T/dt);


  Eigen::Matrix<double, 4, 4> A;
  A << 0.9512, 0, 0, 0,
  0.09754, 1, 0, 0,
  0, 0, 0.9512, 0,
  0, 0, 0.09754, 1;

  Eigen::Matrix<double, 4, 2> B;
  B << 0.9569, 0,
  0.04824, 0,
  0, 0.9569,
  0, 0.04824;

  Eigen::Matrix<double, 4, 4> P;
  P << 1,0,0,0,
  0,10,0,0,
  0,0,1,0,
  0,0,0,10;

  // Eigen::Matrix<double, 4, Dynamic> R;
  Eigen::MatrixXd oldR(4, 2 * MAX_N);
  oldR.resize(4, 2*N);
  printf("Eigen: %f\n", B(0,0));
}

void replan(void) {}


