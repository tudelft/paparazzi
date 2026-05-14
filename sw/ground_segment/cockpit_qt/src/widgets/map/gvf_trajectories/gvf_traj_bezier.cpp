#include "gvf_traj_bezier.h"
#include <stdio.h>
#include <stdlib.h>

// Path to files where data is saved. Choose the path you like.
const char x_val[]  = "var/conf/gvf_parametric_curve_x_values.data";
const char y_val[]  = "var/conf/gvf_parametric_curve_y_values.data";
const char ks_val[] = "var/conf/gvf_parametric_curve_ctrl_values.data";

GVF_traj_bezier::GVF_traj_bezier(QString id, QList<float> param, QList<float> _phi,
                                 float wb, QVector<int> *gvf_settings) :
  GVF_trajectory(id, gvf_settings)
{
  set_param(param, _phi, wb);
  generate_trajectory();
}

// 2D bezier trajectory (parametric representation)
void GVF_traj_bezier::genTraj()
{

  QList<QPointF> points;
  // 0 <= t <= n_segments of the spline
  float max_t = (n_seg > 0) ? n_seg : 1; // At least one spline must be defined
  // 100 pts per each segment
  float num_pts = n_seg * 100;
  float dt = max_t / num_pts;

  for (float t = 0; t <= max_t; t += dt) {
    points.append(traj_point(t));
  }
  createTrajItem(points);
}


// 2D bezier GVF
void GVF_traj_bezier::genVField()
{

  FILE *file_x; FILE *file_y; FILE *file_ks;

  // Only draw the file when data is available
  int cont = 0;
  if ((file_x = fopen(x_val, "r")) != NULL) {
    cont++;
    fclose(file_x);
  }
  if ((file_y = fopen(y_val, "r")) != NULL) {
    cont++;
    fclose(file_y);
  }
  if ((file_ks = fopen(ks_val, "r")) != NULL) {
    cont++;
    fclose(file_ks);
  }

  if (cont == 3) {
    QList<QPointF> vxy_mesh;
    float xmin, xmax, ymin, ymax;
    xmin = xx[0]; xmax = xx[0]; ymin = yy[0]; ymax = yy[0];
    for (int k = 0; k < 3 * n_seg + 1; k++) {
      xmax = (xx[k] > xmax) ? xx[k] : xmax;
      xmin = (xx[k] < xmin) ? xx[k] : xmin;
      ymax = (yy[k] > ymax) ? yy[k] : ymax;
      ymin = (yy[k] < ymin) ? yy[k] : ymin;
    }

    float bound_area = 20 * (xmax - xmin) * (ymax - ymin);
    emit DispatcherUi::get()->gvf_defaultFieldSettings(ac_id, round(bound_area), 60, 60);
    xy_mesh = meshGrid();

    foreach (const QPointF &point, xy_mesh) {

      float phix = point.x() - traj_point(w).x(); // Normal component
      float phiy = point.y() - traj_point(w).y(); // Normal Component
      float sigx = beta * traj_point_deriv(w).x(); // Tangential Component
      float sigy = beta * traj_point_deriv(w).y(); // Tangential Component
      float vx = sigx - kx * phix;
      float vy = sigy - ky * phiy;
      float norm = sqrt(pow(vx, 2) + pow(vy, 2));
      norm = (norm > 0) ? norm : 1; // Avoid division by zero
      vxy_mesh.append(QPointF(vx / norm, vy / norm));

    }
    createVFieldItem(xy_mesh, vxy_mesh);
  } else {
    fprintf(stderr, "Field cannot be created yet, waiting for complete data...\n");
  }

}


/////////////// PRIVATE FUNCTIONS ///////////////
void GVF_traj_bezier::set_param(QList<float> param, QList<float> _phi, float wb)
{


  FILE *file_x; FILE *file_y; FILE *file_ks;
  int k;

  // Write:
  if (param[0] < 0) {
    n_seg = -(int)param[0];
    if ((file_x = fopen(x_val, "w+")) != NULL) {
      for (k = 0; k < 3 * n_seg + 2; k++) {
        fprintf(file_x, "%f ", param[k]);
      }
      fclose(file_x);
    }
  } else if (param[0] > 0) {
    n_seg = (int)param[0];
    if ((file_y = fopen(y_val, "w+")) != NULL) {
      for (k = 0; k < 3 * n_seg + 2; k++) {
        fprintf(file_y, "%f ", param[k]);
      }
      fclose(file_y);
    }
  } else {
    if ((file_ks = fopen(ks_val, "w+")) != NULL) {
      fprintf(file_ks, "%f %f %f", param[1], param[2], param[3]);
      fclose(file_ks);
    }
  }

  float N_SEG;

  // Read:
  if ((file_x = fopen(x_val, "r")) != NULL) {
    fscanf(file_x, "%f ", &N_SEG);
    n_seg = -(int)N_SEG;
    for (k = 0; k < 3 * (int)n_seg + 1; k++) {
      fscanf(file_x, "%f ", &xx[k]);
    }
    fclose(file_x);
  }
  if ((file_y = fopen(y_val, "r")) != NULL) {
    fscanf(file_y, "%f ", &N_SEG);
    n_seg = (int)N_SEG;
    for (k = 0; k < 3 * (int)n_seg + 1; k++) {
      fscanf(file_y, "%f ", &yy[k]);
    }
    fclose(file_y);
  }
  if ((file_ks = fopen(ks_val, "r")) != NULL) {
    fscanf(file_ks, "%f", &kx);
    fscanf(file_ks, "%f", &ky);
    fscanf(file_ks, "%f", &beta);
    fclose(file_ks);
  }

  phi = QPointF(_phi[0], _phi[1]);   //TODO: Display error in GVF viewer??
  w = (beta > 0) ? wb / beta : wb;   // gvf_parametric_w = wb/beta (wb = w*beta)
}

QPointF GVF_traj_bezier::traj_point(float t)
{

  // Just in case w from telemetry is not between bounds
  if (t < 0.0) {
    t = 0.0;
  } else if (t >= (3 * n_seg + 1)) {
    t = (float)(3 * n_seg + 1);
  }

  int k = (int)t;

  /* Each Bézier spline must be evaluated between 0 and 1.
  Remove integer part, get fractional
  Choose which spline:
  (0<=t<=1 -> Spline1 (coefs 0-3),
  1<=t<=2 -> Spline2 (coefs 3-6) ... */
  t = t - k;
  k = 3 * k;
  float x = xx[k] * (1 - t) * (1 - t) * (1 - t) + 3 * (1 - t) * (1 - t) * t * xx[k + 1] +
            3 * (1 - t) * t * t * xx[k + 2] + t * t * t * xx[k + 3];
  float y = yy[k] * (1 - t) * (1 - t) * (1 - t) + 3 * (1 - t) * (1 - t) * t * yy[k + 1] +
            3 * (1 - t) * t * t * yy[k + 2] + t * t * t * yy[k + 3];
  return QPointF(x, y);
}

QPointF GVF_traj_bezier::traj_point_deriv(float t)
{

  // Just in case w from telemetry is not between bounds
  if (t < 0.0) {
    t = 0.0;
  } else if (t >= (3 * n_seg + 1)) {
    t = (float)(3 * n_seg + 1);
  }

  int k = (int)t;

  /* Each Bézier spline must be evaluated between 0 and 1.
  Remove integer part, get fractional
  Choose which spline:
  (0<=t<=1 -> Spline1 (coefs 0-3),
  1<=t<=2 -> Spline2 (coefs 3-6) ... */
  t = t - k;
  k = 3 * k;
  float dx = (xx[k + 1] - xx[k]) * 3 * (1 - t) * (1 - t) +
             6 * (1 - t) * t * (xx[k + 2] - xx[k + 1]) + 3 * t * t * (xx[k + 3] - xx[k + 2]);
  float dy = (yy[k + 1] - yy[k]) * 3 * (1 - t) * (1 - t) +
             6 * (1 - t) * t * (yy[k + 2] - yy[k + 1]) + 3 * t * t * (yy[k + 3] - yy[k + 2]);
  return QPointF(dx, dy);
}

