#ifndef GVF_TRAJ_BEZIER_H
#define GVF_TRAJ_BEZIER_H

#include "gvf_trajectory.h"

class GVF_traj_bezier : public GVF_trajectory
{
  Q_OBJECT
public:
  explicit GVF_traj_bezier(QString id, QList<float> param, QList<float> _phi,
                           float wb, QVector<int> *gvf_settings);

protected:
  virtual void genTraj() override;
  virtual void genVField() override;

private:
  void set_param(QList<float> param, QList<float> _phi, float wb); // GVF PARAMETRIC
  QPointF traj_point(float t);
  QPointF traj_point_deriv(float t);

  // Maximum data size from gvf_parametric is 16 elements.
  float xx[16];
  float yy[16];
  int n_seg;
  float w;
  float kx;
  float ky;
  float beta;
  QPointF phi;
};

#endif // GVF_TRAJ_bezier_H
