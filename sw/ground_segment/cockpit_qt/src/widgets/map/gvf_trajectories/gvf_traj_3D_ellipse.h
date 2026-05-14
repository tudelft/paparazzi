#ifndef GVF_TRAJ_3D_ELLIPSE_H
#define GVF_TRAJ_3D_ELLIPSE_H

#include "gvf_trajectory.h"

class GVF_traj_3D_ellipse : public GVF_trajectory
{
    Q_OBJECT
public:
    explicit GVF_traj_3D_ellipse(QString id, QList<float> param, QList<float> _phi, QVector<int> *gvf_settings);

protected:
    virtual void genTraj() override;
    virtual void genVField() override;

private:
    void set_param(QList<float> param, QList<float> _phi); // GVF PARAMETRIC
    QVector3D traj_point(float t);

    float r;
    float zl;
    float zh;
    float alpha;

    QVector3D phi;
};

#endif // GVF_TRAJ_3D_ELLIPSE_H