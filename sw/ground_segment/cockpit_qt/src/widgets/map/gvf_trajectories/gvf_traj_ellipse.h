#ifndef GVF_TRAJ_ELLIPSE_H
#define GVF_TRAJ_ELLIPSE_H

#include "gvf_trajectory.h"

class GVF_traj_ellipse : public GVF_trajectory
{
    Q_OBJECT
public:
    explicit GVF_traj_ellipse(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings);

protected:
    virtual void genTraj() override;
    virtual void genVField() override;

private:
    void set_param(QList<float> param, int8_t _s, float _ke);

    float a;
    float b;
    float alpha;

    int8_t s;
    float ke;
};

#endif // GVF_TRAJ_ELLIPSE_H