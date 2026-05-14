#ifndef GVF_TRAJ_SIN_H
#define GVF_TRAJ_SIN_H

#include "gvf_trajectory.h"

class GVF_traj_sin : public GVF_trajectory
{
    Q_OBJECT
public:
    explicit GVF_traj_sin(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings);

protected:
    virtual void genTraj() override;
    virtual void genVField() override;

private:
    void set_param(QList<float> param, int8_t _s, float _ke);
    
    float a;
    float b;
    float course;
    float w;
    float off;
    float A;

    int8_t s;
    double ke;

    float dx;
    float dy;
};

#endif // GVF_TRAJ_SIN_H