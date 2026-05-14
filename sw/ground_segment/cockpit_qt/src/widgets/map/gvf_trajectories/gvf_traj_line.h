#ifndef GVF_TRAJ_LINE_H
#define GVF_TRAJ_LINE_H

#include "gvf_trajectory.h"

class GVF_traj_line : public GVF_trajectory
{
    Q_OBJECT
public:
    explicit GVF_traj_line(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings);

protected:
    virtual void genTraj() override;
    virtual void genVField() override;

private:
    void set_param(QList<float> param, int8_t _s, float _ke);
    
    float a;
    float b;
    float course;
    float d1 = 0;
    float d2 = 0;
    
    int8_t s;
    double ke;

    float dx;
    float dy;
};

#endif // GVF_TRAJ_LINE_H