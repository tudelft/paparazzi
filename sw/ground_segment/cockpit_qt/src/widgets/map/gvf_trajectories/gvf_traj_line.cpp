#include "gvf_traj_line.h"

GVF_traj_line::GVF_traj_line(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings) :
    GVF_trajectory(id, gvf_settings)
{   
    set_param(param, _s, _ke);
    generate_trajectory();
}

// Get all the necessary parameters to construct the line trajectory
void GVF_traj_line::set_param(QList<float> param, int8_t _s, float _ke) {

    if(param.size()==6) { // gvf_line_XY1_XY2()
        a = param[0];
        b = param[1];

        float a2 = param[3];
        float b2 = param[4];

        dx = a2 - a;
        dy = b2 - b;

        course = param[2];

    } else if (param.size()>3) { // gvf_line_wp1_wp2()
        QPointF xy_wp1;
        QPointF xy_wp2; 

        auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
        Waypoint::WpFrame frame = ac->getFlightPlan()->getFrame();
        ac->getFlightPlan()->getWaypoint((uint8_t)param[3])->getRelative(frame, xy_wp1.rx(), xy_wp1.ry());
        ac->getFlightPlan()->getWaypoint((uint8_t)param[4])->getRelative(frame, xy_wp2.rx(), xy_wp2.ry());

        a = xy_wp1.x();
        b = xy_wp1.y();

        dx = xy_wp2.x() - xy_wp1.x();
        dy = xy_wp2.y() - xy_wp1.y();

        course = atan2f(dx, dy);

    } else { // gvf_line_XY()
        a = param[0];
        b = param[1];

        dx = 200;
        dy = 200;

        course = param[2];
    }

    s = _s;
    ke = _ke;

    if (param.size() >= 7) { // avoid out of bounds error
        if (param[5]>=0 && param[6]>=0) { // gvf_segment_loop_XY1_XY2
            d1 = param[5];
            d2 = param[6];
            if (abs(course) > M_PI/2) { //TODO: Better fix with course??
                s *= -1;
            }
        }
    }

    xy_off = getACpos();
}

// Line trajectory (parametric representation)
void GVF_traj_line::genTraj() { 
    QList<QPointF> points;

    float dr = sqrt(pow(dx,2) + pow(dy,2));

    if (dr > 0) { 
        float dt = dr/100;
        for (float t = -d1; t <= dr + d2 + dt/2; t+=dt) {
            float x = t*sin(course) + a;
            float y = t*cos(course) + b;
            points.append(QPointF(x,y));
        }
    }

    createTrajItem(points);
}

// Line GVF (implicit function)
void GVF_traj_line::genVField() {
    QList<QPointF> vxy_mesh; 
    xy_off = getACpos();
    
    float bound_area = pow(dx,2) + pow(dy,2) + 1; // to scale the arrows

    emit DispatcherUi::get()->gvf_defaultFieldSettings(ac_id, bound_area, 25, 25);
    xy_mesh = meshGrid(); //4*bound_area TODO: Fix auto scale method

    foreach (const QPointF &point, xy_mesh) {
        double nx = -cos(course);
        double ny =  sin(course);

        double tx =  s*ny;
        double ty = -s*nx;

        double e = (point.x() - a)*nx + (point.y() - b)*ny;

        double vx = tx - (1e-2*ke)*e*nx;
        double vy = ty - (1e-2*ke)*e*ny;

        double norm = sqrt(pow(vx,2) + pow(vy,2));

        vxy_mesh.append(QPointF(vx/norm, vy/norm));
    }

    createVFieldItem(xy_mesh, vxy_mesh, 500);
}