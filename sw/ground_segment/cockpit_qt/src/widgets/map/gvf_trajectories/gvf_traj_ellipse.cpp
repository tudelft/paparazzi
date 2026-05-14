#include "gvf_traj_ellipse.h"

GVF_traj_ellipse::GVF_traj_ellipse(QString id, QList<float> param, int8_t _s, float _ke, QVector<int> *gvf_settings) :
    GVF_trajectory(id, gvf_settings)
{   
    set_param(param, _s, _ke);
    generate_trajectory();
    
}

void GVF_traj_ellipse::set_param(QList<float> param, int8_t _s, float _ke) {
    if (param.size()>5) { // gvf_ellipse_wp()
        auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
        Waypoint::WpFrame frame = ac->getFlightPlan()->getFrame();
        ac->getFlightPlan()->getWaypoint((uint8_t)param[5])->getRelative(frame, xy_off.rx(), xy_off.ry());

    } else { // gvf_ellipse_XY()
        xy_off = QPointF(param[0], param[1]);
    }

    s = _s;
    ke = _ke;
    a = param[2];
    b = param[3];
    alpha = param[4];
}

// Ellipse trajectory (rotated standard ellipse parametric representation)
void GVF_traj_ellipse::genTraj() { 
    QList<QPointF> points;

    float dt = 0.005*2*M_PI;
    for (int i = 0; i <= 2*M_PI/dt + 1; i++) {
        float x = xy_off.x() + a*cos(-alpha)*cos(dt*i) - b*sin(-alpha)*sin(i*dt);
        float y = xy_off.y() + a*sin(-alpha)*cos(dt*i) + b*cos(-alpha)*sin(i*dt);
        points.append(QPointF(x,y));
    }

    createTrajItem(points);
}

// Ellipse GVF (standard ellipse implicit function)
void GVF_traj_ellipse::genVField() { 
    QList<QPointF> vxy_mesh; 

    float bound_area = 8*a*b; // to scale the arrows

    emit DispatcherUi::get()->gvf_defaultFieldSettings(ac_id, round(bound_area), 25, 25);
    xy_mesh = meshGrid();

    // Rotate the space -alpha deg, draw the ellipse vector field for each point in the mesh and then
    // rotate alpha to come back to the original space
    foreach (const QPointF &point, xy_mesh) {
        float xel =  (point.x() - xy_off.x())*cos(-alpha) + (point.y() - xy_off.y())*sin(-alpha); 
        float yel = -(point.x() - xy_off.x())*sin(-alpha) + (point.y() - xy_off.y())*cos(-alpha); 

        float nx =  2*xel*cos(alpha)/pow(a,2) + 2*yel*sin(alpha)/pow(b,2);
        float ny = -2*xel*sin(alpha)/pow(a,2) + 2*yel*cos(alpha)/pow(b,2);

        float tx =  s*ny;
        float ty = -s*nx;

        float e = pow(xel/a,2) + pow(yel/b,2) - 1;

        float vx = tx - ke*e*nx;
        float vy = ty - ke*e*ny;

        float norm = sqrt(pow(vx,2) + pow(vy,2));

        vxy_mesh.append(QPointF(vx/norm, vy/norm));
    }

    createVFieldItem(xy_mesh, vxy_mesh);
}