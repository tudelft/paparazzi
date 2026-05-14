#include "gvf_traj_3D_ellipse.h"

GVF_traj_3D_ellipse::GVF_traj_3D_ellipse(QString id, QList<float> param, QList<float> _phi, QVector<int> *gvf_settings) :
    GVF_trajectory(id, gvf_settings)
{   
    set_param(param, _phi);
    generate_trajectory();
    
}

// 3D ellipse trajectory (parametric representation)
void GVF_traj_3D_ellipse::genTraj() { 
    QList<QPointF> xy_points;
    QList<float> z_points;

    float dt = 0.005*2*M_PI;
    for (float t = 0; t <= 2*M_PI + dt/2; t+=dt) {
        auto point = traj_point(t);

        xy_points.append(QPointF(point.x(),point.y()));
        z_points.append(point.z());
    }

    createTrajItem(xy_points, z_points);
}

// do nothing...
void GVF_traj_3D_ellipse::genVField() { 

}

/////////////// PRIVATE FUNCTIONS ///////////////
void GVF_traj_3D_ellipse::set_param(QList<float> param, QList<float> _phi) {

    if (param.size()>6) { // gvf_parametric_3D_ellipse_wp()
        auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
        Waypoint::WpFrame frame = ac->getFlightPlan()->getFrame();
        ac->getFlightPlan()->getWaypoint((uint8_t)param[6])->getRelative(frame, xy_off.rx(), xy_off.ry());

    } else { // gvf_parametric_3D_ellipse_XYZ()
        xy_off = QPointF(param[0], param[1]);
    }

    r = param[2];
    zl = param[3];
    zh = param[4];
    alpha = param[5];

    phi = QVector3D(_phi[0], _phi[1], _phi[2]); //TODO: Display error in GVF viewer??
}

QVector3D GVF_traj_3D_ellipse::traj_point(float t) {
    QVector3D point;

    point.setX(xy_off.x() + r*cos(t));
    point.setY(xy_off.y() + r*sin(t));
    point.setZ(0.5 * (zh + zl + (zl - zh) * sin(alpha - t)));
    return point;
}