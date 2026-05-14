#include "gvf_traj_3D_lissajous.h"

GVF_traj_3D_lissajous::GVF_traj_3D_lissajous(QString id, QList<float> param, QList<float> _phi, QVector<int> *gvf_settings) :
    GVF_trajectory(id, gvf_settings)
{   
    set_param(param, _phi);
    generate_trajectory();
    
}

// 3D lissajous trajectory (parametric representation)
void GVF_traj_3D_lissajous::genTraj() { 
    QList<QPointF> xy_points;
    QList<float> z_points;

    float max_t = 2*M_PI*2;
    float dt = 0.001*2*M_PI;
    for (float t = 0; t <= max_t + dt/2; t+=dt) {
        auto point = traj_point(t);

        xy_points.append(QPointF(point.x(),point.y()));
        z_points.append(point.z());
    }
    
    createTrajItem(xy_points, z_points);
}

// do nothing...
void GVF_traj_3D_lissajous::genVField() { 

}

/////////////// PRIVATE FUNCTIONS ///////////////
void GVF_traj_3D_lissajous::set_param(QList<float> param, QList<float> _phi) {

    if (param.size()>14) { // gvf_parametric_3D_lissajous_wp_center
        auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
        Waypoint::WpFrame frame = ac->getFlightPlan()->getFrame();
        ac->getFlightPlan()->getWaypoint((uint8_t)param[13])->getRelative(frame, xy_off.rx(), xy_off.ry());
        alt = ac->getFlightPlan()->getWaypoint((uint8_t)param[13])->getAlt();

    } else { // gvf_parametric_3D_lissajous_XYZ
        xy_off = QPointF(param[0], param[1]);
        alt = param[2];
    }

    c = QVector3D(param[3], param[4], param[5]);
    w = QVector3D(param[6], param[7], param[8]);
    d = QVector3D(param[9]*M_PI/180, param[10]*M_PI/180, param[11]*M_PI/180);

    alpha = param[12]*M_PI/180;

    phi = QVector3D(_phi[0], _phi[1], _phi[2]); //TODO: Display error in GVF viewer??
}

QVector3D GVF_traj_3D_lissajous::traj_point(float t) {
    QVector3D point;
    
    float nrx = c.x()*cosf(w.x()*t + d.x());
    float nry = c.y()*cosf(w.y()*t + d.y());

    point.setX(xy_off.x() + nrx*cosf(alpha) - nry*sinf(alpha));
    point.setY(xy_off.y() + nrx*sinf(alpha) + nry*cosf(alpha));
    point.setZ(alt + c.z()*cosf(w.z()*t + d.z()));
    return point;
}