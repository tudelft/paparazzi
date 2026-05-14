#include "gvf_traj_trefoil.h"

GVF_traj_trefoil::GVF_traj_trefoil(QString id, QList<float> param, QList<float> _phi, QVector<int> *gvf_settings) :
    GVF_trajectory(id, gvf_settings)
{   
    set_param(param, _phi);
    generate_trajectory();
}

// 2D trefoil knot trajectory (parametric representation)
void GVF_traj_trefoil::genTraj() { 
    QList<QPointF> points;
    
    float res = 1e7;
    float N;

    if (w1>w2) {
        N = 1/gcd(res*w1,res*w2)*res;
    } else {
        N = 1/gcd(res*w2,res*w1)*res;
    }

    float max_t = 2*M_PI*N;
    float num_pts = N*10;

    float dt = max_t /num_pts;
    for (float t = 0; t <= max_t + dt/2; t+=dt) {
        points.append(traj_point(t));
    }

    createTrajItem(points);
}


// 2D trefoil knot GVF 
void GVF_traj_trefoil::genVField() { 

}

/////////////// PRIVATE FUNCTIONS ///////////////
void GVF_traj_trefoil::set_param(QList<float> param, QList<float> _phi) {
    if (param.size()>7) { // gvf_parametric_2D_trefoil_wp()
        auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
        Waypoint::WpFrame frame = ac->getFlightPlan()->getFrame();
        ac->getFlightPlan()->getWaypoint((uint8_t)param[7])->getRelative(frame, xy_off.rx(), xy_off.ry());

    } else { // gvf_parametric_2D_trefoil_XY()
        xy_off = QPointF(param[0], param[1]);
    }

    w1 = param[2];
    w2 = param[3];
    ratio = param[4];
    r = param[5];
    alpha = param[6]*M_PI/180;

    phi = QPointF(_phi[0], _phi[1]); //TODO: Display error in GVF viewer??
}

QPointF GVF_traj_trefoil::traj_point(float t) {
    float xnr = cos(w1*t) * (r*cos(w2*t) + ratio);
    float ynr = sin(w1*t) * (r*cos(w2*t) + ratio);

    float x =  xy_off.x() + cos(alpha)*xnr - sin(alpha)*ynr;
    float y =  xy_off.y() + sin(alpha)*xnr + cos(alpha)*ynr;

    return QPointF(x,y);
}

float GVF_traj_trefoil::gcd(int a, int b){
    if(b == 0)
        return a;
    return gcd(b, a % b);
}