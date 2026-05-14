#include "point2dtile.h"
#include "point2dlatlon.h"
#include "math.h"

Point2DTile::Point2DTile(double x, double y, int zoom) :
    xp(x), yp(y), zoomp(zoom)
{
}

Point2DTile::Point2DTile(Point2DLatLon latLon, int zoom):
    zoomp(zoom)
{
    double tileX = (latLon.lon() + 180.0) / 360.0 * (1 << zoom);
    double latrad = latLon.lat() * M_PI/180.0;
    double tileY = (1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << zoom);
    xp = tileX;
    yp = tileY;
}

void Point2DTile::changeZoom(int zoom) {
    int oldZoom = zoomp;
    zoomp = zoom;
    int zoom_delta = zoomp - oldZoom;
    xp *= pow(2, zoom_delta);
    yp *= pow(2, zoom_delta);
}

QString Point2DTile::to_istring() {
    QString pStr = "X" + QString::number(xi()) + "Y" + QString::number(yi()) + "Z" + QString::number(zoomp);
    return QString(pStr);
}

bool Point2DTile::isValid() {
    int max = 1 << zoomp;
    bool ret = xp >= 0 && xp < max &&
           yp >= 0 && yp < max;
    return ret;
}

Point2DTile Point2DTile::childPoint(int i, int j) {
    Point2DTile child(xp*2+i, yp*2+j, zoomp+1);
    return child;
}
