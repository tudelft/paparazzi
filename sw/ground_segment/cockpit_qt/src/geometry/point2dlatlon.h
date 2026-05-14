#ifndef POINT2DLATLON_H
#define POINT2DLATLON_H

#include "waypoint.h"
#include <algorithm>

class Point2DTile;

class Point2DLatLon
{
public:
    Point2DLatLon();
    Point2DLatLon(double lat, double lon);
    //Point2DLatLon(Point2DTile pt);
    Point2DLatLon(Waypoint* wp);

    double lat() {return latp;}
    double lon() {return lonp;}
    void setLat(double lat) {latp = std::clamp(lat, -90., 90.);}
    void setLon(double lon) {lonp = std::clamp(lon, -180., 180.);}

    QString toString(bool sexagesimal=false);

private:
    double latp;
    double lonp;
};

Q_DECLARE_METATYPE(Point2DLatLon)

#endif // POINT2DLATLON_H
