#include "maputils.h"
#include <QDebug>
#include "coordinatestransform.h"
#include "point2dpseudomercator.h"

constexpr double EARTH_RADIUS = 6378137.0;

Point2DTile tilePoint(QPointF scenePos, int zoom, int tileSize) {
    return Point2DTile(scenePos.x()/tileSize, scenePos.y()/tileSize, zoom);
}

QPointF scenePoint(Point2DTile tilePoint, int tileSize) {
    return QPointF(tilePoint.x()*tileSize, tilePoint.y()*tileSize);
}

QPointF scenePoint(Point2DLatLon latlon, int zoomLvl, int tileSize) {
    Point2DPseudoMercator pm = CoordinatesTransform::get()->WGS84_to_pseudoMercator(latlon);
    Point2DTile tile_pos = pm.toTile(zoomLvl);
    return scenePoint(tile_pos, tileSize);
}

QPointF scenePoint(Point2DPseudoMercator pm, int zoomLvl, int tileSize) {
    Point2DTile tile_pos = pm.toTile(zoomLvl);
    return scenePoint(tile_pos, tileSize);
}

int zoomLevel(double zoom) {
    return static_cast<int>(ceil(zoom));
}

///
/// \brief distMeters2Tile convert real world meters to tile coordinates. Use only for "small" distances compared to earth radius
/// \param distance in meters
/// \param lat mean latitude
/// \param zoom zoomLevel
/// \return distance in tile coordinates representing the distance in meters.
///
double distMeters2Tile(double distance, double lat, int zoom) {
    double latrad = lat * M_PI/180.0;

    double latrad1 = latrad - distance/(2*EARTH_RADIUS);
    double latrad2 = latrad + distance/(2*EARTH_RADIUS);

    double dy = (asinh(tan(latrad2)) - asinh(tan(latrad1))) * (1 << (zoom-1)) / M_PI;
    return dy;
}

double distTile2Meters(double y, double d, int zoom) {
    double y1 = y + d/2;
    double y2 = y - d/2;

    double n1 = M_PI - 2.0 * M_PI * y1 / static_cast<double>(1 << zoom);
    double n2 = M_PI - 2.0 * M_PI * y2 / static_cast<double>(1 << zoom);

    double dlatrad = atan(0.5 * (exp(n2) - exp(-n2))) - atan(0.5 * (exp(n1) - exp(-n1)));
    return dlatrad * EARTH_RADIUS;
}
