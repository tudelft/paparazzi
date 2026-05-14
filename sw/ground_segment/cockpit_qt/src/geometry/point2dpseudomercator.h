#ifndef POINT2DPSEUDOMERCATOR_H
#define POINT2DPSEUDOMERCATOR_H

#include <string>
#include <QRectF>

class Point2DTile;

class Point2DPseudoMercator
{
public:
    Point2DPseudoMercator();
    Point2DPseudoMercator(double _x, double _y);
    Point2DPseudoMercator(Point2DTile);

    double x() const { return _x; }
    double y() const { return _y; }

    Point2DPseudoMercator operator-(const Point2DPseudoMercator& other) const;
    Point2DPseudoMercator operator+(const Point2DPseudoMercator& other) const;
    Point2DPseudoMercator operator/(double d) const;
    Point2DPseudoMercator operator*(double d) const;



    static QRectF getBounds();


    Point2DTile toTile(int zoom);

    static const QString EPSG;
private:
    //unit: meters
    double _x;
    double _y;
};

#endif // POINT2DPSEUDOMERCATOR_H
