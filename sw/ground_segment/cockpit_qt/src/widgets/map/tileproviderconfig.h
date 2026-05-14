#ifndef TILEPROVIDERCONFIG_H
#define TILEPROVIDERCONFIG_H

#include <QString>
#include <memory>
#include "point2dtile.h"
#include <QtXml>

class TileProviderConfig
{
public:

    TileProviderConfig(QDomElement ele);
    TileProviderConfig(QString name, QString dir, QString addr,
                       int zoomMin, int zoomMax, double xMin, double xMax, double yMin, double yMax,
                       int tileSize, QString format):
        name(name), dir(dir), addr(addr),
        zoomMin(zoomMin), zoomMax(zoomMax), xMin(xMin), xMax(xMax), yMin(yMin), yMax(yMax),
        tileSize(tileSize), format(format)
    {
    }

    void printConfig();
    bool isValid(Point2DTile pt_tile);

    QString name;
    QString dir;
    QString addr;
    int zoomMin;
    int zoomMax;
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    int tileSize;
    QString format;

    int initial_rank;   // kind of the oposite of zValue...
};

#endif // TILEPROVIDERCONFIG_H
