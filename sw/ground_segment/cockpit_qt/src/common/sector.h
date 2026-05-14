#ifndef SECTOR_H
#define SECTOR_H

#include <string>
#include "waypoint.h"
#include <memory>
#include <QColor>

using namespace std;
class Sector
{
public:
    Sector(QList<Waypoint*> corners, QString name, QString color);

    QString getName() {return name;}
    QList<Waypoint*> getCorners() {return corners;}
    QColor getColor(){return color;}


private:
    QString name;
    QColor color;   // override AC color
    QList<Waypoint*> corners;
};

#endif // SECTOR_H
