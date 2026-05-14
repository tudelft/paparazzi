#include "sector.h"

Sector::Sector(QList<Waypoint*> corners, QString name, QString color_str):
    name(name), corners(corners)
{
    color = QColor(color_str);
}
