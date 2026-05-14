#ifndef ACITEMMANAGER_H
#define ACITEMMANAGER_H

#include "map_item.h"
#include "waypoint_item.h"
#include "aircraft_item.h"
#include "path_item.h"
#include "mapwidget.h"
#include "circle_item.h"
#include "arrow_item.h"
#include "gvf_trajectory.h"

class ACItemManager: public QObject
{
    Q_OBJECT
public:
    ACItemManager(QString ac_id, WaypointItem* target, AircraftItem* aircraft_item, ArrowItem* arrow_item, WaypointItem* crash_item, QObject* parent=nullptr);

    void addWaypointItem(WaypointItem*);
    void addPathItem(PathItem*);
    void setCurrentNavShape(MapItem*);
    void setMaxDistCircle(CircleItem*);
    void setCurrentGVF(GVF_trajectory*);

    QList<WaypointItem*> getWaypointsItems() {return waypointItems;}
    WaypointItem* getTarget() {return target;}
    MapItem* getCurrentNavShape() {return current_nav_shape;}
    AircraftItem* getAircraftItem() {return aircraft_item;}
    ArrowItem* getArrowItem() {return arrow_item;}
    WaypointItem* getCrashItem() {return crash_item;}
    void removeItems(MapWidget* map);

private:
    QString ac_id;
    QList<WaypointItem*> waypointItems;
    QList<PathItem*> pathItems;
    WaypointItem* target;
    AircraftItem* aircraft_item;
    MapItem* current_nav_shape;
    CircleItem* max_dist_circle;
    ArrowItem* arrow_item;
    WaypointItem* crash_item;

    GVF_trajectory* gvf_trajectory;
};

#endif // ACITEMMANAGER_H
