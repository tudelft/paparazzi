#ifndef SMPATHITEM_H
#define SMPATHITEM_H

#include "item_edit_state_machine.h"
#include "path_item.h"
#include "waypoint_item.h"

class SmPathItem : public ItemEditStateMachine
{
public:
    SmPathItem(MapWidget* map);
    ~SmPathItem();
    virtual MapItem* update(SmEditEvent event_type, QGraphicsSceneMouseEvent* mouseEvent, WaypointItem* waypoint, QString ac_id, MapItem* item = nullptr);

private:

    enum State {
        IDLE,
        PRESS_INI,
        MOVE_INI,
        MOVING,
    };

    PathItem* path;
    WaypointItem* lastWp;
    WaypointItem* previousWp;
    State state;
    QPointF pressPos;

    QString ac_id;

};

#endif // SMPATHITEM_H
