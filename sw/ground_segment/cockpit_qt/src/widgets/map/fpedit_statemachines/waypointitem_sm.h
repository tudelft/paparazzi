#ifndef SMWAYPOINTITEM_H
#define SMWAYPOINTITEM_H

#include "item_edit_state_machine.h"
#include "waypoint_item.h"

class SmWaypointItem : public ItemEditStateMachine
{
public:
    SmWaypointItem(MapWidget* map);
    ~SmWaypointItem();
    virtual MapItem* update(SmEditEvent event_type, QGraphicsSceneMouseEvent* mouseEvent, WaypointItem* waypoint, QString ac_id, MapItem* item = nullptr);

private:

    enum State {
        IDLE,
        MOVING,
    };

    WaypointItem* wp;
    State state;
};

#endif // SMWAYPOINTITEM_H
