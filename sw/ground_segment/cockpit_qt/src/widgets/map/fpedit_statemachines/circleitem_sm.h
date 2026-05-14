#ifndef SMCIRCLEITEM_H
#define SMCIRCLEITEM_H

#include "item_edit_state_machine.h"
#include "circle_item.h"

class SmCircleItem : public ItemEditStateMachine
{
public:
    SmCircleItem(MapWidget* map);
    ~SmCircleItem();
    virtual MapItem* update(SmEditEvent event_type, QGraphicsSceneMouseEvent* mouseEvent, WaypointItem* waypoint, QString ac_id, MapItem* item = nullptr);

private:
    enum State {
        IDLE,
        PRESSED,   // mouse pressed, move < hysteresis.
        DRAGING,   // mouse dragged after the press. expand the circle according to mouse moves
        RELEASED,  // mouse released after the press. Wait for next press event to set circle radius.
    };

    void adjustCircleRadius(QGraphicsSceneMouseEvent* mouseEvent);
    QPointF pressPos;
    CircleItem* cir;
    State state;
};

#endif // SMCIRCLEITEM_H
