#include "circleitem_sm.h"
SmCircleItem::SmCircleItem(MapWidget* map) : ItemEditStateMachine(map), cir(nullptr), state(IDLE) {}
SmCircleItem::~SmCircleItem() {}
MapItem* SmCircleItem::update(SmEditEvent, QGraphicsSceneMouseEvent*, WaypointItem*, QString, MapItem*) { return nullptr; }
void SmCircleItem::adjustCircleRadius(QGraphicsSceneMouseEvent*) {}
