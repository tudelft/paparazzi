#include "waypointitem_sm.h"
SmWaypointItem::SmWaypointItem(MapWidget* map) : ItemEditStateMachine(map), wp(nullptr), state(IDLE) {}
SmWaypointItem::~SmWaypointItem() {}
MapItem* SmWaypointItem::update(SmEditEvent, QGraphicsSceneMouseEvent*, WaypointItem*, QString, MapItem*) { return nullptr; }
