#include "pathitem_sm.h"
// Stub: flight plan path editing disabled in cockpit_qt
SmPathItem::SmPathItem(MapWidget* map) : ItemEditStateMachine(map), path(nullptr), lastWp(nullptr), previousWp(nullptr), state(IDLE), ac_id(QString()) {}
SmPathItem::~SmPathItem() {}
MapItem* SmPathItem::update(SmEditEvent, QGraphicsSceneMouseEvent*, WaypointItem*, QString, MapItem*) { return nullptr; }
