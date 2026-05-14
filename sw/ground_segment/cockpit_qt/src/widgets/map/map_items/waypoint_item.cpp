#include <QDebug>
#include <QGraphicsScene>
#include "math.h"
#include "maputils.h"
#include "mapwidget.h"
#include "AircraftManager.h"
#include "coordinatestransform.h"
#include "gcs_utils.h"
#include "waypoint_item.h"

// create WaypointItem from position -> create corresponding Waypoint.
WaypointItem::WaypointItem(Point2DLatLon pt, QString ac_id, double neutral_scale_zoom, QObject *parent) :
    MapItem(ac_id, neutral_scale_zoom, parent), moving(false), m_size(0)
{
    original_waypoint = new Waypoint("", 0, pt, 0, this);
    init();
}

WaypointItem::WaypointItem(Point2DLatLon pt, QString ac_id, PprzPalette palette, double neutral_scale_zoom, QObject *parent) :
    MapItem(ac_id, palette, neutral_scale_zoom, parent), moving(false), m_size(0)
{
    original_waypoint = new Waypoint("", 0, pt, 0, this);
    init();
}

// create WaypointItem based on existing Waypoint
WaypointItem::WaypointItem(Waypoint* wp, QString ac_id, double neutral_scale_zoom, QObject *parent) :
    MapItem(ac_id, neutral_scale_zoom, parent), original_waypoint(wp), moving(false), m_size(0)
{
    init();
}

void WaypointItem::init() {
    auto settings = getAppSettings();
    z_value_highlighted = settings.value("map/z_values/highlighted").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/unhighlighted").toDouble();
    z_value = z_value_unhighlighted;

    _waypoint = new Waypoint(original_waypoint, this);
    m_size = settings.value("map/waypoint/size").toInt();

    point = new GraphicsPoint(m_size, palette, this);
    point->setZValue(z_value);


    graphics_text = new GraphicsText(original_waypoint->getName(), QColor(Qt::white), this);
    graphics_text->setZValue(z_value);


    setZoomFactor(1.1);


    connect(
        point, &GraphicsPoint::objectClicked, this,
        [=](QPointF scene_pos) {
            emit itemClicked(scene_pos);
        }
    );

    connect(
        point, &GraphicsPoint::objectDoubleClicked, this,
        [=](QPointF scene_pos) {
            emit itemDoubleClicked(scene_pos);
        }
    );

    connect(
        point, &GraphicsPoint::objectGainedHighlight, this,
        [=]() {
            setHighlighted(true);
            emit itemGainedHighlight();
        }
    );
}

void WaypointItem::addToMap(MapWidget* map) {
    map->scene()->addItem(point);
    map->scene()->addItem(graphics_text);

    connect(
        point, &GraphicsPoint::pointMoved, this,
        [=](QPointF scene_pos) {
            moving = true;
            // reverse position: from scene (mouse) to WGS84.
            Point2DLatLon latlon = CoordinatesTransform::get()->wgs84_from_scene(scene_pos, zoomLevel(map->zoom()), map->tileSize());
            this->setPosition(latlon);
        }
    );

    connect(
        point, &GraphicsPoint::pointMoveFinished, this,
        [=](QPointF scene_pos) {
            Point2DLatLon latlon = CoordinatesTransform::get()->wgs84_from_scene(scene_pos, zoomLevel(map->zoom()), map->tileSize());
            this->setPosition(latlon);
            emit waypointMoveFinished();
        }
    );
}

void WaypointItem::setHighlighted(bool h) {
    MapItem::setHighlighted(h);
    point->setHighlighted(h);
    graphics_text->setHighlighted(h);

    updateZValue();
}

void WaypointItem::setForbidHighlight(bool fh) {
    point->setForbidHighlight(fh);
}

void WaypointItem::setEditable(bool ed) {
    point->setEditable(ed);
    point->setIgnoreEvent(!ed);
}

void WaypointItem::updateZValue() {
    point->setZValue(z_value);
    graphics_text->setZValue(z_value);
}

void WaypointItem::setVisible(bool visible) {
    point->setVisible(visible);
    graphics_text->setVisible(visible);
}

void WaypointItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if(update_event & (UpdateEvent::ITEM_CHANGED | UpdateEvent::MAP_ZOOMED | UpdateEvent::MAP_ROTATED)) {
        QPointF scene_pos = scenePoint(Point2DLatLon(_waypoint), zoomLevel(map->zoom()), map->tileSize());
        double s = getScale(map->zoom(), map->scaleFactor());
        double r = map->getRotation();
        point->setPos(scene_pos);
        point->setScale(s);
        point->setRotation(-r);

        auto rot = QTransform().rotate(-r);
        graphics_text->setPlainText(_waypoint->getName());
        double y_offset = -(m_size * 1.5 + 6);//FIXME: factor 1.5 and 6 are empirical, should be based on font size and point size
        graphics_text->setPos(scene_pos + rot.map(QPointF(5*s, y_offset*s)));
        graphics_text->setScale(s);
        graphics_text->setRotation(-r);
    }
}

void WaypointItem::removeFromScene(MapWidget* map) {
    map->scene()->removeItem(point);
    map->scene()->removeItem(graphics_text);
    delete point;
    delete graphics_text;
}

void WaypointItem::setPosition(Point2DLatLon ll) {
    _waypoint->setLat(ll.lat());
    _waypoint->setLon(ll.lon());
    emit itemChanged();
    emit waypointMoved(ll);
}

void WaypointItem::update() {
    _waypoint->setLat(original_waypoint->getLat());
    _waypoint->setLon(original_waypoint->getLon());
    _waypoint->setAlt(original_waypoint->getAlt());
    _waypoint->setName(original_waypoint->getName());
    emit itemChanged();
}

void WaypointItem::commitPosition() {
    original_waypoint->setLat(_waypoint->getLat());
    original_waypoint->setLon(_waypoint->getLon());
    original_waypoint->setAlt(_waypoint->getAlt());
    emit itemChanged();
}

void WaypointItem::setStyle(GraphicsPoint::Style s){
    point->setStyle(s);
    graphics_text->setStyle(s);
    requestUpdate();
}

void WaypointItem::setAnimation(GraphicsObject::Animation a) {
    point->setAnimation(a);
}

void WaypointItem::setAnimate(bool animate) {
    if(animate) {
        point->setAnimation(GraphicsObject::Animation::WP_MOVING);
    } else {
        point->setAnimation(GraphicsObject::Animation::NONE);
    }
}

QPointF WaypointItem::scenePos() {
    if(point != nullptr) {
        return point->scenePos();
    }
    return QPointF(0,0);
}

void WaypointItem::setIgnoreEvent(bool ignore) {
    point->setIgnoreEvent(ignore);
}
