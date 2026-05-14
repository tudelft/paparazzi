#include <QPen>
#include <QApplication>
#include <QDebug>
#include "maputils.h"
#include <QGraphicsScene>
#include "circle_item.h"
#include <cmath>
#include "mapwidget.h"
#include "map_item.h"
#include "AircraftManager.h"
#include "gcs_utils.h"

CircleItem::CircleItem(WaypointItem* center, double radius, QString ac_id, double neutral_scale_zoom):
  MapItem(ac_id, neutral_scale_zoom),
  center(center), _radius(radius)
{
    init(center);
}

CircleItem::CircleItem(WaypointItem* center, double radius, QString ac_id, PprzPalette palette, double neutral_scale_zoom):
  MapItem(ac_id, palette, neutral_scale_zoom),
  center(center), _radius(radius)
{
    init(center);
}

void CircleItem::init(WaypointItem* center) {

    auto settings = getAppSettings();
    stroke = settings.value("map/circle/stroke").toInt();
    z_value_highlighted = settings.value("map/z_values/highlighted").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/unhighlighted").toDouble();
    z_value = z_value_unhighlighted;

    circle = new GraphicsCircle(0, palette, stroke, this);
    circle->setPos(center->scenePos());
    circle->setZValue(center->zValue() + 0.5);

    graphics_text = new GraphicsText("", palette, this);
    graphics_text->setZValue(z_value + 0.5);

    setZoomFactor(1.1);

    // dependence over center: if center changed, so do the CircleItem.
    connect(
        center, &WaypointItem::itemChanged, this,
        [=]() {
            emit itemChanged();
        }
    );

    connect(
        circle, &GraphicsCircle::objectClicked, this,
        [=](QPointF scene_pos) {
            emit itemClicked(scene_pos);
            // qDebug() << "circle clicked at " << scene_pos;
        }
    );

    connect(
        circle, &GraphicsCircle::objectGainedHighlight, this,
        [=]() {
            setHighlighted(true);
            emit itemGainedHighlight();
        }
    );

    connect(
        center, &MapItem::itemGainedHighlight, this,
        [=]() {
            setHighlighted(true);
            emit itemGainedHighlight();
        }
    );
}

void CircleItem::addToMap(MapWidget* map) {
    map->scene()->addItem(circle);
    map->scene()->addItem(graphics_text);

    connect(
        circle, &GraphicsCircle::circleScaled, this,
        [=](qreal size) {
            _radius = distTile2Meters(circle->pos().y()/map->tileSize(), size/map->tileSize(), zoomLevel(map->zoom()));
            circle->setText(QString::number(static_cast<int>(_radius)) + "m");
            emit circleScaled(_radius);
        }
    );
}


void CircleItem::setHighlighted(bool h) {
    MapItem::setHighlighted(h);
    center->setHighlighted(h);
    circle->setHighlighted(h);
    graphics_text->setHighlighted(h);

    updateZValue();
}

void CircleItem::setForbidHighlight(bool fh) {
    center->setForbidHighlight(fh);
    circle->setForbidHighlight(fh);
}

void CircleItem::setEditable(bool ed) {
    circle->setEditable(ed);
}

void CircleItem::updateZValue() {
    //the circle is above the waypoint
    center->updateZValue();
    circle->setZValue(z_value+0.5);
    graphics_text->setZValue(z_value + 0.5);
}

void CircleItem::setVisible(bool visible) {
    circle->setVisible(visible);
    graphics_text->setVisible(visible);
    if(ownCenter()) {
        center->setVisible(visible);
    }
}

void CircleItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if(update_event & (UpdateEvent::ITEM_CHANGED | UpdateEvent::MAP_ZOOMED)) {
        double s = getScale(map->zoom(), map->scaleFactor());
        circle->setScaleFactor(s);

        auto pos = center->position();
        QPointF scene_pos = scenePoint(pos, zoomLevel(map->zoom()), map->tileSize());
        circle->setPos(scene_pos);

        double pixelRadius = distMeters2Tile(_radius, center->position().lat(), zoomLevel(map->zoom()))*map->tileSize();
        circle->setRadius(pixelRadius);


        double r = map->getRotation();
        auto rot = QTransform().rotate(-r);
        auto text_size = graphics_text->boundingRect().center();
        graphics_text->setPos(scene_pos + rot.map(-text_size*s));
        graphics_text->setScale(s);
        graphics_text->setRotation(-r);
    }
}

void CircleItem::removeFromScene(MapWidget* map) {
    map->scene()->removeItem(circle);
    map->scene()->removeItem(graphics_text);
    delete circle;
    delete graphics_text;
    if(center && center->parent() == this) {    // remove the waypoint only if it owns it.
        map->removeItem(center);
        center = nullptr;
    }
}

void CircleItem::setRadius(double radius) {
    _radius = radius;

    emit itemChanged();
}

void CircleItem::setText(QString text) {
    graphics_text->setPlainText(text);
}

void CircleItem::setStyle(GraphicsCircle::Style s) {
    circle->setStyle(s);
    graphics_text->setStyle(s);
    if(s == GraphicsCircle::Style::CURRENT_NAV) {
        center->setStyle(GraphicsObject::Style::CURRENT_NAV);
    }
}
