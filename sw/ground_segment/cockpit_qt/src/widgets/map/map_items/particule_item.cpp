#include "particule_item.h"
#include <QPen>
#include <QBrush>
#include "mapwidget.h"
#include "maputils.h"

ParticuleItem::ParticuleItem(int id, Point2DLatLon pos, int value, QString ac_id, double neutral_scale_zoom) :
    MapItem(ac_id, PprzPalette(Qt::black), neutral_scale_zoom),
    _id(id), _pos(pos), _value(value)
{
    // Corresponds to the ellipse creation in particule.ml
    // let p = GnoCanvas.ellipse ~fill_color ~props:[`WIDTH_PIXELS 1; `OUTLINE_COLOR "black"] ~x1:(-3.) ~y1:(-3.) ~x2:3. ~y2:3. group
    _ellipse = new QGraphicsEllipseItem(-3, -3, 6, 6);
    _ellipse->setPen(QPen(Qt::black, 1));
    _ellipse->setBrush(QBrush(getColorForValue(_value)));
}

void ParticuleItem::addToMap(MapWidget* map) {
    map->scene()->addItem(_ellipse);
    _ellipse->setZValue(100); // Usually raise to top 
}

void ParticuleItem::setHighlighted(bool h) {
    MapItem::setHighlighted(h);
}

void ParticuleItem::updateZValue() {
    _ellipse->setZValue(z_value);
}

void ParticuleItem::setVisible(bool visible) {
    _ellipse->setVisible(visible);
}

void ParticuleItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if(update_event & (UpdateEvent::ITEM_CHANGED | UpdateEvent::MAP_ZOOMED)) {
        QPointF scene_pos = scenePoint(_pos, zoomLevel(map->zoom()), map->tileSize());
        _ellipse->setPos(scene_pos);
    }
}

void ParticuleItem::removeFromScene(MapWidget* map) {
    map->scene()->removeItem(_ellipse);
    delete _ellipse;
}

QColor ParticuleItem::getColorForValue(int x) {
    // 255 -> red, 0 -> blue
    // let x = if x < 0 then 0 else if x > 255 then 255 else x in
    // sprintf "#%02x%02x%02x" x 0 (255-x)
    if (x < 0) x = 0;
    if (x > 255) x = 255;
    return QColor(x, 0, 255 - x);
}

void ParticuleItem::updateParticule(Point2DLatLon pos, int value) {
    _pos = pos;
    _value = value;
    _ellipse->setBrush(QBrush(getColorForValue(_value)));
    emit itemChanged();
}
