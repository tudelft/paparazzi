#include "intruder_item.h"
#include "gcs_utils.h"
#include "mapwidget.h"

IntruderItem::IntruderItem(QString name, Point2DLatLon pt, double course, double neutral_scale_zoom, QObject *parent) :
    MapItem("__NO_AC__", PprzPalette(Qt::red), neutral_scale_zoom, parent),
    name(name), latlon(pt), course(course), altitude(0.0), speed(0.0), climb(0.0)
{
    auto settings = getAppSettings();

    z_value_highlighted = settings.value("map/z_values/shapes").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/shapes").toDouble();
    z_value = z_value_unhighlighted;

    int size = settings.value("map/intruder_size").toInt();

    graphics_intruder = new GraphicsIntruder(size, palette, this);
    graphics_intruder->setZValue(z_value);

    graphics_text = new GraphicsText(name, QColor(Qt::red), this);
    graphics_text->setZValue(z_value);

    setZoomFactor(1.1);

}


void IntruderItem::addToMap(MapWidget* map) {
    map->scene()->addItem(graphics_intruder);
    map->scene()->addItem(graphics_text);
}

void IntruderItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if(update_event & (UpdateEvent::ITEM_CHANGED | UpdateEvent::MAP_ZOOMED)) {
        QPointF scene_pos = scenePoint(latlon, zoomLevel(map->zoom()), map->tileSize());
        graphics_intruder->setPos(scene_pos);
        double s = getScale(map->zoom(), map->scaleFactor());
        graphics_intruder->setScale(s);
        graphics_intruder->setRotation(course);

        // Update parameters text if configured
        auto settings = getAppSettings();
        bool show_params = settings.value("map/show_intruder_params", false).toBool();
        QString display_text = name;
        if (show_params) {
            display_text += QString::asprintf("\n%+.0f m\n%.1f m/s", altitude, speed);
        }
        graphics_text->setText(display_text);

        double r = map->getRotation();
        auto rot = QTransform().rotate(-r);
        graphics_text->setPos(scene_pos + rot.map(QPointF(5*s, -30*s)));
        graphics_text->setScale(s);
        graphics_text->setRotation(-r);
    }
}

void IntruderItem::setPosition(Point2DLatLon pt) {
    latlon = pt;
    emit itemChanged();
}

void IntruderItem::setCourse(double c) {
    course = c;
    emit itemChanged();
}

void IntruderItem::setAltitude(double a) {
    altitude = a;
    emit itemChanged();
}

void IntruderItem::setSpeed(double s) {
    speed = s;
    emit itemChanged();
}

void IntruderItem::setClimb(double c) {
    climb = c;
    emit itemChanged();
}

void IntruderItem::removeFromScene(MapWidget* map) {
    assert(graphics_intruder != nullptr);
    map->scene()->removeItem(graphics_intruder);
    map->scene()->removeItem(graphics_text);
    delete graphics_intruder;
    delete graphics_text;
}

void IntruderItem::setForbidHighlight(bool fh) {
    graphics_intruder->setForbidHighlight(fh);
}

void IntruderItem::setEditable(bool ed) {
    (void)ed;
}

void IntruderItem::updateZValue() {
    graphics_intruder->setZValue(z_value);
    graphics_text->setZValue(z_value);
}

void IntruderItem::setVisible(bool visible) {
    graphics_intruder->setVisible(visible);
    graphics_text->setVisible(visible);
}
