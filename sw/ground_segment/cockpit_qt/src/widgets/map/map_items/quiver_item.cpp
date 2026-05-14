#include "quiver_item.h"
#include "gcs_utils.h"
#include "mapwidget.h"

QuiverItem::QuiverItem(QString id, QColor color, float width, QObject *parent, double neutral_scale_zoom) :
    MapItem(id, PprzPalette(color), neutral_scale_zoom, parent), pen_width(width)
{
    if(color.isValid()) {
        palette = PprzPalette(color);
    }
    
    auto settings = getAppSettings();
    z_value_highlighted = settings.value("map/z_values/shapes").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/shapes").toDouble();
    z_value = z_value_unhighlighted; 

    setZoomFactor(1.1);
}

QuiverItem::QuiverItem(Point2DLatLon pos, Point2DLatLon vpos, QString id, QColor color, float width, QObject *parent, double neutral_scale_zoom) :
    MapItem(id, PprzPalette(color), neutral_scale_zoom, parent), pen_width(width)
{
    if(color.isValid()) {
        palette = PprzPalette(color);
    }
    
    auto settings = getAppSettings();
    z_value_highlighted = settings.value("map/z_values/shapes").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/shapes").toDouble();
    z_value = z_value_unhighlighted;

    addQuiver(pos,vpos);

    setZoomFactor(1.1);
}

QuiverItem::QuiverItem(QList<Point2DLatLon> pos, QList<Point2DLatLon> vpos, QString id, QColor color, float width, QObject *parent, double neutral_scale_zoom) :
    MapItem(id, PprzPalette(color), neutral_scale_zoom, parent), pen_width(width)
{
    if(color.isValid()) {
        palette = PprzPalette(color);
    }

    auto settings = getAppSettings();
    z_value_highlighted = settings.value("map/z_values/shapes").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/shapes").toDouble();
    z_value = z_value_unhighlighted;

    for (int i=0; i<pos.size(); i++) {
        addQuiver(pos[i],vpos[i]);
    }

    setZoomFactor(1.1);
}

void QuiverItem::addQuiver(Point2DLatLon pos, Point2DLatLon vpos) {
    auto graphics_quiver = new GraphicsQuiver(palette, pen_width, this);
    graphics_quiver->setZValue(z_value);

    double distance, azimut;
    CoordinatesTransform::get()->distance_azimut(pos, vpos, distance, azimut);

    graphics_quiver->setRotation(azimut);

    graphics_quiver_l.append(graphics_quiver);
    latlon_l.append(pos);
    distance_l.append(distance);
}

void QuiverItem::addToMap(MapWidget* map) {
    foreach (GraphicsQuiver* graphics_quiver, graphics_quiver_l) {
        map->scene()->addItem(graphics_quiver);
    }
}

void QuiverItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if(update_event & (UpdateEvent::ITEM_CHANGED | UpdateEvent::MAP_ZOOMED)) {
        for (int i=0; i<graphics_quiver_l.size(); i++) {
            QPointF scene_pos = scenePoint(latlon_l[i], zoomLevel(map->zoom()), map->tileSize());
            auto size = distMeters2Tile(distance_l[i]*map->tileSize(), latlon_l[i].lat(), zoomLevel(map->zoom()));
            double scale = size/graphics_quiver_l[i]->size;

            graphics_quiver_l[i]->setPos(scene_pos);
            graphics_quiver_l[i]->setScale(scale);
        }
    }
}

void QuiverItem::removeFromScene(MapWidget* map) {
    foreach (GraphicsQuiver* graphics_quiver, graphics_quiver_l) {
        assert(graphics_quiver != nullptr);
        map->scene()->removeItem(graphics_quiver);
        delete graphics_quiver;
        graphics_quiver = nullptr;
    }
}

void QuiverItem::setHighlighted(bool sh) {
    MapItem::setHighlighted(sh);
    foreach (GraphicsQuiver* graphics_quiver, graphics_quiver_l) {
        graphics_quiver->setHighlighted(sh);
    }
}

void QuiverItem::setForbidHighlight(bool fh) {
    foreach (GraphicsQuiver* graphics_quiver, graphics_quiver_l) {
        graphics_quiver->setForbidHighlight(fh);
    }
}

void QuiverItem::setEditable(bool ed) {
    (void)ed;
}

void QuiverItem::setVisible(bool vis) {
    foreach (GraphicsQuiver* graphics_quiver, graphics_quiver_l) {
        graphics_quiver->setVisible(vis);
    }
}

void QuiverItem::updateZValue() {
    foreach (GraphicsQuiver* graphics_quiver, graphics_quiver_l) {
        graphics_quiver->setZValue(z_value);
    }
}

// TODO: Probar a eliminar el graphics item "quiver" y en su lugar generar polígonos ¿Se arreglará lo de la memoria?
// QPolygonF ArrowItem::make_polygon(double distance) {
//     double angle = M_PI/8 * exp(-distance / static_cast<double>(DISTANCE_TAU)) + M_PI/32.0;
//     QPolygonF arrow_poly({
//         QPointF(0, 0),
//         QPointF(-m_size * cos(angle),  m_size * sin(angle)),
//         QPointF(-m_size * cos(angle),  -m_size * sin(angle))
//     });
//     return arrow_poly;
// }
