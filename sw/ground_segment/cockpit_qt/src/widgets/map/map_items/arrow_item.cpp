#include "arrow_item.h"
#include "mapwidget.h"
#include "gcs_utils.h"
#include "dispatcher_ui.h"


constexpr double DISTANCE_TAU = 200;

ArrowItem::ArrowItem(QString ac_id, double neutral_scale_zoom, QObject *parent) :
    MapItem(ac_id, neutral_scale_zoom, parent), m_size(30)
{
    auto settings = getAppSettings();

    z_value_highlighted = settings.value("map/z_values/aircraft").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/aircraft").toDouble();
    z_value = z_value_unhighlighted;

    scene_item = new GraphicsGroup(palette, this);
    polygon = new QGraphicsPolygonItem(scene_item);
    connect(scene_item, &GraphicsGroup::mousePressed, this, [=](auto event) {
        (void)event;
        emit centerAC();
        emit DispatcherUi::get()->ac_selected(ac_id);
    });

    scene_item->addToGroup(polygon);
    setZoomFactor(1.1);
}


void ArrowItem::setHighlighted(bool h) {
    MapItem::setHighlighted(h);
    if(h) {
        polygon->setBrush(palette.getColor());
    } else {
        polygon->setBrush(palette.getVariant(1));
    }
    updateZValue();
}

void ArrowItem::updateZValue() {
    //polygon->setZValue(z_value);
    scene_item->setZValue(z_value);
}

void ArrowItem::setVisible(bool visible) {
    scene_item->setVisible(visible);
}

void ArrowItem::setForbidHighlight(bool fh) {
    (void)fh;
}

void ArrowItem::removeFromScene(MapWidget* map) {
    assert(scene_item != nullptr);
    map->scene()->removeItem(scene_item);
    delete scene_item;
    scene_item = nullptr;
}
void ArrowItem::addToMap(MapWidget* map) {
    map->scene()->addItem(scene_item);
}

void ArrowItem::updateGraphics(MapWidget* map, uint32_t update_event) {

    if(update_event & (UpdateEvent::ANY)) {
        QPointF ac_scene_pos = scenePoint(ac_pos, zoomLevel(map->zoom()), map->tileSize());
        QPoint ac_view_pos = map->mapFromScene(ac_scene_pos);
        if(!map->rect().contains(ac_view_pos)) {
            //qDebug() << ac_view_pos;
            auto [arrow_view_pos, angle, distance] = intersect(map->rect(), ac_view_pos);
            QPolygonF arrow_poly = make_polygon(distance);
            polygon->setPolygon(arrow_poly);
            QPointF arrow_scene_pos = map->mapToScene(arrow_view_pos);
            polygon->setPos(arrow_scene_pos);
            double s = getScale(map->zoom(), map->scaleFactor());
            polygon->setScale(s);
            polygon->setRotation(angle - map->getRotation());
            polygon->show();
        } else {
            polygon->hide();
        }
    }
}

std::tuple<QPoint, double, double> ArrowItem::intersect(QRect rect, QPoint p) {
    if(rect.contains(p)) {
        return std::make_tuple(p, -90.0, 0);
    }

    double x = 0;
    double y = 0;
    double angle_deg = 0;

    auto c = rect.center();
    // avoid divide by 0
    if(c.x() == p.x()) {
        x = c.x();
        if(p.y() >= c.y()) {
            y = rect.bottom();
            angle_deg = 90.0;
        } else {
            y = rect.top();
            angle_deg = -90.0;
        }
    } else {
        double slope = static_cast<double>(p.y() - c.y()) / static_cast<double>(p.x() - c.x());
        double slope_corner = static_cast<double>(rect.height()) / static_cast<double>(rect.width());
        double angle = atan2(p.y() - c.y(), p.x() - c.x());
        angle_deg = qRadiansToDegrees(angle);


        if(abs(slope) < abs(slope_corner)) {
            // right of left edge
            if(p.x() > c.x()) {
                // right edge
                x = rect.right();
                y = c.y() + slope * rect.width()/2;

            } else {
                // left edge
                x = rect.left();
                y = c.y() - slope * rect.width()/2;
            }
        } else {
            // top or bottom edge
            if(p.y() > c.y()) {
                // bottom
                y = rect.bottom();
                x = c.x() + rect.height()/(2.0*slope);
            } else {
                // top
                y = rect.top();
                x = c.x() - rect.height()/(2.0*slope);
            }
        }

    }

    auto inter = QPoint(x, y);
    double distance = sqrt(pow(p.x()-inter.x(), 2) + pow(p.y()-inter.y(), 2));
    return std::make_tuple(inter, angle_deg, distance);
}

QPolygonF ArrowItem::make_polygon(double distance) {
    double angle = M_PI/8 * exp(-distance / static_cast<double>(DISTANCE_TAU)) + M_PI/32.0;
    QPolygonF arrow_poly({
        QPointF(0, 0),
        QPointF(-m_size * cos(angle),  m_size * sin(angle)),
        QPointF(-m_size * cos(angle),  -m_size * sin(angle))
    });
    return arrow_poly;
}
