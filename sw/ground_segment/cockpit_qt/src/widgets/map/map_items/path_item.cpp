#include "path_item.h"
#include "maputils.h"
#include <QGraphicsScene>
#include "mapwidget.h"
#include "map_item.h"
#include "AircraftManager.h"
#include "gcs_utils.h"

PathItem::PathItem(QString ac_id, QColor color, double neutral_scale_zoom, QObject *parent) :
    MapItem(ac_id, neutral_scale_zoom, parent),
    closing_line(nullptr), line_width(5), color(color), polygon(nullptr)
{
    if(color.isValid()) {
        palette = PprzPalette(color);
    } else {
        this->color = palette.getColor();
    }
    auto settings = getAppSettings();
    z_value_highlighted = settings.value("map/z_values/highlighted").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/unhighlighted").toDouble();
    z_value = z_value_unhighlighted;
    graphics_text = new GraphicsText("", palette, this);
    graphics_text->setZValue(z_value);
    setZoomFactor(1.1);
}

PathItem::PathItem(QString ac_id, PprzPalette palette, double neutral_scale_zoom, QObject *parent) :
    MapItem(ac_id, palette, neutral_scale_zoom, parent),
    closing_line(nullptr), line_width(5), polygon(nullptr)
{
    if(!color.isValid()) {
        this->color = palette.getColor();
    }
    auto settings = getAppSettings();
    z_value_highlighted = settings.value("map/z_values/highlighted").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/unhighlighted").toDouble();
    z_value = z_value_unhighlighted;
    graphics_text = new GraphicsText("", palette, this);
    graphics_text->setZValue(z_value);
    setZoomFactor(1.1);
}

void PathItem::addPoint(WaypointItem* wp, QColor line_color, bool own) {
    assert(wp != nullptr);
    waypoints.append(wp);

    if(own) {
        wp->setParent(this);
    }

    connect(
        wp, &WaypointItem::itemChanged, this,
        [=]() {
            emit itemChanged();
        }
    );

    if(!line_color.isValid()) {
        line_color = color;
    } 

    if(waypoints.size() > 1){
        GraphicsLine* line = new GraphicsLine(QPointF(0, 0), QPointF(0, 0), line_color, line_width, this);
        line->setIgnoreEvent(true);
        lines.append(line);
        to_be_added.append(line);
        line->setZValue(z_value - 0.5);
    }
}

void PathItem::setClosedPath(bool closed) {
    if(closed) {
        if(!closing_line) {
            // add closing line
            closing_line = new GraphicsLine(QPointF(0, 0), QPointF(0, 0), palette, line_width, this);
            closing_line->setIgnoreEvent(true);
            to_be_added.append(closing_line);
            closing_line->setZValue(z_value - 0.5);
        }
    } else {
        if(closing_line) {
            // remove closing line
            to_be_removed.append(closing_line);
            closing_line = nullptr;
        }
    }
}

void PathItem::setHighlighted(bool h) {
    MapItem::setHighlighted(h);
    graphics_text->setHighlighted(h);
    for(auto wp: waypoints) {
        wp->setHighlighted(h);
    }

    if(closing_line) {
        closing_line->setHighlighted(h);
    }
    for(auto line: lines) {
        line->setHighlighted(h);
    }
    updateZValue();
}

void PathItem::setForbidHighlight(bool sh) {
    graphics_text->setForbidHighlight(sh);
    for(auto wp: waypoints) {
        wp->setForbidHighlight(sh);
    }
    for(auto line: lines) {
        line->setForbidHighlight(sh);
    }
    if(closing_line) {
        closing_line->setForbidHighlight(sh);
    }
}

void PathItem::setEditable(bool ed) {
    for(auto wp: waypoints) {
        wp->setEditable(ed);
    }
    for(auto line: lines) {
        line->setEditable(ed);
    }
    if(closing_line) {
        closing_line->setEditable(ed);
    }
}

void PathItem::updateZValue() {
    graphics_text->setZValue(z_value);
    //waypoints above lines
    for(auto w:waypoints) {
        w->updateZValue();
    }
    for(auto l:lines) {
        l->setZValue(z_value-0.5);
    }
    if(closing_line) {
        closing_line->setZValue(z_value-0.5);
    }
    if(polygon) {
        polygon->setZValue(z_value - 0.5);
    }
}

void PathItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if(update_event & (UpdateEvent::ITEM_CHANGED | UpdateEvent::MAP_ZOOMED)) {
        // add new graphics objects to map
        while(to_be_added.size() > 0) {
            auto l = to_be_added.takeLast();
            map->scene()->addItem(l);
        }

        while(to_be_removed.size() > 0) {
            auto l = to_be_removed.takeLast();
            map->scene()->removeItem(l);
            delete l;
        }

        while(waypoints_to_remove.size() > 0) {
            auto wp = waypoints_to_remove.takeLast();
            map->removeItem(wp);
        }

        double s = getScale(map->zoom(), map->scaleFactor());
    (void)s;

        for(int i=0; i<lines.length(); i++) {
            QPointF start_scene_pos = scenePoint(waypoints[i]->position(), zoomLevel(map->zoom()), map->tileSize());
            QPointF end_scene_pos = scenePoint(waypoints[i+1]->position(), zoomLevel(map->zoom()), map->tileSize());
            lines[i]->setLine(start_scene_pos, end_scene_pos);
        }

        if(closing_line) {
            QPointF start_scene_pos = scenePoint(waypoints.first()->position(), zoomLevel(map->zoom()), map->tileSize());
            QPointF end_scene_pos = scenePoint(waypoints.last()->position(), zoomLevel(map->zoom()), map->tileSize());
            closing_line->setLine(start_scene_pos, end_scene_pos);
        }


        QPolygonF poly;
        QPointF centroid(0, 0);
        for(auto wi: waypoints) {
            auto pt = scenePoint(wi->position(), zoomLevel(map->zoom()), map->tileSize());
            poly.append(pt);
            centroid += pt;
        }
        centroid /= waypoints.size();

        if(polygon) {
            polygon->setPolygon(poly);
        }

        graphics_text->setScale(s);

        auto text_size = graphics_text->boundingRect().center();
        graphics_text->setPos(centroid - text_size*s);
    }
}

void PathItem::addToMap(MapWidget* map) {
    (void)map;
    map->scene()->addItem(graphics_text);
}

void PathItem::removeFromScene(MapWidget* map) {
    map->scene()->removeItem(graphics_text);
    delete graphics_text;

    for(auto l:lines) {
        map->scene()->removeItem(l);
        delete l;
    }
    lines.clear();

    if(closing_line) {
        map->scene()->removeItem(closing_line);
        delete closing_line;
        closing_line = nullptr;
    }

    if(polygon) {
        map->scene()->removeItem(polygon);
        delete polygon;
        polygon = nullptr;
    }

    for(auto wp: waypoints) {
        if(wp->parent() == this) {
            map->removeItem(wp);
        }
    }
    waypoints.clear();
}

void PathItem::setVisible(bool vis) {
    for(auto l:lines) {
        l->setVisible(vis);
    }

    if(closing_line) {
        closing_line->setVisible(vis);
    }

    if(polygon) {
        polygon->setVisible(vis);
    }
}

//void PathItem::setLastLineIgnoreEvents(bool ignore) {
//    if(lines.length() > 0) {
//        lines.last()->setIgnoreEvent(ignore);
//    }
//}

//void PathItem::setLinesIgnoreEvents(bool ignore) {
//    for(auto line: lines) {
//        line->setIgnoreEvent(ignore);
//    }
//}

void PathItem::removeLastWaypoint() {
    auto lastLine = lines.takeLast();
    to_be_removed.append(lastLine);
    auto wp = waypoints.takeLast();
    if(wp->parent() == this) {
        waypoints_to_remove.append(wp);
    }

    if(closing_line && waypoints.size() < 3) {
        to_be_removed.append(closing_line);
        closing_line = nullptr;
    }
}

void PathItem::setStyle(GraphicsLine::Style s) {
    graphics_text->setStyle(s);
    for(auto line: lines) {
        line->setStyle(s);
    }
    if(closing_line) {
        closing_line->setStyle(s);
    }
    if(s == GraphicsLine::Style::CURRENT_NAV) {
        for(auto w:waypoints) {
            w->setStyle(GraphicsObject::Style::CURRENT_NAV);
        }
    }
}

void PathItem::setFilled(bool f) {
    if(polygon == nullptr && f) {
        // create polygon
        polygon = new QGraphicsPolygonItem();
        polygon->setBrush(palette.getBrush());
        polygon->setPen(Qt::NoPen);
        polygon->setZValue(z_value - 0.5);
        to_be_added.append(polygon);

    } else if(polygon != nullptr && !f) {
        // remove polygon
        to_be_removed.append(polygon);
        polygon = nullptr;
    }
}
