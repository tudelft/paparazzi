#include "aircraft_item.h"
#include "mapwidget.h"
#include "AircraftManager.h"
#include <QApplication>
#include <QDebug>
#include "aircraft.h"
#include "gcs_utils.h"
#include "graphics_icon.h"


AircraftItem::AircraftItem(Point2DLatLon pt, QString ac_id, double neutral_scale_zoom, QObject *parent) :
    MapItem(ac_id, neutral_scale_zoom, parent),
    latlon(pt), heading(0.)
{
    auto settings = getAppSettings();
    track_size = settings.value("map/aircraft/track_size", 500).toInt();

    z_value_highlighted = settings.value("map/z_values/aircraft").toDouble();
    z_value_unhighlighted = settings.value("map/z_values/aircraft").toDouble();
    z_value = z_value_unhighlighted;

    auto aircraft = AircraftManager::get()->getAircraft(ac_id);
    int size = settings.value("map/aircraft/size").toInt();

    color_idle = palette.getColor();
    color_unfocused = labelUnfocusedColor(palette.getColor());

    graphics_aircraft = new GraphicsAircraft(palette.getColor(), aircraft->getIcon(), size);
    graphics_text = new GraphicsText(aircraft->name(), palette);

    if(!appConfig()->contains("NO_ALARM")) {
        alarms = new GraphicsGroup(this);
        bat_alarm = new GraphicsIcon(":/pictures/bat_low.svg", size);
        link_alarm = new GraphicsIcon(":/pictures/link_ok.svg", size);
        alarms->addToGroup(bat_alarm);
        alarms->addToGroup(link_alarm);

        auto ac = pprzApp()->toolbox()->aircraftManager()->getAircraft(ac_id);
        auto watcher = ac->getStatus()->getWatcher();
        connect(watcher, &AircraftWatcher::bat_status, this, &AircraftItem::handle_bat_alarm);
        connect(watcher, &AircraftWatcher::link_status, this, &AircraftItem::handle_link_alarm);
    } else {
        alarms = nullptr;
        bat_alarm = nullptr;
        link_alarm = nullptr;
    }

    setZoomFactor(1.1);
}

void AircraftItem::addToMap(MapWidget* map) {
    map->scene()->addItem(graphics_aircraft);
    map->scene()->addItem(graphics_text);
    if(alarms) map->scene()->addItem(alarms);
}

void AircraftItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if(update_event & (UpdateEvent::ITEM_CHANGED | UpdateEvent::MAP_ZOOMED | UpdateEvent::MAP_ROTATED)) {
        QPointF scene_pos = scenePoint(latlon, zoomLevel(map->zoom()), map->tileSize());
        graphics_aircraft->setPos(scene_pos);
        double s = getScale(map->zoom(), map->scaleFactor());
        graphics_aircraft->setScale(s);
        graphics_aircraft->setRotation(heading);

        double r = map->getRotation();
        auto rot = QTransform().rotate(-r);
        graphics_text->setScale(s);
        graphics_text->setPos(scene_pos + rot.map(QPointF(10, 10)));
        graphics_text->setRotation(-r);

        if (alarms) {
            auto dh = graphics_aircraft->boundingRect().height()/2 + alarms->boundingRect().height();

            alarms->setScale(s);
            alarms->setPos(scene_pos + rot.map(s*QPointF(-alarms->boundingRect().width()/2, -dh)));
            alarms->setRotation(-r);
        }

        for(auto &line: graphics_lines) {
            map->scene()->removeItem(line);
            delete line;
        }
        graphics_lines.clear();

        if(track_points.size() > 2) {
            auto settings = getAppSettings();
            double z_tracks = settings.value("map/z_values/aircraft").toDouble();
            int track_width = settings.value("map/aircraft/track_width").toInt();
            for(int i=0; i<track_points.size()-1; i++) {
                auto pta = scenePoint(track_points[i], zoomLevel(map->zoom()), map->tileSize());
                auto ptb = scenePoint(track_points[i+1], zoomLevel(map->zoom()), map->tileSize());
                auto line = new GraphicsLine(pta, ptb, palette, track_width);
                graphics_lines.append(line);
                line->setZValue(z_tracks);
                map->scene()->addItem(line);
            }

        }
    }
}

void AircraftItem::setPosition(Point2DLatLon pt) {
    latlon = pt;
    auto settings = getAppSettings();

    track_points.append(pt);
    while (track_points.size() > track_size && track_size > 0) {
        track_points.removeFirst();
    }

    emit itemChanged();
}

void AircraftItem::setTrackSize(int size) {
    track_size = size;
    while(track_points.size() > track_size && track_size > 0) {
        track_points.removeFirst();
    }
    emit itemChanged();
}

int AircraftItem::getTrackSize() {
    return track_size;
}

void AircraftItem::clearTrack() {
    track_points.clear();
    emit itemChanged();
}

void AircraftItem::setLabelVisible(bool visible) {
    graphics_text->setVisible(visible);
}

void AircraftItem::setHeading(double h) {
    heading = h;
    emit itemChanged();
}

void AircraftItem::setHighlighted(bool h) {
    MapItem::setHighlighted(h);
    graphics_aircraft->setHighlighted(h);
    for(auto &line: graphics_lines) {
        line->setHighlighted(h);
    }
    if(h) {
        graphics_text->setDefaultTextColor(color_idle);
    } else {
        graphics_text->setDefaultTextColor(color_unfocused);
    }

    updateZValue();
}

void AircraftItem::updateZValue() {
    auto settings = getAppSettings();
    graphics_aircraft->setZValue(z_value);
    graphics_text->setZValue(z_value);
    if(alarms) alarms->setZValue(z_value);
    double z_tracks = settings.value("map/z_values/aircraft").toDouble();
    for(auto &line: graphics_lines) {
        line->setZValue(z_tracks);
    }
}

void AircraftItem::setVisible(bool visible) {
    graphics_aircraft->setVisible(visible);
    graphics_text->setVisible(visible);
    if(alarms) alarms->setVisible(visible);
    for(auto &line: graphics_lines) {
        line->setVisible(visible);
    }
}

void AircraftItem::setForbidHighlight(bool fh) {
    graphics_aircraft->setForbidHighlight(fh);
}

void AircraftItem::removeFromScene(MapWidget* map) {
    assert(graphics_aircraft != nullptr);
    map->scene()->removeItem(graphics_aircraft);
    map->scene()->removeItem(graphics_text);
    if(alarms) map->scene()->removeItem(alarms);

    for(auto &line: graphics_lines) {
        map->scene()->removeItem(line);
        delete line;
    }
    graphics_lines.clear();

    delete graphics_aircraft;
    delete graphics_text;
    if(alarms) delete alarms;
}

void AircraftItem::handle_bat_alarm(AircraftWatcher::BatStatus bs) {
    if(!alarms) return;
    if(!alarms) return;
    if(!alarms) return;
    switch (bs) {
    case AircraftWatcher::BatStatus::CATASTROPHIC:
        bat_alarm->setIcon(":/pictures/bat_catastrophic.svg");
        bat_alarm->show();
        break;
    case AircraftWatcher::BatStatus::CRITIC:
        bat_alarm->setIcon(":/pictures/bat_critic.svg");
        bat_alarm->show();
        break;
    case AircraftWatcher::BatStatus::LOW:
        bat_alarm->setIcon(":/pictures/bat_low.svg");
        bat_alarm->show();
        break;
    case AircraftWatcher::BatStatus::OK:
        bat_alarm->hide();
        break;
    }
    alarms->arrange();
    emit itemChanged();
}

void AircraftItem::handle_link_alarm(AircraftWatcher::LinkStatus ls) {
    if(!alarms) return;
    if(!alarms) return;
    if(!alarms) return;
    switch (ls) {
    case AircraftWatcher::LinkStatus::LINK_OK:
        link_alarm->hide();
        break;
    case AircraftWatcher::LinkStatus::LINK_PARTIALY_LOST:
        link_alarm->show();
        link_alarm->setIcon(":/pictures/link_warning.svg");
        break;
    case AircraftWatcher::LinkStatus::LINK_LOST:
        link_alarm->setIcon(":/pictures/link_nok.svg");
        link_alarm->show();
        break;
    }
    alarms->arrange();
    emit itemChanged();
}
