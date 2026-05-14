#include "map_item.h"
#include "math.h"
#include <QApplication>
#include <QDebug>
#include <utility>
#include "mapwidget.h"

#include "AircraftManager.h"

MapItem::MapItem(QString ac_id, double neutral_scale_zoom, QObject *parent) :
    QObject(parent),
    ac_id(ac_id),
    zoom_factor(1), neutral_scale_zoom(neutral_scale_zoom),
    z_value(0),
    visible(true)
{
    auto color = AircraftManager::get()->getAircraft(ac_id)->getColor();
    this->palette = PprzPalette(color);
}

MapItem::MapItem(QString ac_id, PprzPalette palette, double neutral_scale_zoom, QObject *parent) :
    QObject(parent),
    ac_id(ac_id), palette(palette),
    zoom_factor(1), neutral_scale_zoom(neutral_scale_zoom),
    z_value(0),
    visible(true)
{
}

QColor MapItem::unfocusedColor(const QColor &color) {
    int h, s, v, a;
    color.getHsv(&h, &s, &v, &a);

    int a2 = qMin(static_cast<int>(a/2), 255);
    int s3 = static_cast<int>(s/2);
    QColor c = QColor(color);
    c.setHsv(h, s3, v, a2);

    return c;
}

QColor MapItem::trackUnfocusedColor(const QColor &color) {
    int h, s, v, a;
    color.getHsv(&h, &s, &v, &a);

    int s3 = static_cast<int>(s/2);
    QColor c = QColor(color);
    c.setHsv(h, s3, v, a);

    return c;
}

QColor MapItem::labelUnfocusedColor(const QColor &color) {
    int h, s, v, a;
    color.getHsv(&h, &s, &v, &a);

    int v2 = static_cast<int>(v/2);
    QColor c = QColor(color);
    c.setHsv(h, s, v2, a);

    return c;
}

double MapItem::getScale(double zoom, double scale_factor) {
    return pow(zoom_factor, zoom - neutral_scale_zoom)/scale_factor;
}

