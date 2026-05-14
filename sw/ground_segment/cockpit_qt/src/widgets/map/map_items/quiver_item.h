#ifndef QUIVERITEM_H
#define QUIVERITEM_H

#include "map_item.h"
#include "graphics_quiver.h"

class QuiverItem : public MapItem
{
    Q_OBJECT
public:
    QuiverItem(QString id, QColor = Qt::red, float width = 0.5, QObject *parent = nullptr, double neutral_scale_zoom = 15);
    QuiverItem(Point2DLatLon pos, Point2DLatLon vpos, QString id, QColor = Qt::red, float width = 0.5, QObject *parent = nullptr, double neutral_scale_zoom = 15);
    QuiverItem(QList<Point2DLatLon> pos, QList<Point2DLatLon> vpos, QString id, QColor = Qt::red, float width = 0.5, QObject *parent = nullptr, double neutral_scale_zoom = 15);

    virtual void addToMap(MapWidget* map) override;
    virtual void updateGraphics(MapWidget* map, uint32_t update_event) override;
    virtual void removeFromScene(MapWidget* map) override;
    virtual void setHighlighted(bool sh) override;
    virtual void setForbidHighlight(bool fh) override;
    virtual void updateZValue() override;
    virtual void setEditable(bool ed) override;

    virtual void setVisible(bool vis);
    void addQuiver(Point2DLatLon pos, Point2DLatLon vpos);
    void removeQuivers();

private:
    float pen_width;
    QList<GraphicsQuiver*> graphics_quiver_l;

    QList<Point2DLatLon> latlon_l;
    QList<Point2DLatLon> vlatlon_l;
    QList<double> distance_l;
};

#endif // QUIVERITEM_H
