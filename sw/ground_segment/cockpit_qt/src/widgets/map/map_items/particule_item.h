#ifndef PARTICULEITEM_H
#define PARTICULEITEM_H

#include "map_item.h"
#include <QGraphicsEllipseItem>

class MapWidget;

class ParticuleItem : public MapItem
{
    Q_OBJECT
public:
    explicit ParticuleItem(int id, Point2DLatLon pos, int value, QString ac_id, double neutral_scale_zoom = 15);
    virtual void addToMap(MapWidget* mw);
    virtual void setHighlighted(bool h);
    virtual void updateZValue();
    virtual void setVisible(bool visible);
    virtual void setForbidHighlight(bool fh) {}
    virtual void setEditable(bool ed) {}
    virtual void updateGraphics(MapWidget* map, uint32_t update_event);
    virtual void removeFromScene(MapWidget* map);

    void updateParticule(Point2DLatLon pos, int value);
    int getId() { return _id; }

private:
    int _id;
    Point2DLatLon _pos;
    int _value;
    QGraphicsEllipseItem* _ellipse;

    QColor getColorForValue(int value);
};

#endif // PARTICULEITEM_H
