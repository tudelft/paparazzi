#ifndef ARROWITEM_H
#define ARROWITEM_H

#include <QObject>
#include "map_item.h"
#include "graphics_group.h"

class ArrowItem : public MapItem
{
    Q_OBJECT
//Q_PROPERTY(int size MEMBER m_size)
    Q_PROPERTY(int size MEMBER m_size DESIGNABLE true)
public:
    explicit ArrowItem(QString ac_id, double neutral_scale_zoom = 15, QObject *parent = nullptr);

    virtual void setHighlighted(bool h);
    virtual void updateZValue();
    virtual void setVisible(bool visible);
    virtual void setForbidHighlight(bool fh);
    virtual void setEditable(bool ed){(void)ed;}
    virtual void removeFromScene(MapWidget* map);
    virtual void addToMap(MapWidget* mw);

    void setAcPos(Point2DLatLon pos) {
        ac_pos = pos;
        emit itemChanged();
    }

signals:
    void centerAC();

protected:
    virtual void updateGraphics(MapWidget* map, uint32_t update_event);

private:

    std::tuple<QPoint, double, double> intersect(QRect rect, QPoint p);
    QPolygonF make_polygon(double distance);

    int m_size;
    GraphicsGroup* scene_item;
    QGraphicsPolygonItem* polygon;
    Point2DLatLon ac_pos;

};

#endif // ARROWITEM_H
