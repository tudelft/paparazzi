#ifndef SHAPEITEM_H
#define SHAPEITEM_H

#include "map_item.h"
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>
#include <QVector>

class MapWidget;

enum ShapeType {
    ShapeCircle = 0,
    ShapePolygon = 1,
    ShapeSegment = 2,
    ShapeText = 3
};

class ShapeItem : public MapItem
{
    Q_OBJECT
public:
    explicit ShapeItem(int id, ShapeType type, QString ac_id = "SHAPE");
    virtual ~ShapeItem();

    virtual void addToMap(MapWidget* mw);
    virtual void setHighlighted(bool h);
    virtual void setForbidHighlight(bool fh) {}
    virtual void setEditable(bool ed) {}
    virtual void updateZValue();
    virtual void setVisible(bool visible);
    virtual void updateGraphics(MapWidget* map, uint32_t update_event);
    virtual void removeFromScene(MapWidget* map);

    void updateShape(QList<Point2DLatLon> points, double radius, QString text, QColor lineColor, QColor fillColor, int opacity);

private:
    int _id;
    ShapeType _type;
    QList<Point2DLatLon> _points;
    double _radius;
    QString _text;
    
    QGraphicsItem* _graphicsItem;
    QGraphicsTextItem* _textItem;
};

#endif // SHAPEITEM_H
