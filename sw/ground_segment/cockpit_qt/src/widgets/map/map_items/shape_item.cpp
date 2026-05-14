#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QDebug>
#include <QFont>
#include "maputils.h"
#include "shape_item.h"
#include "mapwidget.h"
#include "AircraftManager.h"

ShapeItem::ShapeItem(int id, ShapeType type, QString ac_id)
    : MapItem(ac_id, 0), _id(id), _type(type), _graphicsItem(nullptr), _textItem(nullptr)
{
    z_value = 15;
    _type = type;
    if (_type == ShapeCircle) {
        _graphicsItem = new QGraphicsEllipseItem();
    } else if (_type == ShapePolygon) {
        _graphicsItem = new QGraphicsPolygonItem();
    } else if (_type == ShapeSegment) {
        _graphicsItem = new QGraphicsPolygonItem(); // QGraphicsLineItem doesn't map to a path easily, we'll draw it as polygon of two points (poly line)
    } else if (_type == ShapeText) {
        _graphicsItem = new QGraphicsTextItem();
    }
}

ShapeItem::~ShapeItem() {
    if (_graphicsItem) {
        delete _graphicsItem;
    }
    if (_textItem) {
        delete _textItem;
    }
}

void ShapeItem::addToMap(MapWidget* mw) {
    if (_graphicsItem) {
        mw->scene()->addItem(_graphicsItem);
        _graphicsItem->setZValue(z_value);
    }
    if (_textItem) {
        mw->scene()->addItem(_textItem);
        _textItem->setZValue(z_value + 0.1);
    }
}

void ShapeItem::removeFromScene(MapWidget* map) {
    if (_graphicsItem) {
        map->scene()->removeItem(_graphicsItem);
    }
    if (_textItem) {
        map->scene()->removeItem(_textItem);
    }
}

void ShapeItem::setVisible(bool visible) {
    if (_graphicsItem) {
        _graphicsItem->setVisible(visible);
    }
    if (_textItem) {
        _textItem->setVisible(visible);
    }
}

void ShapeItem::setHighlighted(bool h) {
}

void ShapeItem::updateZValue() {
    if (_graphicsItem) {
        _graphicsItem->setZValue(z_value);
    }
    if (_textItem) {
        _textItem->setZValue(z_value + 0.1);
    }
}

void ShapeItem::updateShape(QList<Point2DLatLon> points, double radius, QString text, QColor lineColor, QColor fillColor, int opacity) {
    _points = points;
    _radius = radius;
    _text = text;
    fillColor.setAlpha(opacity);
    lineColor.setAlpha(opacity);

    if (text != "NULL" && !text.isEmpty() && _type != ShapeText) {
        if (!_textItem) {
            _textItem = new QGraphicsTextItem();
            _textItem->setZValue(z_value + 0.1);
            if (_graphicsItem && _graphicsItem->scene()) {
                _graphicsItem->scene()->addItem(_textItem);
            }
        }
        _textItem->setPlainText(text);
        _textItem->setDefaultTextColor(lineColor);
        _textItem->setFont(QFont("Arial", 14, QFont::Bold));
        _textItem->setVisible(true);
    } else if (_textItem) {
        _textItem->setVisible(false);
    }

    if (_type == ShapeCircle) {
        auto* circle = dynamic_cast<QGraphicsEllipseItem*>(_graphicsItem);
        circle->setPen(QPen(lineColor, 2));
        circle->setBrush(QBrush(fillColor));
    } else if (_type == ShapePolygon) {
        auto* poly = dynamic_cast<QGraphicsPolygonItem*>(_graphicsItem);
        poly->setPen(QPen(lineColor, 2));
        poly->setBrush(QBrush(fillColor));
    } else if (_type == ShapeSegment) {
        auto* poly = dynamic_cast<QGraphicsPolygonItem*>(_graphicsItem);
        poly->setPen(QPen(lineColor, 2));
        poly->setBrush(Qt::NoBrush);
    } else if (_type == ShapeText) {
        auto* txt = dynamic_cast<QGraphicsTextItem*>(_graphicsItem);
        txt->setPlainText(text);
        txt->setDefaultTextColor(fillColor);
        txt->setFont(QFont("Arial", 14, QFont::Bold));
    }
    
    emit itemChanged();
}

void ShapeItem::updateGraphics(MapWidget* map, uint32_t update_event) {
    if (!_graphicsItem || _points.isEmpty()) return;

    if (_textItem && _textItem->isVisible()) {
        _textItem->setPos(scenePoint(_points[0], zoomLevel(map->zoom()), map->tileSize()));
    }

    if (_type == ShapeCircle) {
        auto* circle = dynamic_cast<QGraphicsEllipseItem*>(_graphicsItem);
        QPointF p = scenePoint(_points[0], zoomLevel(map->zoom()), map->tileSize());
        // Radius scale depends on zoom level
        int zoom = map->zoom();
        double wgs84_a = 6378137.0;
        double r = _radius * (256 * pow(2.0, zoom)) / (2 * M_PI * wgs84_a * cos(_points[0].lat() * M_PI / 180.0));
        circle->setRect(p.x() - r, p.y() - r, r*2, r*2);
    } else if (_type == ShapePolygon || _type == ShapeSegment) {
        auto* poly = dynamic_cast<QGraphicsPolygonItem*>(_graphicsItem);
        QPolygonF qp;
        for (const auto& pt : _points) {
            qp << scenePoint(pt, zoomLevel(map->zoom()), map->tileSize());
        }
        poly->setPolygon(qp);
    } else if (_type == ShapeText) {
        auto* txt = dynamic_cast<QGraphicsTextItem*>(_graphicsItem);
        txt->setPos(scenePoint(_points[0], zoomLevel(map->zoom()), map->tileSize()));
    }
}
