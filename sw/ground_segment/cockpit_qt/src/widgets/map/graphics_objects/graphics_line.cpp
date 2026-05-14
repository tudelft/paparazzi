#include "graphics_line.h"
#include <QDebug>
#include <QPainter>
#include <QApplication>
#include <QSettings>
#include "gcs_utils.h"

#define COLOR_IDLE 0
#define COLOR_UNFOCUSED 1

GraphicsLine::GraphicsLine(QPointF a, QPointF b, PprzPalette palette, int stroke, QObject *parent) :
    GraphicsObject(palette, parent),
    QGraphicsItem (),
    A(a), B(b),
    base_stroke(stroke), stroke(stroke)
{
    current_color = COLOR_IDLE;
}


QRectF GraphicsLine::boundingRect() const {
    QPointF topLeft = QPointF(qMin(A.x(), B.x()), qMin(A.y(), B.y()));
    QPointF bottomRight = QPointF(qMax(A.x(), B.x()), qMax(A.y(), B.y()));
    QRectF rect = QRectF(topLeft, bottomRight);
    rect = rect.united(last_bounding_rect);
    return rect;
}


void GraphicsLine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void)option;
    (void)widget;

    if(style == Style::DEFAULT) {
        painter->setPen(QPen(palette.getVariant(current_color), stroke));
    } else if(style == Style::CURRENT_NAV) {
        painter->setPen(QPen(Qt::green));
    }

    painter->drawLine(A, B);

    QPointF topLeft = QPointF(qMin(A.x(), B.x()), qMin(A.y(), B.y()));
    QPointF bottomRight = QPointF(qMax(A.x(), B.x()), qMin(A.y(), B.y()));
    last_bounding_rect = QRectF(topLeft, bottomRight);
}

QPainterPath GraphicsLine::shape() const {
    QPainterPath path;
    path.moveTo(A);
    path.lineTo(B);
    QPainterPathStroker pps;
    pps.setWidth(stroke);
    QPainterPath p = pps.createStroke(path);
    p.addPath(path);
    return p;
}

void GraphicsLine::changeFocus() {
    auto settings = getAppSettings();
    if(!isHighlighted()) {
        current_color = COLOR_UNFOCUSED;
        stroke = static_cast<int>(base_stroke / settings.value("map/size_highlight_factor").toDouble());
    } else {
        stroke = base_stroke;
        current_color = COLOR_IDLE;
    }
}

void GraphicsLine::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
    GraphicsObject::mousePressEvent(event);
}

void GraphicsLine::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
}

void GraphicsLine::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
    (void)event;
}

void GraphicsLine::setLine(QPointF a, QPointF b) {
    A=a;
    B=b;
    prepareGeometryChange();
}
