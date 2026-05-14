#include "graphics_intruder.h"
#include <QPainter>
#include <qmath.h>

GraphicsIntruder::GraphicsIntruder(int size, PprzPalette palette, QObject *parent) :
    GraphicsObject(palette, parent),
    QGraphicsItem (),
    size(size)
{

}

QRectF GraphicsIntruder::boundingRect() const {
    return QRectF(-size, -size, 2*size, 2*size);
}

void GraphicsIntruder::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void)option;
    (void)widget;
    painter->setPen(QPen(Qt::red, 3));

    painter->drawEllipse(QPoint(0, 0), size/3, size/3);

    painter->drawLine(0, 0, 0, -size);

    painter->drawLine(0, -size, -size/10, -9*size/10);
    painter->drawLine(0, -size, size/10, -9*size/10);
}


void GraphicsIntruder::changeFocus() {

}
