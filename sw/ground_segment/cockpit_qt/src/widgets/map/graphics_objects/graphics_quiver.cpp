#include "graphics_quiver.h"
#include <QDebug>

GraphicsQuiver::GraphicsQuiver(PprzPalette palette, float width, QObject *parent) : 
    GraphicsObject(palette, parent),
    QGraphicsItem (),
    width(width*2)
{

}

QRectF GraphicsQuiver::boundingRect() const
{
    return QRectF(-size, -size, 2*size, 2*size);
}

void GraphicsQuiver::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    (void)option;
    (void)widget;
    painter->setPen(QPen(palette.getVariant(current_color),width));
    painter->drawLine(0, 0, 0, -size + size/40);

    painter->drawLine(0, -size, -size/10, -9*size/10);
    painter->drawLine(0, -size, size/10, -9*size/10);
}

void GraphicsQuiver::changeFocus() {

    if(isHighlighted()) {
        current_color = COLOR_IDLE;     
        //setVisible(true);
    } else {
        current_color = COLOR_UNFOCUSED;
        //setVisible(false);
    }

    update();
}