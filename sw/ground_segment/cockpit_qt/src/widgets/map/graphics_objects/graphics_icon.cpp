#include "graphics_icon.h"
#include <QPainter>
#include <QPixmap>

GraphicsIcon::GraphicsIcon(QString icon_path, int size, QObject *parent) :
    GraphicsObject(PprzPalette(), parent),
    size(size)
{
    renderer = new QSvgRenderer(icon_path, this);
}

QRectF GraphicsIcon::boundingRect() const {
    return QRectF(0, 0, size*scale_factor, size*scale_factor);
}

void GraphicsIcon::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void)option;
    (void)widget;
    painter->setRenderHint(QPainter::Antialiasing);
    renderer->render(painter, QRect(0, 0, size*scale_factor, size*scale_factor));
}

void GraphicsIcon::changeFocus() {
    update();
}

void GraphicsIcon::setIcon(QString filename) {
    renderer->deleteLater();
    renderer = new QSvgRenderer(filename, this);
}
