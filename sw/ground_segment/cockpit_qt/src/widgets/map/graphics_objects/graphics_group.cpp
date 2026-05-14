#include "graphics_group.h"
#include <QDebug>
#include <numeric>

GraphicsGroup::GraphicsGroup(QObject *parent) :
    GraphicsGroup(PprzPalette(), parent)
{}

GraphicsGroup::GraphicsGroup(PprzPalette palette, QObject *parent) :
    GraphicsObject(palette, parent),
    QGraphicsItemGroup()
{

}

void GraphicsGroup::arrange() {
    auto items = childItems();

    QSizeF max_size = std::accumulate(items.begin(), items.end(), QSizeF(0, 0), [](QSizeF s, QGraphicsItem* i) {
        auto is = i->boundingRect().size();
        return s.expandedTo(is);
    });

    int x = 0;
    for(auto &item: childItems()) {
        auto s = item->boundingRect().size();
        auto h = s.height();
        int y = (max_size.height() - h) / 2;

        if(item->isVisible()) {
            item->setPos(x, y);
            x += s.width();
        } else {
            item->setPos(0, y);
        }
    }
}

QRectF GraphicsGroup::boundingRect() const
{
   return childrenBoundingRect();
}

void GraphicsGroup::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    emit mousePressed(event);
}

void GraphicsGroup::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    emit mouseMoved(event);
}

void GraphicsGroup::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    emit mouseReleased(event);
}

void GraphicsGroup::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    emit mouseDoubleClicked(event);
}

void GraphicsGroup::changeFocus() {

}
