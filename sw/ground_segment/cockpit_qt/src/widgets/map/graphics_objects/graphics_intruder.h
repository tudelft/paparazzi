#ifndef GRAPHICSINTRUDER_H
#define GRAPHICSINTRUDER_H

#include <QGraphicsItem>
#include "graphics_object.h"

class GraphicsIntruder : public GraphicsObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit GraphicsIntruder(int size, PprzPalette palette, QObject *parent = nullptr);

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    //QPainterPath shape() const override;

protected:
    virtual void changeFocus() override;

private:
    int size;
};

#endif // GRAPHICSINTRUDER_H
