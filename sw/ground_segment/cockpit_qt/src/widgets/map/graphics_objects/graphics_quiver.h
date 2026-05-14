#ifndef GRAPHICSQUIVER_H
#define GRAPHICSQUIVER_H

#include <QGraphicsItem>
#include <QPainter>
#include "graphics_object.h"

#define COLOR_IDLE 0
#define COLOR_UNFOCUSED 1

class GraphicsQuiver : public GraphicsObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit GraphicsQuiver(PprzPalette palette, float width, QObject *parent = nullptr);

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    float size = 20;

protected:
    virtual void changeFocus() override;

private:
    float width;
    uint8_t current_color = COLOR_IDLE;
};

#endif // GRAPHICSQUIVER_H