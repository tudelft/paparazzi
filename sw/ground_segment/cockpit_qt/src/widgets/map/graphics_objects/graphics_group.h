#ifndef GRAPHICS_GROUP_H
#define GRAPHICS_GROUP_H

#include <QObject>
#include <QGraphicsItem>
#include "graphics_object.h"
#include <QBrush>
#include "pprzpalette.h"

class GraphicsGroup : public GraphicsObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit GraphicsGroup(QObject *parent = nullptr);
    explicit GraphicsGroup(PprzPalette palette, QObject *parent = nullptr);

    QRectF boundingRect() const override;

    void arrange();

signals:
    void mousePressed(QGraphicsSceneMouseEvent *event);
    void mouseMoved(QGraphicsSceneMouseEvent *event);
    void mouseReleased(QGraphicsSceneMouseEvent *event);
    void mouseDoubleClicked(QGraphicsSceneMouseEvent *event);

protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void changeFocus() override;

};

#endif // GRAPHICS_GROUP_H
