#ifndef GRAPHICSPOINT_H
#define GRAPHICSPOINT_H

#include <QObject>
#include <QGraphicsItem>
#include "graphics_object.h"
#include <QBrush>
#include "pprzpalette.h"

enum PointMoveState {
    PMS_IDLE,
    PMS_PRESSED,
    PMS_MOVED,
};

class GraphicsPoint : public GraphicsObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
    Q_PROPERTY(int size MEMBER halfSize)
public:
    explicit GraphicsPoint(int size, PprzPalette palette, QObject *parent = nullptr);

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QPainterPath shape() const override;

signals:
    void pointMoved(QPointF scenePos);
    void pointMoveFinished(QPointF scenePos);

protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void changeFocus() override;



public slots:

private:
    int halfSize;
    QPointF pressPos;
    PointMoveState move_state;

    int current_color;

    int animation_couter;
};

#endif // GRAPHICSPOINT_H
