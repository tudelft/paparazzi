#ifndef GRAPHICSCIRCLE_H
#define GRAPHICSCIRCLE_H

#include <QObject>
#include <QGraphicsItem>
#include <QFont>
#include "graphics_object.h"
#include "pprzpalette.h"

enum CircleScaleState {
    CSS_IDLE,
    CSS_PRESSED,
    CSS_SCALED,
};

class GraphicsCircle : public GraphicsObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit GraphicsCircle(double radius, PprzPalette palette, int stroke, QObject *parent = nullptr);
    void setRadius(double r);
    void displayRadius(bool dpr) {display_radius = dpr;}
    void setTextPos(QPointF pos) {textPos = pos;}
    void setText(QString t) {text = t;}
    void setFilled(bool f) {filled = f;}

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QPainterPath shape() const override;

signals:
    void circleScaled(qreal size);
    void circleScaleFinished();

protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void changeFocus() override;


public slots:

private:

    double dr;
    QPointF textPos;
    double radius;
    CircleScaleState scale_state;
    QFont font_radius;

    int current_color;

    int base_stroke;
    int stroke;
    QPainterPath path_draw;
    QPainterPath path_shape;
    QString text;

    bool display_radius;
    QRectF last_bounding_rect;

    bool filled;
};

#endif // GRAPHICSCIRCLE_H


