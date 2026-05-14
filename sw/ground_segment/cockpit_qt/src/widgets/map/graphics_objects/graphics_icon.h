#pragma once

#include <QObject>
#include <QGraphicsItem>
#include "graphics_object.h"
#include <QtGui>
#include <QSvgRenderer>

class GraphicsIcon : public GraphicsObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    GraphicsIcon(QString icon_path, int size, QObject *parent = nullptr);
    void setIcon(QString filename);

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

protected:
    virtual void changeFocus() override;

private:
    int size;
    QSvgRenderer* renderer;
};
