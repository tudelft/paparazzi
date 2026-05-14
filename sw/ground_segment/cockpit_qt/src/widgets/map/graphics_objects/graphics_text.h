#ifndef GRAPHICS_TEXT_H
#define GRAPHICS_TEXT_H

#include <QGraphicsSimpleTextItem>
#include <QPen>
#include <QBrush>
#include "graphics_object.h"

class GraphicsText : public GraphicsObject, public QGraphicsSimpleTextItem
{
public:
    GraphicsText(const QString &text, PprzPalette palette,QObject *parent = nullptr);
    void setStyle(Style s);
    void setDefaultTextColor(const QColor& color) { setBrush(QBrush(color)); }
    void setPlainText(const QString& text) { setText(text); }

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
protected:
    virtual void changeFocus() override;

};

#endif // GRAPHICS_TEXT_H
