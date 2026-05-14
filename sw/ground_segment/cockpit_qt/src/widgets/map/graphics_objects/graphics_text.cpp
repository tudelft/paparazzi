#include "graphics_text.h"
#include <QDebug>
#include <QPen>

GraphicsText::GraphicsText(const QString &text, PprzPalette palette,QObject *parent):
    GraphicsObject(palette, parent),
    QGraphicsSimpleTextItem (text)

{
    setDefaultTextColor(palette.getColor());
    
    QPen pen(Qt::black);
    pen.setWidthF(1.0);
    setPen(pen);
}

void GraphicsText::changeFocus() {
    switch (style) {
    case DEFAULT:
        if(isHighlighted()) {
            setVisible(true);
        } else {
            setVisible(false);
        }

        break;
    case CARROT:
        setVisible(false);
        break;
    case CURRENT_NAV:
        setVisible(false);
        break;
    case GCS:
        setVisible(false);
        break;
    case DCSHOT:
        setVisible(false);
        break;
    case CRASH:
        setVisible(false);
        break;
    }

    update();
}

void GraphicsText::setStyle(Style s) {
    GraphicsObject::setStyle(s);
    changeFocus();
}

#include <QPainter>
#include <QPainterPath>
#include <QFontMetrics>

void GraphicsText::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void)option;
    (void)widget;
    
    QFont f = font();
    QPointF pos(0, QFontMetricsF(f).ascent());
    
    QPainterPath path;
    path.addText(pos, f, text());
    
    // Draw outline
    painter->save();
    painter->setRenderHint(QPainter::Antialiasing);
    QPen p = pen();
    p.setWidthF(std::max(1.0, p.widthF() * 2.5));
    p.setJoinStyle(Qt::RoundJoin);
    painter->strokePath(path, p);
    painter->restore();
    
    // Draw the bright fill text using native OS hinting
    // This avoids vector subpixel aliasing muting the color
    painter->save();
    painter->setPen(QPen(brush().color()));
    painter->setFont(f);
    painter->drawText(pos, text());
    painter->restore();
}