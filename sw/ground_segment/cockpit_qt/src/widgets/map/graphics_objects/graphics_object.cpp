#include "graphics_object.h"
#include <QDebug>

GraphicsObject::GraphicsObject(PprzPalette palette, QObject *parent) : QObject(parent),
    editable(true), scale_factor(1.0), ignore_events(false), style(DEFAULT), animation(NONE), palette(palette), highlighted(true)
{
    animation_timer = new QTimer(this);
}


void GraphicsObject::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    (void)event;
    if(!highlighted && !forbid_highlight) {
        highlighted = true;
        emit objectGainedHighlight();
    }
}

void GraphicsObject::setHighlighted(bool h) {
    highlighted = h;
    changeFocus();
}

void GraphicsObject::setAnimation(Animation a) {
    animation = a;
    if(a == NONE && animation_timer->isActive()) {
        animation_timer->stop();
    } else if(a != NONE && !animation_timer->isActive()) {
        animation_timer->start();
    }
}
