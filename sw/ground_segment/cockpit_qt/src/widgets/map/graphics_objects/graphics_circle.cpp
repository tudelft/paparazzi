#include "graphics_circle.h"
#include "math.h"
#include <QApplication>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QPen>
#include <QColor>
#include <QPainter>
#include <QPainterPath>
#include "gcs_utils.h"

#define COLOR_IDLE 0
#define COLOR_PRESSED 1
#define COLOR_SCALING 2
#define COLOR_UNFOCUSED 3

QPointF TEXT_OFFSET(10, -10);

GraphicsCircle::GraphicsCircle(double radius, PprzPalette palette, int stroke, QObject *parent) :
        GraphicsObject(palette, parent),
        QGraphicsItem (),
        radius(radius), scale_state(CSS_IDLE),
        current_color(0),
        base_stroke(stroke), stroke(stroke), text(), display_radius(true), filled(false)
{
    font_radius.setWeight(QFont::DemiBold);
}

QRectF GraphicsCircle::boundingRect() const {
    double out_draw = radius + stroke*scale_factor;
    QRectF outter_draw = QRectF(-out_draw, -out_draw, 2*out_draw, 2*out_draw);

    QFontMetrics fm(font_radius);
    auto t_size = fm.size(Qt::TextSingleLine, text);
    auto t_rect = QRect(textPos.x(), textPos.y()-20, t_size.width(), t_size.height()+20);
    auto bounding = outter_draw.united(t_rect);
    return bounding;

}


void GraphicsCircle::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void)option;
    (void)widget;
    path_draw = QPainterPath();
    auto settings = getAppSettings();

    double out_draw = radius + stroke*scale_factor;
    QRectF outter_draw = QRectF(-out_draw, -out_draw, 2*out_draw, 2*out_draw);
    double in_draw = radius - stroke*scale_factor;
    QRectF inner_draw = QRectF(-in_draw, -in_draw, 2*in_draw, 2*in_draw);

    if(style == DEFAULT) {
        path_draw.addEllipse(outter_draw);
        path_draw.addEllipse(inner_draw);
    } else if (style == CURRENT_NAV) {
        path_draw.addEllipse(QRectF(-radius, -radius, 2*radius, 2*radius));
    }

    path_shape = QPainterPath();
    path_shape.addEllipse(outter_draw);
    if(!filled) {
        path_shape.addEllipse(inner_draw);
    }

    if(style == DEFAULT) {
        painter->setBrush(QBrush(palette.getVariant(current_color)));
        painter->setPen(Qt::NoPen);
        painter->drawPath(path_draw);
    } else if (style == CURRENT_NAV) {
        painter->setPen(QPen(Qt::green));
        painter->drawPath(path_draw);
    }

    if(filled) {
        auto fillpath = QPainterPath();
        fillpath.addEllipse(inner_draw);
        auto brush = palette.getBrush();
        painter->setBrush(QBrush(brush));
        painter->setPen(Qt::NoPen);
        painter->drawPath(fillpath);
    }

    if(display_radius) {
        font_radius.setPointSize (qMax(1, static_cast<int>(settings.value("map/items_font").toInt()*scale_factor)));
        path_draw = QPainterPath();
        path_draw.addText(textPos.x(), textPos.y(), font_radius, text);
        painter->setBrush(QBrush(palette.getVariant(COLOR_IDLE)));
        painter->setPen(Qt::NoPen);
        painter->drawPath(path_draw);
    }
}

QPainterPath GraphicsCircle::shape() const
{
    return path_shape;
}



void GraphicsCircle::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }

    if(!editable) {
        event->ignore();
        return;
    }

    textPos = event->pos() + TEXT_OFFSET;
    QPointF pressPos = QPointF(event->pos().x() * scale(), event->pos().y() * scale());
    dr = sqrt(pressPos.x()*pressPos.x() + pressPos.y()*pressPos.y()) - radius;
    GraphicsObject::mousePressEvent(event);
    if(abs(dr) < stroke*2) {
        scale_state = CSS_PRESSED;
        display_radius = true;
    }
    changeFocus();
}

void GraphicsCircle::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
    auto settings = getAppSettings();
    QPointF mousePos = event->pos();
    double r = sqrt(mousePos.x()*mousePos.x() + mousePos.y()*mousePos.y()) - dr;

    if(scale_state == CSS_PRESSED) {
        if(qAbs(radius - r) > settings.value("map/move_hyteresis").toInt() && editable) {
            scale_state = CSS_SCALED;
            changeFocus();
        }
    } else if(scale_state == CSS_SCALED) {
        prepareGeometryChange();
        radius = static_cast<int>(r);
        textPos = mousePos + TEXT_OFFSET;
        update();
        emit circleScaled(radius);
    }
}

void GraphicsCircle::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }

    if(scale_state == CSS_PRESSED) {
        emit objectClicked(event->scenePos());
    }
    else if(scale_state == CSS_SCALED) {
        emit circleScaleFinished();
    }
    scale_state = CSS_IDLE;
    display_radius = false;
    changeFocus();
    update();
}

void GraphicsCircle::setRadius(double r) {
    prepareGeometryChange();
    radius = abs(r);
}

void GraphicsCircle::changeFocus() {
    auto settings = getAppSettings();
    if(!isHighlighted()) {
        current_color = COLOR_UNFOCUSED;
        stroke = static_cast<int>(base_stroke / settings.value("map/size_highlight_factor").toDouble());
    } else {
        stroke = base_stroke;
        switch (scale_state) {
        case CSS_IDLE:
            current_color = COLOR_IDLE;
            break;
        case CSS_PRESSED:
            current_color = COLOR_PRESSED;
            break;
        case CSS_SCALED:
            current_color = COLOR_SCALING;
            break;
        }
    }
    update();
}
