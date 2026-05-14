#include "graphics_point.h"
#include "math.h"
#include <QApplication>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QPainter>
#include <QPainterPath>
#include <QTimer>
#include "gcs_utils.h"

#define COLOR_IDLE 0
#define COLOR_PRESSED 1
#define COLOR_MOVED 2
#define COLOR_UNFOCUSED 3

GraphicsPoint::GraphicsPoint(int size, PprzPalette palette, QObject *parent) :
    GraphicsObject(palette, parent),
    QGraphicsItem (),
    halfSize(size), move_state(PMS_IDLE), animation_couter(0)
{
    current_color = COLOR_IDLE;

    animation_timer->setInterval(500);
    connect(animation_timer, &QTimer::timeout, this, [=]()
    {
        animation_couter += 1;
        prepareGeometryChange();
    });
}

QRectF GraphicsPoint::boundingRect() const {
    return QRectF(-halfSize-1, -halfSize-1, 2*halfSize+2, 2*halfSize+2);
}


void GraphicsPoint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void)option;
    (void)widget;
    auto settings = getAppSettings();

    if(animation == WP_MOVING) {
        painter->rotate(animation_couter * 45 + 1);
    }

    switch (style) {
    case DEFAULT: {
        double fx = 0.8;
        if(!isHighlighted()) {
            current_color = COLOR_UNFOCUSED;
            fx /= settings.value("map/size_highlight_factor").toDouble();
        }

        double r = halfSize * fx * 0.8; // scale to match previous area
        
        painter->save();
        painter->setRenderHint(QPainter::Antialiasing);
        painter->rotate(45); // Default rotation

        QPainterPath path;
        QPolygonF poly;
        poly.append(QPointF(-r, -r));
        poly.append(QPointF(r, -r));
        poly.append(QPointF(r, r));
        poly.append(QPointF(-r, r));
        poly.append(QPointF(-r, -r));
        path.addPolygon(poly);

        painter->setPen(QPen(Qt::black, 1.5, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
        painter->setBrush(QBrush(palette.getVariant(current_color)));
        painter->drawPath(path);
        
        painter->restore();
        }
        break;
    case CARROT: {
        QPainterPath path;
        QPolygonF poly;
        poly.append(QPointF(halfSize*cos(M_PI/6), -halfSize*sin(M_PI/6)));
        poly.append(QPointF(halfSize*cos(5*M_PI/6), -halfSize*sin(5*M_PI/6)));
        poly.append(QPointF(halfSize*cos(3*M_PI/2), -halfSize*sin(3*M_PI/2)));

        path.addPolygon(poly);
        painter->setBrush(QBrush(QColor(255, 170, 0)));
        painter->setPen(Qt::NoPen);
        painter->drawPath(path);
        }
        break;
    case CURRENT_NAV: {
        //draw nothing
        }
        break;
    case GCS: {
        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(palette.getVariant(current_color), 3));
        painter->drawEllipse(QPoint(0,0), 2*halfSize/3, 2*halfSize/3);
        painter->drawLine(0, -halfSize, 0, halfSize);
        painter->drawLine(-halfSize, 0, halfSize, 0);
        }
        break;
    case DCSHOT: {
        painter->setBrush(QBrush(palette.getVariant(current_color)));
        painter->drawEllipse(QPoint(0, 0), halfSize/2, halfSize/2);
        }
        break;
    case CRASH: {
        // Draw a red cross with thickness 3
        painter->setPen(QPen(QColor(255, 0, 0), 3));
        painter->drawLine(QPoint(-halfSize/2, -halfSize/2), QPoint(halfSize/2, halfSize/2));
        painter->drawLine(QPoint(-halfSize/2, halfSize/2), QPoint(halfSize/2, -halfSize/2));
        }
        break;
    }

}

QPainterPath GraphicsPoint::shape() const {
    if(style == CURRENT_NAV || style == CARROT) {
        QPainterPath path;
        return path;
    } else {
        return QGraphicsItem::shape();
    }
}

void GraphicsPoint::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
    GraphicsObject::mousePressEvent(event);
    pressPos = QPointF(event->pos().x() * scale(), event->pos().y() * scale());
    move_state = PMS_PRESSED;
    changeFocus();
}

void GraphicsPoint::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
    auto settings = getAppSettings();
    if(move_state == PMS_PRESSED) {
        QPointF dp = event->pos() - pressPos;
        double d = sqrt(dp.x()*dp.x() + dp.y()*dp.y());
        if(d > settings.value("map/move_hyteresis").toInt() && editable) {
            move_state = PMS_MOVED;
            changeFocus();
        }
    } else if(move_state == PMS_MOVED) {
        setPos(event->scenePos() - pressPos);
        emit pointMoved(event->scenePos() - pressPos);
    }
}

void GraphicsPoint::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
    if(move_state == PMS_PRESSED) {
        emit objectClicked(event->scenePos());
    }
    else if(move_state == PMS_MOVED) {
        emit pointMoveFinished(event->scenePos() - pressPos);
    }
    move_state = PMS_IDLE;
    changeFocus();
}

void GraphicsPoint::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    if(ignore_events) {
        event->ignore();
        return;
    }
    emit objectDoubleClicked(event->scenePos());
}

void GraphicsPoint::changeFocus() {
    if(!isHighlighted()) {
        current_color = COLOR_UNFOCUSED;
    } else {
        switch (move_state) {
        case PMS_IDLE:
            current_color = COLOR_IDLE;
            break;
        case PMS_PRESSED:
            current_color = COLOR_PRESSED;
            break;
        case PMS_MOVED:
            current_color = COLOR_MOVED;
            break;
        }
    }
    update();
}
