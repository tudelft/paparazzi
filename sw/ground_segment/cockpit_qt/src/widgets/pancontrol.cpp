#include "pancontrol.h"
#include <QPainter>
#include <QMouseEvent>
#include <QTransform>

constexpr int PAN_STEP = 50;
constexpr int ARROW_SIZE = 40;

PanControl::PanControl(QWidget *parent) : QWidget(parent)
{
    setFixedSize(ARROW_SIZE * 3, ARROW_SIZE * 3);
    setAttribute(Qt::WA_TranslucentBackground);
    setArrows();

    move_timer = new QTimer(this);
    move_timer->setInterval(50);
    connect(move_timer, &QTimer::timeout, this, [=](){
        emit panRequested(current_dx, current_dy);
    });
}

void PanControl::setArrows() {
    double s = ARROW_SIZE;
    double s2 = s / 2.0;
    double s4 = s / 4.0;
    
    QPolygonF basePoly;
    basePoly << QPointF(0, 0)
             << QPointF(s2, s2)
             << QPointF(s4, s2)
             << QPointF(s4, s)
             << QPointF(-s4, s)
             << QPointF(-s4, s2)
             << QPointF(-s2, s2);

    double angles[4] = {0, 180, -90, 90}; // N, S, W, E
    double dxs[4] = {1.5 * s, 1.5 * s, 0, 3.0 * s};
    double dys[4] = {0, 3.0 * s, 1.5 * s, 1.5 * s};
    
    // dx and dy per step (match OCaml move dx dy logic)
    dir_dx[0] = 0; dir_dy[0] = -PAN_STEP;        // N: up
    dir_dx[1] = 0; dir_dy[1] = PAN_STEP;         // S: down
    dir_dx[2] = -PAN_STEP; dir_dy[2] = 0;        // W: left
    dir_dx[3] = PAN_STEP; dir_dy[3] = 0;         // E: right

    for(int i = 0; i < 4; i++) {
        QTransform t;
        t.translate(dxs[i], dys[i]);
        t.rotate(angles[i]);
        arrowPolys[i] = t.map(basePoly);
    }
}

QSize PanControl::sizeHint() const {
    return minimumSizeHint();
}

QSize PanControl::minimumSizeHint() const {
    return QSize(ARROW_SIZE * 3, ARROW_SIZE * 3);
}

void PanControl::paintEvent(QPaintEvent *event) {
    (void)event;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // EXACT RESTORE to when it worked
    painter.fillRect(rect(), Qt::transparent);

    // Mimic OCaml style: Fill Color #a0a0ff
    QColor fillColor("#a0a0ff");
    fillColor.setAlpha(200);

    painter.setBrush(fillColor);
    painter.setPen(QPen(Qt::darkGray, 1));

    for(int i = 0; i < 4; i++) {
        painter.drawPolygon(arrowPolys[i]);
    }
}

bool PanControl::isInArrow(const QMouseEvent* event, int& dx, int& dy) {
    for(int i = 0; i < 4; i++) {
        if(arrowPolys[i].containsPoint(event->pos(), Qt::OddEvenFill)) {
            dx = dir_dx[i];
            dy = dir_dy[i];
            return true;
        }
    }
    return false;
}

void PanControl::mousePressEvent(QMouseEvent *event) {
    if(event->button() == Qt::LeftButton) {
        if(isInArrow(event, current_dx, current_dy)) {
            emit panRequested(current_dx, current_dy);
            move_timer->start();
        }
    }
}

void PanControl::mouseReleaseEvent(QMouseEvent *event) {
    (void)event;
    move_timer->stop();
}

