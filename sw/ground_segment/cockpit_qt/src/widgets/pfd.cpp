#include "pfd.h"
#include <QPaintEvent>
#include <QPainter>
#include <math.h>
#include <QDebug>
#include <QApplication>
#include <algorithm>
#include <AircraftManager.h>
#include "dispatcher_ui.h"
#include <QPixmap>
#include <QBitmap>
#include <QRegion>
#include <QPainterPath>


Pfd::Pfd(QWidget *parent) : QWidget(parent), rotation(0), current_ac(""), border_stroke(6)
{
    connect(DispatcherUi::get(), &DispatcherUi::ac_selected, this, &Pfd::changeCurrentAC);
    connect(DispatcherUi::get(), &DispatcherUi::mapRotated, this, &Pfd::setRotation);
    connect(PprzDispatcher::get(), &PprzDispatcher::flight_param, this, &Pfd::updateEulers);
    places[0] = ROLL;
    places[1] = PITCH;
    places[2] = YAW;

    pix_roll_fixedwing = QPixmap(":/pictures/pfd_aircraft_roll.svg");
    pix_pitch_fixedwing = QPixmap(":/pictures/pfd_aircraft_pitch.svg");
    pix_yaw_fixedwing = QPixmap(":/pictures/pfd_aircraft_yaw.svg");
    pix_no_ac = QPixmap(":/pictures/no_ac.svg");

    pix_roll_rotorcraft = QPixmap(":/pictures/pfd_aircraft_roll_rotorcraft.svg");
    pix_pitch_rotorcraft = QPixmap(":/pictures/pfd_aircraft_pitch_rotorcraft.svg");
    pix_yaw_rotorcraft = QPixmap(":/pictures/pfd_aircraft_yaw_rotorcraft.svg");

    pix_roll_rover = QPixmap(":/pictures/pfd_rover_roll.svg");
    pix_pitch_rover = QPixmap(":/pictures/pfd_rover_pitch.svg");
    pix_yaw_rover = QPixmap(":/pictures/pfd_rover_yaw.svg");
}

void Pfd::resizeEvent(QResizeEvent *event) {
    (void)event;
    QRegion reg;
    for(int i=0; i<3; i++){
        QRect rect = placeRect(i);
        rect.setWidth(rect.width()+border_stroke);
        rect.setHeight(rect.height()+border_stroke);
        QPointF center = placeCenter(i);
        QPoint p(static_cast<int>(center.x()), static_cast<int>(center.y()));
        rect.moveCenter(p);

        QRegion ri(rect, QRegion::Ellipse);
        reg = reg.united(ri);
    }

    setMask(reg);
}


void Pfd::paintEvent(QPaintEvent *event) {
    (void)event;

    //int side = qMin(width(), height());
    //QRect rect = QRect(-side/4,-side/4, side/2, side/2);

    if(eulers.find(current_ac) != eulers.end()) {
        auto ac_color = AircraftManager::get()->getAircraft(current_ac)->getColor();
        for(int i=0; i<3; i++) {
            QRect rect = placeRect(i);
            QPointF center = placeCenter(i);
            if(places[i] == PITCH) {
                float pitch = eulers[current_ac].pitch;
                float roll = eulers[current_ac].roll;
                paintPitch(rect, center, ac_color, pitch, roll);
            } else if(places[i] == ROLL) {
                float pitch = eulers[current_ac].pitch;
                float roll = eulers[current_ac].roll;
                paintRoll(rect, center, ac_color, pitch, roll);
            } else {
                float yaw = eulers[current_ac].yaw;
                paintYaw(rect, center, ac_color, yaw);
            }
        }

    } else {
        auto no_ac_color = QColor(0x808080);
        QRect rect = placeRect(0);
        QPointF center = placeCenter(0);
        paintPitch(rect, center, no_ac_color, 0, 0);
        rect = placeRect(1);
        center = placeCenter(1);
        paintRoll(rect, center, no_ac_color, 0, 0);
        rect = placeRect(2);
        center = placeCenter(2);
        paintYaw(rect, center, no_ac_color, 0);

    }



}

void Pfd::paintPitch(QRect rect,  QPointF center, QColor ac_color, float pitch, float roll) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(center);

    painter.setBrush(QBrush(QColor(0x00b0ea)));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(rect);

    double alpha = static_cast<double>(-roll) * M_PI/180.0;
    double y = sin(alpha) * rect.height() / 2;
    double x = cos(alpha) * rect.width() / 2 - border_stroke;

    painter.setBrush(QColor(0x986701));
    painter.drawChord(rect, static_cast<int>(16*(alpha - M_PI)*180.0/M_PI), static_cast<int>(16*(2*(M_PI_2-alpha))*180.0/M_PI));

    QPen pen = QPen(Qt::white, 4);
    painter.setPen(pen);
    QLine line(static_cast<int>(-x), static_cast<int>(y) , static_cast<int>(x), static_cast<int>(y));
    //QLine line(rect.left(), 0 , rect.right() , 0);
    painter.drawLine(line);

    pen = QPen(Qt::black, 2, Qt::DashDotLine);
    painter.setPen(pen);
    line = QLine(rect.left()+border_stroke, 0 , rect.right()-border_stroke , 0);
    painter.drawLine(line);

    QPixmap* pix = getIcon(PITCH);

    painter.rotate(static_cast<double>(pitch));
    painter.drawPixmap(-rect.width()/2, -rect.height()/2, rect.width(), rect.height(), pix->scaled(pix->width(),pix->height(),Qt::KeepAspectRatio));

    QPainterPath pp;
    pp.addEllipse(rect);
    pp.addEllipse(rect.center(), rect.left()+border_stroke, rect.top()+border_stroke);
    painter.setBrush(ac_color);
    painter.setPen(Qt::NoPen);
    painter.drawPath(pp);
}

void Pfd::paintRoll(QRect rect,  QPointF center, QColor ac_color, float pitch, float roll) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(center);
    painter.setBrush(QBrush(QColor(0x00b0ea)));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(rect);

    double alpha = static_cast<double>(pitch) * M_PI/180.0;
    double y = sin(alpha) * rect.height() / 2;
    double x = cos(alpha) * rect.width() / 2 - border_stroke;

    painter.setBrush(QColor(0x986701));
    painter.drawChord(rect, static_cast<int>(16*(alpha - M_PI)*180.0/M_PI), static_cast<int>(16*(2*(M_PI_2-alpha))*180.0/M_PI));

    QPen pen = QPen(Qt::white, 4);
    painter.setPen(pen);
    QLine line(static_cast<int>(-x), static_cast<int>(y), static_cast<int>(x), static_cast<int>(y));
    painter.drawLine(line);

    pen = QPen(Qt::black, 2, Qt::DashDotLine);
    painter.setPen(pen);
    line = QLine(rect.left()+border_stroke, 0 , rect.right()-border_stroke , 0);
    painter.drawLine(line);

    QPixmap* pix = getIcon(ROLL);

    painter.rotate(static_cast<double>(roll));
    painter.drawPixmap(-rect.width()/2, -rect.height()/2, rect.width(), rect.height(), pix->scaled(pix->width(),pix->height(),Qt::KeepAspectRatio));

    QPainterPath pp;
    pp.addEllipse(rect);
    pp.addEllipse(rect.center(), rect.left()+border_stroke, rect.top()+border_stroke);
    painter.setBrush(ac_color);
    painter.setPen(Qt::NoPen);
    painter.drawPath(pp);
}

void Pfd::paintYaw(QRect rect, QPointF center, QColor ac_color, float yaw) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(center);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(Qt::black)));
    painter.drawEllipse(rect);

    painter.rotate(rotation);
    QRect rect_rose = QRect(rect.left() + border_stroke, rect.top() + border_stroke, rect.width()-2*border_stroke, rect.height()-2*border_stroke);
    QPixmap rose(":/pictures/wind_rose.svg");
    painter.drawPixmap(rect_rose, rose);

    QPixmap* pix = getIcon(YAW);
    painter.rotate(static_cast<double>(yaw));
    painter.drawPixmap(-rect.width()/2, -rect.height()/2, rect.width(), rect.height(), pix->scaled(pix->width(),pix->height(),Qt::KeepAspectRatio));

    QPainterPath pp;
    pp.addEllipse(rect);
    pp.addEllipse(rect.center(), rect.left()+border_stroke, rect.top()+border_stroke);
    painter.setBrush(ac_color);
    painter.setPen(Qt::NoPen);
    painter.drawPath(pp);
}


void Pfd::mousePressEvent(QMouseEvent *event) {
    (void)event;
    for(int i=2; i>=0; i--){
        QRect rect = placeRect(i);
        QPointF center = placeCenter(i);
        QPoint p(static_cast<int>(center.x()), static_cast<int>(center.y()));
        rect.moveCenter(p);
        if(rect.contains(event->pos())) {
            double r = sqrt(pow(event->pos().x()-center.x(), 2) + pow(event->pos().y()-center.y(), 2));
            if(r < rect.width()/2) {
                Axis tmp = places[0];
                places[0] = places[i];
                places[i] = tmp;
                update();
                return;
            }
        }
    }

    event->ignore();
    QWidget::mousePressEvent(event);
}

void Pfd::changeCurrentAC(QString id) {
    if(AircraftManager::get()->aircraftExists(id)) {
        current_ac = id;
        update();
    }
}

void Pfd::setRotation(double rot) {
    rotation = rot;
    update();
}

void Pfd::updateEulers(pprzlink::Message msg) {
    QString ac_id;
    float roll, pitch, heading;
    msg.getField("ac_id", ac_id);
    if(AircraftManager::get()->aircraftExists(ac_id)) {
        msg.getField("roll", roll);
        msg.getField("pitch", pitch);
        msg.getField("heading", heading);
        eulers[ac_id] = Eulers {roll, pitch, heading};
        update();
    }
}


QRect Pfd::placeRect(int i) {
    int side = qMin(width(), height());

    if(i==0) {
        double sizeA = side*0.8;
        return QRect(static_cast<int>(-sizeA/2),static_cast<int>(-sizeA/2), static_cast<int>(sizeA), static_cast<int>(sizeA));
    } else if(i==1) {
        double sizeB = side*0.35;
        return QRect(static_cast<int>(-sizeB/2),static_cast<int>(-sizeB/2), static_cast<int>(sizeB), static_cast<int>(sizeB));
    } else {
        double sizeC = side*0.35;
        return QRect(static_cast<int>(-sizeC/2),static_cast<int>(-sizeC/2), static_cast<int>(sizeC), static_cast<int>(sizeC));
    }
}

QPointF Pfd::placeCenter(int i) {
    int side = qMin(width(), height());
    double dx = (width()-side)/2;
    double dy = (height()-side)/2;
    QRectF pfd_rect = QRectF(dx,dy, side, side);

    if(i==0) {
        double sizeA = side*0.8;
        return QPointF(pfd_rect.left() + sizeA/2, pfd_rect.center().y());
    } else if(i==1) {
        double sizeB = side*0.35;
        QRect rectB = QRect(static_cast<int>(-sizeB/2), static_cast<int>(-sizeB/2), static_cast<int>(sizeB), static_cast<int>(sizeB));
        return pfd_rect.topRight()-rectB.topRight();
    } else {
        double sizeC = side*0.35;
        QRect rectC = QRect(static_cast<int>(-sizeC/2),static_cast<int>(-sizeC/2), static_cast<int>(sizeC), static_cast<int>(sizeC));
        return pfd_rect.bottomRight() - rectC.bottomRight();
    }
}

QPixmap* Pfd::getIcon(Axis axis) {
    if(current_ac == "") {
        return &pix_no_ac;
    }

    auto firmware = AircraftManager::get()->getAircraft(current_ac)->getAirframe()->getFirmware();
    if(firmware == "fixedwing") {
        switch (axis) {
        case ROLL:
            return &pix_roll_fixedwing;
        case PITCH:
            return &pix_pitch_fixedwing;
        case YAW:
            return &pix_yaw_fixedwing;
        }
    } else if(firmware == "rotorcraft") {
        switch (axis) {
        case ROLL:
            return &pix_roll_rotorcraft;
        case PITCH:
            return &pix_pitch_rotorcraft;
        case YAW:
            return &pix_yaw_rotorcraft;
        }
    } else if(firmware == "rover") {
        switch (axis) {
        case ROLL:
            return &pix_roll_rover;
        case PITCH:
            return &pix_pitch_rover;
        case YAW:
            return &pix_yaw_rover;
        }
    } else {
        throw runtime_error("[PFD] firmware not supported yet!");
    }
    return nullptr;
}

QSize Pfd::sizeHint() const
{
    return QSize(200, 200);
}

QSize Pfd::minimumSizeHint() const
{
    return QSize(200, 200);
}
