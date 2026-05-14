#include <QPainter>
#include <QPaintEvent>
#include <QDebug>
#include <QIcon>
#include <QWheelEvent>
#include <QPainterPath>
#include <qmath.h>
#include "dispatcher_ui.h"
#include "AircraftManager.h"
#include "wind_sock.h"
#include "gcs_utils.h"
#include "pprz_dispatcher.h"

WindSock::WindSock(QWidget *parent) : QWidget(parent),
    current_ac_id(""), compass(0), wind_threshold(1),
    speed_unit(MS), rotate_state(IDLE),
    _size(100, 100), m_background_color(Qt::transparent),
    m_arrow_color(Qt::red), m_pen_color(Qt::black)
{
    setAttribute(Qt::WA_TranslucentBackground);
    
    connect(DispatcherUi::get(), &DispatcherUi::ac_selected, this, &WindSock::setAC);

    PprzDispatcher::get()->bind("WIND", this, [=](QString sender, pprzlink::Message msg){
        (void)sender;
        QString ac_id;
        msg.getField("ac_id", ac_id);
        double dir = getFloatingField(msg, "dir");
        double wspeed = getFloatingField(msg, "wspeed");
        setWindData(ac_id, dir, wspeed, 3); // Highest priority: Ground estimator
    });

    // PprzDispatcher::get()->bind("NPS_WIND", this, [=](QString sender, pprzlink::Message msg){
    //     (void)sender;
    //     double vx = getFloatingField(msg, "vx");
    //     double vy = getFloatingField(msg, "vy");
    //     // Convert NED wind vector (vx=North, vy=East) to wind direction (blowing FROM, clockwise from North)
    //     double dir = std::atan2(-vy, -vx) * 180.0 / M_PI;
    //     if (dir < 0.0) dir += 360.0;
    //     double wspeed = std::sqrt(vx*vx + vy*vy);
        
    //     // NPS_WIND has no ac_id, so apply it to the currently selected AC
    //     if (current_ac_id != "") {
    //         setWindData(current_ac_id, dir, wspeed, 1); // Lowest priority: Simulator ambient
    //     }
    // });

    // PprzDispatcher::get()->bind("WIND_INFO_RET", this, [=](QString sender, pprzlink::Message msg){
    //     uint8_t flags = 0;
    //     msg.getField("flags", flags);
    //     if (flags & 0x01) { // horizontal wind valid
    //         double east = getFloatingField(msg, "east");
    //         double north = getFloatingField(msg, "north");
            
    //         // Similar to NPS_WIND, east and north define the wind vector (blowing TO)
    //         // Direction from where it blows:
    //         double dir = std::atan2(-east, -north) * 180.0 / M_PI;
    //         if (dir < 0.0) dir += 360.0;
    //         double wspeed = std::sqrt(east*east + north*north);
            
    //         QString ac_id = sender.isEmpty() ? current_ac_id : sender;
    //         setWindData(ac_id, dir, wspeed, 2); // Medium priority: AC embedded estimator
    //     }
    // });
}

void WindSock::resizeEvent(QResizeEvent *event) {
    (void)event;
    // Mask removed to allow larger scaling without clipping edges
}

static int meterpersecond_to_beaufort(double mps) {
    if (mps < 0.3) return 0;
    if (mps < 1.6) return 1;
    if (mps < 3.4) return 2;
    if (mps < 5.5) return 3;
    if (mps < 8.0) return 4;
    if (mps < 10.8) return 5;
    if (mps < 13.9) return 6;
    if (mps < 17.2) return 7;
    if (mps < 20.8) return 8;
    if (mps < 24.5) return 9;
    if (mps < 28.5) return 10;
    if (mps < 32.7) return 11;
    return 12;
}

void WindSock::paintEvent(QPaintEvent *event) {
    (void)event;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if(wind_data.contains(current_ac_id)) {
        double wind_speed = wind_data[current_ac_id].wind_speed;
        double wind_dir = wind_data[current_ac_id].wind_dir;

        if(wind_speed > wind_threshold) {
            painter.save();

            // Set drawing area relative to widget center
            auto tr = rect().center();
            painter.translate(tr);

            int wsWidth = rect().width() * 0.8;
            // base size is 12 (from -6 to 6), scale it to wsWidth
            double scale = wsWidth / 20.0; // Scaled by 20 to be 0.6 of the doubled visual size

            // Rotate based on wind direction 
            painter.rotate(wind_dir + 90); // Ensure the orientation matches
            
            QPolygonF red_left, red_right, white_center, contour;
            red_left << QPointF(-6*scale, 4*scale) << QPointF(-2*scale, 3*scale) << QPointF(-2*scale, -3*scale) << QPointF(-6*scale, -4*scale);
            red_right << QPointF(2*scale, 2*scale) << QPointF(6*scale, 1*scale) << QPointF(6*scale, -1*scale) << QPointF(2*scale, -2*scale);
            white_center << QPointF(-2*scale, 3*scale) << QPointF(2*scale, 2*scale) << QPointF(2*scale, -2*scale) << QPointF(-2*scale, -3*scale);
            contour << QPointF(-6*scale, 4*scale) << QPointF(6*scale, 1*scale) << QPointF(6*scale, -1*scale) << QPointF(-6*scale, -4*scale);

            // Fill polygons
            painter.setPen(Qt::NoPen);
            painter.setBrush(Qt::red);
            painter.drawPolygon(red_left);
            painter.drawPolygon(red_right);
            painter.setBrush(Qt::white);
            painter.drawPolygon(white_center);

            // Get AC color for contour
            QColor ac_color = Qt::white;
            try {
                auto ac = AircraftManager::get()->getAircraft(current_ac_id);
                if (ac) {
                    ac_color = ac->getColor();
                }
            } catch (...) {}

            painter.setBrush(Qt::NoBrush);
            // Draw contour with AC color (matching OCaml outline)
            painter.setPen(QPen(ac_color, 2));
            painter.drawPolygon(contour);

            painter.restore();

            // text (drawn centered, upright, independent of sock rotation)
            painter.resetTransform();
            painter.setPen(m_pen_color);
            auto font = QFont();
            font.setBold(true);
            painter.setFont(font);
            QString txt;
            switch (speed_unit) {
            case MS:
                txt = QString("%1 m/s").arg(wind_speed, 0, 'f', 1);
                break;
            case KT:
                txt = QString("%1 kt").arg(wind_speed*1.9438, 0, 'f', 1);
                break;
            case KMH:
                txt = QString("%1 km/h").arg(wind_speed*3.6, 0, 'f', 1);
                break;
            case BFT:
                txt = QString("%1 bft").arg(meterpersecond_to_beaufort(wind_speed));
                break;
            }
            // Draw a white outline manually for the text
            painter.setPen(Qt::white);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx != 0 || dy != 0) {
                        painter.drawText(rect().adjusted(dx, dy, dx, dy), Qt::AlignCenter, txt);
                    }
                }
            }
            // Draw the actual black text over the outline
            painter.setPen(Qt::black);
            painter.drawText(rect(), Qt::AlignCenter, txt);
        }
    }
}

void WindSock::changeUnit() {
    switch (speed_unit) {
    case MS:
        speed_unit = KT;
        break;
    case KT:
        speed_unit = KMH;
        break;
    case KMH:
        speed_unit = BFT;
        break;
    case BFT:
        speed_unit = MS;
        break;
    }
    update();
}

bool WindSock::onNorth(QMouseEvent* event) {
    auto tr = QTransform();
    auto translation = rect().center() + QPoint(1,1);
    tr.translate(translation.x(), translation.y());
    tr.rotate(compass - 90);
    return tr.mapRect(north_rect).contains(event->pos());
}

void WindSock::mousePressEvent(QMouseEvent *event) {
    if(onNorth(event)) {
        rotate_state = PRESSED;
    } else {
        bool hit_windsock = false;
        
        if(wind_data.contains(current_ac_id)) {
            double wind_speed = wind_data[current_ac_id].wind_speed;
            
            if(wind_speed > wind_threshold) {
                double wind_dir = wind_data[current_ac_id].wind_dir;
                int wsWidth = rect().width() * 0.8;//TODO: Cleanup scaling
                double scale = wsWidth / 20.0;
                
                QPolygonF contour;
                contour << QPointF(-6*scale, 4*scale) << QPointF(6*scale, 1*scale) 
                        << QPointF(6*scale, -1*scale) << QPointF(-6*scale, -4*scale);
                
                QTransform transform;
                transform.translate(rect().center().x(), rect().center().y());
                transform.rotate(wind_dir + 90);
                
                if (transform.map(contour).containsPoint(event->pos(), Qt::OddEvenFill)) {
                    hit_windsock = true;
                }
            }
        }
        
        if (hit_windsock) {
            changeUnit();
            event->accept();
        } else {
            event->ignore(); // Ignored clicks completely fall through to the pan controls
        }
    }
}

void WindSock::mouseDoubleClickEvent(QMouseEvent *event) {
    if(onNorth(event)) {
        emit requestRotation(0);
    } else {
        event->ignore();
    }
}

void WindSock::mouseMoveEvent(QMouseEvent *event) {
    if(rotate_state == PRESSED) {
        auto pos = event->pos() - rect().center();
        double angle = atan2(pos.x(), -pos.y());
        emit requestRotation(qRadiansToDegrees(angle));
    } else {
        event->ignore();
    }
}

void WindSock::mouseReleaseEvent(QMouseEvent *event) {
    (void)event;
    if(rotate_state == PRESSED) {
        rotate_state = IDLE;
    } else {
        event->ignore();
    }
}

void WindSock::wheelEvent(QWheelEvent* event) {
    double rot = compass + event->angleDelta().y() / 10.0;
    emit requestRotation(rot);
}

QSize WindSock::sizeHint() const
{
    return minimumSizeHint();
}

QSize WindSock::minimumSizeHint() const
{
    return _size;
}
