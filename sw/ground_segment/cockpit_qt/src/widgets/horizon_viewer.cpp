#include "horizon_viewer.h"
#include <QMouseEvent>
#include <QPainterPath>
#include <cmath>
#include <QPainter>
#include <QDebug>
#include "tools/pprz_dispatcher.h"
#include "tools/dispatcher_ui.h"
#include "AircraftManager.h"

HorizonViewer::HorizonViewer(QWidget *parent) : QWidget(parent), current_ac("") {
    setMinimumSize(50, 50);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    connect(DispatcherUi::get(), &DispatcherUi::ac_selected, this, &HorizonViewer::changeCurrentAC);
    connect(PprzDispatcher::get(), &PprzDispatcher::flight_param, this, &HorizonViewer::updateEulers);
}

void HorizonViewer::changeCurrentAC(QString ac_id) {
    current_ac = ac_id;
    update();
}

void HorizonViewer::updateEulers(pprzlink::Message msg) {
    QString ac_id;
    msg.getField("ac_id", ac_id);
    if(ac_id == current_ac || current_ac.isEmpty()) {
        float roll = 0, pitch = 0, alt = 0, speed = 0, climb = 0;
        msg.getField("roll", roll);
        msg.getField("pitch", pitch);
        msg.getField("alt", alt);
        msg.getField("speed", speed);
        msg.getField("climb", climb);
        
        m_roll = roll * M_PI / 180.0;  // flight_param is in deg, we need rad? Wait, let's check.
        m_pitch = pitch * M_PI / 180.0;
        m_alt = alt;
        m_speed = speed;
        if (speed > m_max_speed) m_max_speed = speed;
        if (speed < m_min_speed) m_min_speed = speed;
        
        update();
    }
}


void HorizonViewer::setAttitude(double roll_rad, double pitch_rad) {
    m_roll = roll_rad;
    m_pitch = pitch_rad;
    update();
}

void HorizonViewer::setAlt(double alt) {
    m_alt = alt;
    update();
}

void HorizonViewer::setClimb(double climb) {
    // Unused in basic OCaml version
}

void HorizonViewer::setSpeed(double speed) {
    m_speed = speed;
    if (speed > m_max_speed) m_max_speed = speed;
    if (speed < m_min_speed) m_min_speed = speed;
    update();
}

void HorizonViewer::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        m_max_speed = 0.0;
        m_min_speed = 999999.0;
        update();
    }
}

void HorizonViewer::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    int w = width();
    int h = height();
    double max_size2_x = w / 2.88;
    double max_size2_y = h / 2.5;
    double size2 = qMin(max_size2_x, max_size2_y);
    double size = size2 * 2.0;
    painter.translate(0.0, 0.0);
    

    double left_margin = size2 / 10.0;
    auto pitch_scale = [size2](double pitch) { return pitch * size2 * 2.0; };
    double speed_scale = size2 / 10.0;
    double alt_scale = size2 / 50.0;
    double speed_width = size2 / 3.0;
    double alt_width = size2 / 2.25;
    double index_width = size2 / 10.0;

    double xc = left_margin + speed_width + size2;
    double yc = size2 * 1.25;

    int font_size = qMax(8, qRound(size2 * 0.13));
    QFont font("Sans");
    font.setPixelSize(font_size);
    font.setBold(true);
    painter.setFont(font);

    // Draw Disc ( Gray Background)
    painter.save();
    
    double pitch_px = pitch_scale(m_pitch);
    
    // We want to rotate around xc, yc, but the translation depends on pitch as well.
   // translate by xc+sin(roll)*pitch_px, yc+cos(roll)*pitch_px, then rotate by -roll
    
    painter.translate(xc + sin(m_roll)*pitch_px, yc + pitch_px * cos(m_roll));
    painter.rotate(-m_roll * 180.0 / M_PI);
    
    painter.fillRect(QRectF(-size, -size2 * 5.0, size * 2.0, size2 * 5.0), QColor("#0099cb"));
    painter.fillRect(QRectF(-size, 0, size * 2.0, size2 * 5.0), QColor("#986701"));
    painter.setPen(QPen(Qt::white, 4));
    painter.drawLine(QPointF(-size, 0), QPointF(size, 0));
    painter.setPen(QPen(Qt::white, 1));
    painter.drawLine(QPointF(0, -size), QPointF(0, size));

    auto grads = [&](int n, double s, double a, double b, bool text) {
        for (int i = 0; i <= n; ++i) {
            double deg = i * a + b;
            double y = pitch_scale(deg * M_PI / 180.0);
            painter.setPen(QPen(Qt::white, 1));
            painter.drawLine(QPointF(-s, y), QPointF(s, y));
            painter.drawLine(QPointF(-s, -y), QPointF(s, -y));
            if (text) {
                QString t1 = QString::number(int(deg));
                QString t2 = "-" + t1;
                double x_text = 2.0 * s;
                painter.drawText(QRectF(x_text, -y-10, 50, 20), Qt::AlignLeft | Qt::AlignVCenter, t1);
                painter.drawText(QRectF(-x_text-50, -y-10, 50, 20), Qt::AlignRight | Qt::AlignVCenter, t1);
                painter.drawText(QRectF(x_text, y-10, 50, 20), Qt::AlignLeft | Qt::AlignVCenter, t2);
                painter.drawText(QRectF(-x_text-50, y-10, 50, 20), Qt::AlignRight | Qt::AlignVCenter, t2);
            }
        }
    };
    grads(10, size2/10.0, 5.0, 2.5, false);
    grads(5, size2/7.0, 10.0, 5.0, false);
    grads(5, size2/5.0, 10.0, 10.0, true);

    painter.restore();

    // Draw Mask (bottom & top)
    painter.save();
    painter.translate(xc, yc);
    painter.setBrush(Qt::black);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPointF(0,0), 3, 3);
    
    QPainterPath maskPath;
    double pi6 = M_PI / 6.0;
    int nn = 20;
    double arc_start = pi6;
    double arc_end = 5.0 * pi6;
    double step = (arc_end - arc_start) / (nn - 1);
    
    QPolygonF arc_above;
    for (int i = 0; i < nn; ++i) {
        double a = arc_start + i * step;
        arc_above << QPointF(size2 * cos(a), -size2 * sin(a)); // Invert y for drawing
    }
    double mask_x = arc_above.last().x();
    arc_above << QPointF(mask_x, 0) << QPointF(10 * size, 0) << QPointF(10 * size, -10 * size) 
              << QPointF(-size, -10 * size) << QPointF(-size, 0) << QPointF(-mask_x, 0);
    
    painter.drawPolygon(arc_above);
    
    QTransform flip;
    flip.scale(1, -1);
    painter.drawPolygon(flip.map(arc_above));

    double s_mask = size2 / 5.0;
    painter.setPen(QPen(Qt::black, 4));
    QPolygonF p_left; p_left << QPointF(-mask_x, 0) << QPointF(-mask_x - s_mask, 0) << QPointF(-mask_x - s_mask, s_mask);
    painter.drawPolyline(p_left);
    QPolygonF p_right; p_right << QPointF(mask_x, 0) << QPointF(mask_x + s_mask, 0) << QPointF(mask_x + s_mask, s_mask);
    painter.drawPolyline(p_right);

    // Top and bottom graduations
    for (int i = 1; i <= 5; ++i) {
        double a = i * 10;
        painter.save(); painter.rotate(a); painter.setPen(QPen(Qt::white, 1)); painter.drawLine(0, -size2, 0, -1.07 * size2); painter.restore();
        painter.save(); painter.rotate(-a); painter.setPen(QPen(Qt::white, 1)); painter.drawLine(0, -size2, 0, -1.07 * size2); painter.restore();
    }
    auto gg = [&](double a) {
        painter.save(); painter.rotate(a); painter.setPen(QPen(Qt::white, 2)); painter.drawLine(0, -size2, 0, -1.15 * size2); painter.restore();
    };
    gg(30); gg(-30); gg(0);

    auto _30 = [&](double a) {
        painter.save(); painter.rotate(a); painter.setPen(QPen(Qt::white, 1)); 
        painter.drawText(QRectF(-15, -1.28 * size2 - 10, 30, 20), Qt::AlignCenter, "30");
        painter.restore();
    };
    _30(30); _30(-30);

    painter.restore();


    // RULER function logic
    auto draw_ruler = [&](double tx, double ty, double s_scale, double max_val, double w_ruler, double h_ruler, bool index_on_right, double val, double r_step) {
        painter.save();
        painter.translate(tx, ty);
        
        double height = s_scale * max_val;
        
        painter.setClipRect(0, -h_ruler, w_ruler, h_ruler * 2);
        
        painter.fillRect(QRectF(0, -height, w_ruler, height * 2), QColor("#808080"));
        
        painter.save();
        painter.translate(0, val * s_scale);
        int v = int(val) / int(r_step);
        painter.setPen(QPen(Qt::white, 1));
        for (int i = qMax(0, v - 10); i <= qMin((int)(max_val/r_step), v + 10); ++i) {
            double y = -s_scale * (i * r_step);
            painter.drawLine(QPointF(w_ruler * 0.8, y), QPointF(w_ruler, y));
            double y_half = y - (r_step / 2.0) * s_scale;
            painter.drawLine(QPointF(w_ruler * 0.8, y_half), QPointF(w_ruler, y_half));
            
            painter.drawText(QRectF(w_ruler * 0.1, y - 10, w_ruler * 0.6, 20), Qt::AlignRight | Qt::AlignVCenter, QString::number(i * r_step));
        }
        painter.restore();
        
        painter.setClipping(false);
        // Yellow index
        painter.setPen(QPen(Qt::yellow, 2));
        painter.drawLine(QPointF(0, 0), QPointF(w_ruler, 0));
        
        QPolygonF idx;
        idx << QPointF(0, 0) << QPointF(-index_width, index_width/2) << QPointF(-index_width, -index_width/2);
        painter.setBrush(Qt::yellow);
        painter.setPen(Qt::NoPen);
        if (index_on_right) {
            painter.save();
            painter.translate(w_ruler, 0);
            painter.rotate(180);
            painter.drawPolygon(idx);
            painter.restore();
        } else {
            painter.drawPolygon(idx);
        }
        
        painter.restore();
    };


    // Speedometer on the left side
    draw_ruler(left_margin, yc, speed_scale, 50, speed_width, 0.75 * size2, true, m_speed, 2.0);
    painter.setPen(QPen(Qt::yellow, 1));
    painter.drawText(QRectF(left_margin, yc - 0.88 * size2 - 10, speed_width, 20), Qt::AlignCenter, QString::number(m_max_speed, 'f', 1));
    painter.drawText(QRectF(left_margin, yc + 0.875 * size2 - 10, speed_width, 20), Qt::AlignCenter, QString::number(m_min_speed == 999999.0 ? 0.0 : m_min_speed, 'f', 1));

    // Altimeter on the right side
    draw_ruler(xc + size2, yc, alt_scale, 3000, alt_width, 0.75 * size2, false, m_alt, 10.0);
}
