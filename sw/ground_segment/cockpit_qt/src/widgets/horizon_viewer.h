#ifndef HORIZONVIEWER_H
#define HORIZONVIEWER_H

#include <QWidget>
#include <QPainter>
#include <QString>
#include <pprzlinkQt/Message.h>

class HorizonViewer : public QWidget {
    Q_OBJECT
public:
    explicit HorizonViewer(QWidget *parent = nullptr);

public slots:
    void setAttitude(double roll_rad, double pitch_rad);
    void setAlt(double alt);
    void setClimb(double climb);
    void setSpeed(double speed);
    void updateEulers(pprzlink::Message msg);
    void changeCurrentAC(QString ac_id);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    QString current_ac;
    double m_roll = 0.0;
    double m_pitch = 0.0;
    double m_alt = 0.0;
    double m_speed = 0.0;
    double m_max_speed = 0.0;
    double m_min_speed = 999999.0;
};

#endif // HORIZONVIEWER_H
