#ifndef PFD_H
#define PFD_H

#include <QWidget>
#include "pprz_dispatcher.h"
#include "configurable.h"

class Pfd : public QWidget, public Configurable
{
    Q_OBJECT
public:

    struct Eulers {
        float roll;
        float pitch;
        float yaw;
    };

    explicit Pfd(QWidget *parent = nullptr);

    QSize sizeHint() const;
    QSize minimumSizeHint() const;
    void configure(QDomElement) {};

protected:
    void paintEvent(QPaintEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
//    virtual void mouseMoveEvent(QMouseEvent *event);
//    virtual void mouseReleaseEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent *event);

    void paintPitch(QRect rect, QPointF center, QColor ac_color, float pitch, float roll);
    void paintRoll(QRect rect, QPointF center, QColor ac_color, float pitch, float roll);
    void paintYaw(QRect rect, QPointF center, QColor ac_color, float yaw);




signals:

public slots:

private:
    enum Axis {
        ROLL,
        PITCH,
        YAW
    };

    void changeCurrentAC(QString id);
    void updateEulers(pprzlink::Message msg);
    void setRotation(double rot);

    QRect placeRect(int i);
    QPointF placeCenter(int i);

    QPixmap* getIcon(Axis axis);


    Axis places[3];
    std::map<QString, struct Eulers> eulers;
    double rotation;

    QString current_ac;

    QPixmap pix_roll_fixedwing;
    QPixmap pix_pitch_fixedwing;
    QPixmap pix_yaw_fixedwing;
    QPixmap pix_no_ac;

    QPixmap pix_roll_rotorcraft;
    QPixmap pix_pitch_rotorcraft;
    QPixmap pix_yaw_rotorcraft;

    QPixmap pix_roll_rover;
    QPixmap pix_pitch_rover;
    QPixmap pix_yaw_rover;

    int border_stroke;

};

#endif // PFD_H
