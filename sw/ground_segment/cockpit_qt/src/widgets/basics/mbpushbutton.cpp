#include "mbpushbutton.h"
#include <QMouseEvent>
#include <QEnterEvent>
#include <QEvent>

MbPushButton::MbPushButton(QWidget *parent) : QPushButton(parent)
{

}

MbPushButton::MbPushButton(QString txt, QWidget *parent) : QPushButton (txt, parent)
{

}


void MbPushButton::mousePressEvent(QMouseEvent *e) {
    QPushButton::mousePressEvent(e);
}

void MbPushButton::mouseReleaseEvent(QMouseEvent *e) {
    QPushButton::mouseReleaseEvent(e);
    if(e->button() == Qt::RightButton) {
        emit rightClicked();
    }
}

void MbPushButton::mouseMoveEvent(QMouseEvent *e) {
    QPushButton::mouseMoveEvent(e);
}

void MbPushButton::enterEvent(QEnterEvent *event) {
    emit enter();
    QPushButton::enterEvent(event);
}

void MbPushButton::leaveEvent(QEvent *event) {
    emit leave();
    QPushButton::leaveEvent(event);
}
