#include "imagebutton.h"
#include <QEvent>
#include <QEnterEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPixmap>
#include <QBitmap>

ImageButton::ImageButton(QIcon normal, QSize size, bool mask, QWidget *parent) : QPushButton(parent)
{
    icon_normal = normal;
    setIcon(icon_normal);
    setIconSize(size);

    if(mask) {
        QPixmap p = icon().pixmap(iconSize());
        setMask(p.mask());
    }
}

void ImageButton::enterEvent(QEnterEvent* e) {
    QPushButton::enterEvent( e );
    if(icon_hover.isNull()) {
        setIcon(icon_normal);
    } else {
        setIcon(icon_hover);
    }
}

void ImageButton::leaveEvent(QEvent* e) {
    QPushButton::leaveEvent( e );
    setIcon(icon_normal);
}

void ImageButton::mousePressEvent(QMouseEvent *e) {
    QPushButton::mousePressEvent(e);
    // QPushButton::enterEvent( e );
    if(icon_pressed.isNull()) {
        setIcon(icon_normal);
    } else {
        setIcon(icon_pressed);
    }
}

void ImageButton::mouseReleaseEvent(QMouseEvent *e) {
    QPushButton::mouseReleaseEvent(e);
    if(icon_hover.isNull()) {
        setIcon(icon_normal);
    } else {
        setIcon(icon_hover);
    }
}

void ImageButton::paintEvent(QPaintEvent* e) {
    (void)e;
    QPainter painter(this);
    icon().paint(&painter,rect());
}
