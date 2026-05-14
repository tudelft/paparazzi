#include "switch.h"
#include <QEnterEvent>

Switch::Switch(QWidget *parent) : QAbstractButton(parent),
_opacity(0.000),
_height(16),
_margin(3)
{
    setOffset(_height / 2);
    _y = _height / 2;
    setBrush(QColor("#009688"));
    setCheckable(true);
}

Switch::Switch(const QBrush &brush, QWidget *parent) : QAbstractButton(parent),
_opacity(0.000),
_height(16),
_margin(3)
{
    setOffset(_height / 2);
    _y = _height / 2;
    setBrush(brush);
    setCheckable(true);  
}

void Switch::paintEvent(QPaintEvent *e) {
    (void)e;
    QPainter p(this);
    p.setPen(Qt::NoPen);
    if (isEnabled()) {
        setOffset(isChecked() ? width() - (_height) : _height / 2);
        p.setBrush(isChecked() ? brush() : Qt::black);
        p.setOpacity(isChecked() ? 0.5 : 0.38);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.drawRoundedRect(QRect(_margin, _margin, width() - 2 * _margin, height() - 2 * _margin), 8.0, 8.0);
        p.setBrush(isChecked() ? brush() : QBrush("#d5d5d5"));
        p.setOpacity(1.0);
        p.drawEllipse(QRectF(offset() - (_height / 2), _y - (_height / 2), height(), height()));
    } else {
        p.setBrush(Qt::black);
        p.setOpacity(0.12);
        p.drawRoundedRect(QRect(_margin, _margin, width() - 2 * _margin, height() - 2 * _margin), 8.0, 8.0);
        p.setOpacity(1.0);
        p.setBrush(QColor("#BDBDBD"));
        p.drawEllipse(QRectF(offset() - (_height / 2), _y - (_height / 2), height(), height()));
    }
}

void Switch::enterEvent(QEnterEvent *e) {
    setCursor(Qt::PointingHandCursor);
    QAbstractButton::enterEvent(e);
}

QSize Switch::sizeHint() const {
    return QSize(2 * (_height + _margin), _height + 2 * _margin);
}
