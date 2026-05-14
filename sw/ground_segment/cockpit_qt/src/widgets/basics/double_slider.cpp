#include "double_slider.h"
#include <QStyleOptionSlider>
#include <QToolTip>
#include <QWheelEvent>
#include <QDebug>
#include <math.h>

DoubleSlider::DoubleSlider(Qt::Orientation orientation, QWidget *parent) : QSlider(orientation, parent)
{
    connect(this, &QSlider::valueChanged, this, &DoubleSlider::notifyValueChanged);
}


void DoubleSlider::notifyValueChanged(int value) {
    double doubleValue = value * step + min;
    emit doubleValueChanged(doubleValue);
}


void DoubleSlider::setDoubleRange(double mi, double ma, double sp) {
    min = mi;
    max = ma;
    step = sp;
    int nb_steps = ceil((max - min) / step);
    setRange(0, nb_steps);
    if(nb_steps < 20) {
        setTickPosition(QSlider::TicksBelow);
    }
}

void DoubleSlider::setDoubleValue(double v) {
    setValue((v - min)/step);
}

double DoubleSlider::doubleValue() {
    return value()*step + min;
}

void DoubleSlider::sliderChange(QAbstractSlider::SliderChange change)
{
    QSlider::sliderChange(change);

    if (change == QAbstractSlider::SliderValueChange )
    {
        QStyleOptionSlider opt;
        initStyleOption(&opt);
    }
}

void DoubleSlider::wheelEvent(QWheelEvent* e) {
    if(e->angleDelta().y() > 0) {
        setValue(value()+1);
    } else {
        setValue(value()-1);
    }
    e->accept();
}
