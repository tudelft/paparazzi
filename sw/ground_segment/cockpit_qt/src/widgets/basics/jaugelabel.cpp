#include "jaugelabel.h"
#include <QPainter>
#include <QPaintEvent>
#include <algorithm>

JaugeLabel::JaugeLabel(QWidget *parent) : JaugeLabel(0, 1, "%",  parent)
{
}

JaugeLabel::JaugeLabel(double min, double max, QString unit, QWidget *parent) :
    QWidget(parent), min(min), max(max), unit(unit), precision(2), minSize(20,10)
{
    brushBackEnabled = QColor(0xffa500);
    brushLeft = QColor(0x00ff00);
    brushBackDisabled = Qt::red;
}

QSize JaugeLabel::minimumSizeHint() const
{
    return minSize;
    //return QSize(60, 18);
}

void JaugeLabel::paintEvent(QPaintEvent *e) {
    (void)e;
    QPainter p(this);
    p.setPen(Qt::NoPen);

    if(status) {
        p.setBrush(brushBackEnabled);
    } else {
        p.setBrush(brushBackDisabled);
    }
    p.drawRect(rect());

    p.setBrush(brushLeft);
    int x = static_cast<int>(std::clamp((value - min) / (max - min), 0., 1.) * rect().width());
    p.drawRect(0, 0, x, rect().height());

    QFont font = p.font();
    font.setPointSize(12);
    font.setBold(true);
    p.setFont(font);
    p.setPen(Qt::black);
    QString txt = QString::number(value, 'f', precision) + unit;
    p.drawText(rect(), Qt::AlignCenter, txt);

    auto fm = QFontMetricsF(font);
    // Compute minimum widget size based of text size.
    auto txt_rect = fm.boundingRect(txt);
    minSize = QSize(static_cast<int>(txt_rect.width())+6, static_cast<int>(txt_rect.height()));
    setMinimumSize(minSize);

}
