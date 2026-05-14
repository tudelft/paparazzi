#include "colorlabel.h"
#include <QPainter>
#include <QPaintEvent>
#include <QFontMetrics>
#include <QDebug>
#include <math.h>

ColorLabel::ColorLabel(qreal radius, QWidget *parent) : QWidget(parent),
    text(""), brush(Qt::red), minSize(QSize(10, 10)), radius(radius)
{

}

void ColorLabel::paintEvent(QPaintEvent *e) {
    (void)e;
    QPainter p(this);
    // Set bold font
    QFont font = p.font();
    font.setBold(true);
    auto fm = QFontMetricsF(font);
    // Compute minimum widget size based off text size.
    auto txt_rect = fm.boundingRect(text != "" ? text:"00");
    auto manual_rect = fm.boundingRect("MANUAL");
    
    int textWidth = static_cast<int>(ceil(txt_rect.width())) + 6;
    int manualWidth = static_cast<int>(ceil(manual_rect.width())) + 6;
    
    int minWidth = qMax(textWidth, manualWidth);
    int minHeigth = static_cast<int>(ceil(txt_rect.height()));
    
    minSize = QSize(minWidth, minHeigth);
    setMinimumSize(minSize);

    // Draw background
    p.setPen(Qt::NoPen);
    p.setBrush(brush);
    p.drawRoundedRect(rect(), radius, radius);

    // Draw text
    p.setFont(font);
    p.setPen(Qt::black);
    p.drawText(rect(), Qt::AlignCenter, text);
}

QSize ColorLabel::minimumSizeHint() const
{
    return minSize;
}
