#include "colorbar.h"
#include <math.h>

ColorBar::ColorBar(qreal bar_height, QWidget *parent) : QWidget(parent),
    min_color(Qt::red), max_color(Qt::green), 
    bar_height(bar_height), minSize(QSize(10, 10))
{

}

void ColorBar::set_zlimits(float minz, float maxz)
{
    min_value = round(minz*10)/10;
    max_value = round(maxz*10)/10;
}

void ColorBar::paintEvent(QPaintEvent *e) 
{
    (void)e;
    QPainter p(this);
    
    // Compute minimum widget size based of text size
    QFont font = p.font();
    auto fm = QFontMetricsF(font);
    auto txt_rect = fm.boundingRect("0000");

    int labelWidth = static_cast<int>(ceil(txt_rect.width()));
    int labelHeigth = static_cast<int>(ceil(txt_rect.height()));

    int minWidth = labelWidth*2;
    int minHeigth = labelHeigth + 20;
    minSize = QSize(minWidth, minHeigth);
    setMinimumSize(minSize);

    // Draw bar
    QLinearGradient gradient(0, 0, width(), 0);
    QGradientStops stops;
    stops << QGradientStop(0, min_color);
    stops << QGradientStop(1, max_color);
    gradient.setStops(stops);

    auto bar = QRect(labelWidth/2, labelHeigth, width() - labelWidth, height() - labelHeigth);
    p.fillRect(bar, gradient);

    // Draw text labels (dummy but enough)
    auto minz_rect = QRect(0, 0, labelWidth, labelHeigth);
    auto midz_rect = QRect(width()/2 - labelWidth/2, 0, labelWidth, labelHeigth);
    auto maxz_rect = QRect(width() - labelWidth, 0, labelWidth, labelHeigth);
    
    
    p.setFont(font);
    p.setPen(Qt::black);
    p.drawText(minz_rect, Qt::AlignCenter, QString::number(min_value));
    p.drawText(midz_rect, Qt::AlignCenter, QString::number((min_value + max_value)/2));
    p.drawText(maxz_rect, Qt::AlignCenter, QString::number(max_value));

    // Draw markers (dummy but enough)
    auto minz_marker = QRect(labelWidth/2, labelHeigth - 2, 2, height() - labelHeigth);
    auto midz_marker = QRect(width()/2 - 1, labelHeigth - 2, 2, height() - labelHeigth);
    auto maxz_marker = QRect(width() - labelWidth/2 - 2, labelHeigth - 2, 2, height() - labelHeigth);
    p.fillRect(minz_marker, Qt::black);
    p.fillRect(midz_marker, Qt::black);
    p.fillRect(maxz_marker, Qt::black);
}

QSize ColorBar::minimumSizeHint() const
{
    return minSize;
}