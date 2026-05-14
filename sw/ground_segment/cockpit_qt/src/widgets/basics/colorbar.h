#ifndef COLORBAR_H
#define COLORBAR_H

#include <QWidget>
#include <QPaintEvent>
#include <QPainter>
#include <QFontMetrics>

class ColorBar : public QWidget
{
    Q_OBJECT
public:
    explicit ColorBar(qreal bar_height = 20, QWidget *parent = nullptr);

    void set_zlimits(float minz, float maxz);
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    float max_value = 0;
    float min_value = 0;

    QColor min_color;
    QColor max_color;

    qreal bar_height;
    QSize minSize;
};

#endif //COLORBAR_H
