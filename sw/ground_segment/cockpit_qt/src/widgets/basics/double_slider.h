#ifndef DOUBLE_SLIDER_H
#define DOUBLE_SLIDER_H

#include <QWidget>
#include <QSlider>

class DoubleSlider : public QSlider
{
    Q_OBJECT
public:
    explicit DoubleSlider(Qt::Orientation orientation, QWidget *parent = nullptr);

    void setDoubleRange(double min, double max, double step);
    double doubleValue();
    void setDoubleValue(double value);

signals:
    void doubleValueChanged(double value);

public slots:

protected:
    virtual void sliderChange(SliderChange change) override;
    virtual void wheelEvent(QWheelEvent *e) override;

private slots:
    void notifyValueChanged(int value);

private:
    double min;
    double max;
    double step;
};

#endif // DOUBLE_SLIDER_H
