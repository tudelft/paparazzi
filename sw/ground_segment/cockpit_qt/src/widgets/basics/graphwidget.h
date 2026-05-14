#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QtWidgets>

class GraphWidget : public QWidget
{
    Q_OBJECT
public:

    struct DataPoint {
        double value;
        QTime time;
    };

    struct Params {
        double max;
        double min;
        bool autoscale;
        double scale;
    };

    explicit GraphWidget(QWidget *parent = nullptr);

    void setParams(struct Params p) {
        if(p.min > p.max) {
            auto min = p.max;
            p.max = p.min;
            p.min = min;
        }
        params = p;
        if(p.autoscale == true && !data.isEmpty()) {
            params.min = data.last().value;
            params.max = params.min;
        }
    }
    struct Params getParams() {return params;}
    void setAutoscale(bool a) {
        auto new_params = params;
        new_params.autoscale = a;
        params = new_params;
    }

    void setHistory(int h) {history = h; update();}
    int getHistory() {return history;}
    void pushData(double d);
    void clearData();

    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void leaveEvent(QEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

signals:
    void autoscaleChanged(bool);

private:
    struct Params params;

    QList<struct DataPoint> data;
    int history;

    QPoint mousePos;

    std::function<double(int)> val_of_y;
    double val_last_move;
    bool moving;
    int wheelAccumulator;
};

#endif // GRAPHWIDGET_H
