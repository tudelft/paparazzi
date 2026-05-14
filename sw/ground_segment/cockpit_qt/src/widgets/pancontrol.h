#ifndef PANCONTROL_H
#define PANCONTROL_H

#include <QWidget>
#include <QPolygonF>
#include <QTimer>

class PanControl : public QWidget
{
    Q_OBJECT
public:
    explicit PanControl(QWidget *parent = nullptr);
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

signals:
    void panRequested(int dx, int dy);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    void setArrows();
    bool isInArrow(const QMouseEvent* event, int& dx, int& dy);

    QPolygonF arrowPolys[4]; // N, S, W, E
    int dir_dx[4], dir_dy[4];
    QTimer* move_timer;
    int current_dx;
    int current_dy;
};

#endif // PANCONTROL_H
