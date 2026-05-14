#ifndef GRAPHLABEL_H
#define GRAPHLABEL_H

#include <QWidget>
#include <QTime>

class GraphLabel : public QWidget
{
    Q_OBJECT
public:
    explicit GraphLabel(QWidget *parent = nullptr);
    explicit GraphLabel(double min, double max, QWidget *parent = nullptr);

    void pushData(double value);
    void setDualText(bool d) {dual_text = d;}
    void setSecondayText(QString t) {secondary_text = t;}
    void setPrecision(int p) {precision = p;}
    void setUnit(QString u) {unit = u;}

    void setIndicator(bool i) {indicator = i;}
    void setIndicatorAngle(double angle) {indicator_angle = angle;}


    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent*) override;
    //void enterEvent(QEvent*) override;

signals:

public slots:

private:
    QBrush brushTop, brushBottom;
    double timespan;  // total time the graph represent (in seconds)
    double min, max;

    QList<std::tuple<QTime, double>> data;
    QString unit;

    int precision;

    bool dual_text;
    QString secondary_text;

    bool indicator;
    double indicator_angle;
};

#endif // GRAPHLABEL_H
