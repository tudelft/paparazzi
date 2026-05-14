#ifndef JAUGELABEL_H
#define JAUGELABEL_H

#include <QWidget>

class JaugeLabel : public QWidget
{
    Q_OBJECT
public:
    explicit JaugeLabel(QWidget *parent = nullptr);
    explicit JaugeLabel(double min, double max, QString unit, QWidget *parent = nullptr);
    QSize minimumSizeHint() const override;


    void setValue(double v) {value = v; update();}
    void setStatus(bool s) {status = s; update();}
    void setPrecision(int p) {precision = p;}

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QBrush brushLeft;
    QBrush brushBackEnabled;
    QBrush brushBackDisabled;
    double min, max;
    QString unit;
    int precision;

    double value;
    bool status;

    QSize minSize;

};

#endif // JAUGELABEL_H
