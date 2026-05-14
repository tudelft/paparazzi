#ifndef COLORLABEL_H
#define COLORLABEL_H

#include <QWidget>

class ColorLabel : public QWidget
{
    Q_OBJECT
public:
    explicit ColorLabel(qreal radius, QWidget *parent = nullptr);
    void setText(QString t) {text = t; update();}
    void setBrush(QBrush b) {brush = b; update();}
    void setMinSize(QSize s) {minSize = s;}

    QSize minimumSizeHint() const override;

signals:

public slots:

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QString text;
    QBrush brush;
    QSize minSize;
    qreal radius;
};

#endif // COLORLABEL_H
