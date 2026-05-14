#ifndef MBPUSHBUTTON_H
#define MBPUSHBUTTON_H

#include <QWidget>
#include <QPushButton>

class MbPushButton : public QPushButton
{
    Q_OBJECT
public:
    explicit MbPushButton(QWidget *parent = nullptr);
    explicit MbPushButton(QString txt, QWidget *parent = nullptr);

protected:
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void enterEvent(QEnterEvent *event);
    void leaveEvent(QEvent *event);

signals:
    void rightClicked();
    void enter();
    void leave();

public slots:
};

#endif // MBPUSHBUTTON_H
