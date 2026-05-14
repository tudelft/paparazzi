#ifndef AC_SELECTOR_H
#define AC_SELECTOR_H

#include <QWidget>
#include <configurable.h>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QWheelEvent>

class ACSelector : public QWidget, public Configurable
{
    Q_OBJECT

public:
    explicit ACSelector(QWidget *parent = nullptr);
    void configure(QDomElement) {};

    QSize sizeHint() const;
    QSize minimumSizeHint() const;

    void removeAC(QString ac_id);

protected:
//    virtual void enterEvent(QEvent* e);
//    virtual void leaveEvent(QEvent* e);
    virtual void paintEvent(QPaintEvent* e);
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);
    virtual void wheelEvent(QWheelEvent *event);

private:
    void handleNewAC(QString ac_id);


    static int const ARROW_WIDTH = 20;

    QList<QString> ac_ids;
    QString current_ac_id;

};

#endif // AC_SELECTOR_H
