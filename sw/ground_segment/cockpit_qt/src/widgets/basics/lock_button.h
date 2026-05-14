#ifndef LOCK_BUTTON_H
#define LOCK_BUTTON_H

#include <QWidget>
#include <QIcon>
#include <QMouseEvent>
#include <QEvent>
#include <QPaintEvent>

class LockButton : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(QColor color_hover MEMBER m_color_hover DESIGNABLE true)
    Q_PROPERTY(QColor color_idle  MEMBER m_color_idle DESIGNABLE true)
    Q_PROPERTY(QColor color_pressed  MEMBER m_color_pressed DESIGNABLE true)
    Q_PROPERTY(int size READ getSizeProperty WRITE setSizeProperty DESIGNABLE true)
public:
    explicit LockButton(QIcon icon, QWidget *parent = nullptr);
    void setSizeProperty(int s) {setSize(QSize(s,s));}
    int getSizeProperty() {return _size.width();}

    QSize sizeHint() const;
    QSize minimumSizeHint() const;

    void setActive(bool a) { active = a; repaint();}
    bool isActive() {return active;}
    bool isLocked(){return _locked;}
    void setWidget(QWidget* w) {_widget = w;}
    QWidget* widget(){return _widget;}
    void setSize(QSize s);
    void setActiveSide(bool activeLeft);

protected:
    virtual void enterEvent(QEvent* e);
    virtual void leaveEvent(QEvent* e);
    virtual void paintEvent(QPaintEvent* e);
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);

signals:
    void clicked(bool);
    void locked(bool);

public slots:

private:

    enum State {
        IDLE,
        HOVER,
        PRESSED
    };

    State state;

    QSize _size;
    QMargins icon_margins;

    QIcon icon;
    QIcon icon_lock;
    QIcon icon_unlocked;
    bool activeSide;

    bool _locked;
    bool active;

    QColor m_color_idle;
    QColor m_color_hover;
    QColor m_color_pressed;

    QRect lock_rect;
    QRect active_rect;

    QWidget* _widget;
};

#endif // LOCK_BUTTON_H
