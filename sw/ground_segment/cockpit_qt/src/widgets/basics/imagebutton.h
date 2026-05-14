#ifndef IMAGEBUTTON_H
#define IMAGEBUTTON_H

#include <QWidget>
#include <QPushButton>
#include <QIcon>

class ImageButton : public QPushButton
{
    Q_OBJECT
public:
    explicit ImageButton(QIcon normal, QSize size, bool mask, QWidget *parent = nullptr);

    void setNormalIcon(QIcon ic) {icon_normal = ic;}
    void setHoverIcon(QIcon ic) {icon_hover = ic;}
    void setPressedIcon(QIcon ic) {icon_pressed = ic;}

protected:
    virtual void enterEvent(QEnterEvent* e);
    virtual void leaveEvent(QEvent* e);
    virtual void paintEvent(QPaintEvent* e);
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);

signals:

public slots:

private:
    QIcon icon_normal;
    QIcon icon_hover;
    QIcon icon_pressed;
};

#endif // IMAGEBUTTON_H
