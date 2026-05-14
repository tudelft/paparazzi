#ifndef GRAPHICSOBJECT_H
#define GRAPHICSOBJECT_H

#include <QObject>
#include <QGraphicsSceneMouseEvent>
#include <QColor>
#include <QTimer>
#include "pprzpalette.h"

class GraphicsObject : public QObject
{
    Q_OBJECT
public:

    enum Style {
        DEFAULT,
        CURRENT_NAV,
        CARROT,
        GCS,
        DCSHOT,
        CRASH,
    };

    enum Animation {
        NONE,
        WP_MOVING,
    };

    explicit GraphicsObject(PprzPalette palette, QObject *parent = nullptr);
    bool isHighlighted() {return highlighted;}
    virtual void setHighlighted(bool h);
    void setForbidHighlight(bool fh) {forbid_highlight = fh;}
    void setEditable(bool ed) {editable = ed;}
    void setScaleFactor(double s) {scale_factor = s;}
    void setIgnoreEvent(bool ignore) {ignore_events = ignore;}
    void setStyle(Style s) {style = s;}
    void setAnimation(Animation a);

protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
    virtual void changeFocus() = 0;
    bool forbid_highlight;
    bool editable;
    double scale_factor;
    bool ignore_events;
    Style style;
    Animation animation;
    QTimer* animation_timer;
    PprzPalette palette;

signals:
    void objectClicked(QPointF scene_pos);
    void objectDoubleClicked(QPointF scene_pos);
    void objectGainedHighlight();

private:
    bool highlighted;



};

#endif // GRAPHICSOBJECT_H
