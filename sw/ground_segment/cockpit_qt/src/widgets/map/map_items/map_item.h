#ifndef MAPITEM_H
#define MAPITEM_H

#include <QGraphicsItem>
#include <QGraphicsItemGroup>
#include "point2dlatlon.h"
#include <QGraphicsSceneMouseEvent>
#include <QVariant>
#include "pprzpalette.h"

class MapWidget;

enum MoveState {
    MIS_IDLE,
    MIS_PRESSED,
    MIS_MOVED,
};

enum UpdateEvent {
    ITEM_CHANGED    = 0x01,
    MAP_MOVED       = 0x02,
    MAP_ZOOMED      = 0x04,
    MAP_ROTATED     = 0x08,
    MOUSE_MOVED     = 0x10,
    ANY             = 0xFF,
};

class MapItem : public QObject
{
    Q_OBJECT
public:
    MapItem(QString ac_id, double neutral_scale_zoom = 15, QObject *parent = nullptr);
    MapItem(QString ac_id, PprzPalette palette, double neutral_scale_zoom = 15, QObject *parent = nullptr);
    QColor unfocusedColor(const QColor&);
    QColor trackUnfocusedColor(const QColor&);
    QColor labelUnfocusedColor(const QColor&);
    virtual void addToMap(MapWidget* map) = 0;
    virtual void updateGraphics(MapWidget* map, uint32_t update_event=static_cast<uint32_t>(UpdateEvent::ANY)) = 0;
    virtual void removeFromScene(MapWidget* map) = 0;
    virtual void setHighlighted(bool h) {
        highlighted = h;
        z_value = h ? z_value_highlighted : z_value_unhighlighted;
    }
    virtual void setForbidHighlight(bool fh) = 0;
    virtual void setEditable(bool ed) = 0;
    double zoomFactor() {return zoom_factor;}
    double neutralScaleZoom() {return neutral_scale_zoom;}
    void setZoomFactor(double zf) {zoom_factor = zf;}
    virtual void updateZValue() = 0;
    bool isVisible() {return visible;}
    virtual void setVisible(bool visible) = 0;
    qreal zValue() {return z_value;}
    QString acId() {return ac_id;}
    void requestUpdate() {emit itemChanged();}
    void setZValues(qreal zh, qreal zuh) {
        z_value_highlighted = zh;
        z_value_unhighlighted = zuh;
        updateZValue();
    }
    void setPalette(PprzPalette p) {
        palette = p;
    }

signals:
    void itemClicked(QPointF scene_pos);
    void itemDoubleClicked(QPointF scene_pos);
    void itemGainedHighlight();
    void itemChanged();

protected:
    double getScale(double zoom, double scale_factor);

    const QString ac_id;
    PprzPalette palette;

    double zoom_factor;
    double neutral_scale_zoom;
    qreal z_value;
    bool highlighted;
    bool visible;

    qreal z_value_highlighted;
    qreal z_value_unhighlighted;

};

#endif // MAPITEM_H
