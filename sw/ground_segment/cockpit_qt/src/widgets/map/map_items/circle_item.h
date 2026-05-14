#ifndef CIRCLEITEM_H
#define CIRCLEITEM_H

#include "map_item.h"
#include "waypoint_item.h"
#include "graphics_circle.h"

class MapWidget;

class CircleItem : public MapItem
{
    Q_OBJECT
public:
    explicit CircleItem(WaypointItem* center, double radius, QString ac_id, double neutral_scale_zoom = 15);
    explicit CircleItem(WaypointItem* center, double radius, QString ac_id, PprzPalette palette, double neutral_scale_zoom = 15);
    WaypointItem* getCenter() { return center;}
    GraphicsCircle* getGraphicsCircle() {return circle;}
    void setOwnCenter(bool own) { if(own) {center->setParent(this);} }
    bool ownCenter() {return center->parent() == this;}
    void setScalable(bool scalable) {
        circle->setIgnoreEvent(!scalable);
    }
    void setFilled(bool f) {circle->setFilled(f);}
    virtual void addToMap(MapWidget* mw);
    virtual void setHighlighted(bool h);
    void setForbidHighlight(bool fh);
    virtual void setEditable(bool ed);
    virtual void updateZValue();
    virtual void setVisible(bool visible);
    virtual void updateGraphics(MapWidget* map, uint32_t update_event);
    virtual void removeFromScene(MapWidget* map);
    void setText(QString text);
    double radius() {return _radius;}
    ///
    /// \brief setRadius
    /// \param radius in meters
    ///
    void setRadius(double radius);

    void setStyle(GraphicsCircle::Style s);

signals:
    void circleMoved(Point2DLatLon latlon_pos);
    void circleScaled(double radius);

protected:

private:
    void init(WaypointItem* center);
    WaypointItem* center;
    GraphicsCircle* circle;
    GraphicsText* graphics_text;
    double _radius;
    int altitude;
    int stroke;

    bool highlighted;
};

#endif // CIRCLEITEM_H
