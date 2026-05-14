#ifndef SEGMENT_H
#define SEGMENT_H

#include "map_item.h"
#include <QBrush>
#include "waypoint_item.h"
#include "graphics_line.h"
#include <QGraphicsPolygonItem>

class PathItem : public MapItem
{
    Q_OBJECT
public:
    explicit PathItem(QString ac_id, QColor color=QColor(), double neutral_scale_zoom = 15, QObject *parent = nullptr);
    explicit PathItem(QString ac_id, PprzPalette palette, double neutral_scale_zoom = 15, QObject *parent = nullptr);
    void addPoint(WaypointItem* waypoint, QColor line_color=QColor(), bool own=false);
    void setClosedPath(bool closed);
    virtual void setHighlighted(bool h);
    virtual void setForbidHighlight(bool sh);
    virtual void setEditable(bool ed);
    virtual void updateZValue();
    virtual void updateGraphics(MapWidget* map, uint32_t update_event);
    virtual void removeFromScene(MapWidget* map);
    virtual void addToMap(MapWidget* mw);
    virtual void setVisible(bool vis) ;
    void setText(QString text) {graphics_text->setPlainText(text);}
    WaypointItem* getLastWaypoint() {return waypoints.last();}
    QList<WaypointItem*> getWaypoints() {return waypoints;}
    int count() {return waypoints.size();}
//    void setLinesIgnoreEvents(bool ignore);
//    void setLastLineIgnoreEvents(bool ignore);
    void removeLastWaypoint();
    void setFilled(bool f);

    void setStyle(GraphicsLine::Style s);

signals:

public slots:

protected:


private:
    QList<WaypointItem*> waypoints;
    QList<GraphicsLine*> lines;
    QList<QGraphicsItem*> to_be_added;
    QList<QGraphicsItem*> to_be_removed;
    QList<WaypointItem*> waypoints_to_remove;
    GraphicsLine* closing_line;
    GraphicsText* graphics_text;
    int line_width;
    QColor color;

    QGraphicsPolygonItem* polygon;
};

#endif // SEGMENT_H
