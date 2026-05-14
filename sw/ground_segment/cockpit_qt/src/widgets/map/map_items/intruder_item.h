#ifndef INTRUDERITEM_H
#define INTRUDERITEM_H

#include "map_item.h"
#include "graphics_intruder.h"
#include "graphics_text.h"

class IntruderItem : public MapItem
{
    Q_OBJECT
public:
    explicit IntruderItem(QString name, Point2DLatLon pt, double course, double neutral_scale_zoom = 15, QObject *parent = nullptr);
    virtual void addToMap(MapWidget* map) override;
    virtual void updateGraphics(MapWidget* map, uint32_t update_event) override;
    virtual void removeFromScene(MapWidget* map) override;
    virtual void setForbidHighlight(bool fh) override;
    virtual void setEditable(bool ed) override;
    virtual void updateZValue() override;
    virtual void setVisible(bool visible) override;

    void setPosition(Point2DLatLon pt);
    void setCourse(double c);
    void setAltitude(double a);
    void setSpeed(double s);
    void setClimb(double c);

signals:

private:
    GraphicsIntruder* graphics_intruder;
    GraphicsText* graphics_text;

    QString name;
    Point2DLatLon latlon;
    double course;
    double altitude;
    double speed;
    double climb;

};

#endif // INTRUDERITEM_H
