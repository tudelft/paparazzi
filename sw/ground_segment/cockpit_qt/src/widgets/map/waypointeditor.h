#ifndef WAYPOINTEDITOR_H
#define WAYPOINTEDITOR_H

#include <QtWidgets>
#include <memory>
#include "waypoint.h"
#include "waypoint_item.h"
#include "dispatcher_ui.h"

class WaypointEditor : public QDialog
{
    Q_OBJECT
public:
    explicit WaypointEditor(WaypointItem* wi, QString ac_id, QWidget *parent = nullptr);

signals:

public slots:

private:
    WaypointItem* wi;
    QString ac_id;
};

#endif // WAYPOINTEDITOR_H
