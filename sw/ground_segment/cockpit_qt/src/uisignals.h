#ifndef UISIGNALS_H
#define UISIGNALS_H
#include <QObject>
#include "pprzmessage.h"

class Waypoint;

class UiSignals : public virtual QObject
{
    Q_OBJECT
signals:
    void ac_selected(int);
    void create_waypoint(Waypoint*);
};

#endif // UISIGNALS_H
