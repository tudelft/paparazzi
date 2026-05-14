#ifndef AIRCRAFTWATCHER_H
#define AIRCRAFTWATCHER_H

#include <QObject>
#include "pprz_dispatcher.h"

class AircraftWatcher : public QObject
{
    Q_OBJECT
public:
    explicit AircraftWatcher(QString ac_id, QObject *parent = nullptr);
    void init();

    void watch_bat(pprzlink::Message msg);
    void watch_links(pprzlink::Message msg);

    enum BatStatus{
        OK,
        LOW,
        CRITIC,
        CATASTROPHIC,
    };

    enum LinkStatus {
        LINK_OK,
        LINK_LOST,
        LINK_PARTIALY_LOST
    };

signals:
    void bat_status(BatStatus);
    void link_status(LinkStatus);

private:


    QString ac_id;

    float bat_low;
    float bat_critic;
    float bat_catastrophic;

    QMap<QString, float> link_times;

};

#endif // AIRCRAFTWATCHER_H
