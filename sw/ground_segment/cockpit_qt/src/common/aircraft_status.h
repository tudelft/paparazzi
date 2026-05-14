#ifndef AIRCRAFTSTATUS_H
#define AIRCRAFTSTATUS_H

#include <QObject>
#include <QMap>
#include "pprz_dispatcher.h"
#include <optional>
#include "aircraft_watcher.h"


class AircraftStatus : public QObject
{
    Q_OBJECT
public:
    explicit AircraftStatus(QString ac_id, QObject *parent = nullptr);

    void updateMessage(pprzlink::Message msg);
    std::optional<pprzlink::Message> getMessage(QString name);
    QMap<QString, pprzlink::Message> getTelemetryMessages() {return telemetry_messages;}

    AircraftWatcher* getWatcher() {return watcher;}

signals:
    void flight_param();
    void ap_status();
    void nav_status();
    void circle_status();
    void segment_status();
    void engine_status();
    void waypoint_moved();
    void dl_values();
    void telemetry_status();
    void fly_by_wire();
    void svsinfo();
    void text_message(QString text);

public slots:

private:
    QString ac_id;
    qint64 last_bat_warn_time;

    QString last_ap_mode;
    QString last_block_name;
    bool in_kill_mode = false;
    bool ground_prox = false;
    int last_gps_acc = 0;
    QString last_rc_mode;

    QMap<QString, bool> last_links_state;
    bool all_links_lost_reported = false;

    QMap<QString, pprzlink::Message> last_messages;
    QMap<QString, pprzlink::Message> telemetry_messages;

    AircraftWatcher* watcher;
};

#endif // AIRCRAFTSTATUS_H
