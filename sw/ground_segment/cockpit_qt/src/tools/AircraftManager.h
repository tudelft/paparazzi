//
// Created by fabien on 22/02/2020.
//

#ifndef PPRZGCS_AIRCRAFTMANAGER_H
#define PPRZGCS_AIRCRAFTMANAGER_H

#include <QColor>
#include "aircraft.h"
#include <optional>
#include <map>
#include <pprzlinkQt/Message.h>
#include <QMap>
#include "PprzToolbox.h"
#include "PprzApplication.h"
#include <QtXml>
#include <QtNetwork>

class ConfigData: public QObject
{
    Q_OBJECT
public:

    ConfigData(QString ac_id, QString ac_name, QColor color, QObject* parent = nullptr);

    void setFlightPlan(QString uri);
    void setAirframe(QString uri);
    void setSettings(QString uri);

    QDomDocument getFlightPlan() {return flight_plan;}
    QDomDocument getAirframe() {return airframe;}
    QDomDocument getSettings() {return settings;}

    QString getFlightPlanUri() {return uri_flight_plan;}
    QString getAirframeUri() { return uri_airframe;}

    QString getId() {return ac_id;}
    QString getName() {return ac_name;}
    QColor getColor() {return color;}

    bool isReal() {return real;}
    void setReal(bool r) {real = r;}

    bool isComplete() {
        return !flight_plan.isNull() &&
               !airframe.isNull() &&
               !settings.isNull();
    }

signals:
    void configReady(ConfigData*);

private:

    void setData(QDomDocument* doc, QString uri);

    QString ac_id;
    QString ac_name;
    QColor color;

    QDomDocument flight_plan;
    QDomDocument airframe;
    // QDomDocument radio;
    QDomDocument settings;

    QString uri_flight_plan;
    QString uri_airframe;

    bool real;
};



class AircraftManager : public PprzTool{
    Q_OBJECT
public:
    explicit AircraftManager(PprzApplication* app, PprzToolbox* toolbox);
    virtual void setToolbox(PprzToolbox* toolbox) override;
    static AircraftManager* get() {
        return pprzApp()->toolbox()->aircraftManager();
    }

    Aircraft* getAircraft(QString id);
    optional<Aircraft*> getAircraftByName(QString name);
    QList<Aircraft*> getAircrafts();
    void newAircraftConfig(pprzlink::Message msg);
    bool aircraftExists(QString id);
    void removeAircraft(QString ac_id);

    void addFPAircraft(QString ac_id, QString flightplan);

signals:
    void waypoint_changed(Waypoint*, QString ac_id);    //wp moved from backend
    void waypoint_added(Waypoint*, QString ac_id);      //new waypoint (for flightplan edition)

private:
    static QColor parseColor(QString str);
    QDomDocument getXml(QString uri);

    QMap<QString, Aircraft*> aircrafts;

private slots:
    void addAircraft(ConfigData* config);
    void moveWaypoint(pprzlink::Message msg);

};




#endif //PPRZGCS_AIRCRAFTMANAGER_H
