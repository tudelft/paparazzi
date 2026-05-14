#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include <QObject>
#include <QColor>
#include "flightplan.h"
#include "setting_menu.h"
#include "airframe.h"
#include "point2dlatlon.h"
#include "pprz_dispatcher.h"
#include "aircraft_status.h"

class ConfigData;

class Aircraft: public QObject
{
    Q_OBJECT
public:
    Aircraft(ConfigData* config, QObject* parent=nullptr);
    QColor getColor(){return color;}
    QString getId(){return ac_id;}
    QString getIcon(){return icon;}
    QString name() {return _name;}
    QString speechName();
    FlightPlan* getFlightPlan() {return flight_plan;}
    //SettingMenu& getSettingMenu() {return setting_menu;}
    SettingMenu* getSettingMenu() {return setting_menu;}
    Airframe* getAirframe() {return airframe;}
    AircraftStatus* getStatus() {return status;}
    ConfigData* getConfig() {return config;}
    QList<ChecklistItem*> getChecklist() {return checklist;}
    void setConfig(ConfigData* c) {config = c;}

    Point2DLatLon getPosition() {return position;}
    void setPosition(Point2DLatLon pos) {position = pos;}

    void setSetting(Setting*, QString value);
    void setSetting(Setting*, float value);
    void setSetting(uint8_t setting_no, float value);

    bool isReal() {return real;}
    bool checklistFinished();

private:
    QString ac_id;
    QColor color;
    QString icon;
    QString _name;
    FlightPlan* flight_plan;
    SettingMenu* setting_menu;
    Airframe* airframe;
    ConfigData* config;
    QList<ChecklistItem*> checklist;

    Point2DLatLon position;

    AircraftStatus* status;

    // false if this is a "flight plan only" AC. Just to edit the flight plan.
    bool real;
    QString _speech_name;


};

#endif // AIRCRAFT_H
