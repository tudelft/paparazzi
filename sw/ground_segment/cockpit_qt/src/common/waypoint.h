#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <memory>
#include <iostream>
#include <map>
#include <QtXml>
#include <memory>
#include <QObject>

class Point2DLatLon;
class FlightPlan;

class Waypoint: public QObject
{
    Q_OBJECT
public:

    enum WpFrame {
        WGS84,
        UTM,
        LTP
    };

    Waypoint(Waypoint* original, QObject* parent=nullptr);
    Waypoint(QString name, uint8_t id, Point2DLatLon pos, double alt, QObject* parent=nullptr);
    Waypoint(QDomElement wp, uint8_t wp_id, Waypoint* orig, double defaultAlt, double ground_alt, WpFrame frame_type, QObject* parent=nullptr);

    uint8_t getId() const {return id;}
    double getLat() const;
    double getLon() const;
    void setLat (double lat);
    void setLon(double lon);
    void setRelative(WpFrame frame, double dx, double dy, Waypoint* wp=nullptr);
    void getRelative(WpFrame frame, double &dx, double &dy, Waypoint* wp=nullptr);

    void setAlt(double alt) {this->alt = alt;}
    double getAlt() const {return alt;}
    WpFrame getType() const {return type;}
    QString getName() const {return name;}
    void setName(QString new_name);
    Waypoint* getOrigin() {return origin;}
    void setOrigin(Waypoint* wp) {origin = wp;}
    QMap<QString, QString>& getXmlAttributes() { return xml_attibutes;}
    FlightPlan* getFlightPlan() {return flight_plan;}

    friend std::ostream& operator<<(std::ostream& os, const Waypoint& wp);

private:

    void affectFlightPlan();

    FlightPlan* flight_plan;

    WpFrame type;

    uint8_t id;

    double lat;
    double lon;

    Waypoint* origin;

    double alt;

    QString name;

    QMap<QString, QString> xml_attibutes;
};


#endif // WAYPOINT_H
