#ifndef FLIGHTPLAN_H
#define FLIGHTPLAN_H

#include <QString>
#include "waypoint.h"
#include "sector.h"
#include "block.h"
#include <memory>
#include <QtXml>
#include "point2dlatlon.h"
#include <QObject>

struct BlockGroup {
    QString group_name;
    QList<shared_ptr<Block>> blocks;
};

struct Exception {
    QString cond;
    QString deroute;
};

struct Variable {
    enum Type {
        VARIABLE,
        ABI_BINDING,
    };

    Variable(Type t, QString var_name) {
        _var_name = var_name;
        type = t;
    }

    Type type;
    QString _var_name;
    map<QString, QString> attributes;
};


class FlightPlan: public QObject
{
    Q_OBJECT
public:

    FlightPlan(QString ac_id, QObject* parent=nullptr);
    FlightPlan(QString ac_id, QDomDocument doc, QObject* parent=nullptr);

    QList<Waypoint*>& getWaypoints() {return  waypoints;}
    Waypoint* getWaypoint(uint8_t id);
    Waypoint* getWaypoint(QString name);
    QList<shared_ptr<Block>>& getBlocks() {return  blocks;}
    QList<shared_ptr<BlockGroup>> getGroups();
    QList<shared_ptr<Exception>> getExeptions() {return  exceptions;}
    QList<shared_ptr<Variable>> getVariables() {return  variables;}
    QList<shared_ptr<Sector>> getSectors() {return  sectors;}
    shared_ptr<Block> getBlock(uint8_t id);
    double getDefaultAltitude() {return defaultAlt;}
    double getMaxDistFromHome() {return max_dist_from_home;}
    double getGroundAlt() {return ground_alt;}
    void setGroundAlt(double ga) {ground_alt = ga;}
    Waypoint* getOrigin() {return origin;}
    Waypoint::WpFrame getFrame() {return frame_type;}

    Waypoint* addWaypoint(QString name, double x, double y);
    Waypoint* addWaypoint(QString name, Point2DLatLon latlon);

    QString requestNewName(QString requeste_name);

    /**
     * @brief boundingBox of the flightPlan (NW, SE)
     * @return NW, SE
     */
    std::tuple<Point2DLatLon, Point2DLatLon> boundingBox();

    /**
     * @brief boundingBoxWith boundingBox of the flightPlan + given point
     * @param pt
     * @return NW, SE
     */
    std::tuple<Point2DLatLon, Point2DLatLon> boundingBoxWith(Point2DLatLon pt);

private:
    void parse_exceptions(QDomElement exs);
    void parse_variables(QDomElement vars);
    void parse_sectors(QDomElement secs);
    void parse_waypoints(QDomElement wps);
    void parse_blocks(QDomElement blks);
    void parse_block_stages(QDomElement blk, shared_ptr<Block> block);

    QList<Waypoint*> waypoints;
    QList<shared_ptr<Exception>> exceptions;
    QList<shared_ptr<Block>> blocks;
    QList<shared_ptr<Variable>> variables;
    QList<shared_ptr<Sector>> sectors;
    Waypoint* origin;

    double defaultAlt;
    QString name;
    double max_dist_from_home;
    double ground_alt;
    double security_height;

    Waypoint::WpFrame frame_type;

    QString ac_id;


};

#endif // FLIGHTPLAN_H
