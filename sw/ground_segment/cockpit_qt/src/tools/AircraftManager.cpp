//
// Created by fabien on 22/02/2020.
//

#include "AircraftManager.h"
#include <QDebug>
#include <QApplication>
#include "flightplan.h"
#include "setting_menu.h"
#include "airframe.h"
#include <QSettings>
#include "dispatcher_ui.h"
#include "gcs_utils.h"
#include <optional>

AircraftManager::AircraftManager(PprzApplication* app, PprzToolbox* toolbox) : PprzTool(app, toolbox)
{

}

void AircraftManager::setToolbox(PprzToolbox* toolbox) {
    PprzTool::setToolbox(toolbox);
    connect(PprzDispatcher::get(), &PprzDispatcher::waypoint_moved, this, &AircraftManager::moveWaypoint);
}

Aircraft* AircraftManager::getAircraft(QString id) {
    if(aircrafts.find(id) != aircrafts.end()) {
        return aircrafts[id];
    } else {
        throw runtime_error("No such aircraft!");
    }
}

optional<Aircraft*> AircraftManager::getAircraftByName(QString name) {
    (void)name;
    auto af = std::find_if(aircrafts.begin(), aircrafts.end(),
        [=](Aircraft* ac){
            return ac->name() == name;
        });
    if(af == aircrafts.end()) {
        return nullopt;
    }
    return af.value();
}

QList<Aircraft*> AircraftManager::getAircrafts() {
    return aircrafts.values();
}

void AircraftManager::moveWaypoint(pprzlink::Message msg) {
    QString ac_id;
    uint8_t wp_id = 0;
    float alt, ground_alt;
    msg.getField("ac_id", ac_id);
    msg.getField("wp_id", wp_id);
    double lat = getFloatingField(msg, "lat");
    double lon = getFloatingField(msg, "long");
    msg.getField("alt", alt);
    msg.getField("ground_alt", ground_alt);

    if(AircraftManager::get()->aircraftExists(ac_id) && wp_id != 0) {
        Waypoint* wp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getWaypoint(wp_id);
        wp->setLat(lat);
        wp->setLon(lon);
        wp->setAlt(static_cast<double>(alt));
        emit waypoint_changed(wp, ac_id);
    }
}

void AircraftManager::newAircraftConfig(pprzlink::Message msg) {
    QString id, ac_name, default_gui_color, flight_plan, airframe, radio, settings;
    msg.getField("ac_id", id);
    msg.getField("ac_name", ac_name);
    msg.getField("default_gui_color", default_gui_color);
    msg.getField("flight_plan", flight_plan);
    msg.getField("airframe", airframe);
    msg.getField("radio", radio);
    msg.getField("settings", settings);

    QColor color = parseColor(default_gui_color);

    if(aircraftExists(id)) {
        qDebug() << "Aircraft " << id << " already exits!";
        return;
    }

    if (appConfig()->contains("AC_IDS")) {
        QString ac_ids_str = appConfig()->value("AC_IDS").toString();
        if (!ac_ids_str.isEmpty()) {
            QStringList allowed_ids = ac_ids_str.split(",", Qt::SkipEmptyParts);
            if (!allowed_ids.contains(id)) {
                qDebug() << "Aircraft" << id << "ignored because of AC_IDS filter.";
                return;
            }
        }
    }

    if (appConfig()->contains("AC_IDS")) {
        QString ac_ids_str = appConfig()->value("AC_IDS").toString();
        if (!ac_ids_str.isEmpty()) {
            QStringList allowed_ids = ac_ids_str.split(",", Qt::SkipEmptyParts);
            if (!allowed_ids.contains(id)) {
                qDebug() << "Aircraft" << id << "ignored because of AC_IDS filter.";
                return;
            }
        }
    }

   auto config = new ConfigData(id, ac_name, color);

   connect(config, &ConfigData::configReady, this, &AircraftManager::addAircraft);

   config->setFlightPlan(flight_plan);
   config->setAirframe(airframe);
   config->setSettings(settings);
}

void AircraftManager::addAircraft(ConfigData* config) {
    aircrafts[config->getId()] = new Aircraft(config);
    aircrafts[config->getId()]->getStatus()->getWatcher()->init();
    emit DispatcherUi::get()->new_ac_config(config->getId());
}

void AircraftManager::addFPAircraft(QString ac_id, QString flightplan) {
    static int last_color = (int) Qt::red;
    auto color = QColor((Qt::GlobalColor)last_color++);
    auto config = new ConfigData(ac_id, ac_id, color);
    config->setReal(false);
    QFileInfo fi(flightplan);
    config->setFlightPlan(QString("file://%1").arg(fi.absoluteFilePath()));

    aircrafts[config->getId()] = new Aircraft(config);
    emit DispatcherUi::get()->new_ac_config(config->getId());
    emit DispatcherUi::get()->ac_selected(ac_id);
}

bool AircraftManager::aircraftExists(QString id) {
    return aircrafts.find(id) != aircrafts.end();
}

void AircraftManager::removeAircraft(QString ac_id) {
    if(aircraftExists(ac_id)) {
        emit DispatcherUi::get()->ac_deleted(ac_id);
        auto ac = aircrafts[ac_id];
        aircrafts.remove(ac_id);

        QString ac_selected = "";
        if(aircrafts.size() > 0) {
            ac_selected = aircrafts.firstKey();
        }
        emit DispatcherUi::get()->ac_selected(ac_selected);

        ac->deleteLater();

    }
}


QColor AircraftManager::parseColor(QString str) {
    QColor color = QColor();
    auto settings = getAppSettings();

    if(str[0] == '#' && str.size() == 13) {
        int r = QStringView(str).mid(1, 4).toInt(nullptr, 16) >> 8;
        int g = QStringView(str).mid(5, 4).toInt(nullptr, 16) >> 8;
        int b = QStringView(str).mid(9, 4).toInt(nullptr, 16) >> 8;
        color = QColor(r, g, b);
    } else {
        color = QColor(str);
    }

    if(!color.isValid()) {
        color = QColor(settings.value("aircraft_default_color").toString());
    }

    return color;
}


ConfigData::ConfigData(QString ac_id, QString ac_name, QColor color, QObject* parent) :
    QObject(parent),
    ac_id(ac_id), ac_name(ac_name), color(color), real(true)
{
}

void ConfigData::setData(QDomDocument* doc, QString uri) {
    QString separator = "://";
    int sepi = uri.indexOf(separator);

    if(uri.left(sepi) == "file") {
        QString path = uri.mid(sepi + separator.size());
        if(path == "replay") {
            return;
        }
        QFile f(path);
        if(!f.open(QIODevice::ReadOnly)) {
            throw std::runtime_error("Error while loading flightplan file");
        }
        doc->setContent(&f);
        f.close();
        if(isComplete()) {
            emit configReady(this);
        }
    } else if(uri.left(sepi) == "http") {
        auto netacc = new QNetworkAccessManager(this);
        connect(netacc, &QNetworkAccessManager::finished, this, [=](QNetworkReply* reply) {
            auto data = reply->readAll();
            if(reply->error() == QNetworkReply::NetworkError::NoError) {
                doc->setContent(data);
                if(isComplete()) {
                    emit configReady(this);
                }
            } else {
                qDebug() << "Error for" << uri << ":" << reply->error();
            }
            reply->deleteLater();
        });
        QNetworkRequest request(uri);
        netacc->get(request);
    } else {
        throw std::runtime_error("unknown protocol " + uri.left(sepi).toStdString());
    }
}

void ConfigData::setFlightPlan(QString uri) {
    uri_flight_plan = uri;
    setData(&flight_plan, uri);
}

void ConfigData::setAirframe(QString uri) {
    uri_airframe = uri;
    setData(&airframe, uri);
}

void ConfigData::setSettings(QString uri) {
    setData(&settings, uri);
}
