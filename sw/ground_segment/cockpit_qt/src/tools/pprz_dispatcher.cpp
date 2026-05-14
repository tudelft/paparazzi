#include "pprz_dispatcher.h"
#include <iostream>
#include "AircraftManager.h"
#include "dispatcher_ui.h"
#include <QDebug>
#include <PprzApplication.h>
#include <QDateTime>
#include "pprzmain.h"
#include <QSettings>
#include "gcs_utils.h"

using namespace std;

Q_DECLARE_METATYPE(pprzlink::Message)

PprzDispatcher::PprzDispatcher(PprzApplication* app, PprzToolbox* toolbox) : PprzTool(app, toolbox), started(false), silent_mode(false)
{

}

void PprzDispatcher::setToolbox(PprzToolbox* toolbox) {
    PprzTool::setToolbox(toolbox);

    auto settings_path = appConfig()->value("SETTINGS_PATH").toString();

    QSettings settings(settings_path, QSettings::IniFormat);

    QString ivy_name = settings.value("ivy/name").toString();
    pprzlink_id = settings.value("pprzlink/id").toString();
    auto messages = appConfig()->value("MESSAGES").toString();

    dict = new pprzlink::MessageDictionary(messages);
    link = new pprzlink::IvyQtLink(*dict, ivy_name, this);

    connect(link, &pprzlink::IvyQtLink::serverConnected, this, [=]() {
        started = true;
       requestAircrafts();
    });

    qRegisterMetaType<pprzlink::Message>();

    connect(this, &PprzDispatcher::dl_values, this, &PprzDispatcher::updateSettings);

    connect(&server_check_timer, &QTimer::timeout, this, [this]() {
        auto now = QDateTime::currentMSecsSinceEpoch();
        if(now - time_msg_server > 1000) {
            pprzApp()->mainWindow()->setServerStatus(false);
        } else {
            pprzApp()->mainWindow()->setServerStatus(true);
        }
    });


    long bid = link->BindMessage(dict->getDefinition("WAYPOINT_MOVED"), nullptr,
        [=](QString ac_id, pprzlink::Message msg) {
            (void)ac_id;
            QString id;
            uint8_t wp_id = 0;
            msg.getField("ac_id", id);
            msg.getField("wp_id", wp_id);
            if(AircraftManager::get()->aircraftExists(id) && wp_id != 0) {
                emit(waypoint_moved(msg));
            }
    });
    _bindIds.append(bid);

    bid = link->BindMessage(dict->getDefinition("AIRCRAFT_DIE"), nullptr,
        [=](QString ac_id, pprzlink::Message msg) {
            (void)ac_id;
            QString id;
            msg.getField("ac_id", id);

            if(AircraftManager::get()->aircraftExists(id)) {
                AircraftManager::get()->removeAircraft(id);
            }
        }
    );
    _bindIds.append(bid);

    bindDeftoSignal("AP_STATUS", &PprzDispatcher::ap_status);
    bindDeftoSignal("NAV_STATUS", &PprzDispatcher::nav_status);
    bindDeftoSignal("CIRCLE_STATUS", &PprzDispatcher::circle_status);
    bindDeftoSignal("SEGMENT_STATUS", &PprzDispatcher::segment_status);
    bindDeftoSignal("ENGINE_STATUS", &PprzDispatcher::engine_status);
    bindDeftoSignal("DL_VALUES", &PprzDispatcher::dl_values);
    bindDeftoSignal("TELEMETRY_STATUS", &PprzDispatcher::telemetry_status);
    bindDeftoSignal("FLY_BY_WIRE", &PprzDispatcher::fly_by_wire);
    bindDeftoSignal("FLIGHT_PARAM", &PprzDispatcher::flight_param);
    bindDeftoSignal("SVSINFO", &PprzDispatcher::svsinfo);


    connect(DispatcherUi::get(), &DispatcherUi::move_waypoint_ui, this,
        [=](Waypoint* wp, QString ac_id) {
            //Do not send the message if this is a "flight plan only" AC.
            if(AircraftManager::get()->getAircraft(ac_id)->isReal()) {
                pprzlink::Message msg(dict->getDefinition("MOVE_WAYPOINT"));
                msg.setSenderId(pprzlink_id);
                msg.addField("ac_id", ac_id);
                msg.addField("wp_id", wp->getId());
                msg.addField("lat", wp->getLat());
                msg.addField("long", wp->getLon());
                msg.addField("alt", wp->getAlt());
                if(started) {
                    this->sendMessage(msg);
                }
            }
        });

    bid = link->BindMessage(dict->getDefinition("NEW_AIRCRAFT"), nullptr,
        [=](QString ac_id, pprzlink::Message msg) {
            (void)ac_id;
            QString id;
            msg.getField("ac_id", id);
            requestConfig(id);
        }
    );
    _bindIds.append(bid);
}

PprzDispatcher::~PprzDispatcher() {
    unbindAll();
}

void PprzDispatcher::unbindAll() {
    const QList<long> &constList = _bindIds;
    for(auto id: constList) {
        (void)id;
        link->UnbindMessage(id);
    }
    _bindIds.clear();
}

void PprzDispatcher::bindDeftoSignal(QString const &name, sig_ptr_t sig) {
    long bid = link->BindMessage(dict->getDefinition(name), nullptr,
        [=](QString sender, pprzlink::Message msg) {
            if(sender == "ground") {
                time_msg_server = QDateTime::currentMSecsSinceEpoch();
            }

            QString id;
            msg.getField("ac_id", id);

            if(AircraftManager::get()->aircraftExists(id)) {
                if(DispatcherUi::get()->getSelectedAcId() == "") {
                    emit(DispatcherUi::get()->ac_selected(QString(id)));
                }
                emit((this->*sig)(msg));
                AircraftManager::get()->getAircraft(id)->getStatus()->updateMessage(msg);
            }
        }
    );
    _bindIds.append(bid);
}

void PprzDispatcher::requestConfig(QString ac_id) {
    pprzlink::Message reqConfig(dict->getDefinition("CONFIG_REQ"));
    reqConfig.addField("ac_id", ac_id);
    reqConfig.setSenderId(pprzlink_id);
    link->sendRequest(reqConfig, [=](QString ai, pprzlink::Message msg) {
        (void)ai;
        AircraftManager::get()->newAircraftConfig(msg);
    });
}

void PprzDispatcher::sendMessage(pprzlink::Message msg) {
    assert(started);
    if(!silent_mode) {
        msg.setSenderId(pprzlink_id);
        link->sendMessage(msg);
    }
}

void PprzDispatcher::requestAircrafts() {
    auto def_ar = dict->getDefinition("AIRCRAFTS_REQ");
    pprzlink::Message msg(def_ar);
    msg.setSenderId(pprzlink_id);

    link->sendRequest(msg, [=](QString sender, pprzlink::Message msg) {
        (void)sender;
        QString ac_list;
        msg.getField("ac_list", ac_list);
        for(QString id: ac_list.split(",")) {
            if(id == "") {continue;}
            if(!AircraftManager::get()->aircraftExists(id)) {
                requestConfig(id);
            } else {
                qDebug() << id << " already exists.";
            }
        }
    });
}

long PprzDispatcher::bind(QString msg_name, pprzlink::messageCallback_t cb) {
    try {
        long ret = link->BindMessage(dict->getDefinition(msg_name), nullptr, cb);
        _bindIds.append(ret);
        return ret;
    } catch(const pprzlink::no_such_message& e) {
        // Suppress message parsing errors for unconfigured telemetry (e.g. GRID, SLAM)
    }
    return -1;
}

long PprzDispatcher::bind(QString msg_name, QObject* context, pprzlink::messageCallback_t cb) {
    try
    {
         long ret = link->BindMessage(dict->getDefinition(msg_name), context, cb);
        _bindIds.append(ret);
        return ret;
    }
    catch(const pprzlink::no_such_message& e) {
        // Suppress message parsing errors for unconfigured telemetry (e.g. GRID, SLAM)
    }
    return -1;
}

void PprzDispatcher::unBind(long bid) {
    link->UnbindMessage(bid);
    _bindIds.removeAll(bid);
}

void PprzDispatcher::stop() {
    link->stop();
}

void PprzDispatcher::start() {
    QSettings settings(appConfig()->value("SETTINGS_PATH").toString(), QSettings::IniFormat);
    link->start(appConfig()->value("IVY_BUS").toString());

    server_check_timer.setInterval(1000);
    server_check_timer.start();
}


void PprzDispatcher::updateSettings(pprzlink::Message msg) {
    QString ac_id;
    QString values;
    msg.getField("ac_id", ac_id);
    msg.getField("values", values);
    QString id = QString(ac_id);

    auto ac = AircraftManager::get()->getAircraft(id);

    auto settings = ac->getSettingMenu()->getAllSettings();
    sort(settings.begin(), settings.end(),
        [](Setting* sl, Setting* sr) {
                return sl->getNo() < sr->getNo();
    });

    if(settings.size() == 0) {
        return;
    }

    size_t i=0;
    for(QString &token: values.split(",")) {
        if(settings.size() <= static_cast<int>(i)) {
            break;
        }
        if(token != "" && token != "?") {
            double s = token.toDouble();
            assert(settings[i]->getNo() == static_cast<uint8_t>(i));
            float value = static_cast<float>(s);
            emit DispatcherUi::get()->settingUpdated(ac_id, settings[i], value);
        }
        i++;
    }
}
