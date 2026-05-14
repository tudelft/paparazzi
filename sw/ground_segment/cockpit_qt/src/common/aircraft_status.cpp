#include "aircraft_status.h"
#include <QDebug>
#include <QDateTime>
#include "coordinatestransform.h"
#include "AircraftManager.h"

AircraftStatus::AircraftStatus(QString ac_id, QObject *parent) : QObject(parent),
    ac_id(ac_id), last_bat_warn_time(0)
{
    watcher = new AircraftWatcher(ac_id, this);

    //listen for NAVIGATION_REF to update origin waypoint of fixedwings
    PprzDispatcher::get()->bind("NAVIGATION_REF", this, [=](QString sender, pprzlink::Message msg) {
        if(sender == ac_id) {
            int32_t utm_east, utm_north;
            uint8_t utm_zone;
            float ground_alt;
            msg.getField("utm_east", utm_east);
            msg.getField("utm_north", utm_north);
            msg.getField("utm_zone", utm_zone);
            msg.getField("ground_alt", ground_alt);

            auto latlon = CoordinatesTransform::get()->utm_to_wgs84(utm_east, utm_north, utm_zone, true);
            auto orig = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getOrigin();
            orig->setLat(latlon.lat());
            orig->setLon(latlon.lon());
        }
    });


    PprzDispatcher::get()->bind("BAT_LOW", this, [=](QString sender, pprzlink::Message msg) {
        (void)sender;
        QString id;
        try {
            msg.getField("ac_id", id);
            if(id == ac_id) {
                QString level;
                msg.getField("level", level);
                qint64 now = QDateTime::currentSecsSinceEpoch();
                if (now > last_bat_warn_time + 10) {
                    emit text_message(QString("%1, BAT LOW %2").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, level));
                    last_bat_warn_time = now;
                }
            }
        } catch(...) {}
    });

    PprzDispatcher::get()->bind("TELEMETRY_ERROR", this, [=](QString sender, pprzlink::Message msg) {
        (void)sender;
        QString id;
        try {
            msg.getField("ac_id", id);
            if(id == ac_id) {
                QString message;
                msg.getField("message", message);
                emit text_message(QString("gcs, %1").arg(message));
            }
        } catch(...) {}
    });

    PprzDispatcher::get()->bind("INFO_MSG", this, [=](QString sender, pprzlink::Message msg) {
        if(sender == ac_id) {
            QString text;
            try {
                msg.getField("msg", text);
                emit text_message(QString("%1, %2").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, text));
            } catch(...) {}
        }
    });

    PprzDispatcher::get()->bind("AUTOPILOT_VERSION", this, [=](QString sender, pprzlink::Message msg) {
        if(sender == ac_id) {
            QString desc;
            try {
                msg.getField("desc", desc);
                emit text_message(QString("%1, Version %2").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, desc));
            } catch(...) {}
        }
    });

    PprzDispatcher::get()->bind("TCAS_TA", this, [=](QString sender, pprzlink::Message msg) {
        if(sender == ac_id) {
            uint8_t other_id;
            try {
                msg.getField("ac_id", other_id);
                emit text_message(QString("tcas TA : %1 -> %2").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, AircraftManager::get()->getAircraft(QString::number(other_id)) ? AircraftManager::get()->getAircraft(QString::number(other_id))->speechName() : QString::number(other_id)));
            } catch(...) {}
        }
    });

    PprzDispatcher::get()->bind("TCAS_RA", this, [=](QString sender, pprzlink::Message msg) {
        if(sender == ac_id) {
            uint8_t other_id;
            uint8_t resolve;
            try {
                msg.getField("ac_id", other_id);
                msg.getField("resolve", resolve);
                QString resolve_str = "NONE";
                if (resolve == 1) resolve_str = "LEVEL";
                else if (resolve == 2) resolve_str = "CLIMB";
                else if (resolve == 3) resolve_str = "DESCEND";
                emit text_message(QString("TCAS RA : %1 -> %2 %3").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, AircraftManager::get()->getAircraft(QString::number(other_id)) ? AircraftManager::get()->getAircraft(QString::number(other_id))->speechName() : QString::number(other_id), resolve_str));
            } catch(...) {}
        }
    });

    //listen for INS_REF to update origin waypoint of rotorcrafts
    PprzDispatcher::get()->bind("INS_REF", this, [=](QString sender, pprzlink::Message msg) {
        if(sender == ac_id) {

            int32_t lat0, lon0, alt0;
            msg.getField("lat0", lat0);
            msg.getField("lon0", lon0);
            msg.getField("alt0", alt0);

            auto orig = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getOrigin();
            orig->setLat(lat0/1e7);
            orig->setLon(lon0/1e7);
            orig->setAlt(alt0/1e3);
        }
    });

}

void AircraftStatus::updateMessage(pprzlink::Message msg) {
    QString id;
    msg.getField("ac_id", id);
    if(id == ac_id) {
        auto name = msg.getDefinition().getName();
        last_messages[name] = msg;

        if(name == "FLIGHT_PARAM") {
            try {
                float agl;
                msg.getField("agl", agl);
                uint32_t ftime = 0;
                if(last_messages.contains("AP_STATUS")) {
                    last_messages["AP_STATUS"].getField("flight_time", ftime);
                }
                if (!ground_prox && ftime > 10 && agl < 20.0f) {
                    emit text_message(QString("%1, Ground Proximity Warning").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id));
                    ground_prox = true;
                } else if (agl > 25.0f) {
                    ground_prox = false;
                }
            } catch(...) {}
            emit flight_param();
        }
        else if(name == "AP_STATUS") {
            try {
                QString ap_mode, kill_mode;
                msg.getField("ap_mode", ap_mode);
                msg.getField("kill_mode", kill_mode);
                if(!last_ap_mode.isEmpty() && ap_mode != last_ap_mode) {
                    emit text_message(QString("%1, %2").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, ap_mode));
                }
                last_ap_mode = ap_mode;
                if(kill_mode != "OFF") {
                    if(!in_kill_mode) {
                        emit text_message(QString("%1, mayday, kill mode").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id));
                    }
                    in_kill_mode = true;
                } else {
                    in_kill_mode = false;
                }
            } catch(...) {}
            emit ap_status();
        }
        else if(name == "NAV_STATUS") {
            try {
                uint8_t cur_block;
                msg.getField("cur_block", cur_block);
                auto ac = AircraftManager::get()->getAircraft(ac_id);
                if(ac && ac->getFlightPlan()) {
                    auto b = ac->getFlightPlan()->getBlock(cur_block);
                    if(b) {
                        QString block_name = b->getName();
                        if(!last_block_name.isEmpty() && last_block_name != block_name) {
                            emit text_message(QString("%1, %2").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, block_name));
                        }
                        last_block_name = block_name;
                    }
                }
            } catch(...) {}
            emit nav_status();
        }
        else if(name == "CIRCLE_STATUS") {
            emit circle_status();
        }
        else if(name == "SEGMENT_STATUS") {
            emit segment_status();
        }
        else if(name == "ENGINE_STATUS") {
            emit engine_status();
            watcher->watch_bat(msg);
        }
        else if(name == "WAYPOINT_MOVED") {
            emit waypoint_moved();
        }
        else if(name == "DL_VALUES") {
            emit dl_values();
        }
        else if(name == "TELEMETRY_STATUS") {
            QString link_id;
            msg.getField("link_id", link_id);

            QString display_id = link_id;
            bool is_num = false;
            int id_val = link_id.toInt(&is_num);
            if (is_num && id_val == -1) {
                display_id = "single";
            }

            float t_lost = 0.0f;
            try { msg.getField("time_since_last_msg", t_lost); } catch(...) {}

            if(!last_links_state.contains(link_id)) {
                emit text_message(QString("%1, link %2 detected").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, display_id));
                last_links_state[link_id] = false;
            }

            bool is_lost = (t_lost > 5.0f);
            if(is_lost && !last_links_state[link_id]) {
                emit text_message(QString("%1, link %2 lost").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, display_id));
            } else if(!is_lost && last_links_state[link_id]) {
                emit text_message(QString("%1, link %2 re-connected").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id, display_id));
            }
            last_links_state[link_id] = is_lost;

            bool any_up = false;
            for(auto lost : last_links_state) if(!lost) any_up = true;
            if(!any_up && !all_links_lost_reported) {
                emit text_message(QString("%1, all links lost").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id));
                all_links_lost_reported = true;
            } else if (any_up) {
                all_links_lost_reported = false;
            }

            if(telemetry_messages.contains("no_id") && link_id != "no_id") {
                telemetry_messages.remove("no_id");
            }

            telemetry_messages[link_id] = msg;
            emit telemetry_status();
            watcher->watch_links(msg);
        }
        else if(name == "FLY_BY_WIRE") {
            try {
                QString rc_mode;
                msg.getField("rc_mode", rc_mode);
                if(rc_mode == "FAILSAFE" && last_rc_mode != "FAILSAFE") {
                    emit text_message(QString("%1, mayday, AP Failure. Switch to manual.").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id));
                }
                last_rc_mode = rc_mode;
            } catch(...) {}
            emit fly_by_wire();
        }
        else if(name == "SVSINFO") {
            try {
                uint16_t pacc;
                msg.getField("pacc", pacc);
                int new_acc = 0;
                if (pacc < 1000) new_acc = 1;
                else if (pacc > 2000) new_acc = 3;
                else new_acc = 2;

                if (new_acc != last_gps_acc) {
                    if (new_acc == 1 && last_gps_acc != 0) {
                        emit text_message(QString("%1, GPS accuracy better than 10 meter").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id));
                    } else if (new_acc == 2 && last_gps_acc != 0) {
                        emit text_message(QString("%1, low GPS accuracy").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id));
                    } else if (new_acc == 3) {
                        emit text_message(QString("%1, Warning: GPS accuracy worse than 20 meter").arg(AircraftManager::get()->getAircraft(ac_id) ? AircraftManager::get()->getAircraft(ac_id)->speechName() : ac_id));
                    }
                    last_gps_acc = new_acc;
                }
            } catch(...) {}
            emit svsinfo();
        }
    }
}

std::optional<pprzlink::Message> AircraftStatus::getMessage(QString name) {
    if(name == "TELEMETRY_STATUS") {
        throw std::runtime_error("TELEMETRY_STATUS messages should be handled via the specialized methods!");
        return std::nullopt;
    }
    if(last_messages.contains(name)) {
        return last_messages[name];
    } else {
        return std::nullopt;
    }
}
