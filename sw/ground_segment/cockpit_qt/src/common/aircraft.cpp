#include "aircraft.h"
#include "pprz_dispatcher.h"
#include "dispatcher_ui.h"
#include <QDebug>
#include "AircraftManager.h"
#include "gcs_utils.h"

Aircraft::Aircraft(ConfigData* config, QObject* parent): QObject(parent),
    ac_id(config->getId()), color(config->getColor()), _name(config->getName()), config(config), position(Point2DLatLon(0,0)), real(true)
{
    config->setParent(this);
    QSettings app_settings(appConfig()->value("SETTINGS_PATH").toString(), QSettings::IniFormat);
    flight_plan = new FlightPlan(ac_id, config->getFlightPlan(), this);
    setting_menu = new SettingMenu(config->getSettings(), this);
    airframe = new Airframe(config->getAirframe(), this);
    if(airframe->getIconName() != "") {
        icon = user_or_app_path("pictures/aircraft_icons/" + QString(airframe->getIconName()) + ".svg");
    }
    checklist = airframe->getChecklistItems();
    status = new AircraftStatus(ac_id, this);
    real = config->isReal();
}
void Aircraft::setSetting(Setting* setting, QString value) {
    int idx = setting->getValues().indexOf(value);
    if(idx != -1) {
        setSetting(setting, static_cast<float>(idx));
    }
}

void Aircraft::setSetting(Setting* setting, float value) {
    auto coef = setting->getAltUnitCoef();
    if(value < setting->getMin() / coef || value > setting->getMax() / coef) {
        qDebug() << "Warning: send setting value " << value << ", out of bounds for setting " << setting->getName();
    }

    pprzlink::Message dlSetting(PprzDispatcher::get()->getDict()->getDefinition("DL_SETTING"));
    dlSetting.addField("ac_id", ac_id);
    dlSetting.addField("index", setting->getNo());
    dlSetting.addField("value", value);
    PprzDispatcher::get()->sendMessage(dlSetting);
}

void Aircraft::setSetting(uint8_t setting_no, float value) {
    for(auto set: setting_menu->getAllSettings()) {
        if(set->getNo() == setting_no) {
            setSetting(set, value);
            break;
        }
    }
}

bool Aircraft::checklistFinished() {
    for(auto item: checklist) {
        if(item->type == "checkbox" && item->value != "true") {
            return false;
        }
        else if(item->type == "text" && item->value == "") {
            return false;
        }
    }
    return true;
}

QString Aircraft::speechName() {
    if (_speech_name == "") {
        _speech_name = airframe->getSpeechName();
        if (_speech_name == "") {
            _speech_name = _name;
        } else {
            _speech_name.replace("$AC_ID", ac_id);
            _speech_name.replace("\"", ""); // strip double quotes just in case
        }
    }
    return _speech_name;
}
