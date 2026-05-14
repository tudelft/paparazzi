#include "setting.h"
#include "iostream"
// #include <sstream> (removed)
#include <assert.h>

// #include <algorithm> (removed)
// #include <iterator> (removed)

#include "units.h"

Setting::Setting(QDomElement setel, uint8_t& setting_no, QObject* parent) : QObject(parent),
    setting_no(setting_no)
{
    //var, min, max, step
    var = setel.attribute("var");
    min = setel.attribute("min").toDouble();
    max = setel.attribute("max").toDouble();
    step = setel.attribute("step").toDouble();

    last_set_values[0] = min;
    last_set_values[1] = min;

    //shortname
    shortname= setel.attribute("shortname", var);

    //values
    if(setel.hasAttribute("values")) {
        for(auto &val: setel.attribute("values").split("|")) {
            values.push_back(val);
        }
    }

    unit = setel.attribute("unit", "");
    alt_unit = setel.attribute("alt_unit", "");

    if(setel.hasAttribute("alt_unit_coef")) {
        alt_unit_coef = setel.attribute("alt_unit_coef").toDouble();
    }

    param = setel.attribute("param", "");
    m_show_auto = (setel.attribute("auto", "false") == "true");

    //module, handler, type, persistent, param
    //TODO


    for(auto sets = setel.firstChildElement(); !sets.isNull(); sets = sets.nextSiblingElement()) {
        if(sets.tagName() == "key_press") {
            shared_ptr<KeyPress> kp = make_shared<KeyPress>();
            auto key = sets.attribute("key");
            auto value = sets.attribute("value");
            kp->key = key;
            kp->value = value.toFloat();
            key_presses.push_back(kp);
        } else if (sets.tagName() == "strip_button") {
            shared_ptr<StripButton> sb = make_shared<StripButton>();
            auto name = sets.attribute("name");
            auto value = sets.attribute("value");

            sb->name = name;
            sb->value = value.toFloat();

            if(sets.hasAttribute("group")) {
                sb->group = sets.attribute("group");
            }

            if(sets.hasAttribute("icon")) {
                sb->icon = sets.attribute("icon");
            }

            sb->setting_no = setting_no;

            strip_buttons.push_back(sb);
        } else {
            auto msg = "Tag " + sets.tagName() + " unknown for dl_setting!";
            runtime_error(msg.toStdString());
        }

    }

}

float Setting::getAltUnitCoef(QString altUnit) {
    auto coef_alt = [=](QString alt) -> double {
        auto coef = Units::get()->getCoef(unit, alt);
        if(coef.has_value()) {
            return coef.value();
        } else {
            return 1.0;
        }
    };

    if(altUnit != "") {
        return static_cast<float>(coef_alt(altUnit));
    }
    else if(alt_unit_coef.has_value()) {
        return alt_unit_coef.value();
    }
    else{
        return static_cast<float>(coef_alt(alt_unit));
    }
}


ostream& operator<<(ostream& os, const Setting& set) {
    os << "{";

    if(set.shortname != "") {
        os << set.shortname.toStdString();
    } else {
        os << set.var.toStdString();
    }

    os << " (" << to_string(set.setting_no) << ")";

    if(set.values.size() > 0) {
        os << " [";
        for (auto p = set.values.begin(); p != set.values.end(); ++p) {
            os << p->toStdString();
            if (p != set.values.end() - 1) {
                os << ", ";
            }

        }
        os << "]";
    }

    if(set.key_presses.size() > 0) {
        os << ", [";
        for (auto p = set.key_presses.begin(); p != set.key_presses.end(); ++p) {
            os << **p;
            if (p != set.key_presses.end() - 1) {
                os << ", ";
            }

        }
        os << "]";
    }

    if(set.strip_buttons.size() > 0) {
        os << ", [";
        for (auto p = set.strip_buttons.begin(); p != set.strip_buttons.end(); ++p) {
            os << **p;
            if (p != set.strip_buttons.end() - 1) {
                os << ", ";
            }

        }
        os << "]";
    }

    os << "}";

    return os;
}


ostream& operator<<(ostream& os, const Setting::KeyPress& kp) {
    os << kp.key.toStdString() << ": " << kp.value;
    return os;
}

ostream& operator<<(ostream& os, const Setting::StripButton& sb) {
    os << sb.group.toStdString() << "." << sb.name.toStdString() << ": " << sb.value;
    if(sb.icon != "") {
        os << " " << sb.icon.toStdString();
    }
    return os;
}
