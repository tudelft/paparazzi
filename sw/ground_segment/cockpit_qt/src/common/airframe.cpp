#include "airframe.h"
#include <iostream>
#include <algorithm>

QString getAttribute(QDomElement ele, QString name) {
    auto result = ele.attribute(name, "");
    if(result == "") {
        name = name.toUpper();
        result = ele.attribute(name, "");
    }
    return result;
}

Airframe::Airframe(QDomDocument doc, QObject* parent):  QObject(parent), doc(doc)
{
    auto air_root = doc.firstChildElement( "airframe" );
    name = getAttribute(air_root, "name");

    auto firmware_node = air_root.firstChildElement( "firmware" );
    firmware = getAttribute(firmware_node, "name");
}

QList<Param> Airframe::getParams() {
    QList<Param> params;

    for(auto section=doc.firstChildElement().firstChildElement("section");
        !section.isNull();
        section=section.nextSiblingElement("section")) {
        for(auto define=section.firstChildElement("define");
            !define.isNull();
            define=define.nextSiblingElement("define")) {
            auto full_name = section.attribute("prefix", "") + define.attribute("name");
            auto value = define.attribute("value");
            auto unit = define.attribute("unit", "");
            params.append({full_name, unit, value});
        }
    }
    return params;
}

void Airframe::setParams(QMap<QString, QString> changed_params) {
    for(auto section=doc.firstChildElement().firstChildElement("section");
        !section.isNull();
        section=section.nextSiblingElement("section")) {
        for(auto define=section.firstChildElement("define");
            !define.isNull();
            define=define.nextSiblingElement("define")) {
            auto full_name = section.attribute("prefix", "") + define.attribute("name");

            for(auto it = changed_params.begin(); it!=changed_params.end(); ++it) {
                if(full_name == it.key()) {
                    define.setAttribute("value", it.value());
                }
            }
        }
    }
}

void Airframe::saveSettings(QString filename) {
    QFile file(filename);
    if( !file.open( QIODevice::WriteOnly | QIODevice::Text ) )
    {
        qDebug( "Failed to open file for writing." );
        return;
    }
    QTextStream stream( &file );
    stream << doc.toString();
    file.close();
}

QString Airframe::getIconName() {
    auto d = getDefine("AC_ICON", "GCS");
    if(d.has_value()) {
        return d.value().value;
    }

    return firmware;
}

float Airframe::getAltShiftPlus() {
    auto d = getDefine("ALT_SHIFT_PLUS", "GCS");
    if(d.has_value()) {
        return d.value().value.toDouble();
    }

    if(firmware == "fixedwing") {
        return ALT_PLUS_FW;
    } else if (firmware == "rotorcraft") {
        return ALT_PLUS_RC;
    }

    return 0;
}

float Airframe::getAltShiftPlusPlus() {
    auto d = getDefine("ALT_SHIFT_PLUS_PLUS", "GCS");
    if(d.has_value()) {
        return d.value().value.toDouble();
    }

    if(firmware == "fixedwing") {
        return ALT_PLUS_PLUS_FW;
    } else if (firmware == "rotorcraft") {
        return ALT_PLUS_PLUS_RC;
    }

    return 0;
}

float Airframe::getAltShiftMinus() {
    auto d = getDefine("ALT_SHIFT_MINUS", "GCS");
    if(d.has_value()) {
        return d.value().value.toDouble();
    }

    if(firmware == "fixedwing") {
        return ALT_MINUS_FW;
    } else if (firmware == "rotorcraft") {
        return ALT_MINUS_RC;
    }

    return 0;
}

/**
 * @brief Airframe::getDefine
 * @param def_name: full name of the define, including prefix
 * @param section: optional section name
 * @return
 */
std::optional<Param> Airframe::getDefine(QString def_name, QString section_name) {
    for(auto section=doc.firstChildElement().firstChildElement("section");
        !section.isNull();
        section=section.nextSiblingElement("section")) {
        if(section_name != "" && section_name != section.attribute("name", "")) {
            continue;
        }
        for(auto define=section.firstChildElement("define");
            !define.isNull();
            define=define.nextSiblingElement("define")) {
            auto full_name = section.attribute("prefix", "") + define.attribute("name");
            if(full_name == def_name) {
                auto value = define.attribute("value");
                auto unit = define.attribute("unit", "");
                struct Param ret =  {full_name, unit, value};
                return ret;
            }
        }
    }
    return std::nullopt;
}

QList<ChecklistItem*> Airframe::getChecklistItems() {
    QList<ChecklistItem*> items;

    for(auto section=doc.firstChildElement().firstChildElement("checklist");
        !section.isNull();
        section=section.nextSiblingElement("checklist")) {
        for(auto item=section.firstChildElement("item");
            !item.isNull();
            item=item.nextSiblingElement("item")) {
            auto name = item.attribute("name");
            auto description = item.firstChild().nodeValue();
            auto type = item.attribute("type", "checkbox");
            auto checklist_item = new ChecklistItem({name, description, type, ""});
            items.append(checklist_item);
        }
    }
    return items;
}

QString Airframe::getSpeechName() {
    for (auto section=doc.firstChildElement().firstChildElement("section");
        !section.isNull();
        section=section.nextSiblingElement("section")) {
        for (auto define=section.firstChildElement("define");
            !define.isNull();
            define=define.nextSiblingElement("define")) {
            if (define.attribute("name") == "SPEECH_NAME") {
                return define.attribute("value");
            }
        }
    }
    return "";
}
