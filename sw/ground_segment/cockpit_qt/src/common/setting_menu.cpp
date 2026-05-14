#include "setting_menu.h"
#include "iostream"
#include <clocale>
#include <assert.h>
#include <algorithm>
#include <map>

SettingMenu::SettingMenu(QDomDocument doc, QObject* parent): QObject(parent) {
    auto st_root = doc.firstChildElement( "settings" );
    if(st_root.isNull()) {
        st_root = doc.firstChildElement("generated_settings");
    }
    auto sets = st_root.firstChildElement("dl_settings");
    uint8_t setting_no = 0;
    init(sets, setting_no);
}

SettingMenu::SettingMenu(QDomElement setel, uint8_t& setting_no, QObject* parent): QObject(parent) {
    init(setel, setting_no);
}

void SettingMenu::init(QDomElement setel, uint8_t& setting_no) {
    name = setel.attribute("name", "");

    for(auto sets = setel.firstChildElement(); !sets.isNull(); sets = sets.nextSiblingElement()) {

        if(sets.tagName() == "dl_settings") {
            auto menu = new SettingMenu(sets, setting_no, this);
            setting_menus.push_back(menu);
        } else if (sets.tagName() ==  "dl_setting") {
            auto setting = new Setting(sets, setting_no, this);
            settings.push_back(setting);
            ++setting_no;
        } else {
            auto msg = "Tag " + sets.tagName() + " unknown for dl_settings!";
            runtime_error(msg.toStdString());
        }

    }

}

QList<Setting*> SettingMenu::getAllSettings() {
    QList<Setting*> v;
    v.append(settings);

    for(auto menu: setting_menus) {
        auto sets = menu->getAllSettings();
        v.append(sets);
    }

    return v;
}

QList<shared_ptr<SettingMenu::ButtonGroup>> SettingMenu::getButtonGroups() {

    QList<shared_ptr<Setting::StripButton>> buttons;
    for(auto setting: getAllSettings()) {
        for(auto &sb: setting->getStripButtons()) {
            buttons.push_back(sb);
        }
    }

    std::map<QString, shared_ptr<ButtonGroup>> groups_map;

    for(auto &b: buttons) {
        if(groups_map.find(b->group) == groups_map.end()) {
            groups_map[b->group] = make_shared<ButtonGroup>();
            groups_map[b->group]->group_name = b->group;
        }
        groups_map[b->group]->buttons.push_back(b);
    }

    QList<shared_ptr<ButtonGroup>> groups;

    for( auto it = groups_map.begin(); it != groups_map.end(); ++it ) {
        groups.push_back( it->second );
    }

    return groups;
}

