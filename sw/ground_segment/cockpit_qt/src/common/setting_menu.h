#ifndef SETTINGS_H
#define SETTINGS_H

#include <memory>
#include "setting.h"
#include <QtXml>
#include <QObject>

using namespace std;

class SettingMenu: public QObject
{
    Q_OBJECT
public:

    struct ButtonGroup {
        QString group_name;
        QList<shared_ptr<Setting::StripButton>> buttons;
    };

    SettingMenu(QDomDocument doc, QObject* parent);
    SettingMenu(QDomElement setel, uint8_t& setting_no, QObject* parent);

    QList<SettingMenu*> getSettingMenus() {return setting_menus;}
    QList<Setting*> getSettings() {return settings;}
    QList<Setting*> getAllSettings();
    QList<shared_ptr<ButtonGroup>> getButtonGroups();

    QString getName() {return name;}

private:
    void init(QDomElement setel, uint8_t& setting_no);

    QString name;
    QList<SettingMenu*> setting_menus;
    QList<Setting*> settings;
};

#endif // SETTINGS_H
