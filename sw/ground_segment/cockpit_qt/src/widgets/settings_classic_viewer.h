
#ifndef SETTINGS_CLASSIC_VIEWER_H
#define SETTINGS_CLASSIC_VIEWER_H

#include <QWidget>
#include <QString>
#include <QDomDocument>
#include <QBoxLayout>
#include <map>
#include <functional>

class Setting;

class SettingsClassicViewer : public QWidget
{
    Q_OBJECT
public:
    explicit SettingsClassicViewer(QString ac_id, QWidget *parent = nullptr);

private:
    void buildSettings(QDomNode node, QWidget *parentWidget, QBoxLayout *parentLayout, uint8_t& setting_no);
    QWidget* makeSettingWidget(QDomElement e, QWidget *parent, uint8_t setting_no);
    void updateSettings(QString id, Setting* setting, float value);

    QString ac_id;

    std::map<uint8_t, std::function<void(float)>> value_setters;
    std::map<uint8_t, std::function<void(float)>> label_setters;
    std::map<uint8_t, bool> initialized;
};

#endif // SETTINGS_CLASSIC_VIEWER_H
