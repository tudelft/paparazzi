#ifndef AIRFRAME_H
#define AIRFRAME_H

#include <QString>
#include <QtXml>
#include <optional>

struct Param {
    QString name;
    QString unit;
    QString value;
};

struct ChecklistItem {
    QString name;
    QString description;
    QString type;
    QString value;
};

class Airframe: public QObject
{
    Q_OBJECT
public:

    Airframe(QDomDocument doc, QObject* parent=nullptr);

    QString getFirmware() {return firmware;}
    QString getIconName();
    QString getSpeechName();
    float getAltShiftPlus();
    float getAltShiftPlusPlus();
    float getAltShiftMinus();
    std::optional<struct Param> getDefine(QString name, QString section="");

    void saveSettings(QString filename);
    void setParams(QMap<QString, QString> changed_params);
    QList<Param> getParams();
    QList<ChecklistItem*> getChecklistItems();

private:
    QString name;
    QString firmware;

    QDomDocument doc;

    static constexpr float ALT_PLUS_FW = 10;
    static constexpr float ALT_MINUS_FW = -10;
    static constexpr float ALT_PLUS_PLUS_FW = 30;

    static constexpr float ALT_PLUS_RC = 1;
    static constexpr float ALT_MINUS_RC = -1;
    static constexpr float ALT_PLUS_PLUS_RC = 3;
};

#endif // AIRFRAME_H
