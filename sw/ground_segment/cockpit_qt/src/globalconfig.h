#ifndef GLOBALCONFIG_H
#define GLOBALCONFIG_H

#include <QObject>
#include <QMap>
#include <QVariant>

class GlobalConfig : public QObject
{
    Q_OBJECT
public:
    static GlobalConfig* get() {
        if(singleton == nullptr) {
            singleton = new GlobalConfig();
        }
        return singleton;
    }

    void setValue(const QString &key, const QVariant &value);
    QVariant value(const QString &key, const QVariant &default_value=QVariant());
    bool contains(const QString &key) {
        return map.contains(key);
    }

signals:

private:
    static GlobalConfig* singleton;
    explicit GlobalConfig(QObject *parent = nullptr);

    QMap<QString, QVariant> map;



};

#endif // GLOBALCONFIG_H
