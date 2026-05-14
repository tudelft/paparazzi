#include "globalconfig.h"


GlobalConfig* GlobalConfig::singleton = nullptr;

GlobalConfig::GlobalConfig(QObject *parent) : QObject(parent)
{

}

void GlobalConfig::setValue(const QString &key, const QVariant &value) {
    map[key] = value;
}

QVariant GlobalConfig::value(const QString &key, const QVariant &default_value) {
    if(map.contains(key)) {
        return map[key];
    } else {
        return default_value;
    }
}
