#ifndef PARTICULES_H
#define PARTICULES_H

#include <QObject>
#include <QMap>
#include <QString>
#include <pprzlinkQt/Message.h>
#include "particule_item.h"

// Translating particules.ml to C++ 
class Particules : public QObject {
    Q_OBJECT
public:
    static Particules* get();
    void init();

private slots:
    void onPlumesMsg(pprzlink::Message msg);

private:
    Particules(QObject *parent = nullptr);
    static Particules* m_instance;
    QMap<int, ParticuleItem*> _particules;
};

#endif // PARTICULES_H
