#ifndef INTRUDERS_H
#define INTRUDERS_H

#include <QObject>
#include <QMap>
#include <QPointer>
#include <QString>
#include <QTimer>
#include <pprzlinkQt/Message.h>

class IntruderItem;

class Intruder {
public:
    QPointer<IntruderItem> track;
    double last_update;
};

class Intruders : public QObject {
    Q_OBJECT
public:
    static Intruders* get();
    void init();

private slots:
    void onIntruderMsg(QString sender, pprzlink::Message msg);
    void removeOldIntruders();

private:
    Intruders(QObject *parent = nullptr);
    static Intruders* m_instance;
    QMap<QString, Intruder> _intruders;
    QTimer* _timer;
};

#endif // INTRUDERS_H
