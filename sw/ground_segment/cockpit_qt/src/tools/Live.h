#ifndef LIVE_H
#define LIVE_H

#include <QObject>
#include <QMap>
#include <QString>
#include <pprzlinkQt/Message.h>

// Translating live.ml to C++ (Live module handles live data dispatching from Ivy)
class Live : public QObject {
    Q_OBJECT
public:
    static Live* get();
    void init();

signals:
    void messageReceived(QString ac_id, QString msg_name);

private slots:
    void onPprzMessage();

private:
    Live(QObject *parent = nullptr);
    static Live* m_instance;
};

#endif // LIVE_H
