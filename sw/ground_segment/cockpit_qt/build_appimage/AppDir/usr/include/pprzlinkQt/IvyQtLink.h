#ifndef IVYQTLINK_H
#define IVYQTLINK_H

#include <QObject>
#include <pprzlinkQt/MessageDictionary.h>
#include <pprzlinkQt/Message.h>

class IvyQt;
class Peer;

namespace pprzlink {

    using messageCallback_t = std::function<void(QString, Message)>;

    class IvyQtLink : public QObject
    {
        Q_OBJECT
    public:

        explicit IvyQtLink(MessageDictionary const & dict , QString appName, QObject *parent = nullptr);
        ~IvyQtLink();

        void start(QString domain = "127.255.255.255:2010");
        void stop();
        long BindMessage(MessageDefinition const & def, QObject* context, messageCallback_t cb);
        void UnbindMessage(long bindId);
        void sendMessage(const Message& msg);
        long sendRequest(const Message& msg, messageCallback_t cb);
	    IvyQt* ivy() {return bus;}

    signals:
        void serverConnected();

    private:
      const MessageDictionary &dictionary;
      QString domain;
      QString appName;
      IvyQt* bus;
      unsigned int requestNb;
      QMap<QString, long> requestBindId;

      void getMessageData(const Message& msg, QString &ac_id, QString &name, QString &fields);

      QString regexpForMessageDefinition(MessageDefinition const & def);
      QString messageRegexp(MessageDefinition const & def);
      void messageCallback(QStringList params, const messageCallback_t &cb);

      std::map<int, std::function<void(Peer*, QStringList)>> messagesCallbackMap;
      //std::map<long,AircraftCallback*> aircraftCallbackMap;
      //std::map<long,RequestCallback*> requestCallbackMap;

    };




}

#endif // IVYQTLINK_H
