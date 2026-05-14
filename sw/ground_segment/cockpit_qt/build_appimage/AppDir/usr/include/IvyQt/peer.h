#ifndef PEER_H
#define PEER_H

#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include <QRegularExpression>
#include <QTimer>
#include <QElapsedTimer>

class IvyQt;

enum PeerStatus {
    INIT,
    SYNCHRONIZED,
    INI_SUBS_OK,
    APP_QUIT,
};

class Peer : public QObject
{
    Q_OBJECT
    friend IvyQt;
public:
    Peer(QTcpSocket* tcp_socket, QObject* parent = nullptr);
    ~Peer();

    QString peer_id(QHostAddress addr, quint16 port);

    QString name() {return appName;}

    void sendDirectMessage(int identifier, QString params);

    /**
     * @brief sendQuit ask peer to quit
     */
    void sendQuit();

    /**
     * @brief sendPing
     * @return ping id. subscribe to pingCompleted to get final result.
     */
    int sendPing();

    QStringList getBindings();

    QHostAddress getHost();

    quint16 getPort();

signals:
    void message(QString id, QStringList params);
    void directMessage(int identifier, QString params);
    void quitRequest();
    void peerDied(Peer*);
    // app is ready for this peer : handshake info has been sent and received
    void ready(Peer*);
    void pingCompleted(int ping_id, qint64 msec);

private:    /// to be accessed by IvyQt

    /**
     * @brief sendMessage send the message to the peer
     * @param message
     */
    void sendMessage(QString message);

    /**
     * @brief sendBye tells peers that this app is about to quit
     */
    void sendBye();

    /**
     * @brief bind to a regex
     * @param bindId
     * @param regex
     */
    void bind(int bindId, QString regex);

    /**
     * @brief unBind from a regex
     * @param bindId
     */
    void unBind(int bindId);

    /**
     * @brief end_initial_bindings
     */
    void end_initial_bindings();

    /**
     * @brief sendId
     * @param port of the TCP socket
     * @param myName name of this apliation
     */
    void sendId(quint16 port, QString myName);

    void setInfoSent() {
        info_sent = true;
        if(info_rcv) {
            emit ready(this);
        }
    }

    /**
     * @brief stop disconnect from peer
     */
    void stop();

    void setFlushTimeout(int msec) {
        flushTimer->setInterval(msec);
    }

private:    /// really private stuff
    void process();
    void parseMessage(QByteArray message);
    void subscribe(QString id, QString regex);
    void unsubscribe(QString id);
    void handle_message(QString id, QString params);

    void send_data(int type, QString ident, QString params);

    QString peerId;
    QString appName;
    QTcpSocket* socket;
    quint16 appPort;
    QByteArray rcv_array;
    PeerStatus state;

    QMap<QString /*numerical identifier*/, QRegularExpression /*regex*/> subscriptions;

    // does handshake info has been send to peer ?
    bool info_sent;

    // does handshake info has been received from peer ?
    bool info_rcv;

    QTimer* flushTimer;
    QMap<int, QElapsedTimer> ping_ids;
    int last_ping_id;
};

#endif // PEER_H
