#include <IvyQt/peer.h>

Peer::Peer(QTcpSocket* tcp_socket, QObject* parent) :
    QObject(parent),
    peerId(""), appName(""), socket(tcp_socket), state(INIT),
    info_sent(false), info_rcv(false)
{
    flushTimer = new QTimer(this);
    flushTimer->setSingleShot(true);
    connect(flushTimer, &QTimer::timeout, this, [=]() {
        if(socket) {
            socket->flush();
        }
    });

    connect(socket, &QTcpSocket::readyRead, this, [=]() {
        rcv_array += tcp_socket->readAll();
        process();
    });

    connect(socket, &QTcpSocket::disconnected, this, [=]() {
        state = APP_QUIT;
        emit peerDied(this);
    });


}

Peer::~Peer() {
}

QStringList Peer::getBindings() {
    QStringList list;
    for(auto &b: subscriptions) {
        list.append(b.pattern());
    }
    return list;
}

QHostAddress Peer::getHost() {
    if(socket) {
        return socket->peerAddress();
    } else {
        return QHostAddress();
    }
}

quint16 Peer::getPort() {
    if(socket) {
        return socket->peerPort();
    } else {
        return 0;
    }
}

void Peer::process() {
    if(rcv_array.contains('\n')) {
        int bytes = rcv_array.indexOf('\n') + 1;       // Find the end of message
        QByteArray message = rcv_array.left(bytes);    // Cut the message
        rcv_array = rcv_array.mid(bytes);     // Keep the data read too early

        parseMessage(message);                        // process message

        process();                                    // Call itself again to process remaining messages
    }
}

void Peer::parseMessage(QByteArray message) {
    assert(message.at(message.length()-1) == '\n');

    int space_pos = message.indexOf(' ');
    int stx_pos = message.indexOf(0x02);       // Find 0x02

    auto type = message.left(space_pos);
    auto ident = message.mid(space_pos + 1, stx_pos-space_pos-1);
    auto params=message.mid(stx_pos+1, message.length() - stx_pos - 2);

    if(type == "0") {
        // Bye
        stop();
        state = APP_QUIT;
    } else if(type == "1") {
        // Subscription
        if(state == SYNCHRONIZED || state == INI_SUBS_OK) {
            subscribe(ident, params);
        } else {
            qDebug() << "Received subscription while in state " << state;
        }
    } else if(type == "2") {
        // Text message
        if (state == INI_SUBS_OK) {
            handle_message(ident, params);
        } else {
            qDebug() << "Received text message while in state " << state;
        }
    } else if(type == "3") {
        // Error
        qDebug() << "App " << appName << " encoutered error " << params;
    } else if(type == "4") {
        // Subscription deletion
        if(state == INI_SUBS_OK) {
            unsubscribe(ident);
        } else {
            qDebug() << "Received subscription deletion while in state " << state;
        }
    } else if(type == "5") {
        // End of initial subscriptions.
        if (state == SYNCHRONIZED) {
            state = INI_SUBS_OK;
            info_rcv = true;
            if(info_sent) {
                emit ready(this);
            }
        } else {
            qDebug() << "Received endOfSub while in state " << state;
        }
    } else if(type == "6") {
        if(state == INIT) {
            // Peer ID
            appPort = ident.toUInt();
            peerId = peer_id(socket->peerAddress(), appPort);
            appName = params;
            state = SYNCHRONIZED;
        } else {
            qDebug() << "Received peerID while in state " << state;
        }
    } else if(type == "7") {
        // Direct message
        if(state == INI_SUBS_OK) {
            emit directMessage(ident.toInt(), params);
        } else {
            qDebug() << "Received Direct message while in state " << state;
        }
    } else if(type == "8") {
        // Quit message
        emit quitRequest();
    }
    else if(type == "9") {
        // Ping message
        // answer with pong, and the same ping id.
        send_data(10, ident, "");
    }
    else if(type == "10") {
        // Pong message
        int pong_id = ident.toInt();
        if(ping_ids.contains(pong_id)) {
            qint64 ping_time = ping_ids[pong_id].elapsed();
            ping_ids.remove(pong_id);
            emit pingCompleted(pong_id, ping_time);
        } else {
            qDebug() << "received pong, but no ping was sent!";
        }
    }
    else {
        qDebug() << "message " << type << " not handled. Is it in the doc ???";
    }
}

void Peer::subscribe(QString id, QString regex) {
    if(subscriptions.contains(id)) {
        // if id is already used, replace the old regex by the new one
        unsubscribe(id);
    }
    subscriptions[id] = QRegularExpression(regex);
}

void Peer::unsubscribe(QString id) {
    if(subscriptions.contains(id)) {
        subscriptions.remove(id);
    } else {
        qCritical() << QString("subscribe id %1 unknown!").arg(id);
    }

}

void Peer::handle_message(QString id, QString params) {
    auto parameters = params.split(QChar(0x03));
    parameters.removeLast();
    emit message(id, parameters);
}

void Peer::stop() {
    if(socket) {
        socket->disconnectFromHost();
    }
}

void Peer::sendMessage(QString message) {
    for(auto i=subscriptions.begin(); i!=subscriptions.end(); ++i) {
        auto match = i.value().match(message);
        if(match.hasMatch()) {
            auto caps = match.capturedTexts();
            // remove the implicit capturing group number 0, capturing the substring matched by the entire pattern
            caps.removeAt(0);
            QString params;
            for(auto& cap: caps) {
                params += cap + QChar(0x03);
            }
            QString ident = i.key();
            send_data(2, ident, params);
        }
    }
}


void Peer::sendBye() {
    send_data(0, "0", "");
}

void Peer::bind(int bindId, QString regex) {
    send_data(1, QString::number(bindId), regex);
}


void Peer::unBind(int bindId) {
    send_data(4, QString::number(bindId), "");
}


void Peer::end_initial_bindings() {
    send_data(5, "0", "");
}

void Peer::sendId(quint16 port, QString myName) {
    send_data(6, QString::number(port), myName);
}

void Peer::sendDirectMessage(int identifier, QString params) {
    send_data(7, QString::number(identifier), params);
}

void Peer::sendQuit() {
    send_data(8, "0", "");
}

int Peer::sendPing() {
    last_ping_id += 1;
    int ping_id = last_ping_id;
    ping_ids[ping_id] = QElapsedTimer();
    ping_ids[ping_id].start();
    send_data(9, QString::number(ping_id), "");
    return ping_id;
}

void Peer::send_data(int type, QString ident, QString params) {
    QByteArray data = (QString::number(type) + " " + ident + QString(QChar(0x02)) + params + "\n").toUtf8();
    socket->write(data);
    flushTimer->start();
}

QString Peer::peer_id(QHostAddress addr, quint16 port) {
    return addr.toString() + ":" + QString::number(port);
}
