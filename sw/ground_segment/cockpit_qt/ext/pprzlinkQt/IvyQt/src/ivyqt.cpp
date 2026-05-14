#include <utility>
#include <IvyQt/ivyqt.h>
#include <QUdpSocket>
#include <QRandomGenerator>


Q_LOGGING_CATEGORY(lcIvy,"IVY")


IvyQt::IvyQt(QString name, QString msgReady, QObject *parent) :
    QObject(parent),
    name(name), msgReady(msgReady),
    udp_socket(nullptr), server(nullptr),
    nextRegexId(0), nextBindId(0), flush_timeout(0),
    running(false), stopRequested(false), logLevel(IVY_LOG_NONE)
{

}

void IvyQt::start(QString domain, int udp_port) {
    if(running) {
        return;
    }
    stopRequested = false;
    // start TCP server
    server = new QTcpServer(this);
    bool res = server->listen();
    assert(res == true);
    connect(server, &QTcpServer::newConnection, this, &IvyQt::tcphandle);

    // start UDP socket
    udp_socket = new QUdpSocket(this);
    udp_socket->bind(udp_port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    connect(udp_socket, &QUdpSocket::readyRead, this, &IvyQt::udphandle);

    // generate a uniq ID
    watcherId = QString::number(QRandomGenerator::system()->generate64());

    // forge broadcast data
    auto data = QString("3 %1 %2 %3\n").arg(QString::number(server->serverPort()), watcherId, name).toUtf8();
    udp_socket->writeDatagram(data, QHostAddress(domain), udp_port);

    if(logLevel >= IVY_LOG_LOW) {
        qCInfo(lcIvy()) << QString("Start on %1:%2 using TCP port %3.").arg(domain).arg(udp_port).arg(server->serverPort());
    }

    running = true;
}

int IvyQt::bindMessage(QString regex, QObject* context, std::function<void(Peer*, QStringList)> callback) {

    auto existing = std::find_if(regexes.begin(), regexes.end(),
        [=](QString reg){
            return reg == regex;
        });

    int regexId;

    if(existing == regexes.end()) {
        // this regex is not used yet
        //qDebug() << "new regex:" << regex;
        regexId = nextRegexId;
        nextRegexId += 1;
        regexes[regexId] = regex;

        // publish regex to all peers
        for(auto peer: std::as_const(peers)) {
            peer->bind(regexId, regex);
        }

    } else {
        //qDebug() << "regex exist:" << regex << existing.key();
        regexId = existing.key();
    }


    Binding b = {
        regexId,
        callback,
        context,
    };
    int bindId = nextBindId;
    bindings[bindId] = b;
    nextBindId += 1;


    if(context != nullptr) {
        connect(context, &QObject::destroyed, this, [=](QObject* obj) {
            unBindMessage(bindId);
            if(logLevel >= IVY_LOG_MEDIUM) {
                qCInfo(lcIvy()) << "Request unbind" << bindId << "with destroyed context" << context;
            }
        });
    }

    if(logLevel >= IVY_LOG_MEDIUM) {
        qCInfo(lcIvy()) << "Bind" << regex << "with id" << bindId << "and context" << context;
    }

    return bindId;
}

int IvyQt::bindMessage(QString regex, std::function<void(Peer*, QStringList)> callback) {
    return bindMessage(regex, nullptr, callback);
}

void IvyQt::unBindMessage(int bindId) {
    // Schedule the unbind for the next event loop
    // If unBindMessagePrivate is called in a callback,
    // "bindings" is modified while iterating over it, causing the program to crash
    QTimer::singleShot(0, this, [=]{unBindMessagePrivate(bindId);});
    bindings_removing.append(bindId);
}

void IvyQt::unBindMessagePrivate(int bindId) {
    bindings.remove(bindId);
    bindings_removing.removeAll(bindId);
    if(logLevel >= IVY_LOG_MEDIUM) {
        qCInfo(lcIvy()) << "Unbind" << bindId;
    }

    // clear unsued regexes
    QMutableMapIterator<int, QString> reg(regexes);
    while (reg.hasNext()) {
        reg.next();

        bool regex_used = std::any_of(bindings.begin(), bindings.end(),
            [=](Binding bd){
                return bd.regexId == reg.key();
            });
        if(!regex_used) {
            for(auto peer: std::as_const(peers)) {
                // no more callbacks to this regex. Advise all peers to remove this regex.
                peer->unBind(reg.key());
            }
            //remove regex
            reg.remove();
        }
    }
}

void IvyQt::stop() {
    stopRequested = true;
    if(!running) {
        return;
    }
    if(logLevel >= IVY_LOG_LOW) {
        qCInfo(lcIvy()) << "Stopping...";
    }
    if(peers.length() == 0) {
        completeStop();
    } else {
        //stop will completed after all peer has been deleted
        for(auto peer: std::as_const(peers)) {
            peer->sendBye();
            peer->stop();
        }
    }

}

void IvyQt::send(QString message) {
    for(auto peer: std::as_const(peers)) {
        peer->sendMessage(message);
    }
}

QList<Peer*> IvyQt::getPeers() {
    return peers;
}

Peer* IvyQt::getPeer(QString name) {
    for(auto peer: std::as_const(peers)) {
        if(peer->name() == name) {
            return peer;
        }
    }
    return nullptr;
}

void IvyQt::setFlushTimeout(int msec) {
    for(auto peer: std::as_const(peers)) {
        peer->setFlushTimeout(msec);
    }
}

void IvyQt::tcphandle() {
    // accept action
    auto tcp_socket = server->nextPendingConnection();
    newPeer(tcp_socket);
}

void IvyQt::udphandle() {
    while (udp_socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udp_socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        udp_socket->readDatagram(datagram.data(), datagram.size(),
                                &sender, &senderPort);
        QString msg = QString(datagram.data()).trimmed();
        // "<protocol number> <port> <???> <app name>"
        auto params = msg.split(" ");
        if(params[0] != "3") {
            qDebug() << "Protocol version not supported!";
            return;
        }
        quint16 tcp_port = params[1].toUInt();
        if(tcp_port == server->serverPort() && watcherId == params[2]) {
            //that's my own message, ignoring it.
            return;
        }

        auto socket = new QTcpSocket(this);

        socket->connectToHost(sender, tcp_port);
        connect(socket, &QTcpSocket::connected, this, [=]() {
            newPeer(socket);
        });
    }
}

void IvyQt::newPeer(QTcpSocket* socket) {
    auto peer = new Peer(socket, this);

    connect(peer, &Peer::ready,         this, &IvyQt::newPeerReady);
    connect(peer, &Peer::peerDied,      this, &IvyQt::deletePeer);
    connect(peer, &Peer::message,       this, [=](QString id, QStringList params) {
        int regexId = id.toInt();
        for(auto binding = bindings.begin(); binding != bindings.end(); ++binding) {
            if(binding->regexId == regexId) {
                if(!bindings_removing.contains(binding.key())) {
                    if(logLevel >= IVY_LOG_HIGH) {
                        qCInfo(lcIvy()) << "Call binding" << binding.key();
                    }
                    binding->cb(peer, params);
                }
            }
        }
    });
    connect(peer, &Peer::directMessage, this, [=](int identifier, QString params) {
        emit directMessage(peer, identifier, params);
    });
    connect(peer, &Peer::quitRequest, this, [=]() {
        emit quitRequest(peer);
    });

    peer->setFlushTimeout(flush_timeout);

    // send a synchro message and initial subscriptions.
    peer->sendId(server->serverPort(), name);
    for(auto reg = regexes.begin(); reg != regexes.end(); reg++) {
        peer->bind(reg.key(), reg.value());
    }
    peer->end_initial_bindings();
    peer->setInfoSent();

    peers.append(peer);
}

void IvyQt::newPeerReady(Peer* peer) {
    if(msgReady != "") {
        peer->sendMessage(msgReady);
    }

    emit peerReady(peer);
}

void IvyQt::deletePeer(Peer* peer) {
    auto peerName = peer->name();
    peers.removeAll(peer);
    peer->deleteLater();

    emit peerDied(peerName);

    // stopping properly so start can be called again.
    if(stopRequested && peers.length() == 0) {
        completeStop();
    }
}

void IvyQt::completeStop() {
    running = false;
    udp_socket->deleteLater();
    server->deleteLater();
    udp_socket = nullptr;
    server = nullptr;
    emit stopped();
    if(logLevel >= IVY_LOG_LOW) {
        qCInfo(lcIvy()) << "Stop completed.";
    }
}
