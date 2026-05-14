#include <pprzlinkQt/IvyQtLink.h>
#include <IvyQt/ivyqt.h>
#include <unistd.h>

namespace pprzlink {
    IvyQtLink::IvyQtLink(MessageDictionary const & dict , QString appName, QObject *parent) :
        QObject(parent),
        dictionary (dict), appName(appName), requestNb(0)
    {
        bus = new IvyQt(appName, appName + " ready", this);

        connect(bus, &IvyQt::peerReady, this, [=](Peer* peer){
            if(peer->name() == "Paparazzi server") {
                emit serverConnected();
            }
        });

    }

    void IvyQtLink::start(QString domain) {
        this->domain = domain;
        auto ip_port = domain.split(":");
        assert(ip_port.size() == 2);
        bus->start(ip_port[0], ip_port[1].toUInt());
    }

    void IvyQtLink::stop() {
        bus->stop();
    }

    IvyQtLink::~IvyQtLink()
    {
      bus->stop();
    }

    long IvyQtLink::BindMessage(const MessageDefinition &def, QObject* context, messageCallback_t cb)
    {
      auto regexp = regexpForMessageDefinition(def);
      //std::cout << "Binding to " << regexp << std::endl;

      auto mcb = [=](Peer* peer, QStringList params) {
        (void)peer;
        messageCallback(params, cb);
      };

      auto id = bus->bindMessage(regexp, context, mcb);
      messagesCallbackMap[id] = mcb;
      return id;
    }

    void IvyQtLink::UnbindMessage(long bindId)
      {
        if (messagesCallbackMap.find(bindId) != messagesCallbackMap.end()) {
          bus->unBindMessage(bindId);
          messagesCallbackMap.erase(bindId);
          return;
        }

//        if (aircraftCallbackMap.find(bindId) != aircraftCallbackMap.end()) {
//          bus->UnbindMsg(bindId);
//          delete (aircraftCallbackMap[bindId]);
//          aircraftCallbackMap.erase(bindId);
//          return;
//        }

//        if (requestCallbackMap.find(bindId) != requestCallbackMap.end()) {
//          bus->UnbindMsg(bindId);
//          delete (requestCallbackMap[bindId]);
//          requestCallbackMap.erase(bindId);
//          return;
//        }
      }

    QString IvyQtLink::messageRegexp(const MessageDefinition &def)
      {
        static const std::map<BaseType, QString> typeRegex = {
          {BaseType::CHAR,   "."},
          {BaseType::INT8,   "-?\\d+"},
          {BaseType::INT16,  "-?\\d+"},
          {BaseType::INT32,  "-?\\d+"},
          {BaseType::UINT8,  "\\d+"},
          {BaseType::UINT16, "\\d+"},
          {BaseType::UINT32, "\\d+"},
          {BaseType::FLOAT,  "-?\\d+(?:\\.)?(?:\\d)*"},
          {BaseType::DOUBLE,  "-?\\d+(?:\\.)?(?:\\d)*"},
          {BaseType::STRING, "(?:\"[^\"]+\"|[^ ]+)"}
        };
        // ac_id MSG_NAME msgField*
        QString sstr = "(" + def.getName() + ")";

        for (size_t i = 0; i < def.getNbFields(); ++i)
        {
          //std::cout << def.getField(i).getName() << " : " << def.getField(i).getType().toString() << std::endl;
          auto iter = typeRegex.find(def.getField(i).getType().getBaseType());
          if (iter == typeRegex.end())
          {
            throw wrong_message_format(
              "IvyregexpForMessageDefinition found NOT_A_TYPE in message " + def.getField(i).getName());
          }
          QString baseRegex = iter->second;

          if (def.getField(i).getType().isArray())
          {
            if (def.getField(i).getType().getBaseType()==BaseType::CHAR)
            {
              sstr += " (\"[^\"]*\")";
            }
            else
            {
              if (def.getField(i).getType().getArraySize() == 0)
              {
                // Dynamic array
                // s => s,s,s,s => (s(?:,s)*)
                sstr += " (" + baseRegex + "(?:," + baseRegex + ")*)";
              }
              else
              {
                // Static array
                // s => s,s,s => (s(?:,s){SIZE-1})
                sstr += " (" + baseRegex + "(?:," + baseRegex + "){" + QString::number(def.getField(i).getType().getArraySize() - 1)
                     + "})";
              }
            }
          }
          else
          {
              sstr += " (" + baseRegex + ")";
          }
        }
        sstr += "$";

        return sstr;
      }

    QString IvyQtLink::regexpForMessageDefinition(MessageDefinition const & def) {
        QString sstr = "^([^ ]*) " + messageRegexp(def);
        return sstr;
      }

    void IvyQtLink::getMessageData(const Message& msg, QString &ac_id, QString &name, QString &fields) {
        QString fieldsStream;
        if (msg.getSenderId().index()==0) // The variant holds a string
        {
          ac_id = std::get<QString>(msg.getSenderId());
        }
        else
        {

          uint8_t ac_id_int = std::get<uint8_t>(msg.getSenderId());
          ac_id = QString::number(ac_id_int);
        }
        const auto &def=msg.getDefinition();

        for (size_t i=0;i<def.getNbFields();++i)
        {
          if (i!=0) {
            fieldsStream += " ";
          }
          auto val= msg.getRawValue(i);
          val.setOutputInt8AsInt(true);
          std::stringstream ss;
          ss << val;
	  auto qs = QString::fromStdString(ss.str());
          if(qs.contains(' ')) {
            qs.prepend('"');
            qs.append('"');
          }
          fieldsStream += qs;
        }

        name = def.getName();
        fields = fieldsStream;

      }

    void IvyQtLink::sendMessage(const Message& msg)
    {
      const auto &def=msg.getDefinition();
      if(def.isRequest()) {
        throw message_is_request("Message " + def.getName() + " is a request message. Use sendRequest instead!");
      }

      QString ac_id;
      QString name;
      QString fields;

      getMessageData(msg, ac_id, name, fields);

      auto message = ac_id + " " + def.getName() + " " + fields;

      bus->send(message);

    }


    long IvyQtLink::sendRequest(const Message& msg, messageCallback_t cb) {
        const auto &def=msg.getDefinition();

        if(!def.isRequest()) {
          throw message_is_not_request("Message " + def.getName() + " is not a request message. Use sendMessage instead!");
        }

        // remove last 4 characters (_REQ) from request name to get the answer message name
        auto ansName = def.getName().mid(0, def.getName().size() - 4);
        auto ansDef = dictionary.getDefinition(ansName);

        QString ac_id;
        QString name;
        QString fields;

        getMessageData(msg, ac_id, name, fields);

        QString requestId = QString::number(getpid()) + "_" + QString::number(requestNb++);


        auto mcb = [=](Peer* peer, QStringList params) {
          (void)peer;
          messageCallback(params, [=](QString ac_id,Message msg) {
              cb(std::move(ac_id), std::move(msg));

              // find bind id, then unbind answer message
              auto iter = requestBindId.find(requestId);
              if (iter != requestBindId.end())
              {
                long id = iter.value();
                UnbindMessage(id);
                requestBindId.remove(requestId);
              }
            });
        };


        QString regexpStream = "^" + requestId + " ([^ ]*) " + messageRegexp(ansDef);
        auto id = bus->bindMessage(regexpStream, mcb);
        messagesCallbackMap[id] = mcb;
        requestBindId[requestId] = id;
        QString message = ac_id + " " + requestId + " " + def.getName() + " " + fields;
        bus->send(message);

        return id;
      }

    void IvyQtLink::messageCallback(QStringList params, const messageCallback_t &cb)
      {
        MessageDefinition def = dictionary.getDefinition(params[1]);
        Message msg(def);
        if (def.getNbFields() != (size_t)(params.size() - 2) )
        {
          QString sstr = params[1] + " message with wrong number of fields (expected " + QString::number(def.getNbFields()) + " / got " + QString::number(params.size()-2)
               + ")";
          throw wrong_message_format(sstr);
        }
        for (int i = 2; i < params.size(); ++i)
        {
          const auto& field = def.getField(i - 2);
          // Need deserializing string to build FieldValue

          // For char arrays and strings remove possible quotes
          if ((field.getType().getBaseType()==BaseType::STRING || (field.getType().getBaseType()==BaseType::CHAR && field.getType().isArray())) && params[i][0]=='"')
          {
            QString str = params[i];
            //std::cout << str.substr(1,str.size()-2) << std::endl;
            QString val = str.mid(1,str.size()-2);
            msg.addField(field.getName(), val);
          }
          else
          {
            QString sstr(params[i]);
            if (field.getType().isArray())
            {
              switch (field.getType().getBaseType())
              {
                case BaseType::NOT_A_TYPE:
                  throw std::logic_error("NOT_A_TYPE for field " + field.getName().toStdString() + " in message " + params[1].toStdString());
                  break;
                case BaseType::CHAR:
                  throw wrong_message_format("Wrong field format for a char[] " + params[i]);
                  break;
                case BaseType::INT8:
                case BaseType::INT16:
                case BaseType::INT32:
                case BaseType::UINT8:
                case BaseType::UINT16:
                case BaseType::UINT32:
                case BaseType::FLOAT:
                case BaseType::DOUBLE:
                {
                  // Parse all numbers as a double
                  std::vector<double> values;
                  auto splitted = sstr.split(",");
                  for(auto ele: splitted) {
                      bool ok = false;
                      double val = ele.toDouble(&ok);
                      if(!ok) {
                          throw wrong_message_format("Wrong format for array "+params[i]);
                      }
                      values.push_back(val);
                  }
                  msg.addField(field.getName(), values); // The value will be statically cast to the right type
                }
                  break;
                case BaseType::STRING:
                  msg.addField(field.getName(), params[i]);
                  break;
              }
            }
            else
            {
              switch (field.getType().getBaseType())
              {
                case BaseType::NOT_A_TYPE:
                 throw std::logic_error("NOT_A_TYPE for field " + field.getName().toStdString() + " in message " + params[1].toStdString());
                  break;
                case BaseType::CHAR:
                {
                  char val;
                  val = sstr[0].toLatin1();         // TODO vérifier
                  msg.addField(field.getName(), val);
                }
                  break;
                case BaseType::INT8:
                case BaseType::INT16:
                case BaseType::INT32:
                case BaseType::UINT8:
                case BaseType::UINT16:
                case BaseType::UINT32:
                case BaseType::FLOAT:
                case BaseType::DOUBLE:
                {
                  // Parse all numbers as a double
                  double val;
                  val = sstr.toDouble();
                  msg.addField(field.getName(), val); // The value will be statically cast to the right type
                }
                  break;
                case BaseType::STRING:
                  msg.addField(field.getName(), params[i]);
                  break;
              }
            }
          }
        }
        QString sender(params[0]);
        if (params[0][0]=='"')
        {
          // Remove quotes from string if needed
          sender=sender.mid(1,sender.size()-2);
        }

        cb(sender, msg);
      }
}


