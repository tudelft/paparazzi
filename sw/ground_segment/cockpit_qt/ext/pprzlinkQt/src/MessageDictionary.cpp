/*
 * Copyright 2019 garciafa
 * This file is part of PprzLinkCPP
 *
 * PprzLinkCPP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PprzLinkCPP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ModemTester.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/** \file MessageDictionnary.cpp
 *
 *
 */

#include <pprzlinkQt/MessageDictionary.h>
#include <iostream>
#include <pprzlinkQt/pprzlink_exception.h>

// TODO Implement the dictionnary as a singleton !

namespace pprzlink {

  MessageDictionary::MessageDictionary(QString const &fileName)
  {

      QDomDocument xml;
      QFile f(fileName);
      if(!f.open(QIODevice::ReadOnly)) {
          throw std::runtime_error("Error loading dictionnary file: " + fileName.toStdString());
      }
      xml.setContent(&f);
      f.close();

      QDomElement rootElem = xml.documentElement();

      if(rootElem.tagName() !="protocol")
      {
        throw bad_message_file("Root element is not protocol in xml messages file (found " + rootElem.tagName() + ").");
      }


      for(auto msg_class = rootElem.firstChildElement("msg_class");
              !msg_class.isNull();
              msg_class = msg_class.nextSiblingElement("msg_class")) {

          auto className = msg_class.attribute("name", "");
          int classId = msg_class.attribute("id", "-1").toInt();
          if (className == "" || classId == -1)
          {
            throw bad_message_file(fileName + " msg_class as no name or id.");
          }
          classMap[classId] = className;

          for(auto message = msg_class.firstChildElement("message");
                      !message.isNull();
                      message = message.nextSiblingElement("message")) {
              auto messageName = message.attribute("name", "");
              int messageId = message.attribute("id", "-1").toInt();
              if (messageName == "" || messageId == -1)
              {
                throw bad_message_file(fileName + " in class : " + className + " message as no name or id.");
              }
              try
              {
                MessageDefinition def(message, classId);
                messagesDict[messageName]=def;
                msgNameToId[messageName] = std::make_pair(classId,messageId);
              } catch (bad_message_file &e)
              {
                throw bad_message_file(fileName + " in class : " + className + " message " + messageName + " has a bad field.");
              }
          }
      }
  }

  const MessageDefinition &MessageDictionary::getDefinition(QString const & name) const
  {
    auto iter = messagesDict.find(name);
    if (iter == messagesDict.end())
    {
      QString sstr = "could not find message with name " + name;
      throw no_such_message(sstr);
    }
    else
    {
      return iter->second;
    }
  }

  const MessageDefinition &MessageDictionary::getDefinition(int classId, int msgId) const
  {
    for(auto iter=msgNameToId.begin(); iter!=msgNameToId.end(); ++iter) {
        if(iter.value() == std::make_pair(classId, msgId)) {
            QString name = iter.key();
            //std::cout << "message with id (" << classId << ":" << msgId << ") = " << name << std::endl;
            //std::cout << messagesDict.find(name)->second.toString() << std::endl;
            return messagesDict.find(name)->second;
        }
    }

    throw no_such_message("could not find message with id (" + QString::number(classId) + ":" + QString::number(msgId) + ")");
  }

  std::pair<int, int> MessageDictionary::getMessageId(QString name) const
  {
    if(msgNameToId.contains(name)) {
        return msgNameToId[name];
    } else {
        throw no_such_message("No message with name " + name);
    }
  }

  QString MessageDictionary::getMessageName(int classId, int msgId) const
  {
      for(auto iter=msgNameToId.begin(); iter!=msgNameToId.end(); ++iter) {
          if(iter.value() == std::make_pair(classId, msgId)) {
              return iter.key();
          }
      }
      throw no_such_message("could not find message with id (" + QString::number(classId) + ":" + QString::number(msgId) + ")");
  }

  int MessageDictionary::getClassId(QString name) const
  {
      for(auto iter=classMap.begin(); iter!=classMap.end(); ++iter) {
          if(iter.value() == name) {
              return iter.key();
          }
      }

      throw no_such_class("could not find class named " + name);
  }

  QString MessageDictionary::getClassName(int id) const
  {
    if(classMap.contains(id)) {
        return classMap[id];
    } else {
        throw no_such_class("could not find class with id " + QString::number(id));
    }
  }

  std::vector<MessageDefinition> MessageDictionary::getMsgsForClass(QString className) const
  {
    return getMsgsForClass(getClassId(className));
  }

  std::vector<MessageDefinition> MessageDictionary::getMsgsForClass(int classId) const
  {
    std::vector<MessageDefinition> result;
    for (auto msgPair : messagesDict)
    {
      auto &def = msgPair.second;
      if (def.getClassId()==classId)
        result.push_back(def);
    }
    return result;
  }
}

