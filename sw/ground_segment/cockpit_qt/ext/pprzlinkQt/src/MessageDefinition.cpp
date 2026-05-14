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

/** \file MessageDefinition.cpp
 *
 *
 */

#include <iostream>
#include <sstream>
#include <pprzlinkQt/MessageDefinition.h>
#include <pprzlinkQt/pprzlink_exception.h>

namespace pprzlink {
  MessageDefinition::MessageDefinition()
    : classId(0), id(0)
  {
  }

  MessageDefinition::MessageDefinition(QDomElement xml, int classId)
    : classId(classId), id(-1)
  {
    name = xml.attribute("name");
    id = xml.attribute("id").toUInt();


    for(auto field = xml.firstChildElement("field");
            !field.isNull();
            field = field.nextSiblingElement("field")) {
        auto fieldName = field.attribute("name", "");
        auto fieldTypeStr = field.attribute("type", "");
        if (fieldName == "" || fieldTypeStr == "")
        {
          throw bad_message_file("Bad field");
        }
        MessageField msgField(fieldName, fieldTypeStr);

        fieldNameToIndex[fieldName] = fields.size();
        fields.push_back(msgField);
    }

  }

  uint8_t MessageDefinition::getClassId() const
  {
    return classId;
  }

  uint8_t MessageDefinition::getId() const
  {
    return id;
  }

  const QString &MessageDefinition::getName() const
  {
    return name;
  }

  const MessageField &MessageDefinition::getField(int index) const
  {
    return fields[index];
  }

  const MessageField &MessageDefinition::getField(const QString &name) const
  {
    auto found =fieldNameToIndex.find(name);
    if (found==fieldNameToIndex.end()) {
      throw no_such_field("No field " + name + " in message " + getName());
    }
    return fields[found->second];
  }

  size_t MessageDefinition::getNbFields() const
  {
    return fields.size();
  }

  bool MessageDefinition::hasFieldName(const QString &name) const
  {
    return fieldNameToIndex.find(name)!=fieldNameToIndex.end();
  }

  QString MessageDefinition::toString() const
  {
    QString sstr;
    sstr += getName() + "(" + QString::number(getId()) + ") in class " + QString::number(getClassId()) + "\n";
    for (size_t i=0;i<getNbFields();++i)
    {
      sstr += "\t" + getField(i).getName() + " : " + getField(i).getType().toString() + "\n";
    }

    return sstr;
  }

  size_t MessageDefinition::getMinimumSize() const
  {
    int size=0;
    for (auto field: fields)
    {
        size+=field.getSize();
    }
    return size;
  }

  bool MessageDefinition::isRequest() const
  {
      QString req = "_REQ";
      if (name.length() >= req.length()) {
          return (name.right(req.length()) == req);
      } else {
          return false;
      }
  }
}
