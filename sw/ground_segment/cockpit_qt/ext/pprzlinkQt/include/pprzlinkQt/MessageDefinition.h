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

/** \file MessageDefinition.h
 *
 *
 */

#ifndef PPRZLINKCPP_MESSAGEDEFINITION_H
#define PPRZLINKCPP_MESSAGEDEFINITION_H

#include <vector>
#include <pprzlinkQt/MessageField.h>
#include <QtXml>
#include <map>

namespace pprzlink {

  class MessageDefinition {
  public:
    MessageDefinition ();

    explicit MessageDefinition (QDomElement xml,int classId);

    [[nodiscard]] uint8_t getClassId() const;

    [[nodiscard]] uint8_t getId() const;

    [[nodiscard]] const QString &getName() const;

    [[nodiscard]] size_t getNbFields() const;

    [[nodiscard]] const MessageField& getField(int index) const;

    [[nodiscard]] const MessageField& getField(const QString &name) const;

    [[nodiscard]] bool hasFieldName(const QString &name) const;

    [[nodiscard]] QString toString() const;

    [[nodiscard]] size_t getMinimumSize() const;

    [[nodiscard]] bool isRequest() const;

  private:
    uint8_t classId;
    uint8_t id;
    QString name;
    std::vector<MessageField> fields;
    std::map<QString, size_t> fieldNameToIndex;
  };
}
#endif //PPRZLINKCPP_MESSAGEDEFINITION_H
