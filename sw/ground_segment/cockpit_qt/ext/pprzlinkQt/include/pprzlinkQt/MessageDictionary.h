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

/** \file MessageDictionnary.h
 *
 *
 */

#ifndef PPRZLINKCPP_MESSAGEDICTIONARY_H
#define PPRZLINKCPP_MESSAGEDICTIONARY_H

#include <QMap>
#include <map>
#include <pprzlinkQt/MessageDefinition.h>

namespace pprzlink {
  class MessageDictionary {
  public:
    explicit MessageDictionary(QString const &fileName);

    [[nodiscard]] const MessageDefinition &getDefinition(QString const &name) const;

    [[nodiscard]] const MessageDefinition &getDefinition(int classId, int msgId) const;

    [[nodiscard]] std::pair<int,int> getMessageId(QString name) const;
    [[nodiscard]] QString getMessageName(int classId, int msgId) const;

    [[nodiscard]] int getClassId(QString name) const;
    [[nodiscard]] QString getClassName(int id) const;

    [[nodiscard]] std::vector<MessageDefinition> getMsgsForClass(QString className) const;
    [[nodiscard]] std::vector<MessageDefinition> getMsgsForClass(int classId) const;

  private:
    std::map<QString, MessageDefinition> messagesDict;
    QMap<QString, std::pair<int, int>> msgNameToId;
    QMap<int,QString> classMap;
  };
}
#endif //PPRZLINKCPP_MESSAGEDICTIONARY_H
