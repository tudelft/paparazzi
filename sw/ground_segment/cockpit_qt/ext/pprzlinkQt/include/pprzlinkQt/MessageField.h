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

/** \file MessageField.h
 *
 *
 */

#ifndef PPRZLINKCPP_MESSAGEFIELD_H
#define PPRZLINKCPP_MESSAGEFIELD_H

#include <string>
#include <pprzlinkQt/MessageFieldTypes.h>
#include <QString>

namespace pprzlink {

  class MessageField {
  public:
    MessageField(const QString &name, const FieldType &type);

    MessageField(const QString &name, const QString &typeString);

    [[nodiscard]] const QString &getName() const;

    [[nodiscard]] const FieldType &getType() const;

    [[nodiscard]] size_t getSize() const;
  private:
    QString name;
    FieldType type;
    size_t size;
  };
}

#endif //PPRZLINKCPP_MESSAGEFIELD_H
