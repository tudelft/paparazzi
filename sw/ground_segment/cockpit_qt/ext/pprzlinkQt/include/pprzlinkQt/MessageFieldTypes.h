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

/** \file MessageFieldTypes.h
 *
 *
 */

#ifndef PPRZLINKCPP_MESSAGEFIELDTYPES_H
#define PPRZLINKCPP_MESSAGEFIELDTYPES_H

#include <QString>

namespace pprzlink {

  enum class BaseType {
    NOT_A_TYPE,
    CHAR,
    INT8,
    INT16,
    INT32,
    UINT8,
    UINT16,
    UINT32,
    FLOAT,
    DOUBLE,
    STRING
  };

  size_t sizeofBaseType(BaseType type);

  class FieldType {
  public:
    explicit FieldType(QString const &typeString);

    [[nodiscard]] BaseType getBaseType() const;

    [[nodiscard]] bool isArray() const;

    [[nodiscard]] size_t getArraySize() const;

    [[nodiscard]] QString toString() const;

  private:
    BaseType baseType;
    int arraySize; // 0 for dynamic, -1 for not an array
  };
}
#endif //PPRZLINKCPP_MESSAGEFIELDTYPES_H
