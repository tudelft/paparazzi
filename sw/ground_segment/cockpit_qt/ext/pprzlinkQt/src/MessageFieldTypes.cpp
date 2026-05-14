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

/** \file MessageFieldTypes.cpp
 *
 *
 */

#include <map>
#include <sstream>
#include <iostream>
#include <pprzlinkQt/pprzlink_exception.h>
#include <pprzlinkQt/MessageFieldTypes.h>

namespace pprzlink {

  const std::map<BaseType,size_t> sizeofTypeMap = {
    {BaseType::CHAR, 1},
    {BaseType::INT8, 1},
    {BaseType::INT16, 2},
    {BaseType::INT32, 4},
    {BaseType::UINT8, 1},
    {BaseType::UINT16, 2},
    {BaseType::UINT32, 4},
    {BaseType::FLOAT, 4},
    {BaseType::DOUBLE, 8},
    {BaseType::STRING, 0}
  };

  static const std::map<BaseType,QString> typeMap{
    {BaseType::CHAR, "char"},
    {BaseType::INT8, "int8"},
    {BaseType::INT16, "int16"},
    {BaseType::INT32, "int32"},
    {BaseType::UINT8, "uint8"},
    {BaseType::UINT16, "uint16"},
    {BaseType::UINT32, "uint32"},
    {BaseType::FLOAT, "float"},
    {BaseType::DOUBLE, "double"},
    {BaseType::STRING, "string"}
  };

  FieldType::FieldType(QString const &typeString)
  : baseType(BaseType::NOT_A_TYPE), arraySize(-1)
  {
    for (const auto& pairs : typeMap)
    {
      if (typeString.startsWith(pairs.second))
      {
        baseType = pairs.first;
        break;
      }
    }
    if (baseType==BaseType::NOT_A_TYPE)
      throw pprzlink::bad_message_file("Field with type string "+ typeString + " resolved to NOT_A_TYPE");
    auto openSquareBracketPos=typeString.indexOf('[');
    if (openSquareBracketPos != -1) // This is an array
    {
      auto closeSquareBracketPos=typeString.indexOf(']');
      if (openSquareBracketPos==closeSquareBracketPos-1)
      {
        // Dynamic array
        arraySize=0;
      }
      else
      {
        QString sstr = typeString.mid(openSquareBracketPos+1, closeSquareBracketPos-openSquareBracketPos-1);
        arraySize = sstr.toInt();
      }
    }
  }

  BaseType FieldType::getBaseType() const
  {
    return baseType;
  }

  bool FieldType::isArray() const
  {
    return arraySize != -1;
  }

  size_t FieldType::getArraySize() const
  {
    return arraySize;
  }

  QString FieldType::toString() const
  {
    QString sstr;
    auto baseTypeStr = typeMap.find(baseType)->second;
    sstr += baseTypeStr;
    if (arraySize>0)
    {
      sstr += '[' + QString::number(arraySize) + ']';
    }
    else if (arraySize==0)
    {
      sstr += "[]";
    }
    return sstr;
  }

  size_t sizeofBaseType(BaseType type)
  {
    if (type==BaseType::NOT_A_TYPE)
      throw std::logic_error("Type NOT_A_TYPE in sizeofBaseType");

    return sizeofTypeMap.find(type)->second;
  }
}
