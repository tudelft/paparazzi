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

/** \file MessageField.cpp
 *
 *
 */

#include <pprzlinkQt/MessageField.h>
#include <iostream>

namespace pprzlink {

  MessageField::MessageField(const QString &name, const FieldType &type)
    : name(name), type(type), size(0)
  {
    if (getType().getBaseType()==BaseType::STRING)
      size = 0;
    else if (getType().isArray())
      size=sizeofBaseType(getType().getBaseType())*getType().getArraySize();
    else
      size=sizeofBaseType(getType().getBaseType());
  }

  MessageField::MessageField(const QString &name, const QString &typeString)
    : name(name), type(typeString), size(0)
  {
    if (getType().getBaseType()==BaseType::STRING)
      size = 0;
    else if (getType().isArray())
      size+=sizeofBaseType(getType().getBaseType())*getType().getArraySize();
    else
      size+=sizeofBaseType(getType().getBaseType());
  }

  const QString &MessageField::getName() const
  {
    return name;
  }

  const FieldType &MessageField::getType() const
  {
    return type;
  }

  size_t MessageField::getSize() const
  {
    return size;
  }
}
