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

/** \file Message.cpp
 *
 *
 */

#include <iostream>
#include <pprzlinkQt/Message.h>

namespace pprzlink {

  Message::Message() : sender_id(static_cast<uint8_t>(0)),receiver_id(static_cast<uint8_t>(0)),component_id(static_cast<uint8_t>(0))
  {
  }

  Message::Message(const MessageDefinition &def) : def(def),sender_id(static_cast<uint8_t>(0)),receiver_id(static_cast<uint8_t>(0)),component_id(static_cast<uint8_t>(0))
  {
  }

  size_t Message::getNbValues() const
  {
    return fieldValues.size();
  }

  const MessageDefinition &Message::getDefinition() const
  {
    return def;
  }

  template<typename StreamType>
  void addFieldToStream(StreamType &stream,const MessageField &field,std::any value, bool uint8_as_int=false)
  {
    switch (field.getType().getBaseType())
    {
      case BaseType::CHAR :
        stream << std::any_cast<char>(value);
        break;
      case BaseType::INT8:
        if (uint8_as_int)
        {
          stream << (int)std::any_cast<int8_t>(value);
        }
        else
        {
          stream << std::any_cast<int8_t>(value);
        }
        break;
      case BaseType::INT16:
        stream << std::any_cast<int16_t>(value);
        break;
      case BaseType::INT32:
        stream << std::any_cast<int32_t>(value);
        break;
      case BaseType::UINT8:
        if (uint8_as_int)
        {
          stream << (int)std::any_cast<uint8_t>(value);
        }
        else
        {
          stream << std::any_cast<uint8_t>(value);
        }
        break;
      case BaseType::UINT16:
        stream << std::any_cast<uint16_t>(value);
        break;
      case BaseType::UINT32:
        stream << std::any_cast<uint32_t>(value);
        break;
      case BaseType::FLOAT:
        stream << std::any_cast<float>(value);
        break;
      case BaseType::DOUBLE:
        stream << std::any_cast<double>(value);
        break;
      case BaseType::STRING:
          stream << std::any_cast<QString>(value);
          break;
      case BaseType::NOT_A_TYPE:
        // THIS SHOULD NOT HAPPEN !!!!
        throw std::logic_error(QString(" field "+ field.getName() +"as type NOT_A_TYPE !").toStdString());
        break;
    }
  }

  template<typename StreamType>
  void addVectorToStream([[maybe_unused]] StreamType &stream,const Message &msg,int index, [[maybe_unused]] bool uint8_as_int=false)
  {
    const auto& value = msg.getRawValue(index);
    switch (value.getType().getBaseType())
    {
      case BaseType::CHAR :
      {
        std::vector<char> vec;

      }
        break;
      case BaseType::INT8:
        break;
      case BaseType::INT16:
        break;
      case BaseType::INT32:
        break;
      case BaseType::UINT8:
        break;
      case BaseType::UINT16:
        break;
      case BaseType::UINT32:
        break;
      case BaseType::FLOAT:
        break;
      case BaseType::DOUBLE:
        break;
      case BaseType::STRING:
        break;
      case BaseType::NOT_A_TYPE:
        // THIS SHOULD NOT HAPPEN !!!!
        throw std::logic_error(" field " + value.getName().toStdString() + "as type NOT_A_TYPE !");
        break;
    }
  }

  QString Message::toString() const
  {
    QString sstr = def.getName() + " [";
    if (def.getNbFields())
    for (size_t i=0;i<def.getNbFields();++i)
    {
      if (i != 0)
      {
        sstr += "; ";
      }
      auto name = def.getField(i).getName();

      sstr += name + "=";
      auto found = fieldValues.find(name);
      if (found == fieldValues.end())
      {
        sstr += "NOTSET";
      }
      else
      {
        if (found->second.getType().isArray() && found->second.getType().getBaseType()!=BaseType::CHAR) {
          sstr += "{";
        }
        auto val =found->second;
        val.setOutputInt8AsInt(true);
        std::stringstream ss;
        ss << val;
        sstr += QString::fromStdString(ss.str());
        if (found->second.getType().isArray() && found->second.getType().getBaseType()!=BaseType::CHAR) {
          sstr += "}";
        }
      }
    }
    sstr += "]";

    return sstr;
  }

  const FieldValue &Message::getRawValue(const QString &name) const
  {
    auto found = fieldValues.find(name);
    if (found==fieldValues.end())
    {
      throw pprzlink::no_such_field("No value for field " + name);
    }
    return found->second;
  }

  const FieldValue &Message::getRawValue(int index) const
  {
    auto name = def.getField(index).getName();
    auto found = fieldValues.find(name);
    if (found==fieldValues.end())
    {
      throw pprzlink::no_such_field("No value for field " + name);
    }
    return found->second;
  }

  const std::variant<QString, uint8_t> &Message::getSenderId() const
  {
    return sender_id;
  }

  uint8_t Message::getReceiverId() const
  {
    return receiver_id;
  }

  uint8_t Message::getComponentId() const
  {
    return component_id;
  }

  uint8_t Message::getClassId() const
  {
    return def.getClassId();
  }

  void Message::setSenderId(const std::variant<QString, uint8_t> &senderId)
  {
    sender_id = senderId;
  }

  void Message::setReceiverId(uint8_t receiverId)
  {
    receiver_id = receiverId;
  }

  void Message::setComponentId(uint8_t componentId)
  {
    component_id = componentId;
  }

  unsigned long long makeValue(BytesBuffer const &buffer, size_t offset, size_t elemSize)
  {
    unsigned long long value=0;

    for (size_t byteIndex=0; byteIndex < elemSize; ++byteIndex)
    {
      value |= buffer[offset + byteIndex] << (byteIndex * 8u);
    }
    return value;
  }

  template<typename BASE_TYPE>
  std::vector<BASE_TYPE> makeVector(BytesBuffer const &buffer, size_t offset, size_t nbElem, size_t elemSize)
  {
    std::vector<BASE_TYPE> vec;
    for (size_t elemIndex=0; elemIndex < nbElem; ++elemIndex)
    {
      vec.push_back((BASE_TYPE)makeValue(buffer,offset+elemSize*elemIndex,elemSize));
    }
    return vec;
  }
  size_t Message::addFieldToBuffer(size_t index, BytesBuffer &buffer) const
  {
    {
      auto name = def.getField(index).getName();
      auto const &it = fieldValues.find(name);
      if (it == fieldValues.end())
      {
        throw field_has_no_value("In message " + def.getName() + " field " + name + " has not value !");
      }
      return it->second.addToBuffer(buffer);
    }
  }

  void Message::addFieldFromBuffer(size_t index, BytesBuffer const &buffer, size_t& offset)
  {
    auto const & field =  getDefinition().getField(index);
    auto const & fieldType = field.getType();
    auto size = field.getSize();
    unsigned long long value=0;

    // For arrays
    if (fieldType.isArray())
    {
      auto elemSize = sizeofBaseType(field.getType().getBaseType());
      if (size==0) // Variable length array
      {
        size = elemSize * makeValue(buffer,offset,1); // Read the length of the array
        offset++;
      }
      switch (fieldType.getBaseType())
      {
        case BaseType::CHAR:
        {
          auto vec = makeVector<char>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::INT8:
        {
          auto vec = makeVector<int8_t>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::INT16:
        {
          auto vec = makeVector<int16_t>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::INT32:
        {
          auto vec = makeVector<int32_t>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::UINT8:
        {
          auto vec = makeVector<uint8_t>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::UINT16:
        {
          auto vec = makeVector<uint16_t>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::UINT32:
        {
          auto vec = makeVector<uint32_t>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::FLOAT:
        {
          auto vec = makeVector<float>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        case BaseType::DOUBLE:
        {
          auto vec = makeVector<double>(buffer,offset,size/elemSize,elemSize);
          addField(field.getName(),vec);
          break;
        }
        default:
          throw std::logic_error("Type "+ fieldType.toString().toStdString() + " is not correct for PPRZ Transport.");
          break;
      }
      offset+=size;
    }
      // If not an array and not a string (last case should not occur as strings are for Ivy only...)
    else if (!fieldType.isArray() && size)
    {
      value = makeValue(buffer,offset,size);
      offset+=size;

      switch (fieldType.getBaseType())
      {
        case BaseType::CHAR:
          addField(field.getName(),(char)value);
          break;
        case BaseType::INT8:
          addField(field.getName(),(int8_t)value);
          break;
        case BaseType::INT16:
          addField(field.getName(),(int16_t)value);
          break;
        case BaseType::INT32:
          addField(field.getName(),(int32_t)value);
          break;
        case BaseType::UINT8:
          addField(field.getName(),(uint8_t)value);
          break;
        case BaseType::UINT16:
          addField(field.getName(),(uint16_t)value);
          break;
        case BaseType::UINT32:
          addField(field.getName(),(uint32_t)value);
          break;
        case BaseType::FLOAT:
        {
          float *vf = (float*) &value;
          addField(field.getName(), (float) *vf);
          break;
        }
        case BaseType::DOUBLE:
        {
          double *vf = (double*) &value;
          addField(field.getName(), (double) *vf);
          break;
        }
        case BaseType::STRING:
        {
          // A string is like a variable size array of char (char[])
          size = makeValue(buffer,offset,1); // Read the length of the string
          offset++;
          auto vec = makeVector<char>(buffer,offset,size,1);
          addField(field.getName(),vec);
          break;
        }
        default:
          throw std::logic_error("Type "+ fieldType.toString().toStdString()+ " is not correct for PPRZ Transport.");
          break;
      }
    }
  }

  size_t Message::getByteSize() const
  {
    size_t size= 0;
    if (fieldValues.size() != getDefinition().getNbFields())
    {
      throw pprzlink::field_has_no_value("Cannot get size of an incomplete message.");
    }
    for (auto value : fieldValues)
    {
      size+=value.second.getByteSize();
    }

    return size;
  }
}
